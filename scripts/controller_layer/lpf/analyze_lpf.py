#!/usr/bin/env python3
"""
analyze_lpf.py  — LPF Command-Filter 效果分析腳本 (3-way comparison)
=====================================================================

資料來源
--------
CSV 由 analy_arm_controller.py (action mode) 錄製，
訂閱 /right_joint_trajectory_controller/controller_state
  _cmd   = reference.positions  = JTC 內插的軌跡參考點
                                  ← 這是 action goal 的插值，尚未經過 hardware LPF
  _state = joint_states.position = 馬達實際回饋（hardware 讀回）
  _err   = reference - actual    = 追蹤誤差

LPF 在哪裡作用？
-----------------
  JTC reference ─── CommandInterface ──→ OpenArm_v10LPF_HW::write()
                                              │ LPF(pos_commands_)
                                              ▼
                                      arm_pos_cmd_buffer_  (filtered)
                                              │ arm_control_loop() @500Hz
                                              ▼
                                         CAN:MIT → Motor
                                              │
                                          _state (hardware feedback)

所以：
  _cmd   = JTC trajectory reference (NOT filtered) → 三組幾乎相同
  _state = 馬達實際位置（受 LPF 平滑後的指令驅動）→ 越低 cutoff 越平滑
  _err   = cmd − state  → LPF 造成相位延遲，誤差峰值會在快速轉折處放大

分析策略
--------
1. 模擬 LPF(cmd)   → 重現「實際送去馬達的指令」曲線
2. state 速度平滑度 → ∂state/∂t RMS；越低 = 運動越平滑
3. state FFT        → 高頻分量應隨 cutoff 降低而減少
4. error 比較       → RMSE / Max|err|；LPF 引入相位延遲會增大誤差
5. Summary bars     → 全關節比較

使用方式:
  python3 analyze_lpf.py
  python3 analyze_lpf.py --joints 1 3 5
  python3 analyze_lpf.py --no-show
"""

import csv
import os
import math
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
BASE    = os.path.dirname(os.path.abspath(__file__))
CSV_DIR = os.path.join(BASE, "csv", "20260422")
IMG_DIR = os.path.join(BASE, "img", "20260422")
os.makedirs(IMG_DIR, exist_ok=True)

FILE_NO  = os.path.join(CSV_DIR, "right_arm_combined_120542_no_lpf.csv")
FILE_20  = os.path.join(CSV_DIR, "right_arm_combined_131706_20lpf.csv")
FILE_10  = os.path.join(CSV_DIR, "right_arm_combined_131855_10lpf.csv")

N_JOINTS = 7

# Plot style per dataset
DATASETS = {
    "No LPF":    {"file": FILE_NO,  "color": "#9E9E9E", "lw": 1.2},
    "20 Hz LPF": {"file": FILE_20,  "color": "#FF5722", "lw": 1.0},
    "10 Hz LPF": {"file": FILE_10,  "color": "#2196F3", "lw": 1.0},
}

# ---------------------------------------------------------------------------
# Load CSV
# ---------------------------------------------------------------------------
def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            vals = {k: float(v) for k, v in row.items()}
            # Drop rows that contain any NaN (e.g. startup period before /joint_states)
            if all(math.isfinite(v) for v in vals.values()):
                rows.append(vals)
    data = {k: np.array([r[k] for r in rows]) for k in rows[0]}
    data["time"] = data["time"] - data["time"][0]
    return data


# ---------------------------------------------------------------------------
# Simulate first-order IIR LPF  (matches OpenArm_v10LPF_HW implementation)
#   alpha = dt / (rc + dt),  rc = 1 / (2π * cutoff_hz)
# ---------------------------------------------------------------------------
def simulate_lpf(signal, time_arr, cutoff_hz):
    """Apply causal first-order IIR LPF (same formula as LowPassFilter::update)."""
    if cutoff_hz <= 0:
        return signal.copy()
    filtered = np.empty_like(signal)
    filtered[0] = signal[0]
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    for i in range(1, len(signal)):
        dt = float(time_arr[i] - time_arr[i - 1])
        if dt < 1e-9:
            dt = 1e-9
        alpha = dt / (rc + dt)
        filtered[i] = filtered[i - 1] + alpha * (signal[i] - filtered[i - 1])
    return filtered


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def diff_signal(arr, time_arr):
    dt = np.diff(time_arr)
    dt = np.where(dt < 1e-9, 1e-9, dt)
    return np.diff(arr) / dt


def fft_mag(arr, dt_avg):
    n   = len(arr)
    mag = np.abs(np.fft.rfft(arr - arr.mean())) * 2.0 / n
    freq = np.fft.rfftfreq(n, d=dt_avg)
    return freq, mag


def metrics(err):
    rmse = math.sqrt(float(np.nanmean(err ** 2)))
    mae  = float(np.nanmean(np.abs(err)))
    maxe = float(np.nanmax(np.abs(err)))
    return rmse, mae, maxe


def vel_rms(sig, time_arr):
    d = diff_signal(sig, time_arr)
    valid = np.isfinite(d)
    if not np.any(valid):
        return float('nan')
    return math.sqrt(float(np.mean(d[valid] ** 2)))


# ---------------------------------------------------------------------------
# Print summary table
# ---------------------------------------------------------------------------
def print_table(datasets, joints):
    keys = list(datasets.keys())
    print(f"\n{'='*90}")
    print("  LPF Analysis — Tracking Error & State Smoothness")
    print(f"  NOTE: _cmd = JTC trajectory reference (unfiltered, same for all 3 runs)")
    print(f"        _state = actual motor position (affected by LPF)")
    print(f"        LPF effect is visible in: ∂state/∂t smoothness & tracking error")
    print(f"{'='*90}")
    hdr = f"  {'Joint':<8} {'Metric':<28}"
    for k in keys:
        hdr += f"  {k:>14}"
    print(hdr)
    print(f"  {'-'*86}")

    for j in joints:
        err_k = f"openarm_right_joint{j}_err"
        st_k  = f"openarm_right_joint{j}_state"
        cmd_k = f"openarm_right_joint{j}_cmd"

        vals = {}
        for k, info in datasets.items():
            d = info["data"]
            r, m, x = metrics(d[err_k])
            sv = vel_rms(d[st_k],  d["time"])
            cv = vel_rms(d[cmd_k], d["time"])
            vals[k] = {"rmse": r, "mae": m, "maxe": x, "sv": sv, "cv": cv}

        def row(label, key):
            line = f"  J{j:<7} {label:<28}"
            for k in keys:
                line += f"  {vals[k][key]:>14.6f}"
            return line

        print(row("RMSE(err) [rad]",       "rmse"))
        print(row("MAE(err)  [rad]",        "mae"))
        print(row("Max|err|  [rad]",        "maxe"))
        print(row("∂state/∂t RMS [rad/s]", "sv"))
        print(row("∂cmd/∂t RMS   [rad/s]", "cv"))
        print(f"  {'-'*86}")


# ---------------------------------------------------------------------------
# Plot 1 — Key insight: simulated motor cmd vs actual state
# Shows the LPF that is invisible to the recording
# ---------------------------------------------------------------------------
def plot_sim_lpf(datasets, joints, show):
    """
    For each joint show:
      col 0: raw_cmd (same for all) + simulated filtered cmds
      col 1: actual _state for all 3 datasets
      col 2: residual (state − sim_filtered_cmd) vs (state − raw_cmd)
    """
    cutoffs = {"No LPF": None, "20 Hz LPF": 20.0, "10 Hz LPF": 10.0}
    nj = len(joints)
    fig, axes = plt.subplots(nj, 3, figsize=(19, 3.8 * nj))
    if nj == 1:
        axes = [axes]

    for row_i, j in enumerate(joints):
        cmd_k = f"openarm_right_joint{j}_cmd"
        st_k  = f"openarm_right_joint{j}_state"

        ax0, ax1, ax2 = axes[row_i]

        # --- col 0: raw cmd + simulated filtered cmds ---
        # Use no_lpf dataset as the "reference trajectory"
        d_ref = datasets["No LPF"]["data"]
        ax0.plot(d_ref["time"], d_ref[cmd_k],
                 color="#9E9E9E", lw=1.5, label="JTC reference (raw cmd)", zorder=5)

        for label, info in datasets.items():
            c = cutoffs[label]
            if c is None:
                continue
            d = info["data"]
            sim = simulate_lpf(d[cmd_k], d["time"], c)
            ax0.plot(d["time"], sim, color=info["color"], lw=1.0,
                     ls="--", label=f"sim LPF({c:.0f}Hz) → motor")

        ax0.set_title(f"J{j}  JTC cmd vs simulated motor cmd\n(dashed = what actually reaches motor)", fontsize=8)
        ax0.set_ylabel("rad")
        ax0.legend(fontsize=6.5)
        ax0.grid(True, alpha=0.3)

        # --- col 1: actual hardware state ---
        for label, info in datasets.items():
            d = info["data"]
            ax1.plot(d["time"], d[st_k],
                     color=info["color"], lw=info["lw"], label=label)
        ax1.set_title(f"J{j}  Actual motor state\n(hardware feedback)", fontsize=8)
        ax1.set_ylabel("rad")
        ax1.legend(fontsize=6.5)
        ax1.grid(True, alpha=0.3)

        # --- col 2: state − simulated_motor_cmd (true lag error) ---
        for label, info in datasets.items():
            d = info["data"]
            c = cutoffs[label]
            sim = simulate_lpf(d[cmd_k], d["time"], c) if c else d[cmd_k]
            residual = d[st_k] - sim
            ax2.plot(d["time"], residual,
                     color=info["color"], lw=0.8, label=label)
        ax2.axhline(0, color="k", lw=0.5, ls="--")
        ax2.set_title(f"J{j}  state − motor_cmd residual\n(pure hardware tracking error, phase lag excluded)", fontsize=8)
        ax2.set_ylabel("rad")
        ax2.legend(fontsize=6.5)
        ax2.grid(True, alpha=0.3)

    for ax_row in axes:
        for ax in ax_row:
            ax.set_xlabel("time (s)")

    fig.suptitle(
        "LPF Analysis — Simulated Motor Command vs Hardware State\n"
        "Col 1: raw JTC cmd + simulated filtered motor cmd  |  "
        "Col 2: actual state  |  Col 3: true tracking residual",
        fontsize=11, y=1.01)
    plt.tight_layout()
    out = os.path.join(IMG_DIR, "lpf_analysis_sim_motor_cmd.png")
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"\n  [Saved] {out}")
    if show: plt.show()
    plt.close(fig)


# ---------------------------------------------------------------------------
# Plot 2 — State velocity smoothness (∂state/∂t)
# This is the most direct proof of LPF effectiveness
# ---------------------------------------------------------------------------
def plot_state_velocity(datasets, joints, show):
    nj = len(joints)
    fig, axes = plt.subplots(nj, 2, figsize=(15, 3.2 * nj))
    if nj == 1:
        axes = [axes]

    for row_i, j in enumerate(joints):
        st_k = f"openarm_right_joint{j}_state"

        ax0, ax1 = axes[row_i]

        vels = {}
        for label, info in datasets.items():
            d    = info["data"]
            v    = diff_signal(d[st_k], d["time"])
            t    = d["time"][1:]
            rms  = math.sqrt(float(np.mean(v ** 2)))
            vels[label] = (t, v, rms)
            ax0.plot(t, v, color=info["color"], lw=0.7,
                     label=f"{label}  RMS={rms:.4f}")

        ax0.set_title(f"J{j}  ∂state/∂t  (motor velocity — hardware side)", fontsize=8)
        ax0.set_ylabel("rad/s")
        ax0.legend(fontsize=6.5)
        ax0.grid(True, alpha=0.3)

        # histogram
        for label, info in datasets.items():
            t, v, rms = vels[label]
            ax1.hist(np.abs(v), bins=80, color=info["color"], alpha=0.55,
                     label=f"{label}  RMS={rms:.4f}")
        ax1.set_title(f"J{j}  |∂state/∂t| histogram\n(less spread = smoother motion)", fontsize=8)
        ax1.set_xlabel("rad/s")
        ax1.legend(fontsize=6.5)
        ax1.grid(True, alpha=0.3)

    for ax_row in axes:
        ax_row[0].set_xlabel("time (s)")

    fig.suptitle(
        "LPF Analysis — State Velocity (∂state/∂t)\n"
        "Key proof: lower LPF cutoff → lower state velocity RMS → smoother motion",
        fontsize=11, y=1.01)
    plt.tight_layout()
    out = os.path.join(IMG_DIR, "lpf_analysis_state_velocity.png")
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"  [Saved] {out}")
    if show: plt.show()
    plt.close(fig)


# ---------------------------------------------------------------------------
# Plot 3 — FFT of state (hardware response in frequency domain)
# ---------------------------------------------------------------------------
def plot_fft_state(datasets, joints, show):
    nj  = len(joints)
    fig, axes = plt.subplots(nj, 1, figsize=(12, 3.2 * nj))
    if nj == 1:
        axes = [axes]

    for row_i, j in enumerate(joints):
        st_k = f"openarm_right_joint{j}_state"
        ax   = axes[row_i]

        for label, info in datasets.items():
            d      = info["data"]
            dt_avg = float(np.mean(np.diff(d["time"])))
            freq, mag = fft_mag(d[st_k], dt_avg)
            ax.semilogy(freq, mag + 1e-9, color=info["color"],
                        lw=1.0, label=label)

        ax.axvline(10, color="#2196F3", ls="--", lw=0.9, alpha=0.8, label="cutoff 10 Hz")
        ax.axvline(20, color="#FF5722", ls="--", lw=0.9, alpha=0.8, label="cutoff 20 Hz")
        ax.set_title(f"J{j}  FFT of state (log scale)", fontsize=8)
        ax.set_xlabel("frequency (Hz)")
        ax.set_ylabel("|X(f)| rad")
        ax.set_xlim(0, 50)
        ax.legend(fontsize=6.5)
        ax.grid(True, alpha=0.3)

    fig.suptitle(
        "LPF Analysis — Frequency Content of Hardware State\n"
        "10 Hz LPF should show least energy above its cutoff frequency",
        fontsize=11, y=1.01)
    plt.tight_layout()
    out = os.path.join(IMG_DIR, "lpf_analysis_fft_state.png")
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"  [Saved] {out}")
    if show: plt.show()
    plt.close(fig)


# ---------------------------------------------------------------------------
# Plot 4 — Tracking error (cmd − state)
# Shows phase-lag penalty from LPF during fast transitions
# ---------------------------------------------------------------------------
def plot_tracking_error(datasets, joints, show):
    nj  = len(joints)
    fig, axes = plt.subplots(nj, 2, figsize=(15, 3.2 * nj))
    if nj == 1:
        axes = [axes]

    for row_i, j in enumerate(joints):
        err_k = f"openarm_right_joint{j}_err"
        ax0, ax1 = axes[row_i]

        for label, info in datasets.items():
            d = info["data"]
            r, _, _ = metrics(d[err_k])
            ax0.plot(d["time"], d[err_k],
                     color=info["color"], lw=0.8, label=f"{label}  RMSE={r:.4f}")

        ax0.axhline(0, color="k", lw=0.5, ls="--")
        ax0.set_title(f"J{j}  Tracking error (JTC reference − hardware state)", fontsize=8)
        ax0.set_ylabel("rad")
        ax0.legend(fontsize=6.5)
        ax0.grid(True, alpha=0.3)

        # error histogram
        for label, info in datasets.items():
            d = info["data"]
            ax1.hist(d[err_k], bins=80, color=info["color"], alpha=0.55,
                     label=label)
        ax1.set_title(f"J{j}  Error distribution\n(wider base = more jerk; higher peak = steady-state OK)", fontsize=8)
        ax1.set_xlabel("rad")
        ax1.legend(fontsize=6.5)
        ax1.grid(True, alpha=0.3)

    for ax_row in axes:
        ax_row[0].set_xlabel("time (s)")

    fig.suptitle(
        "LPF Analysis — Tracking Error (JTC reference − actual state)\n"
        "LPF introduces phase-lag → larger error at fast transitions; but smoother steady-state",
        fontsize=11, y=1.01)
    plt.tight_layout()
    out = os.path.join(IMG_DIR, "lpf_analysis_tracking_error.png")
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"  [Saved] {out}")
    if show: plt.show()
    plt.close(fig)


# ---------------------------------------------------------------------------
# Plot 5 — Summary bars (RMSE error + state velocity RMS)
# ---------------------------------------------------------------------------
def plot_summary_bars(datasets, joints, show):
    keys   = list(datasets.keys())
    colors = [datasets[k]["color"] for k in keys]
    xticks = [f"J{j}" for j in joints]
    x      = np.arange(len(joints))
    w      = 0.25
    offsets = np.linspace(-(len(keys)-1)/2 * w, (len(keys)-1)/2 * w, len(keys))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    for ki, (k, off, col) in enumerate(zip(keys, offsets, colors)):
        d = datasets[k]["data"]
        rmse_vals = [metrics(d[f"openarm_right_joint{j}_err"])[0] for j in joints]
        sv_vals   = [vel_rms(d[f"openarm_right_joint{j}_state"], d["time"]) for j in joints]

        bars1 = ax1.bar(x + off, rmse_vals, w, label=k, color=col, alpha=0.85)
        for bar in bars1:
            ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height(),
                     f"{bar.get_height():.4f}", ha="center", va="bottom", fontsize=6)

        bars2 = ax2.bar(x + off, sv_vals, w, label=k, color=col, alpha=0.85)
        for bar in bars2:
            ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height(),
                     f"{bar.get_height():.4f}", ha="center", va="bottom", fontsize=6)

    ax1.set_xticks(x); ax1.set_xticklabels(xticks)
    ax1.set_ylabel("RMSE (rad)")
    ax1.set_title("Tracking Error RMSE\n(cmd−state; lower = better reference tracking)")
    ax1.legend(fontsize=8); ax1.grid(True, axis="y", alpha=0.3)

    ax2.set_xticks(x); ax2.set_xticklabels(xticks)
    ax2.set_ylabel("∂state/∂t RMS (rad/s)")
    ax2.set_title("State Velocity RMS\n(KEY: lower = smoother motion; proves LPF works)")
    ax2.legend(fontsize=8); ax2.grid(True, axis="y", alpha=0.3)

    fig.suptitle("LPF Analysis Summary — No LPF vs 20 Hz vs 10 Hz", fontsize=13)
    plt.tight_layout()
    out = os.path.join(IMG_DIR, "lpf_analysis_summary_bars.png")
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"  [Saved] {out}")
    if show: plt.show()
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Analyse LPF command filter (3-way: no/10Hz/20Hz)")
    parser.add_argument("--joints", nargs="+", type=int,
                        default=list(range(1, N_JOINTS + 1)))
    parser.add_argument("--no-show", action="store_true")
    args   = parser.parse_args()
    joints = args.joints
    show   = not args.no_show

    print("\nLoading CSV files …")
    for label, info in DATASETS.items():
        info["data"] = load_csv(info["file"])
        d = info["data"]
        print(f"  {label:<12}: {len(d['time'])} samples, duration {d['time'][-1]:.2f} s")

    print(f"\nAnalysing joints: {joints}")
    print("\n[INFO] _cmd = JTC trajectory reference (same for all 3 — NOT filtered)")
    print("[INFO] LPF acts inside hardware write() before CAN command → invisible to JTC")
    print("[INFO] LPF effect is observed in: _state smoothness & ∂state/∂t RMS\n")

    print_table(DATASETS, joints)

    print("\nGenerating plots …")
    plot_sim_lpf(DATASETS, joints, show)
    plot_state_velocity(DATASETS, joints, show)
    plot_fft_state(DATASETS, joints, show)
    plot_tracking_error(DATASETS, joints, show)
    plot_summary_bars(DATASETS, joints, show)

    print(f"\nAll images saved to: {IMG_DIR}\nDone.\n")


if __name__ == "__main__":
    main()
