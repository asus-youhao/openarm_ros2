#!/usr/bin/env python3
"""
analyze_lpf_v2.py  — LPF Command-Filter 改進版分析腳本 (3-way comparison)
==========================================================================

主要修正 vs v1
--------------
1. diff_signal 修正：跳過 dt < 1ms 的重複時間戳（v1 用 1e-9 clamp 導致假速度 ~870k rad/s）
2. 新增 Jerk RMS（∂²state/∂t²）：比速度更能看出抖動程度
3. 新增 HF Energy Ratio：FFT 能量高於截止頻率佔總能量比率
4. 改善百分比欄位：每個指標相對 No LPF 的改善 %
5. 移除無意義的 ∂cmd/∂t（duplicate timestamp 問題）
6. 新增 Plot 6：改善百分比 Report Card 視覺化

LPF 作用路徑
--------------
  JTC reference (_cmd) ─── CommandInterface ──→ OpenArm_v10LPF_HW::write()
                                                      │ IIR LPF(pos_commands_)
                                                      ▼
                                              arm_pos_cmd_buffer_ (filtered)
                                                      │ @500 Hz CAN:MIT
                                                      ▼
                                                  Motor
                                                      │
                                              _state = hardware feedback

  _cmd   = JTC reference（未經 LPF，三組幾乎相同）
  _state = 馬達實際位置（受 LPF 平滑後的指令驅動）
  _err   = cmd − state（LPF 引入相位延遲，誤差在快速轉折處增大）

使用方式:
  python3 analyze_lpf_v2.py
  python3 analyze_lpf_v2.py --joints 1 3 5 7
  python3 analyze_lpf_v2.py --no-show
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

# LPF cutoff frequency per dataset (None = bypass)
CUTOFFS = {"No LPF": None, "20 Hz LPF": 20.0, "10 Hz LPF": 10.0}

# Plot style per dataset
DATASETS = {
    "No LPF":    {"file": FILE_NO,  "color": "#9E9E9E", "lw": 1.2, "cutoff": None},
    "20 Hz LPF": {"file": FILE_20,  "color": "#FF5722", "lw": 1.0, "cutoff": 20.0},
    "10 Hz LPF": {"file": FILE_10,  "color": "#2196F3", "lw": 1.0, "cutoff": 10.0},
}

# Minimum dt threshold for numerical differentiation (avoids duplicate-timestamp spikes)
DT_MIN_DIFF = 1e-3   # 1 ms  (ROS2 controller state typically publishes at ~100–200 Hz)

# ---------------------------------------------------------------------------
# Load CSV
# ---------------------------------------------------------------------------
def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            vals = {k: float(v) for k, v in row.items()}
            # Drop rows that contain any non-finite value (startup NaN period)
            if all(math.isfinite(v) for v in vals.values()):
                rows.append(vals)
    data = {k: np.array([r[k] for r in rows]) for k in rows[0]}
    data["time"] = data["time"] - data["time"][0]
    return data


# ---------------------------------------------------------------------------
# Simulate first-order IIR LPF  (same formula as OpenArm_v10LPF_HW::write)
#   alpha = dt / (rc + dt),  rc = 1 / (2π * cutoff_hz)
# ---------------------------------------------------------------------------
def simulate_lpf(signal, time_arr, cutoff_hz):
    if cutoff_hz is None or cutoff_hz <= 0:
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
# Numerical differentiation — FIXED: skip samples where dt < DT_MIN_DIFF
#
# Why: the CSV sometimes has consecutive rows with identical ROS timestamps
# (dt=0). v1 clamped to 1e-9 s, producing ~6M rad/s spikes.
# Fix: return only the valid-dt subset as (values, corresponding timestamps).
# ---------------------------------------------------------------------------
def diff_valid(arr, time_arr, dt_min=DT_MIN_DIFF):
    """
    Returns (deriv, t_mid) only for steps where dt >= dt_min.
    Both arrays are shorter than arr by at least 1 element.
    """
    dt    = np.diff(time_arr)
    dx    = np.diff(arr)
    valid = dt >= dt_min
    deriv = dx[valid] / dt[valid]
    t_mid = (time_arr[:-1] + time_arr[1:])[valid] / 2.0
    return deriv, t_mid


def vel_rms(sig, time_arr):
    """RMS of ∂sig/∂t, skipping duplicate-timestamp steps."""
    d, _ = diff_valid(sig, time_arr)
    if len(d) == 0:
        return float("nan")
    return math.sqrt(float(np.mean(d ** 2)))


def jerk_rms(sig, time_arr):
    """RMS of ∂²sig/∂t² — second derivative of state (smoothed motion indicator)."""
    d1, t1 = diff_valid(sig, time_arr)
    if len(d1) < 2:
        return float("nan")
    d2, _ = diff_valid(d1, t1)
    if len(d2) == 0:
        return float("nan")
    return math.sqrt(float(np.mean(d2 ** 2)))


def metrics(err):
    rmse = math.sqrt(float(np.nanmean(err ** 2)))
    mae  = float(np.nanmean(np.abs(err)))
    maxe = float(np.nanmax(np.abs(err)))
    return rmse, mae, maxe


def hf_energy_ratio(sig, time_arr, cutoff_hz=10.0):
    """
    Fraction of FFT energy above cutoff_hz relative to total energy.
    Lower → more high-frequency content removed → LPF more effective.
    """
    dt_avg = float(np.mean(np.diff(time_arr)))
    if dt_avg <= 0:
        return float("nan")
    n    = len(sig)
    mag  = np.abs(np.fft.rfft(sig - sig.mean())) ** 2
    freq = np.fft.rfftfreq(n, d=dt_avg)
    total = mag.sum()
    if total == 0:
        return float("nan")
    hf = mag[freq >= cutoff_hz].sum()
    return float(hf / total)


# ---------------------------------------------------------------------------
# Print summary table with improvement %
# ---------------------------------------------------------------------------
def print_table(datasets, joints):
    keys = list(datasets.keys())
    ref_key = keys[0]   # "No LPF" is the reference baseline

    print(f"\n{'='*100}")
    print("  LPF Analysis — State Smoothness & Tracking Error  (v2)")
    print(f"  NOTE: _cmd = JTC trajectory reference (unfiltered, same for all 3 runs)")
    print(f"        _state = actual motor position (affected by hardware LPF)")
    print(f"        diff fix: dt < {DT_MIN_DIFF*1e3:.0f} ms skipped (avoids duplicate-timestamp spikes)")
    print(f"{'='*100}")

    hdr = f"  {'Joint':<8} {'Metric':<35}"
    for k in keys:
        hdr += f"  {k:>14}"
    hdr += f"  {'20Hz improv%':>13}  {'10Hz improv%':>13}"
    print(hdr)
    print(f"  {'-'*98}")

    def improv(ref_val, val):
        if ref_val == 0 or not math.isfinite(ref_val) or not math.isfinite(val):
            return float("nan")
        return (ref_val - val) / abs(ref_val) * 100.0

    def fmt_pct(v):
        if not math.isfinite(v):
            return "      nan  "
        sign = "+" if v > 0 else ""
        return f"  {sign}{v:>9.2f}%"

    for j in joints:
        err_k = f"openarm_right_joint{j}_err"
        st_k  = f"openarm_right_joint{j}_state"

        vals = {}
        for k, info in datasets.items():
            d = info["data"]
            r, m, x = metrics(d[err_k])
            sv = vel_rms(d[st_k],  d["time"])
            jr = jerk_rms(d[st_k], d["time"])
            hf = hf_energy_ratio(d[st_k], d["time"], cutoff_hz=10.0)
            cv = vel_rms(d[f"openarm_right_joint{j}_cmd"], d["time"])
            vals[k] = {"rmse": r, "mae": m, "maxe": x,
                       "sv": sv, "jerk": jr, "hf": hf, "cv": cv}

        def row(label, key, lower_is_better=True):
            line = f"  J{j:<7} {label:<35}"
            row_vals = []
            for k in keys:
                line += f"  {vals[k][key]:>14.6f}"
                row_vals.append(vals[k][key])
            # improvement % (positive = improved)
            ref_v = vals[ref_key][key]
            for ki, k in enumerate(keys[1:], 1):
                v = vals[k][key]
                pct = improv(ref_v, v) if lower_is_better else improv(v, ref_v)
                line += fmt_pct(pct)
            return line

        print(row("RMSE(err)[rad] ↑ = LPF phase lag",   "rmse", lower_is_better=False))
        print(row("Max|err|[rad]",                       "maxe", lower_is_better=False))
        print(row("∂state/∂t RMS[rad/s] ↓ = smoother",  "sv"))
        print(row("Jerk RMS[rad/s²]  ↓ = less abrupt",  "jerk"))
        print(row("HF energy ratio[>10Hz] ↓ = less HF", "hf"))
        print(row("∂cmd/∂t RMS[rad/s] (ref only)",       "cv"))
        print(f"  {'-'*98}")


# ---------------------------------------------------------------------------
# Plot 1 — Simulated motor command vs actual state
# ---------------------------------------------------------------------------
def plot_sim_lpf(datasets, joints, show):
    nj  = len(joints)
    fig, axes = plt.subplots(nj, 3, figsize=(19, 3.8 * nj))
    if nj == 1:
        axes = [axes]

    for row_i, j in enumerate(joints):
        cmd_k = f"openarm_right_joint{j}_cmd"
        st_k  = f"openarm_right_joint{j}_state"
        ax0, ax1, ax2 = axes[row_i]

        d_ref = datasets["No LPF"]["data"]
        ax0.plot(d_ref["time"], d_ref[cmd_k],
                 color="#9E9E9E", lw=1.5, label="JTC reference (raw cmd)", zorder=5)

        for label, info in datasets.items():
            c = info["cutoff"]
            if c is None:
                continue
            d = info["data"]
            sim = simulate_lpf(d[cmd_k], d["time"], c)
            ax0.plot(d["time"], sim, color=info["color"], lw=1.0,
                     ls="--", label=f"sim LPF({c:.0f}Hz) → motor")

        ax0.set_title(f"J{j}  JTC cmd vs simulated motor cmd\n(dashed = what actually reaches motor)", fontsize=8)
        ax0.set_ylabel("rad"); ax0.legend(fontsize=6.5); ax0.grid(True, alpha=0.3)

        for label, info in datasets.items():
            d = info["data"]
            ax1.plot(d["time"], d[st_k], color=info["color"], lw=info["lw"], label=label)
        ax1.set_title(f"J{j}  Actual motor state\n(hardware feedback)", fontsize=8)
        ax1.set_ylabel("rad"); ax1.legend(fontsize=6.5); ax1.grid(True, alpha=0.3)

        for label, info in datasets.items():
            d = info["data"]
            c = info["cutoff"]
            sim = simulate_lpf(d[cmd_k], d["time"], c) if c else d[cmd_k]
            residual = d[st_k] - sim
            ax2.plot(d["time"], residual, color=info["color"], lw=0.8, label=label)
        ax2.axhline(0, color="k", lw=0.5, ls="--")
        ax2.set_title(f"J{j}  state − motor_cmd residual\n(phase lag excluded)", fontsize=8)
        ax2.set_ylabel("rad"); ax2.legend(fontsize=6.5); ax2.grid(True, alpha=0.3)

    for ax_row in axes:
        for ax in ax_row:
            ax.set_xlabel("time (s)")

    fig.suptitle(
        "LPF Analysis — Simulated Motor Command vs Hardware State\n"
        "Col 1: JTC cmd + simulated filtered motor cmd  |  Col 2: actual state  |  Col 3: residual",
        fontsize=11, y=1.01)
    plt.tight_layout()
    _save(fig, "lpf_v2_sim_motor_cmd.png", show)


# ---------------------------------------------------------------------------
# Plot 2 — State velocity smoothness + Jerk
# ---------------------------------------------------------------------------
def plot_state_smoothness(datasets, joints, show):
    nj  = len(joints)
    fig, axes = plt.subplots(nj, 2, figsize=(15, 3.2 * nj))
    if nj == 1:
        axes = [axes]

    for row_i, j in enumerate(joints):
        st_k = f"openarm_right_joint{j}_state"
        ax0, ax1 = axes[row_i]

        for label, info in datasets.items():
            d = info["data"]
            v, tv = diff_valid(d[st_k], d["time"])
            rms = math.sqrt(float(np.mean(v ** 2))) if len(v) else float("nan")
            ax0.plot(tv, v, color=info["color"], lw=0.7, alpha=0.85,
                     label=f"{label}  RMS={rms:.4f}")

        ax0.set_title(f"J{j}  ∂state/∂t  (motor velocity)", fontsize=8)
        ax0.set_ylabel("rad/s"); ax0.legend(fontsize=6.5); ax0.grid(True, alpha=0.3)

        for label, info in datasets.items():
            d = info["data"]
            v1, tv1 = diff_valid(d[st_k], d["time"])
            v2, _   = diff_valid(v1, tv1)
            if len(v2) == 0:
                continue
            jrms = math.sqrt(float(np.mean(v2 ** 2)))
            ax1.plot(tv1[1:len(tv1)-len(tv1)+len(v2)+1][:len(v2)],
                     v2, color=info["color"], lw=0.6, alpha=0.75,
                     label=f"{label}  RMS={jrms:.4f}")

        ax1.set_title(f"J{j}  ∂²state/∂t²  (jerk — less = smoother change)", fontsize=8)
        ax1.set_ylabel("rad/s²"); ax1.legend(fontsize=6.5); ax1.grid(True, alpha=0.3)

    for ax_row in axes:
        ax_row[0].set_xlabel("time (s)")
        ax_row[1].set_xlabel("time (s)")

    fig.suptitle(
        "LPF Analysis — State Velocity & Jerk\n"
        "Lower RMS = smoother motion  (key proof of LPF effectiveness)",
        fontsize=11, y=1.01)
    plt.tight_layout()
    _save(fig, "lpf_v2_state_smoothness.png", show)


# ---------------------------------------------------------------------------
# Plot 3 — FFT of state
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
            n      = len(d[st_k])
            mag    = np.abs(np.fft.rfft(d[st_k] - d[st_k].mean())) * 2.0 / n
            freq   = np.fft.rfftfreq(n, d=dt_avg)
            hf_r   = hf_energy_ratio(d[st_k], d["time"], cutoff_hz=10.0)
            ax.semilogy(freq, mag + 1e-9, color=info["color"],
                        lw=1.0, label=f"{label}  HF>10Hz={hf_r:.4f}")

        ax.axvline(10, color="#2196F3", ls="--", lw=1.0, alpha=0.9, label="cutoff 10 Hz")
        ax.axvline(20, color="#FF5722", ls="--", lw=1.0, alpha=0.9, label="cutoff 20 Hz")
        ax.set_title(f"J{j}  FFT of hardware state (log scale)  — HF>10Hz ratio in legend", fontsize=8)
        ax.set_xlabel("frequency (Hz)"); ax.set_ylabel("|X(f)| rad")
        ax.set_xlim(0, 50); ax.legend(fontsize=6.5); ax.grid(True, alpha=0.3)

    fig.suptitle(
        "LPF Analysis — Frequency Content of Hardware State\n"
        "10 Hz LPF should show least energy above cutoff  |  HF ratio = fraction of energy > 10 Hz",
        fontsize=11, y=1.01)
    plt.tight_layout()
    _save(fig, "lpf_v2_fft_state.png", show)


# ---------------------------------------------------------------------------
# Plot 4 — Tracking error
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
        ax0.set_ylabel("rad"); ax0.legend(fontsize=6.5); ax0.grid(True, alpha=0.3)

        for label, info in datasets.items():
            d = info["data"]
            ax1.hist(d[err_k], bins=80, color=info["color"], alpha=0.55, label=label)
        ax1.set_title(f"J{j}  Error distribution", fontsize=8)
        ax1.set_xlabel("rad"); ax1.legend(fontsize=6.5); ax1.grid(True, alpha=0.3)

    for ax_row in axes:
        ax_row[0].set_xlabel("time (s)")

    fig.suptitle(
        "LPF Analysis — Tracking Error (JTC reference − actual state)\n"
        "LPF introduces phase-lag → larger RMSE at fast transitions (expected cost)",
        fontsize=11, y=1.01)
    plt.tight_layout()
    _save(fig, "lpf_v2_tracking_error.png", show)


# ---------------------------------------------------------------------------
# Plot 5 — Summary bars (RMSE + state velocity RMS)
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
            h = bar.get_height()
            if math.isfinite(h):
                ax1.text(bar.get_x() + bar.get_width()/2, h,
                         f"{h:.4f}", ha="center", va="bottom", fontsize=5.5)

        bars2 = ax2.bar(x + off, sv_vals, w, label=k, color=col, alpha=0.85)
        for bar in bars2:
            h = bar.get_height()
            if math.isfinite(h):
                ax2.text(bar.get_x() + bar.get_width()/2, h,
                         f"{h:.4f}", ha="center", va="bottom", fontsize=5.5)

    ax1.set_xticks(x); ax1.set_xticklabels(xticks)
    ax1.set_ylabel("RMSE (rad)")
    ax1.set_title("Tracking Error RMSE\n(larger with LPF = expected phase-lag cost)")
    ax1.legend(fontsize=8); ax1.grid(True, axis="y", alpha=0.3)

    ax2.set_xticks(x); ax2.set_xticklabels(xticks)
    ax2.set_ylabel("∂state/∂t RMS (rad/s)")
    ax2.set_title("State Velocity RMS\n(KEY: lower = smoother motion)")
    ax2.legend(fontsize=8); ax2.grid(True, axis="y", alpha=0.3)

    fig.suptitle("LPF Analysis Summary — No LPF vs 20 Hz vs 10 Hz", fontsize=13)
    plt.tight_layout()
    _save(fig, "lpf_v2_summary_bars.png", show)


# ---------------------------------------------------------------------------
# Plot 6 (NEW) — LPF Report Card: improvement % per joint per metric
# ---------------------------------------------------------------------------
def plot_report_card(datasets, joints, show):
    """
    Heat-map + bar chart showing improvement percentage vs No LPF baseline.
    Green = improved, Red = degraded.
    Metrics shown: ∂state/∂t RMS (smoothness), Jerk RMS, HF energy ratio.
    """
    keys    = list(datasets.keys())
    ref_key = keys[0]  # "No LPF"
    lpf_keys = keys[1:]  # ["20 Hz LPF", "10 Hz LPF"]
    colors   = [datasets[k]["color"] for k in lpf_keys]

    metric_labels = ["∂state/∂t RMS", "Jerk RMS", "HF energy\n>10Hz ratio"]
    metric_keys   = ["sv", "jerk", "hf"]

    # Compute improvement % for each joint × metric × lpf_key
    improv_data = {k: {mk: [] for mk in metric_keys} for k in lpf_keys}
    for j in joints:
        st_k = f"openarm_right_joint{j}_state"
        d_ref = datasets[ref_key]["data"]
        ref_sv   = vel_rms(d_ref[st_k], d_ref["time"])
        ref_jerk = jerk_rms(d_ref[st_k], d_ref["time"])
        ref_hf   = hf_energy_ratio(d_ref[st_k], d_ref["time"])

        for k in lpf_keys:
            d = datasets[k]["data"]
            sv_v   = vel_rms(d[st_k], d["time"])
            jerk_v = jerk_rms(d[st_k], d["time"])
            hf_v   = hf_energy_ratio(d[st_k], d["time"])
            # positive % = improvement (lower is better for all 3 metrics)
            pct = lambda ref, val: (ref - val) / abs(ref) * 100 if abs(ref) > 1e-12 else 0.0
            improv_data[k]["sv"].append(pct(ref_sv, sv_v))
            improv_data[k]["jerk"].append(pct(ref_jerk, jerk_v))
            improv_data[k]["hf"].append(pct(ref_hf, hf_v))

    n_metrics = len(metric_keys)
    n_lpf     = len(lpf_keys)
    fig, axes = plt.subplots(1, n_metrics, figsize=(5 * n_metrics, max(4, 0.5 * len(joints) + 2)))

    x     = np.arange(len(joints))
    jlbls = [f"J{j}" for j in joints]

    for mi, (mk, mlabel) in enumerate(zip(metric_keys, metric_labels)):
        ax  = axes[mi]
        w   = 0.35
        offsets = [-w/2, w/2] if n_lpf == 2 else [0]

        for ki, (k, off, col) in enumerate(zip(lpf_keys, offsets, colors)):
            pcts = improv_data[k][mk]
            bar_colors = ["#4CAF50" if v >= 0 else "#F44336" for v in pcts]
            bars = ax.barh(x + off, pcts, w, color=bar_colors, alpha=0.85,
                           label=k, edgecolor="white", lw=0.5)
            for bar in bars:
                bw = bar.get_width()
                if math.isfinite(bw):
                    xpos = bw + 0.3 if bw >= 0 else bw - 0.3
                    ax.text(xpos, bar.get_y() + bar.get_height()/2,
                            f"{bw:+.1f}%", va="center", fontsize=6,
                            ha="left" if bw >= 0 else "right",
                            color="#2E7D32" if bw >= 0 else "#C62828")

        ax.axvline(0, color="black", lw=0.8)
        ax.set_yticks(x)
        ax.set_yticklabels(jlbls)
        ax.set_xlabel("Improvement vs No LPF (%)")
        ax.set_title(f"{mlabel}\n(green = better, red = worse)", fontsize=9)
        ax.legend(fontsize=7)
        ax.grid(True, axis="x", alpha=0.3)
        ax.invert_yaxis()

    fig.suptitle(
        "LPF Report Card — Improvement % vs No LPF Baseline\n"
        "Positive = LPF makes it better  |  All 3 metrics: lower is better",
        fontsize=12, y=1.02)
    plt.tight_layout()
    _save(fig, "lpf_v2_report_card.png", show)


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------
def _save(fig, fname, show):
    out = os.path.join(IMG_DIR, fname)
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"  [Saved] {out}")
    if show:
        plt.show()
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="LPF analysis v2 (fixed dt, jerk, HF energy, report card)")
    parser.add_argument("--joints", nargs="+", type=int, default=list(range(1, N_JOINTS + 1)))
    parser.add_argument("--no-show", action="store_true")
    args   = parser.parse_args()
    joints = args.joints
    show   = not args.no_show

    print("\nLoading CSV files …")
    for label, info in DATASETS.items():
        info["data"] = load_csv(info["file"])
        d = info["data"]
        dt = np.diff(d["time"])
        dup = (dt < DT_MIN_DIFF).sum()
        print(f"  {label:<12}: {len(d['time'])} samples, {d['time'][-1]:.2f} s, "
              f"dup-ts skipped={dup}/{len(dt)}")

    print(f"\nAnalysing joints: {joints}")
    print(f"[INFO] diff_valid threshold = {DT_MIN_DIFF*1e3:.0f} ms  (skips duplicate timestamps)")
    print("[INFO] LPF effect visible in: ∂state/∂t ↓, Jerk ↓, HF energy ↓")
    print("[INFO] Expected LPF cost: tracking RMSE ↑ (phase lag)\n")

    print_table(DATASETS, joints)

    print("\nGenerating plots …")
    plot_sim_lpf(DATASETS, joints, show)
    plot_state_smoothness(DATASETS, joints, show)
    plot_fft_state(DATASETS, joints, show)
    plot_tracking_error(DATASETS, joints, show)
    plot_summary_bars(DATASETS, joints, show)
    plot_report_card(DATASETS, joints, show)

    print(f"\nAll images saved to: {IMG_DIR}\nDone.\n")


if __name__ == "__main__":
    main()
