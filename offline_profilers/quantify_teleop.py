#!/usr/bin/env python3
"""
quantify_teleop.py
==================
Quantitative baseline report for placo IK + VR teleop sessions.

Reads CSVs produced by `placo_ik_main.py` and computes the
four metric families from `docs/vr_realtime_ik_analysis.md`:

  1. Realtime         — ik_ms p50/p95/max, loop period jitter, deadline miss%
  2. Tracking         — pos/ori/track err p50/p95, end-to-end lag (if available)
  3. Smoothness       — max_joint_delta p95/p99, guard hit%, EE jerk, FFT bands
  4. Robustness       — success rate, low-sigma time fraction, λ_dls activity

Optional columns (from the Set1-A extension) unlock extra panels:
  q_cmd_0..6        → per-joint FFT bands
  tf_x,tf_y,tf_z    → closed-loop EE error (commanded vs actually achieved)
  tracker_t_recv    → wall-to-wall tracker→IK input lag

Usage:
  python3 quantify_teleop.py <csv> [<csv> ...] [--out report.md] [--no-plot]
  python3 quantify_teleop.py 20260521/*.csv --label-from-name --out 0521.md
"""
from __future__ import annotations

import argparse
import csv as _csv
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except Exception:
    HAVE_MPL = False


# ── CSV loading ───────────────────────────────────────────────────────────────
def _load_csv(path: str) -> Dict[str, np.ndarray]:
    """Load CSV → dict of column → float array. Empty cells become NaN."""
    with open(path, newline="") as fh:
        reader = _csv.reader(fh)
        header = next(reader)
        cols: List[List[float]] = [[] for _ in header]
        for row in reader:
            for i, v in enumerate(row):
                if i >= len(cols):
                    continue
                if v == "" or v is None:
                    cols[i].append(np.nan)
                else:
                    try:
                        cols[i].append(float(v))
                    except ValueError:
                        cols[i].append(np.nan)
    return {h: np.asarray(cols[i], dtype=float) for i, h in enumerate(header)}


# ── Metric primitives ─────────────────────────────────────────────────────────
def _pct(x: np.ndarray, p: float) -> float:
    x = x[np.isfinite(x)]
    return float(np.percentile(x, p)) if x.size else float("nan")


def _fraction(x: np.ndarray, cond: np.ndarray) -> float:
    n = int(np.sum(np.isfinite(x)))
    return float(np.sum(cond & np.isfinite(x))) / max(n, 1)


def _ee_jerk(x: np.ndarray, y: np.ndarray, z: np.ndarray, t: np.ndarray) -> np.ndarray:
    """3rd derivative magnitude of EE position. Returns mm/s^3."""
    dt = np.diff(t)
    dt = np.where(dt > 1e-4, dt, 1e-4)
    vx, vy, vz = np.diff(x) / dt, np.diff(y) / dt, np.diff(z) / dt
    ax = np.diff(vx) / dt[1:]
    ay = np.diff(vy) / dt[1:]
    az = np.diff(vz) / dt[1:]
    jx = np.diff(ax) / dt[2:]
    jy = np.diff(ay) / dt[2:]
    jz = np.diff(az) / dt[2:]
    return np.sqrt(jx * jx + jy * jy + jz * jz) * 1000.0   # mm/s^3


def _band_power(sig: np.ndarray, fs: float, lo: float, hi: float) -> float:
    """Welch-style band power in [lo, hi] Hz. Returns mean PSD * BW."""
    sig = sig[np.isfinite(sig)]
    if sig.size < 32:
        return float("nan")
    sig = sig - np.mean(sig)
    # Hann window FFT (one-shot; good enough for diagnostic)
    win = np.hanning(sig.size)
    spec = np.fft.rfft(sig * win)
    freqs = np.fft.rfftfreq(sig.size, d=1.0 / fs)
    psd = (np.abs(spec) ** 2) / (np.sum(win ** 2) * fs)
    mask = (freqs >= lo) & (freqs <= hi)
    if not np.any(mask):
        return float("nan")
    return float(np.sum(psd[mask]) * (freqs[1] - freqs[0]))


# ── Per-session metric block ──────────────────────────────────────────────────
@dataclass
class SessionMetrics:
    label:    str
    path:     str
    n_steps:  int
    duration_sec: float
    realtime:   Dict[str, float] = field(default_factory=dict)
    tracking:   Dict[str, float] = field(default_factory=dict)
    smoothness: Dict[str, float] = field(default_factory=dict)
    robustness: Dict[str, float] = field(default_factory=dict)
    extras:     Dict[str, float] = field(default_factory=dict)   # opt'l columns


def _compute(label: str, path: str, d: Dict[str, np.ndarray]) -> SessionMetrics:
    t = d["t"]
    n = t.size
    dur = float(t[-1] - t[0]) if n > 1 else 0.0
    m = SessionMetrics(label=label, path=path, n_steps=n, duration_sec=dur)

    # 1. Realtime
    ik = d["ik_ms"]
    dlm = d["deadline_missed"]
    loop_dt_ms = np.diff(t) * 1000.0 if n > 1 else np.array([np.nan])
    nominal_dt = float(np.nanmedian(loop_dt_ms)) if loop_dt_ms.size else float("nan")
    m.realtime = {
        "ik_ms_p50":         _pct(ik, 50),
        "ik_ms_p95":         _pct(ik, 95),
        "ik_ms_max":         float(np.nanmax(ik)) if ik.size else float("nan"),
        "deadline_miss_pct": 100.0 * _fraction(dlm, dlm > 0.5),
        "loop_dt_ms_med":    nominal_dt,
        "loop_dt_ms_std":    float(np.nanstd(loop_dt_ms)),
        "rate_hz_est":       1000.0 / nominal_dt if nominal_dt and nominal_dt > 0 else float("nan"),
    }

    # 2. Tracking
    pos_err = d.get("pos_err_mm", np.array([]))
    ori_err = d.get("ori_err_deg", np.array([]))
    trk_err = d.get("track_err_mm", np.array([]))
    m.tracking = {
        "pos_err_mm_p50":  _pct(pos_err, 50),
        "pos_err_mm_p95":  _pct(pos_err, 95),
        "ori_err_deg_p50": _pct(ori_err, 50),
        "ori_err_deg_p95": _pct(ori_err, 95),
        "track_err_mm_p50": _pct(trk_err, 50),
        "track_err_mm_p95": _pct(trk_err, 95),
    }

    if "tracker_t_recv" in d:
        # End-to-end lag: log timestamp `t` is captured just after IK solve;
        # `tracker_t_recv` is when the source pose arrived. Difference ≈ input-to-IK lag.
        lag = (d["t"] - d["tracker_t_recv"]) * 1000.0
        lag = lag[np.isfinite(lag) & (lag >= 0) & (lag < 1000)]
        if lag.size:
            m.tracking["tracker_lag_ms_p50"] = float(np.percentile(lag, 50))
            m.tracking["tracker_lag_ms_p95"] = float(np.percentile(lag, 95))

    # 3. Smoothness
    jump = d.get("max_joint_delta_deg", np.array([]))
    guard = d.get("joint_jump_guard", np.array([]))
    m.smoothness = {
        "max_joint_delta_deg_p95": _pct(jump, 95),
        "max_joint_delta_deg_p99": _pct(jump, 99),
        "guard_hit_pct":           100.0 * _fraction(guard, guard > 0.5),
    }
    if n > 4:
        jerk = _ee_jerk(d["x"], d["y"], d["z"], t)
        m.smoothness["ee_jerk_p50_mm_s3"] = _pct(jerk, 50)
        m.smoothness["ee_jerk_p95_mm_s3"] = _pct(jerk, 95)

    # Per-joint FFT bands (only when q_cmd_0..6 exist)
    fs = m.realtime["rate_hz_est"]
    if "q_cmd_0" in d and np.isfinite(fs) and fs > 4.0:
        lo_power = []
        hi_power = []
        for i in range(7):
            col = f"q_cmd_{i}"
            if col not in d:
                continue
            sig = d[col]
            # Resample-by-mask: drop guard-rejected NaN gaps, fill with previous
            mask = np.isfinite(sig)
            if not np.any(mask):
                continue
            sig_f = sig.copy()
            for k in range(1, sig_f.size):
                if not np.isfinite(sig_f[k]):
                    sig_f[k] = sig_f[k - 1]
            sig_f[~np.isfinite(sig_f)] = 0.0
            lo_power.append(_band_power(sig_f, fs, 0.5, 5.0))
            hi_power.append(_band_power(sig_f, fs, 10.0, min(fs * 0.45, 50.0)))
        if lo_power:
            m.extras["qcmd_lo_band_mean"] = float(np.nanmean(lo_power))
            m.extras["qcmd_hi_band_mean"] = float(np.nanmean(hi_power))
            base_hi = np.nanmean(hi_power[:3]) if len(hi_power) >= 3 else float("nan")
            wrist_hi = np.nanmean(hi_power[4:]) if len(hi_power) >= 5 else float("nan")
            m.extras["qcmd_hi_wrist_over_base"] = (wrist_hi / base_hi) if base_hi else float("nan")

    # 4. Robustness
    succ = d.get("success", np.array([]))
    sig_min = d.get("sigma_min", np.array([]))
    lam = d.get("lambda_dls", np.array([]))
    m.robustness = {
        "success_pct":          100.0 * _fraction(succ, succ > 0.5),
        "sigma_low_pct":        100.0 * _fraction(sig_min, sig_min < 0.05),
        "sigma_min_p05":        _pct(sig_min, 5),
        "lambda_dls_max":       float(np.nanmax(lam)) if lam.size else float("nan"),
    }

    # TF closed-loop error (optional)
    if "tf_x" in d and np.any(np.isfinite(d["tf_x"])):
        dx = d["x"] - d["tf_x"]
        dy = d["y"] - d["tf_y"]
        dz = d["z"] - d["tf_z"]
        cl = np.sqrt(dx * dx + dy * dy + dz * dz) * 1000.0
        m.extras["cl_err_mm_p50"] = _pct(cl, 50)
        m.extras["cl_err_mm_p95"] = _pct(cl, 95)

    return m


# ── Pass/fail thresholds for grading ──────────────────────────────────────────
# Each entry: (green_limit, yellow_limit, direction)
#   direction = "lo" → lower is better (most metrics)
#   direction = "hi" → higher is better (e.g. success%)
# Metric is GREEN if it passes the green_limit, YELLOW if it only passes the
# yellow_limit, otherwise RED. Metrics not in this dict get no colour
# (e.g. purely informational columns like λ_dls_max or EE jerk).
THRESHOLDS: Dict[str, Tuple[float, float, str]] = {
    # Realtime
    "ik_ms_p50":              (2.0,  5.0,  "lo"),
    "ik_ms_p95":              (3.0,  10.0, "lo"),
    "ik_ms_max":              (10.0, 20.0, "lo"),
    "deadline_miss_pct":      (0.1,  0.5,  "lo"),
    "loop_dt_ms_std":         (2.0,  10.0, "lo"),
    # Tracking
    "pos_err_mm_p50":         (3.0,  5.0,  "lo"),
    "pos_err_mm_p95":         (5.0,  10.0, "lo"),
    "ori_err_deg_p50":        (0.5,  1.5,  "lo"),
    "ori_err_deg_p95":        (3.0,  5.0,  "lo"),
    "track_err_mm_p50":       (3.0,  5.0,  "lo"),
    "track_err_mm_p95":       (8.0,  15.0, "lo"),
    "tracker_lag_ms_p50":     (30.0, 60.0, "lo"),
    "tracker_lag_ms_p95":     (60.0, 100.0, "lo"),
    # Smoothness
    "max_joint_delta_deg_p95": (5.0,  10.0, "lo"),
    "max_joint_delta_deg_p99": (8.0,  15.0, "lo"),
    "guard_hit_pct":          (0.1,  1.0,  "lo"),
    "qcmd_hi_wrist_over_base": (3.0,  10.0, "lo"),
    # Robustness
    "success_pct":            (99.0, 95.0, "hi"),
    "sigma_low_pct":          (5.0,  15.0, "lo"),
    "sigma_min_p05":          (0.1,  0.05, "hi"),
    "cl_err_mm_p50":          (5.0,  15.0, "lo"),
    "cl_err_mm_p95":          (10.0, 30.0, "lo"),
}

GRADE_GREEN  = "🟢"
GRADE_YELLOW = "🟡"
GRADE_RED    = "🔴"


def _grade(value: float, key: str) -> str:
    """Return emoji for the value vs THRESHOLDS[key]. '' if no threshold or NaN."""
    if value is None or not np.isfinite(value):
        return ""
    spec = THRESHOLDS.get(key)
    if spec is None:
        return ""
    g, y, direction = spec
    if direction == "lo":
        if value <= g:
            return GRADE_GREEN
        if value <= y:
            return GRADE_YELLOW
        return GRADE_RED
    else:  # higher better
        if value >= g:
            return GRADE_GREEN
        if value >= y:
            return GRADE_YELLOW
        return GRADE_RED


# ── Markdown report ───────────────────────────────────────────────────────────
def _fmt(v: float, n: int = 2) -> str:
    if v is None or not np.isfinite(v):
        return "—"
    return f"{v:.{n}f}"


def _fmt_graded(v: float, n: int, key: str) -> str:
    """Format with leading emoji; falls back to plain number when no threshold."""
    badge = _grade(v, key)
    text  = _fmt(v, n)
    return f"{badge} {text}" if badge else text


def _threshold_legend() -> str:
    """Render the pass/fail thresholds as a compact reference table."""
    pretty = {
        "ik_ms_p50": "ik_ms p50",
        "ik_ms_p95": "ik_ms p95",
        "ik_ms_max": "ik_ms max",
        "deadline_miss_pct": "deadline miss %",
        "loop_dt_ms_std": "loop Δt std (ms)",
        "pos_err_mm_p50": "pos_err_mm p50",
        "pos_err_mm_p95": "pos_err_mm p95",
        "ori_err_deg_p50": "ori_err_deg p50",
        "ori_err_deg_p95": "ori_err_deg p95",
        "track_err_mm_p50": "track_err_mm p50",
        "track_err_mm_p95": "track_err_mm p95",
        "tracker_lag_ms_p50": "tracker→IK lag p50 ms",
        "tracker_lag_ms_p95": "tracker→IK lag p95 ms",
        "max_joint_delta_deg_p95": "max |Δq| deg p95",
        "max_joint_delta_deg_p99": "max |Δq| deg p99",
        "guard_hit_pct": "guard hit %",
        "qcmd_hi_wrist_over_base": "q_cmd hi band wrist/base",
        "success_pct": "success %",
        "sigma_low_pct": "σ_min<0.05 time %",
        "sigma_min_p05": "σ_min p05",
        "cl_err_mm_p50": "closed-loop err p50 mm",
        "cl_err_mm_p95": "closed-loop err p95 mm",
    }
    rows = ["| Metric | 🟢 if | 🟡 if (else 🔴) |", "|---|---|---|"]
    for k, (g, y, d) in THRESHOLDS.items():
        op = "≤" if d == "lo" else "≥"
        rows.append(f"| {pretty.get(k, k)} | {op} {g} | {op} {y} |")
    return "\n".join(rows)


def _markdown_report(sessions: List[SessionMetrics]) -> str:
    lines: List[str] = []
    lines.append("# Teleop Quantitative Report\n")
    lines.append(f"_Sessions analysed: {len(sessions)}_\n")
    lines.append(
        "_Grading: 🟢 = passes target · 🟡 = acceptable · 🔴 = fails target · "
        "(blank = informational, no threshold)._\n"
    )

    # Header summary
    lines.append("## Session summary\n")
    lines.append("| Label | Steps | Duration (s) | Est. rate (Hz) |")
    lines.append("|---|---:|---:|---:|")
    for s in sessions:
        lines.append(f"| {s.label} | {s.n_steps} | {_fmt(s.duration_sec, 1)} "
                     f"| {_fmt(s.realtime['rate_hz_est'], 1)} |")
    lines.append("")

    def _table(title: str, rows: List[Tuple[str, str, str]]):
        """rows = [(metric_label, fmt_digits, dict_key), ...] from one of the 4 blocks."""
        lines.append(f"## {title}\n")
        head = "| Metric | " + " | ".join(s.label for s in sessions) + " |"
        sep  = "|---|" + "---|" * len(sessions)
        lines.append(head)
        lines.append(sep)
        for lbl, digits, key in rows:
            cells = [lbl]
            for s in sessions:
                block = getattr(s, title.split()[0].lower(), {})
                if not isinstance(block, dict):
                    block = s.extras
                v = block.get(key)
                if v is None:
                    v = s.extras.get(key)
                cells.append(_fmt_graded(v, int(digits), key))
            lines.append("| " + " | ".join(cells) + " |")
        lines.append("")

    _table("Realtime", [
        ("ik_ms p50",          "2", "ik_ms_p50"),
        ("ik_ms p95",          "2", "ik_ms_p95"),
        ("ik_ms max",          "2", "ik_ms_max"),
        ("deadline miss %",    "2", "deadline_miss_pct"),
        ("loop Δt median (ms)", "2", "loop_dt_ms_med"),
        ("loop Δt std (ms)",   "2", "loop_dt_ms_std"),
    ])
    _table("Tracking", [
        ("pos_err_mm p50",      "2", "pos_err_mm_p50"),
        ("pos_err_mm p95",      "2", "pos_err_mm_p95"),
        ("ori_err_deg p50",     "2", "ori_err_deg_p50"),
        ("ori_err_deg p95",     "2", "ori_err_deg_p95"),
        ("track_err_mm p50",    "2", "track_err_mm_p50"),
        ("track_err_mm p95",    "2", "track_err_mm_p95"),
        ("tracker→IK lag p50 ms", "1", "tracker_lag_ms_p50"),
        ("tracker→IK lag p95 ms", "1", "tracker_lag_ms_p95"),
    ])
    _table("Smoothness", [
        ("max |Δq| deg p95",  "2", "max_joint_delta_deg_p95"),
        ("max |Δq| deg p99",  "2", "max_joint_delta_deg_p99"),
        ("guard hit %",       "2", "guard_hit_pct"),
        ("EE jerk p50 (mm/s³)", "1", "ee_jerk_p50_mm_s3"),
        ("EE jerk p95 (mm/s³)", "1", "ee_jerk_p95_mm_s3"),
        ("q_cmd hi band wrist/base", "2", "qcmd_hi_wrist_over_base"),
    ])
    _table("Robustness", [
        ("success %",            "2", "success_pct"),
        ("σ_min<0.05 time %",    "2", "sigma_low_pct"),
        ("σ_min p05",            "4", "sigma_min_p05"),
        ("λ_dls max",            "5", "lambda_dls_max"),
        ("closed-loop err p50 mm", "2", "cl_err_mm_p50"),
        ("closed-loop err p95 mm", "2", "cl_err_mm_p95"),
    ])

    lines.append("## Files\n")
    for s in sessions:
        lines.append(f"- `{s.label}` ← {s.path}")
    lines.append("")

    lines.append("## Threshold reference\n")
    lines.append(_threshold_legend())
    lines.append("")
    return "\n".join(lines)


# ── Plot ──────────────────────────────────────────────────────────────────────
def _plot(sessions: List[SessionMetrics], data: List[Dict[str, np.ndarray]],
          out_png: str) -> None:
    if not HAVE_MPL:
        print("  [plot] matplotlib not available — skipping PNG")
        return
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    colors = plt.cm.tab10(np.linspace(0, 1, len(sessions)))

    # (1) ik_ms CDF
    ax = axes[0, 0]
    for s, d, c in zip(sessions, data, colors):
        x = d["ik_ms"]
        x = np.sort(x[np.isfinite(x)])
        if x.size:
            ax.plot(x, np.linspace(0, 1, x.size), color=c, label=s.label)
    ax.set_xlabel("ik_ms"); ax.set_ylabel("CDF"); ax.set_title("IK latency CDF")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    ax.set_xscale("log")

    # (2) pos_err_mm CDF
    ax = axes[0, 1]
    for s, d, c in zip(sessions, data, colors):
        x = d.get("pos_err_mm", np.array([]))
        x = np.sort(x[np.isfinite(x)])
        if x.size:
            ax.plot(x, np.linspace(0, 1, x.size), color=c, label=s.label)
    ax.set_xlabel("pos_err_mm"); ax.set_ylabel("CDF"); ax.set_title("Position error CDF")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)

    # (3) max joint delta hist
    ax = axes[1, 0]
    for s, d, c in zip(sessions, data, colors):
        x = d.get("max_joint_delta_deg", np.array([]))
        x = x[np.isfinite(x)]
        if x.size:
            ax.hist(x, bins=60, alpha=0.45, color=c, label=s.label,
                    histtype="stepfilled")
    ax.set_xlabel("max |Δq| deg / step"); ax.set_ylabel("count")
    ax.set_title("Per-step joint jump distribution")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)

    # (4) sigma_min over time (last session only — avoid clutter)
    ax = axes[1, 1]
    s, d = sessions[-1], data[-1]
    sig = d.get("sigma_min", np.array([]))
    if sig.size and np.any(np.isfinite(sig)):
        t_rel = d["t"] - d["t"][0]
        ax.plot(t_rel, sig, lw=0.6, color="tab:purple", label=f"σ_min ({s.label})")
        ax.axhline(0.05, color="red", lw=0.5, ls="--", label="σ=0.05 threshold")
    ax.set_xlabel("t (s)"); ax.set_ylabel("σ_min(J)")
    ax.set_title("Singularity proximity (last session)")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
    print(f"  [plot] wrote {out_png}")


# ── Main ──────────────────────────────────────────────────────────────────────
def _derive_label(path: str, mode: str) -> str:
    base = os.path.basename(path).replace(".csv", "")
    if mode == "name":
        return base
    # Default: timestamp + arm + mode, e.g. "140801_right_cached"
    parts = base.split("_")
    return "_".join(parts[-3:]) if len(parts) >= 3 else base


# Default results root = sibling `results/` next to this script's parent
# (script lives at placo_ik/offline_profilers/, CSVs at placo_ik/results/).
RESULTS_ROOT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, "results"))


def _pick_int_range(prompt: str, lo: int, hi: int) -> int:
    """Prompt until the user types an int in [lo, hi]."""
    while True:
        s = input(prompt).strip()
        if s.isdigit():
            n = int(s)
            if lo <= n <= hi:
                return n
        print(f"  ! please enter a number between {lo} and {hi}")


def _pick_multi(prompt: str, n: int) -> List[int]:
    """Prompt for one or more file indices, e.g. '1,3' or '1 3' or 'all'."""
    while True:
        s = input(prompt).strip().lower()
        if s in ("all", "*"):
            return list(range(1, n + 1))
        parts = [p for p in s.replace(",", " ").split() if p]
        try:
            picks = [int(p) for p in parts]
        except ValueError:
            print("  ! integers only (e.g. '1 3' or '1,3' or 'all')")
            continue
        if picks and all(1 <= k <= n for k in picks):
            return picks
        print(f"  ! every index must be between 1 and {n}")


def _interactive_pick(root: str) -> Tuple[List[str], str]:
    """Walk the user through folder→file selection. Returns (csv_paths, out_dir)."""
    # Step 1: pick subfolder (or root itself if it directly contains CSVs).
    subdirs = sorted(
        d for d in os.listdir(root)
        if os.path.isdir(os.path.join(root, d)) and not d.startswith(".")
    )
    csvs_in_root = sorted(f for f in os.listdir(root) if f.endswith(".csv"))

    options: List[Tuple[str, str]] = []
    if csvs_in_root:
        options.append(("(this folder)", root))
    for d in subdirs:
        options.append((d, os.path.join(root, d)))

    if not options:
        print(f"  no CSV files or subfolders under {root}")
        sys.exit(1)

    print(f"\n  Results root: {root}")
    print("  Pick a folder:")
    for i, (lbl, _) in enumerate(options, 1):
        print(f"    {i:>2}. {lbl}")
    idx = _pick_int_range("  folder # > ", 1, len(options))
    chosen_dir = options[idx - 1][1]

    # Step 2: pick CSV file(s) in the chosen folder.
    csvs = sorted(f for f in os.listdir(chosen_dir) if f.endswith(".csv"))
    if not csvs:
        print(f"  no CSV files in {chosen_dir}")
        sys.exit(1)

    print(f"\n  CSVs in {chosen_dir}:")
    for i, name in enumerate(csvs, 1):
        size_kb = os.path.getsize(os.path.join(chosen_dir, name)) // 1024
        print(f"    {i:>2}. {name}  ({size_kb} KB)")
    picks = _pick_multi(
        "  file # (comma/space-separated, or 'all') > ", len(csvs))
    paths = [os.path.join(chosen_dir, csvs[k - 1]) for k in picks]
    return paths, chosen_dir


def _default_out(out_dir: str) -> Tuple[str, str]:
    """Build default (md_path, png_path) inside out_dir."""
    import datetime as _dt
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    md  = os.path.join(out_dir, f"quantify_{stamp}.md")
    png = os.path.join(out_dir, f"quantify_{stamp}.png")
    return md, png


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv", nargs="*",
                   help="CSV file(s). Omit → interactive folder/file picker.")
    p.add_argument("--root", default=RESULTS_ROOT,
                   help=f"Results root for interactive picker (default: {RESULTS_ROOT})")
    p.add_argument("--out",  default="",
                   help="Markdown report path (default: <csv_dir>/quantify_<timestamp>.md)")
    p.add_argument("--plot", default="",
                   help="Output PNG path (default: same as --out with .png)")
    p.add_argument("--no-plot", action="store_true")
    p.add_argument("--label-from-name", action="store_true",
                   help="Use file basename as label (default: time_arm_mode suffix)")
    args = p.parse_args(argv)

    # Resolve input CSVs and the folder used for default outputs.
    if args.csv:
        csv_paths = list(args.csv)
        out_dir = os.path.dirname(os.path.abspath(csv_paths[0])) or "."
    else:
        csv_paths, out_dir = _interactive_pick(args.root)

    sessions: List[SessionMetrics] = []
    data: List[Dict[str, np.ndarray]] = []
    label_mode = "name" if args.label_from_name else "suffix"
    for path in csv_paths:
        if not os.path.isfile(path):
            print(f"  [skip] not a file: {path}")
            continue
        d = _load_csv(path)
        if "t" not in d or d["t"].size < 2:
            print(f"  [skip] no usable rows: {path}")
            continue
        label = _derive_label(path, label_mode)
        sessions.append(_compute(label, path, d))
        data.append(d)

    if not sessions:
        print("  no sessions parsed", file=sys.stderr)
        return 1

    # Resolve output paths.
    default_md, default_png = _default_out(out_dir)
    out_md  = args.out  or default_md
    out_png = args.plot or (out_md[:-3] + ".png" if out_md.endswith(".md") else default_png)

    report = _markdown_report(sessions)
    with open(out_md, "w") as fh:
        fh.write(report)
    print(f"  [report] wrote {out_md}")

    if not args.no_plot:
        _plot(sessions, data, out_png)

    return 0


if __name__ == "__main__":
    sys.exit(main())
