#!/usr/bin/env python3
"""
spatial_failure_map.py
======================
Voxelize the EE workspace and locate *where in xyz* the IK is unhappy.

Sister tool to quantify_teleop.py:
  quantify_teleop      → time-aggregate health (one number per session)
  spatial_failure_map  → per-voxel distribution (one number per cubic cell)

Outputs:
  • Markdown report — top-N worst voxels + per-metric leaderboard
  • PNG (1×3)       — XY top, XZ side, YZ front max-projection heatmaps

Reuses the interactive folder picker from quantify_teleop.py.

Usage:
  # Interactive (recommended)
  python3 spatial_failure_map.py

  # Direct
  python3 spatial_failure_map.py session.csv --voxel-size 0.05 --metric pos_err_p95
"""
from __future__ import annotations

import argparse
import datetime as _dt
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

# Reuse loader + picker + default root from sibling script.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from quantify_teleop import _load_csv, _interactive_pick, RESULTS_ROOT   # noqa: E402


# ── Voxel aggregation ─────────────────────────────────────────────────────────
@dataclass
class VoxelStat:
    ix: int; iy: int; iz: int          # grid index
    cx: float; cy: float; cz: float    # centre xyz (m)
    n: int
    dwell_sec: float
    pos_err_p50: float
    pos_err_p95: float
    ori_err_p95: float
    sigma_min:   float                  # min across rows in this voxel
    fail_rate:   float
    guard_rate:  float
    sessions:    Tuple[str, ...] = field(default_factory=tuple)


# CLI metric key → (VoxelStat field, "lower" / "higher" = worse)
METRICS: Dict[str, Tuple[str, str]] = {
    "pos_err_p95": ("pos_err_p95", "higher"),
    "pos_err_p50": ("pos_err_p50", "higher"),
    "ori_err_p95": ("ori_err_p95", "higher"),
    "fail_rate":   ("fail_rate",   "higher"),
    "guard_rate":  ("guard_rate",  "higher"),
    "inv_sigma":   ("sigma_min",   "lower"),    # singularity proximity
}


def _safe_p(arr: np.ndarray, p: float) -> float:
    arr = arr[np.isfinite(arr)]
    return float(np.percentile(arr, p)) if arr.size else float("nan")


def _voxelize(xyz: np.ndarray, voxel_size: float
              ) -> Dict[Tuple[int, int, int], np.ndarray]:
    """xyz shape (N, 3). Returns dict (ix,iy,iz) → row indices."""
    keys = np.floor(xyz / voxel_size).astype(int)
    buckets: Dict[Tuple[int, int, int], List[int]] = {}
    for i, (ix, iy, iz) in enumerate(keys):
        buckets.setdefault((int(ix), int(iy), int(iz)), []).append(i)
    return {k: np.asarray(v, dtype=int) for k, v in buckets.items()}


def _aggregate(d: Dict[str, np.ndarray], buckets, voxel_size: float,
               label: str) -> List[VoxelStat]:
    t       = d["t"]
    pos_err = d.get("pos_err_mm", np.zeros(t.size))
    ori_err = d.get("ori_err_deg", np.zeros(t.size))
    sig     = d.get("sigma_min", np.full(t.size, np.nan))
    succ    = d.get("success", np.ones(t.size))
    guard   = d.get("joint_jump_guard", np.zeros(t.size))
    # Per-row dwell ≈ time since previous row, capped to ignore long pauses
    if t.size > 1:
        dt = np.diff(t, prepend=t[0])
        dt = np.clip(dt, 0.0, 0.1)   # cap at 100 ms (skip idle gaps)
    else:
        dt = np.zeros_like(t)

    out: List[VoxelStat] = []
    for (ix, iy, iz), idx in buckets.items():
        cx = (ix + 0.5) * voxel_size
        cy = (iy + 0.5) * voxel_size
        cz = (iz + 0.5) * voxel_size
        sig_v = sig[idx]
        sig_v = sig_v[np.isfinite(sig_v)]
        out.append(VoxelStat(
            ix=ix, iy=iy, iz=iz, cx=cx, cy=cy, cz=cz,
            n=int(idx.size),
            dwell_sec   = float(dt[idx].sum()),
            pos_err_p50 = _safe_p(pos_err[idx], 50),
            pos_err_p95 = _safe_p(pos_err[idx], 95),
            ori_err_p95 = _safe_p(ori_err[idx], 95),
            sigma_min   = float(sig_v.min()) if sig_v.size else float("nan"),
            fail_rate   = 1.0 - float(np.nanmean(succ[idx])),
            guard_rate  = float(np.nanmean(guard[idx])),
            sessions    = (label,),
        ))
    return out


def _merge_sessions(per_session: List[List[VoxelStat]]) -> List[VoxelStat]:
    """Combine voxels with the same (ix,iy,iz) across sessions (n-weighted)."""
    by_key: Dict[Tuple[int, int, int], List[VoxelStat]] = {}
    for sess in per_session:
        for v in sess:
            by_key.setdefault((v.ix, v.iy, v.iz), []).append(v)
    merged: List[VoxelStat] = []
    for key, lst in by_key.items():
        wts = np.array([v.n for v in lst], dtype=float)
        n_total = int(wts.sum())
        sigs = [v.sigma_min for v in lst if np.isfinite(v.sigma_min)]
        merged.append(VoxelStat(
            ix=key[0], iy=key[1], iz=key[2],
            cx=lst[0].cx, cy=lst[0].cy, cz=lst[0].cz,
            n=n_total,
            dwell_sec   = float(sum(v.dwell_sec for v in lst)),
            pos_err_p50 = float(np.average([v.pos_err_p50 for v in lst], weights=wts)),
            pos_err_p95 = float(np.average([v.pos_err_p95 for v in lst], weights=wts)),
            ori_err_p95 = float(np.average([v.ori_err_p95 for v in lst], weights=wts)),
            sigma_min   = float(min(sigs)) if sigs else float("nan"),
            fail_rate   = float(np.average([v.fail_rate for v in lst], weights=wts)),
            guard_rate  = float(np.average([v.guard_rate for v in lst], weights=wts)),
            sessions    = tuple(sorted({s for v in lst for s in v.sessions})),
        ))
    return merged


def _sort_by(voxels: List[VoxelStat], metric: str) -> List[VoxelStat]:
    field_name, direction = METRICS[metric]
    if direction == "lower":
        # worst = smallest value (e.g. sigma_min); NaN → push to end
        return sorted(voxels, key=lambda v: (
            getattr(v, field_name) if np.isfinite(getattr(v, field_name)) else float("inf")
        ))
    return sorted(voxels, key=lambda v: -(
        getattr(v, field_name) if np.isfinite(getattr(v, field_name)) else -float("inf")
    ))


# ── Markdown report ───────────────────────────────────────────────────────────
def _markdown(voxels: List[VoxelStat], topn: int, voxel_size: float,
              primary_metric: str, sessions: List[str]) -> str:
    lines: List[str] = []
    lines.append("# Spatial Failure Map\n")
    lines.append(
        f"_Voxel size: **{voxel_size * 100:.1f} cm** · "
        f"Primary metric: **`{primary_metric}`** · "
        f"Sessions: {len(sessions)} · Qualifying voxels: {len(voxels)}_\n"
    )

    ranked = _sort_by(voxels, primary_metric)
    lines.append(f"## Top-{topn} worst voxels (by `{primary_metric}`)\n")
    lines.append(
        "| # | center (x, y, z) m | n | dwell s | pos50 mm | "
        "pos95 mm | ori95° | σ_min | fail% | guard% | sessions |"
    )
    lines.append("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|")
    for i, v in enumerate(ranked[:topn], 1):
        sess_str = ", ".join(s[:18] + "…" if len(s) > 20 else s for s in v.sessions)
        lines.append(
            f"| {i} | ({v.cx:+.2f}, {v.cy:+.2f}, {v.cz:+.2f}) | {v.n} "
            f"| {v.dwell_sec:.1f} | {v.pos_err_p50:.2f} | {v.pos_err_p95:.2f} "
            f"| {v.ori_err_p95:.2f} | {v.sigma_min:.3f} "
            f"| {v.fail_rate * 100:.1f} | {v.guard_rate * 100:.1f} | {sess_str} |"
        )
    lines.append("")

    # Per-metric top-3 leaderboard
    lines.append("## Per-metric top-3 (independent rankings)\n")
    for m in ("pos_err_p95", "ori_err_p95", "fail_rate", "guard_rate", "inv_sigma"):
        field_name, _ = METRICS[m]
        lines.append(f"**`{m}`** ({'min' if m == 'inv_sigma' else 'max'} of `{field_name}`):")
        for i, v in enumerate(_sort_by(voxels, m)[:3], 1):
            val = getattr(v, field_name)
            lines.append(
                f"  {i}. xyz=({v.cx:+.2f}, {v.cy:+.2f}, {v.cz:+.2f}) "
                f"→ {field_name} = {val:.3f}  (n={v.n}, dwell={v.dwell_sec:.1f}s)"
            )
        lines.append("")

    lines.append("## Files\n")
    for s in sessions:
        lines.append(f"- {s}")
    lines.append("")
    return "\n".join(lines)


# ── Plot ──────────────────────────────────────────────────────────────────────
def _project(voxels: List[VoxelStat], voxel_size: float, metric: str,
             drop_axis: str
             ) -> Tuple[Optional[Tuple[float, float, float, float]],
                        Optional[np.ndarray], str, str]:
    """Max-project the metric onto a 2D plane by dropping one axis."""
    field_name, direction = METRICS[metric]
    drop_to = {
        "x": ((1, 2), "y", "z"),
        "y": ((0, 2), "x", "z"),
        "z": ((0, 1), "x", "y"),
    }
    (a, b), lbl1, lbl2 = drop_to[drop_axis]

    pts: Dict[Tuple[int, int], float] = {}
    for v in voxels:
        idx = (v.ix, v.iy, v.iz)
        i, j = idx[a], idx[b]
        val = getattr(v, field_name)
        if not np.isfinite(val):
            continue
        # For "lower is worse" we invert so higher pixel = worse.
        plotted = (1.0 / max(val, 1e-3)) if direction == "lower" else val
        key = (i, j)
        if key not in pts or plotted > pts[key]:
            pts[key] = plotted

    if not pts:
        return None, None, lbl1, lbl2

    keys = np.array(list(pts.keys()))
    i_lo, j_lo = keys.min(axis=0)
    i_hi, j_hi = keys.max(axis=0)
    grid = np.full((j_hi - j_lo + 1, i_hi - i_lo + 1), np.nan)
    for (i, j), val in pts.items():
        grid[j - j_lo, i - i_lo] = val
    extent = (i_lo * voxel_size, (i_hi + 1) * voxel_size,
              j_lo * voxel_size, (j_hi + 1) * voxel_size)
    return extent, grid, lbl1, lbl2


def _plot(voxels: List[VoxelStat], voxel_size: float, metric: str,
          out_png: str) -> None:
    if not HAVE_MPL:
        print("  [plot] matplotlib unavailable — skipping PNG")
        return

    fig, axes = plt.subplots(1, 3, figsize=(16, 5.2))
    cmap = plt.cm.YlOrRd.copy()
    cmap.set_bad("#e8e8e8")
    titles = [
        "XY (top-down, drop Z = max over height)",
        "XZ (side, drop Y = max over depth)",
        "YZ (front, drop X = max over reach)",
    ]
    for ax, drop, title in zip(axes, ("z", "y", "x"), titles):
        extent, grid, lbl1, lbl2 = _project(voxels, voxel_size, metric, drop)
        if grid is None:
            ax.text(0.5, 0.5, "no data", ha="center", va="center",
                    transform=ax.transAxes)
            ax.set_title(title, fontsize=10)
            continue
        im = ax.imshow(np.ma.masked_invalid(grid), origin="lower", extent=extent,
                       cmap=cmap, aspect="equal", interpolation="nearest")
        cb_label = ("1 / σ_min" if metric == "inv_sigma" else metric)
        plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label=cb_label)
        ax.set_xlabel(f"{lbl1} (m)")
        ax.set_ylabel(f"{lbl2} (m)")
        ax.set_title(title, fontsize=10)
        ax.grid(True, alpha=0.25, linewidth=0.5)

    fig.suptitle(
        f"Spatial failure map · metric={metric} · "
        f"voxel={voxel_size * 100:.0f} cm · {len(voxels)} voxels",
        fontsize=12,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    print(f"  [plot] wrote {out_png}")


# ── Main ──────────────────────────────────────────────────────────────────────
def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("csv", nargs="*",
                   help="CSV file(s). Omit → interactive folder/file picker.")
    p.add_argument("--root", default=RESULTS_ROOT)
    p.add_argument("--voxel-size", type=float, default=0.05,
                   help="Voxel edge length in metres (default 0.05 = 5cm)")
    p.add_argument("--metric", default="pos_err_p95",
                   choices=list(METRICS.keys()),
                   help="Drives ranking AND heatmap colour (default pos_err_p95)")
    p.add_argument("--topn", type=int, default=10,
                   help="Worst-N voxels shown in the MD table (default 10)")
    p.add_argument("--min-count", type=int, default=5,
                   help="Skip voxels with fewer rows than this (default 5)")
    p.add_argument("--out", default="",
                   help="Markdown output (default: <csv_dir>/spatial_<ts>.md)")
    p.add_argument("--plot", default="",
                   help="PNG output (default: same as --out, .png)")
    p.add_argument("--no-plot", action="store_true")
    args = p.parse_args(argv)

    # Inputs.
    if args.csv:
        csv_paths = list(args.csv)
        out_dir = os.path.dirname(os.path.abspath(csv_paths[0])) or "."
    else:
        csv_paths, out_dir = _interactive_pick(args.root)

    per_session: List[List[VoxelStat]] = []
    labels: List[str] = []
    for path in csv_paths:
        if not os.path.isfile(path):
            print(f"  [skip] not a file: {path}")
            continue
        d = _load_csv(path)
        if not all(k in d for k in ("t", "x", "y", "z")):
            print(f"  [skip] missing t/x/y/z: {path}")
            continue
        xyz = np.stack([d["x"], d["y"], d["z"]], axis=1)
        valid = np.isfinite(xyz).all(axis=1)
        if int(valid.sum()) < args.min_count:
            print(f"  [skip] too few valid rows: {path}")
            continue
        # Slice every column to the valid rows so per-voxel indexing lines up.
        d_v = {k: v[valid] if (hasattr(v, "size") and v.size == valid.size) else v
               for k, v in d.items()}
        buckets = _voxelize(xyz[valid], args.voxel_size)
        label = os.path.basename(path).replace(".csv", "")
        per_session.append(_aggregate(d_v, buckets, args.voxel_size, label))
        labels.append(label)

    if not per_session:
        print("  no usable sessions", file=sys.stderr)
        return 1

    merged = _merge_sessions(per_session)
    qualifying = [v for v in merged if v.n >= args.min_count]
    print(f"  voxels: {sum(len(s) for s in per_session)} session-bins → "
          f"{len(merged)} unique → {len(qualifying)} with n≥{args.min_count}")
    if not qualifying:
        print(f"  no voxels with n >= {args.min_count}", file=sys.stderr)
        return 1

    # Resolve output paths.
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_md  = args.out  or os.path.join(out_dir, f"spatial_{stamp}.md")
    out_png = args.plot or (
        out_md[:-3] + ".png" if out_md.endswith(".md")
        else os.path.join(out_dir, f"spatial_{stamp}.png")
    )

    report = _markdown(qualifying, args.topn, args.voxel_size,
                       args.metric, labels)
    with open(out_md, "w") as fh:
        fh.write(report)
    print(f"  [report] wrote {out_md}")

    if not args.no_plot:
        _plot(qualifying, args.voxel_size, args.metric, out_png)
    return 0


if __name__ == "__main__":
    sys.exit(main())
