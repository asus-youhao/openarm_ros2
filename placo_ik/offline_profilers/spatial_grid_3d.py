#!/usr/bin/env python3
"""
spatial_grid_3d.py
==================
Full-picture spatial view that extends spatial_failure_map.py with:

  • 4×3 heatmap grid — 4 metrics (pos_err_p95, ori_err_p95, fail_rate, inv_sigma)
                        × 3 projections (XY, XZ, YZ)
  • 3-D scatter of the worst-N voxels, coloured by the primary metric

Use this when you want to compare different failure modes side-by-side at the
same xyz coords (e.g. is a region bad because of pos_err or sigma_min?).

Outputs (default: alongside the CSV in placo_ik/results/<date>/):
  spatial_grid_<ts>.png        ← 4×3 heatmap grid
  spatial_grid_<ts>_3d.png     ← 3-D scatter
  spatial_grid_<ts>.md         ← per-metric top-N tables

Usage:
  python3 spatial_grid_3d.py                   # interactive picker
  python3 spatial_grid_3d.py session.csv --voxel-size 0.05 --topn 15
"""
from __future__ import annotations

import argparse
import datetime as _dt
import os
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D   # noqa: F401  (registers 3d projection)
    HAVE_MPL = True
except Exception:
    HAVE_MPL = False

# Reuse sibling modules.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from quantify_teleop import _load_csv, _interactive_pick, RESULTS_ROOT       # noqa: E402
from spatial_failure_map import (                                              # noqa: E402
    METRICS, VoxelStat, _voxelize, _aggregate, _merge_sessions,
    _sort_by, _project,
)

# Pretty labels used in headings and axis titles.
METRIC_LABELS: Dict[str, str] = {
    "pos_err_p95": "pos_err p95 (mm)",
    "ori_err_p95": "ori_err p95 (deg)",
    "fail_rate":   "fail rate",
    "guard_rate":  "guard hit rate",
    "inv_sigma":   "1 / σ_min  (singularity proximity)",
}
GRID_METRICS = ("pos_err_p95", "ori_err_p95", "fail_rate", "inv_sigma")


def _plot_grid(voxels: List[VoxelStat], voxel_size: float,
               metrics: Tuple[str, ...], out_png: str) -> None:
    """4×3 heatmap grid (metric × projection)."""
    if not HAVE_MPL:
        print("  [plot] matplotlib unavailable — skip grid PNG")
        return

    rows = len(metrics)
    fig, axes = plt.subplots(rows, 3, figsize=(15, 4.2 * rows))
    if rows == 1:
        axes = axes.reshape(1, 3)
    cmap = plt.cm.YlOrRd.copy()
    cmap.set_bad("#e8e8e8")
    proj_specs = (
        ("z", "XY top-down (drop Z)"),
        ("y", "XZ side (drop Y)"),
        ("x", "YZ front (drop X)"),
    )
    for r, metric in enumerate(metrics):
        for c, (drop, ptitle) in enumerate(proj_specs):
            ax = axes[r, c]
            extent, grid, lbl1, lbl2 = _project(voxels, voxel_size, metric, drop)
            if grid is None:
                ax.text(0.5, 0.5, "no data", ha="center", va="center",
                        transform=ax.transAxes)
                ax.set_title(f"{METRIC_LABELS.get(metric, metric)} — {ptitle}",
                             fontsize=9)
                continue
            im = ax.imshow(np.ma.masked_invalid(grid), origin="lower",
                           extent=extent, cmap=cmap, aspect="equal",
                           interpolation="nearest")
            plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
            ax.set_xlabel(f"{lbl1} (m)", fontsize=8)
            ax.set_ylabel(f"{lbl2} (m)", fontsize=8)
            ax.set_title(f"{METRIC_LABELS.get(metric, metric)} — {ptitle}",
                         fontsize=9)
            ax.tick_params(labelsize=7)
            ax.grid(True, alpha=0.2, linewidth=0.4)

    fig.suptitle(
        f"Spatial multi-metric grid · voxel={voxel_size*100:.0f} cm "
        f"· {len(voxels)} voxels", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
    print(f"  [plot] wrote {out_png}")


def _plot_3d(voxels: List[VoxelStat], primary_metric: str,
             topn: int, out_png: str) -> None:
    """3-D scatter of all voxels (small dots) + top-N highlights."""
    if not HAVE_MPL:
        print("  [plot] matplotlib unavailable — skip 3d PNG")
        return
    field_name, direction = METRICS[primary_metric]

    xs, ys, zs, vals = [], [], [], []
    for v in voxels:
        val = getattr(v, field_name)
        if not np.isfinite(val):
            continue
        plotted = (1.0 / max(val, 1e-3)) if direction == "lower" else val
        xs.append(v.cx); ys.append(v.cy); zs.append(v.cz); vals.append(plotted)

    if not vals:
        print("  [plot] no finite voxels — skip 3d PNG")
        return

    xs = np.asarray(xs); ys = np.asarray(ys); zs = np.asarray(zs)
    vals = np.asarray(vals)
    ranked = _sort_by(voxels, primary_metric)[:topn]

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    sc = ax.scatter(xs, ys, zs, c=vals, cmap="YlOrRd", s=24, alpha=0.85,
                    edgecolors="none")
    cb = plt.colorbar(sc, ax=ax, fraction=0.04, pad=0.08)
    cb.set_label(METRIC_LABELS.get(primary_metric, primary_metric))

    # Annotate top-N with rank number.
    for i, v in enumerate(ranked, 1):
        ax.text(v.cx, v.cy, v.cz, f" {i}", fontsize=8,
                color="black", weight="bold")
        ax.scatter([v.cx], [v.cy], [v.cz], facecolors="none",
                   edgecolors="black", s=140, linewidths=1.4)

    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.set_title(
        f"3-D voxels (top-{topn} circled) · metric = {primary_metric}",
        fontsize=11,
    )
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    print(f"  [plot] wrote {out_png}")


def _markdown(voxels: List[VoxelStat], topn: int, voxel_size: float,
              primary_metric: str, sessions: List[str]) -> str:
    """Same shape as spatial_failure_map but lists top-N per metric in full."""
    out: List[str] = []
    out.append("# Spatial Grid Report (multi-metric)\n")
    out.append(
        f"_Voxel: **{voxel_size*100:.1f} cm** · "
        f"Primary metric: **`{primary_metric}`** · "
        f"Sessions: {len(sessions)} · Qualifying voxels: {len(voxels)}_\n"
    )
    for m in GRID_METRICS + ("guard_rate",):
        field_name, _ = METRICS[m]
        out.append(f"## Top-{topn} by `{m}`  ({METRIC_LABELS.get(m, m)})\n")
        out.append("| # | center (x, y, z) | n | dwell s | "
                   "pos50 | pos95 | ori95 | σ_min | fail% | guard% |")
        out.append("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|")
        for i, v in enumerate(_sort_by(voxels, m)[:topn], 1):
            out.append(
                f"| {i} | ({v.cx:+.2f}, {v.cy:+.2f}, {v.cz:+.2f}) | {v.n} "
                f"| {v.dwell_sec:.1f} | {v.pos_err_p50:.2f} | {v.pos_err_p95:.2f} "
                f"| {v.ori_err_p95:.2f} | {v.sigma_min:.3f} "
                f"| {v.fail_rate*100:.1f} | {v.guard_rate*100:.1f} |"
            )
        out.append("")
    out.append("## Files\n")
    for s in sessions:
        out.append(f"- {s}")
    out.append("")
    return "\n".join(out)


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("csv", nargs="*",
                   help="CSV file(s). Omit → interactive picker.")
    p.add_argument("--root", default=RESULTS_ROOT)
    p.add_argument("--voxel-size", type=float, default=0.05)
    p.add_argument("--metric", default="pos_err_p95", choices=list(METRICS.keys()),
                   help="Primary metric for the 3-D scatter colouring + top-N circling")
    p.add_argument("--topn", type=int, default=10)
    p.add_argument("--min-count", type=int, default=5)
    p.add_argument("--out", default="")
    p.add_argument("--no-plot", action="store_true")
    args = p.parse_args(argv)

    csv_paths, out_dir = (
        (list(args.csv), os.path.dirname(os.path.abspath(args.csv[0])) or ".")
        if args.csv else _interactive_pick(args.root)
    )

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
            continue
        d_v = {k: v[valid] if (hasattr(v, "size") and v.size == valid.size) else v
               for k, v in d.items()}
        per_session.append(_aggregate(
            d_v, _voxelize(xyz[valid], args.voxel_size),
            args.voxel_size, os.path.basename(path).replace(".csv", "")))
        labels.append(os.path.basename(path).replace(".csv", ""))

    if not per_session:
        print("  no usable sessions", file=sys.stderr)
        return 1

    merged = _merge_sessions(per_session)
    qualifying = [v for v in merged if v.n >= args.min_count]
    print(f"  {sum(len(s) for s in per_session)} bins → {len(merged)} unique "
          f"→ {len(qualifying)} with n≥{args.min_count}")
    if not qualifying:
        return 1

    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_md   = args.out or os.path.join(out_dir, f"spatial_grid_{stamp}.md")
    out_grid = out_md[:-3] + ".png"          if out_md.endswith(".md") \
               else os.path.join(out_dir, f"spatial_grid_{stamp}.png")
    out_3d   = out_grid.replace(".png", "_3d.png")

    with open(out_md, "w") as fh:
        fh.write(_markdown(qualifying, args.topn, args.voxel_size,
                           args.metric, labels))
    print(f"  [report] wrote {out_md}")

    if not args.no_plot:
        _plot_grid(qualifying, args.voxel_size, GRID_METRICS, out_grid)
        _plot_3d(qualifying, args.metric, args.topn, out_3d)
    return 0


if __name__ == "__main__":
    sys.exit(main())
