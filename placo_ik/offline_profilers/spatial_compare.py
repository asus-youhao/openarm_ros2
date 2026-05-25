#!/usr/bin/env python3
"""
spatial_compare.py
==================
Two complementary "why is this voxel bad?" analyses, selectable with --mode.

  --mode ori-slice  (default for single CSV)
      Pick worst-N voxels by --metric, then for each one scatter the EE
      orientation (rpy decoded from tf_qx..tf_qw) coloured by per-row error.
      Answers: "at this bad xyz coord, is one orientation in particular bad,
      or is the whole region uniformly bad?"

  --mode lr-diff    (default when both a *_left_* and *_right_* CSV are given)
      Aggregate the two CSVs separately, build the set of voxels touched by
      BOTH arms, then plot left_metric − right_metric per voxel with a
      diverging colormap. Answers: "do the two arms struggle in the same
      places, or are they asymmetric?"

Outputs (in placo_ik/results/<date>/ by default):
  spatial_<mode>_<ts>.md       — voxel-level findings
  spatial_<mode>_<ts>.png      — scatter (ori-slice) or 3-panel diff heatmap (lr-diff)

Usage:
  # ori-slice on the right-arm CSV — focus on top-3 worst voxels for pos_err
  python3 spatial_compare.py right.csv --mode ori-slice --topn 3

  # lr-diff comparing two arm CSVs
  python3 spatial_compare.py left.csv right.csv --mode lr-diff
"""
from __future__ import annotations

import argparse
import datetime as _dt
import math
import os
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except Exception:
    HAVE_MPL = False

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from quantify_teleop import _load_csv, _interactive_pick, RESULTS_ROOT         # noqa: E402
from spatial_failure_map import (                                              # noqa: E402
    METRICS, VoxelStat, _voxelize, _aggregate, _merge_sessions, _sort_by,
)


# ── Quaternion → RPY (ZYX intrinsic) ──────────────────────────────────────────
def _quat_to_rpy_deg(qx: np.ndarray, qy: np.ndarray, qz: np.ndarray,
                     qw: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0)
    pitch = np.arcsin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


# ── ori-slice mode ────────────────────────────────────────────────────────────
def _ori_slice_run(d: Dict[str, np.ndarray], voxel_size: float, metric: str,
                   topn: int, min_count: int, out_md: str, out_png: str) -> None:
    has_tf = all(k in d for k in ("tf_qx", "tf_qy", "tf_qz", "tf_qw"))
    if not has_tf:
        print("  [ori-slice] CSV lacks tf_q* columns — cannot decode orientation")
        return

    xyz = np.stack([d["x"], d["y"], d["z"]], axis=1)
    valid = np.isfinite(xyz).all(axis=1)
    d_v = {k: v[valid] if (hasattr(v, "size") and v.size == valid.size) else v
           for k, v in d.items()}
    buckets = _voxelize(xyz[valid], voxel_size)
    voxels = _aggregate(d_v, buckets, voxel_size, "session")
    qualifying = [v for v in voxels if v.n >= min_count]
    if not qualifying:
        print(f"  [ori-slice] no voxels with n >= {min_count}")
        return
    worst = _sort_by(qualifying, metric)[:topn]

    roll, pitch, yaw = _quat_to_rpy_deg(
        d_v["tf_qx"], d_v["tf_qy"], d_v["tf_qz"], d_v["tf_qw"])
    err = d_v.get("pos_err_mm", np.zeros(d_v["t"].size))

    # ── MD ────────────────────────────────────────────────────────────
    lines: List[str] = []
    lines.append("# Orientation Slice Report\n")
    lines.append(
        f"_Voxel: **{voxel_size*100:.1f} cm** · Metric: **`{metric}`** · "
        f"Showing top {len(worst)} worst voxels (decoded rpy from `tf_q*`)._\n"
    )
    for rank, v in enumerate(worst, 1):
        rows = buckets[(v.ix, v.iy, v.iz)]
        r_arr = roll[rows]; p_arr = pitch[rows]; y_arr = yaw[rows]
        e_arr = err[rows]
        lines.append(
            f"## #{rank} voxel ({v.cx:+.2f}, {v.cy:+.2f}, {v.cz:+.2f}) · "
            f"n={v.n} dwell={v.dwell_sec:.1f}s\n"
        )
        lines.append(
            f"- `pos_err_p50/p95` = {v.pos_err_p50:.2f} / {v.pos_err_p95:.2f} mm  "
            f"`ori_err_p95` = {v.ori_err_p95:.2f}°  `σ_min` = {v.sigma_min:.3f}\n"
        )
        lines.append("- rpy distribution (deg, decoded from measured EE):")
        for nm, a in (("roll", r_arr), ("pitch", p_arr), ("yaw", y_arr)):
            lines.append(
                f"  - **{nm}**: median={np.median(a):+.1f} · "
                f"p05={np.percentile(a,5):+.1f} · p95={np.percentile(a,95):+.1f} · "
                f"span={a.max()-a.min():.1f}"
            )
        # Correlation: which axis correlates with err best?
        corrs = {}
        for nm, a in (("roll", r_arr), ("pitch", p_arr), ("yaw", y_arr)):
            if a.std() > 1e-3 and e_arr.std() > 1e-3:
                corrs[nm] = float(np.corrcoef(a, e_arr)[0, 1])
            else:
                corrs[nm] = float("nan")
        worst_axis = max(corrs, key=lambda k: abs(corrs[k]) if np.isfinite(corrs[k]) else -1)
        lines.append(
            f"- Correlation rpy ↔ pos_err: "
            f"roll={corrs['roll']:+.2f}, pitch={corrs['pitch']:+.2f}, "
            f"yaw={corrs['yaw']:+.2f} → **{worst_axis}** dominates\n"
        )
    lines.append("")
    with open(out_md, "w") as fh:
        fh.write("\n".join(lines))
    print(f"  [report] wrote {out_md}")

    # ── PNG: 3 rows × topn cols, each col = one voxel, row = roll/pitch/yaw scatter ──
    if not HAVE_MPL:
        return
    cols = len(worst)
    fig, axes = plt.subplots(3, cols, figsize=(4.5 * cols, 11), squeeze=False)
    pairs = (("roll", "pitch"), ("roll", "yaw"), ("pitch", "yaw"))
    axis_data = {"roll": roll, "pitch": pitch, "yaw": yaw}
    for c, v in enumerate(worst):
        rows = buckets[(v.ix, v.iy, v.iz)]
        c_err = err[rows]
        for r, (a1, a2) in enumerate(pairs):
            ax = axes[r, c]
            sc = ax.scatter(axis_data[a1][rows], axis_data[a2][rows],
                            c=c_err, cmap="YlOrRd", s=16,
                            edgecolors="none", alpha=0.85)
            ax.set_xlabel(f"{a1} (deg)", fontsize=8)
            ax.set_ylabel(f"{a2} (deg)", fontsize=8)
            ax.grid(True, alpha=0.25, linewidth=0.4)
            ax.tick_params(labelsize=7)
            if r == 0:
                ax.set_title(
                    f"#{c+1}: ({v.cx:+.2f},{v.cy:+.2f},{v.cz:+.2f})  n={v.n}",
                    fontsize=9,
                )
            if c == cols - 1:
                plt.colorbar(sc, ax=ax, fraction=0.046, pad=0.04,
                             label="pos_err (mm)")
    fig.suptitle(
        f"Orientation slice at top-{cols} worst voxels  (metric={metric})",
        fontsize=12,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
    print(f"  [plot] wrote {out_png}")


# ── lr-diff mode ──────────────────────────────────────────────────────────────
def _aggregate_csv(path: str, voxel_size: float, label: str,
                   mirror_y: bool = False
                   ) -> Tuple[Dict[Tuple[int,int,int], VoxelStat], int]:
    """Mirror_y flips the y axis (useful to compare bimanual arms in arm-local frame)."""
    d = _load_csv(path)
    if not all(k in d for k in ("t", "x", "y", "z")):
        return {}, 0
    y = -d["y"] if mirror_y else d["y"]
    xyz = np.stack([d["x"], y, d["z"]], axis=1)
    valid = np.isfinite(xyz).all(axis=1)
    d_v = {k: v[valid] if (hasattr(v, "size") and v.size == valid.size) else v
           for k, v in d.items()}
    voxels = _aggregate(d_v, _voxelize(xyz[valid], voxel_size), voxel_size, label)
    # Restamp centres so they read out correctly in the (possibly mirrored) frame.
    for v in voxels:
        v.cx = (v.ix + 0.5) * voxel_size
        v.cy = (v.iy + 0.5) * voxel_size
        v.cz = (v.iz + 0.5) * voxel_size
    return {(v.ix, v.iy, v.iz): v for v in voxels}, int(valid.sum())


def _lr_diff_project(diff: Dict[Tuple[int,int,int], float], voxel_size: float,
                     drop: str) -> Tuple[Optional[Tuple[float,float,float,float]],
                                          Optional[np.ndarray], str, str]:
    drop_map = {"x": ((1,2), "y", "z"), "y": ((0,2), "x", "z"),
                "z": ((0,1), "x", "y")}
    (a, b), lbl1, lbl2 = drop_map[drop]
    pts: Dict[Tuple[int,int], List[float]] = {}
    for (ix, iy, iz), v in diff.items():
        i, j = (ix, iy, iz)[a], (ix, iy, iz)[b]
        pts.setdefault((i, j), []).append(v)
    if not pts:
        return None, None, lbl1, lbl2
    # Mean diff over the dropped axis at each (i,j).
    flat = {k: float(np.mean(vs)) for k, vs in pts.items()}
    keys = np.array(list(flat.keys()))
    i_lo, j_lo = keys.min(axis=0)
    i_hi, j_hi = keys.max(axis=0)
    grid = np.full((j_hi - j_lo + 1, i_hi - i_lo + 1), np.nan)
    for (i, j), val in flat.items():
        grid[j - j_lo, i - i_lo] = val
    extent = (i_lo * voxel_size, (i_hi + 1) * voxel_size,
              j_lo * voxel_size, (j_hi + 1) * voxel_size)
    return extent, grid, lbl1, lbl2


def _lr_diff_run(left_path: str, right_path: str, voxel_size: float,
                 metric: str, min_count: int, topn: int,
                 out_md: str, out_png: str, mirror_left_y: bool = False) -> None:
    field_name, _ = METRICS[metric]
    left_vox,  n_l = _aggregate_csv(left_path,  voxel_size, "left",
                                    mirror_y=mirror_left_y)
    right_vox, n_r = _aggregate_csv(right_path, voxel_size, "right")
    if not left_vox or not right_vox:
        print("  [lr-diff] unable to aggregate one of the CSVs")
        return

    common_keys = [k for k in left_vox.keys() if k in right_vox]
    common_keys = [k for k in common_keys
                   if left_vox[k].n >= min_count and right_vox[k].n >= min_count]
    if not common_keys:
        print(f"  [lr-diff] no voxels touched by both arms with n >= {min_count}")
        return

    # diff = left − right  (positive = left worse).
    diff: Dict[Tuple[int,int,int], float] = {}
    rows_md: List[Tuple[Tuple[int,int,int], float, float, float]] = []
    for k in common_keys:
        lv, rv = left_vox[k], right_vox[k]
        lvl, rvl = getattr(lv, field_name), getattr(rv, field_name)
        if not (np.isfinite(lvl) and np.isfinite(rvl)):
            continue
        # For "lower-is-worse" metrics flip the sign so positive still = left worse.
        if metric == "inv_sigma":
            d_val = rvl - lvl    # smaller sigma_min = worse → left has smaller → diff positive
        else:
            d_val = lvl - rvl
        diff[k] = d_val
        rows_md.append((k, d_val, lvl, rvl))

    # ── MD ────────────────────────────────────────────────────────────
    rows_md.sort(key=lambda r: -abs(r[1]))
    lines: List[str] = []
    lines.append("# Left vs Right Spatial Diff Report\n")
    lines.append(
        f"_Voxel: **{voxel_size*100:.1f} cm** · Metric: **`{metric}`** ({field_name}) · "
        f"Common voxels with n≥{min_count} on both arms: {len(diff)}_\n"
    )
    lines.append("**Positive diff ⇒ LEFT is worse · Negative diff ⇒ RIGHT is worse**\n")
    lines.append(f"## Top-{topn} voxels by |left − right|\n")
    lines.append("| # | center (x, y, z) | diff | left | right | worse |")
    lines.append("|---|---|---:|---:|---:|---|")
    for i, (k, d_val, lvl, rvl) in enumerate(rows_md[:topn], 1):
        lv = left_vox[k]
        worse = "LEFT" if d_val > 0 else "RIGHT"
        lines.append(
            f"| {i} | ({lv.cx:+.2f}, {lv.cy:+.2f}, {lv.cz:+.2f}) "
            f"| {d_val:+.3f} | {lvl:.3f} | {rvl:.3f} | {worse} |"
        )
    lines.append("")
    # Summary stats.
    diffs = np.array([r[1] for r in rows_md])
    lines.append("## Summary\n")
    lines.append(f"- mean diff = {diffs.mean():+.3f} "
                 f"({'left worse on avg' if diffs.mean()>0 else 'right worse on avg'})")
    lines.append(f"- median diff = {np.median(diffs):+.3f}")
    lines.append(f"- |diff| p95 = {np.percentile(np.abs(diffs),95):.3f}")
    lines.append(f"- voxels where LEFT worse: "
                 f"{int(np.sum(diffs>0))} / {diffs.size}")
    lines.append(f"- voxels where RIGHT worse: "
                 f"{int(np.sum(diffs<0))} / {diffs.size}\n")
    lines.append("## Files\n")
    lines.append(f"- left:  {left_path}")
    lines.append(f"- right: {right_path}")
    with open(out_md, "w") as fh:
        fh.write("\n".join(lines))
    print(f"  [report] wrote {out_md}")

    # ── PNG: 3 projections of the diff map (diverging colormap) ──────
    if not HAVE_MPL:
        return
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.2))
    vmax = max(abs(diffs.min()), abs(diffs.max())) or 1.0
    cmap = plt.cm.RdBu_r.copy()      # red = left worse · blue = right worse
    cmap.set_bad("#e8e8e8")
    proj_titles = (
        ("z", "XY top-down"),
        ("y", "XZ side"),
        ("x", "YZ front"),
    )
    for ax, (drop, title) in zip(axes, proj_titles):
        extent, grid, lbl1, lbl2 = _lr_diff_project(diff, voxel_size, drop)
        if grid is None:
            ax.text(0.5, 0.5, "no data", ha="center", va="center",
                    transform=ax.transAxes)
            ax.set_title(title)
            continue
        im = ax.imshow(np.ma.masked_invalid(grid), origin="lower", extent=extent,
                       cmap=cmap, vmin=-vmax, vmax=vmax, aspect="equal",
                       interpolation="nearest")
        plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04,
                     label=f"left − right ({field_name})")
        ax.set_xlabel(f"{lbl1} (m)"); ax.set_ylabel(f"{lbl2} (m)")
        ax.set_title(title)
        ax.grid(True, alpha=0.25, linewidth=0.5)
    fig.suptitle(
        f"L − R spatial diff · metric={metric} · {len(diff)} common voxels "
        f"(red = LEFT worse, blue = RIGHT worse)", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    print(f"  [plot] wrote {out_png}")


# ── Entry point ──────────────────────────────────────────────────────────────
def _autodetect_mode(csv_paths: List[str]) -> str:
    """If two CSVs and one looks left + the other right → lr-diff. Else ori-slice."""
    if len(csv_paths) == 2:
        l, r = csv_paths
        if ("_left" in os.path.basename(l) and "_right" in os.path.basename(r)) \
           or ("_right" in os.path.basename(l) and "_left" in os.path.basename(r)):
            return "lr-diff"
    return "ori-slice"


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("csv", nargs="*", help="CSV file(s); see --mode for shape rules")
    p.add_argument("--root", default=RESULTS_ROOT)
    p.add_argument("--mode", choices=("auto", "ori-slice", "lr-diff"), default="auto",
                   help="auto: pick lr-diff if 2 CSVs look like left+right, else ori-slice")
    p.add_argument("--voxel-size", type=float, default=0.05)
    p.add_argument("--metric", default="pos_err_p95", choices=list(METRICS.keys()))
    p.add_argument("--topn", type=int, default=5)
    p.add_argument("--min-count", type=int, default=5)
    p.add_argument("--out", default="")
    p.add_argument("--mirror-left-y", action="store_true", dest="mirror_left_y",
                   help="lr-diff only: flip left arm's y so bimanual arms compare "
                        "in arm-local (mirrored) frame. Use when each arm normally "
                        "reaches its own side of the base.")
    args = p.parse_args(argv)

    if args.csv:
        csv_paths = list(args.csv)
        out_dir = os.path.dirname(os.path.abspath(csv_paths[0])) or "."
    else:
        csv_paths, out_dir = _interactive_pick(args.root)

    mode = args.mode if args.mode != "auto" else _autodetect_mode(csv_paths)
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_md  = args.out or os.path.join(out_dir, f"spatial_{mode}_{stamp}.md")
    out_png = out_md[:-3] + ".png" if out_md.endswith(".md") \
              else os.path.join(out_dir, f"spatial_{mode}_{stamp}.png")

    if mode == "lr-diff":
        if len(csv_paths) != 2:
            print("  [lr-diff] need exactly 2 CSVs (a left and a right)",
                  file=sys.stderr)
            return 1
        left, right = csv_paths
        if "_right" in os.path.basename(left):
            left, right = right, left      # swap so left is actually left
        _lr_diff_run(left, right, args.voxel_size, args.metric,
                     args.min_count, args.topn, out_md, out_png,
                     mirror_left_y=args.mirror_left_y)
    else:
        if len(csv_paths) != 1:
            print(f"  [ori-slice] expected 1 CSV, got {len(csv_paths)} — using first",
                  file=sys.stderr)
        d = _load_csv(csv_paths[0])
        _ori_slice_run(d, args.voxel_size, args.metric, args.topn,
                       args.min_count, out_md, out_png)
    return 0


if __name__ == "__main__":
    sys.exit(main())
