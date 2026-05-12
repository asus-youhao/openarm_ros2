#!/usr/bin/env python3
"""
placo_ws_analyze.py
===================
Analyse a reachability CSV (output of placo_ws_reachability.py) and extract
the actual arm workspace as a non-rectangular 3D shape.

比起簡單的 (x_min,x_max) 矩形 box，本工具從掃描結果萃取真實的可達空間：
  · KDTree  — 最近鄰查詢（支援非凸形狀）
  · Convex Hull — 可達點凸包（scipy），快速 membership test
  · Per-Z 輪廓 — 每個 Z 高度的可達面積輪廓圖

Outputs
-------
  <stem>_workspace.npz   — voxel point cloud（直接可載入 WorkspaceMesh）
  <stem>_analysis.png    — 全面視覺化（per-Z slices / 3D / hull cross-sections）
  <stem>_hull.json       — 凸包頂點（輕量，可給其他工具使用）

WorkspaceMesh（runtime workspace check，可在 profiler 中使用）
--------------------------------------------------------------
  from placo_ws_analyze import WorkspaceMesh

  ws = WorkspaceMesh.load("reachability_workspace.npz")

  # 判斷某點是否在可達空間內
  is_ok = ws.contains([0.2, -0.3, 0.5])

  # 夾到最近可達點（取代矩形 clamp）
  clamped_xyz, was_inside = ws.clamp([0.2, -0.3, 0.5])

Usage examples
--------------
  # 分析掃描結果 → 生成 workspace 定義 + 圖表
  python3 placo_ws_analyze.py results/20260511/reachability_20260511_right.csv

  # 指定輸出
  python3 placo_ws_analyze.py scan.csv --out-dir ~/ws_output --name right_ws

  # 只生成圖，不儲存 npz
  python3 placo_ws_analyze.py scan.csv --plot-only

  # 查看特定 Z 範圍的輪廓
  python3 placo_ws_analyze.py scan.csv --z-slices 0.40 0.50 0.60 0.70
"""

import argparse
import json
import os
import sys
from typing import List, Optional, Tuple

import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# WorkspaceMesh  (importable class — use in placo_ik_online_profiler.py)
# ─────────────────────────────────────────────────────────────────────────────
class WorkspaceMesh:
    """
    Non-rectangular workspace check based on a KDTree over scanned reachable
    grid points + optional ConvexHull membership.

    Use case:  replace rectangular box clamp in placo_ik_online_profiler.py
               with real-data-driven workspace boundary.

    Usage:
        ws = WorkspaceMesh.load("reachability_workspace.npz")

        # Membership test (cheap, KDTree)
        inside = ws.contains(np.array([0.2, -0.3, 0.5]))

        # Clamp point to nearest reachable voxel (drop-in for rectangular clamp)
        clamped, was_inside = ws.clamp(np.array([0.2, -0.3, 0.5]))

        # Strict: test convex-hull membership (requires scipy)
        inside_hull = ws.in_convex_hull(np.array([0.2, -0.3, 0.5]))
    """

    def __init__(
        self,
        reachable_xyz: np.ndarray,   # shape (N, 3)
        step:          float,        # grid resolution in metres
        min_orient_rate: float = 0.0,  # orient rate threshold used when building
    ):
        from scipy.spatial import KDTree
        self._xyz   = np.array(reachable_xyz, dtype=np.float64)
        self._step  = float(step)
        self._tree  = KDTree(self._xyz)
        self._hull  = None  # lazily built
        self._hull_del = None
        self._min_or   = min_orient_rate

        # Axis-aligned bounding box (fast pre-filter)
        self._lo = self._xyz.min(axis=0)
        self._hi = self._xyz.max(axis=0)

        print(f"[WorkspaceMesh] {len(self._xyz)} reachable points  "
              f"step={step}m  bbox X[{self._lo[0]:.3f},{self._hi[0]:.3f}]"
              f"  Y[{self._lo[1]:.3f},{self._hi[1]:.3f}]"
              f"  Z[{self._lo[2]:.3f},{self._hi[2]:.3f}]")

    # ── membership tests ──────────────────────────────────────────────────────
    def contains(
        self,
        point: np.ndarray,
        tol_factor: float = 0.60,
    ) -> bool:
        """
        True if `point` is within tol_factor * step of ANY reachable grid voxel.
        tol_factor=0.60 → accepts points within the same voxel cell (half-diagonal ~0.87*step/2).
        """
        dist, _ = self._tree.query(np.asarray(point, dtype=np.float64))
        return dist < self._step * tol_factor

    def in_convex_hull(self, point: np.ndarray) -> bool:
        """
        Strict convex-hull membership test (requires scipy).
        More conservative than KDTree: only accepts points strictly inside
        the convex envelope of reachable voxels.
        """
        from scipy.spatial import Delaunay
        if self._hull_del is None:
            self._hull_del = Delaunay(self._xyz)
        return self._hull_del.find_simplex(
            np.asarray(point, dtype=np.float64)) >= 0

    # ── clamping ──────────────────────────────────────────────────────────────
    def clamp(
        self,
        point: np.ndarray,
        tol_factor: float = 0.60,
    ) -> Tuple[np.ndarray, bool]:
        """
        Drop-in replacement for rectangular workspace clamp.

        Returns:
            (clamped_xyz, was_inside)

        If point is inside the reachable workspace → returns point unchanged.
        If outside → returns nearest reachable voxel centre.

        Example (replace rectangular clamp in online profiler):
            # BEFORE:
            new_x = max(ws["x"][0], min(ws["x"][1], raw_x))

            # AFTER:
            clamped, inside = workspace_mesh.clamp(np.array([raw_x, raw_y, raw_z]))
            new_x, new_y, new_z = clamped
        """
        p = np.asarray(point, dtype=np.float64)
        dist, idx = self._tree.query(p)
        inside = dist < self._step * tol_factor
        if inside:
            return p.copy(), True
        else:
            return self._xyz[idx].copy(), False

    def nearest(self, point: np.ndarray) -> Tuple[np.ndarray, float]:
        """Return (nearest_reachable_xyz, distance_m)."""
        p = np.asarray(point, dtype=np.float64)
        dist, idx = self._tree.query(p)
        return self._xyz[idx].copy(), float(dist)

    # ── persistence ───────────────────────────────────────────────────────────
    def save(self, path: str):
        np.savez_compressed(
            path,
            reachable_xyz    = self._xyz,
            step             = np.float64(self._step),
            min_orient_rate  = np.float64(self._min_or),
        )
        print(f"  [WorkspaceMesh] saved → {path}")

    @classmethod
    def load(cls, path: str) -> "WorkspaceMesh":
        data = np.load(path)
        return cls(
            reachable_xyz    = data["reachable_xyz"],
            step             = float(data["step"]),
            min_orient_rate  = float(data.get("min_orient_rate", 0.0)),
        )

    # ── summary ───────────────────────────────────────────────────────────────
    def summary(self) -> dict:
        """Return a dict with key workspace statistics."""
        zs = np.unique(np.round(self._xyz[:, 2], 6))
        per_z = {}
        for z in zs:
            mask = np.abs(self._xyz[:, 2] - z) < 1e-5
            pts  = self._xyz[mask]
            # approximate area: n_points × (step^2)
            per_z[round(float(z), 4)] = {
                "n_voxels": int(mask.sum()),
                "area_cm2": round(mask.sum() * self._step**2 * 1e4, 1),
            }
        total_vol = len(self._xyz) * self._step**3 * 1e6  # cm³
        return {
            "n_reachable_voxels": len(self._xyz),
            "step_m":             self._step,
            "bbox": {
                "x": [round(float(self._lo[0]), 4), round(float(self._hi[0]), 4)],
                "y": [round(float(self._lo[1]), 4), round(float(self._hi[1]), 4)],
                "z": [round(float(self._lo[2]), 4), round(float(self._hi[2]), 4)],
            },
            "total_volume_cm3":   round(total_vol, 1),
            "per_z_level":        per_z,
        }


# ─────────────────────────────────────────────────────────────────────────────
# CSV loading + aggregation
# ─────────────────────────────────────────────────────────────────────────────
def load_csv(path: str):
    """
    Load reachability CSV.
    Handles both:
      · raw per-(xyz,rpy) rows  (from placo_ws_reachability.py)
      · pre-aggregated rows     (with pos_reachable column)
    Returns (rows, is_aggregated).
    """
    import csv
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) if k not in () else v
                         for k, v in row.items()})
    if not rows:
        raise ValueError(f"Empty CSV: {path}")

    is_agg = "pos_reachable" in rows[0]
    print(f"  Loaded {len(rows)} rows  "
          f"({'aggregated' if is_agg else 'raw per-rpy'})")
    return rows, is_agg


def aggregate(rows: List[dict], min_orient_rate: float = 0.0) -> List[dict]:
    """
    Group raw per-(xyz,rpy) rows → one dict per unique (x,y,z).
    min_orient_rate: only mark pos_reachable=1 if n_ok/n_rpy >= threshold.
      0.0 → any one orientation succeeds = reachable  (default)
      0.5 → need ≥50% of orientations to succeed
      1.0 → ALL orientations must succeed (strictest)
    """
    from collections import defaultdict
    groups = defaultdict(list)
    for r in rows:
        groups[(r["x"], r["y"], r["z"])].append(r)

    agg = []
    for (x, y, z), grp in groups.items():
        n_ok   = sum(r["success"] for r in grp)
        n_total = len(grp)
        orient_rate = n_ok / n_total
        agg.append({
            "x":             x,
            "y":             y,
            "z":             z,
            "pos_reachable": 1 if orient_rate >= max(min_orient_rate, 1e-9) else 0,
            "n_orient_ok":   n_ok,
            "orient_rate":   orient_rate,
            "best_err_mm":   min(r["pos_err_mm"] for r in grp),
        })
    return agg


# ─────────────────────────────────────────────────────────────────────────────
# Build WorkspaceMesh from aggregated data
# ─────────────────────────────────────────────────────────────────────────────
def build_workspace(
    agg: List[dict],
    step: float,
    min_orient_rate: float = 0.0,
) -> WorkspaceMesh:
    pts = np.array(
        [[a["x"], a["y"], a["z"]] for a in agg if a["pos_reachable"]],
        dtype=np.float64,
    )
    if len(pts) == 0:
        raise ValueError("No reachable points found — check min_orient_rate or CSV content")
    return WorkspaceMesh(pts, step=step, min_orient_rate=min_orient_rate)


# ─────────────────────────────────────────────────────────────────────────────
# Plotting
# ─────────────────────────────────────────────────────────────────────────────
def save_analysis_plot(
    agg:       List[dict],
    ws:        WorkspaceMesh,
    step:      float,
    arm:       str,
    plot_path: str,
    z_slices:  Optional[List[float]] = None,
):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap
    from matplotlib.patches import Patch

    cmap_or = plt.cm.RdYlGn
    cmap_sr = LinearSegmentedColormap.from_list("sr", ["#cc2222", "#22aa22"])

    ax_  = np.array([a["x"]           for a in agg])
    ay_  = np.array([a["y"]           for a in agg])
    az_  = np.array([a["z"]           for a in agg])
    pr_  = np.array([a["pos_reachable"] for a in agg], dtype=float)
    or_  = np.array([a["orient_rate"]  for a in agg])

    ok_xyz  = ws._xyz
    all_xyz = np.stack([ax_, ay_, az_], axis=1)
    fail_mask = pr_ == 0

    # Convex hull (try scipy)
    hull_verts = None
    try:
        from scipy.spatial import ConvexHull
        if len(ok_xyz) >= 4:
            hull = ConvexHull(ok_xyz)
            hull_verts = ok_xyz[hull.vertices]
    except Exception:
        pass

    # ── Z slices ──────────────────────────────────────────────────────────────
    z_unique = np.unique(np.round(az_, 6))
    if z_slices:
        # snap requested z to nearest available
        z_plot = []
        for zs in z_slices:
            idx = np.argmin(np.abs(z_unique - zs))
            z_plot.append(z_unique[idx])
        z_plot = sorted(set(round(z, 6) for z in z_plot))
    else:
        # auto-select up to 9 evenly spaced levels
        n_pick = min(9, len(z_unique))
        idxs   = np.linspace(0, len(z_unique)-1, n_pick, dtype=int)
        z_plot = [z_unique[i] for i in idxs]

    n_z  = len(z_plot)
    n_cols_z = min(n_z, 3)
    n_rows_z = (n_z + n_cols_z - 1) // n_cols_z

    # Total figure: 3 rows overview + z-slice rows
    n_overview_rows = 3
    total_rows = n_overview_rows + n_rows_z
    fig = plt.figure(figsize=(18, 5 * total_rows))
    fig.suptitle(
        f"Workspace Shape Analysis  arm={arm}  step={step*100:.0f}cm\n"
        f"Reachable voxels: {len(ok_xyz)}  "
        f"(out of {len(agg)} grid pts, {len(ok_xyz)/len(agg)*100:.1f}%)\n"
        f"Bounding box: "
        f"X[{ws._lo[0]:.2f},{ws._hi[0]:.2f}]  "
        f"Y[{ws._lo[1]:.2f},{ws._hi[1]:.2f}]  "
        f"Z[{ws._lo[2]:.2f},{ws._hi[2]:.2f}]",
        fontsize=11, y=0.99,
    )

    outer = fig.add_gridspec(total_rows, 1, hspace=0.55)
    gs_ov  = outer[0:n_overview_rows].subgridspec(n_overview_rows, 3, hspace=0.50, wspace=0.40)
    gs_z   = outer[n_overview_rows:].subgridspec(n_rows_z, n_cols_z, hspace=0.55, wspace=0.40)

    # ── Overview Row 0: 3D reachable | 3D orient rate | 3D fail ──────────────
    ax3d = fig.add_subplot(gs_ov[0, 0], projection="3d")
    ax3d.scatter(ok_xyz[:, 0], ok_xyz[:, 1], ok_xyz[:, 2],
                 c="green", s=3, alpha=0.15)
    if fail_mask.any():
        ax3d.scatter(ax_[fail_mask], ay_[fail_mask], az_[fail_mask],
                     c="red", s=6, alpha=0.55)
    ax3d.set_xlabel("X", fontsize=6); ax3d.set_ylabel("Y", fontsize=6)
    ax3d.set_zlabel("Z", fontsize=6)
    ax3d.set_title("3D Reachable Space\ngreen=OK  red=ALL_fail", fontsize=8)
    ax3d.tick_params(labelsize=5)

    ax3d2 = fig.add_subplot(gs_ov[0, 1], projection="3d")
    sc = ax3d2.scatter(ax_, ay_, az_, c=or_, cmap=cmap_or,
                       s=3, alpha=0.35, vmin=0, vmax=1)
    plt.colorbar(sc, ax=ax3d2, shrink=0.55, pad=0.12, label="orient. rate")
    ax3d2.set_xlabel("X", fontsize=6); ax3d2.set_ylabel("Y", fontsize=6)
    ax3d2.set_zlabel("Z", fontsize=6)
    ax3d2.set_title("3D Orientation Coverage\n(colour = n_ok/n_rpy)", fontsize=8)
    ax3d2.tick_params(labelsize=5)

    # Convex hull wireframe
    ax3d3 = fig.add_subplot(gs_ov[0, 2], projection="3d")
    ax3d3.scatter(ok_xyz[:, 0], ok_xyz[:, 1], ok_xyz[:, 2],
                  c="steelblue", s=2, alpha=0.10)
    if hull_verts is not None:
        try:
            from scipy.spatial import ConvexHull
            hull2 = ConvexHull(ok_xyz)
            for simplex in hull2.simplices:
                vts = ok_xyz[simplex]
                ax3d3.plot3D(*zip(*np.vstack([vts, vts[0]])),
                             color="navy", alpha=0.25, linewidth=0.5)
        except Exception:
            pass
    ax3d3.set_xlabel("X", fontsize=6); ax3d3.set_ylabel("Y", fontsize=6)
    ax3d3.set_zlabel("Z", fontsize=6)
    ax3d3.set_title("Convex Hull Wireframe\n(conservative boundary)", fontsize=8)
    ax3d3.tick_params(labelsize=5)

    # ── Overview Row 1: XY / XZ / YZ projected reachability ──────────────────
    def _proj_hm(ax_obj, da, db, la, lb, title, val=pr_):
        a_bins = np.unique(np.round(da, 6))
        b_bins = np.unique(np.round(db, 6))
        a_idx  = {v: i for i, v in enumerate(a_bins)}
        b_idx  = {v: i for i, v in enumerate(b_bins)}
        counts = np.zeros((len(b_bins), len(a_bins)))
        totals = np.zeros((len(b_bins), len(a_bins)))
        for xi, yi, vi in zip(da, db, val):
            ai = a_idx.get(round(float(xi), 6))
            bi = b_idx.get(round(float(yi), 6))
            if ai is not None and bi is not None:
                totals[bi, ai] += 1
                counts[bi, ai] += vi
        with np.errstate(invalid="ignore"):
            grid = np.where(totals > 0, counts / totals, np.nan)
        im = ax_obj.imshow(grid, origin="lower", aspect="auto",
                           extent=[a_bins[0]-step/2, a_bins[-1]+step/2,
                                   b_bins[0]-step/2, b_bins[-1]+step/2],
                           vmin=0, vmax=1, cmap=cmap_sr)
        plt.colorbar(im, ax=ax_obj, shrink=0.75, label="pos reach.")
        ax_obj.set_xlabel(la, fontsize=8); ax_obj.set_ylabel(lb, fontsize=8)
        ax_obj.set_title(title, fontsize=8); ax_obj.tick_params(labelsize=7)

    _proj_hm(fig.add_subplot(gs_ov[1, 0]), ax_, ay_, "X (m)", "Y (m)",
             "XY projection (top view)\npos reachability")
    _proj_hm(fig.add_subplot(gs_ov[1, 1]), ax_, az_, "X (m)", "Z (m)",
             "XZ projection (front view)\npos reachability")
    _proj_hm(fig.add_subplot(gs_ov[1, 2]), ay_, az_, "Y (m)", "Z (m)",
             "YZ projection (side view)\npos reachability")

    # ── Overview Row 2: volume per Z | orient rate per Z | area comparison ───
    z_levels = np.unique(np.round(az_, 6))
    reach_per_z   = []
    area_per_z_cm = []
    or_per_z      = []
    for zl in z_levels:
        mz = np.abs(az_ - zl) < 1e-5
        reach_per_z.append(pr_[mz].mean() * 100.0)
        area_per_z_cm.append(pr_[mz].sum() * step**2 * 1e4)
        or_per_z.append(or_[mz].mean() * 100.0)

    ax_zr = fig.add_subplot(gs_ov[2, 0])
    ax_zr.barh(z_levels, reach_per_z, height=step*0.80,
               color=[cmap_sr(v/100) for v in reach_per_z])
    ax_zr.set_xlabel("pos reachable (%)", fontsize=8)
    ax_zr.set_ylabel("Z (m)", fontsize=8)
    ax_zr.set_xlim(0, 108)
    ax_zr.set_title("Reachable positions per Z level", fontsize=8)
    ax_zr.tick_params(labelsize=7)

    ax_za = fig.add_subplot(gs_ov[2, 1])
    ax_za.barh(z_levels, area_per_z_cm, height=step*0.80, color="steelblue", alpha=0.8)
    ax_za.set_xlabel("reachable area (cm²)", fontsize=8)
    ax_za.set_ylabel("Z (m)", fontsize=8)
    ax_za.set_title("Reachable area per Z level\n(approx, based on grid density)", fontsize=8)
    ax_za.tick_params(labelsize=7)

    ax_zo = fig.add_subplot(gs_ov[2, 2])
    ax_zo.barh(z_levels, or_per_z, height=step*0.80,
               color=[cmap_or(v/100) for v in or_per_z])
    ax_zo.set_xlabel("mean orient. coverage (%)", fontsize=8)
    ax_zo.set_ylabel("Z (m)", fontsize=8)
    ax_zo.set_title("Mean orientation coverage per Z level", fontsize=8)
    ax_zo.tick_params(labelsize=7)

    # ── Z-slice panels ────────────────────────────────────────────────────────
    for i, zl in enumerate(z_plot):
        row_i = i // n_cols_z
        col_i = i % n_cols_z
        ax_sl = fig.add_subplot(gs_z[row_i, col_i])

        # All points at this Z
        mz   = np.abs(az_ - zl) < 1e-5
        sc_x = ax_[mz]; sc_y = ay_[mz]
        sc_r = pr_[mz]; sc_o = or_[mz]

        # Colour by orient_rate
        sc2 = ax_sl.scatter(sc_x[sc_r == 1], sc_y[sc_r == 1],
                            c=sc_o[sc_r == 1], cmap=cmap_or,
                            s=max(3, int(step * 600))**2 * 0.18,
                            marker="s", vmin=0, vmax=1,
                            label="reachable")
        if (sc_r == 0).any():
            ax_sl.scatter(sc_x[sc_r == 0], sc_y[sc_r == 0],
                          c="red", s=max(3, int(step * 600))**2 * 0.18,
                          marker="x", alpha=0.7, label="fail")

        # Bounding box of reachable points at this Z
        if (sc_r == 1).any():
            rx = sc_x[sc_r == 1]; ry = sc_y[sc_r == 1]
            bbox = plt.Rectangle(
                (rx.min() - step/2, ry.min() - step/2),
                rx.max() - rx.min() + step,
                ry.max() - ry.min() + step,
                linewidth=1.2, edgecolor="navy", facecolor="none",
                linestyle="--", label=f"Z-slice bbox",
            )
            ax_sl.add_patch(bbox)
            area_cm2 = (sc_r == 1).sum() * step**2 * 1e4
            ax_sl.set_title(
                f"Z = {zl:.3f} m  ({(sc_r==1).sum()} pts, ~{area_cm2:.0f} cm²)",
                fontsize=8)
        else:
            ax_sl.set_title(f"Z = {zl:.3f} m  (0 reachable)", fontsize=8)

        plt.colorbar(sc2, ax=ax_sl, shrink=0.75, label="orient rate")
        ax_sl.set_xlabel("X (m)", fontsize=7)
        ax_sl.set_ylabel("Y (m)", fontsize=7)
        ax_sl.tick_params(labelsize=6)
        ax_sl.grid(True, alpha=0.2)
        ax_sl.set_aspect("equal", adjustable="box")

    plt.savefig(plot_path, dpi=120, bbox_inches="tight")
    plt.close()
    print(f"  Analysis plot → {plot_path}")


# ─────────────────────────────────────────────────────────────────────────────
# Hull JSON export
# ─────────────────────────────────────────────────────────────────────────────
def save_hull_json(ws: WorkspaceMesh, path: str):
    try:
        from scipy.spatial import ConvexHull
        hull = ConvexHull(ws._xyz)
        data = {
            "n_vertices": len(hull.vertices),
            "volume_m3":  round(float(hull.volume), 6),
            "area_m2":    round(float(hull.area), 6),
            "vertices":   ws._xyz[hull.vertices].tolist(),
            "step_m":     ws._step,
            "bbox": {
                "x": [round(float(ws._lo[0]), 4), round(float(ws._hi[0]), 4)],
                "y": [round(float(ws._lo[1]), 4), round(float(ws._hi[1]), 4)],
                "z": [round(float(ws._lo[2]), 4), round(float(ws._hi[2]), 4)],
            },
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        print(f"  Convex hull JSON → {path}  "
              f"(vol={data['volume_m3']*1e6:.0f} cm³, "
              f"vertices={data['n_vertices']})")
    except Exception as e:
        print(f"  [hull] Skipped: {e}")


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────
def _parse_args():
    p = argparse.ArgumentParser(
        description="Analyse reachability CSV → WorkspaceMesh + plots",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("csv", help="Reachability CSV path (from placo_ws_reachability.py)")
    p.add_argument("--arm",      default="right", choices=["right", "left"])
    p.add_argument("--step",     type=float, default=None,
                   help="Grid step (m). Auto-detected from CSV if omitted.")
    p.add_argument("--min-orient-rate", type=float, default=0.0,
                   dest="min_orient_rate",
                   help="Minimum fraction of orientations that must succeed to "
                        "mark a pos as reachable. 0=any, 0.5=half, 1.0=all. "
                        "(default: 0.0)")
    p.add_argument("--out-dir",  default=None, dest="out_dir",
                   help="Output directory (default: same dir as CSV)")
    p.add_argument("--name",     default=None,
                   help="Output file stem (default: CSV stem + '_ws')")
    p.add_argument("--z-slices", type=float, nargs="+", default=None,
                   dest="z_slices",
                   help="Z heights to show in slice panels (auto if omitted)")
    p.add_argument("--plot-only", action="store_true", dest="plot_only",
                   help="Skip saving .npz / .json, only generate plot")
    return p.parse_args()


def _detect_step(agg: List[dict]) -> float:
    """Infer grid step from coordinate differences."""
    xs = sorted(set(round(a["x"], 6) for a in agg))
    if len(xs) >= 2:
        diffs = [abs(xs[i+1]-xs[i]) for i in range(min(5, len(xs)-1))]
        return round(min(d for d in diffs if d > 1e-6), 6)
    return 0.05  # fallback


def main():
    args = _parse_args()

    csv_path = args.csv
    if not os.path.isfile(csv_path):
        print(f"ERROR: CSV not found: {csv_path}", file=sys.stderr)
        sys.exit(1)

    # ── Load + aggregate ──────────────────────────────────────────────────────
    print(f"\n{'═'*65}")
    print(f"  Workspace Analyser")
    print(f"  CSV: {csv_path}")
    rows, is_agg = load_csv(csv_path)

    if is_agg:
        agg = rows
    else:
        agg = aggregate(rows, min_orient_rate=args.min_orient_rate)

    step = args.step or _detect_step(agg)
    print(f"  step={step}m  min_orient_rate={args.min_orient_rate}"
          f"  total_grid={len(agg)}")

    n_reach = sum(a["pos_reachable"] for a in agg)
    print(f"  Reachable positions: {n_reach}/{len(agg)} "
          f"({n_reach/len(agg)*100:.1f}%)")

    # ── Build WorkspaceMesh ───────────────────────────────────────────────────
    ws = build_workspace(agg, step=step, min_orient_rate=args.min_orient_rate)

    # ── Print summary ─────────────────────────────────────────────────────────
    summ = ws.summary()
    print(f"\n  Workspace summary:")
    print(f"  Voxels: {summ['n_reachable_voxels']}  "
          f"Volume: ~{summ['total_volume_cm3']:.0f} cm³")
    print(f"  Bounding box:")
    for ax_name, rng in summ["bbox"].items():
        print(f"    {ax_name}: [{rng[0]:.3f}, {rng[1]:.3f}]  "
              f"(span {(rng[1]-rng[0])*100:.1f} cm)")

    print(f"\n  Per-Z reachable area:")
    print(f"  {'Z(m)':>7}  {'voxels':>7}  {'area_cm²':>9}  bar")
    max_vox = max(v["n_voxels"] for v in summ["per_z_level"].values())
    for zl, info in sorted(summ["per_z_level"].items()):
        bar_len = int(info["n_voxels"] / max(max_vox, 1) * 30)
        print(f"  {zl:7.3f}  {info['n_voxels']:7d}  "
              f"{info['area_cm2']:9.1f}  {'█'*bar_len}")

    # ── Output paths ──────────────────────────────────────────────────────────
    out_dir = args.out_dir or os.path.dirname(os.path.abspath(csv_path))
    os.makedirs(out_dir, exist_ok=True)
    stem     = args.name or (os.path.splitext(os.path.basename(csv_path))[0] + "_ws")
    npz_path  = os.path.join(out_dir, f"{stem}.npz")
    plot_path = os.path.join(out_dir, f"{stem}_analysis.png")
    hull_path = os.path.join(out_dir, f"{stem}_hull.json")

    print(f"\n  Output stem: {stem}")
    print(f"  NPZ:  {npz_path}")
    print(f"  Plot: {plot_path}")
    print(f"{'═'*65}")

    # ── Save ──────────────────────────────────────────────────────────────────
    if not args.plot_only:
        ws.save(npz_path)
        save_hull_json(ws, hull_path)

    # ── Plot ──────────────────────────────────────────────────────────────────
    save_analysis_plot(
        agg=agg,
        ws=ws,
        step=step,
        arm=args.arm,
        plot_path=plot_path,
        z_slices=args.z_slices,
    )

    # ── Usage hint ────────────────────────────────────────────────────────────
    if not args.plot_only:
        print(f"\n  ── 如何在 placo_ik_online_profiler.py 中使用 ──")
        print(f"  from placo_ws_analyze import WorkspaceMesh")
        print(f"  ws_mesh = WorkspaceMesh.load('{npz_path}')")
        print(f"  # 取代矩形 clamp：")
        print(f"  clamped, inside = ws_mesh.clamp(np.array([raw_x, raw_y, raw_z]))")
        print(f"  new_x, new_y, new_z = clamped\n")


if __name__ == "__main__":
    main()
