#!/usr/bin/env python3
"""
placo_ws_view3d.py
==================
互動式 3D Workspace 可視化工具
讀取 placo_ws_reachability.py 產生的 CSV（或 WorkspaceMesh .npz），
用 matplotlib 顯示互動式 3D 圖。

使用方法
--------
  # 從 CSV (raw per-rpy)
  conda run -n pico_teleop_py python3 placo_ws_view3d.py results/reachability_right.csv

  # 從 WorkspaceMesh npz
  conda run -n pico_teleop_py python3 placo_ws_view3d.py results/reachability_right_ws.npz

  # 指定 orient rate 門檻（0=任意角度OK, 0.5=半數, 1.0=全部）
  conda run -n pico_teleop_py python3 placo_ws_view3d.py scan.csv --min-or 0.5

  # 只顯示失敗點
  conda run -n pico_teleop_py python3 placo_ws_view3d.py scan.csv --show fail

  # 存成 PNG 不互動
  conda run -n pico_teleop_py python3 placo_ws_view3d.py scan.csv --save view3d.png
"""

import argparse
import csv
import os
import sys
from collections import defaultdict
from typing import List, Optional

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.cm import ScalarMappable
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.widgets import Slider, Button, CheckButtons


# ── Data loading ──────────────────────────────────────────────────────────────
def _load_csv(path: str):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    return rows


def _aggregate(rows: List[dict], min_or: float = 0.0):
    groups = defaultdict(list)
    for r in rows:
        groups[(r["x"], r["y"], r["z"])].append(r)

    agg = []
    for (x, y, z), grp in groups.items():
        n_ok = sum(r["success"] for r in grp)
        or_  = n_ok / len(grp)
        agg.append({
            "x": x, "y": y, "z": z,
            "pos_reachable": 1 if or_ >= max(min_or, 1e-9) else 0,
            "orient_rate":   or_,
            "n_orient_ok":   n_ok,
            "n_rpy":         len(grp),
            "best_err_mm":   min(r["pos_err_mm"] for r in grp),
        })
    return agg


def load_data(path: str, min_or: float = 0.0):
    """Returns aggregated list of dicts (one per xyz), step estimate."""
    if path.endswith(".npz"):
        data = np.load(path)
        xyz  = data["reachable_xyz"]
        step = float(data.get("step", 0.05))
        # All points in npz are reachable
        agg  = [{"x": float(p[0]), "y": float(p[1]), "z": float(p[2]),
                  "pos_reachable": 1, "orient_rate": 1.0,
                  "n_orient_ok": 1, "n_rpy": 1, "best_err_mm": 0.0}
                for p in xyz]
        print(f"[load] NPZ: {len(agg)} reachable voxels  step={step}m")
        return agg, step

    rows = _load_csv(path)
    agg  = _aggregate(rows, min_or)
    # Detect step
    xs = sorted(set(round(a["x"], 6) for a in agg))
    diffs = [abs(xs[i+1]-xs[i]) for i in range(min(5, len(xs)-1))] if len(xs) > 1 else [0.05]
    step = round(min(d for d in diffs if d > 1e-6), 6)
    n_ok = sum(a["pos_reachable"] for a in agg)
    print(f"[load] CSV: {len(rows)} raw rows → {len(agg)} xyz pts  "
          f"reachable={n_ok}/{len(agg)} ({n_ok/len(agg)*100:.1f}%)  step={step}m")
    return agg, step


# ── Main viewer ───────────────────────────────────────────────────────────────
def view3d(
    agg:     List[dict],
    step:    float,
    arm:     str     = "right",
    show:    str     = "all",     # "all" | "ok" | "fail" | "orient"
    save:    Optional[str] = None,
):
    """
    Interactive 3D + 2D projection viewer.

    Controls (when interactive):
      · Drag  — rotate 3D view
      · Slider Z_min / Z_max — filter Z slice
      · Buttons — toggle OK / FAIL / Hull
    """
    xs   = np.array([a["x"]           for a in agg])
    ys   = np.array([a["y"]           for a in agg])
    zs   = np.array([a["z"]           for a in agg])
    pr   = np.array([a["pos_reachable"] for a in agg], dtype=float)
    or_  = np.array([a["orient_rate"]  for a in agg])

    ok_m  = pr == 1
    fal_m = pr == 0
    n_ok  = ok_m.sum()
    n_tot = len(agg)

    cmap_or = plt.cm.RdYlGn
    cmap_sr = LinearSegmentedColormap.from_list("sr", ["#cc3333", "#33aa33"])

    z_min_g = float(zs.min())
    z_max_g = float(zs.max())

    # ── figure layout ─────────────────────────────────────────────────────────
    interactive = save is None
    if interactive:
        matplotlib.use("TkAgg")

    fig = plt.figure(figsize=(20, 13))
    fig.patch.set_facecolor("#1a1a2e")

    # Main 3D (large)
    ax3d = fig.add_axes([0.02, 0.20, 0.52, 0.74], projection="3d")
    ax3d.set_facecolor("#0f0f1e")

    # 3 small 2D projections on the right
    ax_xy = fig.add_axes([0.57, 0.62, 0.19, 0.30])
    ax_xz = fig.add_axes([0.79, 0.62, 0.19, 0.30])
    ax_yz = fig.add_axes([0.57, 0.22, 0.19, 0.30])
    ax_zr = fig.add_axes([0.79, 0.22, 0.19, 0.30])   # Z reachability bar

    for ax in [ax_xy, ax_xz, ax_yz, ax_zr]:
        ax.set_facecolor("#12122a")
        ax.tick_params(colors="white", labelsize=6)
        for sp in ax.spines.values():
            sp.set_edgecolor("#444466")

    # ── state ─────────────────────────────────────────────────────────────────
    z_lo = [z_min_g]
    z_hi = [z_max_g]
    show_ok   = [True]
    show_fail = [True]
    show_hull = [False]

    # ── draw fn ───────────────────────────────────────────────────────────────
    def _draw():
        ax3d.cla()
        ax3d.set_facecolor("#0f0f1e")

        # Z filter mask
        zm = (zs >= z_lo[0]) & (zs <= z_hi[0])
        ok_zm   = ok_m  & zm
        fail_zm = fal_m & zm

        # Colour by orient_rate
        norm = Normalize(vmin=0, vmax=1)

        if show_ok[0] and ok_zm.any():
            ax3d.scatter(
                xs[ok_zm], ys[ok_zm], zs[ok_zm],
                c=or_[ok_zm], cmap=cmap_or, norm=norm,
                s=max(6, step * 800),
                alpha=0.55, depthshade=True,
                label=f"reachable ({ok_zm.sum()})",
                rasterized=True,
            )

        if show_fail[0] and fail_zm.any():
            ax3d.scatter(
                xs[fail_zm], ys[fail_zm], zs[fail_zm],
                c="#ff3344", s=max(8, step * 900),
                alpha=0.70, depthshade=True, marker="x",
                label=f"ALL fail ({fail_zm.sum()})",
                rasterized=True,
            )

        # Convex hull skeleton
        if show_hull[0] and ok_zm.sum() >= 4:
            try:
                from scipy.spatial import ConvexHull
                hull = ConvexHull(np.stack(
                    [xs[ok_zm], ys[ok_zm], zs[ok_zm]], axis=1))
                pts_h = np.stack([xs[ok_zm], ys[ok_zm], zs[ok_zm]], axis=1)
                for simplex in hull.simplices[::max(1, len(hull.simplices)//120)]:
                    vts = pts_h[simplex]
                    ax3d.plot3D(*zip(*np.vstack([vts, vts[0]])),
                                color="#4488ff", alpha=0.18, linewidth=0.6)
            except Exception:
                pass

        ax3d.set_xlabel("X (m)", color="white", fontsize=8, labelpad=6)
        ax3d.set_ylabel("Y (m)", color="white", fontsize=8, labelpad=6)
        ax3d.set_zlabel("Z (m)", color="white", fontsize=8, labelpad=6)
        ax3d.tick_params(colors="white", labelsize=6)
        ax3d.xaxis.pane.fill = False
        ax3d.yaxis.pane.fill = False
        ax3d.zaxis.pane.fill = False
        ax3d.xaxis.pane.set_edgecolor("#333355")
        ax3d.yaxis.pane.set_edgecolor("#333355")
        ax3d.zaxis.pane.set_edgecolor("#333355")
        ax3d.grid(True, alpha=0.15)

        n_ok_z = ok_zm.sum()
        n_tot_z = zm.sum()
        ax3d.set_title(
            f"arm={arm}  step={step*100:.0f}cm  "
            f"Z=[{z_lo[0]:.2f},{z_hi[0]:.2f}]\n"
            f"Reachable: {n_ok_z}/{n_tot_z} "
            f"({n_ok_z/max(n_tot_z,1)*100:.1f}%)  "
            f"colour=orient_rate  ×=all_fail",
            color="white", fontsize=9, pad=10,
        )
        leg = ax3d.legend(fontsize=7, loc="upper left",
                          facecolor="#22223a", edgecolor="#5555aa")
        for t in leg.get_texts():
            t.set_color("white")

        # ── 2D projections ────────────────────────────────────────────────────
        def _proj(ax_obj, da, db, la, lb, title):
            ax_obj.cla()
            ax_obj.set_facecolor("#12122a")
            a_bins = np.unique(np.round(da, 6))
            b_bins = np.unique(np.round(db, 6))
            a_idx  = {v: i for i, v in enumerate(a_bins)}
            b_idx  = {v: i for i, v in enumerate(b_bins)}
            counts = np.zeros((len(b_bins), len(a_bins)))
            totals = np.zeros((len(b_bins), len(a_bins)))
            for xi, yi, vi in zip(da[zm], db[zm], pr[zm]):
                ai = a_idx.get(round(float(xi), 6))
                bi = b_idx.get(round(float(yi), 6))
                if ai is not None and bi is not None:
                    totals[bi, ai] += 1
                    counts[bi, ai] += vi
            with np.errstate(invalid="ignore"):
                grid = np.where(totals > 0, counts / totals, np.nan)
            im = ax_obj.imshow(
                grid, origin="lower", aspect="auto", cmap=cmap_sr,
                vmin=0, vmax=1,
                extent=[a_bins[0]-step/2, a_bins[-1]+step/2,
                        b_bins[0]-step/2, b_bins[-1]+step/2],
            )
            ax_obj.set_xlabel(la, color="white", fontsize=7)
            ax_obj.set_ylabel(lb, color="white", fontsize=7)
            ax_obj.set_title(title, color="cyan", fontsize=7)
            ax_obj.tick_params(colors="white", labelsize=6)
            for sp in ax_obj.spines.values():
                sp.set_edgecolor("#444466")

        _proj(ax_xy, xs, ys, "X", "Y", "XY top view")
        _proj(ax_xz, xs, zs, "X", "Z", "XZ front view")
        _proj(ax_yz, ys, zs, "Y", "Z", "YZ side view")

        # Z reachability bar
        ax_zr.cla()
        ax_zr.set_facecolor("#12122a")
        z_levels = np.unique(np.round(zs, 6))
        sr_z = []
        for zl in z_levels:
            mz2 = np.abs(zs - zl) < 1e-5
            sr_z.append(pr[mz2].mean() * 100.0)
        colors_z = [cmap_sr(v / 100) for v in sr_z]
        # highlight currently visible range
        h_colors = [
            ("#00ffcc" if z_lo[0] <= zl <= z_hi[0] else c)
            for zl, c in zip(z_levels, colors_z)
        ]
        ax_zr.barh(z_levels, sr_z, height=step * 0.82, color=h_colors, alpha=0.85)
        ax_zr.axvline(100, color="#888899", linestyle=":", linewidth=0.8)
        ax_zr.set_xlabel("reach %", color="white", fontsize=7)
        ax_zr.set_ylabel("Z (m)", color="white", fontsize=7)
        ax_zr.set_title("Reachable/Z\n(cyan=visible)", color="cyan", fontsize=7)
        ax_zr.set_xlim(0, 115)
        ax_zr.tick_params(colors="white", labelsize=6)
        for sp in ax_zr.spines.values():
            sp.set_edgecolor("#444466")

        fig.canvas.draw_idle()

    # ── Colorbar ──────────────────────────────────────────────────────────────
    cbar_ax = fig.add_axes([0.02, 0.14, 0.52, 0.012])
    sm = ScalarMappable(cmap=cmap_or, norm=Normalize(0, 1))
    sm.set_array([])
    cb = fig.colorbar(sm, cax=cbar_ax, orientation="horizontal")
    cb.set_label("Orientation coverage  (0 = all fail · 1 = all 6 RPY succeed)",
                 color="white", fontsize=8)
    cb.ax.xaxis.set_tick_params(color="white")
    plt.setp(cb.ax.xaxis.get_ticklabels(), color="white", fontsize=7)

    # ── title / stats ─────────────────────────────────────────────────────────
    fig.text(0.50, 0.97,
             f"3D Workspace Viewer   arm={arm}",
             ha="center", color="white", fontsize=13, fontweight="bold")
    fig.text(0.50, 0.94,
             f"Total grid: {n_tot}  Reachable: {n_ok} ({n_ok/n_tot*100:.1f}%)  "
             f"step={step*100:.0f}cm",
             ha="center", color="#aaaacc", fontsize=9)

    if not interactive:
        _draw()
        plt.savefig(save, dpi=140, bbox_inches="tight",
                    facecolor=fig.get_facecolor())
        print(f"  Saved → {save}")
        return

    # ── Widgets ───────────────────────────────────────────────────────────────
    # Z-range slider (two separate sliders)
    ax_slo = fig.add_axes([0.57, 0.13, 0.40, 0.018],
                           facecolor="#22223a")
    ax_shi = fig.add_axes([0.57, 0.10, 0.40, 0.018],
                           facecolor="#22223a")
    sl_lo = Slider(ax_slo, "Z_min", z_min_g, z_max_g,
                   valinit=z_min_g, color="#3355aa",
                   valstep=round(step, 4))
    sl_hi = Slider(ax_shi, "Z_max", z_min_g, z_max_g,
                   valinit=z_max_g, color="#3355aa",
                   valstep=round(step, 4))
    for sl in [sl_lo, sl_hi]:
        sl.label.set_color("white"); sl.valtext.set_color("white")

    def _on_slider(_):
        z_lo[0] = min(sl_lo.val, sl_hi.val)
        z_hi[0] = max(sl_lo.val, sl_hi.val)
        _draw()

    sl_lo.on_changed(_on_slider)
    sl_hi.on_changed(_on_slider)

    # CheckButtons for display toggles
    ax_chk = fig.add_axes([0.57, 0.04, 0.20, 0.055],
                           facecolor="#1a1a2e")
    chk = CheckButtons(ax_chk,
                       ["Show OK", "Show Fail", "Convex Hull"],
                       [True, True, False])
    for t in chk.labels:
        t.set_color("white"); t.set_fontsize(8)
    chk.ax.set_facecolor("#1a1a2e")

    def _on_chk(label):
        if label == "Show OK":
            show_ok[0] = not show_ok[0]
        elif label == "Show Fail":
            show_fail[0] = not show_fail[0]
        elif label == "Convex Hull":
            show_hull[0] = not show_hull[0]
        _draw()

    chk.on_clicked(_on_chk)

    # Reset button
    ax_btn = fig.add_axes([0.80, 0.04, 0.10, 0.05], facecolor="#22223a")
    btn = Button(ax_btn, "Reset Z", color="#22223a", hovercolor="#3344aa")
    btn.label.set_color("white")

    def _on_reset(_):
        sl_lo.set_val(z_min_g)
        sl_hi.set_val(z_max_g)

    btn.on_clicked(_on_reset)

    _draw()

    fig.text(0.02, 0.01,
             "Drag 3D plot to rotate  |  Use Z_min/Z_max sliders to filter height  |  "
             "Checkboxes toggle layers  |  Close window to exit",
             color="#6666aa", fontsize=7)

    plt.show()


# ── CLI ───────────────────────────────────────────────────────────────────────
def _parse_args():
    p = argparse.ArgumentParser(
        description="Interactive 3D workspace viewer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("file",    help="Reachability CSV or WorkspaceMesh .npz")
    p.add_argument("--arm",   default="right", choices=["right", "left"])
    p.add_argument("--min-or", type=float, default=0.0, dest="min_or",
                   help="Min orientation rate to count as reachable (default: 0)")
    p.add_argument("--show",  default="all", choices=["all", "ok", "fail", "orient"],
                   help="Which points to show (default: all)")
    p.add_argument("--save",  default=None,
                   help="Save to PNG instead of interactive display")
    return p.parse_args()


def main():
    args = _parse_args()
    if not os.path.isfile(args.file):
        print(f"ERROR: file not found: {args.file}", file=sys.stderr)
        sys.exit(1)

    agg, step = load_data(args.file, min_or=args.min_or)

    view3d(
        agg=agg,
        step=step,
        arm=args.arm,
        show=args.show,
        save=args.save,
    )


if __name__ == "__main__":
    main()
