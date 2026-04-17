#!/usr/bin/env python3
"""
EE Grasp Orientation Visualizer — plot_ee_orientations.py
==========================================================
Visualises the 6 GRASP_ORIENTATIONS used in ik_timing_benchmark.py
as 3-D EE frames with a simplified two-finger gripper shape.

OpenArm EE convention (MoveIt2):
    Approach / pointing direction = local X-axis  → large colored arrow
    Gripper jaw open/close axis   = local Y-axis  → fingers spread along ±Y
    Lateral roll axis             = local Z-axis

Gripper shape in EE-local space (before rotating):

     ╔════╗  ← left finger   (y = +0.06)
     ║    ║
─────╫────╫──────────────────────────────→  approach (EE local +X)
shaft║    ║crossbar
     ║    ║
     ╚════╝  ← right finger  (y = -0.06)

Usage:
    python3 plot_ee_orientations.py               # save → EE_orientations.png
    python3 plot_ee_orientations.py --out my.png  # custom filename
    python3 plot_ee_orientations.py --dpi 200     # higher resolution
"""

import argparse
import shutil
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3DCollection


# ── Quaternion helper (no scipy dependency) ──────────────────────────────────

def _quat_to_mat(qx, qy, qz, qw) -> np.ndarray:
    """Quaternion (qx,qy,qz,qw) → 3×3 rotation matrix."""
    return np.array([
        [1 - 2*(qy**2 + qz**2),   2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),       1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),       2*(qy*qz + qx*qw),   1 - 2*(qx**2 + qy**2)],
    ])


# ── Grasp orientation definitions ────────────────────────────────────────────
#   quat = (qx, qy, qz, qw)  — MoveIt2 / geometry_msgs convention

GRASP_ORIENTATIONS = {
    "side_neg_y": {
        "quat":  (0.0,  0.0,  -0.707, 0.707),
        "desc":  "Side approach\nEE → −Y\n(right arm natural)",
        "color": "#E91E63",   # pink-red
        "view":  (22, 15),    # (elev°, azim°) for best arrow visibility
    },
    "side_pos_y": {
        "quat":  (0.0,  0.0,   0.707, 0.707),
        "desc":  "Side approach\nEE → +Y\n(left arm natural)",
        "color": "#2196F3",   # blue
        "view":  (22, 195),
    },
    "top_down": {
        "quat":  (0.0,  0.707, 0.0,   0.707),
        "desc":  "Top-down grasp\nEE → −Z\n(over-the-top)",
        "color": "#4CAF50",   # green
        "view":  (35, 30),
    },
    "front_pos_x": {
        "quat":  (0.0,  0.0,   0.0,   1.0),
        "desc":  "Frontal approach\nEE → +X",
        "color": "#FF9800",   # orange
        "view":  (22, 105),
    },
    "front_neg_x": {
        "quat":  (0.0,  0.0,   1.0,   0.0),
        "desc":  "Reverse frontal\nEE → −X",
        "color": "#9C27B0",   # purple
        "view":  (22, 285),
    },
    "bottom_up": {
        "quat":  (0.0, -0.707, 0.0,   0.707),
        "desc":  "Bottom-up grasp\nEE → +Z\n(shelf pickup)",
        "color": "#00BCD4",   # cyan
        "view":  (8, 30),
    },
}

# ── Drawing primitives ────────────────────────────────────────────────────────

def _arrow(ax, start, vec, color, lw=2.5, alpha=1.0, arr_ratio=0.22, zorder=4):
    """3-D quiver arrow."""
    ax.quiver(
        *start, *vec,
        color=color, linewidth=lw,
        arrow_length_ratio=arr_ratio,
        alpha=alpha, normalize=False,
        zorder=zorder,
    )


def _gripper(ax, R: np.ndarray, origin=None, scale=0.14,
             color="#37474F", lw=3.0):
    """
    Simplified two-finger gripper in EE-local space.

    EE-local layout (X = approach, Y = jaw-axis, Z = lateral):
        Shaft    : x  ∈ [-0.03, 0.06]  y=0, z=0
        Cross-bar: y  ∈ [-0.06, 0.06]  x=0.06, z=0
        L-finger : x  ∈ [ 0.06, 0.15]  y=+0.06, z=0
        R-finger : x  ∈ [ 0.06, 0.15]  y=-0.06, z=0
    """
    if origin is None:
        origin = np.zeros(3)
    s = scale
    local = np.array([
        [-0.03,  0.00,  0],   # 0  shaft base
        [ 0.06,  0.00,  0],   # 1  shaft tip / crossbar centre
        [ 0.06, -0.06,  0],   # 2  crossbar left  (−Y jaw)
        [ 0.06,  0.06,  0],   # 3  crossbar right (+Y jaw)
        [ 0.15, -0.06,  0],   # 4  left  finger tip
        [ 0.15,  0.06,  0],   # 5  right finger tip
    ]) * s
    world = (R @ local.T).T + origin
    segs = [
        [world[0], world[1]],   # shaft
        [world[2], world[3]],   # crossbar
        [world[2], world[4]],   # −Y finger
        [world[3], world[5]],   # +Y finger
    ]
    ax.add_collection3d(
        Line3DCollection(segs, colors=color, linewidths=lw, zorder=3)
    )


def _world_axes(ax, scale=0.10, alpha=0.30):
    """Thin world-frame XYZ reference axes at origin."""
    cols  = ["#D32F2F", "#388E3C", "#1565C0"]
    names = ["+X",      "+Y",      "+Z"     ]
    I = np.eye(3)
    for i, (col, lbl) in enumerate(zip(cols, names)):
        _arrow(ax, [0, 0, 0], I[i] * scale, col, lw=1.1, alpha=alpha, zorder=2)
        off = I[i] * scale * 1.20
        ax.text(*off, lbl, color=col, fontsize=7, ha="center", va="center",
                alpha=alpha + 0.25, zorder=2)


def _ee_frame_axes(ax, R, scale=0.09):
    """Small EE-local XYZ axes: X=red, Y=green, Z=blue."""
    cols = ["#EF5350", "#66BB6A", "#42A5F5"]
    for i, col in enumerate(cols):
        _arrow(ax, [0, 0, 0], R[:, i] * scale, col, lw=1.4, alpha=0.70, zorder=3)


def _target_sphere(ax, pos, r=0.013, color="#FF7043", alpha=0.65):
    """Small orange sphere representing the target object."""
    u = np.linspace(0, 2 * np.pi, 14)
    v = np.linspace(0,     np.pi,  9)
    x = pos[0] + r * np.outer(np.cos(u), np.sin(v))
    y = pos[1] + r * np.outer(np.sin(u), np.sin(v))
    z = pos[2] + r * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x, y, z, color=color, alpha=alpha, linewidth=0, zorder=1)


def _axis_clean(ax, lim=0.24):
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    ax.set_xlabel("X", fontsize=7, labelpad=2)
    ax.set_ylabel("Y", fontsize=7, labelpad=2)
    ax.set_zlabel("Z", fontsize=7, labelpad=2)
    ax.tick_params(labelsize=5.5)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor("#BDBDBD")
    ax.yaxis.pane.set_edgecolor("#BDBDBD")
    ax.zaxis.pane.set_edgecolor("#BDBDBD")
    ax.grid(True, linewidth=0.4, alpha=0.4)


# ── Per-orientation detail panel ──────────────────────────────────────────────

def _draw_detail(ax, info: dict):
    qx, qy, qz, qw = info["quat"]
    R     = _quat_to_mat(qx, qy, qz, qw)
    color = info["color"]
    elev, azim = info["view"]

    # World-frame reference
    _world_axes(ax, scale=0.10, alpha=0.28)

    # EE-local XYZ small axes
    _ee_frame_axes(ax, R, scale=0.09)

    # Gripper body
    _gripper(ax, R, scale=0.14, color=color, lw=3.0)

    # Large approach arrow (EE local X → world)
    approach = R @ np.array([1.0, 0.0, 0.0]) * 0.18
    _arrow(ax, [0, 0, 0], approach, color, lw=4.0, arr_ratio=0.20, alpha=0.95, zorder=5)

    # Target sphere ahead of gripper
    target = R @ np.array([0.22, 0.0, 0.0])
    _target_sphere(ax, target, r=0.012, color="#FF7043")
    ax.text(*target * 1.28, "target", fontsize=5.5, color="#BF360C",
            ha="center", va="center", zorder=6)

    _axis_clean(ax, lim=0.26)
    ax.set_title(info["desc"], fontsize=8.5, fontweight="bold",
                 color=color, pad=3, linespacing=1.4)
    ax.view_init(elev=elev, azim=azim)


# ── Overview panel — all 6 at once ────────────────────────────────────────────

def _draw_overview(ax):
    """All 6 approach arrows radiating from a common origin + gripper shapes."""

    _world_axes(ax, scale=0.16, alpha=0.40)

    legend_handles = []
    for name, info in GRASP_ORIENTATIONS.items():
        qx, qy, qz, qw = info["quat"]
        R     = _quat_to_mat(qx, qy, qz, qw)
        color = info["color"]

        # Approach arrow
        approach = R @ np.array([1.0, 0.0, 0.0]) * 0.22
        _arrow(ax, [0, 0, 0], approach, color, lw=3.5, arr_ratio=0.18, alpha=0.90, zorder=4)

        # Gripper (smaller so they don't overlap too much)
        _gripper(ax, R, scale=0.11, color=color, lw=2.2)

        # Target sphere at tip of approach
        tip = R @ np.array([0.28, 0.0, 0.0])
        _target_sphere(ax, tip, r=0.010, color=color, alpha=0.50)

        # Legend entry
        legend_handles.append(
            Line2D([0], [0], color=color, linewidth=3,
                   label=f"{name}  ({info['desc'].splitlines()[0]})")
        )

    ax.legend(
        handles=legend_handles,
        loc="upper left",
        fontsize=7.5,
        framealpha=0.88,
        edgecolor="#90A4AE",
        bbox_to_anchor=(0.02, 0.98),
    )

    _axis_clean(ax, lim=0.32)
    ax.set_title("All 6 EE Approach Directions  (overview)\n"
                 "Arrows from origin · each color = one orientation",
                 fontsize=10, fontweight="bold", color="#263238", pad=6)
    ax.view_init(elev=22, azim=30)


# ── Legend box for EE-frame axis colors ───────────────────────────────────────

def _frame_axis_legend(fig):
    handles = [
        Line2D([0], [0], color="#EF5350", lw=2.5, label="EE local  X (approach)"),
        Line2D([0], [0], color="#66BB6A", lw=2.5, label="EE local  Y (jaw axis)"),
        Line2D([0], [0], color="#42A5F5", lw=2.5, label="EE local  Z (lateral) "),
        Line2D([0], [0], color="#FF7043", lw=0,
               marker="o", ms=6, label="target object"),
    ]
    fig.legend(
        handles=handles,
        loc="lower center",
        ncol=4,
        fontsize=8,
        framealpha=0.90,
        edgecolor="#90A4AE",
        bbox_to_anchor=(0.5, -0.01),
        title="EE frame axes (small arrows in detail panels)",
        title_fontsize=8,
    )


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="EE Grasp Orientation Visualizer")
    parser.add_argument("--out", default="EE_orientations.png",
                        help="Output PNG (default: EE_orientations.png)")
    parser.add_argument("--dpi", type=int, default=150,
                        help="PNG resolution (default: 150)")
    args = parser.parse_args()

    # ── Figure layout ──────────────────────────────────────────────────────
    #
    #  Row 0 (tall):  Overview — all 6 in one 3D view
    #  Row 1 & 2:     6 detail panels (3 × 2)
    #
    fig = plt.figure(figsize=(20, 18))
    fig.patch.set_facecolor("#ECEFF1")

    fig.suptitle(
        "OpenArm End-Effector Grasp Orientations\n"
        "Large colored arrow = EE approach direction (local X-axis of EE frame)",
        fontsize=13, fontweight="bold", color="#263238", y=1.00,
    )

    gs = gridspec.GridSpec(
        3, 3,
        figure=fig,
        height_ratios=[1.55, 1.0, 1.0],
        left=0.04, right=0.97,
        top=0.95, bottom=0.06,
        wspace=0.15, hspace=0.38,
    )

    # Overview spans all 3 columns of row 0
    ax_overview = fig.add_subplot(gs[0, :], projection="3d")
    ax_overview.set_facecolor("#F9F9F9")
    _draw_overview(ax_overview)

    # 6 detail subplots (rows 1–2, 3 cols each)
    orient_list = list(GRASP_ORIENTATIONS.items())
    for idx, (name, info) in enumerate(orient_list):
        row = 1 + idx // 3
        col = idx % 3
        ax = fig.add_subplot(gs[row, col], projection="3d")
        ax.set_facecolor("#FAFAFA")
        _draw_detail(ax, info)

    _frame_axis_legend(fig)

    out = args.out
    plt.savefig(out, dpi=args.dpi, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    print(f"[plot] Saved → {out}")

    for viewer in ("eog", "feh", "display", "xdg-open"):
        if shutil.which(viewer):
            subprocess.Popen([viewer, out],
                             stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
            print(f"[plot] Opened with {viewer}")
            break


if __name__ == "__main__":
    main()
