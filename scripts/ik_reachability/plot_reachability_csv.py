#!/usr/bin/env python3
"""
Reachability CSV Plotter and Bimanual Overlap Analyzer
======================================================
Offline analysis tool (does not require ROS or MoveIt by default).

Features:
    1. Single-arm visualization (left or right)
    2. Combined bimanual visualization
    3. Bimanual overlap detection (--overlap) — points reachable by both arms
    4. Overlay robot TF arm chains (--live-tf or --tf-file)

Usage examples:
    python3 plot_reachability_csv.py                          # Combined bimanual view
    python3 plot_reachability_csv.py --overlap                # Highlight overlap with green ★
    python3 plot_reachability_csv.py --arm left               # Show left arm only
    python3 plot_reachability_csv.py --arm right              # Show right arm only
    python3 plot_reachability_csv.py --live-tf                # Read TF live from ROS (requires hardware running)
    python3 plot_reachability_csv.py --tf-file tf_chains.json # Load saved TF chains offline
    python3 plot_reachability_csv.py --no-unreachable         # Hide unreachable (gray) points
    python3 plot_reachability_csv.py --stats                  # Print stats only, skip plotting

Recommended TF workflow:
    1. When hardware is running, save TF chains: python3 plot_reachability_csv.py --live-tf --no-plot
         This will create a tf_chains.json file for offline use.
    2. Plot offline using the saved TF: python3 plot_reachability_csv.py --tf-file tf_chains.json

Color convention:
    left  -> blue  (#2196F3)
    right -> red   (#E91E63)
    overlap -> green (#4CAF50)
"""

import argparse
import csv
import json
import sys
import time
from pathlib import Path
import os
from datetime import datetime


# Color palette (matches ik_reachability_sampler.py)
_COLOR = {
    "left":    "#2196F3",   # blue
    "right":   "#E91E63",   # red
    "overlap": "#4CAF50",   # green
    "gray":    "#BDBDBD",   # unreachable
}


# ---------------------------------------------------------------------------
# Load CSV
# ---------------------------------------------------------------------------
def load_csv(path: str):
    """Return (reachable_list, unreachable_list), each as (x, y, z)."""
    reachable, unreachable = [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            pt = (float(row["x"]), float(row["y"]), float(row["z"]))
            (reachable if row["reachable"] == "1" else unreachable).append(pt)
    return reachable, unreachable


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------
def print_stats(arm: str, reachable, unreachable):
    total = len(reachable) + len(unreachable)
    pct = 100 * len(reachable) / total if total else 0
    print(f"\n{'='*50}")
    print(f"  [{arm.upper()} ARM]  reachable: {len(reachable)} / {total}  ({pct:.1f}%)")
    if reachable:
        xs = [p[0] for p in reachable]
        ys = [p[1] for p in reachable]
        zs = [p[2] for p in reachable]
        print(f"  X range: [{min(xs):.3f}, {max(xs):.3f}]")
        print(f"  Y range: [{min(ys):.3f}, {max(ys):.3f}]")
        print(f"  Z range: [{min(zs):.3f}, {max(zs):.3f}]")
    print(f"{'='*50}")


def find_overlap(left_reach, right_reach, tol: float = 0.01):
    """Find points reachable by both arms (distance < tol considered the same voxel)."""
    right_set = {(round(x / tol), round(y / tol), round(z / tol))
                 for x, y, z in right_reach}
    overlap = []
    for x, y, z in left_reach:
        key = (round(x / tol), round(y / tol), round(z / tol))
        if key in right_set:
            overlap.append((x, y, z))
    return overlap


def print_overlap_stats(overlap, left_reach, right_reach):
    print(f"\n{'='*50}")
    print(f"  [BIMANUAL OVERLAP]")
    print(f"  Left  reachable: {len(left_reach)} pts")
    print(f"  Right reachable: {len(right_reach)} pts")
    print(f"  Overlap (both):  {len(overlap)} pts")
    if left_reach:
        print(f"  Overlap / Left:  {100*len(overlap)/len(left_reach):.1f}%")
    if right_reach:
        print(f"  Overlap / Right: {100*len(overlap)/len(right_reach):.1f}%")
    if overlap:
        xs = [p[0] for p in overlap]
        ys = [p[1] for p in overlap]
        zs = [p[2] for p in overlap]
        print(f"  Overlap X: [{min(xs):.3f}, {max(xs):.3f}]")
        print(f"  Overlap Y: [{min(ys):.3f}, {max(ys):.3f}]")
        print(f"  Overlap Z: [{min(zs):.3f}, {max(zs):.3f}]")
    print(f"{'='*50}\n")


# ---------------------------------------------------------------------------
# TF chain - read live from ROS (requires rclpy) or load from JSON
# ---------------------------------------------------------------------------
_TF_LINKS = {
    "left": [
        "openarm_body_link0",
        "openarm_left_link0",
        "openarm_left_link1",
        "openarm_left_link2",
        "openarm_left_link3",
        "openarm_left_link4",
        "openarm_left_link5",
        "openarm_left_link6",
        "openarm_left_link7",
    ],
    "right": [
        "openarm_body_link0",
        "openarm_right_link0",
        "openarm_right_link1",
        "openarm_right_link2",
        "openarm_right_link3",
        "openarm_right_link4",
        "openarm_right_link5",
        "openarm_right_link6",
        "openarm_right_link7",
    ],
}
_TF_JOINT_LABELS = ["world", "body", "link0", "J1", "J2", "J3", "J4", "J5", "J6", "EE"]


def fetch_tf_chains_live(arms, save_path: str = "tf_chains.json") -> dict:
    """Fetch TF chains for each arm from ROS and save to JSON.
    Returns: {arm: [(x,y,z), ...]}"""
    try:
        import rclpy
        from rclpy.node import Node
        from tf2_ros import Buffer, TransformListener
    except ImportError:
        print("[ERROR] rclpy not found. Cannot use --live-tf.")
        return {}

    rclpy.init()
    node = Node("tf_chain_reader")
    tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
    _ = TransformListener(tf_buffer, node)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    print("Reading TF data (3s)...")
    deadline = time.time() + 3.0
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.1)

    chains = {}
    for arm in arms:
        pts = [(0.0, 0.0, 0.0)]  # world origin
        for link in _TF_LINKS[arm]:
            try:
                tf = tf_buffer.lookup_transform(
                    "world", link,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=1.0),
                )
                t = tf.transform.translation
                pts.append((t.x, t.y, t.z))
            except Exception as e:
                print(f"  [WARN] TF world->{link}: {e}")
        chains[arm] = pts
        print(f"  {arm}: {len(pts)} links resolved  (EE = {pts[-1]})")

    node.destroy_node()
    rclpy.shutdown()

    # Save JSON for offline use
    with open(save_path, "w") as f:
        json.dump(chains, f, indent=2)
    print(f"TF chains saved -> {save_path}")
    return chains


def load_tf_chains_json(path: str) -> dict:
    """Load TF chains JSON and return {arm: [(x,y,z), ...]}."""
    with open(path) as f:
        raw = json.load(f)
    # Convert JSON list-of-list into list-of-tuple
    return {arm: [tuple(pt) for pt in pts] for arm, pts in raw.items()}


# ---------------------------------------------------------------------------
# Plot helpers
# ---------------------------------------------------------------------------
_CHAIN_LABELS = _TF_JOINT_LABELS


def _scatter(ax, pts, color, label, alpha=0.8, marker="o", size=40):
    if not pts:
        return
    xs, ys, zs = zip(*pts)
    ax.scatter(xs, ys, zs, s=size, c=color, alpha=alpha,
               label=label, depthshade=True, marker=marker)


def _draw_chain(ax, chain_pts, color, arm):
    """Draw a TF chain on a 3D axes (helper)."""
    if not chain_pts or len(chain_pts) < 2:
        return
    cx = [p[0] for p in chain_pts]
    cy = [p[1] for p in chain_pts]
    cz = [p[2] for p in chain_pts]
    ax.plot(cx, cy, cz, "-o", color=color, linewidth=3,
            markersize=6, label=f"{arm} TF chain", zorder=10)
    for i, (x, y, z) in enumerate(chain_pts):
        lbl = _CHAIN_LABELS[i] if i < len(_CHAIN_LABELS) else str(i)
        ax.text(x, y, z, f"  {lbl}", fontsize=7, color=color)
    # Mark EE with a large star
    ax.scatter([cx[-1]], [cy[-1]], [cz[-1]], s=200, marker="*",
               color=color, zorder=11, label=f"{arm} EE (TF)")


def plot_single(arm: str, reachable, unreachable, show_unreachable: bool, chain_pts=None, save_dir: str = None):
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    if show_unreachable and unreachable:
        _scatter(ax, unreachable, _COLOR["gray"], "unreachable", alpha=0.06, size=10)
    _scatter(ax, reachable, _COLOR[arm], f"{arm} reachable ({len(reachable)})")
    _draw_chain(ax, chain_pts, _COLOR[arm], arm)

    _set_axes(ax)
    color_name = "blue" if arm == "left" else "red"
    ax.set_title(f"OpenArm V10 — {arm.capitalize()} Arm EE Reachability [{color_name}]\n"
                 f"{len(reachable)} / {len(reachable)+len(unreachable)} reachable")
    ax.legend()
    # Save 3D and 2D projections if requested
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        fig.savefig(Path(save_dir) / f"{arm}_3d.png", dpi=150)
        # save 2D projections as well
        try:
            _save_projections_single(save_dir, arm, reachable, unreachable, show_unreachable)
        except Exception as e:
            print(f"[WARN] failed to save projections: {e}")

    plt.tight_layout()
    plt.show()


def _save_projections_single(save_dir: str, arm: str, reachable, unreachable, show_unreachable: bool):
    """Save 2D projection images: X-Y, X-Z, Y-Z for a single arm."""
    os.makedirs(save_dir, exist_ok=True)
    # XY
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6, 5))
    if show_unreachable and unreachable:
        ux, uy, _ = zip(*unreachable)
        ax.scatter(ux, uy, c=_COLOR["gray"], alpha=0.06, s=8)
    if reachable:
        rx, ry, _ = zip(*reachable)
        ax.scatter(rx, ry, c=_COLOR[arm], alpha=0.8, s=10)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"{arm} XY")
    fig.tight_layout()
    fig.savefig(Path(save_dir) / f"{arm}_xy.png", dpi=150)
    plt.close(fig)

    # XZ
    fig, ax = plt.subplots(figsize=(6, 5))
    if show_unreachable and unreachable:
        ux, _, uz = zip(*unreachable)
        ax.scatter(ux, uz, c=_COLOR["gray"], alpha=0.06, s=8)
    if reachable:
        rx, _, rz = zip(*reachable)
        ax.scatter(rx, rz, c=_COLOR[arm], alpha=0.8, s=10)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Z (m)")
    ax.set_title(f"{arm} XZ")
    fig.tight_layout()
    fig.savefig(Path(save_dir) / f"{arm}_xz.png", dpi=150)
    plt.close(fig)

    # YZ
    fig, ax = plt.subplots(figsize=(6, 5))
    if show_unreachable and unreachable:
        _, uy, uz = zip(*unreachable)
        ax.scatter(uy, uz, c=_COLOR["gray"], alpha=0.06, s=8)
    if reachable:
        _, ry, rz = zip(*reachable)
        ax.scatter(ry, rz, c=_COLOR[arm], alpha=0.8, s=10)
    ax.set_xlabel("Y (m)")
    ax.set_ylabel("Z (m)")
    ax.set_title(f"{arm} YZ")
    fig.tight_layout()
    fig.savefig(Path(save_dir) / f"{arm}_yz.png", dpi=150)
    plt.close(fig)


def plot_both(left_r, left_u, right_r, right_u, overlap,
              show_unreachable: bool, show_overlap_only: bool,
              tf_chains: dict = None, save_dir: str = None):
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(13, 9))
    ax = fig.add_subplot(111, projection="3d")

    if show_unreachable and not show_overlap_only:
        all_u = left_u + right_u
        if all_u:
            _scatter(ax, all_u, _COLOR["gray"], None, alpha=0.04, size=8)

    if not show_overlap_only:
        _scatter(ax, left_r,  _COLOR["left"],  f"left  reachable ({len(left_r)})")
        _scatter(ax, right_r, _COLOR["right"], f"right reachable ({len(right_r)})")

    if overlap:
        _scatter(ax, overlap, _COLOR["overlap"],
                 f"OVERLAP — both arms ({len(overlap)})",
                 alpha=1.0, marker="*", size=120)

    if tf_chains:
        for arm, pts in tf_chains.items():
            _draw_chain(ax, pts, _COLOR[arm], arm)

    _set_axes(ax)
    title_suffix = " — OVERLAP ONLY" if show_overlap_only else ""
    ax.set_title(
        f"OpenArm V10 — Bimanual EE Reachability{title_suffix}\n"
        f"left={len(left_r)} (blue)  right={len(right_r)} (red)  "
        f"overlap={len(overlap)} (green★)"
    )
    ax.legend()
    # Save images (3D + 2D projections) if requested
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        fig.savefig(Path(save_dir) / "bimanual_3d.png", dpi=150)
        # Save 2D projections
        try:
            # XY
            import matplotlib.pyplot as plt
            fig2, ax2 = plt.subplots(figsize=(7, 6))
            if show_unreachable and not show_overlap_only:
                all_u = left_u + right_u
                if all_u:
                    ux, uy, _ = zip(*all_u)
                    ax2.scatter(ux, uy, s=8, c=_COLOR["gray"], alpha=0.04)
            if not show_overlap_only:
                if left_r:
                    lx, ly, _ = zip(*left_r); ax2.scatter(lx, ly, c=_COLOR["left"], s=10, alpha=0.7)
                if right_r:
                    rx, ry, _ = zip(*right_r); ax2.scatter(rx, ry, c=_COLOR["right"], s=10, alpha=0.7)
            if overlap:
                ox, oy, _ = zip(*overlap); ax2.scatter(ox, oy, c=_COLOR["overlap"], s=80, marker="*")
            ax2.set_xlabel("X (m)"); ax2.set_ylabel("Y (m)"); ax2.set_title("Bimanual XY")
            fig2.tight_layout(); fig2.savefig(Path(save_dir) / "bimanual_xy.png", dpi=150); plt.close(fig2)

            # XZ
            fig3, ax3 = plt.subplots(figsize=(7, 6))
            if show_unreachable and not show_overlap_only:
                all_u = left_u + right_u
                if all_u:
                    ux, _, uz = zip(*all_u); ax3.scatter(ux, uz, s=6, c=_COLOR["gray"], alpha=0.04)
            if not show_overlap_only:
                if left_r:
                    lx, _, lz = zip(*left_r); ax3.scatter(lx, lz, c=_COLOR["left"], s=10, alpha=0.7)
                if right_r:
                    rx, _, rz = zip(*right_r); ax3.scatter(rx, rz, c=_COLOR["right"], s=10, alpha=0.7)
            if overlap:
                ox, _, oz = zip(*overlap); ax3.scatter(ox, oz, c=_COLOR["overlap"], s=80, marker="*")
            ax3.set_xlabel("X (m)"); ax3.set_ylabel("Z (m)"); ax3.set_title("Bimanual XZ")
            fig3.tight_layout(); fig3.savefig(Path(save_dir) / "bimanual_xz.png", dpi=150); plt.close(fig3)

            # YZ
            fig4, ax4 = plt.subplots(figsize=(7, 6))
            if show_unreachable and not show_overlap_only:
                all_u = left_u + right_u
                if all_u:
                    _, uy, uz = zip(*all_u); ax4.scatter(uy, uz, s=6, c=_COLOR["gray"], alpha=0.04)
            if not show_overlap_only:
                if left_r:
                    _, ly, lz = zip(*left_r); ax4.scatter(ly, lz, c=_COLOR["left"], s=10, alpha=0.7)
                if right_r:
                    _, ry, rz = zip(*right_r); ax4.scatter(ry, rz, c=_COLOR["right"], s=10, alpha=0.7)
            if overlap:
                _, oy, oz = zip(*overlap); ax4.scatter(oy, oz, c=_COLOR["overlap"], s=80, marker="*")
            ax4.set_xlabel("Y (m)"); ax4.set_ylabel("Z (m)"); ax4.set_title("Bimanual YZ")
            fig4.tight_layout(); fig4.savefig(Path(save_dir) / "bimanual_yz.png", dpi=150); plt.close(fig4)

        except Exception as e:
            print(f"[WARN] failed to save bimanual projections: {e}")

    plt.tight_layout()
    plt.show()


def _set_axes(ax):
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Reachability CSV Plotter & Bimanual Overlap Analyser (offline, no ROS needed)"
    )
    parser.add_argument("--arm",    choices=["left", "right", "both"], default="both")
    parser.add_argument("--left",   default="left_reachability.csv",  metavar="CSV")
    parser.add_argument("--right",  default="right_reachability.csv", metavar="CSV")
    parser.add_argument("--overlap",        action="store_true", help="Highlight bimanual overlap region")
    parser.add_argument("--overlap-only",   action="store_true", help="Show only overlapping points")
    parser.add_argument("--no-unreachable", action="store_true", help="Hide unreachable grey points")
    parser.add_argument("--no-plot",        action="store_true", help="Only print statistics, no plot")
    parser.add_argument("--stats",          action="store_true", help="Same as --no-plot")
    parser.add_argument("--tol",    type=float, default=0.01,
                        help="Distance tolerance for overlap matching in m (default 0.01)")
    # TF chain options
    tf_group = parser.add_mutually_exclusive_group()
    tf_group.add_argument("--live-tf",  action="store_true",
                          help="Read TF chains live from ROS (requires hardware running)")
    tf_group.add_argument("--tf-file",  default=None, metavar="JSON",
                          help="Load saved TF JSON for offline use (e.g., tf_chains.json)")
    parser.add_argument("--tf-save",  default="tf_chains.json", metavar="JSON",
                        help="Path to save TF chains when using --live-tf (default: tf_chains.json)")
    parser.add_argument("--save", action="store_true", help="Save plots to timestamped folder under --save-root")
    parser.add_argument("--save-root", default="./img", metavar="DIR",
                        help="Root folder to save images (default: ./img)")
    args = parser.parse_args()

    no_plot = args.no_plot or args.stats
    show_unreachable = not args.no_unreachable
    arms = ["left", "right"] if args.arm == "both" else [args.arm]

    # --- TF chain loading ---
    tf_chains = {}
    if args.live_tf:
        tf_chains = fetch_tf_chains_live(arms, save_path=args.tf_save)
        if tf_chains:
            print(f"[TF] Loaded live TF for: {list(tf_chains.keys())}")
    elif args.tf_file:
        if not Path(args.tf_file).exists():
            print(f"[ERROR] TF file not found: {args.tf_file}")
            sys.exit(1)
        tf_chains = load_tf_chains_json(args.tf_file)
        # Keep only arms needed for this run
        tf_chains = {a: v for a, v in tf_chains.items() if a in arms}
        print(f"[TF] Loaded from {args.tf_file}: {list(tf_chains.keys())}")

    # Determine save directory if requested (used for both single-arm and both)
    save_dir = None
    if args.save:
        ts = datetime.now().strftime("%y%m%d%H%M")
        save_dir = os.path.join(args.save_root, ts)

    # --- Single arm mode ---
    if args.arm != "both":
        csv_path = args.left if args.arm == "left" else args.right
        if not Path(csv_path).exists():
            print(f"[ERROR] File not found: {csv_path}")
            sys.exit(1)
        reachable, unreachable = load_csv(csv_path)
        print_stats(args.arm, reachable, unreachable)
        if not no_plot:
            plot_single(args.arm, reachable, unreachable, show_unreachable,
                        chain_pts=tf_chains.get(args.arm), save_dir=save_dir)
        return

    # --- Both arms ---
    for path in [args.left, args.right]:
        if not Path(path).exists():
            print(f"[ERROR] File not found: {path}")
            sys.exit(1)

    left_r,  left_u  = load_csv(args.left)
    right_r, right_u = load_csv(args.right)

    print_stats("left",  left_r,  left_u)
    print_stats("right", right_r, right_u)

    overlap = find_overlap(left_r, right_r, tol=args.tol)
    print_overlap_stats(overlap, left_r, right_r)

    if not no_plot:
        plot_both(left_r, left_u, right_r, right_u,
                  overlap if (args.overlap or args.overlap_only) else [],
                  show_unreachable=show_unreachable,
                  show_overlap_only=args.overlap_only,
                  tf_chains=tf_chains if tf_chains else None,
                  save_dir=save_dir)


if __name__ == "__main__":
    main()
