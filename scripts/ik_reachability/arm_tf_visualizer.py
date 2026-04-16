#!/usr/bin/env python3
"""
Arm TF Chain Visualizer

Reads TF transforms from /tf and /tf_static, then prints and plots the
position of each arm link (base → link1 → ... → link7) for both arms.

Prerequisites (must be running):
  ros2 launch openarm_bringup openarm_o6_bimanual.launch.py

Usage:
  python3 arm_tf_visualizer.py
  python3 arm_tf_visualizer.py --side left     # only left arm
  python3 arm_tf_visualizer.py --side right    # only right arm
  python3 arm_tf_visualizer.py --no-plot       # text only, no matplotlib
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


# ---------------------------------------------------------------------------
# Link chain definitions (matches URDF / SRDF)
# ---------------------------------------------------------------------------
BASE_FRAME = "world"

CHAIN = {
    "left": [
        "world",
        "openarm_body_link0",
        "openarm_left_link0",
        "openarm_left_link1",
        "openarm_left_link2",
        "openarm_left_link3",
        "openarm_left_link4",
        "openarm_left_link5",
        "openarm_left_link6",
        "openarm_left_link7",   # EE (joint7 output)
    ],
    "right": [
        "world",
        "openarm_body_link0",
        "openarm_right_link0",
        "openarm_right_link1",
        "openarm_right_link2",
        "openarm_right_link3",
        "openarm_right_link4",
        "openarm_right_link5",
        "openarm_right_link6",
        "openarm_right_link7",  # EE (joint7 output)
    ],
}

JOINT_LABEL = [
    "base_body", "shoulder_base",
    "joint1", "joint2", "joint3", "joint4",
    "joint5", "joint6", "joint7 (EE)",
]


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------
class TFArmVisualizer(Node):
    def __init__(self):
        super().__init__("arm_tf_visualizer")
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def lookup(self, parent: str, child: str, timeout_sec: float = 2.0):
        """Return (x, y, z, qx, qy, qz, qw) or None on failure."""
        try:
            tf = self.tf_buffer.lookup_transform(
                parent,
                child,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=timeout_sec),
            )
            t = tf.transform.translation
            r = tf.transform.rotation
            return (t.x, t.y, t.z, r.x, r.y, r.z, r.w)
        except Exception as e:
            self.get_logger().warn(f"TF {parent} -> {child}: {e}")
            return None

    def get_chain_poses(self, side: str):
        """Return list of (label, x, y, z) in world frame for the given side."""
        chain = CHAIN[side]
        results = []
        for i, link in enumerate(chain):
            if link == BASE_FRAME:
                results.append((JOINT_LABEL[0] if i == 0 else "world", 0.0, 0.0, 0.0))
                continue
            pose = self.lookup(BASE_FRAME, link)
            label = JOINT_LABEL[i] if i < len(JOINT_LABEL) else link
            if pose is not None:
                results.append((label, pose[0], pose[1], pose[2]))
            else:
                results.append((label, None, None, None))
        return results


# ---------------------------------------------------------------------------
# Text printout
# ---------------------------------------------------------------------------
def print_chain(side: str, poses):
    print(f"\n=== {side.upper()} ARM chain (world frame) ===")
    print(f"  {'Label':<20} {'X (m)':>9} {'Y (m)':>9} {'Z (m)':>9}")
    print(f"  {'-'*20} {'-'*9} {'-'*9} {'-'*9}")
    for label, x, y, z in poses:
        if x is None:
            print(f"  {label:<20} {'N/A':>9} {'N/A':>9} {'N/A':>9}")
        else:
            print(f"  {label:<20} {x:>9.4f} {y:>9.4f} {z:>9.4f}")


# ---------------------------------------------------------------------------
# Matplotlib 3D plot
# ---------------------------------------------------------------------------
def plot_chains(left_poses, right_poses):
    try:
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError:
        print("[WARN] matplotlib not installed, skipping plot.")
        return

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    def draw_chain(poses, color, label_prefix):
        xs, ys, zs, labels = [], [], [], []
        for lbl, x, y, z in poses:
            if x is not None:
                xs.append(x); ys.append(y); zs.append(z)
                labels.append(lbl)
        if not xs:
            return
        # Draw arm links as line
        ax.plot(xs, ys, zs, "-o", color=color, linewidth=2, markersize=5,
                label=f"{label_prefix} arm")
        # Annotate joints
        for x, y, z, lbl in zip(xs, ys, zs, labels):
            ax.text(x, y, z, f"  {lbl}", fontsize=7, color=color)
        # Mark EE with larger star
        ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], s=120, marker="*",
                   color=color, zorder=5)

    if left_poses:
        draw_chain(left_poses, color="#2196F3", label_prefix="Left")
    if right_poses:
        draw_chain(right_poses, color="#E91E63", label_prefix="Right")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)
    ax.set_title("OpenArm V10 – Arm TF Chain (world frame)\nEE = link7  [blue=left, pink=right]")
    ax.legend()
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Arm TF Chain Visualizer")
    parser.add_argument("--side", choices=["left", "right", "both"], default="both")
    parser.add_argument("--no-plot", action="store_true", help="Print only, skip matplotlib")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = TFArmVisualizer()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    # Spin briefly to collect TF data
    print("Waiting for TF data (3s)...")
    deadline = time.time() + 3.0
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.1)

    sides = ["left", "right"] if args.side == "both" else [args.side]

    left_poses = right_poses = None
    for side in sides:
        poses = node.get_chain_poses(side)
        print_chain(side, poses)
        if side == "left":
            left_poses = poses
        else:
            right_poses = poses

    if not args.no_plot:
        plot_chains(left_poses, right_poses)

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
