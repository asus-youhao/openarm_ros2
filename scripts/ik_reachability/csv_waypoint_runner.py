#!/usr/bin/env python3
"""
CSV Waypoint Runner — MoveIt2 waypoint executor
================================================
Reads reachable points from a CSV produced by ik_reachability_sampler.py
and executes them sequentially using pymoveit2.

Prerequisites (must be running):
    ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
    ros2 launch openarm_bimanual_moveit_config move_group_only.launch.py

Usage examples:
    # Run all reachable points on the right arm
    python3 csv_waypoint_runner.py --csv right_reachability.csv --arm right

    # Left arm, dwell 0.5s per point, return home after finishing
    python3 csv_waypoint_runner.py --csv left_reachability.csv --arm left --dwell 0.5 --home

    # Run only the first 10 points as a quick test
    python3 csv_waypoint_runner.py --csv right_reachability.csv --arm right --max-pts 10

    # Override EE orientation (right arm natural approach = -Y)
    python3 csv_waypoint_runner.py --csv right_reachability.csv --arm right \
        --quat 0.0 0.0 -0.707 0.707

    # Bimanual alternating execution (left CSV + right CSV)
    python3 csv_waypoint_runner.py \
        --left  left_reachability.csv  \
        --right right_reachability.csv \
        --dwell 1.0 --home

Options:
    --csv       CSV path (single-arm mode)
    --arm       left | right (single-arm mode)
    --left      Left-arm CSV (bimanual mode)
    --right     Right-arm CSV (bimanual mode)
    --quat      EE quaternion qx qy qz qw (default: identity)
    --dwell     Dwell time after reaching each point (default: 0.3s)
    --max-pts   Maximum number of points to execute (default: all)
    --home      Return to home after completing run
    --home-first Return home before starting
    --cartesian Use Cartesian planning (default: joint-space)
    --timeout   Per-point planning timeout in seconds (default: 5.0)
    --no-sort   Do not sort by Z (default: sort low-to-high)
"""

import argparse
import csv
import sys
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


# ---------------------------------------------------------------------------
# Per-arm configuration
# ---------------------------------------------------------------------------
_ARM_CONFIG = {
    "left": {
        "joint_names": [f"openarm_left_joint{i}" for i in range(1, 8)],
        "base_link":   "world",
        "ee_link":     "openarm_left_link7",
        "group_name":  "left_arm",
        "home_joints": [0.0] * 7,
        # Natural approach direction: left arm base roll=+90° -> EE faces +Y
        "default_quat": (0.0, 0.0, 0.707, 0.707),  # qx qy qz qw
    },
    "right": {
        "joint_names": [f"openarm_right_joint{i}" for i in range(1, 8)],
        "base_link":   "world",
        "ee_link":     "openarm_right_link7",
        "group_name":  "right_arm",
        "home_joints": [0.0] * 7,
        # Natural approach direction: right arm base roll=-90° -> EE faces -Y
        "default_quat": (0.0, 0.0, -0.707, 0.707),  # qx qy qz qw
    },
}


# ---------------------------------------------------------------------------
# CSV loading
# ---------------------------------------------------------------------------
def load_reachable_pts(csv_path: str, sort_by_z: bool = True):
    """Load CSV and return only points with reachable=1. Optionally sort by Z."""
    pts = []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            if row["reachable"] == "1":
                pts.append((float(row["x"]), float(row["y"]), float(row["z"])))
    if sort_by_z:
        pts.sort(key=lambda p: p[2])  # sort low-to-high to avoid large jumps
    return pts


# ---------------------------------------------------------------------------
# MoveIt2 runner
# ---------------------------------------------------------------------------
class WaypointRunner:
    def __init__(self, arm: str, quat_override=None, timeout: float = 5.0,
                 cartesian: bool = False):
        self.arm = arm
        cfg = _ARM_CONFIG[arm]
        self.home_joints = cfg["home_joints"]
        self.cartesian = cartesian
        self.timeout = timeout

        qx, qy, qz, qw = quat_override if quat_override else cfg["default_quat"]
        self.quat_xyzw = [qx, qy, qz, qw]

        self.node = Node(f"csv_waypoint_runner_{arm}")
        cb_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self.node,
            joint_names=cfg["joint_names"],
            base_link_name=cfg["base_link"],
            end_effector_name=cfg["ee_link"],
            group_name=cfg["group_name"],
            callback_group=cb_group,
        )
        self.moveit2.max_velocity = 0.3       # safety speed limit
        self.moveit2.max_acceleration = 0.3
        self.moveit2.planning_time = timeout

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

        self.node.get_logger().info(
            f"[{arm}] WaypointRunner ready. EE={cfg['ee_link']}  "
            f"quat={self.quat_xyzw}  cartesian={cartesian}"
        )

    def go_home(self):
        self.node.get_logger().info(f"[{self.arm}] Going home...")
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

    def move_to(self, x, y, z) -> bool:
        """Plan and execute a move to (x,y,z). Returns True on success."""
        pos = Point(x=x, y=y, z=z)
        self.moveit2.move_to_pose(
            position=pos,
            quat_xyzw=self.quat_xyzw,
            cartesian=self.cartesian,
        )
        self.moveit2.wait_until_executed()
        # pymoveit2 does not return success/failure directly; use get_result or assume success
        return True

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()


# ---------------------------------------------------------------------------
# Single-arm execution
# ---------------------------------------------------------------------------
def run_arm(runner: WaypointRunner, pts: list, max_pts: int,
            dwell: float, go_home_after: bool, home_first: bool):
    arm = runner.arm
    log = runner.node.get_logger()

    if home_first:
        runner.go_home()
        time.sleep(1.0)

    total = min(len(pts), max_pts) if max_pts > 0 else len(pts)
    log.info(f"[{arm}] Starting waypoint run: {total} points")

    success = 0
    fail = 0
    t0 = time.time()

    for i, (x, y, z) in enumerate(pts[:total]):
        log.info(f"[{arm}] [{i+1}/{total}]  -> ({x:.3f}, {y:.3f}, {z:.3f})")
        ok = runner.move_to(x, y, z)
        if ok:
            success += 1
            log.info(f"[{arm}]   REACHED ({success} ok / {fail} fail)")
            time.sleep(dwell)
        else:
            fail += 1
            log.warning(f"[{arm}]   FAILED  ({success} ok / {fail} fail)")

    elapsed = time.time() - t0
    log.info(
        f"\n[{arm}] Done. {success}/{total} succeeded, {fail} failed "
        f"in {elapsed:.1f}s ({elapsed/total:.1f}s/pt)"
    )

    if go_home_after:
        runner.go_home()

    return success, fail


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="CSV Waypoint Runner — MoveIt2")
    # Single-arm mode
    parser.add_argument("--csv",   default=None, help="Reachable CSV path (single-arm)")
    parser.add_argument("--arm",   default="right", choices=["left", "right"],
                        help="Single-arm mode: left | right (default: right)")
    # Bimanual mode
    parser.add_argument("--left",  default=None, metavar="CSV", help="Left-arm CSV (bimanual mode)")
    parser.add_argument("--right", default=None, metavar="CSV", help="Right-arm CSV (bimanual mode)")
    # Action parameters
    parser.add_argument("--quat",  type=float, nargs=4, default=None,
                        metavar=("QX", "QY", "QZ", "QW"),
                        help="EE quaternion (override default)")
    parser.add_argument("--dwell",      type=float, default=0.3,
                        help="Dwell time after reaching a point (default: 0.3s)")
    parser.add_argument("--max-pts",    type=int,   default=0,
                        help="Maximum number of points to execute (0=all)")
    parser.add_argument("--home",       action="store_true",
                        help="Return to home after finishing")
    parser.add_argument("--home-first", action="store_true",
                        help="Return home before starting")
    parser.add_argument("--cartesian",  action="store_true",
                        help="Use Cartesian planning (default: joint-space)")
    parser.add_argument("--timeout",    type=float, default=5.0,
                        help="Per-point planning timeout in seconds (default: 5.0)")
    parser.add_argument("--no-sort",    action="store_true",
                        help="Do not sort by Z (default: sort low-to-high)")
    args, ros_args = parser.parse_known_args()

    sort_by_z = not args.no_sort

    # Determine mode
    bimanual = args.left is not None and args.right is not None
    if not bimanual and args.csv is None:
        print("[ERROR] Please specify --csv (single-arm) or --left + --right (bimanual)")
        sys.exit(1)

    rclpy.init(args=ros_args)

    print("Waiting for MoveIt to initialize (3s)...")
    time.sleep(3.0)

    try:
        if bimanual:
            # ---- Bimanual mode: alternating execution ----
            left_pts  = load_reachable_pts(args.left,  sort_by_z=sort_by_z)
            right_pts = load_reachable_pts(args.right, sort_by_z=sort_by_z)
            max_pts = args.max_pts if args.max_pts > 0 else max(len(left_pts), len(right_pts))

            print(f"Bimanual mode: left={len(left_pts)} pts, right={len(right_pts)} pts")

            left_runner  = WaypointRunner("left",  quat_override=args.quat,
                                          timeout=args.timeout, cartesian=args.cartesian)
            right_runner = WaypointRunner("right", quat_override=args.quat,
                                          timeout=args.timeout, cartesian=args.cartesian)

            if args.home_first:
                left_runner.go_home()
                right_runner.go_home()
                time.sleep(1.0)

            L = min(len(left_pts), max_pts)
            R = min(len(right_pts), max_pts)
            total = max(L, R)

            l_ok = l_fail = r_ok = r_fail = 0
            for i in range(total):
                if i < L:
                    x, y, z = left_pts[i]
                    print(f"[LEFT  {i+1}/{L}] -> ({x:.3f},{y:.3f},{z:.3f})")
                    if left_runner.move_to(x, y, z):
                        l_ok += 1
                    else:
                        l_fail += 1
                if i < R:
                    x, y, z = right_pts[i]
                    print(f"[RIGHT {i+1}/{R}] -> ({x:.3f},{y:.3f},{z:.3f})")
                    if right_runner.move_to(x, y, z):
                        r_ok += 1
                    else:
                        r_fail += 1
                time.sleep(args.dwell)

            print(f"\nDone. Left: {l_ok} ok / {l_fail} fail   Right: {r_ok} ok / {r_fail} fail")

            if args.home:
                left_runner.go_home()
                right_runner.go_home()

            left_runner.shutdown()
            right_runner.shutdown()

        else:
            # ---- Single-arm mode ----
            csv_path = args.csv
            if not Path(csv_path).exists():
                print(f"[ERROR] File not found: {csv_path}")
                sys.exit(1)

            pts = load_reachable_pts(csv_path, sort_by_z=sort_by_z)
            if not pts:
                print("[ERROR] CSV has no reachable points (reachable=1)")
                sys.exit(1)

            print(f"[{args.arm}] Loaded {len(pts)} reachable waypoints from {csv_path}")

            runner = WaypointRunner(args.arm, quat_override=args.quat,
                                    timeout=args.timeout, cartesian=args.cartesian)
            run_arm(runner, pts, args.max_pts, args.dwell,
                    go_home_after=args.home, home_first=args.home_first)
            runner.shutdown()

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
