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

Timing output (auto-generated unless --no-timing):
    <arm>_waypoint_timing.csv   per-point planning_ms / execution_ms / total_ms
    <arm>_waypoint_timing.png   4-panel plot: histogram · timeline · 3-D heatmap · CDF
"""

import argparse
import csv
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # non-interactive backend; works without display
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


# ---------------------------------------------------------------------------
# Real-time budget reference lines
# ---------------------------------------------------------------------------
# NOTE: These budgets are for MoveGroup OMPL planning (~100–1000 ms range).
# /compute_ik benchmarks use ik_timing_benchmark.py with 100/200/500 Hz lines.
# MoveGroup OMPL typical: 100–500 ms → realistic control rates are 1–10 Hz.
_RT_BUDGETS_MS = {
    "10 Hz":  1000.0 / 10.0,   # 100 ms  ← fast OMPL, e.g. Pilz or simple path
    "5 Hz":   1000.0 /  5.0,   # 200 ms  ← typical RRT-Connect good case
    "2 Hz":   1000.0 /  2.0,   # 500 ms  ← typical RRT-Connect median
    "1 Hz":   1000.0 /  1.0,   # 1000 ms ← slow / complex path
}
_RT_COLORS = {"10 Hz": "#7B1FA2", "5 Hz": "#E53935", "2 Hz": "#FB8C00", "1 Hz": "#43A047"}
# _RT_COLORS = {"60 Hz": "#E53935", "30 Hz": "#FB8C00", "10 Hz": "#43A047"}  # old
_ARM_COLOR = {"right": "#E91E63", "left": "#2196F3"}


# ---------------------------------------------------------------------------
# Named EE orientations (qx, qy, qz, qw)
# ---------------------------------------------------------------------------
GRASP_ORIENTATIONS = {
    "side_neg_y":   (0.0,    0.0,   -0.707,  0.707),  # side approach -Y (natural right)
    "side_pos_y":   (0.0,    0.0,    0.707,  0.707),  # side approach +Y (natural left)
    "top_down":     (0.0,    0.707,  0.0,    0.707),  # top-down grasp (EE -> -Z)
    "front_pos_x":  (0.0,    0.0,    0.0,    1.0),    # frontal approach +X
    "front_neg_x":  (0.0,    0.0,    1.0,    0.0),    # frontal approach -X
    "bottom_up":    (0.0,   -0.707,  0.0,    0.707),  # bottom-up (shelf pickup)
}


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
        "default_quat": GRASP_ORIENTATIONS["side_pos_y"],
    },
    "right": {
        "joint_names": [f"openarm_right_joint{i}" for i in range(1, 8)],
        "base_link":   "world",
        "ee_link":     "openarm_right_link7",
        "group_name":  "right_arm",
        "home_joints": [0.0] * 7,
        # Natural approach direction: right arm base roll=-90° -> EE faces -Y
        "default_quat": GRASP_ORIENTATIONS["side_neg_y"],
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
            use_move_group_action=True,  # single MoveGroup action: plan+exec atomic
            # use_move_group_action=False  # separate plan()+execute() via service
        )
        self.moveit2.max_velocity = 0.3       # safety speed limit
        self.moveit2.max_acceleration = 0.3
        self.moveit2.planning_time = timeout

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

        # Wait for MoveGroup move_action server (required for use_move_group_action=True)
        _move_client = self.moveit2._MoveIt2__move_action_client
        self.node.get_logger().info(f"[{arm}] Waiting for move_action server...")
        while not _move_client.server_is_ready():
            time.sleep(0.5)
        self.node.get_logger().info(f"[{arm}] move_action server ready.")

        self.node.get_logger().info(
            f"[{arm}] WaypointRunner ready. EE={cfg['ee_link']}  "
            f"quat={self.quat_xyzw}  cartesian={cartesian}"
        )

    def go_home(self):
        self.node.get_logger().info(f"[{self.arm}] Going home...")
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

    def move_to(self, x, y, z) -> tuple[bool, float, float, float]:
        """Plan and execute a move to (x,y,z).

        Returns (success, planning_ms, execution_ms, total_ms).
        planning_ms  = wall-clock time from move_to_pose() call until it returns
                       (covers IK + trajectory generation inside move_group)
        execution_ms = wall-clock time for wait_until_executed()
                       (covers robot motion)
        total_ms     = planning_ms + execution_ms
        """
        pos = Point(x=x, y=y, z=z)

        t0 = time.perf_counter()
        self.moveit2.move_to_pose(
            position=pos,
            quat_xyzw=self.quat_xyzw,
            cartesian=self.cartesian,
        )
        t_plan_done = time.perf_counter()
        self.moveit2.wait_until_executed()
        t_done = time.perf_counter()

        planning_ms  = (t_plan_done - t0) * 1000.0
        execution_ms = (t_done - t_plan_done) * 1000.0
        total_ms     = (t_done - t0) * 1000.0
        # pymoveit2 does not expose a success flag; treat completion as success
        return True, planning_ms, execution_ms, total_ms

    def plan_separate(self, x, y, z, quat_xyzw=None, do_execute: bool = True):
        """Split OMPL planning time from robot motion time.

        ── Why plan() + execute() + spin_once wait ─────────────────────────────
        use_move_group_action=True is needed for MoveGroup action, but we want
        to separately time OMPL planning (plan()) vs robot motion (execute()).

        ── spin_once issue ──────────────────────────────────────────────────────
        plan() calls rclpy.spin_once(node) internally (via plan_async's joint
        state wait loop AND plan()'s future wait loop). Each spin_once call uses
        the global SingleThreadedExecutor and may leave node.executor in an
        inconsistent state relative to our MultiThreadedExecutor.

        After plan() returns, callbacks for execute_trajectory arrive but may be
        processed by neither executor reliably.

        ── Fix ──────────────────────────────────────────────────────────────────
        Replace the pure time.sleep() polling loop with a rclpy.spin_once()-based
        wait that handles BOTH REQUESTING and EXECUTING states:

            while query_state() != IDLE:
                rclpy.spin_once(node, timeout_sec=0.05)

        This mirrors what wait_until_executed() does internally, but without its
        early-return bug (which returns False immediately when called in EXECUTING
        state because it checks `if not __is_motion_requested`).

        ── wait_until_executed() bug ────────────────────────────────────────────
        def wait_until_executed(self):
            if not self.__is_motion_requested:   ← False in EXECUTING state
                return False                     ← returns immediately!

        When the goal acceptance callback fires (REQUESTING→EXECUTING),
        __is_motion_requested becomes False.  If wait_until_executed() is called
        after this transition it returns False without waiting for motion to finish.

        Returns:
            (ok, ompl_ms, planned_duration_ms, motion_ms, total_ms,
             n_waypoints, joint_path_len)
        """
        from pymoveit2 import MoveIt2State

        quat = quat_xyzw if quat_xyzw is not None else self.quat_xyzw
        pos  = Point(x=x, y=y, z=z)

        js0 = self.moveit2.joint_state
        initial_joints = np.array(js0.position) if js0 is not None else None

        # ── Phase 1: Pure OMPL planning (synchronous, blocks via spin_once) ───
        t0 = time.perf_counter()
        trajectory = self.moveit2.plan(
            position=pos,
            quat_xyzw=quat,
            cartesian=self.cartesian,
        )
        t_plan_done = time.perf_counter()
        ompl_ms = (t_plan_done - t0) * 1000.0

        if trajectory is None:
            return False, ompl_ms, 0.0, 0.0, ompl_ms, 0, 0.0

        if not do_execute:
            return True, ompl_ms, 0.0, 0.0, ompl_ms, 0, 0.0

        # ── Phase 2: Execute trajectory (async) ──────────────────────────────
        self.moveit2.execute(trajectory)

        # ── Phase 3: Wait for IDLE using spin_once ───────────────────────────
        # After plan() called spin_once internally, MultiThreadedExecutor may be
        # unreliable for execute_trajectory callbacks.  Call spin_once here just
        # like wait_until_executed() does — but without its is_motion_requested
        # early-return bug.  We simply spin until query_state() returns IDLE.
        deadline_exec = time.perf_counter() + self.timeout + 60.0
        while True:
            state = self.moveit2.query_state()
            if state == MoveIt2State.IDLE:
                break
            if time.perf_counter() > deadline_exec:
                t_done = time.perf_counter()
                return (False, ompl_ms, 0.0,
                        (t_done - t_plan_done) * 1000.0,
                        (t_done - t0) * 1000.0, 0, 0.0)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        t_done = time.perf_counter()
        ok = self.moveit2.motion_suceeded

        motion_ms = (t_done - t_plan_done) * 1000.0
        total_ms  = (t_done - t0) * 1000.0

        joint_path_len = 0.0
        if initial_joints is not None:
            js_end = self.moveit2.joint_state
            if js_end is not None:
                joint_path_len = float(
                    np.sum(np.abs(np.array(js_end.position) - initial_joints))
                )

        return bool(ok), ompl_ms, 0.0, motion_ms, total_ms, 0, joint_path_len

    def try_orientations(self, x, y, z, orientations=None, do_execute: bool = True):
        """Try each EE orientation in turn; stop and return on the first success.

        Parameters
        ----------
        orientations : dict | None
            {name: (qx, qy, qz, qw)}.  Defaults to GRASP_ORIENTATIONS (all 6).

        Returns
        -------
        (success, orient_name, ompl_ms, motion_ms, total_ms, quat_list)
        """
        if orientations is None:
            orientations = GRASP_ORIENTATIONS

        for name, quat in orientations.items():
            self.node.get_logger().info(
                f"[{self.arm}] trying orient={name} quat={quat}"
            )
            ok, ompl_ms, _, motion_ms, total_ms, _, _ = self.plan_separate(
                x, y, z, quat_xyzw=list(quat), do_execute=do_execute
            )
            if ok:
                return True, name, ompl_ms, motion_ms, total_ms, list(quat)
            # Brief pause between orientation attempts
            time.sleep(0.1)

        return False, "", 0.0, 0.0, 0.0, [0.0, 0.0, 0.0, 1.0]

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()


# ---------------------------------------------------------------------------
# Single-arm execution
# ---------------------------------------------------------------------------
def run_arm(runner: WaypointRunner, pts: list, max_pts: int,
            dwell: float, go_home_after: bool, home_first: bool,
            split_timing: bool = False, all_orientations: bool = False):
    """
    Execute waypoints and collect per-point timing.
    Returns (success_count, fail_count, timing_records).

    split_timing=False, all_orientations=False (default):
        Uses move_to_pose() + wait_until_executed() — original combined mode.
        Records: {x, y, z, success, planning_ms, execution_ms, total_ms}

    split_timing=True:
        Uses plan()+execute() with direct polling — TRUE OMPL/motion separation.
        Records: {x, y, z, success, ompl_ms, planned_duration_ms,
                  motion_ms, total_ms, n_waypoints, joint_path_len}

    all_orientations=True:
        Tries all 6 GRASP_ORIENTATIONS per point; first success wins.
        Records: {x, y, z, success, orient_name, qx, qy, qz, qw,
                  ompl_ms, motion_ms, total_ms}
    """
    arm = runner.arm
    log = runner.node.get_logger()

    if home_first:
        runner.go_home()
        time.sleep(1.0)

    total = min(len(pts), max_pts) if max_pts > 0 else len(pts)
    if all_orientations:
        mode = "all-orientations"
    elif split_timing:
        mode = "split (OMPL separate)"
    else:
        mode = "combined"
    log.info(f"[{arm}] Starting waypoint run: {total} points  mode={mode}")

    success = 0
    fail = 0
    timing_records = []
    t0 = time.time()

    for i, (x, y, z) in enumerate(pts[:total]):
        log.info(f"[{arm}] [{i+1}/{total}]  -> ({x:.3f}, {y:.3f}, {z:.3f})")

        if all_orientations:
            ok, orient_name, ompl_ms, motion_ms, total_ms, quat = \
                runner.try_orientations(x, y, z)
            timing_records.append({
                "x": x, "y": y, "z": z,
                "success":      ok,
                "orient_name":  orient_name,
                "qx": quat[0] if quat else 0.0,
                "qy": quat[1] if quat else 0.0,
                "qz": quat[2] if quat else 0.0,
                "qw": quat[3] if quat else 1.0,
                "ompl_ms":      ompl_ms,
                "motion_ms":    motion_ms,
                "total_ms":     total_ms,
            })
            if ok:
                success += 1
                log.info(
                    f"[{arm}]   REACHED ({success} ok / {fail} fail)  "
                    f"orient={orient_name}  ompl={ompl_ms:.0f}ms  "
                    f"motion={motion_ms:.0f}ms  total={total_ms:.0f}ms"
                )
                time.sleep(dwell)
            else:
                fail += 1
                log.warning(f"[{arm}]   FAILED all orientations ({success} ok / {fail} fail)")

        elif split_timing:
            ok, ompl_ms, plan_dur_ms, motion_ms, total_ms, n_wpts, jpath = \
                runner.plan_separate(x, y, z, do_execute=True)
            timing_records.append({
                "x": x, "y": y, "z": z,
                "success":             ok,
                "ompl_ms":             ompl_ms,
                "planned_duration_ms": plan_dur_ms,
                "motion_ms":           motion_ms,
                "total_ms":            total_ms,
                "n_waypoints":         n_wpts,
                "joint_path_len":      jpath,
                "planning_ms":         ompl_ms,    # legacy alias
                "execution_ms":        motion_ms,  # legacy alias
            })
            if ok:
                success += 1
                log.info(
                    f"[{arm}]   REACHED ({success} ok / {fail} fail)  "
                    f"ompl={ompl_ms:.0f}ms  plan_dur={plan_dur_ms:.0f}ms  "
                    f"motion={motion_ms:.0f}ms  total={total_ms:.0f}ms  "
                    f"wpts={n_wpts}  jpath={jpath:.3f}rad"
                )
                time.sleep(dwell)
            else:
                fail += 1
                log.warning(
                    f"[{arm}]   FAILED  ({success} ok / {fail} fail)  "
                    f"ompl={ompl_ms:.0f}ms (no path found)"
                )
        else:
            ok, plan_ms, exec_ms, total_ms = runner.move_to(x, y, z)
            timing_records.append({
                "x": x, "y": y, "z": z,
                "success":      ok,
                "planning_ms":  plan_ms,
                "execution_ms": exec_ms,
                "total_ms":     total_ms,
            })
            if ok:
                success += 1
                log.info(
                    f"[{arm}]   REACHED ({success} ok / {fail} fail)  "
                    f"plan={plan_ms:.0f}ms  exec={exec_ms:.0f}ms  total={total_ms:.0f}ms"
                )
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

    return success, fail, timing_records


# ---------------------------------------------------------------------------
# Timing CSV + statistics
# ---------------------------------------------------------------------------
def save_timing_csv(records: list[dict], path: str):
    """Save timing records to CSV.  Automatically detects mode from record keys."""
    if not records:
        return
    first = records[0]
    has_orient = "orient_name" in first
    has_split  = "ompl_ms"     in first

    if has_orient:
        fields = ["x", "y", "z", "success",
                  "orient_name", "qx", "qy", "qz", "qw",
                  "ompl_ms", "motion_ms", "total_ms"]
    elif has_split:
        fields = ["x", "y", "z", "success",
                  "ompl_ms", "planned_duration_ms", "motion_ms", "total_ms",
                  "n_waypoints", "joint_path_len"]
    else:
        fields = ["x", "y", "z", "success", "planning_ms", "execution_ms", "total_ms"]

    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for r in records:
            row = {
                "x": f"{r['x']:.4f}",
                "y": f"{r['y']:.4f}",
                "z": f"{r['z']:.4f}",
                "success": int(r["success"]),
            }
            if has_orient:
                row.update({
                    "orient_name": r.get("orient_name", ""),
                    "qx": f"{r.get('qx', 0.0):.4f}",
                    "qy": f"{r.get('qy', 0.0):.4f}",
                    "qz": f"{r.get('qz', 0.0):.4f}",
                    "qw": f"{r.get('qw', 1.0):.4f}",
                    "ompl_ms":   f"{r['ompl_ms']:.2f}",
                    "motion_ms": f"{r['motion_ms']:.2f}",
                    "total_ms":  f"{r['total_ms']:.2f}",
                })
            elif has_split:
                row.update({
                    "ompl_ms":             f"{r['ompl_ms']:.2f}",
                    "planned_duration_ms": f"{r['planned_duration_ms']:.2f}",
                    "motion_ms":           f"{r['motion_ms']:.2f}",
                    "total_ms":            f"{r['total_ms']:.2f}",
                    "n_waypoints":         int(r["n_waypoints"]),
                    "joint_path_len":      f"{r['joint_path_len']:.4f}",
                })
            else:
                row.update({
                    "planning_ms":  f"{r['planning_ms']:.2f}",
                    "execution_ms": f"{r['execution_ms']:.2f}",
                    "total_ms":     f"{r['total_ms']:.2f}",
                })
            w.writerow(row)
    print(f"[timing] Saved → {path}")


def print_timing_stats(arm: str, records: list[dict]):
    """Print timing summary.  Detects mode from record keys:
    has_orient (orient_name key) > has_split (ompl_ms key) > normal.
    """
    has_orient = bool(records) and "orient_name" in records[0]
    has_split  = bool(records) and "ompl_ms"     in records[0] and not has_orient
    ok_recs    = [r for r in records if r["success"]]
    fails      = sum(1 for r in records if not r["success"])
    n          = len(ok_recs)

    if n == 0:
        print(f"\n[{arm}] All calls failed — no timing data.")
        return

    if has_orient:
        ompl_t   = np.array([r["ompl_ms"]   for r in ok_recs])
        motion_t = np.array([r["motion_ms"] for r in ok_recs])
        tot_t    = np.array([r["total_ms"]  for r in ok_recs])
        from collections import Counter
        orient_counts = Counter(r["orient_name"] for r in ok_recs)

        print(f"\n{'='*65}")
        print(f" All-Orientation Timing — {arm} arm  ({n} ok / {fails} fail / {len(records)} total)")
        print(f"{'='*65}")
        for label, arr in [
            ("OMPL planning  (first success orientation)  ", ompl_t),
            ("Robot motion   (actual execution wall-clock)", motion_t),
            ("Total          (OMPL + motion)              ", tot_t),
        ]:
            print(f"  {label}")
            print(f"    mean={arr.mean():.0f}ms  "
                  f"median={np.median(arr):.0f}ms  "
                  f"P95={np.percentile(arr,95):.0f}ms  "
                  f"max={arr.max():.0f}ms")
        print(f"\n  成功 orientation 分佈:")
        for name, cnt in sorted(orient_counts.items(), key=lambda x: -x[1]):
            print(f"    {name}: {cnt} 次 ({cnt/n*100:.0f}%)")
        print()
        for label, budget_ms in _RT_BUDGETS_MS.items():
            pct = np.mean(ompl_t <= budget_ms) * 100.0
            print(f"  OMPL within {label} ({budget_ms:.1f} ms): {pct:.1f}%")
        print(f"{'='*65}\n")

    elif has_split:
        ompl_t    = np.array([r["ompl_ms"]            for r in ok_recs])
        plandur_t = np.array([r["planned_duration_ms"] for r in ok_recs])
        motion_t  = np.array([r["motion_ms"]           for r in ok_recs])
        tot_t     = np.array([r["total_ms"]            for r in ok_recs])
        n_wpts    = np.array([r["n_waypoints"]         for r in ok_recs])
        jpath     = np.array([r["joint_path_len"]      for r in ok_recs])

        print(f"\n{'='*65}")
        print(f" Split Timing — {arm} arm  ({n} ok / {fails} fail / {len(records)} total)")
        print(f"{'='*65}")
        for label, arr in [
            ("OMPL planning  (IK + path search + time-param)", ompl_t),
            ("Planned traj duration (TOTG/IPTP output)      ", plandur_t),
            ("Robot motion   (actual execution wall-clock)  ", motion_t),
            ("Total          (OMPL + motion)                ", tot_t),
        ]:
            print(f"  {label}")
            print(f"    mean={arr.mean():.0f}ms  "
                  f"median={np.median(arr):.0f}ms  "
                  f"P95={np.percentile(arr,95):.0f}ms  "
                  f"max={arr.max():.0f}ms")
        print(f"\n  Trajectory waypoints  : "
              f"mean={n_wpts.mean():.0f}  min={n_wpts.min()}  max={n_wpts.max()}")
        print(f"  Joint-space path len  : "
              f"mean={jpath.mean():.3f} rad  P95={np.percentile(jpath,95):.3f} rad")
        pct_ompl = np.mean(ompl_t / np.maximum(tot_t, 1.0)) * 100
        ratio    = ompl_t / np.maximum(motion_t, 1.0)
        print(f"  OMPL / motion ratio   : mean={ratio.mean():.2f}×  "
              f"(OMPL is {pct_ompl:.0f}% of total time)")
        print()
        for label, budget_ms in _RT_BUDGETS_MS.items():
            pct = np.mean(ompl_t <= budget_ms) * 100.0
            print(f"  OMPL within {label} ({budget_ms:.1f} ms): {pct:.1f}%")
        print(f"{'='*65}\n")
    else:
        plan_t = np.array([r["planning_ms"]  for r in ok_recs])
        exec_t = np.array([r["execution_ms"] for r in ok_recs])
        tot_t  = np.array([r["total_ms"]     for r in ok_recs])

        print(f"\n{'='*58}")
        print(f" Timing Summary — {arm} arm  ({n} ok / {fails} fail / {len(records)} total)")
        print(f"{'='*58}")
        for label, arr in [("Planning (IK+traj)", plan_t),
                           ("Execution",          exec_t),
                           ("Total",              tot_t)]:
            print(f"  {label}:")
            print(f"    mean={arr.mean():.0f}ms  "
                  f"median={np.median(arr):.0f}ms  "
                  f"P95={np.percentile(arr,95):.0f}ms  "
                  f"max={arr.max():.0f}ms")
        print()
        for label, budget_ms in _RT_BUDGETS_MS.items():
            pct = np.mean(plan_t <= budget_ms) * 100.0
            print(f"  Planning within {label} ({budget_ms:.1f} ms): {pct:.1f}%")
        print(f"{'='*58}\n")


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def plot_timing(arm_records: dict[str, list[dict]], save_png: str):
    """
    arm_records: {"right": [...], "left": [...]}  (one or both arms)

    Automatically detects split-timing mode (ompl_ms key present) and renders
    different panels:

    Normal mode  → 4 panels: histogram · timeline (plan+exec) · 3D heatmap · CDF
    Split mode   → 4 panels: OMPL histogram · OMPL+motion timeline · 3D OMPL heatmap
                              · CDF comparing OMPL vs planned_duration
    """
    first_recs = next(iter(arm_records.values()))
    has_orient = bool(first_recs) and "orient_name" in first_recs[0]
    has_split  = bool(first_recs) and "ompl_ms" in first_recs[0] and not has_orient

    if has_orient:
        title_mode = "All-Orientation OMPL + Motion Timing"
    elif has_split:
        title_mode = "Split OMPL + Motion Timing"
    else:
        title_mode = "Planning & Execution Timing"

    fig = plt.figure(figsize=(18, 13))
    fig.suptitle(
        f"MoveIt2 Waypoint Runner — {title_mode}\n"
        "Real-time Teleoperation Evaluation",
        fontsize=15, fontweight="bold"
    )
    gs = gridspec.GridSpec(2, 3, figure=fig,
                           left=0.07, right=0.97,
                           top=0.91, bottom=0.08,
                           wspace=0.35, hspace=0.42)

    ax_hist = fig.add_subplot(gs[0, 0])
    ax_time = fig.add_subplot(gs[0, 1:])
    ax_3d   = fig.add_subplot(gs[1, 0:2], projection="3d")
    ax_cdf  = fig.add_subplot(gs[1, 2])

    first_arm = next(iter(arm_records))

    for arm, records in arm_records.items():
        color   = _ARM_COLOR.get(arm, "gray")
        ok_recs = [r for r in records if r["success"]]
        ok_idx  = [i for i, r in enumerate(records) if r["success"]]
        if not ok_recs:
            continue

        if has_orient:
            ompl_arr   = np.array([r["ompl_ms"]   for r in ok_recs])
            motion_arr = np.array([r["motion_ms"] for r in ok_recs])

            ax_hist.hist(ompl_arr, bins=40, color=color, alpha=0.55,
                         edgecolor="white",
                         label=f"{arm} OMPL (n={len(ompl_arr)})")
            ax_time.bar(ok_idx, ompl_arr, color=color, alpha=0.75, width=1.0,
                        label=f"{arm} OMPL planning")
            ax_time.bar(ok_idx, motion_arr, bottom=ompl_arr,
                        color=color, alpha=0.25, width=1.0,
                        label=f"{arm} robot motion")
            st  = np.sort(ompl_arr)
            cdf = np.arange(1, len(st) + 1) / len(st)
            ax_cdf.plot(st, cdf * 100.0, color=color, linewidth=1.8, label=f"{arm} OMPL")

        elif has_split:
            ompl_arr    = np.array([r["ompl_ms"]            for r in ok_recs])
            plandur_arr = np.array([r["planned_duration_ms"] for r in ok_recs])
            motion_arr  = np.array([r["motion_ms"]           for r in ok_recs])

            ax_hist.hist(ompl_arr, bins=40, color=color, alpha=0.55,
                         edgecolor="white",
                         label=f"{arm} OMPL (n={len(ompl_arr)})")
            ax_time.bar(ok_idx, ompl_arr, color=color, alpha=0.75, width=1.0,
                        label=f"{arm} OMPL planning")
            ax_time.bar(ok_idx, motion_arr, bottom=ompl_arr,
                        color=color, alpha=0.25, width=1.0,
                        label=f"{arm} robot motion")
            ax_time.plot(ok_idx, plandur_arr + ompl_arr,
                         color=color, linestyle=":", linewidth=1.5,
                         marker=".", markersize=2, alpha=0.80,
                         label=f"{arm} plan_dur + OMPL")
            for arr_cdf, style, lbl in [
                (ompl_arr,    "-",  f"{arm} OMPL"),
                (plandur_arr, "--", f"{arm} plan_dur"),
            ]:
                st  = np.sort(arr_cdf)
                cdf = np.arange(1, len(st) + 1) / len(st)
                ax_cdf.plot(st, cdf * 100.0, color=color, linewidth=1.8,
                            linestyle=style, label=lbl)
        else:
            plan_arr = np.array([r["planning_ms"]  for r in ok_recs])
            exec_arr = np.array([r["execution_ms"] for r in ok_recs])

            # ── Histogram: planning time ──
            ax_hist.hist(plan_arr, bins=40, color=color, alpha=0.55,
                         edgecolor="white",
                         label=f"{arm} planning (n={len(plan_arr)})")

            # ── Timeline: planning + execution stacked ──
            ax_time.bar(ok_idx, plan_arr, color=color, alpha=0.55, width=1.0,
                        label=f"{arm} planning")
            ax_time.bar(ok_idx, exec_arr, bottom=plan_arr,
                        color=color, alpha=0.25, width=1.0,
                        label=f"{arm} execution")
            if len(plan_arr) >= 20:
                win = 20
                rm = np.convolve(plan_arr, np.ones(win) / win, mode="valid")
                ax_time.plot(ok_idx[win - 1:], rm, color=color, linewidth=2.0,
                             label=f"{arm} plan rolling-mean")

            # ── CDF: planning time ──
            sorted_t = np.sort(plan_arr)
            cdf = np.arange(1, len(sorted_t) + 1) / len(sorted_t)
            ax_cdf.plot(sorted_t, cdf * 100.0, color=color, linewidth=2.0, label=arm)

    # ── 3-D heatmap: OMPL (or planning) time for first arm ──────────────────
    ok_recs = [r for r in arm_records[first_arm] if r["success"]]
    if ok_recs:
        xs = [r["x"] for r in ok_recs]
        ys = [r["y"] for r in ok_recs]
        zs = [r["z"] for r in ok_recs]
        if has_orient or has_split:
            ts = [r["ompl_ms"] for r in ok_recs]
            cbar_lbl = "OMPL planning time (ms)"
        else:
            ts = [r["planning_ms"] for r in ok_recs]
            cbar_lbl = "planning time (ms)"
        sc = ax_3d.scatter(xs, ys, zs, c=ts, cmap="plasma", s=18, alpha=0.75,
                           vmin=0, vmax=np.percentile(ts, 98))
        fig.colorbar(sc, ax=ax_3d, shrink=0.55, pad=0.01, label=cbar_lbl)
        ax_3d.set_xlabel("X (m)", labelpad=6, fontsize=9)
        ax_3d.set_ylabel("Y (m)", labelpad=6, fontsize=9)
        ax_3d.set_zlabel("Z (m)", labelpad=6, fontsize=9)
        hmap_mode = "OMPL planning" if (has_orient or has_split) else "planning"
        ax_3d.set_title(f"3-D Workspace — {hmap_mode} time heatmap ({first_arm})",
                        fontsize=10)

    # ── Budget lines ─────────────────────────────────────────────────────────
    for label, ms in _RT_BUDGETS_MS.items():
        c  = _RT_COLORS[label]
        kw = dict(color=c, linestyle="--", linewidth=1.2, alpha=0.85)
        ax_hist.axvline(ms, **kw, label=f"{label} ({ms:.1f} ms)")
        ax_time.axhline(ms, **kw, label=f"{label} ({ms:.1f} ms)")
        ax_cdf.axvline(ms,  **kw, label=f"{label} ({ms:.1f} ms)")

    # ── Stats annotation on histogram ────────────────────────────────────────
    for idx, (arm, records) in enumerate(arm_records.items()):
        ok_recs = [r for r in records if r["success"]]
        if not ok_recs:
            continue
        if has_orient:
            ompl_a = np.array([r["ompl_ms"] for r in ok_recs])
            txt = (f"{arm} OMPL: mean={ompl_a.mean():.0f}ms "
                   f"P95={np.percentile(ompl_a,95):.0f}ms")
        elif has_split:
            ompl_a = np.array([r["ompl_ms"]            for r in ok_recs])
            pdur_a = np.array([r["planned_duration_ms"] for r in ok_recs])
            txt = (f"{arm} OMPL: mean={ompl_a.mean():.0f}ms "
                   f"P95={np.percentile(ompl_a,95):.0f}ms\n"
                   f"{arm} plan_dur: mean={pdur_a.mean():.0f}ms "
                   f"P95={np.percentile(pdur_a,95):.0f}ms")
        else:
            arr = np.array([r["planning_ms"] for r in ok_recs])
            txt = (f"{arm}: mean={arr.mean():.0f}ms "
                   f"P95={np.percentile(arr,95):.0f}ms "
                   f"max={arr.max():.0f}ms")
        ax_hist.text(0.98, 0.98 - idx * 0.14, txt,
                     transform=ax_hist.transAxes, ha="right", va="top",
                     fontsize=7.5, linespacing=1.4,
                     bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.75))

    # ── Decorate axes ─────────────────────────────────────────────────────────
    hist_xlabel = "OMPL planning time (ms)" if (has_orient or has_split) else "Planning time (ms)"
    hist_title  = "OMPL Planning Time Distribution" if (has_orient or has_split) else "Planning Time Distribution"
    ax_hist.set_xlabel(hist_xlabel, fontsize=10)
    ax_hist.set_ylabel("Count", fontsize=10)
    ax_hist.set_title(hist_title, fontsize=11)
    ax_hist.legend(fontsize=8)

    if has_orient:
        time_title = "Per-Waypoint: OMPL (dark) + Motion (light)  [all-orient mode]"
    elif has_split:
        time_title = "Per-Waypoint: OMPL (dark) + Motion (light)  \u00b7\u00b7\u00b7 OMPL+plan_dur"
    else:
        time_title = "Per-Waypoint Timing  (planning + execution)"
    ax_time.set_xlabel("Waypoint index", fontsize=10)
    ax_time.set_ylabel("Time (ms)", fontsize=10)
    ax_time.set_title(time_title, fontsize=11)
    ax_time.legend(fontsize=7, loc="upper right", ncol=2)
    ax_time.set_ylim(bottom=0)

    cdf_xlabel = "Time (ms)" if (has_orient or has_split) else "Planning time (ms)"
    ax_cdf.set_xlabel(cdf_xlabel, fontsize=10)
    ax_cdf.set_ylabel("Cumulative % of waypoints", fontsize=10)
    ax_cdf.set_title("CDF — real-time budget analysis", fontsize=11)
    ax_cdf.legend(fontsize=8)
    ax_cdf.set_ylim(0, 102)
    ax_cdf.grid(True, alpha=0.3)
    ax_cdf.yaxis.set_major_formatter(
        mticker.FuncFormatter(lambda v, _: f"{v:.0f}%")
    )

    plt.savefig(save_png, dpi=150, bbox_inches="tight")
    print(f"[plot] Saved → {save_png}")
    plt.close(fig)

    # Try to open with a system viewer
    for viewer in ("eog", "feh", "display", "xdg-open"):
        if shutil.which(viewer):
            subprocess.Popen([viewer, save_png],
                             stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
            print(f"[plot] Opened with {viewer}")
            break


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
    parser.add_argument("--no-timing",  action="store_true",
                        help="Skip timing CSV and plot generation")
    parser.add_argument("--split-timing", action="store_true",
                        help="Separate OMPL planning time from robot motion time "
                             "(uses plan()+execute() instead of move_to_pose()+wait). "
                             "Adds ompl_ms / planned_duration_ms / motion_ms / "
                             "n_waypoints / joint_path_len to CSV and plot.")
    parser.add_argument("--all-orient", action="store_true",
                        help="Try all 6 GRASP_ORIENTATIONS per point; first success wins. "
                             "CSV adds orient_name + qx/qy/qz/qw columns.")
    parser.add_argument("--timing-csv", default=None,
                        help="Override timing CSV filename (default: <arm>_waypoint_timing.csv)")
    parser.add_argument("--save-png",   default=None,
                        help="Override plot PNG filename (default: <arm>_waypoint_timing.png)")
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
            left_timing  = []
            right_timing = []
            for i in range(total):
                if i < L:
                    x, y, z = left_pts[i]
                    print(f"[LEFT  {i+1}/{L}] -> ({x:.3f},{y:.3f},{z:.3f})")
                    if args.split_timing:
                        ok, ompl_ms, pd_ms, mot_ms, t_ms, nw, jp = \
                            left_runner.plan_separate(x, y, z)
                        left_timing.append({
                            "x": x, "y": y, "z": z, "success": ok,
                            "ompl_ms": ompl_ms, "planned_duration_ms": pd_ms,
                            "motion_ms": mot_ms, "total_ms": t_ms,
                            "n_waypoints": nw, "joint_path_len": jp,
                            "planning_ms": ompl_ms, "execution_ms": mot_ms,
                        })
                        if ok:
                            l_ok += 1
                            print(f"  ompl={ompl_ms:.0f}ms  plan_dur={pd_ms:.0f}ms  "
                                  f"motion={mot_ms:.0f}ms  total={t_ms:.0f}ms")
                        else:
                            l_fail += 1
                    else:
                        ok, p_ms, e_ms, t_ms = left_runner.move_to(x, y, z)
                        left_timing.append({"x": x, "y": y, "z": z, "success": ok,
                                            "planning_ms": p_ms, "execution_ms": e_ms,
                                            "total_ms": t_ms})
                        if ok:
                            l_ok += 1
                            print(f"  plan={p_ms:.0f}ms  exec={e_ms:.0f}ms  total={t_ms:.0f}ms")
                        else:
                            l_fail += 1
                if i < R:
                    x, y, z = right_pts[i]
                    print(f"[RIGHT {i+1}/{R}] -> ({x:.3f},{y:.3f},{z:.3f})")
                    if args.split_timing:
                        ok, ompl_ms, pd_ms, mot_ms, t_ms, nw, jp = \
                            right_runner.plan_separate(x, y, z)
                        right_timing.append({
                            "x": x, "y": y, "z": z, "success": ok,
                            "ompl_ms": ompl_ms, "planned_duration_ms": pd_ms,
                            "motion_ms": mot_ms, "total_ms": t_ms,
                            "n_waypoints": nw, "joint_path_len": jp,
                            "planning_ms": ompl_ms, "execution_ms": mot_ms,
                        })
                        if ok:
                            r_ok += 1
                            print(f"  ompl={ompl_ms:.0f}ms  plan_dur={pd_ms:.0f}ms  "
                                  f"motion={mot_ms:.0f}ms  total={t_ms:.0f}ms")
                        else:
                            r_fail += 1
                    else:
                        ok, p_ms, e_ms, t_ms = right_runner.move_to(x, y, z)
                        right_timing.append({"x": x, "y": y, "z": z, "success": ok,
                                             "planning_ms": p_ms, "execution_ms": e_ms,
                                             "total_ms": t_ms})
                        if ok:
                            r_ok += 1
                            print(f"  plan={p_ms:.0f}ms  exec={e_ms:.0f}ms  total={t_ms:.0f}ms")
                        else:
                            r_fail += 1
                time.sleep(args.dwell)

            print(f"\nDone. Left: {l_ok} ok / {l_fail} fail   Right: {r_ok} ok / {r_fail} fail")

            if args.home:
                left_runner.go_home()
                right_runner.go_home()

            left_runner.shutdown()
            right_runner.shutdown()

            if not args.no_timing:
                arm_records = {}
                if left_timing:
                    arm_records["left"] = left_timing
                    print_timing_stats("left", left_timing)
                    csv_path = args.timing_csv or "left_waypoint_timing.csv"
                    save_timing_csv(left_timing, csv_path)
                if right_timing:
                    arm_records["right"] = right_timing
                    print_timing_stats("right", right_timing)
                    csv_path = args.timing_csv or "right_waypoint_timing.csv"
                    save_timing_csv(right_timing, csv_path)
                if arm_records:
                    png = args.save_png or "bimanual_waypoint_timing.png"
                    plot_timing(arm_records, png)

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
            _, _, timing_records = run_arm(
                runner, pts, args.max_pts, args.dwell,
                go_home_after=args.home, home_first=args.home_first,
                split_timing=args.split_timing,
                all_orientations=args.all_orient,
            )
            runner.shutdown()

            if not args.no_timing and timing_records:
                arm = args.arm
                print_timing_stats(arm, timing_records)
                csv_out = args.timing_csv or f"{arm}_waypoint_timing.csv"
                save_timing_csv(timing_records, csv_out)
                png_out = args.save_png or f"{arm}_waypoint_timing.png"
                plot_timing({arm: timing_records}, png_out)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
