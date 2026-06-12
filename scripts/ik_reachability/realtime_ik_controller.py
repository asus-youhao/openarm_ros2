#!/usr/bin/env python3
"""
realtime_ik_controller.py — Real-time IK-based arm controller (bypasses OMPL)
==============================================================================
Subscribes to pose commands, calls /compute_ik (1–20 ms, no OMPL), and
publishes JointTrajectory commands directly to the arm controller.

This is **Stage 3** of the VR/AR MoveIt2 evaluation plan: replacing the
OMPL MoveGroup pipeline with a lean IK → JointTrajectory loop for
low-latency VR/AR teleoperation.

Architecture:
    VR/AR (30–90 Hz)
         │  /right/target_pose  (geometry_msgs/PoseStamped)
         ▼
    realtime_ik_controller.py
         │  /compute_ik service  (1–20 ms per call, no random sampling)
         │  Uses last joint solution as IK seed → continuous, jerk-free
         ▼
    /right_arm_controller/joint_trajectory
         │  (trajectory_msgs/JointTrajectory, horizon = --horizon ms)
         ▼
    OpenArm O6 right arm

Comparison with MoveGroup OMPL approach:
    OMPL MoveGroup:   ~100–600 ms planning → 1–5 Hz control rate
    This node (/compute_ik):  1–20 ms → 50–200 Hz control rate
    MoveIt Servo:     < 10 ms (Jacobian, no IK service) → 100+ Hz
                      → see servo_teleop.launch.py

WARNING — Safety:
    /compute_ik does NOT check self-collisions or environment collisions by
    default. Use avoid_collisions=True in the IK request for basic collision
    avoidance, but it is far less thorough than OMPL.
    Only use in a clear, known workspace.

Prerequisites (must be running):
    ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
    ros2 launch openarm_bimanual_moveit_config move_group_only.launch.py

Usage examples:
    # Control right arm from /right/target_pose at 50 ms trajectory horizon
    python3 realtime_ik_controller.py --arm right

    # Left arm, 33 ms horizon (matches 30 Hz VR frame rate)
    python3 realtime_ik_controller.py --arm left --horizon 33

    # Disable collision checking for maximum speed (~2–5 ms IK)
    python3 realtime_ik_controller.py --arm right --no-collisions

    # Return home on exit
    python3 realtime_ik_controller.py --arm right --home-on-exit

    # Benchmark IK latency only (no real execution — dry-run mode)
    python3 realtime_ik_controller.py --arm right --dry-run

    # Override default EE orientation (qx qy qz qw)
    python3 realtime_ik_controller.py --arm right --quat 0.0 0.0 -0.707 0.707

Test the controller by publishing poses:
    # Move to a single pose (publish once)
    ros2 topic pub --once /right/target_pose geometry_msgs/PoseStamped \\
      "{header: {frame_id: world}, pose: {position: {x: 0.4, y: -0.2, z: 0.3},
       orientation: {x: 0.0, y: 0.0, z: -0.707, w: 0.707}}}"

    # Continuous stream at 20 Hz (simulates VR input)
    ros2 topic pub --rate 20 /right/target_pose geometry_msgs/PoseStamped \\
      "{header: {frame_id: world}, pose: {position: {x: 0.4, y: -0.2, z: 0.4},
       orientation: {x: 0.0, y: 0.0, z: -0.707, w: 0.707}}}"

Monitor IK latency:
    ros2 topic echo /right/ik_latency_ms  (std_msgs/Float32)

CSV batch testing (without VR/AR device, uses right_reachability_.csv):
    # Run all reachable points, measure IK latency + motion timing
    python3 realtime_ik_controller.py --arm right --csv right_reachability_.csv

    # First 20 points, 0.5 s dwell between moves, go home before starting
    python3 realtime_ik_controller.py --arm right --csv right_reachability_.csv \\
        --max-pts 20 --dwell 0.5 --home-first

    # Dry-run: measure IK latency only, NO robot motion
    python3 realtime_ik_controller.py --arm right --csv right_reachability_.csv \\
        --dry-run

    # 100 ms trajectory horizon (slower, heavier moves)
    python3 realtime_ik_controller.py --arm right --csv right_reachability_.csv \\
        --horizon 100 --dwell 0.2 --home-first

    # Save timing CSV + PNG to custom paths
    python3 realtime_ik_controller.py --arm right --csv right_reachability_.csv \\
        --timing-csv my_ik_batch.csv

Outputs (CSV batch mode):
    <arm>_ik_batch_timing.csv   per-point: x y z success ik_ms wait_ms total_ms
    <arm>_ik_batch_timing.png   3-panel: histogram · timeline · 3-D workspace heatmap

Options:
    --arm            left | right  (default: right)
    --horizon        Trajectory time horizon in ms (default: 50).
                     In batch mode this is the JointTrajectory duration sent to
                     the controller. The node waits horizon+dwell before next pt.
                     e.g., 30 Hz VR → --horizon 33
    --quat           Default EE orientation qx qy qz qw
                     (if not set in incoming PoseStamped orientation)
    --no-collisions  Disable IK collision checking (faster, less safe)
    --home-on-exit   Send arm to home position on Ctrl-C
    --dry-run        Compute IK but do not publish to controller
    --rate-limit     Maximum command rate in Hz (default: 100, realtime mode only)
    --timeout-ik     /compute_ik service call timeout in seconds (default: 0.1)

    CSV batch options:
    --csv PATH       Run batch test from a reachability CSV file
    --max-pts N      Limit to first N CSV points
    --dwell SEC      Extra wait after each trajectory horizon (default: 0.3 s)
    --home-first     Go to home position before starting CSV run
    --no-sort        Do not sort by Z (default: sort low-to-high)
    --timing-csv P   Output CSV path (default: <arm>_ik_batch_timing.csv)
    --no-plot        Skip PNG plot generation

IK seed strategy:
    Each IK call uses the previous successful joint solution as the seed
    (robot_state in the IK request). This ensures:
    · The solver prefers the nearest solution → smooth, continuous motion
    · No sudden large joint jumps when multiple IK solutions exist
    · Graceful degradation near IK boundaries
"""

import argparse
import csv as _csv_mod
import pathlib
import statistics as _statistics
import sys
import threading
import time
from collections import deque

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import rclpy
import rclpy.executors
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ---------------------------------------------------------------------------
# Per-arm configuration  (mirrors csv_waypoint_runner.py _ARM_CONFIG)
# ---------------------------------------------------------------------------
_ARM_CONFIG = {
    "left": {
        "joint_names":   [f"openarm_left_joint{i}" for i in range(1, 8)],
        "base_link":     "world",
        "ee_link":       "openarm_left_link7",
        "group_name":    "left_arm",
        "home_joints":   [0.0] * 7,
        "default_quat":  (0.0, 0.0, 0.707, 0.707),   # side_pos_y
        "cmd_topic":     "/left_arm_controller/joint_trajectory",
        "pose_topic":    "/left/target_pose",
        "latency_topic": "/left/ik_latency_ms",
    },
    "right": {
        "joint_names":   [f"openarm_right_joint{i}" for i in range(1, 8)],
        "base_link":     "world",
        "ee_link":       "openarm_right_link7",
        "group_name":    "right_arm",
        "home_joints":   [0.0] * 7,
        "default_quat":  (0.0, 0.0, -0.707, 0.707),  # side_neg_y
        "cmd_topic":     "/right_arm_controller/joint_trajectory",
        "pose_topic":    "/right/target_pose",
        "latency_topic": "/right/ik_latency_ms",
    },
}

# ---------------------------------------------------------------------------
# CSV loading  (mirrors csv_waypoint_runner.py)
# ---------------------------------------------------------------------------
def load_reachable_pts(csv_path: str, sort_by_z: bool = True):
    """Load CSV produced by ik_reachability_sampler.py → only reachable=1 rows.

    If the CSV has qx/qy/qz/qw columns (from joint_states_to_ee_poses.py),
    the recorded orientation is returned as a 4th tuple element; otherwise None.
    """
    pts = []
    with open(csv_path, newline="") as f:
        for row in _csv_mod.DictReader(f):
            if row["reachable"] == "1":
                x, y, z = float(row["x"]), float(row["y"]), float(row["z"])
                quat = None
                if "qx" in row and row["qx"] != "":
                    quat = (float(row["qx"]), float(row["qy"]),
                            float(row["qz"]), float(row["qw"]))
                pts.append((x, y, z, quat))
    if sort_by_z:
        pts.sort(key=lambda p: p[2])  # low-to-high Z reduces large joint jumps
    return pts


# ---------------------------------------------------------------------------
# Named EE orientations — mirrors csv_waypoint_runner.py GRASP_ORIENTATIONS
# The reachability sampler (ik_reachability_sampler.py) tries ALL 6 and marks
# a point reachable=1 if ANY succeeds.  Use --all-orient to match that logic.
# ---------------------------------------------------------------------------
GRASP_ORIENTATIONS = {
    "side_neg_y":   (0.0,    0.0,   -0.707,  0.707),  # natural right-arm approach
    "side_pos_y":   (0.0,    0.0,    0.707,  0.707),  # natural left-arm approach
    "top_down":     (0.0,    0.707,  0.0,    0.707),  # top-down grasp
    "front_pos_x":  (0.0,    0.0,    0.0,    1.0),    # frontal +X
    "front_neg_x":  (0.0,    0.0,    1.0,    0.0),    # frontal -X
    "bottom_up":    (0.0,   -0.707,  0.0,    0.707),  # bottom-up (shelf pickup)
}


# IK error code names for diagnostic messages
_IK_ERROR_NAMES = {
    MoveItErrorCodes.SUCCESS:           "SUCCESS",
    MoveItErrorCodes.NO_IK_SOLUTION:    "NO_IK_SOLUTION",
    MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
    MoveItErrorCodes.TIMED_OUT:         "TIMED_OUT",
}


# ---------------------------------------------------------------------------
# Controller node
# ---------------------------------------------------------------------------
class RealtimeIKController(Node):
    """ROS2 node: subscribe to pose → call /compute_ik → publish JointTrajectory."""

    def __init__(self, arm: str, horizon_ms: float, avoid_collisions: bool,
                 dry_run: bool, rate_limit_hz: float, ik_timeout_sec: float,
                 quat_override=None, home_on_exit: bool = False):
        super().__init__(f"realtime_ik_controller_{arm}")
        self.arm = arm
        self.cfg = _ARM_CONFIG[arm]
        self.horizon_sec = horizon_ms / 1000.0
        self.avoid_collisions = avoid_collisions
        self.dry_run = dry_run
        self.min_cmd_period = 1.0 / rate_limit_hz
        self.ik_timeout_sec = ik_timeout_sec
        self.home_on_exit = home_on_exit

        # Default EE quaternion (qx, qy, qz, qw)
        if quat_override:
            self.default_quat = quat_override
        else:
            self.default_quat = self.cfg["default_quat"]

        # IK seed: last successful joint solution (prevents inter-call jumps)
        self._seed_joints = self.cfg["home_joints"][:]
        self._seed_lock = threading.Lock()

        # Rate limiting: track last command time
        self._last_cmd_time = 0.0

        # Running statistics
        self._stats: deque = deque(maxlen=200)  # circular buffer of IK latencies
        self._n_success = 0
        self._n_failure = 0

        cb = ReentrantCallbackGroup()

        # /compute_ik service client
        self._ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=cb)
        self.get_logger().info("[init] Waiting for /compute_ik service...")
        if not self._ik_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error("[init] /compute_ik service not available!")
            raise RuntimeError("/compute_ik service not found. Is move_group running?")
        self.get_logger().info("[init] /compute_ik ready.")

        # Subscribe to current joint states (for IK seed bootstrap)
        self._js_ready = threading.Event()
        self._js_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10,
            callback_group=cb)
        self.get_logger().info("[init] Waiting for /joint_states...")
        if not self._js_ready.wait(timeout=10.0):
            self.get_logger().warn("[init] No /joint_states received — using zero seed.")

        # JointTrajectory publisher (output to controller)
        if not dry_run:
            self._traj_pub = self.create_publisher(
                JointTrajectory, self.cfg["cmd_topic"], 10)
            self.get_logger().info(
                f"[init] Publishing to {self.cfg['cmd_topic']}")
        else:
            self._traj_pub = None
            self.get_logger().warn("[init] DRY-RUN mode — IK computed but not published.")

        # IK latency publisher (for monitoring / plotting)
        self._latency_pub = self.create_publisher(
            Float32, self.cfg["latency_topic"], 10)

        # Subscribe to incoming pose commands
        self._pose_sub = self.create_subscription(
            PoseStamped, self.cfg["pose_topic"], self._pose_cb, 10,
            callback_group=cb)

        self.get_logger().info(
            f"[{arm}] RealtimeIKController ready.\n"
            f"  listening on: {self.cfg['pose_topic']}\n"
            f"  output topic: {self.cfg['cmd_topic']}\n"
            f"  horizon: {horizon_ms:.0f} ms  |  "
            f"collisions: {avoid_collisions}  |  "
            f"rate_limit: {rate_limit_hz:.0f} Hz  |  "
            f"dry_run: {dry_run}"
        )

    # ------------------------------------------------------------------ #
    #  Joint state bootstrap (first message → seed)
    # ------------------------------------------------------------------ #
    def _joint_state_cb(self, msg: JointState):
        """Bootstrap IK seed from current joint state on first message."""
        if self._js_ready.is_set():
            return
        arm_joints = self.cfg["joint_names"]
        positions = []
        for jname in arm_joints:
            if jname in msg.name:
                idx = msg.name.index(jname)
                positions.append(msg.position[idx])
            else:
                positions.append(0.0)
        with self._seed_lock:
            self._seed_joints = positions
        self._js_ready.set()
        self.get_logger().info(
            f"[init] Seed bootstrapped from /joint_states: "
            f"{[f'{v:.3f}' for v in positions]}")

    # ------------------------------------------------------------------ #
    #  Incoming pose callback (hot path)
    # ------------------------------------------------------------------ #
    def _pose_cb(self, msg: PoseStamped):
        now = time.perf_counter()

        # Rate limiting: skip if too soon since last command
        if (now - self._last_cmd_time) < self.min_cmd_period:
            return
        self._last_cmd_time = now

        # Build IK request
        req = GetPositionIK.Request()
        ik = req.ik_request
        ik.group_name = self.cfg["group_name"]
        ik.avoid_collisions = self.avoid_collisions
        ik.ik_link_name = self.cfg["ee_link"]

        # Fill in the desired EE pose
        pose = msg.pose
        # If incoming orientation is identity (0,0,0,1), use default quat
        qx, qy, qz, qw = (pose.orientation.x, pose.orientation.y,
                           pose.orientation.z, pose.orientation.w)
        if abs(qw) < 0.001 and abs(qx) < 0.001 and abs(qy) < 0.001 and abs(qz) < 0.001:
            # all-zero quaternion: use default
            qx, qy, qz, qw = self.default_quat

        ik.pose_stamped.header.frame_id = self.cfg["base_link"]
        ik.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ik.pose_stamped.pose.position.x = pose.position.x
        ik.pose_stamped.pose.position.y = pose.position.y
        ik.pose_stamped.pose.position.z = pose.position.z
        ik.pose_stamped.pose.orientation.x = qx
        ik.pose_stamped.pose.orientation.y = qy
        ik.pose_stamped.pose.orientation.z = qz
        ik.pose_stamped.pose.orientation.w = qw

        # Use last solution as seed for continuity
        with self._seed_lock:
            seed = self._seed_joints[:]
        robot_state = RobotState()
        robot_state.joint_state.name = self.cfg["joint_names"]
        robot_state.joint_state.position = seed
        ik.robot_state = robot_state

        # Set IK timeout (shorter than obstacle-rich environments)
        from builtin_interfaces.msg import Duration
        ik.timeout.sec = 0
        ik.timeout.nanosec = int(self.ik_timeout_sec * 1e9)

        # --- Call /compute_ik (synchronous future call) ---
        t_ik_start = time.perf_counter()
        future = self._ik_client.call_async(req)

        # Block until result (we are in the callback thread — spin in background)
        deadline = t_ik_start + self.ik_timeout_sec + 0.02  # extra margin
        while not future.done():
            if time.perf_counter() > deadline:
                self.get_logger().warn("[IK] Timed out waiting for future.")
                self._n_failure += 1
                return
            time.sleep(0.001)

        t_ik_end = time.perf_counter()
        ik_ms = (t_ik_end - t_ik_start) * 1000.0

        # Publish latency for monitoring
        self._latency_pub.publish(Float32(data=float(ik_ms)))
        self._stats.append(ik_ms)

        response = future.result()
        if response is None:
            self.get_logger().warn("[IK] No response from service.")
            self._n_failure += 1
            return

        err = response.error_code.val
        if err != MoveItErrorCodes.SUCCESS:
            err_name = _IK_ERROR_NAMES.get(err, f"code={err}")
            self.get_logger().warn(
                f"[IK] Failed: {err_name}  "
                f"pos=({pose.position.x:.3f}, {pose.position.y:.3f}, "
                f"{pose.position.z:.3f})  lat={ik_ms:.1f} ms")
            self._n_failure += 1
            return

        self._n_success += 1

        # Extract joint solution
        sol_state = response.solution.joint_state
        sol_positions = []
        for jname in self.cfg["joint_names"]:
            if jname in sol_state.name:
                idx = sol_state.name.index(jname)
                sol_positions.append(sol_state.position[idx])
            else:
                sol_positions.append(0.0)

        # Update seed (thread-safe)
        with self._seed_lock:
            self._seed_joints = sol_positions[:]

        # Log every 50 commands
        total = self._n_success + self._n_failure
        if total % 50 == 1:
            stats_str = self._fmt_stats()
            self.get_logger().info(
                f"[IK] ok={self._n_success} fail={self._n_failure}  "
                f"lat={ik_ms:.1f} ms  {stats_str}")

        if self.dry_run:
            return

        # Build JointTrajectory and publish
        traj = JointTrajectory()
        traj.header = Header()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.cfg["joint_names"]

        pt = JointTrajectoryPoint()
        pt.positions = sol_positions
        # Velocities = 0 at target (controller interpolates)
        pt.velocities = [0.0] * len(sol_positions)
        pt.accelerations = [0.0] * len(sol_positions)

        from builtin_interfaces.msg import Duration as BDuration
        secs = int(self.horizon_sec)
        nsecs = int((self.horizon_sec - secs) * 1e9)
        pt.time_from_start = BDuration(sec=secs, nanosec=nsecs)

        traj.points = [pt]
        self._traj_pub.publish(traj)

    def _fmt_stats(self) -> str:
        """Format recent IK latency statistics."""
        if not self._stats:
            return "(no data)"
        import statistics
        vals = list(self._stats)
        return (f"median={statistics.median(vals):.1f} ms  "
                f"p95={sorted(vals)[int(len(vals)*0.95)]:.1f} ms  "
                f"max={max(vals):.1f} ms")

    def send_home(self):
        """Publish a JointTrajectory to move arm to home position (all zeros)."""
        if self.dry_run or self._traj_pub is None:
            return
        self.get_logger().info(f"[{self.arm}] Sending home trajectory...")
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.cfg["joint_names"]
        pt = JointTrajectoryPoint()
        pt.positions = self.cfg["home_joints"]
        pt.velocities = [0.0] * 7
        from builtin_interfaces.msg import Duration as BDuration
        pt.time_from_start = BDuration(sec=3, nanosec=0)  # 3s to reach home
        traj.points = [pt]
        self._traj_pub.publish(traj)

    # ------------------------------------------------------------------ #
    #  CSV batch helpers
    # ------------------------------------------------------------------ #
    def call_ik_sync(self, x: float, y: float, z: float,
                     quat_xyzw=None) -> tuple:
        """Synchronously call /compute_ik. Returns (success, positions, ik_ms).

        Safe to call from main thread while executor spins in background.
        """
        qx, qy, qz, qw = quat_xyzw if quat_xyzw else self.default_quat

        req = GetPositionIK.Request()
        ik = req.ik_request
        ik.group_name = self.cfg["group_name"]
        ik.avoid_collisions = self.avoid_collisions
        ik.ik_link_name = self.cfg["ee_link"]
        ik.pose_stamped.header.frame_id = self.cfg["base_link"]
        ik.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ik.pose_stamped.pose.position.x = x
        ik.pose_stamped.pose.position.y = y
        ik.pose_stamped.pose.position.z = z
        ik.pose_stamped.pose.orientation.x = qx
        ik.pose_stamped.pose.orientation.y = qy
        ik.pose_stamped.pose.orientation.z = qz
        ik.pose_stamped.pose.orientation.w = qw

        with self._seed_lock:
            seed = self._seed_joints[:]
        robot_state = RobotState()
        robot_state.joint_state.name = self.cfg["joint_names"]
        robot_state.joint_state.position = seed
        ik.robot_state = robot_state

        from builtin_interfaces.msg import Duration as BDuration
        ik.timeout.sec = 0
        ik.timeout.nanosec = int(self.ik_timeout_sec * 1e9)

        t0 = time.perf_counter()
        future = self._ik_client.call_async(req)

        # Poll; background executor processes the response callback
        deadline = t0 + self.ik_timeout_sec + 0.05
        while not future.done():
            if time.perf_counter() > deadline:
                ik_ms = (time.perf_counter() - t0) * 1000.0
                return False, [], ik_ms
            time.sleep(0.001)

        ik_ms = (time.perf_counter() - t0) * 1000.0
        resp = future.result()
        if resp is None or resp.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, [], ik_ms

        sol = resp.solution.joint_state
        positions = []
        for jname in self.cfg["joint_names"]:
            if jname in sol.name:
                positions.append(sol.position[sol.name.index(jname)])
            else:
                positions.append(0.0)

        with self._seed_lock:
            self._seed_joints = positions[:]
        self._latency_pub.publish(Float32(data=float(ik_ms)))
        self._stats.append(ik_ms)
        return True, positions, ik_ms

    def call_ik_any_orient(self, x: float, y: float, z: float) -> tuple:
        """Try all 6 named orientations, return first success.

        This matches ik_reachability_sampler.py's query_any() strategy, so
        the success rate should match the CSV's reachable=1 rows closely.

        Returns (success, positions, total_ik_ms, orient_name, quat_xyzw).
        total_ik_ms = cumulative time across all orientation attempts.
        """
        total_ik_ms = 0.0
        for name, quat in GRASP_ORIENTATIONS.items():
            ok, positions, ik_ms = self.call_ik_sync(x, y, z, quat)
            total_ik_ms += ik_ms
            if ok:
                return True, positions, total_ik_ms, name, quat
        return False, [], total_ik_ms, None, None

    def send_joints_traj(self, positions: list, horizon_sec: float = None):
        """Publish a single-point JointTrajectory to move to `positions`."""
        if self._traj_pub is None:
            return
        if horizon_sec is None:
            horizon_sec = self.horizon_sec
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.cfg["joint_names"]
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.velocities = [0.0] * len(positions)
        pt.accelerations = [0.0] * len(positions)
        from builtin_interfaces.msg import Duration as BDuration
        secs = int(horizon_sec)
        nsecs = int((horizon_sec - secs) * 1e9)
        pt.time_from_start = BDuration(sec=secs, nanosec=nsecs)
        traj.points = [pt]
        self._traj_pub.publish(traj)

    def run_csv_batch(self, pts: list, horizon_ms: float, dwell_sec: float,
                      home_first: bool, quat_xyzw, timing_csv_path: str,
                      do_plot: bool) -> list:
        """Sequentially visit CSV waypoints and record IK latency + wait time.

        For each point:
          1. Call /compute_ik  → measure ik_ms
          2. Publish JointTrajectory with horizon_sec duration
          3. Sleep horizon_sec + dwell_sec  (approximates motion completion)
          4. Record record: x y z success ik_ms wait_ms total_ms

        Returns list of per-point dicts.
        """
        horizon_sec = horizon_ms / 1000.0
        n_total = len(pts)

        if home_first:
            self.get_logger().info("[batch] Going home first (3.5 s wait)...")
            self.send_home()
            time.sleep(3.5)

        records = []
        n_ok = 0

        for i, (*xyz_tuple, recorded_quat) in enumerate(pts):
            x, y, z = xyz_tuple
            self.get_logger().info(
                f"[batch] {i+1}/{n_total}  ({x:.3f}, {y:.3f}, {z:.3f})")

            t_start = time.perf_counter()
            if all_orient:
                ok, joints, ik_ms, orient_name, _ = self.call_ik_any_orient(x, y, z)
            else:
                # Use recorded quaternion if available, else fall back to default
                effective_quat = recorded_quat if recorded_quat is not None else quat_xyzw
                ok, joints, ik_ms = self.call_ik_sync(x, y, z, effective_quat)
                orient_name = None

            if not ok:
                self.get_logger().warn(
                    f"[batch]   FAILED  ik_ms={ik_ms:.1f} ms")
                rec = {"x": x, "y": y, "z": z, "success": 0,
                       "ik_ms": ik_ms, "wait_ms": 0.0, "total_ms": ik_ms}
                if all_orient:
                    rec["orient_name"] = ""
                records.append(rec)
                continue

            n_ok += 1
            if not self.dry_run:
                self.send_joints_traj(joints, horizon_sec)
                time.sleep(horizon_sec + dwell_sec)

            total_ms = (time.perf_counter() - t_start) * 1000.0
            wait_ms = total_ms - ik_ms
            orient_str = f"  orient={orient_name}" if orient_name else ""
            self.get_logger().info(
                f"[batch]   OK  ik={ik_ms:.1f} ms  "
                f"wait={wait_ms:.0f} ms  total={total_ms:.0f} ms{orient_str}")
            rec = {"x": x, "y": y, "z": z, "success": 1,
                   "ik_ms": ik_ms, "wait_ms": wait_ms, "total_ms": total_ms}
            if all_orient:
                rec["orient_name"] = orient_name or ""
            records.append(rec)

        # Summary
        ok_recs = [r for r in records if r["success"]]
        self.get_logger().info(
            f"\n[batch] === DONE ===  {n_ok}/{n_total} succeeded")
        if ok_recs:
            ik_vals = [r["ik_ms"] for r in ok_recs]
            self.get_logger().info(
                f"  IK latency:  "
                f"median={_statistics.median(ik_vals):.1f} ms  "
                f"p95={sorted(ik_vals)[int(len(ik_vals)*0.95)]:.1f} ms  "
                f"max={max(ik_vals):.1f} ms")

        if timing_csv_path:
            _save_timing_csv(records, timing_csv_path)
            self.get_logger().info(f"  Timing CSV → {timing_csv_path}")
        if do_plot and timing_csv_path and ok_recs:
            png_path = str(pathlib.Path(timing_csv_path).with_suffix(".png"))
            _plot_timing(records, self.arm, png_path)
            self.get_logger().info(f"  Plot PNG  → {png_path}")

        return records


# ---------------------------------------------------------------------------
# Batch output helpers
# ---------------------------------------------------------------------------
def _save_timing_csv(records: list, out_path: str):
    if not records:
        return
    fieldnames = ["x", "y", "z", "success", "ik_ms", "wait_ms", "total_ms"]
    if records and "orient_name" in records[0]:
        fieldnames = ["x", "y", "z", "success", "orient_name",
                      "ik_ms", "wait_ms", "total_ms"]
    with open(out_path, "w", newline="") as f:
        w = _csv_mod.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        w.writerows(records)


def _plot_timing(records: list, arm: str, out_path: str):
    ok = [r for r in records if r["success"]]
    if not ok:
        return

    ik_vals = np.array([r["ik_ms"] for r in ok])
    xs = np.array([r["x"] for r in ok])
    ys = np.array([r["y"] for r in ok])
    zs = np.array([r["z"] for r in ok])
    med = float(np.median(ik_vals))
    p95 = float(np.percentile(ik_vals, 95))

    fig = plt.figure(figsize=(15, 5))
    fig.suptitle(
        f"realtime_ik_controller — {arm} arm  "
        f"(n={len(ok)})  median={med:.1f} ms  P95={p95:.1f} ms",
        fontsize=12)

    # 1. IK latency histogram
    ax0 = fig.add_subplot(1, 3, 1)
    ax0.hist(ik_vals, bins=min(30, len(ok)), color="#1976D2",
             edgecolor="white", alpha=0.85)
    ax0.axvline(med, color="#E53935", lw=2, ls="--",
                label=f"median {med:.1f} ms")
    ax0.axvline(p95, color="#FB8C00", lw=1.5, ls=":",
                label=f"P95 {p95:.1f} ms")
    ax0.set_xlabel("/compute_ik latency (ms)")
    ax0.set_ylabel("Count")
    ax0.set_title("IK Latency Distribution")
    ax0.legend(fontsize=8)

    # 2. Timeline
    ax1 = fig.add_subplot(1, 3, 2)
    ax1.plot(range(len(ok)), ik_vals, "o-", ms=3, lw=1, color="#1976D2", alpha=0.8)
    ax1.axhline(med, color="#E53935", lw=1.5, ls="--", label=f"median {med:.1f}")
    ax1.set_xlabel("Waypoint index")
    ax1.set_ylabel("IK latency (ms)")
    ax1.set_title("IK Latency Timeline")
    ax1.legend(fontsize=8)

    # 3. 3-D workspace heatmap
    ax2 = fig.add_subplot(1, 3, 3, projection="3d")
    sc = ax2.scatter(xs, ys, zs, c=ik_vals, cmap="viridis", s=25, alpha=0.85)
    fig.colorbar(sc, ax=ax2, label="IK (ms)", shrink=0.7, pad=0.1)
    ax2.set_xlabel("X (m)", fontsize=8)
    ax2.set_ylabel("Y (m)", fontsize=8)
    ax2.set_zlabel("Z (m)", fontsize=8)
    ax2.set_title("/compute_ik latency\nover workspace")

    plt.tight_layout()
    plt.savefig(out_path, dpi=120, bbox_inches="tight")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="realtime_ik_controller — low-latency IK-based arm control",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--arm", choices=["left", "right"], default="right",
                        help="Which arm to control (default: right)")
    parser.add_argument("--horizon", type=float, default=50.0, metavar="MS",
                        help="Trajectory time horizon in ms (default: 50)")
    parser.add_argument("--quat", type=float, nargs=4, metavar=("QX", "QY", "QZ", "QW"),
                        default=None, help="Default EE orientation qx qy qz qw")
    parser.add_argument("--no-collisions", action="store_true",
                        help="Disable IK collision checking (faster, less safe)")
    parser.add_argument("--home-on-exit", action="store_true",
                        help="Send arm to home on Ctrl-C")
    parser.add_argument("--dry-run", action="store_true",
                        help="Compute IK but do not publish trajectories")
    parser.add_argument("--rate-limit", type=float, default=100.0, metavar="HZ",
                        help="Maximum command publish rate in Hz (default: 100)")
    parser.add_argument("--timeout-ik", type=float, default=0.1, metavar="SEC",
                        help="/compute_ik service timeout in seconds (default: 0.1)")

    # --- CSV batch test mode ---
    batch = parser.add_argument_group("CSV batch test (no VR/AR device needed)")
    batch.add_argument("--csv", metavar="PATH",
                       help="Run batch test from a reachability CSV file")
    batch.add_argument("--max-pts", type=int, default=None, metavar="N",
                       help="Limit to first N CSV points")
    batch.add_argument("--dwell", type=float, default=0.3, metavar="SEC",
                       help="Extra wait after each trajectory horizon (default: 0.3 s)")
    batch.add_argument("--home-first", action="store_true",
                       help="Go to home position before starting CSV run")
    batch.add_argument("--no-sort", action="store_true",
                       help="Do not sort CSV points by Z (default: sort low-to-high)")
    batch.add_argument("--timing-csv", metavar="PATH",
                       help="Output timing CSV path (default: <arm>_ik_batch_timing.csv)")
    batch.add_argument("--all-orient", action="store_true",
                       help="Try all 6 named EE orientations per point (first success wins).\n"
                            "The reachability CSV is generated this way (query_any), so\n"
                            "--all-orient gives ~100%% success on reachable points.\n"
                            "Without this flag, only the default/--quat orientation is tried.")
    batch.add_argument("--no-plot", action="store_true",
                       help="Skip PNG plot generation")

    args = parser.parse_args()

    rclpy.init()
    node = None
    try:
        node = RealtimeIKController(
            arm=args.arm,
            horizon_ms=args.horizon,
            avoid_collisions=not args.no_collisions,
            dry_run=args.dry_run,
            rate_limit_hz=args.rate_limit,
            ik_timeout_sec=args.timeout_ik,
            quat_override=tuple(args.quat) if args.quat else None,
            home_on_exit=args.home_on_exit,
        )

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)

        if args.csv:
            # ── CSV batch mode ──────────────────────────────────────────────
            # Executor spins in a background thread; main thread runs the batch.
            bg = threading.Thread(target=executor.spin, daemon=True)
            bg.start()

            pts = load_reachable_pts(args.csv, sort_by_z=not args.no_sort)
            if args.max_pts:
                pts = pts[: args.max_pts]
            timing_csv = args.timing_csv or f"{args.arm}_ik_batch_timing.csv"
            quat = tuple(args.quat) if args.quat else None

            print(
                f"\n[batch] CSV={args.csv}  pts={len(pts)}  "
                f"horizon={args.horizon:.0f} ms  dwell={args.dwell:.2f} s  "
                f"dry_run={args.dry_run}\n"
            )
            try:
                node.run_csv_batch(
                    pts=pts,
                    horizon_ms=args.horizon,
                    dwell_sec=args.dwell,
                    home_first=args.home_first,
                    quat_xyzw=quat,
                    timing_csv_path=timing_csv,
                    do_plot=not args.no_plot,
                    all_orient=args.all_orient,
                )
            except KeyboardInterrupt:
                print("\n[batch] Interrupted.")
        else:
            # ── Realtime subscriber mode (original) ─────────────────────────
            print(
                f"\n[realtime_ik_controller] Running. "
                f"Send poses to {_ARM_CONFIG[args.arm]['pose_topic']}\n"
                f"  IK latency: ros2 topic echo "
                f"{_ARM_CONFIG[args.arm]['latency_topic']}\n"
                f"  Press Ctrl-C to stop.\n"
            )
            try:
                executor.spin()
            except KeyboardInterrupt:
                pass

    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        raise
    finally:
        if node is not None and node.home_on_exit:
            node.send_home()
            time.sleep(0.3)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
