#!/usr/bin/env python3
import os as _os, sys as _sys; _sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), ".."))
"""
tracker_ee_delta_ik_forward_reach_tf.py — ForwardReachIK + TF2 arm-down init
==============================================================================
★ 合併版本：ForwardReachIK 約束 + TF2 隨機起始位置（手臂初始垂下）★

與其他版本的差異對照：

  tracker_ee_delta_ik_controller_tf.py   — TF2 + plain /compute_ik + 右臂垂下init
  tracker_ee_delta_ik_forward_reach.py   — TF2 + ForwardReachIK + 右臂前伸home
  tracker_ee_delta_ik_forward_reach_tf.py [本檔案]
                                          — TF2 + ForwardReachIK + 右臂垂下init ✓

右臂初始化方式（本檔案）：
  home_joints = [0.0] * 7               ← 全零，手臂自然垂下
  home_pose   = (0, -0.153, 0.262, 1, 0, 0, 0)  ← 與左臂鏡像
  workspace   = X[-0.30,0.30] Y[-0.45,-0.05] Z[0.10,0.65]  ← 含垂下範圍

ForwardReachIK 的作用：
  · 當手臂從垂下移動到工作區域時，IK 自動挑選「肘部朝下、手臂向前」的解
  · 防止「抓背」（j3>0）、防止手臂後擺（j1<-0.5）
  · 工作空間 z_min=0.10 允許垂下起始位置，移動到功能區 (z≥0.35) 後
    ForwardReach 約束開始完全發揮效果

注意：從垂下位置 (j2≈0) 移動時，ForwardReachIK 種子偏向 j2≈1.27（前伸 home），
KDL 需要從高位種子解低位目標，成功率可能低於從前伸 home 開始。
建議使用 --home-first 讓手臂移到前伸位置後再操作。

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Prerequisites：
    ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
    ros2 launch openarm_bimanual_moveit_config move_group_only.launch.py

Dependencies:
    forward_reach_ik_solver.py  (同目錄)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Commands:

  # 右臂鍵盤控制（從垂下位置開始，使用 ForwardReachIK）
  python3 tracker_ee_delta_ik_forward_reach_tf.py --arm right --pattern keyboard

  # 右臂 VR tracker 控制
  python3 tracker_ee_delta_ik_forward_reach_tf.py --arm right --pattern ee_delta

  # 雙臂同時（各開 Terminal）
  python3 tracker_ee_delta_ik_forward_reach_tf.py --arm right --pattern ee_delta &
  python3 tracker_ee_delta_ik_forward_reach_tf.py --arm left  --pattern ee_delta

  # 先回前伸 home 再開始（建議）
  python3 tracker_ee_delta_ik_forward_reach_tf.py --arm right --pattern keyboard --home-first

  # 鍵盤 + 顯示 ForwardReachIK 詳細嘗試（除錯）
  python3 tracker_ee_delta_ik_forward_reach_tf.py --arm right --pattern keyboard --natural-verbose

  # 停用 ForwardReachIK（退回單 seed /compute_ik，對比用）
  python3 tracker_ee_delta_ik_forward_reach_tf.py --arm right --pattern keyboard --no-forward-ik

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Options:
    --arm            left | right  (default: right)
    --pattern        keyboard | sine | circle | lemniscate | topic | ee_delta
    --tracker-side   left | right  (default: same as --arm)
    --calib-yaw      Yaw offset degrees tracker → arm frame (default: 0)
    --calib-rpy      'roll,pitch,yaw' degrees, overrides --calib-yaw
    --no-rot-tracking  Track position only, keep orientation fixed
    --rate           Control loop Hz (default: 20)
    --horizon        JointTrajectory duration ms (default: 60)
    --step           Linear step m (keyboard, default: 0.005)
    --rot-step       Rotation step degrees (keyboard, default: 1.0)
    --radius         Amplitude/radius m for auto patterns (default: 0.04)
    --duration       Auto-pattern duration seconds (default: 30)
    --no-collisions  Disable IK collision checking
    --home-first     Send arm home before starting
    --home-on-exit   Return home on Ctrl-C
    --dry-run        Compute IK but do not publish commands
    --timeout-ik     IK service timeout seconds (default: 0.1)
    --timing-csv     Output timing CSV path (default: delta_ik_timing.csv)
    --no-plot        Skip PNG plot generation
    [ForwardReachIK specific]
    --no-forward-ik  Disable ForwardReachIKSolver (use original single-seed IK)
    --natural-verbose  Print each IK retry attempt score (debug)
"""

import argparse
import csv as _csv_mod
import math
import pathlib
import sys
import threading
import time

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from copy import deepcopy
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# TF2 for getting current EE pose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# ForwardReachIKSolver: multi-seed IK with forward-reach-only enforcement
from forward_reach_ik_solver import ForwardReachIKSolver

# ---------------------------------------------------------------------------
# Arm configuration
# ---------------------------------------------------------------------------
_ARM_CONFIG = {
    "left": {
        "joint_names":   [f"openarm_left_joint{i}" for i in range(1, 8)],
        "base_link":     "world",
        "ee_link":       "openarm_left_link7",
        "group_name":    "left_arm",
        "home_joints":   [0.0] * 7,
        "default_quat":  (1.0, 0.0, 0.0, 0.0),
        # EE at all-joints-zero: arm pointing down, qx=1 (180° around X from left base rpy)
        "home_pose":     (0.0, 0.153, 0.262, 1.0, 0.0, 0.0, 0.0),
        "workspace":     {"x": (-0.30, 0.30), "y": (0.05, 0.45), "z": (0.10, 0.65)},
        "cmd_topic":     "/left_joint_trajectory_controller/joint_trajectory",
        "latency_topic": "/left/delta_ik_latency_ms",
        "delta_topic":   "/pico_left/delta_twist",
    },
    "right": {
        "joint_names":   [f"openarm_right_joint{i}" for i in range(1, 8)],
        "base_link":     "world",
        "ee_link":       "openarm_right_link7",
        "group_name":    "right_arm",
        # arm-down: all joints zero, mirror of left arm (Y negated)
        "home_joints":   [0.0, 0.0, 0.0, 1.570796, 0.0, 0.0, 0.0],
        "default_quat":  (0.707, 0.707, 0.707, 0.0),
        # EE at all-joints-zero: (0, -0.153, 0.262) — mirror of left
        # qx=1 (180° around X) = arm-down EE orientation for right arm
        "home_pose":     (0.2160, -0.1535, 0.4780, 0.7071, 0.0000, 0.7071, 0.0000),
        # Workspace expanded to z_min=0.10 to accommodate arm-down starting position
        "workspace":     {"x": (-0.30, 0.30), "y": (-0.45, -0.05), "z": (0.10, 0.65)},
        "cmd_topic":     "/right_joint_trajectory_controller/joint_trajectory",
        "latency_topic": "/right/delta_ik_latency_ms",
        "delta_topic":   "/pico_right/delta_twist",
    },
}

# ---------------------------------------------------------------------------
# Quaternion utilities
# ---------------------------------------------------------------------------
def quat_multiply(q1, q2):
    """Quaternion multiplication: q1 * q2  (both as (x, y, z, w))."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


def quat_from_rpy(roll, pitch, yaw):
    """Convert roll/pitch/yaw (rad) to quaternion (x, y, z, w)."""
    cr, cp, cy = math.cos(roll/2), math.cos(pitch/2), math.cos(yaw/2)
    sr, sp, sy = math.sin(roll/2), math.sin(pitch/2), math.sin(yaw/2)
    return (
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*sy,
    )


def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (x/n, y/n, z/n, w/n)


def quat_to_rpy(q):
    """Return (roll, pitch, yaw) in radians from quaternion (x,y,z,w)."""
    x, y, z, w = q
    sinr_cosp = 2*(w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2*(w*y - z*x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2*(w*z + x*y)
    cosy_cosp = 1 - 2*(y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def rotate_vec_by_quat(v, q):
    """Rotate vector v (3-tuple) by quaternion q (x,y,z,w) → rotated vec."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    t_x = 2*(qy*vz - qz*vy)
    t_y = 2*(qz*vx - qx*vz)
    t_z = 2*(qx*vy - qy*vx)
    return (
        vx + qw*t_x + qy*t_z - qz*t_y,
        vy + qw*t_y + qz*t_x - qx*t_z,
        vz + qw*t_z + qx*t_y - qy*t_x,
    )


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------
class PicoDeltaIKController(Node):
    """
    Maintains current EE pose, applies deltas, calls /compute_ik (with
    ForwardReachIKSolver for forward-reach-only enforcement), publishes short-horizon
    JointTrajectory to the arm controller.

    Arm starts at all-zeros (arm pointing down), TF2 syncs actual state at startup.
    """

    def __init__(self, args):
        super().__init__("pico_forward_reach_ik_tf_controller")
        self.args = args
        cfg = _ARM_CONFIG[args.arm]
        self.cfg = cfg

        self._cbg = ReentrantCallbackGroup()

        # Current accumulated EE pose: [x, y, z, qx, qy, qz, qw]
        hx, hy, hz, hqx, hqy, hqz, hqw = cfg["home_pose"]
        self._pose = [hx, hy, hz, hqx, hqy, hqz, hqw]

        # Last successful joint solution (used as IK seed)
        self._last_joints = list(cfg["home_joints"])
        self._joints_lock = threading.Lock()

        # Latest joint states from /joint_states
        self._joint_states: dict = {}
        self._js_lock = threading.Lock()

        # ── ForwardReachIKSolver: multi-seed with forward-reach-only enforcement ──
        if not getattr(args, 'no_forward_ik', False):
            self._natural_ik = ForwardReachIKSolver(arm=args.arm, node=self)
            self.get_logger().info(
                f"[ForwardReachIK] enabled for {args.arm} arm "
                f"(verbose={getattr(args,'natural_verbose',False)})"
            )
        else:
            self._natural_ik = None
            self.get_logger().info("[ForwardReachIK] DISABLED — using original single-seed IK")

        # TF2 buffer and listener for getting current EE pose from joint7
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.get_logger().info(
            f"TF2 listener initialized for {cfg['ee_link']} → {cfg['base_link']}"
        )

        # IK service client
        self._ik_client = self.create_client(
            GetPositionIK, "/compute_ik",
            callback_group=self._cbg,
        )

        # Trajectory publisher
        self._traj_pub = self.create_publisher(
            JointTrajectory,
            cfg["cmd_topic"],
            10,
        )

        # Latency publisher
        self._latency_pub = self.create_publisher(
            Float32,
            cfg["latency_topic"],
            10,
        )

        # Joint states subscriber
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_states_cb,
            10,
            callback_group=self._cbg,
        )

        # Delta twist subscriber (for 'topic' pattern = real Pico)
        self._delta_enabled = False
        self._pending_delta = None
        self._delta_lock = threading.Lock()
        if args.pattern == "topic":
            self.create_subscription(
                TwistStamped,
                cfg["delta_topic"],
                self._delta_cb,
                10,
                callback_group=self._cbg,
            )
            self.get_logger().info(
                f"Subscribed to {cfg['delta_topic']} for Pico input"
            )

        # ── ee_delta pattern: subscribe to /ee_delta/{tracker_side} ──────────
        if args.pattern == "ee_delta":
            tracker_side = getattr(args, 'tracker_side', None) or args.arm
            ee_topic = f"/ee_delta/{tracker_side}"
            self.create_subscription(
                PoseStamped,
                ee_topic,
                self._ee_delta_cb,
                10,
                callback_group=self._cbg,
            )
            self.get_logger().info(
                f"[ee_delta] subscribed to {ee_topic}  "
                f"(arm={args.arm}, calib_yaw={getattr(args,'calib_yaw',0.0):.1f}deg)"
            )
            crpy = getattr(args, 'calib_rpy', None)
            if crpy:
                rr, rp, ry = [math.radians(float(v)) for v in crpy.split(',')]
            else:
                rr, rp = 0.0, 0.0
                ry = math.radians(getattr(args, 'calib_yaw', 0.0))
            self._calib_q = quat_from_rpy(rr, rp, ry)

        # Session reference for ee_delta mode
        self._ee_delta_ref_xyz   = None
        self._ee_delta_ref_q     = None
        self._ee_delta_last_t    = None
        self._ee_delta_gap_sec   = 0.35

        # Timing records
        self._timing_records = []

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _joint_states_cb(self, msg: JointState):
        with self._js_lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_states[name] = pos

    def _delta_cb(self, msg: TwistStamped):
        with self._delta_lock:
            self._pending_delta = msg

    def _ee_delta_cb(self, msg: PoseStamped):
        with self._delta_lock:
            self._pending_delta = msg

    # ------------------------------------------------------------------
    # ForwardReachIK seed setter helper
    # ------------------------------------------------------------------
    def _set_ik_seed(self, joints: list):
        """
        Used by ForwardReachIKSolver.call_natural() to swap the seed
        before each IK retry. Writes through _joints_lock so that
        _build_ik_request() picks up the updated seed.
        """
        with self._joints_lock:
            self._last_joints = list(joints)

    # ------------------------------------------------------------------
    # IK core
    # ------------------------------------------------------------------
    def _wait_for_ik_service(self, timeout=10.0):
        t0 = time.time()
        while not self._ik_client.wait_for_service(timeout_sec=0.5):
            if time.time() - t0 > timeout:
                self.get_logger().error("/compute_ik service not available")
                return False
            self.get_logger().info("Waiting for /compute_ik...")
        return True

    def _build_ik_request(self):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.cfg["group_name"]
        req.ik_request.avoid_collisions = not self.args.no_collisions
        req.ik_request.ik_link_name = self.cfg["ee_link"]

        rs = RobotState()
        rs.joint_state.name = list(self.cfg["joint_names"])
        with self._joints_lock:
            rs.joint_state.position = list(self._last_joints)
        req.ik_request.robot_state = rs

        p = Pose()
        p.position.x, p.position.y, p.position.z = (
            self._pose[0], self._pose[1], self._pose[2]
        )
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (
            self._pose[3], self._pose[4], self._pose[5], self._pose[6]
        )

        req.ik_request.pose_stamped.pose = p
        req.ik_request.pose_stamped.header.frame_id = self.cfg["base_link"]
        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()

        t_sec = int(self.args.timeout_ik)
        t_nsec = int((self.args.timeout_ik - t_sec) * 1e9)
        req.ik_request.timeout.sec = t_sec
        req.ik_request.timeout.nanosec = t_nsec
        return req

    def call_ik_sync(self):
        """
        Call /compute_ik synchronously (single seed = current _last_joints).
        Returns (success, joints, ik_ms) or (False, None, ms) on failure.
        """
        req = self._build_ik_request()
        t0 = time.time()
        future = self._ik_client.call_async(req)
        deadline = t0 + self.args.timeout_ik + 0.05
        while not future.done():
            time.sleep(0.005)
            if time.time() > deadline:
                return False, None, (time.time() - t0) * 1000
        ik_ms = (time.time() - t0) * 1000

        resp = future.result()
        if resp is None:
            return False, None, ik_ms
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            if getattr(self.args, 'natural_verbose', False):
                self.get_logger().warn(
                    f"IK error_code={resp.error_code.val} "
                    f"(NO_IK_SOLUTION=-31, PLANNING_FAILED=-1)"
                )
            return False, None, ik_ms

        sol_js = resp.solution.joint_state
        name_to_pos = dict(zip(sol_js.name, sol_js.position))
        joints = [name_to_pos[n] for n in self.cfg["joint_names"]]
        return True, joints, ik_ms

    def _call_ik(self):
        """
        IK dispatcher:
          · ForwardReachIK enabled  → multi-seed with forward-reach-only enforcement
          · ForwardReachIK disabled → original single-seed call_ik_sync()
        Always returns (ok, joints, ik_ms).
        """
        if self._natural_ik is not None:
            return self._natural_ik.call_natural(
                base_ik_fn=self.call_ik_sync,
                current_joints=list(self._last_joints),
                set_seed_fn=self._set_ik_seed,
                verbose=getattr(self.args, 'natural_verbose', False),
            )
        result = self.call_ik_sync()
        if result[0]:
            return result
        return False, None, result[2]

    # ------------------------------------------------------------------
    # Trajectory publisher
    # ------------------------------------------------------------------
    def _publish_trajectory(self, joints: list):
        """Publish a short-horizon JointTrajectory to the arm controller."""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self.cfg["joint_names"])

        pt = JointTrajectoryPoint()
        pt.positions = joints
        pt.velocities = [0.0] * len(joints)
        pt.accelerations = [0.0] * len(joints)

        horizon_sec = self.args.horizon / 1000.0
        pt.time_from_start.sec = int(horizon_sec)
        pt.time_from_start.nanosec = int((horizon_sec % 1) * 1e9)

        msg.points = [pt]
        self._traj_pub.publish(msg)

    def send_home(self):
        """Send arm to home joint configuration (all zeros = arm-down)."""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self.cfg["joint_names"])
        pt = JointTrajectoryPoint()
        pt.positions = list(self.cfg["home_joints"])
        pt.velocities = [0.0] * len(pt.positions)
        pt.time_from_start.sec = 3
        pt.time_from_start.nanosec = 0
        msg.points = [pt]
        self._traj_pub.publish(msg)
        self.get_logger().info("Sent home command (3 s trajectory)")
        hx, hy, hz, hqx, hqy, hqz, hqw = self.cfg["home_pose"]
        self._pose = [hx, hy, hz, hqx, hqy, hqz, hqw]
        with self._joints_lock:
            self._last_joints = list(self.cfg["home_joints"])

    def get_current_ee_from_tf(self, timeout_sec=0.5):
        """
        Get current EE pose via TF2 (base_link → ee_link).
        Returns (x, y, z, qx, qy, qz, qw) or None on failure.
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                self.cfg["base_link"],
                self.cfg["ee_link"],
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec)
            )
            t = transform.transform.translation
            r = transform.transform.rotation
            return (t.x, t.y, t.z, r.x, r.y, r.z, r.w)
        except TransformException as ex:
            self.get_logger().error(
                f"TF lookup failed ({self.cfg['base_link']} → {self.cfg['ee_link']}): {ex}"
            )
            return None

    # ------------------------------------------------------------------
    # Delta application
    # ------------------------------------------------------------------
    def apply_delta_world(self, dx, dy, dz, droll, dpitch, dyaw):
        """Apply positional delta (world frame) + rotational delta (EE frame)."""
        ws = self.cfg["workspace"]
        new_x = max(ws["x"][0], min(ws["x"][1], self._pose[0] + dx))
        new_y = max(ws["y"][0], min(ws["y"][1], self._pose[1] + dy))
        new_z = max(ws["z"][0], min(ws["z"][1], self._pose[2] + dz))
        self._pose[0], self._pose[1], self._pose[2] = new_x, new_y, new_z

        current_q = (self._pose[3], self._pose[4], self._pose[5], self._pose[6])
        delta_q = quat_from_rpy(droll, dpitch, dyaw)
        new_q = quat_normalize(quat_multiply(current_q, delta_q))
        self._pose[3], self._pose[4], self._pose[5], self._pose[6] = new_q

    def set_target_and_send(self, delta_xyz, delta_quat):
        """
        Set arm EE target = session_ref_ee + R_calib * delta_xyz.
        Uses TF for session reference capture (NO drift from home).
        Calls _call_ik() which routes through ForwardReachIKSolver when enabled.
        Returns (success, ik_ms).
        """
        ws = self.cfg["workspace"]

        if self._ee_delta_ref_xyz is not None:
            current_xyz = self._ee_delta_ref_xyz
            current_q   = self._ee_delta_ref_q
        else:
            current_xyz = tuple(self._pose[:3])
            current_q   = tuple(self._pose[3:7])
            self.get_logger().warn(
                "set_target_and_send: no session ref yet, using last commanded pose")

        dx_arm = rotate_vec_by_quat(delta_xyz, self._calib_q)

        new_x = max(ws["x"][0], min(ws["x"][1], current_xyz[0] + dx_arm[0]))
        new_y = max(ws["y"][0], min(ws["y"][1], current_xyz[1] + dx_arm[1]))
        new_z = max(ws["z"][0], min(ws["z"][1], current_xyz[2] + dx_arm[2]))

        if getattr(self.args, 'no_rot_tracking', False):
            new_q = current_q
        else:
            new_q = quat_normalize(quat_multiply(delta_quat, current_q))

        saved_pose = list(self._pose)
        self._pose  = [new_x, new_y, new_z, new_q[0], new_q[1], new_q[2], new_q[3]]

        t_total = time.time()
        ok, joints, ik_ms = self._call_ik()
        if ok:
            with self._joints_lock:
                self._last_joints = joints
            if not self.args.dry_run:
                self._publish_trajectory(joints)
            self._latency_pub.publish(Float32(data=float(ik_ms)))
        else:
            self._pose = saved_pose
            self.get_logger().warn(
                f"IK failed at [{new_x:.3f},{new_y:.3f},{new_z:.3f}] ({ik_ms:.1f}ms)"
            )
        total_ms = (time.time() - t_total) * 1000
        self._timing_records.append({
            "t": time.time(),
            "x": self._pose[0], "y": self._pose[1], "z": self._pose[2],
            "success": int(ok), "ik_ms": ik_ms, "total_ms": total_ms,
            "dx": delta_xyz[0], "dy": delta_xyz[1], "dz": delta_xyz[2],
        })
        return ok, ik_ms

    def step_and_send(self, dx, dy, dz, droll=0.0, dpitch=0.0, dyaw=0.0):
        """
        Apply world-frame delta, call IK (via ForwardReachIKSolver), publish trajectory.
        Returns (success, ik_ms).
        """
        saved_pose = list(self._pose)
        self.apply_delta_world(dx, dy, dz, droll, dpitch, dyaw)

        t_total = time.time()
        ok, joints, ik_ms = self._call_ik()

        if ok:
            with self._joints_lock:
                self._last_joints = joints
            if not self.args.dry_run:
                self._publish_trajectory(joints)
            self._latency_pub.publish(Float32(data=float(ik_ms)))
        else:
            self._pose = saved_pose
            self.get_logger().warn(
                f"IK failed at [{self._pose[0]:.3f}, {self._pose[1]:.3f}, {self._pose[2]:.3f}] "
                f"({ik_ms:.1f} ms)"
            )

        total_ms = (time.time() - t_total) * 1000
        self._timing_records.append({
            "t": time.time(),
            "x": self._pose[0], "y": self._pose[1], "z": self._pose[2],
            "success": int(ok), "ik_ms": ik_ms, "total_ms": total_ms,
            "dx": dx, "dy": dy, "dz": dz,
        })
        return ok, ik_ms

    # ------------------------------------------------------------------
    # Patterns
    # ------------------------------------------------------------------
    def run_pattern_ee_delta(self):
        """Subscribe to /ee_delta/{tracker_side} for VR tracker control."""
        rate_hz = self.args.rate
        dt = 1.0 / rate_hz
        t_start = time.time()
        tracker_side = getattr(self.args, 'tracker_side', None) or self.args.arm

        natural_tag = "ON" if self._natural_ik else "OFF"
        print(f"\n{'='*65}")
        print(f"  EE Delta Mode  |  arm={self.args.arm}  tracker=/ee_delta/{tracker_side}")
        print(f"  ForwardReachIK={natural_tag}  "
              f"calib_yaw={getattr(self.args,'calib_yaw',0.):.1f}deg  "
              f"rot_tracking={not getattr(self.args,'no_rot_tracking',False)}")
        print(f"  workspace X{self.cfg['workspace']['x']}  "
              f"Y{self.cfg['workspace']['y']}  Z{self.cfg['workspace']['z']}")
        print(f"  Waiting for /ee_delta/{tracker_side} messages ...")
        print(f"  (run tracker_ee_delta.py and press SPACE to start recording)")
        print(f"{'='*65}\n")

        while True:
            t_now = time.time()
            with self._delta_lock:
                msg = self._pending_delta
                self._pending_delta = None

            if msg is not None and isinstance(msg, PoseStamped):
                dx = msg.pose.position.x
                dy = msg.pose.position.y
                dz = msg.pose.position.z
                dq = (msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w)
                if abs(dq[3]) < 0.01 and all(abs(v) < 0.01 for v in dq[:3]):
                    dq = (0., 0., 0., 1.)

                t_now_wall = time.time()
                gap = (t_now_wall - self._ee_delta_last_t
                       if self._ee_delta_last_t is not None else 999.0)
                is_new_session = (
                    self._ee_delta_ref_xyz is None or
                    gap > self._ee_delta_gap_sec
                )
                self._ee_delta_last_t = t_now_wall

                if is_new_session:
                    ref = self.get_current_ee_from_tf(timeout_sec=0.3)
                    if ref is not None:
                        self._ee_delta_ref_xyz = tuple(ref[:3])
                        self._ee_delta_ref_q   = tuple(ref[3:7])
                        print(f"\n  [EE-δ] NEW SESSION (gap={gap:.2f}s) | "
                              f"arm_ref=[{ref[0]:.3f},{ref[1]:.3f},{ref[2]:.3f}] (from TF)")
                    else:
                        self._ee_delta_ref_xyz = tuple(self._pose[:3])
                        self._ee_delta_ref_q   = tuple(self._pose[3:7])
                        print(f"\n  [EE-δ] NEW SESSION (gap={gap:.2f}s) | "
                              f"arm_ref=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}] "
                              f"(TF FAILED → last commanded)")

                ok, ik_ms = self.set_target_and_send((dx, dy, dz), dq)
                elapsed = time.time() - t_start
                n = len(self._timing_records)
                sr = sum(r["success"] for r in self._timing_records) / n * 100 if n else 0.
                with self._joints_lock:
                    j3_deg = math.degrees(self._last_joints[2]) if self._last_joints else 0.0
                print(
                    f"\r  t={elapsed:6.1f}s  "
                    f"pos=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}]  "
                    f"j3={j3_deg:+.0f}°  "
                    f"IK={ik_ms:5.1f}ms  {'OK' if ok else 'FAIL'}  sr={sr:.0f}%",
                    end="", flush=True,
                )

            sleep = dt - (time.time() - t_now)
            if sleep > 0:
                time.sleep(sleep)

    def run_pattern_sine(self):
        """Sinusoidal X motion to test IK continuity."""
        freq = 0.3
        amp = self.args.radius
        rate_hz = self.args.rate
        duration = self.args.duration
        dt = 1.0 / rate_hz
        t_start = time.time()
        phase_prev = 0.0

        print(f"\n{'='*60}")
        print(f"  Sine pattern: X ± {amp*100:.1f} cm at {freq} Hz")
        print(f"  ForwardReachIK: {'ON' if self._natural_ik else 'OFF'}")
        print(f"  Press Ctrl-C to stop")
        print(f"{'='*60}\n")

        while time.time() - t_start < duration:
            t_now = time.time()
            elapsed = t_now - t_start
            phase_now = 2 * math.pi * freq * elapsed
            dx = amp * (math.sin(phase_now) - math.sin(phase_prev))
            phase_prev = phase_now

            ok, ik_ms = self.step_and_send(dx, 0.0, 0.0)
            n = len(self._timing_records)
            sr = sum(r["success"] for r in self._timing_records) / n * 100
            with self._joints_lock:
                j3_deg = math.degrees(self._last_joints[2]) if self._last_joints else 0.0
            print(
                f"\r  t={elapsed:5.1f}s  "
                f"pose=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}]  "
                f"j3={j3_deg:+.0f}°  IK={ik_ms:5.1f}ms  "
                f"{'OK' if ok else 'FAIL'}  sr={sr:.0f}%",
                end="", flush=True,
            )
            sleep = dt - (time.time() - t_now)
            if sleep > 0:
                time.sleep(sleep)
        print()

    def run_pattern_circle(self):
        """Circle in Y-Z plane."""
        radius = self.args.radius
        rate_hz = self.args.rate
        duration = self.args.duration
        dt = 1.0 / rate_hz
        omega = 2 * math.pi / 6.0
        t_start = time.time()
        angle_prev = 0.0

        print(f"\n{'='*60}")
        print(f"  Circle: Y-Z plane radius={radius*100:.1f}cm")
        print(f"  ForwardReachIK: {'ON' if self._natural_ik else 'OFF'}")
        print(f"  Press Ctrl-C to stop")
        print(f"{'='*60}\n")

        while time.time() - t_start < duration:
            t_now = time.time()
            elapsed = t_now - t_start
            angle_now = omega * elapsed
            dy = radius * (math.cos(angle_now) - math.cos(angle_prev))
            dz = radius * (math.sin(angle_now) - math.sin(angle_prev))
            angle_prev = angle_now

            ok, ik_ms = self.step_and_send(0.0, dy, dz)
            n = len(self._timing_records)
            sr = sum(r["success"] for r in self._timing_records) / n * 100
            with self._joints_lock:
                j3_deg = math.degrees(self._last_joints[2]) if self._last_joints else 0.0
            print(
                f"\r  t={elapsed:5.1f}s  "
                f"pose=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}]  "
                f"j3={j3_deg:+.0f}°  IK={ik_ms:5.1f}ms  "
                f"{'OK' if ok else 'FAIL'}  sr={sr:.0f}%",
                end="", flush=True,
            )
            sleep = dt - (time.time() - t_now)
            if sleep > 0:
                time.sleep(sleep)
        print()

    def run_pattern_lemniscate(self):
        """Lemniscate (figure-8) in X-Y plane."""
        scale = self.args.radius
        rate_hz = self.args.rate
        duration = self.args.duration
        dt = 1.0 / rate_hz
        omega = 2 * math.pi / 10.0
        t_start = time.time()
        prev_angle = 0.0

        print(f"\n{'='*60}")
        print(f"  Lemniscate (figure-8): X-Y plane scale={scale*100:.1f}cm")
        print(f"  ForwardReachIK: {'ON' if self._natural_ik else 'OFF'}")
        print(f"  Press Ctrl-C to stop")
        print(f"{'='*60}\n")

        while time.time() - t_start < duration:
            t_now = time.time()
            elapsed = t_now - t_start
            angle = omega * elapsed

            def lemniscate_xy(theta):
                denom = 1 + math.sin(theta)**2
                return (scale * math.cos(theta) / denom,
                        scale * math.sin(theta) * math.cos(theta) / denom)

            x_now, y_now = lemniscate_xy(angle)
            x_prev, y_prev = lemniscate_xy(prev_angle)
            dx = x_now - x_prev
            dy = y_now - y_prev
            prev_angle = angle

            ok, ik_ms = self.step_and_send(dx, dy, 0.0)
            n = len(self._timing_records)
            sr = sum(r["success"] for r in self._timing_records) / n * 100
            print(
                f"\r  t={elapsed:5.1f}s  "
                f"pose=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}]  "
                f"IK={ik_ms:5.1f}ms  {'OK' if ok else 'FAIL'}  sr={sr:.0f}%",
                end="", flush=True,
            )
            sleep = dt - (time.time() - t_now)
            if sleep > 0:
                time.sleep(sleep)
        print()

    def run_pattern_topic(self):
        """Read delta from /pico_right/delta_twist topic (real Pico VR input)."""
        rate_hz = self.args.rate
        dt = 1.0 / rate_hz
        t_start = time.time()

        print(f"\n{'='*60}")
        print(f"  Pico VR Topic mode: listening on {self.cfg['delta_topic']}")
        print(f"  ForwardReachIK: {'ON' if self._natural_ik else 'OFF'}")
        print(f"  Press Ctrl-C to stop")
        print(f"{'='*60}\n")

        while True:
            t_now = time.time()
            with self._delta_lock:
                msg = self._pending_delta
                self._pending_delta = None

            if msg is not None:
                dx = msg.twist.linear.x
                dy = msg.twist.linear.y
                dz = msg.twist.linear.z
                droll  = msg.twist.angular.x
                dpitch = msg.twist.angular.y
                dyaw   = msg.twist.angular.z
                if abs(dx)+abs(dy)+abs(dz)+abs(droll)+abs(dpitch)+abs(dyaw) > 1e-6:
                    ok, ik_ms = self.step_and_send(dx, dy, dz, droll, dpitch, dyaw)
                    elapsed = time.time() - t_start
                    print(
                        f"\r  t={elapsed:5.1f}s  "
                        f"pose=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}]  "
                        f"IK={ik_ms:5.1f}ms  {'OK' if ok else 'FAIL'}",
                        end="", flush=True,
                    )
            else:
                time.sleep(0.005)

            sleep = dt - (time.time() - t_now)
            if sleep > 0:
                time.sleep(sleep)

    def run_pattern_keyboard(self):
        """Interactive keyboard control simulating Pico joystick."""
        import select
        import termios
        import tty

        # ── 同步初始狀態從實際機器人讀取 ──────────────────────────────────
        # TF2 sync: 取得實際 EE 姿態，避免因 home_pose 硬碼值與 FK 不一致導致 IK 失敗
        print("  同步 EE 姿態從 TF2...")
        time.sleep(0.5)  # 等待 TF buffer 充入
        tf_pose = self.get_current_ee_from_tf(timeout_sec=2.0)
        if tf_pose is not None:
            self._pose = list(tf_pose)
            print(f"  TF EE: x={tf_pose[0]:.4f} y={tf_pose[1]:.4f} z={tf_pose[2]:.4f}")
            print(f"  TF quat: [{tf_pose[3]:.3f}, {tf_pose[4]:.3f}, {tf_pose[5]:.3f}, {tf_pose[6]:.3f}]")
        else:
            print("  ⚠ TF 未取得 — 使用 hardcoded home_pose")
        with self._js_lock:
            js_snap = dict(self._joint_states)
        jnames = self.cfg["joint_names"]
        if all(n in js_snap for n in jnames):
            actual_joints = [js_snap[n] for n in jnames]
            with self._joints_lock:
                self._last_joints = actual_joints
            print(f"  /joint_states: {[f'{v:.3f}' for v in actual_joints]}")
        else:
            print("  ⚠ /joint_states 未就緒 — 使用 hardcoded home_joints")
        print()

        step = self.args.step
        rot_step = math.radians(self.args.rot_step)

        KEY_MAP = {
            'w': ( step,   0,     0,     0,        0,        0      ),
            's': (-step,   0,     0,     0,        0,        0      ),
            'a': ( 0,      step,  0,     0,        0,        0      ),
            'd': ( 0,     -step,  0,     0,        0,        0      ),
            'q': ( 0,      0,     step,  0,        0,        0      ),
            'e': ( 0,      0,    -step,  0,        0,        0      ),
            'u': ( 0,      0,     0,     rot_step,  0,        0      ),
            'j': ( 0,      0,     0,    -rot_step,  0,        0      ),
            'i': ( 0,      0,     0,     0,      rot_step,   0      ),
            'k': ( 0,      0,     0,     0,     -rot_step,   0      ),
            'o': ( 0,      0,     0,     0,         0,    rot_step  ),
            'l': ( 0,      0,     0,     0,         0,   -rot_step  ),
        }

        print(f"\n{'='*60}")
        print(f"  Keyboard mode — ForwardReachIK={'ON' if self._natural_ik else 'OFF'}")
        print(f"  Linear step: {step*1000:.1f} mm | Rotation step: {math.degrees(rot_step):.1f}°")
        print()
        print(f"  Position (world frame):  W/S=+X/-X  A/D=+Y/-Y  Q/E=+Z/-Z")
        print(f"  Rotation (EE frame):     U/J=Roll   I/K=Pitch  O/L=Yaw")
        print(f"  Step size:               + = larger   - = smaller")
        print(f"  R = reset to home  |  P = print pose  |  Ctrl-C = quit")
        ws = self.cfg["workspace"]
        print(f"\n  Workspace: X{ws['x']}  Y{ws['y']}  Z{ws['z']}")
        print(f"{'='*60}")
        print(f"\n  Current EE: [{self._pose[0]:.3f}, {self._pose[1]:.3f}, {self._pose[2]:.3f}]")

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            while True:
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if r:
                    ch = sys.stdin.read(1)
                    if ch == '\x03':
                        raise KeyboardInterrupt
                    elif ch in KEY_MAP:
                        dx, dy, dz, dr, dp, dyw = KEY_MAP[ch]
                        ok, ik_ms = self.step_and_send(dx, dy, dz, dr, dp, dyw)
                        r_deg, p_deg, y_deg = [math.degrees(v) for v in quat_to_rpy(
                            (self._pose[3], self._pose[4], self._pose[5], self._pose[6])
                        )]
                        with self._joints_lock:
                            j3_deg = math.degrees(self._last_joints[2]) if self._last_joints else 0.0
                        print(
                            f"\r  [{ch}] pos=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}]"
                            f" rpy=[{r_deg:.1f}°,{p_deg:.1f}°,{y_deg:.1f}°]"
                            f" j3={j3_deg:+.0f}°"
                            f" IK={ik_ms:5.1f}ms {'✓' if ok else '✗'}"
                            + " " * 5,
                            flush=True,
                        )
                    elif ch == '+':
                        step *= 1.5
                        rot_step *= 1.5
                        for k in KEY_MAP:
                            KEY_MAP[k] = tuple(v * 1.5 if abs(v) > 0 else v for v in KEY_MAP[k])
                        print(f"\n  [+] step={step*1000:.1f}mm")
                    elif ch == '-':
                        step /= 1.5
                        rot_step /= 1.5
                        for k in KEY_MAP:
                            KEY_MAP[k] = tuple(v / 1.5 if abs(v) > 0 else v for v in KEY_MAP[k])
                        print(f"\n  [-] step={step*1000:.1f}mm")
                    elif ch.lower() == 'r':
                        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                        print(f"\n  [R] Resetting to home (arm-down)...")
                        self.send_home()
                        time.sleep(3.5)
                        tty.setraw(fd)
                        print(f"  Home reached.")
                    elif ch.lower() == 'p':
                        r_deg, p_deg, y_deg = [math.degrees(v) for v in quat_to_rpy(
                            (self._pose[3], self._pose[4], self._pose[5], self._pose[6])
                        )]
                        with self._joints_lock:
                            joints_copy = list(self._last_joints)
                        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                        print(f"\n  Pose: x={self._pose[0]:.4f} y={self._pose[1]:.4f} z={self._pose[2]:.4f}")
                        print(f"  RPY:  roll={r_deg:.2f}° pitch={p_deg:.2f}° yaw={y_deg:.2f}°")
                        print(f"  Joints: {[f'{v:.3f}' for v in joints_copy]}")
                        if self._natural_ik and joints_copy:
                            self._natural_ik.print_diagnosis(joints_copy, label="current")
                        tty.setraw(fd)
                else:
                    time.sleep(0.005)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # ------------------------------------------------------------------
    # Results save
    # ------------------------------------------------------------------
    def save_results(self):
        if not self._timing_records:
            return
        csv_path = pathlib.Path(self.args.timing_csv)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with open(csv_path, "w", newline="") as f:
            writer = _csv_mod.DictWriter(f, fieldnames=self._timing_records[0].keys())
            writer.writeheader()
            writer.writerows(self._timing_records)
        self.get_logger().info(f"Saved {len(self._timing_records)} records → {csv_path}")

        if self.args.no_plot:
            return
        try:
            self._plot_results(csv_path)
        except Exception as e:
            self.get_logger().warn(f"Plot failed: {e}")

    def _plot_results(self, csv_path: pathlib.Path):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import statistics

        ik_times = [r["ik_ms"] for r in self._timing_records if r["success"]]
        if not ik_times:
            return

        fig, axes = plt.subplots(1, 3, figsize=(15, 4))
        natural_tag = "ForwardReachIK=ON" if self._natural_ik else "OFF"
        fig.suptitle(
            f"ForwardReachIK-TF — {self.args.arm} arm  [{self.args.pattern}]  {natural_tag}  "
            f"n={len(ik_times)}  "
            f"success={sum(r['success'] for r in self._timing_records)/len(self._timing_records)*100:.1f}%"
        )

        axes[0].hist(ik_times, bins=30, color="steelblue", edgecolor="white")
        axes[0].axvline(statistics.median(ik_times), color="red",
                        label=f"median={statistics.median(ik_times):.1f}ms")
        axes[0].set_xlabel("IK latency (ms)")
        axes[0].set_ylabel("Count")
        axes[0].set_title("IK Latency Distribution")
        axes[0].legend()

        t0 = self._timing_records[0]["t"]
        ts = [r["t"] - t0 for r in self._timing_records]
        ik_all = [r["ik_ms"] for r in self._timing_records]
        colors = ["green" if r["success"] else "red" for r in self._timing_records]
        axes[1].scatter(ts, ik_all, c=colors, s=10, alpha=0.7)
        axes[1].set_xlabel("Time (s)")
        axes[1].set_ylabel("IK latency (ms)")
        axes[1].set_title("IK Latency Timeline")

        from mpl_toolkits.mplot3d import Axes3D
        fig2 = plt.figure(figsize=(6, 5))
        ax3 = fig2.add_subplot(111, projection="3d")
        xs = [r["x"] for r in self._timing_records if r["success"]]
        ys = [r["y"] for r in self._timing_records if r["success"]]
        zs = [r["z"] for r in self._timing_records if r["success"]]
        ik_c = [r["ik_ms"] for r in self._timing_records if r["success"]]
        sc = ax3.scatter(xs, ys, zs, c=ik_c, cmap="viridis", s=5)
        fig2.colorbar(sc, ax=ax3, label="IK ms")
        ax3.set_xlabel("X"); ax3.set_ylabel("Y"); ax3.set_zlabel("Z")
        ax3.set_title("EE Trajectory (colored by IK ms)")

        png_path = csv_path.with_suffix(".png")
        fig.tight_layout()
        fig.savefig(str(png_path), dpi=120, bbox_inches="tight")
        fig2.savefig(str(png_path).replace(".png", "_3d.png"), dpi=120, bbox_inches="tight")
        plt.close("all")
        self.get_logger().info(f"Plots saved → {png_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(
        description="Pico VR delta IK controller — ForwardReachIK + TF2 arm-down init",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--arm", default="right", choices=["left", "right"])
    p.add_argument("--pattern", default="ee_delta",
                   choices=["keyboard", "sine", "circle", "lemniscate", "topic", "ee_delta"])
    p.add_argument("--tracker-side", default=None,
                   help="tracker topic side: left|right (default: same as --arm)")
    p.add_argument("--calib-yaw", type=float, default=0.0,
                   help="Yaw offset degrees tracker → arm frame (default: 0)")
    p.add_argument("--calib-rpy", type=str, default=None,
                   help="'roll,pitch,yaw' degrees, overrides --calib-yaw")
    p.add_argument("--no-rot-tracking", action="store_true",
                   help="Track position only, keep orientation fixed")
    p.add_argument("--rate", type=float, default=20.0,
                   help="Control loop Hz (default: 20)")
    p.add_argument("--horizon", type=float, default=60.0,
                   help="JointTrajectory horizon ms (default: 60)")
    p.add_argument("--step", type=float, default=0.005,
                   help="Linear step m (keyboard, default: 0.005)")
    p.add_argument("--rot-step", type=float, default=1.0,
                   help="Rotation step degrees (keyboard, default: 1.0)")
    p.add_argument("--radius", type=float, default=0.04,
                   help="Amplitude/radius m for auto patterns (default: 0.04)")
    p.add_argument("--duration", type=float, default=30.0,
                   help="Auto-pattern duration seconds (default: 30)")
    p.add_argument("--no-collisions", action="store_true",
                   help="Disable IK collision checking")
    p.add_argument("--home-first", action="store_true",
                   help="Send arm home before starting")
    p.add_argument("--home-on-exit", action="store_true",
                   help="Return home on Ctrl-C")
    p.add_argument("--dry-run", action="store_true",
                   help="Compute IK but do not publish to controller")
    p.add_argument("--timeout-ik", type=float, default=0.1,
                   help="IK service timeout seconds (default: 0.1)")
    p.add_argument("--timing-csv", default="delta_ik_timing.csv",
                   help="Output timing CSV path")
    p.add_argument("--no-plot", action="store_true",
                   help="Skip PNG plot generation")
    # ── ForwardReachIK flags ─────────────────────────────────────────────────
    p.add_argument("--no-forward-ik", action="store_true",
                   help="Disable ForwardReachIKSolver (use original single-seed IK)")
    p.add_argument("--natural-verbose", action="store_true",
                   help="Print each ForwardReachIK retry attempt score (debug)")
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = PicoDeltaIKController(args)

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    natural_tag = "ON" if not getattr(args, 'no_forward_ik', False) else "OFF"
    print(f"\n  Pico Delta IK Controller — ForwardReachIK={natural_tag} + TF2 arm-down init")
    print(f"  ARM: {args.arm} | PATTERN: {args.pattern} | RATE: {args.rate}Hz")
    print(f"  Horizon: {args.horizon}ms | Collisions: {not args.no_collisions}")
    print(f"  Dry-run: {args.dry_run}")

    try:
        if not node._wait_for_ik_service():
            print("\nERROR: /compute_ik not available. Is move_group running?")
            return

        print("  /compute_ik service ready ✓")

        if args.home_first:
            print("\n  Moving to home position (arm-down)...")
            node.send_home()
            time.sleep(3.5)

        if args.pattern == "keyboard":
            node.run_pattern_keyboard()
        elif args.pattern == "sine":
            node.run_pattern_sine()
        elif args.pattern == "circle":
            node.run_pattern_circle()
        elif args.pattern == "lemniscate":
            node.run_pattern_lemniscate()
        elif args.pattern == "topic":
            node.run_pattern_topic()
        elif args.pattern == "ee_delta":
            node.run_pattern_ee_delta()

    except KeyboardInterrupt:
        print("\n\n  Interrupted.")
    finally:
        records = node._timing_records
        if records:
            n = len(records)
            success_n = sum(r["success"] for r in records)
            ik_times = [r["ik_ms"] for r in records if r["success"]]
            import statistics
            print(f"\n{'='*60}")
            print(f"  Summary ({args.arm} arm, {args.pattern})")
            print(f"  Total commands : {n}")
            print(f"  Success rate   : {success_n}/{n} ({success_n/n*100:.1f}%)")
            if ik_times:
                print(f"  IK latency     : "
                      f"median={statistics.median(ik_times):.1f}ms  "
                      f"mean={statistics.mean(ik_times):.1f}ms  "
                      f"p99={sorted(ik_times)[int(len(ik_times)*0.99)]:.1f}ms  "
                      f"max={max(ik_times):.1f}ms")
                if len(records) > 1:
                    print(f"  Effective rate : "
                          f"{n / (records[-1]['t'] - records[0]['t']):.1f} Hz")

            # ── ForwardReachIK statistics ────────────────────────────────────
            if node._natural_ik is not None:
                stats = node._natural_ik.get_stats()
                total = stats["total"]
                first = stats["natural_first"]
                retried = stats["retried"]
                failed = stats["failed"]
                first_pct = first / total * 100 if total else 0.0
                print(f"\n  ForwardReachIK stats:")
                print(f"    Total IK calls  : {total}")
                print(f"    Natural 1st-try : {first}  ({first_pct:.1f}%)")
                print(f"    Retried         : {retried}")
                print(f"    Failed (all)    : {failed}")
                if total > 0 and first_pct < 80:
                    print(f"    ⚠ Low first-try rate — consider widening j3 soft_hi")
            print(f"{'='*60}")

        if args.home_on_exit:
            print("  Sending home...")
            node.send_home()
            time.sleep(3.5)

        node.save_results()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
