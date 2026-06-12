#!/usr/bin/env python3
import os as _os, sys as _sys; _sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), ".."))
"""
tracker_ee_delta_ik_controller.py — tracker_ee_delta → OpenArm IK controller
=============================================================================
模擬 / 接收 Pico VR 搖桿 的「相對位移 + 旋轉增量」，透過 /compute_ik
（不使用 OMPL）直接控制 OpenArm 機械臂。

架構：
    Pico VR 搖桿（未來）
         │  按住板機鍵 → 手柄移動 → 發布 TwistStamped delta
         │  OR
    鍵盤模擬器（現在，--pattern keyboard）
         │
         ▼
    pico_delta_ik_controller.py
         │  accumulated_pose += apply_delta(delta)
         │  /compute_ik (seed = last joint solution, 1–20 ms)
         │  短時程 JointTrajectory (--horizon ms)
         ▼
    /right_arm_controller/joint_trajectory
         ▼
    OpenArm O6

為什麼選 Pilz PTP（而非 OMPL）：
    - OMPL motion 中位數 1323 ms → 搖桿回饋嚴重滯後
    - Pilz PTP planning 中位數 3.9 ms，小步 motion ~50–100 ms → ~10 Hz 可控
    - BUT 本腳本不使用 Pilz！改用 /compute_ik + 短時程 JointTrajectory：
        planning < 10 ms, motion = horizon (50ms) → 可達 20 Hz
    - 搭配 IK seed（上一步解）→ 防止多解跳動

✅ 為什麼不用 realtime_ik_controller.py：
    接收「絕對 PoseStamped」，搖桿輸出「相對增量」，架構不符。
    本腳本內部維護累積絕對位姿，從外部接收相對增量。

📌 支援的 Input 模式（--pattern）：
    keyboard   : 鍵盤即時控制（WASD + QE = XYZ，UIOJKL = RPY，空白 = 停止）
    sine       : 自動 X 軸正弦波（無硬體測試）
    circle     : Y-Z 平面圓形（無硬體測試）
    lemniscate : X-Y 平面 8 字形（無硬體測試）
    topic      : 訂閱 /pico/delta_twist（來自真實 Pico VR 橋接節點）
    ee_delta   : 訂閱 /ee_delta/{tracker-side} (PoseStamped, from tracker_ee_delta.py) [NEW]

    == ee_delta 模式說明 ==
    tracker_ee_delta.py 發布的 /ee_delta/{side} (geometry_msgs/PoseStamped):
        .pose.position    = delta XYZ from reference [m]  (tracker frame)
        .pose.orientation = delta quaternion from reference
        .header.frame_id  = "tracker_delta"

    信號鏈：
        tracker_ee_delta.py ──► /ee_delta/{tracker-side} ──► THIS node
            arm_target_xyz  = arm_home_xyz  + R_calib * delta_xyz
            arm_target_quat = arm_home_quat * delta_quat   (if --rot-tracking)
            → /compute_ik → JointTrajectory → arm controller

    重要：tracker_ee_delta 送的是「離 reference 的累積總 delta」（非每幀增量），
    所以 arm 目標 = home + delta（無漂移），recording 停止後 arm 維持最後位置。

    座標系校準（--calib-yaw）：
        tracker 站在手臂正前方時通常不需要校準（calib-yaw=0）。
        若 tracker 方向與 arm X 軸不對齊，調整 --calib-yaw（度）。
        完整校準：--calib-rpy roll,pitch,yaw（度）。

    啟動方式（雙臂）：
        # Terminal 1 — tracker 側
        python tracker_ee_delta.py --tracker both --filter-hz 10 --scale 0.5

        # Terminal 2 — right arm IK
        python tracker_ee_delta_ik_controller.py --arm right --pattern ee_delta --home-first

        # Terminal 3 — left arm IK
        python tracker_ee_delta_ik_controller.py --arm left  --pattern ee_delta --home-first

    scale 建議初始值 0.3~0.5（避免 arm 移動過快）。
    本節點訂閱後即可直接控制。
    按住板機鍵 = enable_topic = True 才更新位姿（防止手柄靜止時不斷累積誤差）。

🦾 人體手臂可動範圍（Human Arm ROM）→ 工作空間設計依據：
    OpenArm 世界座標系：X=前方, Y=左方, Z=上方
    右臂肩關節位於 world 約 y ≈ -0.2

    工作空間限制（防止反轉象限 / 不自然姿態）：
        X : [-0.10,  0.45]  前伸 (達 45cm)，可稍微後縮
        Y : [-0.55, -0.05]  右側專用，絕不越過身體中線
        Z : [ 0.25,  0.80]  腰部到頭頂

    典型人體動作對應點位（右臂）：
        姿態              X       Y       Z     動作說明
        ─────────────────────────────────────────────────────
        自然垂放           0.10  -0.25   0.30  手臂自然放下
        準備位 (home)      0.23  -0.25   0.54  前伸到肚臍高度 ← 預設
        前伸平舉           0.40  -0.20   0.54  水平前伸
        側舉 (外展)        0.10  -0.50   0.54  手臂側向展開
        舉手               0.15  -0.20   0.75  手舉高過頭
        抓取桌面           0.35  -0.25   0.30  向下抓取

    EE 方向（四元數 qx qy qz qw）：
        手掌朝下·指向前方  (0.0,  0.0, -0.707, 0.707)  ← right arm 預設
        手掌朝下·指向左方  (0.0,  0.0,  0.000, 1.000)
        手掌朝上·指向前方  (0.707, 0.0, 0.0,   0.707)

    避免反轉象限的策略：
        1. 工作空間 Y 軸強制限制在右側（不允許越過中線）
        2. IK seed = 上一步解（確保連續性，不跳到另一組解）
        3. 工作空間 clamp 在 apply_delta_world() 中已實作

使用方式：
    # 鍵盤模擬（預設右臂）
    python3 pico_delta_ik_controller.py --arm right --pattern keyboard

    # 自動正弦波測試（不需硬體）
    python3 pico_delta_ik_controller.py --arm right --pattern sine --duration 20

    # 圓形測試（Y-Z 平面）
    python3 pico_delta_ik_controller.py --arm right --pattern circle --radius 0.05

    # 8 字形測試
    python3 pico_delta_ik_controller.py --arm right --pattern lemniscate

    # 接收真實 Pico 訊號（需 Pico 橋接節點）
    python3 pico_delta_ik_controller.py --arm right --pattern topic

    # 不撞碰撞（速度快，確保工作區域清空）
    python3 pico_delta_ik_controller.py --arm right --pattern sine --no-collisions

    # 停用碰撞並加快率（unsafe but fast）
    python3 pico_delta_ik_controller.py --arm right --pattern circle --rate 20 --no-collisions

    # 模擬時存入 CSV（方便後續分析）
    python3 pico_delta_ik_controller.py --arm right --pattern circle --duration 15 --timing-csv delta_ik_circle.csv

鍵盤操作說明（--pattern keyboard）：
    移動（世界座標系）：
        W / S  : +X / -X
        A / D  : +Y / -Y
        Q / E  : +Z / -Z
    旋轉（EE 座標系）:
        U / J  : +Roll / -Roll
        I / K  : +Pitch / -Pitch
        O / L  : +Yaw / -Yaw
    步進大小：
        +      : 增加步長 (×1.5)
        -      : 縮小步長 (÷1.5)
    其他：
        R      : 重置到 home 位置
        P      : 列印目前 EE 位姿
        Ctrl-C : 退出

Prerequisites（必須先啟動）：
    ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
    ros2 launch openarm_bimanual_moveit_config move_group_only.launch.py

Options:
    --arm            left | right  (default: right)
    --pattern        keyboard | sine | circle | lemniscate | topic  (default: keyboard)
    --rate           Control loop Hz (default: 20)
    --horizon        JointTrajectory duration in ms (default: 60)
    --step           Linear step size in meters per cycle (default: 0.005)
    --rot-step       Rotation step in degrees per cycle (default: 1.0)
    --radius         Radius for circle/lemniscate in meters (default: 0.04)
    --duration       Auto-pattern duration seconds (default: 30)
    --no-collisions  Disable IK collision checking (faster, less safe)
    --home-first     Move to home before starting
    --home-on-exit   Return home on Ctrl-C
    --dry-run        Compute IK but do NOT publish to controller
    --timing-csv P   Save per-command timing CSV (default: delta_ik_timing.csv)
    --no-plot        Skip PNG generation on finish
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
        "default_quat":  (0.0, 0.0, 0.707, 0.707),
        # EE at all-joints-zero: (0, 0.153, 0.262) mirrored from right
        # IMPORTANT: re-run ik_reachability_sampler.py if workspace seems wrong
        "home_pose":     (0.0, 0.153, 0.262, 1.0, 0.0, 0.0, 0.0),
        # Tentative limits — update after running ik_reachability_sampler.py
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
        # IK-solved home: (0.0, -0.25, 0.45) qx=1, verified 500ms timeout from zeros seed
        "home_joints":   [0.0, 1.2735, -1.5708, 1.8287, 1.5708, -0.5552, 0.0],
        "default_quat":  (1.0, 0.0, 0.0, 0.0),
        # Central workspace home (v10 URDF), confirmed by /compute_ik
        # EE arm-down orientation: qx=1 (180° around X)
        "home_pose":     (0.0, -0.25, 0.45, 1.0, 0.0, 0.0, 0.0),
        # Workspace for v10 URDF, shoulder at (0, -0.031, 0.698)
        "workspace":     {"x": (-0.20, 0.25), "y": (-0.45, -0.10), "z": (0.25, 0.65)},
        "cmd_topic":     "/right_joint_trajectory_controller/joint_trajectory",
        "latency_topic": "/right/delta_ik_latency_ms",
        "delta_topic":   "/pico_right/delta_twist",
    },
}

# ---------------------------------------------------------------------------
# Quaternion utilities (no tf2 import needed for core math)
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
    # Sandwich product: q * [0,v] * q^-1 (conjugate for unit quat)
    # Expand: v' = v + 2*qw*(q_vec × v) + 2*(q_vec × (q_vec × v))
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
    Maintains current EE pose, applies deltas, calls /compute_ik, publishes
    short-horizon JointTrajectory commands to the arm controller.
    """

    def __init__(self, args):
        super().__init__("pico_delta_ik_controller")
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

        # Joint states subscriber (to seed IK from real robot state)
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_states_cb,
            10,
            callback_group=self._cbg,
        )

        # Delta twist subscriber (for 'topic' pattern = real Pico)
        self._delta_enabled = False   # gate: true when trigger pressed
        self._pending_delta = None    # latest msg from Pico or tracker
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
            # Build calibration quaternion from --calib-rpy (or --calib-yaw shortcut)
            crpy = getattr(args, 'calib_rpy', None)
            if crpy:
                rr, rp, ry = [math.radians(float(v)) for v in crpy.split(',')]
            else:
                rr, rp = 0.0, 0.0
                ry = math.radians(getattr(args, 'calib_yaw', 0.0))
            self._calib_q = quat_from_rpy(rr, rp, ry)  # tracker→arm frame rotation

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
        """Real Pico VR delta twist callback."""
        with self._delta_lock:
            self._pending_delta = msg

    def _ee_delta_cb(self, msg: PoseStamped):
        """tracker_ee_delta PoseStamped callback: stores latest delta."""
        with self._delta_lock:
            self._pending_delta = msg

    # ------------------------------------------------------------------
    # IK + Command publish
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
        from builtin_interfaces.msg import Duration as BuiltinDuration
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.cfg["group_name"]
        req.ik_request.avoid_collisions = not self.args.no_collisions
        req.ik_request.ik_link_name = self.cfg["ee_link"]

        # Seed from last successful solution
        rs = RobotState()
        rs.joint_state.name = list(self.cfg["joint_names"])
        with self._joints_lock:
            rs.joint_state.position = list(self._last_joints)
        req.ik_request.robot_state = rs

        # Target EE pose
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

        # Timeout for IK solver
        t_sec = int(self.args.timeout_ik)
        t_nsec = int((self.args.timeout_ik - t_sec) * 1e9)
        req.ik_request.timeout.sec = t_sec
        req.ik_request.timeout.nanosec = t_nsec
        return req

    def call_ik_sync(self):
        """Call /compute_ik synchronously, return (success, joints, ik_ms)."""
        req = self._build_ik_request()
        t0 = time.time()
        future = self._ik_client.call_async(req)
        # NOTE: do NOT call rclpy.spin_once() here — the MultiThreadedExecutor
        # spin thread is already running and will resolve the future.
        # Calling spin_once() from the main thread pollutes the executor and
        # causes callbacks to never fire (same bug as plan() in pymoveit2).
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
            return False, None, ik_ms

        # The solution contains ALL robot joints (both arms + hands = 36 joints).
        # Extract only the target arm joints by name to avoid reading wrong arm's values.
        sol_js = resp.solution.joint_state
        name_to_pos = dict(zip(sol_js.name, sol_js.position))
        joints = [name_to_pos[n] for n in self.cfg["joint_names"]]
        return True, joints, ik_ms

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
        """Send arm to home joint configuration."""
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
        # Reset to home pose
        hx, hy, hz, hqx, hqy, hqz, hqw = self.cfg["home_pose"]
        self._pose = [hx, hy, hz, hqx, hqy, hqz, hqw]
        with self._joints_lock:
            self._last_joints = list(self.cfg["home_joints"])

    # ------------------------------------------------------------------
    # Delta application
    # ------------------------------------------------------------------
    def apply_delta_world(self, dx, dy, dz, droll, dpitch, dyaw):
        """
        Apply positional delta in world frame, rotational delta in EE frame.
        Clamps to human-like workspace limits to prevent IK flipping to
        unnatural configurations (e.g. arm reaching behind or across body).
        """
        ws = self.cfg["workspace"]

        # Position: world frame with clamping
        new_x = max(ws["x"][0], min(ws["x"][1], self._pose[0] + dx))
        new_y = max(ws["y"][0], min(ws["y"][1], self._pose[1] + dy))
        new_z = max(ws["z"][0], min(ws["z"][1], self._pose[2] + dz))
        self._pose[0], self._pose[1], self._pose[2] = new_x, new_y, new_z

        # Rotation: apply in EE frame (post-multiply = rotate in body frame)
        current_q = (self._pose[3], self._pose[4], self._pose[5], self._pose[6])
        delta_q = quat_from_rpy(droll, dpitch, dyaw)
        # new_q = current_q * delta_q  (rotate in body/EE frame)
        new_q = quat_normalize(quat_multiply(current_q, delta_q))
        self._pose[3], self._pose[4], self._pose[5], self._pose[6] = new_q

    def set_target_and_send(self, delta_xyz, delta_quat):
        """
        Set arm EE target = home_xyz + R_calib*delta_xyz, home_quat*delta_quat.
        Replaces self._pose directly (no accumulation → no drift).
        Returns (success, ik_ms).
        """
        hx, hy, hz, hqx, hqy, hqz, hqw = self.cfg["home_pose"]
        ws = self.cfg["workspace"]

        # Rotate delta position from tracker frame → arm world frame
        dx_arm = rotate_vec_by_quat(delta_xyz, self._calib_q)

        new_x = max(ws["x"][0], min(ws["x"][1], hx + dx_arm[0]))
        new_y = max(ws["y"][0], min(ws["y"][1], hy + dx_arm[1]))
        new_z = max(ws["z"][0], min(ws["z"][1], hz + dx_arm[2]))

        if getattr(self.args, 'no_rot_tracking', False):
            new_q = (hqx, hqy, hqz, hqw)
        else:
            home_q = (hqx, hqy, hqz, hqw)
            new_q  = quat_normalize(quat_multiply(home_q, delta_quat))

        saved_pose   = list(self._pose)
        self._pose   = [new_x, new_y, new_z, new_q[0], new_q[1], new_q[2], new_q[3]]

        t_total = time.time()
        ok, joints, ik_ms = self.call_ik_sync()
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

    def run_pattern_ee_delta(self):
        """
        Subscribe to /ee_delta/{tracker_side} (from tracker_ee_delta.py).
        Each PoseStamped contains total delta from tracker reference.
        Arm target = home + calib_rot * delta (no drift, no accumulation).
        """
        rate_hz = self.args.rate
        dt = 1.0 / rate_hz
        t_start = time.time()
        tracker_side = getattr(self.args, 'tracker_side', None) or self.args.arm

        print(f"\n{'='*65}")
        print(f"  EE Delta Mode  |  arm={self.args.arm}  tracker=/ee_delta/{tracker_side}")
        print(f"  calib_yaw={getattr(self.args,'calib_yaw',0.):.1f}deg  "
              f"rot_tracking={not getattr(self.args,'no_rot_tracking',False)}")
        print(f"  home_xyz = {self.cfg['home_pose'][:3]}")
        print(f"  workspace X{self.cfg['workspace']['x']}  "
              f"Y{self.cfg['workspace']['y']}  Z{self.cfg['workspace']['z']}")
        print(f"  Waiting for /ee_delta/{tracker_side} messages ...")
        print(f"  (run tracker_ee_delta.py and press SPACE there to start recording)")
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
                # Skip degenerate zero-quaternion
                if abs(dq[3]) < 0.01 and all(abs(v) < 0.01 for v in dq[:3]):
                    dq = (0., 0., 0., 1.)
                ok, ik_ms = self.set_target_and_send((dx, dy, dz), dq)
                elapsed = time.time() - t_start
                n = len(self._timing_records)
                sr = sum(r["success"] for r in self._timing_records) / n * 100 if n else 0.
                print(
                    f"\r  t={elapsed:6.1f}s  "
                    f"pos=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}]  "
                    f"delta=[{dx:+.3f},{dy:+.3f},{dz:+.3f}]  "
                    f"IK={ik_ms:5.1f}ms  {'OK' if ok else 'FAIL'}  sr={sr:.0f}%",
                    end="", flush=True,
                )

            sleep = dt - (time.time() - t_now)
            if sleep > 0:
                time.sleep(sleep)

    def step_and_send(self, dx, dy, dz, droll=0.0, dpitch=0.0, dyaw=0.0):
        """
        Apply delta, call IK, publish JointTrajectory.
        Returns (success, ik_ms).
        """
        # Save pose in case IK fails (rollback)
        saved_pose = list(self._pose)

        self.apply_delta_world(dx, dy, dz, droll, dpitch, dyaw)

        t_total = time.time()
        ok, joints, ik_ms = self.call_ik_sync()

        if ok:
            with self._joints_lock:
                self._last_joints = joints
            if not self.args.dry_run:
                self._publish_trajectory(joints)
            self._latency_pub.publish(Float32(data=float(ik_ms)))
        else:
            # Rollback pose on IK failure
            self._pose = saved_pose
            self.get_logger().warn(
                f"IK failed at [{self._pose[0]:.3f}, {self._pose[1]:.3f}, {self._pose[2]:.3f}] "
                f"({ik_ms:.1f} ms)"
            )

        total_ms = (time.time() - t_total) * 1000
        self._timing_records.append({
            "t": time.time(),
            "x": self._pose[0], "y": self._pose[1], "z": self._pose[2],
            "success": int(ok),
            "ik_ms": ik_ms,
            "total_ms": total_ms,
            "dx": dx, "dy": dy, "dz": dz,
        })
        return ok, ik_ms

    # ------------------------------------------------------------------
    # Auto-test patterns
    # ------------------------------------------------------------------
    def run_pattern_sine(self):
        """Sinusoidal X motion to test IK continuity without hardware."""
        freq = 0.3      # Hz
        amp = self.args.radius  # m
        rate_hz = self.args.rate
        duration = self.args.duration
        dt = 1.0 / rate_hz
        t_start = time.time()
        t_prev = t_start
        phase_prev = 0.0

        self.get_logger().info(
            f"[Sine] X amplitude={amp:.3f}m  freq={freq}Hz  "
            f"rate={rate_hz}Hz  duration={duration}s"
        )
        print(f"\n{'='*60}")
        print(f"  Sine pattern: X ± {amp*100:.1f} cm at {freq} Hz")
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
            success_rate = sum(r["success"] for r in self._timing_records) / n * 100
            print(
                f"\r  t={elapsed:5.1f}s  "
                f"pose=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}]  "
                f"IK={ik_ms:5.1f}ms  "
                f"{'OK' if ok else 'FAIL'}  "
                f"success={success_rate:.0f}%",
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
        omega = 2 * math.pi / 6.0   # complete 1 circle per 6 seconds
        t_start = time.time()
        angle_prev = 0.0

        self.get_logger().info(
            f"[Circle] Y-Z plane radius={radius:.3f}m  rate={rate_hz}Hz"
        )
        print(f"\n{'='*60}")
        print(f"  Circle: Y-Z plane radius={radius*100:.1f}cm")
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
            success_rate = sum(r["success"] for r in self._timing_records) / n * 100
            print(
                f"\r  t={elapsed:5.1f}s  "
                f"pose=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}]  "
                f"IK={ik_ms:5.1f}ms  "
                f"{'OK' if ok else 'FAIL'}  "
                f"success={success_rate:.0f}%",
                end="", flush=True,
            )

            sleep = dt - (time.time() - t_now)
            if sleep > 0:
                time.sleep(sleep)
        print()

    def run_pattern_lemniscate(self):
        """Lemniscate (figure-8) in X-Y plane (Bernoulli curve)."""
        scale = self.args.radius
        rate_hz = self.args.rate
        duration = self.args.duration
        dt = 1.0 / rate_hz
        omega = 2 * math.pi / 10.0  # 10-second loop
        t_start = time.time()

        self.get_logger().info(
            f"[Lemniscate] X-Y plane scale={scale:.3f}m  rate={rate_hz}Hz"
        )
        print(f"\n{'='*60}")
        print(f"  Lemniscate (figure-8): X-Y plane scale={scale*100:.1f}cm")
        print(f"  Press Ctrl-C to stop")
        print(f"{'='*60}\n")

        t_prev = t_start
        prev_angle = 0.0

        while time.time() - t_start < duration:
            t_now = time.time()
            elapsed = t_now - t_start
            angle = omega * elapsed

            # Bernoulli lemniscate parametric: (a·cos(t)/(1+sin²t), a·sin(t)·cos(t)/(1+sin²t))
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
            success_rate = sum(r["success"] for r in self._timing_records) / n * 100
            print(
                f"\r  t={elapsed:5.1f}s  "
                f"pose=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}]  "
                f"IK={ik_ms:5.1f}ms  "
                f"{'OK' if ok else 'FAIL'}  "
                f"success={success_rate:.0f}%",
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
        print(f"  Expecting TwistStamped with enable field via linear.w")
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
                droll = msg.twist.angular.x
                dpitch = msg.twist.angular.y
                dyaw = msg.twist.angular.z
                # linear.w > 0.5 means trigger pressed (enable motion)
                # Note: TwistStamped has no .w — use a workaround via header.seq
                # For Pico bridge: twist.linear set to zero when trigger not pressed
                if abs(dx) + abs(dy) + abs(dz) + abs(droll) + abs(dpitch) + abs(dyaw) > 1e-6:
                    ok, ik_ms = self.step_and_send(dx, dy, dz, droll, dpitch, dyaw)
                    elapsed = time.time() - t_start
                    print(
                        f"\r  t={elapsed:5.1f}s  "
                        f"pose=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}]  "
                        f"IK={ik_ms:5.1f}ms  {'OK' if ok else 'FAIL'}",
                        end="", flush=True,
                    )
            else:
                time.sleep(0.005)  # executor thread handles callbacks

            sleep = dt - (time.time() - t_now)
            if sleep > 0:
                time.sleep(sleep)

    def run_pattern_keyboard(self):
        """Interactive keyboard control simulating Pico joystick."""
        import select
        import termios
        import tty

        step = self.args.step
        rot_step = math.radians(self.args.rot_step)

        KEY_MAP = {
            'w': ( step,   0,     0,     0,       0,       0     ),
            's': (-step,   0,     0,     0,       0,       0     ),
            'a': ( 0,      step,  0,     0,       0,       0     ),
            'd': ( 0,     -step,  0,     0,       0,       0     ),
            'q': ( 0,      0,     step,  0,       0,       0     ),
            'e': ( 0,      0,    -step,  0,       0,       0     ),
            'u': ( 0,      0,     0,     rot_step, 0,      0     ),
            'j': ( 0,      0,     0,    -rot_step, 0,      0     ),
            'i': ( 0,      0,     0,     0,     rot_step,  0     ),
            'k': ( 0,      0,     0,     0,    -rot_step,  0     ),
            'o': ( 0,      0,     0,     0,       0,    rot_step ),
            'l': ( 0,      0,     0,     0,       0,   -rot_step ),
        }

        print(f"\n{'='*60}")
        print(f"  Keyboard mode — simulating Pico VR joystick")
        print(f"  Linear step: {step*1000:.1f} mm | Rotation step: {math.degrees(rot_step):.1f}°")
        print()
        print(f"  Position (world frame):  W/S=+X/-X  A/D=+Y/-Y  Q/E=+Z/-Z")
        print(f"  Rotation (EE frame):     U/J=Roll   I/K=Pitch  O/L=Yaw")
        print(f"  Step size:               + = larger   - = smaller")
        print(f"  R = reset to home  |  P = print pose  |  Ctrl-C = quit")
        ws = self.cfg["workspace"]
        print(f"\n  Workspace limits (human arm, no-flip zone):")
        print(f"    X [{ws['x'][0]:.2f}, {ws['x'][1]:.2f}]  Y [{ws['y'][0]:.2f}, {ws['y'][1]:.2f}]  Z [{ws['z'][0]:.2f}, {ws['z'][1]:.2f}]")
        print(f"{'='*60}")
        print(f"\n  Current EE pose: [{self._pose[0]:.3f}, {self._pose[1]:.3f}, {self._pose[2]:.3f}]")

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            while True:
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if r:
                    ch = sys.stdin.read(1)
                    if ch == '\x03':  # Ctrl-C
                        raise KeyboardInterrupt
                    elif ch in KEY_MAP:
                        dx, dy, dz, dr, dp, dyw = KEY_MAP[ch]
                        ok, ik_ms = self.step_and_send(dx, dy, dz, dr, dp, dyw)
                        r_deg, p_deg, y_deg = [math.degrees(v) for v in quat_to_rpy(
                            (self._pose[3], self._pose[4], self._pose[5], self._pose[6])
                        )]
                        print(
                            f"\r  [{ch}] pos=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}]"
                            f" rpy=[{r_deg:.1f}°,{p_deg:.1f}°,{y_deg:.1f}°]"
                            f" IK={ik_ms:5.1f}ms {'✓' if ok else '✗'}"
                            + " " * 10,
                            flush=True,
                        )
                    elif ch == '+':
                        step *= 1.5
                        rot_step *= 1.5
                        for k in KEY_MAP:
                            KEY_MAP[k] = tuple(v * 1.5 if abs(v) > 0 else v
                                               for v in KEY_MAP[k])
                        print(f"\n  [+] step={step*1000:.1f}mm")
                    elif ch == '-':
                        step /= 1.5
                        rot_step /= 1.5
                        for k in KEY_MAP:
                            KEY_MAP[k] = tuple(v / 1.5 if abs(v) > 0 else v
                                               for v in KEY_MAP[k])
                        print(f"\n  [-] step={step*1000:.1f}mm")
                    elif ch.lower() == 'r':
                        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                        print(f"\n  [R] Resetting to home...")
                        self.send_home()
                        time.sleep(3.5)
                        tty.setraw(fd)
                        print(f"  Home reached. Current: {self._pose[:3]}")
                    elif ch.lower() == 'p':
                        r_deg, p_deg, y_deg = [math.degrees(v) for v in quat_to_rpy(
                            (self._pose[3], self._pose[4], self._pose[5], self._pose[6])
                        )]
                        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                        print(f"\n  Pose: x={self._pose[0]:.4f} y={self._pose[1]:.4f} z={self._pose[2]:.4f}")
                        print(f"  Quat: qx={self._pose[3]:.4f} qy={self._pose[4]:.4f} "
                              f"qz={self._pose[5]:.4f} qw={self._pose[6]:.4f}")
                        print(f"  RPY:  roll={r_deg:.2f}° pitch={p_deg:.2f}° yaw={y_deg:.2f}°")
                        tty.setraw(fd)
                else:
                    time.sleep(0.005)  # executor thread handles callbacks

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

        ik_times = [r["ik_ms"] for r in self._timing_records if r["success"]]
        if not ik_times:
            return

        import statistics
        fig, axes = plt.subplots(1, 3, figsize=(15, 4))
        fig.suptitle(
            f"Pico Delta IK — {self.args.arm} arm  [{self.args.pattern}]   "
            f"n={len(ik_times)}  success={sum(r['success'] for r in self._timing_records)/len(self._timing_records)*100:.1f}%"
        )

        # Histogram
        axes[0].hist(ik_times, bins=30, color="steelblue", edgecolor="white")
        axes[0].axvline(statistics.median(ik_times), color="red", label=f"median={statistics.median(ik_times):.1f}ms")
        axes[0].set_xlabel("IK latency (ms)")
        axes[0].set_ylabel("Count")
        axes[0].set_title("IK Latency Distribution")
        axes[0].legend()

        # Timeline
        t0 = self._timing_records[0]["t"]
        ts = [r["t"] - t0 for r in self._timing_records]
        ik_all = [r["ik_ms"] for r in self._timing_records]
        colors = ["green" if r["success"] else "red" for r in self._timing_records]
        axes[1].scatter(ts, ik_all, c=colors, s=10, alpha=0.7)
        axes[1].set_xlabel("Time (s)")
        axes[1].set_ylabel("IK latency (ms)")
        axes[1].set_title("IK Latency Timeline")

        # 3D trajectory
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
        description="Pico VR delta IK controller for OpenArm",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--arm", default="right", choices=["left", "right"])
    p.add_argument("--pattern", default="ee_delta",
                   choices=["keyboard", "sine", "circle", "lemniscate", "topic", "ee_delta"])
    p.add_argument("--tracker-side", default=None,
                   help="tracker side to subscribe: left|right (default: same as --arm)")
    p.add_argument("--calib-yaw", type=float, default=0.0,
                   help="Rotation (yaw, degrees) to align tracker X → arm world X (default: 0)")
    p.add_argument("--calib-rpy", type=str, default=None,
                   help="Full calibration as 'roll,pitch,yaw' degrees, overrides --calib-yaw")
    p.add_argument("--no-rot-tracking", action="store_true",
                   help="Only track position delta, keep arm orientation fixed at home")
    p.add_argument("--rate", type=float, default=20.0,
                   help="Control loop Hz (default: 20)")
    p.add_argument("--horizon", type=float, default=60.0,
                   help="JointTrajectory horizon ms (default: 60)")
    p.add_argument("--step", type=float, default=0.005,
                   help="Linear step size m (keyboard mode, default: 0.005)")
    p.add_argument("--rot-step", type=float, default=1.0,
                   help="Rotation step degrees (keyboard mode, default: 1.0)")
    p.add_argument("--radius", type=float, default=0.04,
                   help="Amplitude/radius m for auto patterns (default: 0.04)")
    p.add_argument("--duration", type=float, default=30.0,
                   help="Auto-pattern duration seconds (default: 30)")
    p.add_argument("--no-collisions", action="store_true",
                   help="Disable IK collision checking")
    p.add_argument("--home-first", action="store_true",
                   help="Send arm to home before starting")
    p.add_argument("--home-on-exit", action="store_true",
                   help="Return home on Ctrl-C")
    p.add_argument("--dry-run", action="store_true",
                   help="Compute IK but do not publish commands")
    p.add_argument("--timeout-ik", type=float, default=0.1,
                   help="IK service timeout seconds (default: 0.1)")
    p.add_argument("--timing-csv", default="delta_ik_timing.csv",
                   help="Output timing CSV path")
    p.add_argument("--no-plot", action="store_true",
                   help="Skip PNG plot generation")
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = PicoDeltaIKController(args)

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(f"\n  Pico Delta IK Controller")
    print(f"  ARM: {args.arm} | PATTERN: {args.pattern} | RATE: {args.rate}Hz")
    print(f"  Horizon: {args.horizon}ms | Collisions: {not args.no_collisions}")
    print(f"  Dry-run: {args.dry_run}")

    try:
        # Wait for IK service
        if not node._wait_for_ik_service():
            print("\nERROR: /compute_ik not available. Is move_group running?")
            return

        print("  /compute_ik service ready ✓")

        # Optionally go home first
        if args.home_first:
            print("\n  Moving to home position...")
            node.send_home()
            time.sleep(3.5)

        # Run pattern
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
        # Print summary
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
                print(f"  Effective rate : {n / (records[-1]['t'] - records[0]['t']):.1f} Hz"
                      if len(records) > 1 else "")
            print(f"{'='*60}")

        # Go home on exit if requested
        if args.home_on_exit:
            print("  Sending home...")
            node.send_home()
            time.sleep(3.5)

        # Save results
        node.save_results()

        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
