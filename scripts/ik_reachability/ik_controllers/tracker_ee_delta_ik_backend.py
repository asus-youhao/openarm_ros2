#!/usr/bin/env python3
_IK_DIR = __import__('os').path.dirname(__import__('os').path.abspath(__file__))
import os as _os, sys as _sys
# Parent dir (ik_reachability/) for ROS/rclpy imports
_sys.path.insert(0, _os.path.join(_IK_DIR, ".."))
# ik_solver/ sub-folder takes priority for solver modules
_sys.path.insert(0, _os.path.join(_IK_DIR, "ik_solver"))
"""
tracker_ee_delta_ik_backend.py — 多 IK 後端控制器（無需 MoveIt）
=================================================================
★ 支援 4 種 IK 後端：scipy | pybullet | pinocchio | placo ★

與其他版本對照：
  tracker_ee_delta_ik_pure.py   — scipy only
  tracker_ee_delta_ik_backend.py [本檔案]
                                 — scipy | pybullet | pinocchio | placo

所有後端共用相同 FK/IK 界面：
    ok, joints, ms = solver.solve(target_xyz, target_quat, last_joints,
                                  verbose=False, no_rot=False)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Prerequisites（只需機器人 driver，不需要 move_group）：
    ros2 launch openarm_bringup openarm_o6_bimanual.launch.py

Dependencies (conda pico_teleop_py)：
    pip install pybullet           # for --solver pybullet
    pip install pin                # for --solver pinocchio  (Pinocchio 3.x)
    pip install placo              # for --solver placo
    # scipy 和 numpy 已預裝

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Commands:

  # scipy (default, 無需額外安裝)
  python3 tracker_ee_delta_ik_backend.py --arm right --pattern keyboard

  # PyBullet IK
  conda run -n pico_teleop_py python3 tracker_ee_delta_ik_backend.py \\
    --arm right --pattern keyboard --solver pybullet

  # Pinocchio IK  
  conda run -n pico_teleop_py python3 tracker_ee_delta_ik_backend.py \\
    --arm right --pattern ee_delta --solver pinocchio

  # Placo IK
  conda run -n pico_teleop_py python3 tracker_ee_delta_ik_backend.py \\
    --arm right --pattern keyboard --solver placo

  # VR tracker + 位置 only IK
  conda run -n pico_teleop_py python3 tracker_ee_delta_ik_backend.py \\
    --arm right --pattern ee_delta --solver pinocchio --no-rot-tracking

  # 雙臂（兩個 terminal）
  conda run -n pico_teleop_py python3 tracker_ee_delta_ik_backend.py \\
    --arm right --pattern ee_delta --solver pinocchio &
  conda run -n pico_teleop_py python3 tracker_ee_delta_ik_backend.py \\
    --arm left  --pattern ee_delta --solver pinocchio

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Options:
    --arm            left | right  (default: right)
    --solver         scipy | pybullet | pinocchio | placo  (default: scipy)
    --pattern        keyboard | sine | circle | lemniscate | topic | ee_delta
    --tracker-side   left | right  (default: same as --arm)
    --calib-yaw      Yaw offset degrees tracker → arm frame (default: 0)
    --calib-rpy      'roll,pitch,yaw' degrees, overrides --calib-yaw
    --no-rot-tracking  Position-only IK (faster, no orientation tracking)
    --rate           Control loop Hz (default: 20)
    --horizon        JointTrajectory duration ms (default: 60)
    --step           Linear step m (keyboard, default: 0.005)
    --rot-step       Rotation step degrees (keyboard, default: 1.0)
    --radius         Amplitude/radius m for auto patterns (default: 0.04)
    --duration       Auto-pattern duration seconds (default: 30)
    --home-first     Send arm home before starting
    --home-on-exit   Return home on Ctrl-C
    --dry-run        Compute IK but do not publish to controller
    --natural-verbose  Print each IK attempt detail (debug)
    --timing-csv     Output timing CSV path (default: delta_ik_timing.csv)
    --no-plot        Skip PNG plot generation
"""

import argparse
import csv as _csv_mod
import math
import os
import pathlib
import sys
import threading
import time

import numpy as np
import rclpy
import rclpy.executors
from copy import deepcopy
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# TF2 for syncing actual EE pose at startup
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


# ─────────────────────────────────────────────────────────────────────────────
# Solver factory
# ─────────────────────────────────────────────────────────────────────────────
def _load_solver(backend: str, arm: str):
    backend = backend.lower()
    if backend == "scipy":
        from pure_python_ik_solver import PurePythonIKSolver
        return PurePythonIKSolver(arm)
    elif backend == "pybullet":
        from pybullet_ik_solver import PybulletIKSolver
        return PybulletIKSolver(arm)
    elif backend in ("pinocchio", "pin"):
        from pinocchio_ik_solver import PinocchioIKSolver
        return PinocchioIKSolver(arm)
    elif backend == "placo":
        from placo_ik_solver import PlacoIKSolver
        return PlacoIKSolver(arm)
    else:
        raise ValueError(f"Unknown IK backend: {backend!r}. "
                         f"Choose from: scipy | pybullet | pinocchio | placo")


# ─────────────────────────────────────────────────────────────────────────────
# Arm configuration
# ─────────────────────────────────────────────────────────────────────────────
_ARM_CONFIG = {
    "left": {
        "joint_names":  [f"openarm_left_joint{i}" for i in range(1, 8)],
        "base_link":    "world",
        "ee_link":      "openarm_left_link7",
        # j2=-0.7 (MIRRORED from right j2=+0.7): LEFT URDF j2 range [-190°,+10°]
        "home_joints":  [0.0, -0.0, 0.0, 1.5708, 0.0, 0.0, 0.0],
        # FK(home_joints) via pinocchio: EE=(0.2160, +0.2952, 0.5297), quat=(0.6642,-0.2425,0.6642,-0.2425)
        "home_pose":    (0.2160, 0.2952, 0.5297, 0.6642, -0.2425, 0.6642, -0.2425),
        "workspace":    {"x": (-0.30, 0.30), "y": (0.05, 0.45), "z": (0.05, 0.65)},
        "cmd_topic":    "/left_joint_trajectory_controller/joint_trajectory",
        "latency_topic":"/left/delta_ik_latency_ms",
        "delta_topic":  "/pico_left/delta_twist",
    },
    "right": {
        "joint_names":  [f"openarm_right_joint{i}" for i in range(1, 8)],
        "base_link":    "world",
        "ee_link":      "openarm_right_link7",
        "home_joints":  [0.0, 0.0, 0.0, 1.5708, 0.0, 0.0, 0.0],  # j4=90° forward-reach
        "home_pose":    (0.2160, -0.2952, 0.5297, 0.6642, 0.2425, 0.6642, 0.2425),  # FK(home_joints) via pinocchio
        "workspace":    {"x": (-0.30, 0.30), "y": (-0.45, -0.05), "z": (0.05, 0.65)},
        "cmd_topic":    "/right_joint_trajectory_controller/joint_trajectory",
        "latency_topic":"/right/delta_ik_latency_ms",
        "delta_topic":  "/pico_right/delta_twist",
    },
}


# ─────────────────────────────────────────────────────────────────────────────
# Quaternion utilities
# ─────────────────────────────────────────────────────────────────────────────
def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


def quat_from_rpy(roll, pitch, yaw):
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
    x, y, z, w = q
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


def rotate_vec_by_quat(v, q):
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


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 Node
# ─────────────────────────────────────────────────────────────────────────────
class BackendDeltaIKController(Node):
    """
    Incremental EE delta controller supporting 4 IK backends (no MoveIt).
    Backend is selected at startup via --solver argument.
    """

    def __init__(self, args):
        super().__init__("pico_backend_ik_controller")
        self.args = args
        cfg = _ARM_CONFIG[args.arm]
        self.cfg = cfg

        self._cbg = ReentrantCallbackGroup()

        # EE pose state: [x, y, z, qx, qy, qz, qw]
        hx, hy, hz, hqx, hqy, hqz, hqw = cfg["home_pose"]
        self._pose = [hx, hy, hz, hqx, hqy, hqz, hqw]

        # Last successful joint solution (seed for next IK call)
        self._last_joints = list(cfg["home_joints"])
        self._joints_lock = threading.Lock()

        # Joint states from /joint_states
        self._joint_states: dict = {}
        self._js_lock = threading.Lock()

        # ★ IK solver — selected by --solver ★
        self._solver_name = args.solver
        self._ik_solver = _load_solver(args.solver, args.arm)
        self.get_logger().info(
            f"[IK backend] {args.solver} loaded for {args.arm} arm"
        )

        # TF2 for syncing actual EE at startup/session reset
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Publishers
        self._traj_pub    = self.create_publisher(
            JointTrajectory, cfg["cmd_topic"], 10)
        self._latency_pub = self.create_publisher(
            Float32, cfg["latency_topic"], 10)

        # Subscribers
        self.create_subscription(
            JointState, "/joint_states",
            self._joint_states_cb, 10, callback_group=self._cbg)

        self._pending_delta = None
        self._delta_lock    = threading.Lock()
        self._msg_count     = 0   # counts _ee_delta_cb invocations

        if args.pattern == "topic":
            self.create_subscription(
                TwistStamped, cfg["delta_topic"],
                self._delta_cb, 10, callback_group=self._cbg)

        if args.pattern == "ee_delta":
            tracker_side = getattr(args, "tracker_side", None) or args.arm
            ee_topic = f"/ee_delta/{tracker_side}"
            self.create_subscription(
                PoseStamped, ee_topic,
                self._ee_delta_cb, 10, callback_group=self._cbg)
            crpy = getattr(args, "calib_rpy", None)
            if crpy:
                rr, rp, ry = [math.radians(float(v)) for v in crpy.split(",")]
            else:
                rr, rp = 0.0, 0.0
                ry = math.radians(getattr(args, "calib_yaw", 0.0))
            self._calib_q = quat_from_rpy(rr, rp, ry)

        self._ee_delta_ref_xyz  = None
        self._ee_delta_ref_q    = None
        self._ee_delta_last_t   = None
        self._ee_delta_gap_sec  = 0.35
        self._timing_records    = []

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
        self._msg_count += 1
        # Print first 5 and every 30th so we can verify messages arrive
        if self._msg_count <= 5 or self._msg_count % 30 == 0:
            p = msg.pose.position
            o = msg.pose.orientation
            print(
                f"\n[CB#{self._msg_count}] ee_delta"
                f" pos=({p.x:.4f},{p.y:.4f},{p.z:.4f})"
                f" quat=({o.x:.4f},{o.y:.4f},{o.z:.4f},{o.w:.4f})"
            )

    # ------------------------------------------------------------------
    # IK call — routes to active backend
    # ------------------------------------------------------------------
    def _call_ik(self):
        with self._joints_lock:
            last_joints = list(self._last_joints)
        return self._ik_solver.solve(
            target_xyz  = tuple(self._pose[:3]),
            target_quat = tuple(self._pose[3:7]),
            last_joints = last_joints,
            verbose     = getattr(self.args, "natural_verbose", False),
            no_rot      = getattr(self.args, "no_rot_tracking", False),
        )

    # ------------------------------------------------------------------
    # Trajectory publisher
    # ------------------------------------------------------------------
    def _publish_trajectory(self, joints: list):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = list(self.cfg["joint_names"])
        pt = JointTrajectoryPoint()
        pt.positions     = joints
        pt.velocities    = [0.0] * len(joints)
        pt.accelerations = [0.0] * len(joints)
        hs = self.args.horizon / 1000.0
        pt.time_from_start.sec    = int(hs)
        pt.time_from_start.nanosec = int((hs % 1) * 1e9)
        msg.points = [pt]
        self._traj_pub.publish(msg)

    def send_home(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = list(self.cfg["joint_names"])
        pt = JointTrajectoryPoint()
        pt.positions     = list(self.cfg["home_joints"])
        pt.velocities    = [0.0] * len(pt.positions)
        pt.time_from_start.sec = 3
        pt.time_from_start.nanosec = 0
        msg.points = [pt]
        self._traj_pub.publish(msg)
        hx, hy, hz, hqx, hqy, hqz, hqw = self.cfg["home_pose"]
        self._pose = [hx, hy, hz, hqx, hqy, hqz, hqw]
        with self._joints_lock:
            self._last_joints = list(self.cfg["home_joints"])

    def send_home_confirmed(
        self,
        pos_tol:    float = 0.025,   # m: TF vs home_pose must be within this
        motion_sec: float = 3.5,     # wait time after each cmd (≥ trajectory duration)
        max_tries:  int   = 5,       # max re-send attempts
        poll_sec:   float = 0.3,     # TF polling interval during wait
    ) -> bool:
        """
        Send home trajectory, then verify arrival via TF.
        If arm has not reached home (TF pos error > pos_tol),
        re-publish the command and wait again, up to max_tries times.

        Returns True if arm reached home, False if max_tries exhausted.
        """
        home_xyz = self.cfg["home_pose"][:3]          # (x, y, z)
        jnames   = self.cfg["joint_names"]

        for attempt in range(1, max_tries + 1):
            # ── Send the trajectory ──
            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names  = list(jnames)
            pt = JointTrajectoryPoint()
            pt.positions     = list(self.cfg["home_joints"])
            pt.velocities    = [0.0] * len(pt.positions)
            pt.time_from_start.sec = int(motion_sec * 0.85)   # traj finishes before we check
            pt.time_from_start.nanosec = 0
            msg.points = [pt]
            self._traj_pub.publish(msg)
            print(f"  [home] attempt {attempt}/{max_tries} — cmd sent, waiting {motion_sec:.1f}s...")

            # ── Wait for motion + poll TF ──
            t_deadline = time.time() + motion_sec
            arrived    = False
            while time.time() < t_deadline:
                time.sleep(poll_sec)
                tf_pose = self.get_current_ee_from_tf(timeout_sec=0.3)
                if tf_pose is None:
                    continue
                dist = math.sqrt(
                    (tf_pose[0] - home_xyz[0]) ** 2 +
                    (tf_pose[1] - home_xyz[1]) ** 2 +
                    (tf_pose[2] - home_xyz[2]) ** 2
                )
                print(f"  [home]   TF xyz=({tf_pose[0]:.3f},{tf_pose[1]:.3f},{tf_pose[2]:.3f})"
                      f"  dist_to_home={dist*100:.1f}cm", end="\r", flush=True)
                if dist <= pos_tol:
                    arrived = True
                    break

            print()  # newline after \r

            if arrived:
                print(f"  [home] ✓ reached home in attempt {attempt}  dist={dist*100:.1f}cm")
                # Sync internal pose + joints from TF / joint_states
                tf_pose = self.get_current_ee_from_tf(timeout_sec=1.0)
                if tf_pose is not None:
                    self._pose = list(tf_pose)
                with self._js_lock:
                    js_snap = dict(self._joint_states)
                if all(n in js_snap for n in jnames):
                    with self._joints_lock:
                        self._last_joints = [js_snap[n] for n in jnames]
                return True
            else:
                # Check joint_states to see how far off we are
                with self._js_lock:
                    js_snap = dict(self._joint_states)
                home_j = self.cfg["home_joints"]
                j_err  = max(
                    abs(js_snap.get(n, 0) - home_j[i])
                    for i, n in enumerate(jnames)
                ) if js_snap else 99.0
                print(f"  [home] ✗ attempt {attempt} timed out  j_max_err={math.degrees(j_err):.1f}° — retrying...")

        print(f"  [home] ✗ could not confirm home after {max_tries} attempts — continuing anyway")
        hx, hy, hz, hqx, hqy, hqz, hqw = self.cfg["home_pose"]
        self._pose = [hx, hy, hz, hqx, hqy, hqz, hqw]
        with self._joints_lock:
            self._last_joints = list(self.cfg["home_joints"])
        return False

    def get_current_ee_from_tf(self, timeout_sec=0.5):
        try:
            t = self._tf_buffer.lookup_transform(
                self.cfg["base_link"], self.cfg["ee_link"],
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec),
            )
            tr = t.transform.translation
            ro = t.transform.rotation
            return (tr.x, tr.y, tr.z, ro.x, ro.y, ro.z, ro.w)
        except TransformException as ex:
            self.get_logger().error(f"TF failed: {ex}")
            return None

    # ------------------------------------------------------------------
    # Delta application
    # ------------------------------------------------------------------
    def apply_delta_world(self, dx, dy, dz, droll, dpitch, dyaw):
        ws = self.cfg["workspace"]
        self._pose[0] = max(ws["x"][0], min(ws["x"][1], self._pose[0] + dx))
        self._pose[1] = max(ws["y"][0], min(ws["y"][1], self._pose[1] + dy))
        self._pose[2] = max(ws["z"][0], min(ws["z"][1], self._pose[2] + dz))
        q = (self._pose[3], self._pose[4], self._pose[5], self._pose[6])
        new_q = quat_normalize(quat_multiply(q, quat_from_rpy(droll, dpitch, dyaw)))
        self._pose[3:7] = list(new_q)

    def set_target_and_send(self, delta_xyz, delta_quat):
        ws = self.cfg["workspace"]
        if self._ee_delta_ref_xyz is not None:
            base_xyz = self._ee_delta_ref_xyz
            base_q   = self._ee_delta_ref_q
        else:
            base_xyz = tuple(self._pose[:3])
            base_q   = tuple(self._pose[3:7])

        dx_arm = rotate_vec_by_quat(delta_xyz, self._calib_q)
        new_x = max(ws["x"][0], min(ws["x"][1], base_xyz[0] + dx_arm[0]))
        new_y = max(ws["y"][0], min(ws["y"][1], base_xyz[1] + dx_arm[1]))
        new_z = max(ws["z"][0], min(ws["z"][1], base_xyz[2] + dx_arm[2]))

        if getattr(self.args, "no_rot_tracking", False):
            new_q = base_q
        else:
            new_q = quat_normalize(quat_multiply(delta_quat, base_q))

        saved = list(self._pose)
        self._pose = [new_x, new_y, new_z, new_q[0], new_q[1], new_q[2], new_q[3]]

        t_total = time.time()
        ok, joints, ik_ms = self._call_ik()
        if ok:
            with self._joints_lock:
                self._last_joints = joints
            if not self.args.dry_run:
                self._publish_trajectory(joints)
            self._latency_pub.publish(Float32(data=float(ik_ms)))
        else:
            # ── DEBUG: show exactly what IK was given and why it may fail ──
            n_total = len(self._timing_records) + 1
            if n_total <= 5 or n_total % 20 == 0:
                rr, pp, yy = quat_to_rpy(new_q)
                with self._joints_lock:
                    seed = list(self._last_joints)
                print(
                    f"\n  [IK-FAIL #{n_total}]"
                    f" target_xyz=({new_x:.4f},{new_y:.4f},{new_z:.4f})"
                    f" rpy=({math.degrees(rr):.1f},{math.degrees(pp):.1f},{math.degrees(yy):.1f})°"
                    f"  ik_ms={ik_ms:.1f}"
                    f"\n  base_xyz=({base_xyz[0]:.4f},{base_xyz[1]:.4f},{base_xyz[2]:.4f})"
                    f" delta=({delta_xyz[0]:.4f},{delta_xyz[1]:.4f},{delta_xyz[2]:.4f})"
                    f"\n  dq=({delta_quat[0]:.4f},{delta_quat[1]:.4f},{delta_quat[2]:.4f},{delta_quat[3]:.4f})"
                    f" base_q=({base_q[0]:.4f},{base_q[1]:.4f},{base_q[2]:.4f},{base_q[3]:.4f})"
                    f"\n  seed_joints={[f'{v:.3f}' for v in seed]}"
                )
            self._pose = saved

        self._timing_records.append({
            "t": time.time(),
            "x": self._pose[0], "y": self._pose[1], "z": self._pose[2],
            "success": int(ok), "ik_ms": ik_ms,
            "total_ms": (time.time() - t_total) * 1000,
            "dx": delta_xyz[0], "dy": delta_xyz[1], "dz": delta_xyz[2],
        })
        return ok, ik_ms

    def step_and_send(self, dx, dy, dz, droll=0.0, dpitch=0.0, dyaw=0.0):
        saved = list(self._pose)
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
            self._pose = saved
        self._timing_records.append({
            "t": time.time(),
            "x": self._pose[0], "y": self._pose[1], "z": self._pose[2],
            "success": int(ok), "ik_ms": ik_ms,
            "total_ms": (time.time() - t_total) * 1000,
            "dx": dx, "dy": dy, "dz": dz,
        })
        return ok, ik_ms

    # ------------------------------------------------------------------
    # Patterns
    # ------------------------------------------------------------------
    def run_pattern_keyboard(self):
        import select, termios, tty

        print("  Syncing from TF2 and /joint_states...")
        time.sleep(0.5)
        tf_pose = self.get_current_ee_from_tf(timeout_sec=2.0)
        if tf_pose is not None:
            self._pose = list(tf_pose)
            print(f"  TF EE: {[f'{v:.4f}' for v in tf_pose[:3]]}")
        else:
            print("  ⚠ TF unavailable — using hardcoded home_pose")
        with self._js_lock:
            js_snap = dict(self._joint_states)
        jnames = self.cfg["joint_names"]
        if all(n in js_snap for n in jnames):
            actual = [js_snap[n] for n in jnames]
            with self._joints_lock:
                self._last_joints = actual
            print(f"  /joint_states: {[f'{v:.3f}' for v in actual]}")
        else:
            print("  ⚠ /joint_states not ready — using home_joints")
        print()

        step     = self.args.step
        rot_step = math.radians(self.args.rot_step)
        KEY_MAP  = {
            "w": ( step,   0,     0,     0,         0,        0),
            "s": (-step,   0,     0,     0,         0,        0),
            "a": ( 0,      step,  0,     0,         0,        0),
            "d": ( 0,     -step,  0,     0,         0,        0),
            "q": ( 0,      0,     step,  0,         0,        0),
            "e": ( 0,      0,    -step,  0,         0,        0),
            "u": ( 0,      0,     0,     rot_step,  0,        0),
            "j": ( 0,      0,     0,    -rot_step,  0,        0),
            "i": ( 0,      0,     0,     0,     rot_step,     0),
            "k": ( 0,      0,     0,     0,    -rot_step,     0),
            "o": ( 0,      0,     0,     0,         0,  rot_step),
            "l": ( 0,      0,     0,     0,         0, -rot_step),
        }

        print(f"\n{'='*65}")
        print(f"  Keyboard — arm={self.args.arm}  solver={self._solver_name}  (no MoveIt)")
        print(f"  W/S=±X  A/D=±Y  Q/E=±Z  U/J=Roll  I/K=Pitch  O/L=Yaw")
        print(f"  + / -  = scale step  |  R = home  |  P = pose  |  Ctrl-C = quit")
        ws = self.cfg["workspace"]
        print(f"  Workspace: X{ws['x']}  Y{ws['y']}  Z{ws['z']}")
        print(f"{'='*65}")
        print(f"  Current EE: {self._pose[:3]}")

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if r:
                    ch = sys.stdin.read(1)
                    if ch == "\x03":
                        raise KeyboardInterrupt
                    elif ch in KEY_MAP:
                        dx, dy, dz, dr, dp, dyw = KEY_MAP[ch]
                        ok, ik_ms = self.step_and_send(dx, dy, dz, dr, dp, dyw)
                        rr, pp, yy = [math.degrees(v) for v in quat_to_rpy(
                            (self._pose[3], self._pose[4], self._pose[5], self._pose[6]))]
                        with self._joints_lock:
                            j3d = math.degrees(self._last_joints[2]) if self._last_joints else 0.
                        print(
                            f"\r  [{ch}] "
                            f"pos=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}]"
                            f" rpy=[{rr:.1f}°,{pp:.1f}°,{yy:.1f}°]"
                            f" j3={j3d:+.1f}°"
                            f" IK={ik_ms:5.1f}ms {'✓' if ok else '✗'}" + " "*5,
                            flush=True,
                        )
                    elif ch == "+":
                        step *= 1.5
                        for k in KEY_MAP:
                            KEY_MAP[k] = tuple(v * 1.5 if abs(v) > 0 else v for v in KEY_MAP[k])
                        print(f"\n  [+] step={step*1000:.1f}mm")
                    elif ch == "-":
                        step /= 1.5
                        for k in KEY_MAP:
                            KEY_MAP[k] = tuple(v / 1.5 if abs(v) > 0 else v for v in KEY_MAP[k])
                        print(f"\n  [-] step={step*1000:.1f}mm")
                    elif ch.lower() == "r":
                        termios.tcsetattr(fd, termios.TCSADRAIN, old)
                        print("\n  [R] Sending home...")
                        self.send_home()
                        time.sleep(3.5)
                        tty.setraw(fd)
                        print("  Home reached.")
                    elif ch.lower() == "p":
                        rr, pp, yy = [math.degrees(v) for v in quat_to_rpy(
                            (self._pose[3], self._pose[4], self._pose[5], self._pose[6]))]
                        with self._joints_lock:
                            jc = list(self._last_joints)
                        termios.tcsetattr(fd, termios.TCSADRAIN, old)
                        print(f"\n  Pose: {[f'{v:.4f}' for v in self._pose[:3]]}")
                        print(f"  RPY: roll={rr:.2f}° pitch={pp:.2f}° yaw={yy:.2f}°")
                        print(f"  Joints: {[f'{v:.3f}' for v in jc]}")
                        tty.setraw(fd)
                else:
                    time.sleep(0.005)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def run_pattern_ee_delta(self):
        dt = 1.0 / self.args.rate
        t_start = time.time()
        tracker_side = getattr(self.args, "tracker_side", None) or self.args.arm
        print(f"\n{'='*65}")
        print(f"  EE Delta  arm={self.args.arm}  solver={self._solver_name}")
        print(f"  tracker=/ee_delta/{tracker_side}  rot={not getattr(self.args,'no_rot_tracking',False)}")
        print(f"{'='*65}\n")

        # ── Sync self._pose and _last_joints from TF / joint_states before loop ──
        # (same as run_pattern_keyboard — prevents wrong hardcoded home_pose from
        #  causing IK failures when delta arrives)
        print("  Syncing from TF2 and /joint_states...")
        time.sleep(0.5)  # let TF buffer warm up
        tf_pose = self.get_current_ee_from_tf(timeout_sec=2.0)
        if tf_pose is not None:
            self._pose = list(tf_pose)
            print(f"  TF EE: {[f'{v:.4f}' for v in tf_pose[:3]]}")
        else:
            print("  ⚠ TF unavailable — ee_delta will try TF each NEW session")
        with self._js_lock:
            js_snap = dict(self._joint_states)
        jnames = self.cfg["joint_names"]
        if all(n in js_snap for n in jnames):
            actual = [js_snap[n] for n in jnames]
            with self._joints_lock:
                self._last_joints = actual
            j3d = math.degrees(actual[2])
            print(f"  /joint_states: {[f'{v:.3f}' for v in actual]}  j3={j3d:+.1f}°")
        else:
            print("  ⚠ /joint_states not ready — using home_joints as seed")
        print(f"  Waiting for /ee_delta/{tracker_side} messages...\n")

        _no_msg_t = time.time()   # track idle time for periodic "waiting" print
        _loop_n   = 0
        while True:
            t_now = time.time()
            _loop_n += 1
            with self._delta_lock:
                msg = self._pending_delta
                self._pending_delta = None

            if msg is None:
                # Print waiting status every 2 s so we know the loop is alive
                idle_s = time.time() - _no_msg_t
                if idle_s >= 2.0:
                    _no_msg_t = time.time()
                    print(
                        f"\r  [WAIT] no ee_delta msg  cb_count={self._msg_count}"
                        f"  loop={_loop_n}  idle={idle_s:.0f}s"
                        f"  self._pose=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}]",
                        end="", flush=True,
                    )
            elif not isinstance(msg, PoseStamped):
                print(f"\n  [WARN] unexpected msg type: {type(msg)}")
            else:
                _no_msg_t = time.time()  # reset idle timer
                dx = msg.pose.position.x
                dy = msg.pose.position.y
                dz = msg.pose.position.z
                dq = (msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w)
                dq_zero = abs(dq[3]) < 0.01 and all(abs(v) < 0.01 for v in dq[:3])
                if dq_zero:
                    dq = (0., 0., 0., 1.)

                t_wall = time.time()
                gap = (t_wall - self._ee_delta_last_t) if self._ee_delta_last_t else 999.
                is_new = self._ee_delta_ref_xyz is None or gap > self._ee_delta_gap_sec
                self._ee_delta_last_t = t_wall

                if is_new:
                    ref = self.get_current_ee_from_tf(0.3)
                    if ref is not None:
                        self._ee_delta_ref_xyz = tuple(ref[:3])
                        self._ee_delta_ref_q   = tuple(ref[3:7])
                        # Also update self._pose so it stays consistent with TF
                        self._pose = list(ref)
                        print(f"\n  [EE-δ] NEW (gap={gap:.2f}s) ref={[f'{v:.4f}' for v in ref[:3]]} q={[f'{v:.4f}' for v in ref[3:]]} (TF)")
                    else:
                        # TF unavailable — use current self._pose (already synced at startup)
                        self._ee_delta_ref_xyz = tuple(self._pose[:3])
                        self._ee_delta_ref_q   = tuple(self._pose[3:7])
                        print(f"\n  [EE-δ] NEW (gap={gap:.2f}s) ref={[f'{v:.4f}' for v in self._pose[:3]]} q={[f'{v:.4f}' for v in self._pose[3:]]} (last cmd)")

                # Debug: show raw delta on first few messages
                n_total = len(self._timing_records)
                if n_total < 5:
                    print(
                        f"\n  [LOOP msg#{n_total+1}] dx={dx:.5f} dy={dy:.5f} dz={dz:.5f}"
                        f" dq=({dq[0]:.4f},{dq[1]:.4f},{dq[2]:.4f},{dq[3]:.4f})"
                        f" dq_was_zero={dq_zero}  is_new={is_new}"
                    )

                ok, ik_ms = self.set_target_and_send((dx, dy, dz), dq)
                elapsed = time.time() - t_start
                n  = len(self._timing_records)
                sr = sum(r["success"] for r in self._timing_records) / n * 100 if n else 0.
                with self._joints_lock:
                    j3d = math.degrees(self._last_joints[2]) if self._last_joints else 0.
                print(
                    f"\r  t={elapsed:6.1f}s "
                    f"pos=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}] "
                    f"j3={j3d:+.0f}° IK={ik_ms:5.1f}ms {'OK' if ok else 'FAIL'} sr={sr:.0f}%  cb={self._msg_count}",
                    end="", flush=True,
                )

            sleep = dt - (time.time() - t_now)
            if sleep > 0:
                time.sleep(sleep)

    def run_pattern_sine(self):
        freq = 0.3
        amp  = self.args.radius
        dt   = 1.0 / self.args.rate
        t0   = time.time()
        prev = 0.0
        print(f"\n  Sine: X±{amp*100:.1f}cm  solver={self._solver_name}  Ctrl-C to stop\n")
        while time.time() - t0 < self.args.duration:
            tn = time.time()
            ph = 2 * math.pi * freq * (tn - t0)
            ok, ms = self.step_and_send(amp * (math.sin(ph) - math.sin(prev)), 0, 0)
            prev = ph
            n  = len(self._timing_records)
            sr = sum(r["success"] for r in self._timing_records) / n * 100
            with self._joints_lock:
                j3d = math.degrees(self._last_joints[2]) if self._last_joints else 0.
            print(
                f"\r  t={tn-t0:5.1f}s "
                f"pos=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}] "
                f"j3={j3d:+.0f}° IK={ms:5.1f}ms {'OK' if ok else 'FAIL'} sr={sr:.0f}%",
                end="", flush=True,
            )
            sleep = dt - (time.time() - tn)
            if sleep > 0:
                time.sleep(sleep)
        print()

    def run_pattern_circle(self):
        r    = self.args.radius
        dt   = 1.0 / self.args.rate
        om   = 2 * math.pi / 6.0
        t0   = time.time()
        prev = 0.0
        print(f"\n  Circle: Y-Z r={r*100:.1f}cm  solver={self._solver_name}  Ctrl-C to stop\n")
        while time.time() - t0 < self.args.duration:
            tn = time.time()
            a  = om * (tn - t0)
            ok, ms = self.step_and_send(0, r*(math.cos(a)-math.cos(prev)), r*(math.sin(a)-math.sin(prev)))
            prev = a
            n  = len(self._timing_records)
            sr = sum(r["success"] for r in self._timing_records) / n * 100
            print(f"\r  t={tn-t0:5.1f}s pos=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}] "
                  f"IK={ms:5.1f}ms {'OK' if ok else 'FAIL'} sr={sr:.0f}%", end="", flush=True)
            sleep = dt - (time.time() - tn)
            if sleep > 0:
                time.sleep(sleep)
        print()

    def run_pattern_lemniscate(self):
        sc   = self.args.radius
        dt   = 1.0 / self.args.rate
        om   = 2 * math.pi / 10.0
        t0   = time.time()
        prev = 0.0

        def _lem(theta):
            d = 1 + math.sin(theta)**2
            return sc * math.cos(theta) / d, sc * math.sin(theta) * math.cos(theta) / d

        print(f"\n  Lemniscate: scale={sc*100:.1f}cm  solver={self._solver_name}  Ctrl-C to stop\n")
        while time.time() - t0 < self.args.duration:
            tn = time.time()
            a  = om * (tn - t0)
            x, y = _lem(a)
            xp, yp = _lem(prev)
            ok, ms = self.step_and_send(x-xp, y-yp, 0)
            prev = a
            n  = len(self._timing_records)
            sr = sum(r["success"] for r in self._timing_records) / n * 100
            print(f"\r  t={tn-t0:5.1f}s pos=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}] "
                  f"IK={ms:5.1f}ms {'OK' if ok else 'FAIL'} sr={sr:.0f}%", end="", flush=True)
            sleep = dt - (time.time() - tn)
            if sleep > 0:
                time.sleep(sleep)
        print()

    def run_pattern_topic(self):
        dt = 1.0 / self.args.rate
        t0 = time.time()
        print(f"\n  Topic: {self.cfg['delta_topic']}  solver={self._solver_name}  Ctrl-C to stop\n")
        while True:
            tn = time.time()
            with self._delta_lock:
                msg = self._pending_delta
                self._pending_delta = None
            if msg is not None:
                dx, dy, dz = msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z
                dr, dp, dyw = msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z
                if abs(dx)+abs(dy)+abs(dz)+abs(dr)+abs(dp)+abs(dyw) > 1e-6:
                    ok, ms = self.step_and_send(dx, dy, dz, dr, dp, dyw)
                    print(f"\r  t={tn-t0:5.1f}s pos=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}] "
                          f"IK={ms:5.1f}ms {'OK' if ok else 'FAIL'}", end="", flush=True)
            else:
                time.sleep(0.005)
            sleep = dt - (time.time() - tn)
            if sleep > 0:
                time.sleep(sleep)

    # ------------------------------------------------------------------
    def _make_output_paths(self):
        """Auto-generate dated output paths.
        Structure:  <script_dir>/results/yyyymmdd/
          files:    delta_ik_timing_mmddHHMM_<arm>_<solver>.{csv,png}
        """
        import datetime
        now  = datetime.datetime.now()
        date_folder = now.strftime("%Y%m%d")
        stem = now.strftime("%m%d%H%M") + f"_{self.args.arm}_{self._solver_name}"
        out_dir = pathlib.Path(_IK_DIR) / "results" / date_folder
        out_dir.mkdir(parents=True, exist_ok=True)
        csv_path = out_dir / f"delta_ik_timing_{stem}.csv"
        png_path = out_dir / f"delta_ik_timing_{stem}.png"
        return csv_path, png_path

    def save_results(self):
        if not self._timing_records:
            return
        csv_path, png_path = self._make_output_paths()
        with open(csv_path, "w", newline="") as f:
            w = _csv_mod.DictWriter(f, fieldnames=self._timing_records[0].keys())
            w.writeheader()
            w.writerows(self._timing_records)
        self.get_logger().info(f"Saved {len(self._timing_records)} records → {csv_path}")
        if not self.args.no_plot:
            try:
                self._plot(csv_path, png_path)
            except Exception as e:
                self.get_logger().warn(f"Plot failed: {e}")

    def _plot(self, csv_path, png_path):
        import matplotlib; matplotlib.use("Agg")
        import matplotlib.pyplot as plt, statistics
        ik_times = [r["ik_ms"] for r in self._timing_records if r["success"]]
        if not ik_times:
            return
        fig, axes = plt.subplots(1, 2, figsize=(12, 4))
        fig.suptitle(f"Backend IK ({self._solver_name}) — {self.args.arm} [{self.args.pattern}]  "
                     f"n={len(ik_times)}  "
                     f"sr={sum(r['success'] for r in self._timing_records)/len(self._timing_records)*100:.1f}%")
        axes[0].hist(ik_times, bins=30, color="steelblue", edgecolor="white")
        axes[0].axvline(statistics.median(ik_times), color="red",
                        label=f"median={statistics.median(ik_times):.1f}ms")
        axes[0].set_xlabel("IK ms"); axes[0].set_ylabel("Count"); axes[0].legend()
        t0 = self._timing_records[0]["t"]
        ts = [r["t"]-t0 for r in self._timing_records]
        axes[1].scatter(ts, [r["ik_ms"] for r in self._timing_records],
                        c=["g" if r["success"] else "r" for r in self._timing_records], s=8)
        axes[1].set_xlabel("Time (s)"); axes[1].set_ylabel("IK ms")
        fig.tight_layout()
        fig.savefig(str(png_path), dpi=120)
        plt.close()
        self.get_logger().info(f"Plot → {png_path}")


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description="Backend IK controller — scipy | pybullet | pinocchio | placo",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--arm",     default="right", choices=["left", "right"])
    p.add_argument("--solver",  default="scipy",
                   choices=["scipy", "pybullet", "pinocchio", "placo"],
                   help="IK backend (default: scipy)")
    p.add_argument("--pattern", default="ee_delta",
                   choices=["keyboard","sine","circle","lemniscate","topic","ee_delta"])
    p.add_argument("--tracker-side", default=None)
    p.add_argument("--calib-yaw",    type=float, default=0.0)
    p.add_argument("--calib-rpy",    type=str,   default=None)
    p.add_argument("--no-rot-tracking",  action="store_true")
    p.add_argument("--rate",     type=float, default=20.0)
    p.add_argument("--horizon",  type=float, default=60.0)
    p.add_argument("--step",     type=float, default=0.005)
    p.add_argument("--rot-step", type=float, default=1.0)
    p.add_argument("--radius",   type=float, default=0.04)
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--home-first",   action="store_true")
    p.add_argument("--home-on-exit", action="store_true")
    p.add_argument("--dry-run",      action="store_true")
    p.add_argument("--natural-verbose", action="store_true")
    p.add_argument("--no-plot",   action="store_true")
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = BackendDeltaIKController(args)

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(f"\n{'='*65}")
    print(f"  Backend IK Controller  —  solver={args.solver}  arm={args.arm}")
    print(f"  pattern={args.pattern}  rate={args.rate}Hz  horizon={args.horizon}ms")
    print(f"  no MoveIt required ✓")
    print(f"{'='*65}\n")

    try:
        if args.home_first:
            print("  Moving to home (with TF confirmation)...")
            node.send_home_confirmed(pos_tol=0.025, motion_sec=3.5, max_tries=5)

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
        else:  # ee_delta
            node.run_pattern_ee_delta()

    except KeyboardInterrupt:
        print("\n\n  Interrupted.")
    finally:
        records = node._timing_records
        if records:
            n = len(records)
            ok_n = sum(r["success"] for r in records)
            ik_t = [r["ik_ms"] for r in records if r["success"]]
            import statistics as stats
            print(f"\n{'='*65}")
            print(f"  Summary  solver={args.solver}  arm={args.arm}  pattern={args.pattern}")
            print(f"  Commands: {n}  Success: {ok_n}/{n} ({ok_n/n*100:.1f}%)")
            if ik_t:
                print(f"  IK:  median={stats.median(ik_t):.1f}ms  "
                      f"p99={sorted(ik_t)[int(len(ik_t)*0.99)]:.1f}ms  "
                      f"max={max(ik_t):.1f}ms")
            # Solver stats
            solver_stats = node._ik_solver.get_stats()
            print(f"  Solver: {solver_stats}")
            print(f"{'='*65}")

        if args.home_on_exit:
            node.send_home()
            time.sleep(3.5)

        node.save_results()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
