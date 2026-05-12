#!/usr/bin/env python3
"""
placo_ik_absolute_profiler_ws_mesh.py
=====================================
placo_ik_absolute_profiler.py 的 WorkspaceMesh 版本。
將矩形 workspace box clamp 替換為由 placo_ws_analyze.WorkspaceMesh (.npz)
驅動的真實形狀 clamp。

  --ws-mesh <path.npz>   載入掃描產生的 WorkspaceMesh（取代矩形 box）

原有絕對座標 profiling 完全保留（auto-calibration offset, PlacoSession）。

接收 tracker_ee_absolute.py 發布的 /ee_target/{arm} (PoseStamped)
  → pose.position.{x,y,z}   = 絕對 XYZ，已換算到 ROS 世界座標系 (m)
  → pose.orientation.{x,y,z,w} = 絕對 quaternion，已換算到 ROS 座標系
  → frame_id = "world"（Pico 虛擬世界座標，ROS convention）

相對版 (placo_ik_online_profiler.py) 的差異：
  delta 版   → /ee_delta/{arm}: dx/dy/dz 相對 session-ref 的增量，需累積
  absolute 版 → /ee_target/{arm}: 直接給完整的 target pose，不需累積

【座標系問題：Tracker world ≠ Robot world】
  Pico 的 world origin = 頭盔初始化時的位置
  Robot 的 world origin = robot base
  兩者相差一個常數偏移 offset = robot_EE_TF - tracker_xyz_at_startup

  解法（auto-calibration，啟動時執行一次）：
    1. 從 TF2 取得當前 EE 在 robot world 的位置 (robot_xyz)
    2. 等待第一筆 /ee_target 訊息 (tracker_xyz)
    3. offset = robot_xyz - tracker_xyz
    4. 之後每步 target_xyz = tracker_xyz + offset（限制在 workspace 內）

  可選：--no-auto-calib 跳過自動校準（直接使用 tracker 原始座標）
       --offset x y z    手動指定偏移量（覆蓋自動校準）

【PlacoSession：同 placo_ik_online_profiler.py 的優化版】
  - cached robot（只建一次 RobotWrapper）
  - early exit（pos_err < 3mm 即停止迭代）
  - 1-5 次迭代 ≈ 0.4 ms vs 原版 250 次固定 = 20 ms

Usage（需要同時執行）：
  # Terminal 1: tracker publisher
  conda run -n pico_teleop_py python3 tracker_ee_absolute.py --tracker right

  # Terminal 2: absolute IK profiler
  conda run -n pico_teleop_py python3 placo_ik_absolute_profiler.py --arm right

  # 右臂，手動偏移，dry-run
  conda run -n pico_teleop_py python3 placo_ik_absolute_profiler.py \\
      --arm right --offset 0.0 0.0 0.0 --dry-run

  # 先回 home 再校準
  conda run -n pico_teleop_py python3 placo_ik_absolute_profiler.py \\
      --arm right --home-first

  # 跳過自動校準（直接用 tracker 原始座標）
  conda run -n pico_teleop_py python3 placo_ik_absolute_profiler.py \\
      --arm right --no-auto-calib

Published topics：
  /right_joint_trajectory_controller/joint_trajectory   (JointTrajectory)
  /right/delta_ik_latency_ms                            (Float32) — ik_ms
  /right/placo_profile                                  (String, JSON) — breakdown

CSV columns（與 online_profiler 相同 + absolute-specific 欄位）：
  t, x, y, z, success, ik_ms, total_ms,
  tx, ty, tz,                   ← raw tracker xyz（before offset）
  robot_ms, setup_ms, loop_ms,
  iterations, iter_ms,
  pos_err_mm, track_err_mm,
  mem_kb, deadline_missed
"""

# ── Path setup ────────────────────────────────────────────────────────────────
import os as _os, sys as _sys
_DIR    = _os.path.dirname(_os.path.abspath(__file__))
_PARENT = _os.path.dirname(_DIR)
_sys.path.insert(0, _PARENT)
_sys.path.insert(0, _os.path.join(_DIR, "ik_solver"))

import argparse
import csv
import datetime
import json
import math
import threading
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener, TransformException

from paths import csv_path as _csv_path, png_for as _png_for, ws_mesh_path as _ws_mesh_path
from placo_ik_solver import (
    _find_urdf,
    _HUMAN_RIGHT,
    _HUMAN_LEFT,
    _JOINT_NAMES,
    _quat_to_rot,
)
from placo_ws_analyze import WorkspaceMesh

# ── ARM config ────────────────────────────────────────────────────────────────
_ARM_CONFIG = {
    "left": {
        "joint_names":   [f"openarm_left_joint{i}"  for i in range(1, 8)],
        "base_link":     "world",
        "ee_link":       "openarm_left_link7",
        "home_joints":   [0.0, -0.7, 0.0, 1.5708, 0.0, 0.0, 0.0],
        "home_pose":     (0.2160, 0.2952, 0.5297, 0.6642, -0.2425, 0.6642, -0.2425),
        "workspace":     {"x": (-0.30, 0.30), "y": (0.05, 0.45), "z": (0.05, 0.65)},
        "cmd_topic":     "/left_joint_trajectory_controller/joint_trajectory",
        "latency_topic": "/left/delta_ik_latency_ms",
        "profile_topic": "/left/placo_profile",
        "abs_topic":     "/ee_target/left",
    },
    "right": {
        "joint_names":   [f"openarm_right_joint{i}" for i in range(1, 8)],
        "base_link":     "world",
        "ee_link":       "openarm_right_link7",
        "home_joints":   [0.0, 0.7, 0.0, 1.5708, 0.0, 0.0, 0.0],
        "home_pose":     (0.2160, -0.2952, 0.5297, 0.6642, 0.2425, 0.6642, 0.2425),
        "workspace":     {"x": (-0.30, 0.30), "y": (-0.45, -0.05), "z": (0.05, 0.65)},
        "cmd_topic":     "/right_joint_trajectory_controller/joint_trajectory",
        "latency_topic": "/right/delta_ik_latency_ms",
        "profile_topic": "/right/placo_profile",
        "abs_topic":     "/ee_target/right",
    },
}

# ── IK constants ──────────────────────────────────────────────────────────────
_POS_TOL   = 0.003   # m — early exit
_POS_RELAX = 0.010   # m — success threshold
_W_POS     = 1.0
_W_ORI     = 0.3
_W_JOINTS  = 1e-4
_MAX_ITER  = 250
_DT        = 0.010

# ── Helpers ───────────────────────────────────────────────────────────────────
def _mem_rss_kb() -> int:
    try:
        with open("/proc/self/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1])
    except OSError:
        pass
    return 0


def _qnorm(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    return (x/n, y/n, z/n, w/n) if n > 1e-9 else (0., 0., 0., 1.)


def _qfrom_rpy(r, p, y):
    cr, cp, cy = math.cos(r/2), math.cos(p/2), math.cos(y/2)
    sr, sp, sy = math.sin(r/2), math.sin(p/2), math.sin(y/2)
    return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)


def _qmul(q1, q2):
    """Hamilton product of two unit quaternions (x,y,z,w)."""
    x1, y1, z1, w1 = q1;  x2, y2, z2, w2 = q2
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2)


def _qconj(q):
    """Conjugate (= inverse) of a unit quaternion (x,y,z,w)."""
    return (-q[0], -q[1], -q[2], q[3])


def _qrpy_deg(q):
    """(x,y,z,w) → (roll, pitch, yaw) degrees."""
    qx, qy, qz, qw = q
    roll  = math.degrees(math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2)))
    pitch = math.degrees(math.asin(max(-1., min(1., 2*(qw*qy - qz*qx)))))
    yaw   = math.degrees(math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2)))
    return roll, pitch, yaw


# ── PlacoSession（identical to placo_ik_online_profiler.py）─────────────────
class PlacoSession:
    """
    Cached RobotWrapper + early-exit solve.
    rebuild=True reproduces the original ~25ms/step for comparison.
    """

    def __init__(self, urdf: str, arm: str, rebuild: bool = False,
                 max_iter: int = _MAX_ITER, dt: float = _DT):
        import placo
        self._placo      = placo
        self._urdf       = urdf
        self._arm        = arm
        self._rebuild    = rebuild
        self._max_iter   = max_iter
        self._dt         = dt
        self._joint_names = _JOINT_NAMES[arm]
        self._human_cfg   = _HUMAN_RIGHT if arm == "right" else _HUMAN_LEFT
        self._ee_link     = f"openarm_{arm}_link7"
        self._lo  = [self._human_cfg[n][1] for n in self._joint_names]
        self._hi  = [self._human_cfg[n][2] for n in self._joint_names]
        self._pref = {n: self._human_cfg[n][0] for n in self._joint_names}

        if not rebuild:
            self._robot = placo.RobotWrapper(urdf, placo.Flags.ignore_collisions)
            print(f"[PlacoSession] cached robot built  arm={arm}")
        else:
            self._robot = None
            print(f"[PlacoSession] rebuild mode  arm={arm}")

    def solve_step(self, target_xyz: np.ndarray, target_R: np.ndarray,
                   seed: List[float], no_rot: bool = False) -> Dict:
        placo = self._placo
        t0 = time.perf_counter()

        t_r0 = time.perf_counter()
        if self._rebuild or self._robot is None:
            robot = placo.RobotWrapper(self._urdf, placo.Flags.ignore_collisions)
        else:
            robot = self._robot
        robot_ms = (time.perf_counter() - t_r0) * 1000.0

        t_s0 = time.perf_counter()
        solver = placo.KinematicsSolver(robot)
        solver.enable_joint_limits(True)
        solver.enable_velocity_limits(False)
        solver.mask_fbase(True)
        solver.dt = self._dt
        for name, val in zip(self._joint_names, seed):
            robot.set_joint(name, val)
        robot.update_kinematics()
        pos_task = solver.add_position_task(self._ee_link, target_xyz)
        pos_task.configure("pos", "soft", _W_POS)
        if not no_rot:
            ori_task = solver.add_orientation_task(self._ee_link, target_R)
            ori_task.configure("ori", "soft", _W_ORI)
        jt = solver.add_joints_task()
        jt.set_joints(self._pref)
        jt.configure("naturalness", "soft", _W_JOINTS)
        reg = solver.add_regularization_task(1e-5)
        reg.configure("reg", "soft", 1.0)
        setup_ms = (time.perf_counter() - t_s0) * 1000.0

        t_l0 = time.perf_counter()
        iters_used = 0
        if self._rebuild:
            for _ in range(self._max_iter):
                solver.solve(True)
                robot.update_kinematics()
                iters_used += 1
        else:
            for _ in range(self._max_iter):
                solver.solve(True)
                robot.update_kinematics()
                iters_used += 1
                T_ee = robot.get_T_world_frame(self._ee_link)
                if float(np.linalg.norm(T_ee[:3, 3] - target_xyz)) < _POS_TOL:
                    break
        loop_ms = (time.perf_counter() - t_l0) * 1000.0

        joints  = [robot.get_joint(n) for n in self._joint_names]
        joints  = [max(l, min(h, q)) for q, l, h in zip(joints, self._lo, self._hi)]
        T_ee    = robot.get_T_world_frame(self._ee_link)
        pos_err = float(np.linalg.norm(T_ee[:3, 3] - target_xyz))
        ee_xyz  = list(T_ee[:3, 3])
        solve_ms = robot_ms + setup_ms + loop_ms

        return {
            "robot_ms":   robot_ms,
            "setup_ms":   setup_ms,
            "loop_ms":    loop_ms,
            "solve_ms":   solve_ms,
            "wall_ms":    (time.perf_counter() - t0) * 1000.0,
            "iterations": iters_used,
            "iter_ms":    loop_ms / iters_used if iters_used > 0 else 0.0,
            "mem_kb":     _mem_rss_kb(),
            "pos_err_mm": pos_err * 1000.0,
            "success":    int(pos_err < _POS_RELAX),
            "joints":     joints,
            "ee_xyz":     ee_xyz,
        }


# ── CSV fields ────────────────────────────────────────────────────────────────
_CSV_FIELDS = [
    "t", "x", "y", "z",
    "success", "ik_ms", "total_ms",
    "tx", "ty", "tz",          # raw tracker coords (before offset)
    "robot_ms", "setup_ms", "loop_ms",
    "iterations", "iter_ms",
    "pos_err_mm", "track_err_mm",
    "mem_kb", "deadline_missed",
]


# ── ROS2 Node ─────────────────────────────────────────────────────────────────
class PlacoAbsoluteProfiler(Node):
    """
    Subscribes to /ee_target/{arm} (PoseStamped, absolute, ROS world frame)
    from tracker_ee_absolute.py, computes IK, and publishes JointTrajectory.

    Auto-calibration at startup:
      offset = TF_EE_xyz - first_tracker_xyz
    This maps the Pico virtual world origin to the robot world origin.
    """

    def __init__(self, args):
        super().__init__("placo_ik_absolute_profiler_ws_mesh")
        self.args = args
        cfg = _ARM_CONFIG[args.arm]
        self.cfg = cfg

        self._cbg = ReentrantCallbackGroup()

        # EE pose state
        hx, hy, hz, hqx, hqy, hqz, hqw = cfg["home_pose"]
        self._pose         = [hx, hy, hz, hqx, hqy, hqz, hqw]
        self._last_joints  = list(cfg["home_joints"])
        self._joints_lock  = threading.Lock()
        self._joint_states: Dict = {}
        self._js_lock      = threading.Lock()

        # Auto-calibration state
        # offset maps tracker world → robot world: robot_xyz = tracker_xyz + offset
        manual_offset = getattr(args, "offset", None)
        if manual_offset is not None:
            ox, oy, oz = [float(v) for v in manual_offset.split(",")]
            self._offset = np.array([ox, oy, oz])
            self._calib_done = True
            print(f"  [calib] manual offset = [{ox:.4f}, {oy:.4f}, {oz:.4f}]")
        elif getattr(args, "no_auto_calib", False):
            self._offset = np.zeros(3)
            self._calib_done = True
            print("  [calib] SKIPPED (--no-auto-calib) — using raw tracker coords")
        else:
            self._offset = None
            self._calib_done = False
            print("  [calib] Auto-calibration pending (waiting for TF + /ee_target)...")

        self._first_tracker_xyz  = None
        self._calib_lock         = threading.Lock()
        # Orientation calibration fields
        self._first_tracker_quat = (0., 0., 0., 1.)
        self._q_offset           = (0., 0., 0., 1.)  # arm_home_R ⊗ conj(tracker_startup_R)
        self._rot_calib_done     = (getattr(args, "no_rot", False)
                                    or getattr(args, "no_auto_calib", False))

        # WorkspaceMesh (replaces rectangular box clamp when provided)
        self._ws_mesh: Optional[WorkspaceMesh] = None
        npz_path = getattr(args, "ws_mesh", None)
        if npz_path:
            if not os.path.isfile(npz_path):
                raise FileNotFoundError(f"--ws-mesh file not found: {npz_path}")
            self._ws_mesh = WorkspaceMesh.load(npz_path)

        # Rate / deadline
        self._rate_hz     = float(getattr(args, "rate", 20.0))
        self._horizon_ms  = float(getattr(args, "horizon", 60.0))
        self._deadline_ms = 1000.0 / self._rate_hz

        # URDF + Placo
        urdf = _find_urdf()
        self._placo_session = PlacoSession(
            urdf     = urdf,
            arm      = args.arm,
            rebuild  = args.rebuild,
            max_iter = getattr(args, "max_iter", _MAX_ITER),
            dt       = getattr(args, "dt",       _DT),
        )

        # TF2
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Publishers
        self._traj_pub    = self.create_publisher(JointTrajectory, cfg["cmd_topic"],    10)
        self._latency_pub = self.create_publisher(Float32,         cfg["latency_topic"],10)
        self._profile_pub = self.create_publisher(String,          cfg["profile_topic"],10)

        # Subscribers
        self.create_subscription(
            JointState, "/joint_states",
            self._js_cb, 10, callback_group=self._cbg)
        self.create_subscription(
            PoseStamped, cfg["abs_topic"],
            self._abs_cb, 10, callback_group=self._cbg)

        # Message queue
        self._pending    = None
        self._delta_lock = threading.Lock()
        self._msg_count  = 0

        # Records + CSV
        self._records: List[Dict] = []
        mode = "rebuild" if args.rebuild else "cached"
        csv_p = args.csv or _csv_path("placo_abs", args.arm, mode)
        self._csv_fh     = open(csv_p, "w", newline="")
        self._csv_writer = csv.writer(self._csv_fh)
        self._csv_writer.writerow(_CSV_FIELDS)
        self._csv_path   = csv_p

        print(f"\n{'═'*65}")
        print(f"  Placo IK Absolute Profiler  [WorkspaceMesh]")
        print(f"  arm={args.arm}  mode={'rebuild' if args.rebuild else 'CACHED+early_exit'}")
        if self._ws_mesh is not None:
            summ = self._ws_mesh.summary()
            clamp_str = (f"WorkspaceMesh  {summ['n_reachable_voxels']} voxels  "
                         f"step={summ['step_m']*100:.0f}cm  "
                         f"vol~{summ['total_volume_cm3']:.0f}cm³")
        else:
            clamp_str = "rectangular box (no --ws-mesh)"
        print(f"  rate={self._rate_hz:.0f}Hz  deadline={self._deadline_ms:.1f}ms")
        print(f"  abs topic:      {cfg['abs_topic']}")
        print(f"  cmd topic:      {cfg['cmd_topic']}")
        print(f"  profile topic:  {cfg['profile_topic']}")
        print(f"  CSV: {csv_path}")
        print(f"  workspace clamp: {clamp_str}")
        no_rot_mode = getattr(args, "no_rot", False)
        print(f"  orientation    : {'FIXED home (--no-rot, W_ORI=0)  <- Step 1' if no_rot_mode else 'TRACKED - q_offset auto-calib at startup  <- Step 2'}")
        print(f"{'═'*65}")
        print(f"\n  {'step':>5}  {'ik_ms':>7}  {'robot':>6}  {'loop':>6}  {'iters':>5}  "
              f"{'pos_mm':>6}  {'tr_mm':>6}  {'mem_MB':>6}  {'ddl':>4}")
        print(f"  {'─'*5}  {'─'*7}  {'─'*6}  {'─'*6}  {'─'*5}  "
              f"{'─'*6}  {'─'*6}  {'─'*6}  {'─'*4}")

    # ── Callbacks ──────────────────────────────────────────────────────────────
    def _js_cb(self, msg: JointState):
        with self._js_lock:
            for n, p in zip(msg.name, msg.position):
                self._joint_states[n] = p

    def _abs_cb(self, msg: PoseStamped):
        with self._delta_lock:
            self._pending = msg
        self._msg_count += 1

    # ── TF helper ──────────────────────────────────────────────────────────────
    def _get_tf(self, timeout_sec: float = 0.3) -> Optional[Tuple]:
        try:
            t = self._tf_buffer.lookup_transform(
                self.cfg["base_link"], self.cfg["ee_link"],
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec),
            )
            tr = t.transform.translation
            ro = t.transform.rotation
            return (tr.x, tr.y, tr.z, ro.x, ro.y, ro.z, ro.w)
        except TransformException:
            return None

    # ── Auto-calibration ───────────────────────────────────────────────────────
    def _try_calibrate(self, tracker_xyz: np.ndarray, tracker_quat: tuple) -> bool:
        """
        Try to complete auto-calibration (position offset + orientation q_offset).
        Called from the main loop on first valid tracker message.
        Returns True when calibration is done.
        """
        with self._calib_lock:
            if self._calib_done:
                return True
            if self._first_tracker_xyz is None:
                self._first_tracker_xyz  = tracker_xyz.copy()
                self._first_tracker_quat = tracker_quat

        # Get current EE from TF
        tf_pose = self._get_tf(timeout_sec=2.0)
        if tf_pose is None:
            print("  [calib] ⚠ TF unavailable, using home_pose as reference")
            robot_xyz = np.array(self.cfg["home_pose"][:3])
            arm_q     = tuple(self.cfg["home_pose"][3:7])
        else:
            robot_xyz = np.array(tf_pose[:3])
            arm_q     = tuple(tf_pose[3:7])
            self._pose = list(tf_pose)

        with self._calib_lock:
            self._offset = robot_xyz - self._first_tracker_xyz
            # Orientation: q_offset = arm_q ⊗ conj(tracker_startup_q)
            # → each step: q_target = q_offset ⊗ tracker_current_q
            self._q_offset       = _qnorm(_qmul(arm_q, _qconj(self._first_tracker_quat)))
            self._rot_calib_done = True
            self._calib_done     = True

        arm_rpy  = _qrpy_deg(arm_q)
        trk_rpy  = _qrpy_deg(self._first_tracker_quat)
        off_rpy  = _qrpy_deg(self._q_offset)
        print(f"\n  [calib] ✓ position offset = [{self._offset[0]:+.4f}, "
              f"{self._offset[1]:+.4f}, {self._offset[2]:+.4f}] m")
        print(f"  [calib] ✓ q_offset RPY    = "
              f"[{off_rpy[0]:+.1f},{off_rpy[1]:+.1f},{off_rpy[2]:+.1f}]°")
        print(f"          robot_EE_xyz = {[f'{v:.4f}' for v in robot_xyz]}")
        print(f"          tracker_xyz  = {[f'{v:.4f}' for v in self._first_tracker_xyz]}")
        print(f"          arm_q  RPY   = [{arm_rpy[0]:+.1f},{arm_rpy[1]:+.1f},{arm_rpy[2]:+.1f}]°")
        print(f"          tracker RPY  = [{trk_rpy[0]:+.1f},{trk_rpy[1]:+.1f},{trk_rpy[2]:+.1f}]°")
        return True

    def _try_rot_calibrate(self, tracker_quat: tuple) -> bool:
        """
        Orientation-only calibration — used when position was calibrated via --offset
        so _try_calibrate was skipped.  Non-blocking: returns False if TF not ready.
        """
        tf_pose = self._get_tf(timeout_sec=0.3)
        if tf_pose is None:
            return False
        arm_q = tuple(tf_pose[3:7])
        q_off = _qnorm(_qmul(arm_q, _qconj(tracker_quat)))
        with self._calib_lock:
            self._q_offset       = q_off
            self._rot_calib_done = True
        arm_rpy  = _qrpy_deg(arm_q)
        trk_rpy  = _qrpy_deg(tracker_quat)
        off_rpy  = _qrpy_deg(q_off)
        print(f"\n  [rot_calib] ✓ q_offset RPY = "
              f"[{off_rpy[0]:+.1f},{off_rpy[1]:+.1f},{off_rpy[2]:+.1f}]°")
        print(f"              arm_q  RPY   = [{arm_rpy[0]:+.1f},{arm_rpy[1]:+.1f},{arm_rpy[2]:+.1f}]°")
        print(f"              tracker RPY  = [{trk_rpy[0]:+.1f},{trk_rpy[1]:+.1f},{trk_rpy[2]:+.1f}]°")
        return True

    # ── Home-first（mirrors tracker backend send_home_confirmed）──────────────
    def send_home_confirmed(
        self,
        pos_tol:    float = 0.025,
        motion_sec: float = 3.5,
        max_tries:  int   = 5,
        poll_sec:   float = 0.3,
    ) -> bool:
        home_xyz = self.cfg["home_pose"][:3]
        jnames   = self.cfg["joint_names"]

        for attempt in range(1, max_tries + 1):
            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names  = list(jnames)
            pt = JointTrajectoryPoint()
            pt.positions     = list(self.cfg["home_joints"])
            pt.velocities    = [0.0] * len(pt.positions)
            pt.time_from_start.sec = int(motion_sec * 0.85)
            pt.time_from_start.nanosec = 0
            msg.points = [pt]
            self._traj_pub.publish(msg)
            print(f"  [home] attempt {attempt}/{max_tries} — cmd sent, "
                  f"waiting {motion_sec:.1f}s...")

            t_deadline = time.time() + motion_sec
            arrived = False
            dist    = 999.0
            while time.time() < t_deadline:
                time.sleep(poll_sec)
                tf_pose = self._get_tf(timeout_sec=0.3)
                if tf_pose is None:
                    continue
                dist = math.sqrt(
                    (tf_pose[0] - home_xyz[0])**2 +
                    (tf_pose[1] - home_xyz[1])**2 +
                    (tf_pose[2] - home_xyz[2])**2)
                print(f"  [home]  TF xyz=({tf_pose[0]:.3f},{tf_pose[1]:.3f},"
                      f"{tf_pose[2]:.3f})  dist={dist*100:.1f}cm",
                      end="\r", flush=True)
                if dist <= pos_tol:
                    arrived = True
                    break
            print()

            if arrived:
                print(f"  [home] ✓ reached home  dist={dist*100:.1f}cm")
                tf_pose = self._get_tf(timeout_sec=1.0)
                if tf_pose is not None:
                    self._pose = list(tf_pose)
                with self._js_lock:
                    js_snap = dict(self._joint_states)
                if all(n in js_snap for n in jnames):
                    with self._joints_lock:
                        self._last_joints = [js_snap[n] for n in jnames]
                return True
            else:
                with self._js_lock:
                    js_snap = dict(self._joint_states)
                home_j = self.cfg["home_joints"]
                j_err  = max(
                    abs(js_snap.get(n, 0) - home_j[i])
                    for i, n in enumerate(jnames)
                ) if js_snap else 99.0
                print(f"  [home] ✗ attempt {attempt} timed out  "
                      f"j_max_err={math.degrees(j_err):.1f}° — retrying...")

        print(f"  [home] ✗ could not confirm home after {max_tries} attempts")
        hx, hy, hz, hqx, hqy, hqz, hqw = self.cfg["home_pose"]
        self._pose = [hx, hy, hz, hqx, hqy, hqz, hqw]
        with self._joints_lock:
            self._last_joints = list(self.cfg["home_joints"])
        return False

    # ── Trajectory publish ─────────────────────────────────────────────────────
    def _publish(self, joints: List[float]):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = list(self.cfg["joint_names"])
        pt = JointTrajectoryPoint()
        pt.positions     = list(joints)
        pt.time_from_start.nanosec = int(self._horizon_ms * 1e6)
        msg.points = [pt]
        self._traj_pub.publish(msg)

    # ── Main loop ──────────────────────────────────────────────────────────────
    def run(self):
        """
        Main absolute-target IK loop.

        Key difference from delta version:
        - No session reference / gap detection
        - Target = tracker_xyz + calib_offset (clamped to workspace)
        - Orientation = tracker quaternion directly (already in ROS world frame)
        """
        dt_sec = 1.0 / self._rate_hz

        # Initial sync
        print("\n  Syncing /joint_states and TF2...")
        time.sleep(0.5)
        tf_pose = self._get_tf(2.0)
        if tf_pose is not None:
            self._pose = list(tf_pose)
            print(f"  TF EE: [{tf_pose[0]:.4f}, {tf_pose[1]:.4f}, {tf_pose[2]:.4f}]")
        else:
            print("  ⚠ TF unavailable — using home_pose")
        with self._js_lock:
            js_snap = dict(self._joint_states)
        jnames = self.cfg["joint_names"]
        if all(n in js_snap for n in jnames):
            with self._joints_lock:
                self._last_joints = [js_snap[n] for n in jnames]
            print(f"  joints synced: {[f'{v:.3f}' for v in self._last_joints]}")
        else:
            print("  ⚠ /joint_states not ready — using home_joints")

        print(f"\n  Waiting for {self.cfg['abs_topic']} ...\n")

        _no_msg_t  = time.time()
        step_count = 0
        ws         = self.cfg["workspace"]

        while rclpy.ok():
            t_step = time.perf_counter()

            with self._delta_lock:
                msg = self._pending
                self._pending = None

            if msg is None:
                idle_s = time.time() - _no_msg_t
                if idle_s >= 2.0:
                    _no_msg_t = time.time()
                    print(
                        f"\r  [WAIT] no /ee_target  cb={self._msg_count}"
                        f"  steps={step_count}  idle={idle_s:.0f}s"
                        f"  calib={'OK' if self._calib_done else 'PENDING'}",
                        end="", flush=True,
                    )
            elif isinstance(msg, PoseStamped):
                _no_msg_t = time.time()
                t_wall = time.time()

                # ── Raw tracker absolute xyz + orientation (already ROS frame) ─
                tx = msg.pose.position.x
                ty = msg.pose.position.y
                tz = msg.pose.position.z
                tracker_xyz = np.array([tx, ty, tz])

                tq = (msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w)
                # guard against zero quaternion
                if abs(tq[3]) < 0.01 and all(abs(v) < 0.01 for v in tq[:3]):
                    tq = (0., 0., 0., 1.)
                tq = _qnorm(tq)

                # ── Auto-calibration (first message only) ─────────────────────
                if not self._calib_done:
                    if not self._try_calibrate(tracker_xyz, tq):
                        # Still waiting for TF; skip this step
                        sleep_s = dt_sec - (time.perf_counter() - t_step)
                        if sleep_s > 0:
                            time.sleep(sleep_s)
                        continue

                # Orientation calibration (for --offset / manual position case)
                if not self._rot_calib_done:
                    self._try_rot_calibrate(tq)

                # ── Apply calibration offset → robot world frame ──────────────
                with self._calib_lock:
                    offset = self._offset.copy()

                robot_target_xyz = tracker_xyz + offset

                # ── Workspace clamping ────────────────────────────────────────
                if self._ws_mesh is not None:
                    # Real-shape clamp via WorkspaceMesh KDTree
                    clamped, _inside = self._ws_mesh.clamp(
                        robot_target_xyz)
                    new_x, new_y, new_z = float(clamped[0]), float(clamped[1]), float(clamped[2])
                else:
                    new_x = max(ws["x"][0], min(ws["x"][1], robot_target_xyz[0]))
                    new_y = max(ws["y"][0], min(ws["y"][1], robot_target_xyz[1]))
                    new_z = max(ws["z"][0], min(ws["z"][1], robot_target_xyz[2]))
                target_xyz = np.array([new_x, new_y, new_z])

                # Orientation: apply q_offset calibration
                # Step 1: --no-rot -> W_ORI=0, arm keeps home orientation
                # Step 2: q_target = q_offset * tracker_current_q
                no_rot = getattr(self.args, "no_rot", False)
                if no_rot:
                    tq_target = tuple(self.cfg["home_pose"][3:7])
                else:
                    with self._calib_lock:
                        q_off = self._q_offset
                    tq_target = _qnorm(_qmul(q_off, tq))
                target_R = _quat_to_rot(*tq_target)

                with self._joints_lock:
                    seed = list(self._last_joints)

                # IK solve (profiled)
                t_total = time.perf_counter()
                r = self._placo_session.solve_step(target_xyz, target_R, seed,
                                                   no_rot=no_rot)
                ik_ms    = r["solve_ms"]
                total_ms = (time.perf_counter() - t_total) * 1000.0

                actual_xyz = np.array(r["ee_xyz"])
                track_err  = float(np.linalg.norm(actual_xyz - target_xyz)) * 1000.0

                loop_wall_ms    = (time.perf_counter() - t_step) * 1000.0
                deadline_missed = int(loop_wall_ms > self._deadline_ms)

                if r["success"]:
                    self._pose = [new_x, new_y, new_z,
                                  tq_target[0], tq_target[1], tq_target[2], tq_target[3]]
                    with self._joints_lock:
                        self._last_joints = r["joints"]
                    if not getattr(self.args, "dry_run", False):
                        self._publish(r["joints"])
                    self._latency_pub.publish(Float32(data=float(ik_ms)))

                step_count += 1

                # ── Profile publish (JSON) ────────────────────────────────────
                self._profile_pub.publish(String(data=json.dumps({
                    "step":           step_count,
                    "ik_ms":          round(ik_ms, 3),
                    "robot_ms":       round(r["robot_ms"], 3),
                    "setup_ms":       round(r["setup_ms"], 3),
                    "loop_ms":        round(r["loop_ms"],  3),
                    "iterations":     r["iterations"],
                    "iter_ms":        round(r["iter_ms"], 4),
                    "pos_err_mm":     round(r["pos_err_mm"], 3),
                    "track_err_mm":   round(track_err, 3),
                    "success":        r["success"],
                    "deadline_missed":deadline_missed,
                    "mem_mb":         round(r["mem_kb"] / 1024.0, 1),
                    "mode":           "rebuild" if self.args.rebuild else "cached",
                    "tracker_xyz":    [round(tx,4), round(ty,4), round(tz,4)],
                    "offset":         [round(float(v),4) for v in offset],
                })))

                # ── CSV ────────────────────────────────────────────────────────
                row = {
                    "t":        round(t_wall, 6),
                    "x":        round(new_x, 6),
                    "y":        round(new_y, 6),
                    "z":        round(new_z, 6),
                    "success":  r["success"],
                    "ik_ms":    round(ik_ms,   4),
                    "total_ms": round(total_ms, 4),
                    "tx": round(tx, 6), "ty": round(ty, 6), "tz": round(tz, 6),
                    "robot_ms":   round(r["robot_ms"], 4),
                    "setup_ms":   round(r["setup_ms"], 4),
                    "loop_ms":    round(r["loop_ms"],  4),
                    "iterations": r["iterations"],
                    "iter_ms":    round(r["iter_ms"], 5),
                    "pos_err_mm": round(r["pos_err_mm"], 4),
                    "track_err_mm": round(track_err, 4),
                    "mem_kb":     r["mem_kb"],
                    "deadline_missed": deadline_missed,
                }
                self._csv_writer.writerow([row.get(f, "") for f in _CSV_FIELDS])
                self._csv_fh.flush()
                self._records.append(row)

                # ── Console print ─────────────────────────────────────────────
                if step_count % 5 == 0 or getattr(self.args, "verbose", False):
                    tag = "✓" if r["success"] else "✗"
                    dlm = "!" if deadline_missed else " "
                    print(
                        f"\r  {step_count:5d}"
                        f"  {ik_ms:7.2f}"
                        f"  {r['robot_ms']:6.2f}"
                        f"  {r['loop_ms']:6.2f}"
                        f"  {r['iterations']:5d}"
                        f"  {r['pos_err_mm']:6.2f}"
                        f"  {track_err:6.2f}"
                        f"  {r['mem_kb']/1024:6.1f}"
                        f"  {dlm}{tag}",
                        end="", flush=True,
                    )
                if step_count % 50 == 0:
                    self._print_partial_stats()

            sleep_s = dt_sec - (time.perf_counter() - t_step)
            if sleep_s > 0:
                time.sleep(sleep_s)

    def _print_partial_stats(self):
        rows = self._records
        if not rows:
            return
        n    = len(rows)
        ik   = [r["ik_ms"]    for r in rows]
        loop = [r["loop_ms"]  for r in rows]
        itr  = [r["iterations"] for r in rows]
        ok   = sum(r["success"]          for r in rows)
        miss = sum(r["deadline_missed"]  for r in rows)
        print(f"\n  ─── [{n} steps] sr={ok/n*100:.0f}%  "
              f"ik_med={float(np.median(ik)):.2f}ms  "
              f"loop_med={float(np.median(loop)):.2f}ms  "
              f"iters_med={float(np.median(itr)):.1f}  "
              f"deadline_miss={miss}/{n}={miss/n*100:.0f}%  "
              f"offset=[{float(self._offset[0]):+.3f},"
              f"{float(self._offset[1]):+.3f},"
              f"{float(self._offset[2]):+.3f}]")

    def print_final_stats(self):
        rows = self._records
        if not rows:
            return
        print(f"\n\n{'═'*65}")
        mode = "rebuild" if self.args.rebuild else "CACHED+early_exit"
        print(f"  Final stats  arm={self.args.arm}  mode={mode}  n={len(rows)}")
        if self._calib_done and self._offset is not None:
            print(f"  calib_offset = [{float(self._offset[0]):+.4f}, "
                  f"{float(self._offset[1]):+.4f}, "
                  f"{float(self._offset[2]):+.4f}] m")
        print(f"{'═'*65}")
        for key, unit in [
            ("ik_ms",       "ms  ← total IK"),
            ("robot_ms",    "ms  ← RobotWrapper build"),
            ("setup_ms",    "ms  ← KinematicsSolver + tasks"),
            ("loop_ms",     "ms  ← solve loop"),
            ("iterations",  "iters"),
            ("iter_ms",     "ms/iter"),
            ("pos_err_mm",  "mm  ← IK residual"),
            ("track_err_mm","mm  ← EE vs target"),
        ]:
            vals = [float(r.get(key, 0)) for r in rows]
            if not vals:
                continue
            print(f"  {key:<16} mean={np.mean(vals):7.3f}  "
                  f"median={np.median(vals):7.3f}  "
                  f"p95={np.percentile(vals,95):7.3f}  "
                  f"max={max(vals):7.3f}  [{unit}]")
        n    = len(rows)
        miss = sum(r["deadline_missed"] for r in rows)
        ok   = sum(r["success"]         for r in rows)
        print(f"\n  success_rate : {ok}/{n} = {ok/n*100:.1f}%")
        print(f"  deadline_miss: {miss}/{n} = {miss/n*100:.1f}%"
              f"  (budget={self._deadline_ms:.1f}ms)")
        print(f"\n  CSV: {self._csv_path}")
        try:
            self._save_plot()
        except Exception as e:
            print(f"  [plot] skipped ({e})")

    def _save_plot(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        rows = self._records
        if not rows:
            return
        plot_path = getattr(self.args, "plot", "") or _png_for(self._csv_path)

        t      = list(range(len(rows)))
        ik_ms_ = [r["ik_ms"]          for r in rows]
        rob_ms = [r["robot_ms"]        for r in rows]
        lp_ms  = [r["loop_ms"]         for r in rows]
        iters_ = [r["iterations"]      for r in rows]
        pos_e  = [r["pos_err_mm"]      for r in rows]
        trk_e  = [r["track_err_mm"]    for r in rows]
        missed = [r["deadline_missed"] for r in rows]
        succ   = [r["success"]         for r in rows]

        fig, axes = plt.subplots(3, 2, figsize=(14, 12))
        mode = "rebuild" if self.args.rebuild else "cached"
        fig.suptitle(
            f"Placo Absolute Profiler  arm={self.args.arm}  mode={mode}\n"
            f"n={len(rows)}  rate={self._rate_hz:.0f}Hz  "
            f"deadline={self._deadline_ms:.1f}ms",
            fontsize=11)

        def _hm(ax, v, **kw):
            ax.axhline(float(np.mean(v)), linestyle="--", linewidth=0.8, **kw)

        # IK timing stacked
        ax = axes[0, 0]
        ax.stackplot(t,
            [r["robot_ms"] for r in rows],
            [r["setup_ms"] for r in rows],
            [r["loop_ms"]  for r in rows],
            labels=["robot_build", "setup", "loop"],
            colors=["tomato", "gold", "steelblue"], alpha=0.8)
        ax.axhline(self._deadline_ms, color="red", linestyle="--", linewidth=1.2,
                   label=f"deadline={self._deadline_ms:.0f}ms")
        ax.set_title("IK time breakdown (ms)", fontsize=9)
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        # Iterations
        ax = axes[0, 1]
        ax.plot(t, iters_, color="purple", linewidth=0.7)
        _hm(ax, iters_, color="red", label=f"mean={np.mean(iters_):.1f}")
        ax.set_title("Iterations per step", fontsize=9)
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        # IK residual
        ax = axes[1, 0]
        ax.plot(t, pos_e, color="tomato", linewidth=0.7)
        _hm(ax, pos_e, color="darkred", label=f"mean={np.mean(pos_e):.2f}mm")
        ax.set_title("IK position residual (mm)", fontsize=9)
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        # Track error
        ax = axes[1, 1]
        ax.plot(t, trk_e, color="darkorange", linewidth=0.7)
        _hm(ax, trk_e, color="saddlebrown", label=f"mean={np.mean(trk_e):.2f}mm")
        ax.set_title("EE tracking error (mm)", fontsize=9)
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        # Deadline miss
        ax = axes[2, 0]
        ax.bar(["hit", "miss"],
               [sum(1 for v in missed if not v), sum(missed)],
               color=["steelblue", "red"])
        ax.set_title(f"Deadline: {sum(missed)}/{len(missed)} missed "
                     f"({sum(missed)/max(len(missed),1)*100:.1f}%)", fontsize=9)

        # EE trajectory (XY)
        ax = axes[2, 1]
        xs = [r["x"] for r in rows]
        ys = [r["y"] for r in rows]
        sc = ax.scatter(xs, ys, c=ik_ms_, cmap="plasma", s=8, alpha=0.7)
        plt.colorbar(sc, ax=ax, label="ik_ms")
        ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")
        ax.set_title("EE XY trajectory (colour=ik_ms)", fontsize=9)
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(plot_path, dpi=120)
        plt.close()
        print(f"  [plot] Saved → {plot_path}")

    def destroy_node(self):
        if self._csv_fh:
            self._csv_fh.close()
        super().destroy_node()


# ── CLI ───────────────────────────────────────────────────────────────────────
def _parse_args():
    p = argparse.ArgumentParser(
        description="Placo IK absolute profiler — subscribes /ee_target/{arm}",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--arm",       default="right", choices=["right", "left"])
    p.add_argument("--rate",      type=float, default=20.0,
                   help="Control loop Hz (default: 20)")
    p.add_argument("--horizon",   type=float, default=60.0,
                   help="JointTrajectory duration ms (default: 60)")
    p.add_argument("--max-iter",  type=int,   default=_MAX_ITER, dest="max_iter",
                   help=f"Max iterations (default: {_MAX_ITER})")
    p.add_argument("--dt",        type=float, default=_DT,
                   help=f"Placo solver dt (default: {_DT})")
    p.add_argument("--rebuild",   action="store_true",
                   help="Rebuild RobotWrapper every step (original ~25ms, for comparison)")
    # Calibration
    p.add_argument("--no-auto-calib", action="store_true", dest="no_auto_calib",
                   help="Skip auto-calibration, use raw tracker coordinates")
    p.add_argument("--offset",    default=None,
                   help="Manual offset 'x,y,z' [m] (overrides auto-calib). "
                        "Example: --offset 0.1,0.0,-0.2")
    # Output
    p.add_argument("--csv",       default="", help="Output CSV path")
    p.add_argument("--plot",      default="", help="Output plot PNG path")
    p.add_argument("--dry-run",   action="store_true", dest="dry_run",
                   help="Compute IK but do not publish trajectory")
    p.add_argument("--home-first", action="store_true", dest="home_first",
                   help="Send arm to home position (with TF confirmation) before starting")
    p.add_argument("--ws-mesh",    default=None, dest="ws_mesh",
                   help="Path to WorkspaceMesh .npz (from placo_ws_analyze.py). "
                        "Replaces rectangular box clamp with real scanned workspace shape.")
    p.add_argument("--no-rot",    action="store_true", dest="no_rot",
                   help="Step 1: position-only IK (W_ORI=0). "
                        "Verify position+mesh mapping before enabling orientation "
                        "tracking (Step 2 = without this flag).")
    p.add_argument("--verbose",   action="store_true",
                   help="Print every step (not just every 5th)")
    return p.parse_args()


def main():
    global args
    args = _parse_args()
    # Auto-detect WorkspaceMesh npz for the selected arm
    if not getattr(args, "ws_mesh", None):
        _auto = _ws_mesh_path(args.arm)
        if os.path.isfile(_auto):
            args.ws_mesh = _auto
            print(f"  [ws_mesh] auto-detected: {_auto}")

    rclpy.init()
    node = PlacoAbsoluteProfiler(args)

    executor_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()

    try:
        if args.home_first:
            print("  Moving to home (with TF confirmation)...")
            node.send_home_confirmed(pos_tol=0.025, motion_sec=3.5, max_tries=5)

        node.run()
    except KeyboardInterrupt:
        print("\n\n  Ctrl-C — stopping...")
    finally:
        node.print_final_stats()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    import os
    main()
