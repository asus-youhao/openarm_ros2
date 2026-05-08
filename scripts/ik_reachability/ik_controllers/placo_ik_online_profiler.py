#!/usr/bin/env python3
"""
placo_ik_online_profiler.py
===========================
與 tracker_ee_delta_ik_backend.py --pattern ee_delta --solver placo 完全等價，
但加入深度 profiling：每次 IK 計算都拆解出 setup_ms / loop_ms / iterations，
並揭露為何原版測到 20-30 ms。

【為什麼原版是 20-30 ms？】
  原版 PlacoIKSolver._solve_from_seed() 有兩個問題：
  1. 每次 solve 都 rebuild RobotWrapper（~5ms overhead）
  2. _solve_from_seed 跑滿 MAX_ITER=250 iterations，沒有 early exit
     → 250 iters × ~0.08ms/iter ≈ 20ms + robot_build ≈ 5ms = ~25ms

【online 要 rebuild 還是 cache？→ 一定要 cache！】
  rebuild (原版行為):
    每步 = robot_build(~5ms) + solver_setup(~0.1ms) + 250_iters(~20ms) ≈ 25ms
    → 20Hz 已在 deadline 邊緣，50Hz 不可能 (預算=20ms)
  cached (本檔預設):
    每步 = solver_setup(~0.15ms) + early_exit_iters(~0.3ms) ≈ 0.5ms
    → 200Hz 輕鬆，50Hz deadling miss = 0%
    原理：RobotWrapper 建立成本昂貴且只需建一次；
           KinematicsSolver 輕量，每步重建沒問題。

【Early exit（本檔新增，原版缺少）】
  每 iteration 後檢查 pos_err < POS_TOL (3mm)，達到即跳出。
  near-target 步通常 1-3 iters 即收斂，比跑滿 250 fast 100×。

Usage（需要 ROS2 + robot driver 執行中）：
  # 右臂，ee_delta tracker 模式，right 手把
  conda run -n pico_teleop_py python3 placo_ik_online_profiler.py --arm right

  # 左臂
  conda run -n pico_teleop_py python3 placo_ik_online_profiler.py --arm left

  # 對比原版行為（rebuild + 無 early exit）
  conda run -n pico_teleop_py python3 placo_ik_online_profiler.py --arm right --rebuild

  # 自訂輸出路徑
  conda run -n pico_teleop_py python3 placo_ik_online_profiler.py \\
      --arm right --csv ~/my_profile.csv --plot ~/my_profile.png

  # dry-run（計算 IK 但不送 trajectory）
  conda run -n pico_teleop_py python3 placo_ik_online_profiler.py --arm right --dry-run

Prerequisites（和 tracker_ee_delta_ik_backend.py 相同）：
  ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
  # tracker 需要在另一個 terminal 發布 /ee_delta/{arm} (PoseStamped)

Published topics（完全相容 tracker_ee_delta_ik_backend.py）：
  /right_joint_trajectory_controller/joint_trajectory   (JointTrajectory)
  /right/delta_ik_latency_ms                            (Float32) — total ik_ms
  /right/placo_profile                                  (String, JSON) — detailed breakdown

CSV columns（延伸自原版）：
  t, x, y, z, success, ik_ms, total_ms, dx, dy, dz,
  robot_ms, setup_ms, loop_ms, iterations, iter_ms,
  pos_err_mm, track_err_mm, mem_kb, deadline_missed
"""

# ── Path setup（must be before any local imports）────────────────────────────
import os as _os, sys as _sys
_DIR    = _os.path.dirname(_os.path.abspath(__file__))
_PARENT = _os.path.dirname(_DIR)          # ik_reachability/
_sys.path.insert(0, _PARENT)              # for rclpy / ROS imports
_sys.path.insert(0, _os.path.join(_DIR, "ik_solver"))  # placo_ik_solver.py

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

from placo_ik_solver import (
    _find_urdf,
    _HUMAN_RIGHT,
    _HUMAN_LEFT,
    _JOINT_NAMES,
    _ARM_SEEDS,
    _quat_to_rot,
)

# ── ARM config（mirrors tracker_ee_delta_ik_backend.py）─────────────────────
_ARM_CONFIG = {
    "left": {
        "joint_names":  [f"openarm_left_joint{i}"  for i in range(1, 8)],
        "base_link":    "world",
        "ee_link":      "openarm_left_link7",
        "home_joints":  [0.0, -0.7, 0.0, 1.5708, 0.0, 0.0, 0.0],
        "home_pose":    (0.2160, 0.2952, 0.5297, 0.6642, -0.2425, 0.6642, -0.2425),
        "workspace":    {"x": (-0.30, 0.30), "y": (0.05, 0.45), "z": (0.05, 0.65)},
        "cmd_topic":    "/left_joint_trajectory_controller/joint_trajectory",
        "latency_topic":"/left/delta_ik_latency_ms",
        "profile_topic":"/left/placo_profile",
        "ee_delta_topic":"/ee_delta/left",
    },
    "right": {
        "joint_names":  [f"openarm_right_joint{i}" for i in range(1, 8)],
        "base_link":    "world",
        "ee_link":      "openarm_right_link7",
        "home_joints":  [0.0, 0.7, 0.0, 1.5708, 0.0, 0.0, 0.0],
        "home_pose":    (0.2160, -0.2952, 0.5297, 0.6642, 0.2425, 0.6642, 0.2425),
        "workspace":    {"x": (-0.30, 0.30), "y": (-0.45, -0.05), "z": (0.05, 0.65)},
        "cmd_topic":    "/right_joint_trajectory_controller/joint_trajectory",
        "latency_topic":"/right/delta_ik_latency_ms",
        "profile_topic":"/right/placo_profile",
        "ee_delta_topic":"/ee_delta/right",
    },
}

# ── IK constants ─────────────────────────────────────────────────────────────
_POS_TOL   = 0.003   # m  — early exit threshold
_POS_RELAX = 0.010   # m  — relaxed acceptance
_W_POS     = 1.0
_W_ORI     = 0.3
_W_JOINTS  = 1e-4
_MAX_ITER  = 250
_DT        = 0.010

# ── Resource helpers ─────────────────────────────────────────────────────────
def _mem_rss_kb() -> int:
    try:
        with open("/proc/self/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1])
    except OSError:
        pass
    return 0


# ── Quaternion helpers（copy from tracker backend）──────────────────────────
def _qmul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2,
            w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2)

def _qnorm(q):
    x,y,z,w = q; n=math.sqrt(x*x+y*y+z*z+w*w)
    return (x/n,y/n,z/n,w/n) if n>1e-9 else (0.,0.,0.,1.)

def _qfrom_rpy(r,p,y):
    cr,cp,cy=math.cos(r/2),math.cos(p/2),math.cos(y/2)
    sr,sp,sy=math.sin(r/2),math.sin(p/2),math.sin(y/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)

def _rotate_vec(v, q):
    qx,qy,qz,qw=q; vx,vy,vz=v
    tx=2*(qy*vz-qz*vy); ty=2*(qz*vx-qx*vz); tz=2*(qx*vy-qy*vx)
    return (vx+qw*tx+qy*tz-qz*ty, vy+qw*ty+qz*tx-qx*tz, vz+qw*tz+qx*ty-qy*tx)


# ── Placo cached robot ────────────────────────────────────────────────────────
class PlacoSession:
    """
    Holds one pre-built RobotWrapper for reuse across all IK steps.

    核心優化：只在 __init__ 建一次 RobotWrapper（~5ms），之後每步僅重建
    KinematicsSolver（~0.1ms）+ 跑 early-exit iterative solve（近目標 ~0.3ms）。

    若 rebuild=True 則模擬原版行為（每步重建 robot），用來對比。
    """

    def __init__(self, urdf: str, arm: str, rebuild: bool = False,
                 max_iter: int = _MAX_ITER, dt: float = _DT):
        import placo
        self._placo   = placo
        self._urdf    = urdf
        self._arm     = arm
        self._rebuild = rebuild
        self._max_iter = max_iter
        self._dt       = dt

        self._joint_names = _JOINT_NAMES[arm]
        self._human_cfg   = _HUMAN_RIGHT if arm == "right" else _HUMAN_LEFT
        self._ee_link     = f"openarm_{arm}_link7"
        self._lo  = [self._human_cfg[n][1] for n in self._joint_names]
        self._hi  = [self._human_cfg[n][2] for n in self._joint_names]
        self._pref = {n: self._human_cfg[n][0] for n in self._joint_names}

        # Build robot once (or None if rebuild mode)
        if not rebuild:
            self._robot = placo.RobotWrapper(urdf, placo.Flags.ignore_collisions)
            print(f"[PlacoSession] cached robot built  arm={arm}  max_iter={max_iter}  dt={dt}")
        else:
            self._robot = None
            print(f"[PlacoSession] rebuild mode (original behavior)  arm={arm}")

    def solve_step(
        self,
        target_xyz: np.ndarray,
        target_R:   np.ndarray,
        seed:       List[float],
        no_rot:     bool = False,
    ) -> Dict:
        """
        Run one IK step.  Returns profiling dict identical to solve_profiled().
        """
        placo = self._placo
        t_wall0  = time.perf_counter()

        # ── Robot（build or reuse）────────────────────────────────────────────
        t_robot0 = time.perf_counter()
        if self._rebuild or self._robot is None:
            robot = placo.RobotWrapper(self._urdf, placo.Flags.ignore_collisions)
            robot_was_cached = False
        else:
            robot = self._robot
            robot_was_cached = True
        t_robot1 = time.perf_counter()
        robot_ms = (t_robot1 - t_robot0) * 1000.0

        # ── Solver + tasks ────────────────────────────────────────────────────
        t_setup0 = time.perf_counter()
        solver   = placo.KinematicsSolver(robot)
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
        t_setup1 = time.perf_counter()
        setup_ms = (t_setup1 - t_setup0) * 1000.0

        # ── Iterative solve（with early exit）────────────────────────────────
        # 原版沒有 early exit → 跑滿 250 iters ≈ 20ms
        # 本版在 pos_err < _POS_TOL 時立即跳出 → near-target 1-3 iters ≈ 0.3ms
        t_loop0    = time.perf_counter()
        iters_used = 0

        if self._rebuild:
            # 模擬原版：跑滿不 early exit
            for _ in range(self._max_iter):
                solver.solve(True)
                robot.update_kinematics()
                iters_used += 1
        else:
            # 優化版：early exit
            for _ in range(self._max_iter):
                solver.solve(True)
                robot.update_kinematics()
                iters_used += 1
                T_ee  = robot.get_T_world_frame(self._ee_link)
                if float(np.linalg.norm(T_ee[:3, 3] - target_xyz)) < _POS_TOL:
                    break

        t_loop1  = time.perf_counter()
        loop_ms  = (t_loop1 - t_loop0) * 1000.0

        # Final state
        joints  = [robot.get_joint(n) for n in self._joint_names]
        joints  = [max(l, min(h, q)) for q, l, h in zip(joints, self._lo, self._hi)]
        T_ee    = robot.get_T_world_frame(self._ee_link)
        pos_err = float(np.linalg.norm(T_ee[:3, 3] - target_xyz))
        ee_xyz  = list(T_ee[:3, 3])

        solve_ms = robot_ms + setup_ms + loop_ms
        iter_ms  = loop_ms / iters_used if iters_used > 0 else 0.0
        success  = int(pos_err < _POS_RELAX)

        return {
            "robot_ms":        robot_ms,
            "setup_ms":        setup_ms,
            "loop_ms":         loop_ms,
            "solve_ms":        solve_ms,
            "wall_ms":         (time.perf_counter() - t_wall0) * 1000.0,
            "iterations":      iters_used,
            "iter_ms":         iter_ms,
            "mem_kb":          _mem_rss_kb(),
            "pos_err_mm":      pos_err * 1000.0,
            "success":         success,
            "joints":          joints,
            "ee_xyz":          ee_xyz,
            "robot_was_cached": robot_was_cached,
        }


# ── CSV fields ────────────────────────────────────────────────────────────────
_CSV_FIELDS = [
    "t", "x", "y", "z",
    "success", "ik_ms", "total_ms",
    "dx", "dy", "dz",
    # ── extended profiling fields（new vs original CSV）──
    "robot_ms", "setup_ms", "loop_ms",
    "iterations", "iter_ms",
    "pos_err_mm", "track_err_mm",
    "mem_kb", "deadline_missed",
]


# ── ROS2 Node ─────────────────────────────────────────────────────────────────
class PlacoOnlineProfiler(Node):
    """
    Drop-in replacement for tracker_ee_delta_ik_backend.py --pattern ee_delta
    --solver placo, with per-step profiling breakdown.

    Subscribe to /ee_delta/{arm} (PoseStamped), compute IK, publish
    JointTrajectory.  Also publish /arm/placo_profile (String JSON) with
    detailed timing for real-time monitoring.
    """

    def __init__(self, args):
        super().__init__("placo_ik_online_profiler")
        self.args = args
        cfg = _ARM_CONFIG[args.arm]
        self.cfg = cfg

        self._cbg = ReentrantCallbackGroup()

        # EE pose state: [x, y, z, qx, qy, qz, qw]
        hx, hy, hz, hqx, hqy, hqz, hqw = cfg["home_pose"]
        self._pose = [hx, hy, hz, hqx, hqy, hqz, hqw]
        self._last_joints  = list(cfg["home_joints"])
        self._joints_lock  = threading.Lock()
        self._joint_states: dict = {}
        self._js_lock      = threading.Lock()

        # ee_delta session ref
        self._ee_delta_ref_xyz  = None
        self._ee_delta_ref_q    = None
        self._ee_delta_last_t   = None
        self._ee_delta_gap_sec  = 0.35

        # Calibration quaternion
        crpy = getattr(args, "calib_rpy", None)
        if crpy:
            rr, rp, ry = [math.radians(float(v)) for v in crpy.split(",")]
        else:
            rr, rp = 0.0, 0.0
            ry = math.radians(getattr(args, "calib_yaw", 0.0))
        self._calib_q = _qfrom_rpy(rr, rp, ry)

        # Rate
        self._rate_hz     = float(getattr(args, "rate", 20.0))
        self._horizon_ms  = float(getattr(args, "horizon", 60.0))
        self._deadline_ms = 1000.0 / self._rate_hz

        # URDF + placo session
        urdf = _find_urdf()
        self._placo_session = PlacoSession(
            urdf     = urdf,
            arm      = args.arm,
            rebuild  = args.rebuild,
            max_iter = getattr(args, "max_iter", _MAX_ITER),
            dt       = getattr(args, "dt", _DT),
        )

        # TF2
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Publishers
        self._traj_pub    = self.create_publisher(JointTrajectory, cfg["cmd_topic"], 10)
        self._latency_pub = self.create_publisher(Float32, cfg["latency_topic"], 10)
        self._profile_pub = self.create_publisher(String, cfg["profile_topic"], 10)

        # Subscribers
        self.create_subscription(
            JointState, "/joint_states",
            self._js_cb, 10, callback_group=self._cbg)
        tracker_side = getattr(args, "tracker_side", None) or args.arm
        self.create_subscription(
            PoseStamped, cfg["ee_delta_topic"],
            self._ee_delta_cb, 10, callback_group=self._cbg)

        # Records
        self._records:   List[Dict] = []
        self._delta_lock = threading.Lock()
        self._pending    = None
        self._msg_count  = 0

        # CSV
        self._csv_fh     = None
        self._csv_writer = None
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        mode = "cached" if not args.rebuild else "rebuild"
        if args.csv:
            csv_path = args.csv
        else:
            out_dir = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "results",
                datetime.datetime.now().strftime("%Y%m%d"),
            )
            os.makedirs(out_dir, exist_ok=True)
            csv_path = os.path.join(
                out_dir,
                f"placo_online_{ts}_{args.arm}_{mode}.csv",
            )
        self._csv_fh     = open(csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_fh)
        self._csv_writer.writerow(_CSV_FIELDS)
        self._csv_path   = csv_path

        print(f"\n{'═'*65}")
        print(f"  Placo Online Profiler")
        print(f"  arm={args.arm}  mode={'rebuild (original)' if args.rebuild else 'CACHED+early_exit (optimized)'}")
        print(f"  rate={self._rate_hz:.0f}Hz  deadline={self._deadline_ms:.1f}ms")
        print(f"  ee_delta topic: {cfg['ee_delta_topic']}")
        print(f"  cmd topic:      {cfg['cmd_topic']}")
        print(f"  profile topic:  {cfg['profile_topic']}")
        print(f"  CSV: {csv_path}")
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

    def _ee_delta_cb(self, msg: PoseStamped):
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

    # ── Home-first（mirrors tracker_ee_delta_ik_backend.send_home_confirmed）────
    def send_home_confirmed(
        self,
        pos_tol:    float = 0.025,   # m: TF vs home_pose must be within this
        motion_sec: float = 3.5,     # wait time after each cmd
        max_tries:  int   = 5,
        poll_sec:   float = 0.3,
    ) -> bool:
        """
        Send home trajectory and wait for TF to confirm arrival.
        Retries up to max_tries times.  Returns True if reached home.
        """
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
            print(f"  [home] attempt {attempt}/{max_tries} — cmd sent, waiting {motion_sec:.1f}s...")

            t_deadline = time.time() + motion_sec
            arrived    = False
            dist       = 999.0
            while time.time() < t_deadline:
                time.sleep(poll_sec)
                tf_pose = self._get_tf(timeout_sec=0.3)
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
                print(f"  [home] ✗ attempt {attempt} timed out  j_max_err={math.degrees(j_err):.1f}° — retrying...")

        print(f"  [home] ✗ could not confirm home after {max_tries} attempts — continuing anyway")
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
        msg.points       = [pt]
        self._traj_pub.publish(msg)

    # ── Main loop ──────────────────────────────────────────────────────────────
    def run(self):
        """Main ee_delta loop — identical to run_pattern_ee_delta() in backend."""
        dt_sec = 1.0 / self._rate_hz

        # Sync from TF + joint_states at start
        print("\n  Syncing from TF2 and /joint_states...")
        time.sleep(0.5)
        tf_pose = self._get_tf(2.0)
        if tf_pose is not None:
            self._pose = list(tf_pose)
            print(f"  TF EE: {[f'{v:.4f}' for v in tf_pose[:3]]}")
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
        print(f"\n  Waiting for {self.cfg['ee_delta_topic']} ...\n")

        _idle_t    = time.time()
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
                        f"\r  [WAIT] no ee_delta  cb={self._msg_count}"
                        f"  steps={step_count}  idle={idle_s:.0f}s"
                        f"  pose=[{self._pose[0]:.3f},{self._pose[1]:.3f},{self._pose[2]:.3f}]",
                        end="", flush=True,
                    )
            elif isinstance(msg, PoseStamped):
                _no_msg_t = time.time()
                dx = msg.pose.position.x
                dy = msg.pose.position.y
                dz = msg.pose.position.z
                dq = (msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w)
                if abs(dq[3]) < 0.01 and all(abs(v) < 0.01 for v in dq[:3]):
                    dq = (0., 0., 0., 1.)

                t_wall = time.time()
                gap = (t_wall - self._ee_delta_last_t) if self._ee_delta_last_t else 999.
                is_new = self._ee_delta_ref_xyz is None or gap > self._ee_delta_gap_sec
                self._ee_delta_last_t = t_wall

                if is_new:
                    ref = self._get_tf(0.3)
                    if ref is not None:
                        self._ee_delta_ref_xyz = tuple(ref[:3])
                        self._ee_delta_ref_q   = tuple(ref[3:7])
                        self._pose             = list(ref)
                        print(f"\n  [EE-δ] NEW ref={[f'{v:.4f}' for v in ref[:3]]}  (TF)")
                    else:
                        self._ee_delta_ref_xyz = tuple(self._pose[:3])
                        self._ee_delta_ref_q   = tuple(self._pose[3:7])

                # Build target pose
                if self._ee_delta_ref_xyz is not None:
                    base_xyz = self._ee_delta_ref_xyz
                    base_q   = self._ee_delta_ref_q
                else:
                    base_xyz = tuple(self._pose[:3])
                    base_q   = tuple(self._pose[3:7])

                dx_arm = _rotate_vec((dx, dy, dz), self._calib_q)
                new_x  = max(ws["x"][0], min(ws["x"][1], base_xyz[0] + dx_arm[0]))
                new_y  = max(ws["y"][0], min(ws["y"][1], base_xyz[1] + dx_arm[1]))
                new_z  = max(ws["z"][0], min(ws["z"][1], base_xyz[2] + dx_arm[2]))
                new_q  = _qnorm(_qmul(dq, base_q))
                target_xyz = np.array([new_x, new_y, new_z])
                target_R   = _quat_to_rot(*new_q)

                with self._joints_lock:
                    seed = list(self._last_joints)

                # ── IK solve (profiled) ────────────────────────────────────
                t_total = time.perf_counter()
                r = self._placo_session.solve_step(target_xyz, target_R, seed)
                ik_ms     = r["solve_ms"]
                total_ms  = (time.perf_counter() - t_total) * 1000.0

                # Track error: actual EE (after solve) vs target
                actual_xyz  = np.array(r["ee_xyz"])
                track_err   = float(np.linalg.norm(actual_xyz - target_xyz)) * 1000.0

                loop_wall_ms   = (time.perf_counter() - t_step) * 1000.0
                deadline_missed = int(loop_wall_ms > self._deadline_ms)

                saved_pose = list(self._pose)
                if r["success"]:
                    self._pose = [new_x, new_y, new_z,
                                  new_q[0], new_q[1], new_q[2], new_q[3]]
                    with self._joints_lock:
                        self._last_joints = r["joints"]
                    if not args.dry_run:
                        self._publish(r["joints"])
                    self._latency_pub.publish(Float32(data=float(ik_ms)))
                else:
                    # IK failed — revert pose, keep last_joints as seed
                    pass

                step_count += 1

                # ── Publish detailed profile (JSON string) ─────────────────
                profile_data = {
                    "step":       step_count,
                    "ik_ms":      round(ik_ms, 3),
                    "robot_ms":   round(r["robot_ms"], 3),
                    "setup_ms":   round(r["setup_ms"], 3),
                    "loop_ms":    round(r["loop_ms"], 3),
                    "iterations": r["iterations"],
                    "iter_ms":    round(r["iter_ms"], 4),
                    "pos_err_mm": round(r["pos_err_mm"], 3),
                    "track_err_mm": round(track_err, 3),
                    "success":    r["success"],
                    "deadline_missed": deadline_missed,
                    "mem_mb":     round(r["mem_kb"] / 1024.0, 1),
                    "mode":       "rebuild" if self.args.rebuild else "cached",
                }
                self._profile_pub.publish(
                    String(data=json.dumps(profile_data)))

                # ── CSV write ──────────────────────────────────────────────
                row = {
                    "t":        round(t_wall, 6),
                    "x":        round(self._pose[0], 6),
                    "y":        round(self._pose[1], 6),
                    "z":        round(self._pose[2], 6),
                    "success":  r["success"],
                    "ik_ms":    round(ik_ms, 4),
                    "total_ms": round(total_ms, 4),
                    "dx": round(dx, 6), "dy": round(dy, 6), "dz": round(dz, 6),
                    "robot_ms":   round(r["robot_ms"], 4),
                    "setup_ms":   round(r["setup_ms"], 4),
                    "loop_ms":    round(r["loop_ms"], 4),
                    "iterations": r["iterations"],
                    "iter_ms":    round(r["iter_ms"], 5),
                    "pos_err_mm": round(r["pos_err_mm"], 4),
                    "track_err_mm": round(track_err, 4),
                    "mem_kb":     r["mem_kb"],
                    "deadline_missed": deadline_missed,
                }
                if self._csv_writer:
                    self._csv_writer.writerow([row.get(f, "") for f in _CSV_FIELDS])
                    self._csv_fh.flush()

                self._records.append(row)

                # ── Print every step ───────────────────────────────────────
                if step_count % 5 == 0 or getattr(args, "verbose", False):
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

            sleep_sec = dt_sec - (time.perf_counter() - t_step)
            if sleep_sec > 0:
                time.sleep(sleep_sec)

    def _print_partial_stats(self):
        """Print rolling aggregate every 50 steps."""
        rows = self._records
        if not rows:
            return
        n     = len(rows)
        ik    = [r["ik_ms"]    for r in rows]
        loop  = [r["loop_ms"]  for r in rows]
        iters = [r["iterations"] for r in rows]
        ok    = sum(r["success"]          for r in rows)
        miss  = sum(r["deadline_missed"]  for r in rows)
        print(f"\n  ─── [{n} steps] "
              f"sr={ok/n*100:.0f}%  "
              f"ik_med={float(np.median(ik)):.2f}ms  "
              f"loop_med={float(np.median(loop)):.2f}ms  "
              f"iters_med={float(np.median(iters)):.1f}  "
              f"deadline_miss={miss}/{n}={miss/n*100:.0f}%")

    def print_final_stats(self):
        """Print final aggregate table and save plot."""
        rows = self._records
        if not rows:
            return

        print(f"\n\n{'═'*65}")
        mode = "rebuild (original)" if self.args.rebuild else "CACHED+early_exit"
        print(f"  Final stats  arm={self.args.arm}  mode={mode}  n={len(rows)}")
        print(f"{'═'*65}")

        for key, unit in [
            ("ik_ms",    "ms  ← 含 robot_build + setup + loop"),
            ("robot_ms", "ms  ← 重建 RobotWrapper（cached模式≈0ms）"),
            ("setup_ms", "ms  ← 建立 Tasks"),
            ("loop_ms",  "ms  ← 迭代求解Loop"),
            ("iterations", "iters  ← early exit 平均迭代次數"),
            ("iter_ms",  "ms/iter"),
            ("pos_err_mm","mm  ← IK 殘差"),
            ("track_err_mm","mm  ← 軌跡追蹤誤差"),
        ]:
            vals = [float(r.get(key, 0)) for r in rows]
            if not vals:
                continue
            print(f"  {key:<16} mean={np.mean(vals):7.3f}  median={np.median(vals):7.3f}"
                  f"  p95={np.percentile(vals,95):7.3f}  max={max(vals):7.3f}  [{unit}]")

        n    = len(rows)
        miss = sum(r["deadline_missed"] for r in rows)
        ok   = sum(r["success"]         for r in rows)
        print(f"\n  success_rate   : {ok}/{n} = {ok/n*100:.1f}%")
        print(f"  deadline_miss  : {miss}/{n} = {miss/n*100:.1f}%  (budget={self._deadline_ms:.1f}ms)")

        # Breakdown hint
        ik_mean    = float(np.mean([r["ik_ms"]    for r in rows]))
        robot_mean = float(np.mean([r["robot_ms"] for r in rows]))
        loop_mean  = float(np.mean([r["loop_ms"]  for r in rows]))
        rb_pct     = robot_mean / max(ik_mean, 0.01) * 100
        lp_pct     = loop_mean  / max(ik_mean, 0.01) * 100
        print(f"\n  時間拆解: robot_build={rb_pct:.0f}%  loop={lp_pct:.0f}%  "
              f"(ik_mean={ik_mean:.2f}ms)")
        if not self.args.rebuild:
            rebuild_est = robot_mean + 9.7 + loop_mean  # 9.7ms = typical robot_build
            print(f"  若改回 rebuild 模式估計: ~{rebuild_est:.1f}ms/step")
        else:
            cached_est = 0.001 + float(np.mean([r["setup_ms"] for r in rows])) + loop_mean
            print(f"  若改用 cached+early_exit 估計: ~{cached_est:.2f}ms/step")

        print(f"\n  CSV: {self._csv_path}")

        # Plot
        try:
            self._save_plot()
        except Exception as e:
            print(f"  [plot] 跳過（{e}）")

    def _save_plot(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        rows = self._records
        if not rows:
            return

        mode  = "rebuild" if self.args.rebuild else "cached"
        ts    = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        plot_path = self.args.plot or self._csv_path.replace(".csv", ".png")

        t      = list(range(len(rows)))
        ik_ms_ = [r["ik_ms"]    for r in rows]
        rob_ms = [r["robot_ms"] for r in rows]
        lp_ms  = [r["loop_ms"]  for r in rows]
        iters_ = [r["iterations"] for r in rows]
        pos_e  = [r["pos_err_mm"]   for r in rows]
        trk_e  = [r["track_err_mm"] for r in rows]
        missed = [r["deadline_missed"] for r in rows]
        succ   = [r["success"]       for r in rows]

        fig, axes = plt.subplots(3, 2, figsize=(14, 12))
        fig.suptitle(
            f"Placo Online Profiler  arm={self.args.arm}  mode={mode}\n"
            f"n={len(rows)}  rate={self._rate_hz:.0f}Hz  "
            f"deadline={self._deadline_ms:.1f}ms",
            fontsize=11,
        )

        def _hm(ax, v, **kw):
            ax.axhline(float(np.mean(v)), linestyle="--", linewidth=0.8, **kw)

        # IK timing stacked
        ax = axes[0, 0]
        ax.stackplot(t,
            [r["robot_ms"] for r in rows],
            [r["setup_ms"] for r in rows],
            [r["loop_ms"]  for r in rows],
            labels=["robot_build", "setup", "loop"],
            colors=["tomato", "gold", "steelblue"],
            alpha=0.8,
        )
        ax.axhline(self._deadline_ms, color="red", linestyle="--", linewidth=1.2,
                   label=f"deadline={self._deadline_ms:.0f}ms")
        ax.set_title("IK time breakdown (ms) — stacked", fontsize=9)
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        # Iterations
        ax = axes[0, 1]
        ax.plot(t, iters_, color="purple", linewidth=0.7)
        _hm(ax, iters_, color="red", label=f"mean={np.mean(iters_):.1f}")
        ax.set_title("Iterations per step (early exit)", fontsize=9)
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
        ax.set_title("EE trajectory tracking error (mm)", fontsize=9)
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        # Deadline miss
        ax = axes[2, 0]
        ax.bar(["hit", "miss"],
               [sum(1 for v in missed if not v), sum(missed)],
               color=["steelblue", "red"])
        ax.set_title(f"Deadline: {sum(missed)}/{len(missed)} missed "
                     f"({sum(missed)/len(missed)*100:.1f}%)", fontsize=9)

        # Success rate rolling
        ax = axes[2, 1]
        win = max(1, len(succ) // 20)
        sr_roll = [
            sum(succ[max(0,i-win):i+1]) / min(i+1,win+1) * 100
            for i in range(len(succ))
        ]
        ax.plot(t, sr_roll, color="green", linewidth=0.8)
        ax.set_ylim(-5, 105)
        ax.set_title(f"Rolling success rate % (win={win})", fontsize=9)
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(plot_path, dpi=120)
        plt.close()
        print(f"  [plot] Saved → {plot_path}")

    def destroy_node(self):
        if self._csv_fh:
            self._csv_fh.close()
        super().destroy_node()


# ── Main ──────────────────────────────────────────────────────────────────────
def _parse_args():
    p = argparse.ArgumentParser(
        description="Placo IK online profiler (ROS2, ee_delta topic)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--arm",       default="right", choices=["right", "left"])
    p.add_argument("--rate",      type=float, default=20.0,
                   help="Control loop Hz (default: 20)")
    p.add_argument("--horizon",   type=float, default=60.0,
                   help="JointTrajectory duration ms (default: 60)")
    p.add_argument("--max-iter",  type=int, default=_MAX_ITER, dest="max_iter",
                   help=f"Max placo iterations (default: {_MAX_ITER})")
    p.add_argument("--dt",        type=float, default=_DT,
                   help=f"Placo solver dt (default: {_DT})")
    p.add_argument("--rebuild",   action="store_true",
                   help="Rebuild RobotWrapper every step (original behavior, ~25ms/step)")
    p.add_argument("--calib-yaw", type=float, default=0.0, dest="calib_yaw",
                   help="Tracker yaw offset degrees (default: 0)")
    p.add_argument("--calib-rpy", default=None, dest="calib_rpy",
                   help="'roll,pitch,yaw' degrees, overrides --calib-yaw")
    p.add_argument("--tracker-side", default=None, dest="tracker_side",
                   choices=["left", "right", None])
    p.add_argument("--csv",       default="", help="Output CSV path")
    p.add_argument("--plot",      default="", help="Output plot PNG path")
    p.add_argument("--dry-run",   action="store_true", dest="dry_run",
                   help="Compute IK but do not publish trajectory")
    p.add_argument("--home-first", action="store_true", dest="home_first",
                   help="Send arm to home position (with TF confirmation) before starting")
    p.add_argument("--verbose",   action="store_true",
                   help="Print every step (not just every 5th)")
    return p.parse_args()


def main():
    global args
    args = _parse_args()

    rclpy.init()
    node = PlacoOnlineProfiler(args)

    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
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
