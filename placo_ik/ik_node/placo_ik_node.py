#!/usr/bin/env python3
"""
placo_ik_node.py
================
ROS2 Node: real-time IK controller + profiler.

Imports
-------
  PlacoSession  ← placo_ik_session.py   (pure-Python IK)
  KbdController ← kbd_controller.py     (termios keyboard daemon)

Fixes applied here
------------------
  Fix-1  TfPoller       background daemon thread polls TF every 50 ms;
                        hot-loop reads the cache instantly — no 0–300 ms stall.
  Fix-3  AsyncCsvWriter background queue-drain thread; main loop enqueues rows
                        non-blocking — no disk-I/O jitter.

Architecture
------------
  Main thread  ─── IK hot-loop (20–50 Hz)
  Thread: rclpy spin  ─── ROS callbacks (_js_cb, _ee_delta_cb)
  Thread: kbd_controller  ─── raw-mode stdin daemon
  Thread: TfPoller  ─── TF2 lookup every 50 ms            [Fix-1]
  Thread: AsyncCsvWriter  ─── queue-drain CSV/flush       [Fix-3]
"""

import os as _os, sys as _sys
_HERE = _os.path.dirname(_os.path.abspath(__file__))           # ik_node/
_ROOT = _os.path.dirname(_HERE)                                # project root
_sys.path.insert(0, _ROOT)                                     # paths.py
_sys.path.insert(0, _HERE)                                     # siblings: session, kbd
_sys.path.insert(0, _os.path.join(_ROOT, "ik_solver"))         # placo_ik_solver
_sys.path.insert(0, _os.path.join(_ROOT, "ws_mesh"))           # placo_ws_analyze

import csv
import datetime
import json
import math
import os
import queue
import threading
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener, TransformException

from placo_ik_solver import _find_urdf, _quat_to_rot
from placo_ws_analyze import WorkspaceMesh
from placo_ik_session import PlacoSession, _MAX_ITER
from kbd_controller import KbdController
from ws_boundary import SoftClamp, BoundaryMonitor
from paths import csv_path as _csv_path, png_for as _png_for


# ── ARM config ────────────────────────────────────────────────────────────────
ARM_CONFIG = {
    "left": {
        "joint_names":   [f"openarm_left_joint{i}"  for i in range(1, 8)],
        "base_link":     "world",
        "ee_link":       "openarm_left_link7",
        "home_joints":   [0.0, 0.0, 0.0, 1.5708, 0.0, 0.0, 0.0],
        "home_pose":     (0.2160, 0.1535, 0.4780, 0.7071, -0.0000, 0.7071, -0.0000),
        "workspace":     {"x": (-0.20, 0.62), "y": (0.05, 0.65), "z": (0.15, 0.8)},
        "traj_topic":    "/left_joint_trajectory_controller/joint_trajectory",
        "fwd_cmd_topic": "/left_forward_position_controller/commands",
        "latency_topic": "/left/delta_ik_latency_ms",
        "profile_topic": "/left/placo_profile",
        "ee_delta_topic": "/ee_delta/left",
    },
    "right": {
        "joint_names":   [f"openarm_right_joint{i}" for i in range(1, 8)],
        "base_link":     "world",
        "ee_link":       "openarm_right_link7",
        "home_joints":   [0.0, 0.0, 0.0, 1.5708, 0.0, 0.0, 0.0],
        "home_pose":     (0.216000, -0.153500, 0.478001,0.7071, 0.0000, 0.7071, 0.0000),
        "workspace":     {"x": (-0.20, 0.62), "y": (-0.65, -0.05), "z": (0.15, 0.8)},
        "traj_topic":    "/right_joint_trajectory_controller/joint_trajectory",
        "fwd_cmd_topic": "/right_forward_position_controller/commands",
        "latency_topic": "/right/delta_ik_latency_ms",
        "profile_topic": "/right/placo_profile",
        "ee_delta_topic": "/ee_delta/right",
    },
}

CSV_FIELDS = [
    "t", "x", "y", "z",
    "success", "ik_ms", "total_ms",
    "dx", "dy", "dz",
    "robot_ms", "setup_ms", "loop_ms",
    "iterations", "iter_ms",
    "pos_err_mm", "ori_err_deg", "track_err_mm",
    "max_joint_delta_deg", "joint_jump_guard",
    "mem_kb", "deadline_missed",
    "sigma_min", "lambda_dls",          # Adaptive DLS (方案 C)
]


# ── Quaternion helpers ────────────────────────────────────────────────────────
def _qmul(q1, q2):
    x1, y1, z1, w1 = q1; x2, y2, z2, w2 = q2
    return (w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2,
            w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2)

def _qnorm(q):
    x, y, z, w = q; n = math.sqrt(x*x+y*y+z*z+w*w)
    return (x/n, y/n, z/n, w/n) if n > 1e-9 else (0., 0., 0., 1.)

def _qfrom_rpy(r, p, y):
    cr, cp, cy = math.cos(r/2), math.cos(p/2), math.cos(y/2)
    sr, sp, sy = math.sin(r/2), math.sin(p/2), math.sin(y/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)

def _rotate_vec(v, q):
    qx, qy, qz, qw = q; vx, vy, vz = v
    tx = 2*(qy*vz-qz*vy); ty = 2*(qz*vx-qx*vz); tz = 2*(qx*vy-qy*vx)
    return (vx+qw*tx+qy*tz-qz*ty, vy+qw*ty+qz*tx-qx*tz, vz+qw*tz+qx*ty-qy*tx)

def _qconj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)

def _qdot(q1, q2):
    return sum(a * b for a, b in zip(q1, q2))

def _qslerp(q0, q1, alpha):
    """SLERP from q0 to q1. alpha=1.0 returns q1."""
    q0 = _qnorm(q0)
    q1 = _qnorm(q1)
    alpha = max(0.0, min(1.0, float(alpha)))
    dot = _qdot(q0, q1)
    if dot < 0.0:
        q1 = tuple(-v for v in q1)
        dot = -dot
    if dot > 0.9995:
        return _qnorm(tuple((1.0 - alpha) * a + alpha * b for a, b in zip(q0, q1)))
    theta_0 = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * alpha
    s0 = math.cos(theta) - dot * math.sin(theta) / sin_theta_0
    s1 = math.sin(theta) / sin_theta_0
    return _qnorm(tuple(s0 * a + s1 * b for a, b in zip(q0, q1)))

def _rotate_quat(q, frame_q):
    """Similarity transform: rotate a delta quaternion through frame_q.
    dq_arm = frame_q * dq * conj(frame_q)"""
    return _qnorm(_qmul(_qmul(frame_q, _qnorm(q)), _qconj(frame_q)))


def _parse_lpf_alpha(spec: str, n_joints: int) -> List[float]:
    """Parse --lpf-alpha as single float (uniform) or comma list of len n_joints."""
    parts = [p.strip() for p in str(spec).split(",")]
    if len(parts) == 1:
        return [float(parts[0])] * n_joints
    if len(parts) != n_joints:
        raise ValueError(
            f"--lpf-alpha needs 1 or {n_joints} values, got {len(parts)}: {spec!r}"
        )
    return [float(p) for p in parts]


# ── Fix-1: TF Poller ──────────────────────────────────────────────────────────
class TfPoller:
    """
    Daemon thread that polls TF2 every poll_sec and caches the result.

    The IK hot-loop calls get() which returns instantly from the cache,
    eliminating the 0–300 ms blocking _get_tf() call that caused jitter.
    """

    def __init__(
        self,
        tf_buffer,
        base_link: str,
        ee_link:   str,
        poll_sec:  float = 0.05,
    ):
        self._buf      = tf_buffer
        self._base     = base_link
        self._ee       = ee_link
        self._poll_sec = poll_sec
        self._pose: Optional[Tuple] = None
        self._lock     = threading.Lock()
        self._thread   = threading.Thread(
            target=self._run, daemon=True, name="tf_poller")

    def start(self) -> None:
        self._thread.start()

    def get(self) -> Optional[Tuple]:
        """Instant non-blocking cache read."""
        with self._lock:
            return self._pose

    def _run(self) -> None:
        timeout = rclpy.duration.Duration(seconds=self._poll_sec)
        while True:
            try:
                t  = self._buf.lookup_transform(
                    self._base, self._ee,
                    rclpy.time.Time(), timeout=timeout)
                tr, ro = t.transform.translation, t.transform.rotation
                with self._lock:
                    self._pose = (tr.x, tr.y, tr.z, ro.x, ro.y, ro.z, ro.w)
            except Exception:
                pass
            time.sleep(self._poll_sec * 0.5)   # 2× faster than timeout


# ── Fix-3: Async CSV Writer ───────────────────────────────────────────────────
class AsyncCsvWriter:
    """
    Daemon thread that drains a row queue to disk.

    The IK hot-loop calls put() which is non-blocking — no disk-I/O stalls.
    Flushes automatically whenever the queue drains to empty.
    """

    def __init__(self, filepath: str, fields: List[str]):
        self._fields = fields
        self._path   = filepath
        self._fh     = open(filepath, "w", newline="")
        self._writer = csv.writer(self._fh)
        self._writer.writerow(fields)
        self._q      = queue.Queue(maxsize=2000)
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="csv_writer")

    def start(self) -> None:
        self._thread.start()

    def put(self, row_dict: Dict) -> None:
        """Non-blocking enqueue; silently drops if queue is full."""
        try:
            self._q.put_nowait([row_dict.get(f, "") for f in self._fields])
        except queue.Full:
            pass

    def close(self) -> None:
        """Drain remaining rows then close the file."""
        if self._thread.is_alive():
            self._q.put(None)          # sentinel
            self._thread.join(timeout=3.0)
        self._fh.flush()
        self._fh.close()

    def _run(self) -> None:
        while True:
            row = self._q.get()
            if row is None:
                break
            self._writer.writerow(row)
            if self._q.empty():
                self._fh.flush()       # flush only when queue drains


# ── ROS2 Node ─────────────────────────────────────────────────────────────────
class PlacoOnlineProfiler(Node):
    """
    Real-time IK controller + profiler.

    Input  : /ee_delta/{arm} (PoseStamped)  OR  keyboard XYZ/RPY
    Output : JointTrajectory + latency + profile JSON topics + CSV
    """

    def __init__(self, args):
        super().__init__("placo_ik_online_profiler_ws_mesh")
        self.args = args
        cfg = ARM_CONFIG[args.arm]
        self.cfg = cfg

        self._cbg = ReentrantCallbackGroup()

        # EE pose state [x, y, z, qx, qy, qz, qw]
        hx, hy, hz, hqx, hqy, hqz, hqw = cfg["home_pose"]
        self._pose        = [hx, hy, hz, hqx, hqy, hqz, hqw]
        self._last_joints = list(cfg["home_joints"])
        self._joints_lock = threading.Lock()
        self._joint_states: dict = {}
        self._js_lock     = threading.Lock()

        # ee_delta session reference
        self._ee_delta_ref_xyz = None
        self._ee_delta_ref_q   = None
        self._ee_delta_last_t  = None
        self._ee_delta_gap_sec = float(getattr(args, "ee_delta_gap_sec", 0.8))

        # Calibration quaternion (tracker → arm frame)
        crpy = getattr(args, "calib_rpy", None)
        if crpy:
            rr, rp, ry = [math.radians(float(v)) for v in crpy.split(",")]
        else:
            rr, rp = 0.0, 0.0
            ry = math.radians(getattr(args, "calib_yaw", 0.0))
        self._calib_q = _qfrom_rpy(rr, rp, ry)
        self._no_rot_tracking = bool(getattr(args, "no_rot_tracking", False))

        # Input-side orientation smoothing (Fix-1: SLERP EMA).
        # alpha=1.0 disables smoothing.
        self._ori_lpf_alpha = max(0.0, min(1.0, float(
            getattr(args, "ori_lpf_alpha", 0.35))))
        self._ori_lpf_active = self._ori_lpf_alpha < 0.999
        self._ori_filt_q: Optional[Tuple[float, float, float, float]] = None

        # Hard guard against IK branch jumps (Fix-5).
        self._joint_jump_guard_deg = float(
            getattr(args, "joint_jump_guard_deg", 15.0))

        # Rate / timing
        self._rate_hz     = float(getattr(args, "rate", 20.0))
        self._horizon_ms  = float(getattr(args, "horizon", 60.0))
        self._deadline_ms = 1000.0 / self._rate_hz

        # Workspace clamp
        self._ws_clamp = not getattr(args, "no_ws_clamp", False)
        self._ws_mesh: Optional[WorkspaceMesh] = None
        npz_path = getattr(args, "ws_mesh", None)
        if npz_path:
            if not os.path.isfile(npz_path):
                raise FileNotFoundError(f"--ws-mesh not found: {npz_path}")
            self._ws_mesh  = WorkspaceMesh.load(npz_path)
            self._ws_clamp = True

        # Soft boundary (方案 A + E)
        _soft_margin = float(getattr(args, "boundary_margin", 0.05))
        self._soft_clamp    = SoftClamp(
            ws_mesh  = self._ws_mesh,
            margin_m = _soft_margin,
            box_ws   = self.cfg["workspace"] if not self._ws_mesh else None,
        )
        self._bdry_monitor  = None   # created after super().__init__ publishes

        # IK session — Fix-2: rate_hz → dt=1/rate_hz; vel_limits=True
        urdf = _find_urdf()
        self._placo_session = PlacoSession(
            urdf          = urdf,
            arm           = args.arm,
            rebuild       = args.rebuild,
            max_iter      = getattr(args, "max_iter", _MAX_ITER),
            rate_hz       = self._rate_hz,
            vel_limits    = not getattr(args, "no_vel_limits", False),
            wrist_vel_cap = float(getattr(args, "wrist_vel_cap", 4.0)),
        )

        # Output-side LPF on joint commands  (Fix-D / Fix-3)
        n_joints = len(cfg["joint_names"])
        self._lpf_alpha: List[float] = _parse_lpf_alpha(
            getattr(args, "lpf_alpha", "1.0"), n_joints
        )
        self._lpf_active   = any(a < 0.999 for a in self._lpf_alpha)
        self._filt_joints: Optional[List[float]] = None

        # Set2-D: success-gate behaviour
        # False (default) → always publish (continuous approach, vel-limit saturates)
        # True            → freeze on failure (legacy "stop on OOR" behaviour)
        self._success_gate    = bool(getattr(args, "success_gate", False))
        self._fail_streak     = 0
        self._fail_warn_every = 20    # print warning every N consecutive fails


        # TF2 + Fix-1 poller (started in run() after initial sync)
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_poller   = TfPoller(
            self._tf_buffer, cfg["base_link"], cfg["ee_link"], poll_sec=0.05)

        # Publishers
        # JointTrajectory publisher — used ONLY by send_home_confirmed()
        self._traj_pub = self.create_publisher(
            JointTrajectory, cfg["traj_topic"], 10)
        # ForwardCommandController publisher — used for hot-loop streaming
        self._use_traj = getattr(args, 'use_traj', False)
        if self._use_traj:
            # Legacy mode: use JointTrajectoryController for streaming
            self._fwd_pub = None
        else:
            self._fwd_pub = self.create_publisher(
                Float64MultiArray, cfg["fwd_cmd_topic"], 10)
        self._latency_pub = self.create_publisher(Float32, cfg["latency_topic"], 10)
        self._profile_pub = self.create_publisher(String, cfg["profile_topic"], 10)

        # Subscribers
        self.create_subscription(
            JointState, "/joint_states",
            self._js_cb, 10, callback_group=self._cbg)
        self.create_subscription(
            PoseStamped, cfg["ee_delta_topic"],
            self._ee_delta_cb, 10, callback_group=self._cbg)

        # Message queue
        self._pending    = None
        self._delta_lock = threading.Lock()
        self._msg_count  = 0
        self._records:   List[Dict] = []

        # Keyboard controller
        self._kbd = KbdController(
            init_mode="keyboard" if getattr(args, "keyboard", False) else "tracker")

        # Fix-3: async CSV writer
        mode = "cached" if not args.rebuild else "rebuild"
        csv_p = args.csv or _csv_path("placo_online", args.arm, mode)
        self._csv_writer = AsyncCsvWriter(csv_p, CSV_FIELDS)
        self._csv_path   = csv_p

        # BoundaryMonitor needs publisher → create after node is fully initialised
        self._bdry_monitor = BoundaryMonitor(self, args.arm)

        self._print_banner()

    # ── ROS callbacks ─────────────────────────────────────────────────────────
    def _js_cb(self, msg: JointState):
        with self._js_lock:
            for n, p in zip(msg.name, msg.position):
                self._joint_states[n] = p

    def _ee_delta_cb(self, msg: PoseStamped):
        with self._delta_lock:
            self._pending = msg
        self._msg_count += 1

    # ── TF helpers ────────────────────────────────────────────────────────────
    def _get_tf(self, timeout_sec: float = 0.3) -> Optional[Tuple]:
        """Blocking TF lookup — only for startup init and send_home_confirmed."""
        try:
            t = self._tf_buffer.lookup_transform(
                self.cfg["base_link"], self.cfg["ee_link"],
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec),
            )
            tr, ro = t.transform.translation, t.transform.rotation
            return (tr.x, tr.y, tr.z, ro.x, ro.y, ro.z, ro.w)
        except TransformException:
            return None

    def _reset_orientation_filter(self) -> None:
        self._ori_filt_q = None

    def _filter_orientation(self, q_target) -> Tuple[float, float, float, float]:
        """SLERP EMA on target orientation, after frame calibration."""
        q_target = _qnorm(q_target)
        if not self._ori_lpf_active or self._ori_filt_q is None:
            self._ori_filt_q = q_target
            return q_target
        self._ori_filt_q = _qslerp(self._ori_filt_q, q_target, self._ori_lpf_alpha)
        return self._ori_filt_q

    def _joint_jump_guard(self, prev: List[float], new: List[float]) -> Tuple[bool, float]:
        """Return (triggered, max_delta_deg).  Rejects IK output with any joint > limit."""
        max_delta = max(
            math.degrees(abs(n - p)) for p, n in zip(prev, new)
        ) if prev and new else 0.0
        limit = self._joint_jump_guard_deg
        return (limit > 0.0 and max_delta > limit), max_delta

    # ── Send home ─────────────────────────────────────────────────────────────
    def send_home_confirmed(
        self,
        pos_tol:    float = 0.025,
        motion_sec: float = 3.5,
        max_tries:  int   = 5,
        poll_sec:   float = 0.3,
    ) -> bool:
        """Send home trajectory and wait for TF confirmation."""
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
            print(f"  [home] attempt {attempt}/{max_tries} — waiting {motion_sec:.1f}s...")

            deadline = time.time() + motion_sec
            arrived  = False
            dist     = 999.0
            while time.time() < deadline:
                time.sleep(poll_sec)
                tf = self._get_tf(0.3)
                if tf is None:
                    continue
                dist = math.sqrt(sum((tf[i] - home_xyz[i])**2 for i in range(3)))
                print(f"  [home]   dist={dist*100:.1f}cm", end="\r", flush=True)
                if dist <= pos_tol:
                    arrived = True
                    break
            print()

            if arrived:
                print(f"  [home] ✓ reached home  dist={dist*100:.1f}cm")
                tf = self._get_tf(1.0)
                if tf is not None:
                    self._pose = list(tf)
                with self._js_lock:
                    js = dict(self._joint_states)
                if all(n in js for n in jnames):
                    with self._joints_lock:
                        self._last_joints = [js[n] for n in jnames]
                self._filt_joints = None    # re-anchor LPF on next IK step
                self._reset_orientation_filter()
                return True
            else:
                with self._js_lock:
                    js = dict(self._joint_states)
                j_err = max(
                    abs(js.get(n, 0) - self.cfg["home_joints"][i])
                    for i, n in enumerate(jnames)
                ) if js else 99.0
                print(f"  [home] ✗ attempt {attempt} timed out  "
                      f"j_err={math.degrees(j_err):.1f}°")

        print("  [home] ✗ could not confirm — using home_pose as reference")
        hx, hy, hz, hqx, hqy, hqz, hqw = self.cfg["home_pose"]
        self._pose = [hx, hy, hz, hqx, hqy, hqz, hqw]
        with self._joints_lock:
            self._last_joints = list(self.cfg["home_joints"])
        self._filt_joints = list(self._last_joints)
        self._reset_orientation_filter()
        return False

    def send_home_fwd(
        self,
        pos_tol:    float = 0.025,
        motion_sec: float = 3.5,
        ramp_hz:    float = 50.0,
    ) -> bool:
        """
        Ramp arm to home via ForwardCommandController (linear interpolation).

        Unlike send_home_confirmed() which sends a single JointTrajectory,
        this method publishes intermediate joint positions at `ramp_hz` via
        Float64MultiArray, suitable for when JointTrajectoryController is not
        active.

        Used automatically when --home-first is specified in ForwardCmd mode.
        """
        if self._fwd_pub is None:
            # Fallback: use trajectory controller
            return self.send_home_confirmed(pos_tol=pos_tol, motion_sec=motion_sec)

        jnames = self.cfg["joint_names"]
        home   = self.cfg["home_joints"]
        home_xyz = self.cfg["home_pose"][:3]

        # Read current joint positions
        with self._js_lock:
            js = dict(self._joint_states)
        if not all(n in js for n in jnames):
            print("  [home_fwd] ⚠ joint states not available — waiting 2s...")
            time.sleep(2.0)
            with self._js_lock:
                js = dict(self._joint_states)
            if not all(n in js for n in jnames):
                print("  [home_fwd] ✗ still no joint states — aborting")
                return False

        start = [js[n] for n in jnames]

        # Check if already near home
        max_delta_deg = max(math.degrees(abs(s - h)) for s, h in zip(start, home))
        if max_delta_deg < 1.0:
            print(f"  [home_fwd] already near home (max_Δ={max_delta_deg:.1f}°)")
            return True

        # Compute ramp duration based on max joint delta and speed limit
        # Conservative: max 30°/s to avoid sudden movements
        max_speed_dps = 30.0   # degrees per second
        ramp_sec = max(1.0, max_delta_deg / max_speed_dps)
        ramp_sec = min(ramp_sec, motion_sec)

        n_steps = int(ramp_sec * ramp_hz)
        dt = 1.0 / ramp_hz

        print(f"  [home_fwd] ramping {max_delta_deg:.1f}° over "
              f"{ramp_sec:.1f}s ({n_steps} steps at {ramp_hz}Hz)...")

        for step in range(n_steps + 1):
            alpha = step / max(1, n_steps)  # 0.0 → 1.0
            # Smooth ease-in-out (cosine interpolation)
            alpha_smooth = 0.5 * (1.0 - math.cos(alpha * math.pi))
            interp = [s + alpha_smooth * (h - s) for s, h in zip(start, home)]

            msg = Float64MultiArray()
            msg.data = interp
            self._fwd_pub.publish(msg)

            if step % max(1, n_steps // 10) == 0:
                pct = alpha * 100
                print(f"  [home_fwd]   {pct:.0f}%", end="\r", flush=True)

            time.sleep(dt)

        print()

        # Verify arrival via TF
        time.sleep(0.5)
        tf = self._get_tf(1.0)
        if tf is not None:
            dist = math.sqrt(sum((tf[i] - home_xyz[i])**2 for i in range(3)))
            if dist <= pos_tol:
                print(f"  [home_fwd] ✓ reached home  dist={dist*100:.1f}cm")
                self._pose = list(tf)
                with self._js_lock:
                    js = dict(self._joint_states)
                if all(n in js for n in jnames):
                    with self._joints_lock:
                        self._last_joints = [js[n] for n in jnames]
                self._filt_joints = None
                self._reset_orientation_filter()
                return True
            else:
                print(f"  [home_fwd] ⚠ dist={dist*100:.1f}cm — close but "
                      f"may need adjustment")
        else:
            print("  [home_fwd] ⚠ could not verify via TF")

        # Even if TF check isn't perfect, set internal state to home
        with self._joints_lock:
            self._last_joints = list(home)
        self._filt_joints = None
        self._reset_orientation_filter()
        hx, hy, hz, hqx, hqy, hqz, hqw = self.cfg["home_pose"]
        self._pose = [hx, hy, hz, hqx, hqy, hqz, hqw]
        return True

    # ── Publish to hardware ────────────────────────────────────────────────────
    def _publish(self, joints: List[float]):
        if self._fwd_pub is not None:
            # Solution A: stream directly via ForwardCommandController
            msg = Float64MultiArray()
            msg.data = joints
            self._fwd_pub.publish(msg)
        else:
            # Legacy: JointTrajectoryController
            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names  = list(self.cfg["joint_names"])
            pt = JointTrajectoryPoint()
            pt.positions     = list(joints)
            hs = max(0.001, self._horizon_ms / 1000.0)
            pt.time_from_start.sec = int(hs)
            pt.time_from_start.nanosec = int((hs % 1.0) * 1e9)
            msg.points = [pt]
            self._traj_pub.publish(msg)

    # ── Output LPF on joint command (Fix-D) ───────────────────────────────────
    def _filter_joints(self, raw: List[float]) -> List[float]:
        """
        1st-order per-joint LPF: filt = α·raw + (1-α)·prev.
        α=1.0 → passthrough.  Re-anchors to raw whenever state is None
        (initial step, after home).
        """
        if not self._lpf_active or self._filt_joints is None:
            self._filt_joints = list(raw)
            return list(self._filt_joints)
        self._filt_joints = [
            a * r + (1.0 - a) * p
            for a, r, p in zip(self._lpf_alpha, raw, self._filt_joints)
        ]
        return list(self._filt_joints)

    # ── Main control loop ─────────────────────────────────────────────────────
    def run(self):
        """50/20 Hz IK loop. Starts TfPoller + AsyncCsvWriter background threads."""
        dt_sec = 1.0 / self._rate_hz
        self._kbd.start()

        # ── Startup sync (blocking is fine here — outside hot loop) ──────────
        print("\n  Syncing from TF2 and /joint_states...")
        time.sleep(0.5)
        tf = self._get_tf(2.0)
        if tf:
            self._pose = list(tf)
            print(f"  TF EE: {[f'{v:.4f}' for v in tf[:3]]}")
        else:
            print("  ⚠ TF unavailable — using home_pose")
        with self._js_lock:
            js = dict(self._joint_states)
        jnames = self.cfg["joint_names"]
        if all(n in js for n in jnames):
            with self._joints_lock:
                self._last_joints = [js[n] for n in jnames]
            print(f"  joints synced: {[f'{v:.3f}' for v in self._last_joints]}")
        else:
            print("  ⚠ /joint_states not ready — using home_joints")

        # ── Start background services ─────────────────────────────────────────
        self._tf_poller.start()     # Fix-1: background TF cache
        self._csv_writer.start()    # Fix-3: background CSV drain

        print(f"\n  Waiting for {self.cfg['ee_delta_topic']} ...\n")
        _no_msg_t  = time.time()
        step_count = 0
        ws         = self.cfg["workspace"]

        while rclpy.ok():
            t_step = time.perf_counter()
            kbd    = self._kbd

            # ── Keyboard events (flags set by daemon thread) ──────────────
            if kbd.request_quit:
                print("\n  [kbd] quit — exiting")
                break
            if kbd.request_home:
                kbd.request_home = False
                print("\n  [kbd] sending home...")
                self.send_home_confirmed(pos_tol=0.025, motion_sec=3.5, max_tries=3)
            if kbd.reset_ref:
                kbd.reset_ref          = False
                self._ee_delta_ref_xyz = None
                self._ee_delta_ref_q   = None
                self._ee_delta_last_t  = None
                self._reset_orientation_filter()
                print("\n  [kbd] session reference reset")

            # ── Input routing: tracker topic vs keyboard ───────────────────
            with self._delta_lock:
                tracker_msg   = self._pending
                self._pending = None

            _kbd_mode = kbd.mode
            if _kbd_mode == "keyboard":
                tracker_msg = None
                xyz_d = kbd.pop_xyz()
                rpy_d = kbd.pop_rpy()
                msg   = (xyz_d, rpy_d) if (xyz_d is not None or rpy_d is not None) else None
            else:
                xyz_d = rpy_d = None
                msg   = tracker_msg

            # ── Idle ──────────────────────────────────────────────────────
            if msg is None:
                idle_s = time.time() - _no_msg_t
                if idle_s >= 2.0:
                    _no_msg_t = time.time()
                    paused_str = "  [PAUSED]" if kbd.paused else ""
                    print(
                        f"\r  [WAIT] {_kbd_mode}{paused_str}  cb={self._msg_count}"
                        f"  steps={step_count}  idle={idle_s:.0f}s  ×{kbd.scale:.2f}"
                        f"  [{self._pose[0]:.3f} {self._pose[1]:.3f} {self._pose[2]:.3f}]",
                        end="", flush=True,
                    )

            elif isinstance(msg, (PoseStamped, tuple)) and not kbd.paused:
                _no_msg_t = time.time()
                t_wall    = time.time()

                # ── Build dx/dy/dz + dq ───────────────────────────────────
                if _kbd_mode == "keyboard":
                    xyz_d, rpy_d = msg
                    dx, dy, dz = xyz_d if xyz_d is not None else (0., 0., 0.)
                    dq         = _qfrom_rpy(*rpy_d) if rpy_d is not None else (0., 0., 0., 1.)
                    base_xyz   = tuple(self._pose[:3])
                    base_q     = tuple(self._pose[3:7])
                    dx_arm     = (dx, dy, dz)   # already in robot base frame
                else:
                    _s  = kbd.scale
                    dx  = msg.pose.position.x * _s
                    dy  = msg.pose.position.y * _s
                    dz  = msg.pose.position.z * _s
                    dq  = (msg.pose.orientation.x, msg.pose.orientation.y,
                           msg.pose.orientation.z, msg.pose.orientation.w)
                    if abs(dq[3]) < 0.01 and all(abs(v) < 0.01 for v in dq[:3]):
                        dq = (0., 0., 0., 1.)

                    gap    = (t_wall - self._ee_delta_last_t) if self._ee_delta_last_t else 999.
                    is_new = self._ee_delta_ref_xyz is None or gap > self._ee_delta_gap_sec
                    self._ee_delta_last_t = t_wall

                    if is_new:
                        # Fix-1: instant cache read — was self._get_tf(0.3) = 0–300 ms block
                        ref = self._tf_poller.get()
                        if ref is not None:
                            self._ee_delta_ref_xyz = tuple(ref[:3])
                            self._ee_delta_ref_q   = tuple(ref[3:7])
                            self._pose             = list(ref)
                            print(f"\n  [EE-δ] NEW ref={[f'{v:.4f}' for v in ref[:3]]}  (TF)")
                        else:
                            self._ee_delta_ref_xyz = tuple(self._pose[:3])
                            self._ee_delta_ref_q   = tuple(self._pose[3:7])
                        self._reset_orientation_filter()

                    base_xyz = self._ee_delta_ref_xyz or tuple(self._pose[:3])
                    base_q   = self._ee_delta_ref_q   or tuple(self._pose[3:7])
                    dx_arm   = _rotate_vec((dx, dy, dz), self._calib_q)
                    # Fix-8: apply calibration to orientation delta (was missing)
                    dq       = _rotate_quat(dq, self._calib_q)

                # ── Workspace clamp (方案 A: SoftClamp 取代硬 snap) ─────────
                raw_xyz_arr = np.array([base_xyz[0] + dx_arm[0],
                                        base_xyz[1] + dx_arm[1],
                                        base_xyz[2] + dx_arm[2]])
                dx_arm_arr  = np.array(dx_arm)
                if self._ws_clamp or self._ws_mesh is not None:
                    new_xyz_arr, _bs = self._soft_clamp.apply(raw_xyz_arr, dx_arm_arr)
                    new_x, new_y, new_z = float(new_xyz_arr[0]), float(new_xyz_arr[1]), float(new_xyz_arr[2])
                    self._bdry_monitor.publish(_bs)   # 方案 E: topic + console
                else:
                    new_x, new_y, new_z = float(raw_xyz_arr[0]), float(raw_xyz_arr[1]), float(raw_xyz_arr[2])

                # Fix-1: compute target orientation, then SLERP-filter
                new_q_raw  = _qnorm(_qmul(dq, base_q))
                new_q      = base_q if self._no_rot_tracking else self._filter_orientation(new_q_raw)
                target_xyz = np.array([new_x, new_y, new_z])
                target_R   = _quat_to_rot(*new_q)

                # Fix-6: seed from filtered joints (what the robot is actually tracking)
                with self._joints_lock:
                    seed = list(self._filt_joints or self._last_joints)

                # ── IK solve ──────────────────────────────────────────────
                t_total = time.perf_counter()
                r = self._placo_session.solve_step(
                    target_xyz, target_R, seed, no_rot=self._no_rot_tracking)
                ik_ms    = r["solve_ms"]
                total_ms = (time.perf_counter() - t_total) * 1000.0

                track_err       = float(np.linalg.norm(
                    np.array(r["ee_xyz"]) - target_xyz)) * 1000.0
                loop_wall_ms    = (time.perf_counter() - t_step) * 1000.0
                deadline_missed = int(loop_wall_ms > self._deadline_ms)
                # Fix-5: reject IK solutions with large inter-step joint jumps
                guard_hit, max_joint_delta_deg = self._joint_jump_guard(
                    seed, r["joints"])

                # ── Anti-jump: two complementary methods ──────────────────────
                # Method 1 (proactive) : wrist_vel_cap in PlacoSession — QP hard
                #                        constraint on j5-7; caps single-step Δq
                #                        BEFORE the IK returns.
                # Method 2 (reactive)  : joint_jump_guard above — detects post-IK
                #                        any joint Δ > N°/step; rejects publish.
                # Method 1 should catch most cases; Method 2 is the safety net
                # for redundancy flips / large target leaps method 1 missed.
                #
                # ── Set2-D: continuous approach (no success-freeze) ───────────
                # When guard does NOT trip: always publish (even on IK fail),
                # trusting velocity_limits to saturate the partial solution and
                # _pose drift-protection (track r["ee_xyz"]) to prevent base
                # reference drift.  --success-gate restores legacy freeze.
                #
                # When guard trips: reject this whole step — don't update seed,
                # don't update filter state, don't update _pose, don't publish.
                if guard_hit:
                    r["success"] = 0   # mark fail for CSV accounting
                    self._fail_streak += 1
                    if self._fail_streak % self._fail_warn_every == 0:
                        print(f"\n  [guard] joint Δ={max_joint_delta_deg:.1f}° "
                              f"rejected, streak={self._fail_streak}")
                else:
                    # Filter once (always — keeps LPF state continuous)
                    joints_out = self._filter_joints(r["joints"])
                    # Fix-6 (test3): seed next IK from filtered output, not raw
                    with self._joints_lock:
                        self._last_joints = list(joints_out)

                    if r["success"]:
                        self._pose = [new_x, new_y, new_z,
                                      new_q[0], new_q[1], new_q[2], new_q[3]]
                        self._fail_streak = 0
                    else:
                        # Set2-D: _pose tracks solver's actual EE (drift防護)
                        ee = r["ee_xyz"]
                        self._pose[0] = float(ee[0])
                        self._pose[1] = float(ee[1])
                        self._pose[2] = float(ee[2])
                        # orientation held at previous (no r["ee_R"])
                        self._fail_streak += 1
                        if self._fail_streak % self._fail_warn_every == 0:
                            print(f"\n  [Set2-D] IK pos_err={r['pos_err_mm']:.1f}mm "
                                  f"streak={self._fail_streak} — publishing partial solve")

                    publish_ok = r["success"] or not self._success_gate
                    if publish_ok and not self.args.dry_run:
                        self._publish(joints_out)

                self._latency_pub.publish(Float32(data=float(ik_ms)))

                step_count += 1

                # Publish profile JSON
                self._profile_pub.publish(String(data=json.dumps({
                    "step":       step_count,
                    "ik_ms":      round(ik_ms, 3),
                    "robot_ms":   round(r["robot_ms"], 3),
                    "setup_ms":   round(r["setup_ms"], 3),
                    "loop_ms":    round(r["loop_ms"], 3),
                    "iterations": r["iterations"],
                    "iter_ms":    round(r["iter_ms"], 4),
                    "pos_err_mm": round(r["pos_err_mm"], 3),
                    "ori_err_deg": round(r.get("ori_err_deg", 0.0), 3),
                    "track_err_mm": round(track_err, 3),
                    "max_joint_delta_deg": round(max_joint_delta_deg, 3),
                    "joint_jump_guard": int(guard_hit),
                    "success":    r["success"],
                    "deadline_missed": deadline_missed,
                    "mem_mb":     round(r["mem_kb"] / 1024.0, 1),
                    "mode":       "rebuild" if self.args.rebuild else "cached",
                    "sigma_min":  round(r.get("sigma_min", 0.0), 6),
                    "lambda_dls": round(r.get("lambda_dls", 0.0), 8),
                })))

                # Fix-3: non-blocking enqueue — no disk I/O on hot path
                row = {
                    "t": round(t_wall, 6),
                    "x": round(self._pose[0], 6), "y": round(self._pose[1], 6),
                    "z": round(self._pose[2], 6),
                    "success":  r["success"],
                    "ik_ms":    round(ik_ms, 4),  "total_ms": round(total_ms, 4),
                    "dx": round(dx, 6), "dy": round(dy, 6), "dz": round(dz, 6),
                    "robot_ms":   round(r["robot_ms"], 4),
                    "setup_ms":   round(r["setup_ms"], 4),
                    "loop_ms":    round(r["loop_ms"], 4),
                    "iterations": r["iterations"],
                    "iter_ms":    round(r["iter_ms"], 5),
                    "pos_err_mm": round(r["pos_err_mm"], 4),
                    "ori_err_deg": round(r.get("ori_err_deg", 0.0), 4),
                    "track_err_mm": round(track_err, 4),
                    "max_joint_delta_deg": round(max_joint_delta_deg, 4),
                    "joint_jump_guard": int(guard_hit),
                    "mem_kb":     r["mem_kb"],
                    "deadline_missed": deadline_missed,
                    "sigma_min":  round(r.get("sigma_min", 0.0), 6),
                    "lambda_dls": round(r.get("lambda_dls", 0.0), 8),
                }
                self._csv_writer.put(row)
                self._records.append(row)

                # Console print (every 5 steps or verbose)
                if step_count % 5 == 0 or kbd.verbose or self.args.verbose:
                    tag = "✓" if r["success"] else "✗"
                    dlm = "!" if deadline_missed else " "
                    print(
                        f"\r  {step_count:5d}"
                        f"  {ik_ms:7.2f}  {r['robot_ms']:6.2f}  {r['loop_ms']:6.2f}"
                        f"  {r['iterations']:5d}  {r['pos_err_mm']:6.2f}"
                        f"  {track_err:6.2f}  {r['mem_kb']/1024:6.1f}"
                        f"  ×{kbd.scale:.2f}  {dlm}{tag}",
                        end="", flush=True,
                    )

                if step_count % 50 == 0:
                    self._print_partial_stats()

            sleep_sec = dt_sec - (time.perf_counter() - t_step)
            if sleep_sec > 0:
                time.sleep(sleep_sec)

    # ── Stats ─────────────────────────────────────────────────────────────────
    def _print_partial_stats(self):
        rows = self._records
        if not rows:
            return
        n     = len(rows)
        ik    = [r["ik_ms"] for r in rows]
        loop  = [r["loop_ms"] for r in rows]
        iters = [r["iterations"] for r in rows]
        ok    = sum(r["success"] for r in rows)
        miss  = sum(r["deadline_missed"] for r in rows)
        print(
            f"\n  ─── [{n}] sr={ok/n*100:.0f}%  "
            f"ik_med={float(np.median(ik)):.2f}ms  "
            f"loop_med={float(np.median(loop)):.2f}ms  "
            f"iters_med={float(np.median(iters)):.1f}  "
            f"ddl={miss}/{n}={miss/n*100:.0f}%"
        )

    def print_final_stats(self):
        rows = self._records
        if not rows:
            return
        print(f"\n\n{'═'*65}")
        print(f"  Final stats  arm={self.args.arm}  n={len(rows)}")
        print(f"{'═'*65}")
        for key, unit in [
            ("ik_ms",        "ms — total"),
            ("robot_ms",     "ms — robot build"),
            ("setup_ms",     "ms — task setup"),
            ("loop_ms",      "ms — solve loop"),
            ("iterations",   "iters"),
            ("iter_ms",      "ms/iter"),
            ("pos_err_mm",   "mm — IK residual"),
            ("track_err_mm", "mm — tracking error"),
            ("sigma_min",    "— Jacobian min singular value"),
            ("lambda_dls",   "— adaptive DLS damping"),
        ]:
            vals = [float(r.get(key, 0)) for r in rows]
            print(
                f"  {key:<16} mean={np.mean(vals):7.3f}  "
                f"median={np.median(vals):7.3f}  "
                f"p95={np.percentile(vals,95):7.3f}  "
                f"max={max(vals):7.3f}  [{unit}]"
            )
        n    = len(rows)
        miss = sum(r["deadline_missed"] for r in rows)
        ok   = sum(r["success"] for r in rows)
        print(f"\n  success_rate : {ok}/{n} = {ok/n*100:.1f}%")
        print(f"  deadline_miss: {miss}/{n} = {miss/n*100:.1f}%  "
              f"(budget={self._deadline_ms:.1f}ms)")
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
        plot_path = self.args.plot or _png_for(self._csv_path)
        t      = list(range(len(rows)))
        iters_ = [r["iterations"] for r in rows]
        pos_e  = [r["pos_err_mm"]    for r in rows]
        trk_e  = [r["track_err_mm"]  for r in rows]
        missed = [r["deadline_missed"] for r in rows]
        succ   = [r["success"] for r in rows]

        sigma_ = [r.get("sigma_min",   0.0) for r in rows]
        lam_   = [r.get("lambda_dls",  0.0) for r in rows]
        from placo_ik_session import _DLS_SIGMA_THRESH, _DLS_LAMBDA_BASE, _DLS_LAMBDA_MAX

        fig, axes = plt.subplots(4, 2, figsize=(14, 16))
        fig.suptitle(
            f"Placo Online Profiler  arm={self.args.arm}  "
            f"{'rebuild' if self.args.rebuild else 'cached'}\n"
            f"n={len(rows)}  rate={self._rate_hz:.0f}Hz  deadline={self._deadline_ms:.1f}ms",
            fontsize=11,
        )

        def _hm(ax, v, **kw):
            ax.axhline(float(np.mean(v)), linestyle="--", linewidth=0.8, **kw)

        ax = axes[0, 0]
        ax.stackplot(t,
            [r["robot_ms"] for r in rows],
            [r["setup_ms"] for r in rows],
            [r["loop_ms"]  for r in rows],
            labels=["robot_build", "setup", "loop"],
            colors=["tomato", "gold", "steelblue"], alpha=0.8)
        ax.axhline(self._deadline_ms, color="red", linestyle="--",
                   linewidth=1.2, label=f"deadline={self._deadline_ms:.0f}ms")
        ax.set_title("IK breakdown (ms)"); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        ax = axes[0, 1]
        ax.plot(t, iters_, color="purple", linewidth=0.7)
        _hm(ax, iters_, color="red", label=f"mean={np.mean(iters_):.1f}")
        ax.set_title("Iterations (early exit)"); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        ax = axes[1, 0]
        ax.plot(t, pos_e, color="tomato", linewidth=0.7)
        _hm(ax, pos_e, color="darkred", label=f"mean={np.mean(pos_e):.2f}mm")
        ax.set_title("IK residual (mm)"); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        ax = axes[1, 1]
        ax.plot(t, trk_e, color="darkorange", linewidth=0.7)
        _hm(ax, trk_e, color="saddlebrown", label=f"mean={np.mean(trk_e):.2f}mm")
        ax.set_title("Tracking error (mm)"); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        ax = axes[2, 0]
        ax.bar(["hit", "miss"],
               [sum(1 for v in missed if not v), sum(missed)],
               color=["steelblue", "red"])
        ax.set_title(f"Deadline: {sum(missed)}/{len(missed)} missed")

        ax = axes[2, 1]
        win = max(1, len(succ) // 20)
        sr_roll = [
            sum(succ[max(0, i-win):i+1]) / min(i+1, win+1) * 100
            for i in range(len(succ))
        ]
        ax.plot(t, sr_roll, color="green", linewidth=0.8)
        ax.set_ylim(-5, 105)
        ax.set_title(f"Rolling success rate % (win={win})"); ax.grid(True, alpha=0.3)

        # ── Row 4: Adaptive DLS — σ_min and λ_dls ────────────────────────
        ax = axes[3, 0]
        ax.plot(t, sigma_, color="royalblue", linewidth=0.8, label="σ_min")
        ax.axhline(_DLS_SIGMA_THRESH, color="red", linestyle="--", linewidth=1.0,
                   label=f"σ_thresh={_DLS_SIGMA_THRESH}")
        _hm(ax, sigma_, color="navy", label=f"mean={float(np.mean(sigma_)):.4f}")
        near_sing = sum(1 for s in sigma_ if s < _DLS_SIGMA_THRESH)
        ax.set_title(
            f"σ_min (Jacobian — Adaptive DLS)"
            f"  near-singular: {near_sing}/{len(sigma_)} ({near_sing/max(len(sigma_),1)*100:.0f}%)",
            fontsize=8,
        )
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
        ax.set_ylabel("σ_min"); ax.set_xlabel("step")

        ax = axes[3, 1]
        ax.plot(t, lam_, color="darkorchid", linewidth=0.8, label="λ_dls")
        ax.axhline(_DLS_LAMBDA_BASE, color="gray",  linestyle="--", linewidth=0.8,
                   label=f"λ_base={_DLS_LAMBDA_BASE:.0e}")
        ax.axhline(_DLS_LAMBDA_BASE + _DLS_LAMBDA_MAX, color="red",
                   linestyle="--", linewidth=0.8,
                   label=f"λ_max={_DLS_LAMBDA_BASE + _DLS_LAMBDA_MAX:.0e}")
        _hm(ax, lam_, color="purple", label=f"mean={float(np.mean(lam_)):.2e}")
        ax.set_yscale("log")
        ax.set_title("λ_dls (Adaptive DLS damping — log scale)", fontsize=8)
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3, which="both")
        ax.set_ylabel("λ_dls"); ax.set_xlabel("step")

        plt.tight_layout()
        plt.savefig(plot_path, dpi=120)
        plt.close()
        print(f"  [plot] → {plot_path}")

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._kbd.stop()
        self._csv_writer.close()    # drain queue before closing
        super().destroy_node()

    # ── Banner ────────────────────────────────────────────────────────────────
    def _print_banner(self):
        args = self.args
        cfg  = self.cfg
        print(f"\n{'═'*65}")
        print(f"  Placo Online Profiler")
        print(f"  arm={args.arm}  mode={'rebuild' if args.rebuild else 'CACHED+early_exit'}")
        print(f"  rate={self._rate_hz:.0f}Hz  deadline={self._deadline_ms:.1f}ms")
        print(f"  ee_delta : {cfg['ee_delta_topic']}")
        if self._fwd_pub is not None:
            print(f"  cmd      : {cfg['fwd_cmd_topic']}  (ForwardCommandController)")
        else:
            print(f"  cmd      : {cfg['traj_topic']}  (JointTrajectoryController)")
        print(f"  horizon  : {self._horizon_ms:.1f}ms")
        print(f"  CSV      : {self._csv_path}")
        if self._ws_mesh is not None:
            s = self._ws_mesh.summary()
            print(f"  ws_mesh  : {s['n_reachable_voxels']} voxels  "
                  f"step={s['step_m']*100:.0f}cm  vol~{s['total_volume_cm3']:.0f}cm³")
        else:
            print(f"  ws_clamp : {'box' if self._ws_clamp else 'OFF'}")
        if self._lpf_active:
            alpha_str = (f"{self._lpf_alpha[0]:.2f}" if len(set(self._lpf_alpha)) == 1
                         else "[" + ",".join(f"{a:.2f}" for a in self._lpf_alpha) + "]")
            print(f"  lpf      : α={alpha_str}  (1st-order on joint cmd)")
        else:
            print(f"  lpf      : OFF")
        print(f"  publish  : {'success-gate (legacy freeze)' if self._success_gate else 'always (continuous approach, Set2-D)'}")
        if self._ori_lpf_active:
            print(f"  ori_lpf  : α={self._ori_lpf_alpha:.2f}  (SLERP EMA on target quat, test3)")
        else:
            print(f"  ori_lpf  : OFF")
        print(f"  rot_track: {'OFF' if self._no_rot_tracking else 'ON'}")
        print(f"  guards   : jump>{self._joint_jump_guard_deg:.1f}°/step (post-IK reject)"
              f"  +  wrist_vel_cap (pre-IK QP)"
              f"  +  gap>{self._ee_delta_gap_sec:.2f}s")
        print(f"{'═'*65}")
        print(f"  1-9=scale  +/-=fine  t=mode  p=pause  r=reset  h=home  ?=help  Ctrl-C=quit")
        print(f"  KEYBOARD: w/s=±Y  a/d=±X  q/e=±Z  i/k=pitch  j/l=yaw  u/o=roll")
        print(f"{'═'*65}")
        print(f"\n  {'step':>5}  {'ik_ms':>7}  {'robot':>6}  {'loop':>6}  {'iters':>5}"
              f"  {'pos_mm':>6}  {'tr_mm':>6}  {'mem_MB':>6}  {'scale':>6}  {'ddl':>4}")
        print(f"  {'─'*5}  {'─'*7}  {'─'*6}  {'─'*6}  {'─'*5}"
              f"  {'─'*6}  {'─'*6}  {'─'*6}  {'─'*6}  {'─'*4}")
