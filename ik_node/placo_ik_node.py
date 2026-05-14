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
from std_msgs.msg import Float32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener, TransformException

from placo_ik_solver import _find_urdf, _quat_to_rot
from placo_ws_analyze import WorkspaceMesh
from placo_ik_session import PlacoSession, _MAX_ITER
from kbd_controller import KbdController
from paths import csv_path as _csv_path, png_for as _png_for


# ── ARM config ────────────────────────────────────────────────────────────────
ARM_CONFIG = {
    "left": {
        "joint_names":   [f"openarm_left_joint{i}"  for i in range(1, 8)],
        "base_link":     "world",
        "ee_link":       "openarm_left_link7",
        "home_joints":   [0.0, 0.0, 0.0, 1.5708, 0.0, 0.0, 0.0],
        "home_pose":     (0.2160, 0.1535, 0.4780, 0.7071, -0.0000, 0.7071, -0.0000),
        "workspace":     {"x": (-0.20, 0.42), "y": (0.05, 0.45), "z": (0.35, 0.8)},
        "cmd_topic":     "/left_joint_trajectory_controller/joint_trajectory",
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
        "workspace":     {"x": (-0.20, 0.4), "y": (-0.4, -0.05), "z": (0.35, 0.8)},
        "cmd_topic":     "/right_joint_trajectory_controller/joint_trajectory",
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
    "pos_err_mm", "track_err_mm",
    "mem_kb", "deadline_missed",
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
        self._ee_delta_gap_sec = 0.35   # seconds before resetting session ref

        # Calibration quaternion (tracker → arm frame)
        crpy = getattr(args, "calib_rpy", None)
        if crpy:
            rr, rp, ry = [math.radians(float(v)) for v in crpy.split(",")]
        else:
            rr, rp = 0.0, 0.0
            ry = math.radians(getattr(args, "calib_yaw", 0.0))
        self._calib_q = _qfrom_rpy(rr, rp, ry)

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

        # IK session — Fix-2: rate_hz → dt=1/rate_hz; vel_limits=True
        urdf = _find_urdf()
        self._placo_session = PlacoSession(
            urdf       = urdf,
            arm        = args.arm,
            rebuild    = args.rebuild,
            max_iter   = getattr(args, "max_iter", _MAX_ITER),
            rate_hz    = self._rate_hz,
            vel_limits = not getattr(args, "no_vel_limits", False),
        )

        # Output-side LPF on joint commands  (Fix-D)
        n_joints = len(cfg["joint_names"])
        self._lpf_alpha: List[float] = _parse_lpf_alpha(
            getattr(args, "lpf_alpha", "1.0"), n_joints
        )
        self._lpf_active   = any(a < 0.999 for a in self._lpf_alpha)
        self._filt_joints: Optional[List[float]] = None

        # TF2 + Fix-1 poller (started in run() after initial sync)
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_poller   = TfPoller(
            self._tf_buffer, cfg["base_link"], cfg["ee_link"], poll_sec=0.05)

        # Publishers
        self._traj_pub    = self.create_publisher(JointTrajectory, cfg["cmd_topic"], 10)
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
        return False

    # ── Publish trajectory ────────────────────────────────────────────────────
    def _publish(self, joints: List[float]):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = list(self.cfg["joint_names"])
        pt = JointTrajectoryPoint()
        pt.positions     = list(joints)
        pt.time_from_start.nanosec = int(self._horizon_ms * 1e6)
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

                    base_xyz = self._ee_delta_ref_xyz or tuple(self._pose[:3])
                    base_q   = self._ee_delta_ref_q   or tuple(self._pose[3:7])
                    dx_arm   = _rotate_vec((dx, dy, dz), self._calib_q)

                # ── Workspace clamp ───────────────────────────────────────
                raw_x = base_xyz[0] + dx_arm[0]
                raw_y = base_xyz[1] + dx_arm[1]
                raw_z = base_xyz[2] + dx_arm[2]
                if self._ws_mesh is not None:
                    clamped, _ = self._ws_mesh.clamp(np.array([raw_x, raw_y, raw_z]))
                    new_x, new_y, new_z = float(clamped[0]), float(clamped[1]), float(clamped[2])
                elif self._ws_clamp:
                    new_x = max(ws["x"][0], min(ws["x"][1], raw_x))
                    new_y = max(ws["y"][0], min(ws["y"][1], raw_y))
                    new_z = max(ws["z"][0], min(ws["z"][1], raw_z))
                else:
                    new_x, new_y, new_z = raw_x, raw_y, raw_z

                new_q      = _qnorm(_qmul(dq, base_q))
                target_xyz = np.array([new_x, new_y, new_z])
                target_R   = _quat_to_rot(*new_q)

                with self._joints_lock:
                    seed = list(self._last_joints)

                # ── IK solve ──────────────────────────────────────────────
                t_total = time.perf_counter()
                r = self._placo_session.solve_step(target_xyz, target_R, seed)
                ik_ms    = r["solve_ms"]
                total_ms = (time.perf_counter() - t_total) * 1000.0

                track_err       = float(np.linalg.norm(
                    np.array(r["ee_xyz"]) - target_xyz)) * 1000.0
                loop_wall_ms    = (time.perf_counter() - t_step) * 1000.0
                deadline_missed = int(loop_wall_ms > self._deadline_ms)

                if r["success"]:
                    self._pose = [new_x, new_y, new_z,
                                  new_q[0], new_q[1], new_q[2], new_q[3]]
                    with self._joints_lock:
                        self._last_joints = r["joints"]      # raw, for next IK seed
                    joints_out = self._filter_joints(r["joints"])
                    if not self.args.dry_run:
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
                    "track_err_mm": round(track_err, 3),
                    "success":    r["success"],
                    "deadline_missed": deadline_missed,
                    "mem_mb":     round(r["mem_kb"] / 1024.0, 1),
                    "mode":       "rebuild" if self.args.rebuild else "cached",
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
                    "track_err_mm": round(track_err, 4),
                    "mem_kb":     r["mem_kb"],
                    "deadline_missed": deadline_missed,
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

        fig, axes = plt.subplots(3, 2, figsize=(14, 12))
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
        print(f"  cmd      : {cfg['cmd_topic']}")
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
        print(f"{'═'*65}")
        print(f"  1-9=scale  +/-=fine  t=mode  p=pause  r=reset  h=home  ?=help  Ctrl-C=quit")
        print(f"  KEYBOARD: w/s=±Y  a/d=±X  q/e=±Z  i/k=pitch  j/l=yaw  u/o=roll")
        print(f"{'═'*65}")
        print(f"\n  {'step':>5}  {'ik_ms':>7}  {'robot':>6}  {'loop':>6}  {'iters':>5}"
              f"  {'pos_mm':>6}  {'tr_mm':>6}  {'mem_MB':>6}  {'scale':>6}  {'ddl':>4}")
        print(f"  {'─'*5}  {'─'*7}  {'─'*6}  {'─'*6}  {'─'*5}"
              f"  {'─'*6}  {'─'*6}  {'─'*6}  {'─'*6}  {'─'*4}")
