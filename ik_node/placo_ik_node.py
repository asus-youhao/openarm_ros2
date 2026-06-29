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
_sys.path.insert(0, _os.path.join(_ROOT, "config"))            # arm_config

import collections
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
from tf2_ros import Buffer, TransformListener, TransformException

from placo_ik_solver import _find_urdf, _quat_to_rot
from placo_ik_session import PlacoSession, _MAX_ITER
from kbd_controller import KbdController
from paths import csv_path as _csv_path, png_for as _png_for
from arm_config import ARM_CONFIG   # L2-A: single source of truth

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
    # Set1-A complete: published joint command (post-LPF, post-guard).
    # Empty when joint_jump_guard=1 (nothing actually published that step).
    "q_cmd_0", "q_cmd_1", "q_cmd_2", "q_cmd_3", "q_cmd_4", "q_cmd_5", "q_cmd_6",
    # Closed-loop EE measurement from TfPoller cache at log time.
    "tf_x", "tf_y", "tf_z", "tf_qx", "tf_qy", "tf_qz", "tf_qw",
    # Tracker timing: header.stamp (sender) and node-side recv wall clock.
    "tracker_t_stamp", "tracker_t_recv",
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
    Output : Float64MultiArray (ForwardCommandController) + latency + profile JSON topics + CSV
    """

    def __init__(self, args):
        super().__init__(f"placo_ik_{args.arm}")
        self.args = args
        self.cfg  = ARM_CONFIG[args.arm]
        self._cbg = ReentrantCallbackGroup()

        # ── Timing (used by sub-inits, set first) ────────────────────────
        self._rate_hz     = float(getattr(args, "rate", 20.0))
        self._deadline_ms = 1000.0 / self._rate_hz

        self._init_pose_state()
        self._init_ee_delta(args)
        self._init_ik_session(args)
        self._init_motion_pipe(args)
        self._init_ros(args)
        self._init_telemetry(args)
        self._init_keyboard(args)

        self._print_banner()

    # ── Domain-grouped init methods (L2-B) ───────────────────────────────────

    def _init_pose_state(self):
        """Initialise EE pose and joint state tracking."""
        cfg = self.cfg
        hx, hy, hz, hqx, hqy, hqz, hqw = cfg["home_pose"]
        self._pose        = [hx, hy, hz, hqx, hqy, hqz, hqw]
        self._last_joints = list(cfg["home_joints"])
        self._joints_lock = threading.Lock()
        self._joint_states: dict = {}
        self._js_lock     = threading.Lock()

    def _init_ee_delta(self, args):
        """Initialise EE-delta session reference, calibration and SLERP filter."""
        self._ee_delta_ref_xyz  = None
        self._ee_delta_ref_q    = None
        self._ee_delta_last_t   = None
        self._ee_delta_gap_sec  = float(getattr(args, "ee_delta_gap_sec", 0.05))
        # Explicit re-anchor signal from publisher (PoseStamped with
        # header.frame_id == "reanchor"). Set by _ee_delta_cb, consumed by
        # _build_target. Survives _pending overwrites — gap timing is fallback.
        self._reanchor_pending  = False
        # 0.10 s default: at 40 Hz tracker (25 ms/msg), needs 4+ consecutive dropped
        # messages to trigger accidental re-anchor.  Any intentional dead-man button
        # release (typically ≥ 100–200 ms) will always trigger re-anchor, preventing
        # the arm from snapping back to the old ref on the next button press.

        crpy = getattr(args, "calib_rpy", None)
        if crpy:
            rr, rp, ry = [math.radians(float(v)) for v in crpy.split(",")]
        else:
            rr, rp = 0.0, 0.0
            ry = math.radians(getattr(args, "calib_yaw", 0.0))
        self._calib_q         = _qfrom_rpy(rr, rp, ry)
        self._no_rot_tracking = bool(getattr(args, "no_rot_tracking", False))

        self._ori_lpf_alpha  = max(0.0, min(1.0, float(getattr(args, "ori_lpf_alpha", 0.35))))
        self._ori_lpf_active = self._ori_lpf_alpha < 0.999
        self._ori_filt_q: Optional[Tuple[float, float, float, float]] = None

    def _init_ik_session(self, args):
        """Build PlacoSession (cached RobotWrapper + KinematicsSolver)."""
        urdf = _find_urdf()
        self._placo_session = PlacoSession(
            urdf          = urdf,
            arm           = args.arm,
            max_iter      = getattr(args, "max_iter", _MAX_ITER),
            rate_hz       = self._rate_hz,
            vel_limits    = not getattr(args, "no_vel_limits", False),
            wrist_vel_cap = float(getattr(args, "wrist_vel_cap", 4.0)),
            j3j4_couple   = not getattr(args, "no_j3j4_couple", False),
        )

    def _init_motion_pipe(self, args):
        """Initialise output-side LPF, jump guard, and success-gate state."""
        n_joints = len(self.cfg["joint_names"])
        self._lpf_alpha: List[float] = _parse_lpf_alpha(
            getattr(args, "lpf_alpha", "1.0"), n_joints)
        self._lpf_active   = any(a < 0.999 for a in self._lpf_alpha)
        self._filt_joints: Optional[List[float]] = None

        self._joint_jump_guard_deg = float(getattr(args, "joint_jump_guard_deg", 15.0))
        self._success_gate    = bool(getattr(args, "success_gate", False))
        # guard-clamp mode: instead of rejecting the whole step when a joint jump
        # is detected, clamp each joint's delta to the guard threshold and publish
        # the clamped solution.  This lets the arm crawl toward the target instead
        # of freezing, which is safer near singularities during teleop.
        # Default: True (clamp).  --no-guard-clamp restores the legacy reject mode.
        self._guard_clamp     = not bool(getattr(args, "no_guard_clamp", False))
        self._fail_streak     = 0
        self._fail_warn_every = 20

    def _init_ros(self, args):
        """Create TF poller, ROS publishers and subscribers."""
        cfg = self.cfg
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_poller   = TfPoller(
            self._tf_buffer, cfg["base_link"], cfg["ee_link"], poll_sec=0.05)

        # ForwardCommandController — used for hot-loop streaming + homing
        self._fwd_pub  = self.create_publisher(Float64MultiArray, cfg["fwd_cmd_topic"], 10)
        self._latency_pub = self.create_publisher(Float32, cfg["latency_topic"], 10)
        self._profile_pub = self.create_publisher(String,  cfg["profile_topic"], 10)
        # JointState publisher for arm-specific IK commands (intermediate topic for aggregator)
        ik_cmd_topic = f"/{self.args.arm}_arm_ik_commands"
        self._ik_cmd_pub = self.create_publisher(JointState, ik_cmd_topic, 10)

        self.create_subscription(
            JointState, "/joint_states",
            self._js_cb, 10, callback_group=self._cbg)
        self.create_subscription(
            PoseStamped, cfg["ee_delta_topic"],
            self._ee_delta_cb, 10, callback_group=self._cbg)

        self._pending        = None
        self._pending_t_recv = None     # wall-clock recv time of the pending tracker msg
        self._delta_lock     = threading.Lock()
        self._msg_count      = 0

    def _init_telemetry(self, args):
        """Create async CSV writer and records list.

        _records is a bounded deque (keeps last _RECORDS_MAXLEN rows) so that
        memory usage is O(1) for arbitrarily long runs.  All cumulative stats
        (success, deadline, ik_ms, etc.) are maintained by O(1) running
        accumulators instead of re-scanning the full list each time.
        """
        _RECORDS_MAXLEN = 6000   # ~2 min at 50 Hz; enough for the final plot
        self._records: collections.deque = collections.deque(maxlen=_RECORDS_MAXLEN)

        # O(1) running accumulators — updated per step in _write_step.
        self._stat_n         = 0      # total step count
        self._stat_ok        = 0      # success count
        self._stat_miss      = 0      # deadline-miss count
        self._stat_ik_sum    = 0.0    # sum of ik_ms
        self._stat_loop_sum  = 0.0    # sum of loop_ms
        self._stat_iter_sum  = 0.0    # sum of iterations
        # Small ring buffer for median approximation (last 500 steps)
        _RING = 500
        self._ring_ik   = collections.deque(maxlen=_RING)
        self._ring_loop = collections.deque(maxlen=_RING)
        self._ring_iter = collections.deque(maxlen=_RING)

        csv_p = args.csv or _csv_path("placo_online", args.arm, "cached")
        self._csv_writer = AsyncCsvWriter(csv_p, CSV_FIELDS)
        self._csv_path   = csv_p

    def _init_keyboard(self, args):
        """Create KbdController (starts inactive until run() calls .start())."""
        self._kbd = KbdController(
            init_mode="keyboard" if getattr(args, "keyboard", False) else "tracker")

    # ── ROS callbacks ─────────────────────────────────────────────────────────
    def _js_cb(self, msg: JointState):
        with self._js_lock:
            for n, p in zip(msg.name, msg.position):
                self._joint_states[n] = p

    def _ee_delta_cb(self, msg: PoseStamped):
        t_recv = time.time()
        # Sentinel: trigger-press session-start signal. Latches a flag instead
        # of going into _pending, so a same-tick delta msg can't overwrite it.
        if msg.header.frame_id == "reanchor":
            with self._delta_lock:
                self._reanchor_pending = True
            self._msg_count += 1
            return
        with self._delta_lock:
            self._pending        = msg
            self._pending_t_recv = t_recv
        self._msg_count += 1

    # ── TF helpers ────────────────────────────────────────────────────────────
    def _get_tf(self, timeout_sec: float = 0.3) -> Optional[Tuple]:
        """Blocking TF lookup — only for startup init and send_home_fwd."""
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

    # ── Pre-home unfold sequence ──────────────────────────────────────────────
    def joint_unfold_sequence(
        self,
        ramp_hz:          float = 50.0,
        deg_per_sec:      float = 30.0,
        j4_skip_thresh_deg: float = 70.0,
    ) -> None:
        """
        Sequential single-joint ramp sequence executed before homing to safely
        unfold the arm from a potentially folded/collapsed posture.

        Full sequence (joint4 <= 70 deg at start):
          Step 1 — joint3 : current  → ±90 deg  (left arm -90, right arm +90)
          Step 2 — joint4 : current  → +90 deg  (aligns with home_joints[3])
          Step 3 — joint3 : ±90 deg  →   0 deg  (aligns with home_joints[2])

        Short sequence (joint4 > 70 deg at start):
          Elbow is already bent enough that the joint3 swing is unnecessary
          and may risk collision. Instead, ramp ALL joints simultaneously
          on a single cosine-smoothed timeline:
            joint4         : current  → +90 deg
            all others (6) : current  →   0 deg

        Uses ForwardCommandController (Float64MultiArray) stream.
        Waits up to 3 s for joint states.

        Note: called once only (before send_home_fwd).
        The max_tries inside send_home_fwd are TF arrival retries
        and do NOT re-trigger this unfold sequence.
        """
        print("  [unfold] -- joint_unfold_sequence() start --")

        jnames = self.cfg["joint_names"]
        print(f"  [unfold] joint_names = {jnames}")

        # ── Wait for joint states (up to 3 s) ────────────────────────────────
        wait_deadline = time.time() + 3.0
        while True:
            with self._js_lock:
                js = dict(self._joint_states)
            missing = [n for n in jnames if n not in js]
            if not missing:
                break
            if time.time() > wait_deadline:
                print(f"  [unfold] x joint states still missing after 3 s: {missing}"
                      " -> skip unfold sequence")
                return
            print(f"  [unfold]   waiting /joint_states ... missing: {missing}",
                  end="\r", flush=True)
            time.sleep(0.1)
        print()

        current = [js[n] for n in jnames]
        print(f"  [unfold] joint states (rad): {[f'{v:.4f}' for v in current]}")
        print(f"  [unfold] joint states (deg): {[f'{math.degrees(v):.1f}' for v in current]}")
        dt = 1.0 / ramp_hz

        # ── joint4 mode check ─────────────────────────────────────────────────
        j4_deg = math.degrees(current[3])
        short_mode = j4_deg > j4_skip_thresh_deg
        if short_mode:
            print(f"  [unfold] joint4={j4_deg:.1f} deg > {j4_skip_thresh_deg:.0f} deg "
                  f"-> short mode (all joints ramp simultaneously)")
        else:
            print(f"  [unfold] joint4={j4_deg:.1f} deg <= {j4_skip_thresh_deg:.0f} deg "
                  f"-> full 3-step sequence")

        def _ramp_joint(positions: list, idx: int, target_rad: float, label: str) -> list:
            """Ramp joint[idx] to target_rad with cosine smoothing."""
            start_rad = positions[idx]
            delta = abs(target_rad - start_rad)
            print(f"  [unfold] {label}: start={math.degrees(start_rad):.1f} deg"
                  f"  target={math.degrees(target_rad):.1f} deg"
                  f"  delta={math.degrees(delta):.1f} deg")
            if delta < math.radians(1.0):
                print(f"  [unfold] {label}: already near target (delta < 1 deg) -> skip")
                return positions[:]
            ramp_sec = max(0.5, delta / math.radians(deg_per_sec))
            n_steps  = int(ramp_sec * ramp_hz)
            print(f"  [unfold] {label}: ramp start  {ramp_sec:.1f}s  {n_steps} steps  "
                  f"ramp_hz={ramp_hz}  deg_per_sec={deg_per_sec}")
            result = positions[:]
            for step in range(n_steps + 1):
                alpha        = step / max(1, n_steps)
                alpha_smooth = 0.5 * (1.0 - math.cos(alpha * math.pi))
                result[idx]  = start_rad + alpha_smooth * (target_rad - start_rad)
                msg = Float64MultiArray()
                msg.data = list(result)
                self._fwd_pub.publish(msg)
                if step % max(1, n_steps // 5) == 0:
                    print(f"  [unfold]   {label}: {alpha * 100:.0f}%  "
                          f"joint[{idx}]={math.degrees(result[idx]):.1f} deg",
                          end="\r", flush=True)
                time.sleep(dt)
            print()
            print(f"  [unfold] {label}: done  final={math.degrees(result[idx]):.1f} deg")
            return result

        def _ramp_all(start: list, target: list, label: str) -> list:
            """Ramp all joints from start to target on a single cosine timeline.
            Duration is sized by the largest per-joint delta and deg_per_sec."""
            deltas = [abs(t - s) for s, t in zip(start, target)]
            max_delta = max(deltas)
            print(f"  [unfold] {label}: per-joint delta (deg) = "
                  f"{[f'{math.degrees(d):.1f}' for d in deltas]}")
            if max_delta < math.radians(1.0):
                print(f"  [unfold] {label}: already near target (max delta < 1 deg) -> skip")
                return start[:]
            ramp_sec = max(0.5, max_delta / math.radians(deg_per_sec))
            n_steps  = int(ramp_sec * ramp_hz)
            print(f"  [unfold] {label}: ramp start  {ramp_sec:.1f}s  {n_steps} steps  "
                  f"ramp_hz={ramp_hz}  deg_per_sec={deg_per_sec}")
            result = start[:]
            for step in range(n_steps + 1):
                alpha        = step / max(1, n_steps)
                alpha_smooth = 0.5 * (1.0 - math.cos(alpha * math.pi))
                result = [s + alpha_smooth * (t - s) for s, t in zip(start, target)]
                msg = Float64MultiArray()
                msg.data = list(result)
                self._fwd_pub.publish(msg)
                if step % max(1, n_steps // 5) == 0:
                    print(f"  [unfold]   {label}: {alpha * 100:.0f}%  "
                          f"deg={[f'{math.degrees(v):.1f}' for v in result]}",
                          end="\r", flush=True)
                time.sleep(dt)
            print()
            print(f"  [unfold] {label}: done  "
                  f"final={[f'{math.degrees(v):.1f}' for v in result]}")
            return result

        if short_mode:
            # Short mode: ramp all joints simultaneously
            #   joint4 -> 90 deg, all others -> 0 deg
            target_all = [0.0] * len(current)
            target_all[3] = math.pi / 2.0
            print("\n  [unfold] === Short mode: all joints simultaneous "
                  "(joint4 -> 90, others -> 0) ===")
            current = _ramp_all(current, target_all, "all->home_layout")
            time.sleep(0.3)
        else:
            j3_sign   = -1.0 if self.args.arm == "left" else 1.0
            j3_target = j3_sign * math.pi / 2.0

            # Step 1: joint3 -> +/-90 deg
            print(f"\n  [unfold] === Step 1: joint3 -> {math.degrees(j3_target):+.0f} deg "
                  f"(arm={self.args.arm}) ===")
            current = _ramp_joint(current, 2, j3_target,
                                  f"joint3->{math.degrees(j3_target):+.0f}deg")
            time.sleep(0.3)

            # Step 2: joint4 -> 90 deg
            print("\n  [unfold] === Step 2: joint4 -> 90 deg ===")
            current = _ramp_joint(current, 3, math.pi / 2.0, "joint4->90deg")
            time.sleep(0.3)

            # Step 3: joint3 -> 0 deg
            print("\n  [unfold] === Step 3: joint3 -> 0 deg ===")
            current = _ramp_joint(current, 2, 0.0, "joint3->0deg")
            time.sleep(0.3)

        print("  [unfold] unfold sequence complete")

    # ── Send home ─────────────────────────────────────────────────────────────
    def send_home_fwd(
        self,
        pos_tol:    float = 0.025,
        motion_sec: float = 3.5,
        ramp_hz:    float = 50.0,
    ) -> bool:
        """
        Ramp arm to home via ForwardCommandController (linear interpolation).

        Publishes intermediate joint positions at `ramp_hz` via
        Float64MultiArray.

        Used automatically when --home-first is specified.
        """
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
        # Stream directly via ForwardCommandController
        msg = Float64MultiArray()
        msg.data = joints
        self._fwd_pub.publish(msg)


        # Publish to arm-specific IK command topic (for aggregator to combine)
        ik_cmd_msg = JointState()
        ik_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        ik_cmd_msg.name = list(self.cfg["joint_names"])
        ik_cmd_msg.position = list(joints)
        self._ik_cmd_pub.publish(ik_cmd_msg)

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

    # ── run() helpers (L1-B) ─────────────────────────────────────────────────

    def _startup_sync(self):
        """Sync EE pose + joint positions; use FK as fallback when TF is absent.

        Priority:
          1. /joint_states   → _last_joints (seed for first IK step)
          2. TF2             → _pose         (best EE reference)
          3. FK from joints  → _pose         (if TF absent but joints known)
          4. home_pose       → _pose         (last resort, internal ≠ real state)

        Retries /joint_states for up to 3 s so that a briefly-late driver
        does not force the first IK step to use home_joints as seed (which
        causes the guard-clamp crawl symptom).
        """
        print("\n  Syncing from TF2 and /joint_states...")
        jnames = self.cfg["joint_names"]

        # ── Step 1: wait for /joint_states (up to 3 s) ────────────────────────
        deadline = time.time() + 3.0
        js: dict = {}
        while time.time() < deadline:
            with self._js_lock:
                js = dict(self._joint_states)
            if all(n in js for n in jnames):
                break
            time.sleep(0.1)

        js_ok = all(n in js for n in jnames)
        if js_ok:
            real_joints = [js[n] for n in jnames]
            with self._joints_lock:
                self._last_joints = real_joints
            print(f"  joints synced: {[f'{v:.3f}' for v in real_joints]}")
        else:
            real_joints = None
            print("  ⚠ /joint_states not ready after 3 s — using home_joints as seed "
                  "(first IK step may guard-clamp until arm syncs)")

        # ── Step 2: TF lookup ──────────────────────────────────────────────────
        tf = self._get_tf(1.0)
        if tf:
            self._pose = list(tf)
            print(f"  TF EE: {[f'{v:.4f}' for v in tf[:3]]}")
            return

        # ── Step 3: TF absent — recover pose via FK from real joint positions ──
        if real_joints is not None:
            fk_pose = self._placo_session.fk(real_joints)
            if fk_pose is not None:
                self._pose = fk_pose
                print(f"  ⚠ TF unavailable — pose recovered via FK: "
                      f"{[f'{v:.4f}' for v in fk_pose[:3]]}")
                return

        # ── Step 4: all sources failed — stay with home ────────────────────────
        print("  ⚠ TF & FK unavailable — using home_pose "
              "(internal EE reference ≠ real arm state)")

    def _handle_kbd_events(self) -> bool:
        """Handle keyboard flags. Returns True if quit was requested."""
        kbd = self._kbd
        if kbd.request_quit:
            print("\n  [kbd] quit — exiting")
            return True
        if kbd.request_home:
            kbd.request_home = False
            print("\n  [kbd] sending home...")
            self.send_home_fwd(pos_tol=0.025, motion_sec=3.5)
        if kbd.reset_ref:
            kbd.reset_ref          = False
            self._ee_delta_ref_xyz = None
            self._ee_delta_ref_q   = None
            self._ee_delta_last_t  = None
            self._reset_orientation_filter()
            print("\n  [kbd] session reference reset")
        return False

    def _fetch_pending(self):
        """Drain the pending input slot. Returns (msg, kbd_mode, tracker_t_recv).

        tracker_t_recv is the wall-clock time the latest tracker PoseStamped
        was received by _ee_delta_cb; None in keyboard mode or when no msg.
        """
        kbd      = self._kbd
        kbd_mode = kbd.mode
        with self._delta_lock:
            tracker_msg          = self._pending
            tracker_t_recv       = self._pending_t_recv
            self._pending        = None
            self._pending_t_recv = None
        if kbd_mode == "keyboard":
            xyz_d = kbd.pop_xyz()
            rpy_d = kbd.pop_rpy()
            msg   = (xyz_d, rpy_d) if (xyz_d is not None or rpy_d is not None) else None
            tracker_t_recv = None
        else:
            msg = tracker_msg
        return msg, kbd_mode, tracker_t_recv

    def _print_idle(self, step_count: int, kbd_mode: str):
        """Throttled idle-state console print (fires at most once per 2 s)."""
        idle_s = time.time() - self._no_msg_t
        if idle_s >= 2.0:
            self._no_msg_t = time.time()
            kbd = self._kbd
            paused_str = "  [PAUSED]" if kbd.paused else ""
            print(
                f"\r  [WAIT] {kbd_mode}{paused_str}  cb={self._msg_count}"
                f"  steps={step_count}  idle={idle_s:.0f}s  ×{kbd.scale:.2f}"
                f"  [{self._pose[0]:.3f} {self._pose[1]:.3f} {self._pose[2]:.3f}]",
                end="", flush=True,
            )

    def _build_target(self, msg, kbd_mode: str, t_wall: float):
        """
        Map tracker/keyboard input → IK target dict.

        Handles session-ref reset, calibration rotation,
        and SLERP orientation filter.

        Returns dict with keys:
            target_xyz, target_R, new_q, dx, dy, dz, new_x, new_y, new_z
        Returns None if message should be skipped.
        """
        kbd = self._kbd
        if kbd.paused:
            return None

        # ── Build raw delta (dx, dy, dz) and orientation delta dq ────────────
        if kbd_mode == "keyboard":
            xyz_d, rpy_d = msg
            dx, dy, dz = xyz_d if xyz_d is not None else (0., 0., 0.)
            dq         = _qfrom_rpy(*rpy_d) if rpy_d is not None else (0., 0., 0., 1.)
            base_xyz   = tuple(self._pose[:3])
            base_q     = tuple(self._pose[3:7])
            dx_arm     = (dx, dy, dz)   # already in robot base frame
        else:
            _s = kbd.scale
            dx = msg.pose.position.x * _s
            dy = msg.pose.position.y * _s
            dz = msg.pose.position.z * _s
            dq = (msg.pose.orientation.x, msg.pose.orientation.y,
                  msg.pose.orientation.z, msg.pose.orientation.w)
            if abs(dq[3]) < 0.01 and all(abs(v) < 0.01 for v in dq[:3]):
                dq = (0., 0., 0., 1.)

            gap    = (t_wall - self._ee_delta_last_t) if self._ee_delta_last_t else 999.
            with self._delta_lock:
                reanchor = self._reanchor_pending
                self._reanchor_pending = False
            is_new = (self._ee_delta_ref_xyz is None
                      or reanchor
                      or gap > self._ee_delta_gap_sec)
            self._ee_delta_last_t = t_wall

            if is_new:
                # Fix-1: instant cache read (was blocking _get_tf(0.3) = 0–300 ms)
                ref = self._tf_poller.get()
                src = "sentinel" if reanchor else "TF"
                if ref is not None:
                    self._ee_delta_ref_xyz = tuple(ref[:3])
                    self._ee_delta_ref_q   = tuple(ref[3:7])
                    self._pose             = list(ref)
                    print(f"\n  [EE-δ] NEW ref={[f'{v:.4f}' for v in ref[:3]]}  ({src})")
                else:
                    self._ee_delta_ref_xyz = tuple(self._pose[:3])
                    self._ee_delta_ref_q   = tuple(self._pose[3:7])
                self._reset_orientation_filter()
                # Discard the first message's delta entirely: the tracker app may
                # have accumulated movement during the dead-man button release, so
                # dx/dq of this first frame is unreliable.  Re-anchor only; arm
                # stays put.  Normal delta accumulation starts from the next frame.
                return None

            base_xyz = self._ee_delta_ref_xyz or tuple(self._pose[:3])
            base_q   = self._ee_delta_ref_q   or tuple(self._pose[3:7])
            dx_arm   = _rotate_vec((dx, dy, dz), self._calib_q)
            dq       = _rotate_quat(dq, self._calib_q)   # Fix-8: calib on orientation

        new_x = base_xyz[0] + dx_arm[0]
        new_y = base_xyz[1] + dx_arm[1]
        new_z = base_xyz[2] + dx_arm[2]

        # ── Orientation filter (input-side SLERP EMA) ─────────────────────────
        new_q_raw  = _qnorm(_qmul(dq, base_q))
        new_q      = base_q if self._no_rot_tracking else self._filter_orientation(new_q_raw)
        target_xyz = np.array([new_x, new_y, new_z])
        target_R   = _quat_to_rot(*new_q)

        return {
            "target_xyz": target_xyz, "target_R": target_R, "new_q": new_q,
            "dx": dx, "dy": dy, "dz": dz,
            "new_x": new_x, "new_y": new_y, "new_z": new_z,
        }

    def _run_ik_pipeline(self, target: dict, t_step: float) -> dict:
        """
        Run one IK step, apply jump guard, LPF filter, and publish.

        Anti-jump strategy (two complementary methods):
          Method 1 (proactive): wrist_vel_cap in PlacoSession caps Δq inside QP.
          Method 2 (reactive) : joint_jump_guard — two sub-modes:
            clamp mode (default, --no-guard-clamp to disable):
              Clip each joint delta to the guard threshold and publish the
              clamped solution.  Arm crawls toward target instead of freezing.
              _last_joints is updated so the seed advances each step.
            reject mode (legacy, --no-guard-clamp flag):
              Whole step rejected, _last_joints unchanged → arm freezes until
              IK finds a within-limit solution.

        Set2-D (continuous approach): on IK failure, still publish the partial
        solution; velocity_limits in the controller saturate safely.
        --success-gate restores the legacy freeze-on-failure behaviour.

        Returns merged dict: {r, ik_ms, total_ms, track_err, guard_hit,
                               max_joint_delta_deg, deadline_missed}.
        """
        target_xyz = target["target_xyz"]
        target_R   = target["target_R"]
        new_q      = target["new_q"]

        with self._joints_lock:
            seed = list(self._filt_joints or self._last_joints)

        t_total = time.perf_counter()
        r = self._placo_session.solve_step(
            target_xyz, target_R, seed, no_rot=self._no_rot_tracking)
        ik_ms    = r["solve_ms"]
        total_ms = (time.perf_counter() - t_total) * 1000.0

        track_err       = float(np.linalg.norm(np.array(r["ee_xyz"]) - target_xyz)) * 1000.0
        loop_wall_ms    = (time.perf_counter() - t_step) * 1000.0
        deadline_missed = int(loop_wall_ms > self._deadline_ms)

        guard_hit, max_joint_delta_deg = self._joint_jump_guard(seed, r["joints"])

        joints_out: Optional[List[float]] = None    # set below if step was published
        if guard_hit:
            if self._guard_clamp:
                # Clamp mode: clip each joint delta to the guard threshold so the
                # arm slowly crawls toward the IK solution instead of freezing.
                guard_rad = math.radians(self._joint_jump_guard_deg)
                joints_clamped = [
                    max(s - guard_rad, min(s + guard_rad, j))
                    for s, j in zip(seed, r["joints"])
                ]
                joints_out = self._filter_joints(joints_clamped)
                with self._joints_lock:
                    self._last_joints = list(joints_out)   # advance seed each step
                r["success"] = 0   # still mark as not perfectly solved
                self._fail_streak += 1
                if self._fail_streak % self._fail_warn_every == 0:
                    print(f"\n  [guard-clamp] joint Δ={max_joint_delta_deg:.1f}° "
                          f"clamped to {self._joint_jump_guard_deg:.0f}°, "
                          f"streak={self._fail_streak}")
                if not self.args.dry_run:
                    self._publish(joints_out)
            else:
                # Reject mode (legacy): whole step discarded, arm freezes.
                r["success"] = 0
                self._fail_streak += 1
                if self._fail_streak % self._fail_warn_every == 0:
                    print(f"\n  [guard-reject] joint Δ={max_joint_delta_deg:.1f}° "
                          f"rejected, streak={self._fail_streak}")
        else:
            joints_out = self._filter_joints(r["joints"])
            with self._joints_lock:
                self._last_joints = list(joints_out)

            if r["success"]:
                nx, ny, nz = target["new_x"], target["new_y"], target["new_z"]
                self._pose = [nx, ny, nz, new_q[0], new_q[1], new_q[2], new_q[3]]
                self._fail_streak = 0
            else:
                ee = r["ee_xyz"]
                self._pose[0] = float(ee[0])
                self._pose[1] = float(ee[1])
                self._pose[2] = float(ee[2])
                self._fail_streak += 1
                if self._fail_streak % self._fail_warn_every == 0:
                    print(f"\n  [Set2-D] IK pos_err={r['pos_err_mm']:.1f}mm "
                          f"streak={self._fail_streak} — publishing partial solve")

            if (r["success"] or not self._success_gate) and not self.args.dry_run:
                self._publish(joints_out)

        self._latency_pub.publish(Float32(data=float(ik_ms)))

        return {
            "r": r, "ik_ms": ik_ms, "total_ms": total_ms,
            "track_err": track_err, "guard_hit": guard_hit,
            "max_joint_delta_deg": max_joint_delta_deg,
            "deadline_missed": deadline_missed,
            "joints_out": joints_out,   # None when guard_hit (nothing published)
        }

    def _write_step(self, target: dict, pipeline: dict, t_wall: float, step_count: int,
                    tracker_t_stamp: float = 0.0, tracker_t_recv: Optional[float] = None):
        """Publish JSON profile, enqueue CSV row, and print console line."""
        r                   = pipeline["r"]
        ik_ms               = pipeline["ik_ms"]
        total_ms            = pipeline["total_ms"]
        track_err           = pipeline["track_err"]
        guard_hit           = pipeline["guard_hit"]
        max_joint_delta_deg = pipeline["max_joint_delta_deg"]
        deadline_missed     = pipeline["deadline_missed"]
        joints_out          = pipeline.get("joints_out")    # None when guard_hit
        dx, dy, dz          = target["dx"], target["dy"], target["dz"]

        tf_pose = self._tf_poller.get()   # (x,y,z,qx,qy,qz,qw) or None

        self._profile_pub.publish(String(data=json.dumps({
            "step":              step_count,
            "ik_ms":             round(ik_ms, 3),
            "robot_ms":          round(r["robot_ms"], 3),
            "setup_ms":          round(r["setup_ms"], 3),
            "loop_ms":           round(r["loop_ms"], 3),
            "iterations":        r["iterations"],
            "iter_ms":           round(r["iter_ms"], 4),
            "pos_err_mm":        round(r["pos_err_mm"], 3),
            "ori_err_deg":       round(r.get("ori_err_deg", 0.0), 3),
            "track_err_mm":      round(track_err, 3),
            "max_joint_delta_deg": round(max_joint_delta_deg, 3),
            "joint_jump_guard":  int(guard_hit),
            "success":           r["success"],
            "deadline_missed":   deadline_missed,
            "mem_mb":            round(r["mem_kb"] / 1024.0, 1),
            "mode":              "cached",
            "sigma_min":         round(r.get("sigma_min", 0.0), 6),
            "lambda_dls":        round(r.get("lambda_dls", 0.0), 8),
        })))

        row = {
            "t":                   round(t_wall, 6),
            "x":                   round(self._pose[0], 6),
            "y":                   round(self._pose[1], 6),
            "z":                   round(self._pose[2], 6),
            "success":             r["success"],
            "ik_ms":               round(ik_ms, 4),
            "total_ms":            round(total_ms, 4),
            "dx":                  round(dx, 6), "dy": round(dy, 6), "dz": round(dz, 6),
            "robot_ms":            round(r["robot_ms"], 4),
            "setup_ms":            round(r["setup_ms"], 4),
            "loop_ms":             round(r["loop_ms"], 4),
            "iterations":          r["iterations"],
            "iter_ms":             round(r["iter_ms"], 5),
            "pos_err_mm":          round(r["pos_err_mm"], 4),
            "ori_err_deg":         round(r.get("ori_err_deg", 0.0), 4),
            "track_err_mm":        round(track_err, 4),
            "max_joint_delta_deg": round(max_joint_delta_deg, 4),
            "joint_jump_guard":    int(guard_hit),
            "mem_kb":              r["mem_kb"],
            "deadline_missed":     deadline_missed,
            "sigma_min":           round(r.get("sigma_min", 0.0), 6),
            "lambda_dls":          round(r.get("lambda_dls", 0.0), 8),
            "tracker_t_stamp":     round(tracker_t_stamp, 6) if tracker_t_stamp else "",
            "tracker_t_recv":      round(tracker_t_recv, 6)  if tracker_t_recv  else "",
        }
        # Published joint command (post-LPF). Empty when guard rejected the step.
        if joints_out is not None:
            for i, q in enumerate(joints_out[:7]):
                row[f"q_cmd_{i}"] = round(float(q), 6)
        # Closed-loop EE measurement from TF cache.
        if tf_pose is not None:
            row["tf_x"]  = round(tf_pose[0], 6)
            row["tf_y"]  = round(tf_pose[1], 6)
            row["tf_z"]  = round(tf_pose[2], 6)
            row["tf_qx"] = round(tf_pose[3], 6)
            row["tf_qy"] = round(tf_pose[4], 6)
            row["tf_qz"] = round(tf_pose[5], 6)
            row["tf_qw"] = round(tf_pose[6], 6)
        self._csv_writer.put(row)
        self._records.append(row)

        # Update O(1) running accumulators
        self._stat_n        += 1
        self._stat_ok       += int(r["success"])
        self._stat_miss     += pipeline["deadline_missed"]
        self._stat_ik_sum   += ik_ms
        self._stat_loop_sum += float(r.get("loop_ms", 0.0))
        self._stat_iter_sum += int(r.get("iterations", 0))
        self._ring_ik.append(ik_ms)
        self._ring_loop.append(float(r.get("loop_ms", 0.0)))
        self._ring_iter.append(int(r.get("iterations", 0)))

        kbd = self._kbd
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

    # ── Main control loop ─────────────────────────────────────────────────────
    def run(self):
        """50/20 Hz IK loop. Orchestrates startup, background services, hot loop."""
        dt_sec = 1.0 / self._rate_hz
        self._kbd.start()
        self._startup_sync()
        self._tf_poller.start()     # Fix-1: background TF cache
        self._csv_writer.start()    # Fix-3: background CSV drain

        print(f"\n  Waiting for {self.cfg['ee_delta_topic']} ...\n")
        self._no_msg_t = time.time()
        step_count     = 0

        while rclpy.ok():
            t_step = time.perf_counter()

            if self._handle_kbd_events():
                break

            msg, kbd_mode, tracker_t_recv = self._fetch_pending()

            if msg is None or self._kbd.paused:
                self._print_idle(step_count, kbd_mode)
            else:
                self._no_msg_t = time.time()
                t_wall = time.time()
                # Tracker sender timestamp (PoseStamped.header.stamp); 0 for keyboard.
                if kbd_mode == "keyboard" or not hasattr(msg, "header"):
                    tracker_t_stamp = 0.0
                else:
                    s = msg.header.stamp
                    tracker_t_stamp = float(s.sec) + float(s.nanosec) * 1e-9
                target = self._build_target(msg, kbd_mode, t_wall)
                if target is not None:
                    pipeline = self._run_ik_pipeline(target, t_step)
                    step_count += 1
                    self._write_step(target, pipeline, t_wall, step_count,
                                     tracker_t_stamp, tracker_t_recv)
                    if step_count % 50 == 0:
                        self._print_partial_stats()

            sleep_sec = dt_sec - (time.perf_counter() - t_step)
            if sleep_sec > 0:
                time.sleep(sleep_sec)

    # ── Stats ─────────────────────────────────────────────────────────────────
    def _print_partial_stats(self):
        """O(1) stats print using running accumulators + small ring-buffer median."""
        n = self._stat_n
        if n == 0:
            return
        ok   = self._stat_ok
        miss = self._stat_miss
        # Median from last-500-step ring buffer (O(500) = O(1) effectively)
        ik_med   = float(np.median(list(self._ring_ik)))   if self._ring_ik   else 0.0
        loop_med = float(np.median(list(self._ring_loop))) if self._ring_loop else 0.0
        iter_med = float(np.median(list(self._ring_iter))) if self._ring_iter else 0.0
        print(
            f"\n  ─── [{n}] sr={ok/n*100:.0f}%  "
            f"ik_med={ik_med:.2f}ms  "
            f"loop_med={loop_med:.2f}ms  "
            f"iters_med={iter_med:.1f}  "
            f"ddl={miss}/{n}={miss/n*100:.0f}%"
        )

    def print_final_stats(self):
        rows = list(self._records)   # snapshot of bounded deque (last 6000 steps)
        # Use running accumulators for total n / ok / miss (covers full run, not
        # just the bounded deque window).
        n_total = self._stat_n
        if n_total == 0:
            return
        print(f"\n\n{'═'*65}")
        print(f"  Final stats  arm={self.args.arm}  n={n_total}"
              + (f"  (plot/metrics from last {len(rows)} steps)" if len(rows) < n_total else ""))
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
        miss = self._stat_miss
        ok   = self._stat_ok
        print(f"\n  success_rate : {ok}/{n_total} = {ok/n_total*100:.1f}%")
        print(f"  deadline_miss: {miss}/{n_total} = {miss/n_total*100:.1f}%  "
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

        rows = list(self._records)   # snapshot of bounded deque
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
            f"Placo Online Profiler  arm={self.args.arm}  cached\n"
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
        print(f"  arm={args.arm}  mode=CACHED+early_exit")
        print(f"  rate={self._rate_hz:.0f}Hz  deadline={self._deadline_ms:.1f}ms")
        print(f"  ee_delta : {cfg['ee_delta_topic']}")
        print(f"  cmd      : {cfg['fwd_cmd_topic']}  (ForwardCommandController)")
        print(f"  ik_cmd   : /{args.arm}_arm_ik_commands  (for aggregator)")
        print(f"  CSV      : {self._csv_path}")
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
