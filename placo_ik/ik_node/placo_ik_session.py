#!/usr/bin/env python3
"""
placo_ik_session.py
===================
Pure-Python IK session wrapping placo RobotWrapper + KinematicsSolver.
Zero ROS dependency — safe to import anywhere.

Fix-2 applied
-------------
  * velocity limits enabled by default (prevents single-step joint jumps)
  * dt = 1/rate_hz  (matches actual control period, not a fixed 10 ms)
  * early-exit T_ee cached: avoids one extra FK call per converged step

Anti-vibration patch
--------------------
  * _MAX_ITER 5 → 20  (sufficient for orientation convergence)
  * _W_ORI   0.8 → 1.0 (equal priority with position for teleoperation)
  * _W_REG   1e-5 → 1e-4 (stronger singularity damping)
  * early-exit checks BOTH position AND orientation convergence
  * returns ori_err_deg in profiling dict
"""

import os as _os, sys as _sys
_HERE = _os.path.dirname(_os.path.abspath(__file__))
_ROOT = _os.path.dirname(_HERE)
_sys.path.insert(0, _os.path.join(_ROOT, "ik_solver"))

import threading
import time
import numpy as np
from typing import Dict, List, Optional

# ── IK tuning constants ───────────────────────────────────────────────────────
_POS_TOL   = 0.003    # m   — early-exit position convergence threshold
_POS_RELAX = 0.010    # m   — success acceptance threshold (relaxed)
_ORI_TOL   = 0.050    # rad — early-exit orientation threshold (~2.9°)
_ORI_RELAX = 0.200    # rad — success orientation threshold (~11.5°)
_W_POS     = 1.0      # position task weight (baseline)

# ── IK weight choice (merge note) ─────────────────────────────────────────────
# Two tuning sets exist; we adopt the placo_ik branch values because they pair
# with adaptive DLS (low baseline λ, dynamically boosted at singularities).
#
#   placo_ik (ACTIVE):       _W_ORI=2.0  _W_JOINTS=5e-4  _W_REG=6e-5
#   jitter_test3 (REFERENCE): _W_ORI=1.0  _W_JOINTS=1e-4  _W_REG=1e-4
#
# Why placo_ik is picked:
#   - _W_ORI=2.0 → aggressive orientation tracking; adaptive DLS prevents wobble
#   - _W_REG=6e-5 baseline → DLS adapter boosts near singularity
# Switch by editing the lines below if the alternative is preferred (or A/B test).
_W_ORI     = 2.0      # orientation task weight
_W_JOINTS  = 5e-4     # naturalness (joint preference) weight
_W_REG     = 6e-5     # regularisation weight (DLS baseline, dynamically boosted)
# Iteration budget: a small baseline keeps latency low on the common
# (non-singular) case — ~91% of frames converge in ≤4 iters.  Near a singularity
# we BOTH raise λ (_DLS_LAMBDA_MAX) and grant more iters (_SINGULAR_ITER_BOOST),
# so the stronger damping has enough steps to converge instead of exiting with a
# 10-30 mm residual (the "arm sticks near singularity" symptom).
_MAX_ITER  = 5        # baseline solver iteration cap (non-singular region)

# ── Adaptive DLS damping constants (方案 C — wrist wobble fix) ────────────────
# λ = λ_base + λ_max × clamp((σ_thresh - σ_min) / σ_thresh, 0, 1)²
# Far from singularity  (σ_min ≥ σ_thresh): λ ≈ λ_base  (~6e-5)
# Near singularity      (σ_min → 0)        : λ → λ_base + λ_max  (~5e-2)
_DLS_LAMBDA_BASE  = 6e-5   # baseline λ (= _W_REG, behaviour unchanged when non-singular)
_DLS_LAMBDA_MAX   = 5e-2   # maximum λ boost at singularity (DEBUG_REPORT_20260525)
_DLS_SIGMA_THRESH = 0.15   # σ_min threshold below which damping ramps up

# Adaptive iteration budget: when σ_min < _DLS_SIGMA_THRESH the larger λ above
# shrinks each integration step, so the same pose error needs more steps to
# clear.  Grant _MAX_ITER × _SINGULAR_ITER_BOOST iters in that region only.
# Cost: singular-frame loop_ms rises ~0.5ms→~2ms — still far under 20ms deadline.
_SINGULAR_ITER_BOOST = 4   # near-singularity iteration multiplier (5 → 20)

# ── Wrist velocity cap (teleop smoothness) ────────────────────────────────────
# URDF default wrist (j5/j6/j7) velocity = 20.94 rad/s ≈ 1200°/s — far too fast
# for VR teleop, lets per-step IK noise pass straight through to the motors.
# Override via RobotWrapper.set_velocity_limit() to a teleop-friendly value.
# At 100Hz dt=10ms: 4.0 rad/s → 2.3°/step  (vs URDF: 12°/step).
_WRIST_VEL_CAP    = 1.0    # rad/s; 0 or negative → disabled (use URDF default)
_WRIST_JOINT_IDX  = (4, 5, 6)   # 0-based indices for joint 5, 6, 7

# ── Arm (j1-j4) velocity cap (teleop smoothness) ──────────────────────────────
# Cap the large/medium joints so each solver sub-iteration moves ≤ N degrees,
# mirroring the wrist treatment.  Unlike a joint-space LPF this constraint lives
# inside the QP, so the joints stay coordinated and the EE path is preserved
# (motion just slows, it does not bend).  Specified in deg/iter and converted to
# rad/s at runtime via the actual dt: cap_rad_s = radians(N) / dt — so the
# per-iteration angle stays N° regardless of control rate.
# URDF baselines: j1/j2 16.75 rad/s, j3/j4 5.45 rad/s.  Set 0 to disable (URDF).
_ARM_VEL_CAP_DEG_PER_ITER = 1.0          # per-iteration cap for j1/j2/j3/j4 (deg)
_ARM_JOINT_IDX            = (0, 1, 2, 3)  # 0-based indices for joint 1-4

# ── j3/j4 elbow-torso coupled safety ─────────────────────────────────────────
# When j3 internally rotates (j3>0 right / j3<0 left), the bent elbow (j4≥90°)
# swings toward the torso.  We apply two layers of protection:
#   1. Soft: extra JointsTask pulls j4 toward a safe pref during the QP solve.
#   2. Hard: post-solve clip enforces a dynamic j4 upper limit.
#
# Coupling rule:
#   j4_safe_hi = j4_nominal_hi - _J3_J4_COUPLE_RATE × max(0, j3_inward)
#   j3=0.00 → j4_hi = 2.20  (no restriction)
#   j3=0.15 → j4_hi = 2.20 - 0.225 = 1.975 (~113°)
#   j3=0.25 → j4_hi = 2.20 - 0.375 = 1.825 (~105°)
#   j3=0.35 → j4_hi = 2.20 - 0.525 = 1.675 (~96°)  ← max inward allowed
#
# Tune _J3_J4_COUPLE_RATE on hardware: larger = stricter j4 restriction.
# Set 0.0 to disable coupling entirely.
_J3_J4_COUPLE_RATE  = 1.5    # rad reduction in j4_hi per rad of j3 inward rotation
_J4_ELBOW_SAFE_MIN  = 1.30   # floor: j4_hi never clamped below ~74° (always keep some flex)
_J3_J4_COUPLE_START = 0.02   # dead-band: coupling inactive below this j3 inward angle
_J3_INWARD_IDX      = 2      # 0-based index: joint3
_J4_ELBOW_IDX       = 3      # 0-based index: joint4


def _rot_error_rad(current_R: np.ndarray, target_R: np.ndarray) -> float:
    """Shortest geodesic angle (rad) between two 3×3 rotation matrices.
    From jitter_test3 — used for orientation-aware early exit + reporting."""
    R_err = current_R.T @ target_R
    cos_angle = (float(np.trace(R_err)) - 1.0) * 0.5
    return float(np.arccos(np.clip(cos_angle, -1.0, 1.0)))


# ── Resource helper ───────────────────────────────────────────────────────────
def _mem_rss_kb() -> int:
    try:
        with open("/proc/self/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1])
    except OSError:
        pass
    return 0


# ── Placo IK session ──────────────────────────────────────────────────────────
class PlacoSession:
    """
    Holds one cached RobotWrapper for reuse across all IK steps.

    Parameters
    ----------
    urdf       : absolute path to the robot URDF  (from _find_urdf())
    arm        : "right" | "left"
    max_iter   : solver iteration cap
    rate_hz    : actual control-loop rate; dt is set to 1/rate_hz  [Fix-2]
    vel_limits : enable joint velocity limits in the solver         [Fix-2]
    """

    def __init__(
        self,
        urdf:           str,
        arm:            str,
        max_iter:       int   = _MAX_ITER,
        rate_hz:        float = 20.0,
        vel_limits:     bool  = True,
        wrist_vel_cap:  float = _WRIST_VEL_CAP,
        j3j4_couple:    bool  = True,
    ):
        import placo
        from placo_ik_solver import _HUMAN_RIGHT, _HUMAN_LEFT, _JOINT_NAMES

        self._placo      = placo
        self._urdf       = urdf
        self._arm        = arm
        self._max_iter   = max_iter
        self._dt         = 1.0 / rate_hz   # Fix-2: actual control period
        self._vel_limits = vel_limits      # Fix-2
        self._wrist_vel_cap = float(wrist_vel_cap)

        self._joint_names = _JOINT_NAMES[arm]
        human_cfg         = _HUMAN_RIGHT if arm == "right" else _HUMAN_LEFT
        self._ee_link     = f"openarm_{arm}_link7"
        self._lo          = [human_cfg[n][1] for n in self._joint_names]
        self._hi          = [human_cfg[n][2] for n in self._joint_names]
        self._pref        = {n: human_cfg[n][0] for n in self._joint_names}
        self._wrist_joint_names = [self._joint_names[i] for i in _WRIST_JOINT_IDX]
        self._arm_joint_names   = [self._joint_names[i] for i in _ARM_JOINT_IDX]
        self._arm_vel_cap_deg   = _ARM_VEL_CAP_DEG_PER_ITER
        # j3 inward sign: +1 for right (j3>0=inward), -1 for left (j3<0=inward)
        self._j3_inward_sign = 1.0 if arm == "right" else -1.0
        self._j3j4_couple    = j3j4_couple and (_J3_J4_COUPLE_RATE > 0.0)

        self._robot = placo.RobotWrapper(urdf, placo.Flags.ignore_collisions)
        self._apply_velocity_caps(self._robot)

        # Dedicated FK-only wrapper. fk() is called from the EePosePublisher
        # timer thread, concurrently with solve_step() in the hot loop; sharing
        # self._robot would mean two threads writing the same joint state.
        # A second wrapper costs one extra URDF parse at startup and removes
        # the race entirely — no lock in the IK hot path.
        self._fk_robot = placo.RobotWrapper(urdf, placo.Flags.ignore_collisions)
        self._fk_lock  = threading.Lock()

        cap_str = (f"wrist≤{self._wrist_vel_cap:.1f}rad/s"
                   if self._wrist_vel_cap > 0 else "wrist=URDF")
        if self._arm_vel_cap_deg > 0:
            _arm_cap_rad = np.radians(self._arm_vel_cap_deg) / self._dt
            cap_str += (f"  arm(j1-4)≤{self._arm_vel_cap_deg:.1f}°/it"
                        f"={_arm_cap_rad:.2f}rad/s")
        print(
            f"[PlacoSession] cached  arm={arm}  max_iter={max_iter}"
            f"  dt={self._dt*1000:.1f}ms  vel_limits={vel_limits}  {cap_str}"
        )

        # Cache Jacobian column indices for Adaptive DLS (方案 C).
        # v_offsets are URDF-fixed — compute once from the cached robot instance.
        self._jac_cols = [self._robot.get_joint_v_offset(n) for n in self._joint_names]
        print(f"[PlacoSession] adaptive-DLS  σ_thresh={_DLS_SIGMA_THRESH}  "
              f"λ_max={_DLS_LAMBDA_MAX}  jac_cols={self._jac_cols}")

        # Throttled mem sampling: read /proc/self/status only every N steps
        # to avoid 50 Hz syscall overhead during long runs.
        self._mem_sample_every = 50   # refresh every 50 solve_step calls
        self._mem_step_count   = 0
        self._mem_cached_kb    = 0

    # ── Throttled memory sampler ───────────────────────────────────────────────
    def _throttled_mem_kb(self) -> int:
        """Return RSS in KB; re-reads /proc only every _mem_sample_every calls."""
        self._mem_step_count += 1
        if self._mem_step_count % self._mem_sample_every == 1:
            self._mem_cached_kb = _mem_rss_kb()
        return self._mem_cached_kb

    # ── Velocity caps (teleop smoothness) ─────────────────────────────────────
    def _apply_velocity_caps(self, robot) -> None:
        """Override URDF velocity limits for teleop smoothness.

        Wrist (j5-7) → fixed rad/s cap (_wrist_vel_cap).
        Arm   (j1-4) → per-iteration degree cap, converted to rad/s via dt.
        Each is a no-op if its cap is <= 0 (keeps URDF defaults).
        """
        if self._wrist_vel_cap > 0.0:
            for name in self._wrist_joint_names:
                robot.set_velocity_limit(name, self._wrist_vel_cap)
        if self._arm_vel_cap_deg > 0.0:
            cap = float(np.radians(self._arm_vel_cap_deg) / self._dt)
            for name in self._arm_joint_names:
                robot.set_velocity_limit(name, cap)

    # ── Solve one step ────────────────────────────────────────────────────────
    def solve_step(
        self,
        target_xyz: np.ndarray,
        target_R:   np.ndarray,
        seed:       List[float],
        no_rot:     bool = False,
    ) -> Dict:
        """
        Run one IK step.

        Returns a profiling dict with fields:
          robot_ms, setup_ms, loop_ms, solve_ms, wall_ms,
          iterations, iter_ms, mem_kb, pos_err_mm,
          success, joints, ee_xyz, robot_was_cached
        """
        placo   = self._placo
        t_wall0 = time.perf_counter()

        # ── Robot (cached, built once in __init__) ────────────────────────────
        t0 = time.perf_counter()
        robot = self._robot
        robot_ms = (time.perf_counter() - t0) * 1000.0

        # ── Solver + tasks ────────────────────────────────────────────────────
        t0 = time.perf_counter()
        solver = placo.KinematicsSolver(robot)
        solver.enable_joint_limits(True)
        solver.enable_velocity_limits(self._vel_limits)   # Fix-2
        solver.mask_fbase(True)
        solver.dt = self._dt                              # Fix-2: actual period

        for name, val in zip(self._joint_names, seed):
            robot.set_joint(name, val)
        robot.update_kinematics()

        # ── Adaptive DLS (方案 C): Jacobian SVD → σ_min → λ ─────────────────
        # frame_jacobian is a native placo call (~0.01 ms); SVD of 6×7 is trivial.
        J_full    = robot.frame_jacobian(self._ee_link, "world")
        J_arm     = J_full[:, self._jac_cols]              # 6 × n_joints
        _svs      = np.linalg.svd(J_arm, compute_uv=False) # descending order
        sigma_min = float(_svs[-1])
        _ratio    = max(0.0, (_DLS_SIGMA_THRESH - sigma_min) / _DLS_SIGMA_THRESH)
        lambda_dls = _DLS_LAMBDA_BASE + _DLS_LAMBDA_MAX * _ratio * _ratio

        pos_task = solver.add_position_task(self._ee_link, target_xyz)
        pos_task.configure("pos", "soft", _W_POS)
        if not no_rot:
            ori_task = solver.add_orientation_task(self._ee_link, target_R)
            ori_task.configure("ori", "soft", _W_ORI)
        jt = solver.add_joints_task()
        jt.set_joints(self._pref)
        jt.configure("naturalness", "soft", _W_JOINTS)

        # ── j3/j4 elbow-torso safety coupling (soft guidance in QP) ──────────
        # Use seed j3 so the QP already targets a safe j4 pref during this step.
        j3_seed    = float(seed[_J3_INWARD_IDX]) * self._j3_inward_sign
        if self._j3j4_couple and j3_seed > _J3_J4_COUPLE_START:
            j4_name      = self._joint_names[_J4_ELBOW_IDX]
            j4_safe_hi   = max(_J4_ELBOW_SAFE_MIN,
                               self._hi[_J4_ELBOW_IDX] - _J3_J4_COUPLE_RATE * j3_seed)
            # Pull j4 pref down to the safe ceiling (never raises pref above nominal)
            j4_safe_pref = min(self._pref[j4_name], j4_safe_hi)
            jt_j4 = solver.add_joints_task()
            jt_j4.set_joints({j4_name: j4_safe_pref})
            jt_j4.configure("j4_elbow_safety", "soft", 2e-3)  # 4× stronger than naturalness

        reg = solver.add_regularization_task(lambda_dls)   # adaptive λ
        reg.configure("reg", "soft", 1.0)
        setup_ms = (time.perf_counter() - t0) * 1000.0

        # ── Iterative solve with early exit (T_ee cached) ─────────────────────
        t0 = time.perf_counter()
        iters_used = 0
        T_ee = None
        pos_err = float("inf")
        ori_err = 0.0 if no_rot else float("inf")
        # Near a singularity the boosted λ shrinks each step → grant more iters.
        max_iter_dyn = (self._max_iter if sigma_min >= _DLS_SIGMA_THRESH
                        else self._max_iter * _SINGULAR_ITER_BOOST)
        for _ in range(max_iter_dyn):
            solver.solve(True)
            robot.update_kinematics()
            iters_used += 1
            T_ee = robot.get_T_world_frame(self._ee_link)
            pos_err = float(np.linalg.norm(T_ee[:3, 3] - target_xyz))
            if pos_err < _POS_TOL:
                if no_rot:
                    break   # position-only: converged
                ori_err = _rot_error_rad(T_ee[:3, :3], target_R)
                if ori_err < _ORI_TOL:
                    break   # both pos AND ori converged
        loop_ms = (time.perf_counter() - t0) * 1000.0

        # ── Final state (reuse T_ee already computed in last early-exit check) ─
        if T_ee is None:
            T_ee = robot.get_T_world_frame(self._ee_link)
        joints  = [robot.get_joint(n) for n in self._joint_names]

        # ── j3/j4 elbow-torso safety coupling (hard post-clip guarantee) ─────
        # Recompute using the SOLVED j3 value (may differ from seed after QP).
        j3_solved  = joints[_J3_INWARD_IDX] * self._j3_inward_sign
        j3_inward  = max(0.0, j3_solved - _J3_J4_COUPLE_START)
        if self._j3j4_couple and j3_inward > 0.0:
            j4_clip_hi = max(_J4_ELBOW_SAFE_MIN,
                             self._hi[_J4_ELBOW_IDX] - _J3_J4_COUPLE_RATE * j3_inward)
            dynamic_hi = list(self._hi)
            dynamic_hi[_J4_ELBOW_IDX] = j4_clip_hi
        else:
            dynamic_hi = self._hi
        joints  = [max(l, min(h, q)) for q, l, h in zip(joints, self._lo, dynamic_hi)]
        pos_err = float(np.linalg.norm(T_ee[:3, 3] - target_xyz))
        ori_err = 0.0 if no_rot else _rot_error_rad(T_ee[:3, :3], target_R)
        success = int(pos_err < _POS_RELAX and (no_rot or ori_err < _ORI_RELAX))

        solve_ms = robot_ms + setup_ms + loop_ms
        return {
            "robot_ms":         robot_ms,
            "setup_ms":         setup_ms,
            "loop_ms":          loop_ms,
            "solve_ms":         solve_ms,
            "wall_ms":          (time.perf_counter() - t_wall0) * 1000.0,
            "iterations":       iters_used,
            "iter_ms":          loop_ms / iters_used if iters_used else 0.0,
            "mem_kb":           self._throttled_mem_kb(),
            "pos_err_mm":       pos_err * 1000.0,
            "ori_err_rad":      ori_err,
            "ori_err_deg":      float(np.degrees(ori_err)),
            "success":          success,
            "joints":           joints,
            "ee_xyz":           list(T_ee[:3, 3]),
            "robot_was_cached": True,
            "sigma_min":        sigma_min,
            "lambda_dls":       lambda_dls,
        }

    # ── Forward kinematics helper ─────────────────────────────────────────────
    def fk(self, joints: List[float]) -> Optional[List[float]]:
        """
        Compute FK from joint angles.

        Returns [x, y, z, qx, qy, qz, qw] for the EE, or None on error.

        Thread-safe: uses the dedicated _fk_robot wrapper (never the solver's),
        so it can be called from the EePosePublisher timer thread while
        solve_step() runs in the hot loop.
        """
        try:
            with self._fk_lock:
                robot = self._fk_robot
                for name, val in zip(self._joint_names, joints):
                    robot.set_joint(name, val)
                robot.update_kinematics()
                T = robot.get_T_world_frame(self._ee_link)
            xyz = T[:3, 3]
            R   = T[:3, :3]
            # Shepperd's numerically-stable R → quaternion (4-branch)
            tr = R[0, 0] + R[1, 1] + R[2, 2]
            if tr > 0:
                s  = 0.5 / np.sqrt(tr + 1.0)
                qw = 0.25 / s
                qx = (R[2, 1] - R[1, 2]) * s
                qy = (R[0, 2] - R[2, 0]) * s
                qz = (R[1, 0] - R[0, 1]) * s
            elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s  = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                qw = (R[2, 1] - R[1, 2]) / s
                qx = 0.25 * s
                qy = (R[0, 1] + R[1, 0]) / s
                qz = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s  = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                qw = (R[0, 2] - R[2, 0]) / s
                qx = (R[0, 1] + R[1, 0]) / s
                qy = 0.25 * s
                qz = (R[1, 2] + R[2, 1]) / s
            else:
                s  = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                qw = (R[1, 0] - R[0, 1]) / s
                qx = (R[0, 2] + R[2, 0]) / s
                qy = (R[1, 2] + R[2, 1]) / s
                qz = 0.25 * s
            return [float(xyz[0]), float(xyz[1]), float(xyz[2]),
                    float(qx), float(qy), float(qz), float(qw)]
        except Exception:
            return None
