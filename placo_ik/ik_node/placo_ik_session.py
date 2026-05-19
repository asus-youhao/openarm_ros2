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

import time
import numpy as np
from typing import Dict, List

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
#   placo_ik (ACTIVE):       _W_ORI=2.0  _W_JOINTS=5e-4  _W_REG=6e-5  _MAX_ITER=15
#   jitter_test3 (REFERENCE): _W_ORI=1.0  _W_JOINTS=1e-4  _W_REG=1e-4  _MAX_ITER=20
#
# Why placo_ik is picked:
#   - _W_ORI=2.0 → aggressive orientation tracking; adaptive DLS prevents wobble
#   - _W_REG=6e-5 baseline → DLS adapter boosts to ~1e-2 near singularity
#   - _MAX_ITER=15 enough with adaptive λ; jitter_test3 needs 20 because λ is fixed
# Switch by editing 4 lines below if the alternative is preferred (or A/B test).
_W_ORI     = 2.0      # orientation task weight
_W_JOINTS  = 5e-4     # naturalness (joint preference) weight
_W_REG     = 6e-5     # regularisation weight (DLS baseline, dynamically boosted)
_MAX_ITER  = 15       # solver iteration cap

# ── Adaptive DLS damping constants (方案 C — wrist wobble fix) ────────────────
# λ = λ_base + λ_max × clamp((σ_thresh - σ_min) / σ_thresh, 0, 1)²
# Far from singularity  (σ_min ≥ σ_thresh): λ ≈ λ_base  (~6e-5)
# Near singularity      (σ_min → 0)        : λ → λ_base + λ_max  (~1e-2)
_DLS_LAMBDA_BASE  = 6e-5   # baseline λ (= _W_REG, behaviour unchanged when non-singular)
_DLS_LAMBDA_MAX   = 1e-2   # maximum λ boost at singularity
_DLS_SIGMA_THRESH = 0.15   # σ_min threshold below which damping ramps up

# ── Wrist velocity cap (teleop smoothness) ────────────────────────────────────
# URDF default wrist (j5/j6/j7) velocity = 20.94 rad/s ≈ 1200°/s — far too fast
# for VR teleop, lets per-step IK noise pass straight through to the motors.
# Override via RobotWrapper.set_velocity_limit() to a teleop-friendly value.
# At 100Hz dt=10ms: 4.0 rad/s → 2.3°/step  (vs URDF: 12°/step).
_WRIST_VEL_CAP    = 4.0    # rad/s; 0 or negative → disabled (use URDF default)
_WRIST_JOINT_IDX  = (4, 5, 6)   # 0-based indices for joint 5, 6, 7


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
    rebuild    : True → rebuild RobotWrapper every step (original ~25 ms behaviour)
    max_iter   : solver iteration cap
    rate_hz    : actual control-loop rate; dt is set to 1/rate_hz  [Fix-2]
    vel_limits : enable joint velocity limits in the solver         [Fix-2]
    """

    def __init__(
        self,
        urdf:           str,
        arm:            str,
        rebuild:        bool  = False,
        max_iter:       int   = _MAX_ITER,
        rate_hz:        float = 20.0,
        vel_limits:     bool  = True,
        wrist_vel_cap:  float = _WRIST_VEL_CAP,
    ):
        import placo
        from placo_ik_solver import _HUMAN_RIGHT, _HUMAN_LEFT, _JOINT_NAMES

        self._placo      = placo
        self._urdf       = urdf
        self._arm        = arm
        self._rebuild    = rebuild
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

        if not rebuild:
            self._robot = placo.RobotWrapper(urdf, placo.Flags.ignore_collisions)
            self._apply_velocity_caps(self._robot)
            cap_str = (f"wrist≤{self._wrist_vel_cap:.1f}rad/s"
                       if self._wrist_vel_cap > 0 else "wrist=URDF")
            print(
                f"[PlacoSession] cached  arm={arm}  max_iter={max_iter}"
                f"  dt={self._dt*1000:.1f}ms  vel_limits={vel_limits}  {cap_str}"
            )
        else:
            self._robot = None
            print(f"[PlacoSession] rebuild mode  arm={arm}  "
                  f"wrist_vel_cap={self._wrist_vel_cap:.1f}rad/s")

        # Cache Jacobian column indices for Adaptive DLS (方案 C).
        # v_offsets are URDF-fixed — compute once from any robot instance.
        _probe = self._robot if self._robot is not None else \
            placo.RobotWrapper(urdf, placo.Flags.ignore_collisions)
        self._jac_cols = [_probe.get_joint_v_offset(n) for n in self._joint_names]
        print(f"[PlacoSession] adaptive-DLS  σ_thresh={_DLS_SIGMA_THRESH}  "
              f"λ_max={_DLS_LAMBDA_MAX}  jac_cols={self._jac_cols}")

    # ── Wrist velocity cap (teleop smoothness) ────────────────────────────────
    def _apply_velocity_caps(self, robot) -> None:
        """Override URDF wrist velocity limits.  No-op if cap <= 0."""
        if self._wrist_vel_cap <= 0.0:
            return
        for name in self._wrist_joint_names:
            robot.set_velocity_limit(name, self._wrist_vel_cap)

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

        # ── Robot (build once or reuse) ───────────────────────────────────────
        t0 = time.perf_counter()
        if self._rebuild or self._robot is None:
            robot  = placo.RobotWrapper(self._urdf, placo.Flags.ignore_collisions)
            self._apply_velocity_caps(robot)
            cached = False
        else:
            robot  = self._robot
            cached = True
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
        reg = solver.add_regularization_task(lambda_dls)   # adaptive λ
        reg.configure("reg", "soft", 1.0)
        setup_ms = (time.perf_counter() - t0) * 1000.0

        # ── Iterative solve with early exit (T_ee cached) ─────────────────────
        t0 = time.perf_counter()
        iters_used = 0
        T_ee = None
        pos_err = float("inf")
        ori_err = 0.0 if no_rot else float("inf")
        for _ in range(self._max_iter):
            solver.solve(True)
            robot.update_kinematics()
            iters_used += 1
            if not self._rebuild:
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
        joints  = [max(l, min(h, q)) for q, l, h in zip(joints, self._lo, self._hi)]
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
            "mem_kb":           _mem_rss_kb(),
            "pos_err_mm":       pos_err * 1000.0,
            "ori_err_rad":      ori_err,
            "ori_err_deg":      float(np.degrees(ori_err)),
            "success":          success,
            "joints":           joints,
            "ee_xyz":           list(T_ee[:3, 3]),
            "robot_was_cached": cached,
            "sigma_min":        sigma_min,
            "lambda_dls":       lambda_dls,
        }
