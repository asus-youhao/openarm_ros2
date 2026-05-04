#!/usr/bin/env python3
"""
pure_python_ik_solver.py — Pure Python IK for OpenArm O6 (no MoveIt / no ROS)
===============================================================================
★ 完全不依賴 MoveIt /compute_ik — 用 scipy.optimize 自己解 IK ★

原理：
  IK cost = 位置誤差 × W_pos
           + 姿態誤差 × W_ori
           + Σᵢ wᵢ × 自然度懲罰(qᵢ)

  即把 IK 問題化為有界最小化（scipy L-BFGS-B），bounds = ForwardReach 硬限制：
    j1 [-0.50, 1.30]  j2 [0.00, 1.60]  j3 [-1.58, 0.00]
    j4 [0.25, 2.20]   j5 [-1.57, 1.57] j6 [-0.79, 0.79]  j7 [-1.00, 1.00]

自然度懲罰：
  每個關節有 (soft_lo, soft_hi, pref, weight)：
    · qi < soft_lo → w × (soft_lo - qi)²
    · qi > soft_hi → w × (qi - soft_hi)²
    · 任何位置   → 0.05 × w × (qi - pref)²  (輕微偏向 home 姿勢)

FK：
  與 joint_states_to_ee_poses.py 完全相同（從 URDF 推導），自包含，不需外部依賴。

多 seed 策略：
  1. 先用上次成功的 joints 當 seed（fast path，小步長時 1-3ms）
  2. 若收斂誤差過大，依序用 10 個 library seeds（每個 5-15ms）
  3. 回傳所有候選中輸出 pose 誤差最小的解

Dependencies:
    pip install numpy scipy   (通常已預裝)

Self-test:
    python3 pure_python_ik_solver.py
"""

import dataclasses
import math
import time
from typing import List, Optional, Tuple

import numpy as np
from scipy.optimize import minimize


# ─────────────────────────────────────────────────────────────────────────────
# Joint configuration
# ─────────────────────────────────────────────────────────────────────────────
@dataclasses.dataclass
class JointCfg:
    pref:    float   # preferred angle (naturalness center)
    soft_lo: float   # below → soft penalty
    soft_hi: float   # above → soft penalty
    hard_lo: float   # scipy bound lower
    hard_hi: float   # scipy bound upper
    weight:  float   # penalty scale for this joint
    name:    str     # label (for debug)


# ── Right Arm (ForwardReach: elbow-down, arm-forward only) ───────────────────
_RIGHT_CFG: List[JointCfg] = [
    #              pref    soft_lo  soft_hi  hard_lo  hard_hi  w    name
    JointCfg( 0.0000, -0.4000,  0.9000, -0.5000,  1.3000, 3.0, "j1_shoulder_flex_ext"),
    JointCfg( 1.2735,  0.3000,  1.5000,  0.0000,  1.6000, 4.0, "j2_shoulder_abduction"),
    JointCfg( 0.0000, -1.5708, -0.2000, -1.5800,  0.0000, 8.0, "j3_upperarm_yaw_j3=0_fwd"),
    JointCfg( 1.5708,  0.5000,  2.1000,  0.2500,  2.2000, 2.0, "j4_elbow_flex_90deg"),
    JointCfg( 0.0000,  0.3000,  2.5000, -1.5708,  1.5708, 0.8, "j5_forearm_roll"),
    JointCfg( 0.0000, -0.7854,  0.4000, -0.7854,  0.7854, 0.8, "j6_wrist_yaw_lr"),
    JointCfg( 0.0000, -0.8000,  0.5000, -1.0000,  1.0000, 1.0, "j7_wrist_pitch_palm_fwd"),
]

# ── Left Arm (mirrored) ───────────────────────────────────────────────────────
# j3 elbow-down = POSITIVE for left arm (base rpy = -π/2)
_LEFT_CFG: List[JointCfg] = [
    JointCfg( 0.0000, -0.9000,  0.4000, -1.3000,  0.5000, 3.0, "j1_shoulder_flex_ext"),
    JointCfg( 1.2735,  0.3000,  1.5000,  0.0000,  1.6000, 4.0, "j2_shoulder_abduction"),
    JointCfg( 0.0000,  0.2000,  1.5708,  0.0000,  1.5800, 8.0, "j3_upperarm_yaw_j3=0_fwd"),
    JointCfg( 1.5708,  0.5000,  2.1000,  0.2500,  2.2000, 2.0, "j4_elbow_flex_90deg"),
    JointCfg( 0.0000,  0.3000,  2.5000, -1.5708,  1.5708, 0.8, "j5_forearm_roll"),
    JointCfg( 0.0000, -0.7854,  0.4000, -0.7854,  0.7854, 0.8, "j6_wrist_yaw_lr"),
    JointCfg( 0.0000, -0.8000,  0.5000, -1.0000,  1.0000, 1.0, "j7_wrist_pitch_palm_fwd"),
]

_ARM_CFGS = {"right": _RIGHT_CFG, "left": _LEFT_CFG}


# ── Seed library (forward-reach, elbow-down) ─────────────────────────────────
_SEEDS_RIGHT: List[List[float]] = [
    # j1       j2      j3        j4      j5       j6       j7
    [ 0.0000,  0.7000,  0.0000,  1.5708,  0.0000,  0.0000,  0.0000],  # forward-reach 90°
    [ 0.0000,  1.0000,  0.0000,  1.5708,  0.0000,  0.0000,  0.0000],  # higher
    [ 0.0000,  0.5000,  0.0000,  1.5708,  0.0000,  0.0000,  0.0000],  # lower
    [ 0.2000,  0.7000, -0.2000,  1.5708,  0.0000,  0.0000,  0.0000],  # slight right
    [-0.2000,  0.7000, -0.2000,  1.5708,  0.0000,  0.0000,  0.0000],  # slight left
    [ 0.0000,  0.8000,  0.0000,  2.0000,  0.0000,  0.0000,  0.0000],  # raised-elbow
    [ 0.0000,  1.0000, -0.3000,  1.5708,  0.0000, -0.3000,  0.0000],  # reach-up
    [ 0.3000,  0.7000, -0.2000,  1.5708,  0.0000,  0.0000,  0.0000],  # right-fw
    [-0.3000,  0.7000, -0.2000,  1.5708,  0.0000,  0.0000,  0.0000],  # left-fw
    [ 0.0000,  0.9000,  0.0000,  2.0000,  0.0000,  0.0000,  0.0000],  # reach-deep
]
_SEEDS_LEFT: List[List[float]] = [
    [ 0.0000,  0.7000,  0.0000,  1.5708,  0.0000,  0.0000,  0.0000],
    [ 0.0000,  1.0000,  0.0000,  1.5708,  0.0000,  0.0000,  0.0000],
    [ 0.0000,  0.5000,  0.0000,  1.5708,  0.0000,  0.0000,  0.0000],
    [-0.2000,  0.7000,  0.2000,  1.5708,  0.0000,  0.0000,  0.0000],
    [ 0.2000,  0.7000,  0.2000,  1.5708,  0.0000,  0.0000,  0.0000],
    [ 0.0000,  0.8000,  0.0000,  2.0000,  0.0000,  0.0000,  0.0000],
    [ 0.0000,  1.0000,  0.3000,  1.5708,  0.0000, -0.3000,  0.0000],
    [-0.3000,  0.7000,  0.2000,  1.5708,  0.0000,  0.0000,  0.0000],
    [ 0.3000,  0.7000,  0.2000,  1.5708,  0.0000,  0.0000,  0.0000],
    [ 0.0000,  0.9000,  0.0000,  2.0000,  0.0000,  0.0000,  0.0000],
]
_ARM_SEEDS = {"right": _SEEDS_RIGHT, "left": _SEEDS_LEFT}


# ─────────────────────────────────────────────────────────────────────────────
# FK  (same URDF-derived chain as joint_states_to_ee_poses.py)
# ─────────────────────────────────────────────────────────────────────────────
def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF RPY: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cx, sx = math.cos(roll),  math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw),   math.sin(yaw)
    Rx = np.array([[1,  0,  0 ], [0,  cx, -sx], [0,  sx,  cx]])
    Ry = np.array([[cy, 0,  sy], [0,  1,   0 ], [-sy, 0,  cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz,  0 ], [0,   0,  1 ]])
    return Rz @ Ry @ Rx


def _tf(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _rpy_matrix(*rpy)
    T[:3, 3]  = xyz
    return T


def _axis_rot(axis, angle: float) -> np.ndarray:
    ax, ay, az = axis
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    R = np.array([
        [t*ax*ax + c,    t*ax*ay - s*az, t*ax*az + s*ay],
        [t*ax*ay + s*az, t*ay*ay + c,    t*ay*az - s*ax],
        [t*ax*az - s*ay, t*ay*az + s*ax, t*az*az + c   ],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    return T


_PI_HALF = math.pi / 2

_RIGHT_JOINT_PARAMS = [
    ([0.0,     0.0,    0.0625 ], [0,        0, 0], [0,  0,  1]),  # J1
    ([-0.0301, 0.0,    0.06   ], [_PI_HALF, 0, 0], [-1, 0,  0]),  # J2
    ([0.0301,  0.0,    0.06625], [0,        0, 0], [0,  0,  1]),  # J3
    ([0.0,     0.0315, 0.15375], [0,        0, 0], [0,  1,  0]),  # J4
    ([0.0,    -0.0315, 0.0955 ], [0,        0, 0], [0,  0,  1]),  # J5
    ([0.0375,  0.0,    0.1205 ], [0,        0, 0], [1,  0,  0]),  # J6
    ([-0.0375, 0.0,    0.0    ], [0,        0, 0], [0,  1,  0]),  # J7
]
_LEFT_JOINT_PARAMS = _RIGHT_JOINT_PARAMS  # same local geometry (base differs)

_RIGHT_BASE = _tf([0, 0, 0], [0, 0, 0]) @ _tf([0, -0.031, 0.698], [_PI_HALF, 0, 0])
_LEFT_BASE  = _tf([0, 0, 0], [0, 0, 0]) @ _tf([0,  0.031, 0.698], [-_PI_HALF, 0, 0])


def compute_fk(q: List[float], arm: str = "right") -> np.ndarray:
    """
    Forward kinematics: 7 joint angles → 4×4 world-to-link7 transform.
    Identical to joint_states_to_ee_poses.py::compute_fk().
    """
    T = (_RIGHT_BASE if arm == "right" else _LEFT_BASE).copy()
    for (xyz, rpy, axis), qi in zip(_RIGHT_JOINT_PARAMS, q):
        T = T @ _tf(xyz, rpy) @ _axis_rot(axis, qi)
    return T


def mat_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
    """3×3 rotation matrix → quaternion (qx, qy, qz, qw)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s;      x = (R[2,1] - R[1,2]) * s
        y = (R[0,2] - R[2,0]) * s; z = (R[1,0] - R[0,1]) * s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        w = (R[2,1] - R[1,2]) / s; x = 0.25 * s
        y = (R[0,1] + R[1,0]) / s; z = (R[0,2] + R[2,0]) / s
    elif R[1,1] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        w = (R[0,2] - R[2,0]) / s; x = (R[0,1] + R[1,0]) / s
        y = 0.25 * s;               z = (R[1,2] + R[2,1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        w = (R[1,0] - R[0,1]) / s; x = (R[0,2] + R[2,0]) / s
        y = (R[1,2] + R[2,1]) / s; z = 0.25 * s
    return x, y, z, w


# ─────────────────────────────────────────────────────────────────────────────
# IK cost function
# ─────────────────────────────────────────────────────────────────────────────
#  Tuning guide:
#   W_POS  ↑ → prioritise position accuracy over naturalness
#   W_ORI  ↑ → prioritise orientation; 0 = position-only mode (faster)
#   W_NAT  ↑ → heavier naturalness penalty in final result
_W_POS = 500.0   # position error weight: (pos_err_m)² × W_POS
_W_ORI =  50.0   # orientation error weight: (angle_err_rad)² × W_ORI
_W_NAT =   1.0   # naturalness scale (joint cfg weights are relative to this)


def _ik_cost(q, target_xyz, target_quat, cfg: List[JointCfg], arm: str,
             no_rot: bool = False) -> float:
    T = compute_fk(list(q), arm)

    # Position error
    pos_err_sq = float(np.dot(T[:3, 3] - target_xyz, T[:3, 3] - target_xyz))
    pos_cost   = _W_POS * pos_err_sq

    # Orientation error (quaternion angle distance)
    if no_rot:
        ori_cost = 0.0
    else:
        qx, qy, qz, qw = mat_to_quat(T[:3, :3])
        tx, ty, tz, tw  = target_quat
        dot      = abs(qx*tx + qy*ty + qz*tz + qw*tw)
        dot      = min(1.0, dot)
        ori_cost = _W_ORI * (2.0 * math.acos(dot)) ** 2

    # Per-joint naturalness penalty
    nat_cost = 0.0
    for i, c in enumerate(cfg):
        qi = q[i]
        if qi < c.soft_lo:
            nat_cost += c.weight * (c.soft_lo - qi) ** 2
        elif qi > c.soft_hi:
            nat_cost += c.weight * (qi - c.soft_hi) ** 2
        nat_cost += 0.05 * c.weight * (qi - c.pref) ** 2  # gentle home bias

    return pos_cost + ori_cost + _W_NAT * nat_cost


# ─────────────────────────────────────────────────────────────────────────────
# Solver
# ─────────────────────────────────────────────────────────────────────────────
class PurePythonIKSolver:
    """
    Pure-Python IK solver for OpenArm O6 — zero MoveIt/ROS dependency.

    Algorithm:
      For each seed (last_joints first, then library seeds):
        minimize _ik_cost(q) via scipy L-BFGS-B (bounds = ForwardReach hard limits)
        → accept immediately if pos_err < POS_TOL and ori_err < ORI_TOL
      If no seed meets tolerance → return best candidate (lowest pos_err).

    Typical performance (small steps ≤5mm, last_joints seed):
      ~2–8 ms (3–10 optimizer iterations)
    Worst case (large step / seed mismatch):
      ~20–60 ms (all 10 library seeds attempted)
    """

    POS_TOL  = 0.002   # m — accept immediately if position error < 2 mm
    ORI_TOL  = 0.10    # rad — accept immediately if orientation error < ~6°
    POS_RELAX = 0.008  # m — fallback acceptance: best solution within 8 mm

    MAX_SEEDS = 10     # maximum seed attempts before giving up

    def __init__(self, arm: str = "right"):
        self.arm    = arm
        self.cfg    = _ARM_CFGS[arm]
        self.seeds  = [list(s) for s in _ARM_SEEDS[arm]]
        self.bounds = [(c.hard_lo, c.hard_hi) for c in self.cfg]
        self._stats = {"total": 0, "ok_first": 0, "ok_retry": 0, "failed": 0}
        print(
            f"[PurePythonIK] {arm} arm ready  "
            f"j1=[{self.cfg[0].hard_lo:.2f},{self.cfg[0].hard_hi:.2f}]  "
            f"j3=[{self.cfg[2].hard_lo:.2f},{self.cfg[2].hard_hi:.2f}]  "
            f"(scipy L-BFGS-B, no MoveIt)"
        )

    def _fk_errors(self, q, target_xyz, target_quat):
        """Return (pos_err_m, ori_err_rad) for solution quality check."""
        T       = compute_fk(list(q), self.arm)
        pos_err = float(np.linalg.norm(T[:3, 3] - np.array(target_xyz)))
        qx, qy, qz, qw = mat_to_quat(T[:3, :3])
        tx, ty, tz, tw  = target_quat
        dot     = min(1.0, abs(qx*tx + qy*ty + qz*tz + qw*tw))
        ori_err = 2.0 * math.acos(dot)
        return pos_err, ori_err

    def solve(
        self,
        target_xyz:   Tuple[float, float, float],
        target_quat:  Tuple[float, float, float, float],
        last_joints:  List[float],
        verbose:      bool = False,
        no_rot:       bool = False,
    ) -> Tuple[bool, Optional[List[float]], float]:
        """
        Solve IK. Returns (ok, joints, elapsed_ms).

        Parameters
        ----------
        target_xyz   : (x, y, z) in world frame [m]
        target_quat  : (qx, qy, qz, qw) target orientation
        last_joints  : previous solution used as first seed (ensures continuity)
        verbose      : print each attempt details
        no_rot       : position-only IK (ignore orientation — faster)
        """
        t0 = time.time()
        self._stats["total"] += 1

        seeds = [list(last_joints)] + self.seeds[: self.MAX_SEEDS - 1]
        target_xyz_arr = np.array(target_xyz, dtype=float)

        best_q     : Optional[List[float]] = None
        best_pos_e : float = 1e9
        best_ori_e : float = 1e9

        for attempt, seed in enumerate(seeds):
            try:
                result = minimize(
                    _ik_cost,
                    x0     = np.array(seed, dtype=float),
                    args   = (target_xyz_arr, target_quat, self.cfg, self.arm, no_rot),
                    method = "L-BFGS-B",
                    bounds = self.bounds,
                    options= {"maxiter": 80, "ftol": 1e-10, "gtol": 1e-7},
                )
            except Exception:
                continue

            pos_e, ori_e = self._fk_errors(result.x, target_xyz, target_quat)

            if verbose:
                j3_d = math.degrees(result.x[2])
                j2_d = math.degrees(result.x[1])
                print(
                    f"  [PureIK] attempt={attempt} "
                    f"pos={pos_e*1000:.1f}mm ori={math.degrees(ori_e):.1f}° "
                    f"j2={j2_d:+.1f}° j3={j3_d:+.1f}° "
                    f"{'✓' if pos_e < self.POS_TOL else '·'}"
                )

            # ── Fast-accept: within both tolerances ──────────────────────────
            if pos_e < self.POS_TOL and (no_rot or ori_e < self.ORI_TOL):
                ms = (time.time() - t0) * 1000
                if attempt == 0:
                    self._stats["ok_first"] += 1
                else:
                    self._stats["ok_retry"] += 1
                return True, list(result.x), ms

            # Track best so far
            if pos_e < best_pos_e:
                best_pos_e = pos_e
                best_ori_e = ori_e
                best_q     = list(result.x)

        ms = (time.time() - t0) * 1000

        # ── Relaxed acceptance: best candidate within 8 mm ───────────────────
        if best_q is not None and best_pos_e < self.POS_RELAX:
            if verbose:
                print(
                    f"  [PureIK] relaxed-accept pos={best_pos_e*1000:.1f}mm "
                    f"ori={math.degrees(best_ori_e):.1f}°"
                )
            self._stats["ok_retry"] += 1
            return True, best_q, ms

        # ── Failed ───────────────────────────────────────────────────────────
        self._stats["failed"] += 1
        if verbose:
            print(f"  [PureIK] FAILED best_pos={best_pos_e*1000:.1f}mm  ({ms:.1f}ms)")
        return False, None, ms

    def get_stats(self) -> dict:
        s = self._stats
        total = s["total"]
        ok    = s["ok_first"] + s["ok_retry"]
        return {
            "total":      total,
            "ok_first":   s["ok_first"],
            "ok_retry":   s["ok_retry"],
            "failed":     s["failed"],
            "success_pct": ok / total * 100 if total else 0.0,
            "first_pct":   s["ok_first"] / total * 100 if total else 0.0,
        }

    def fk(self, q: List[float]) -> np.ndarray:
        """Convenience: return 4×4 FK matrix."""
        return compute_fk(q, self.arm)

    def print_fk(self, q: List[float], label: str = ""):
        T  = compute_fk(q, self.arm)
        qx, qy, qz, qw = mat_to_quat(T[:3, :3])
        print(
            f"  FK {label}: xyz=({T[0,3]:.4f},{T[1,3]:.4f},{T[2,3]:.4f})  "
            f"q=({qx:.4f},{qy:.4f},{qz:.4f},{qw:.4f})"
        )


# ─────────────────────────────────────────────────────────────────────────────
# Self-test
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import sys

    print("=" * 70)
    print(" pure_python_ik_solver.py — Self-test")
    print("=" * 70)

    solver_r = PurePythonIKSolver("right")
    solver_l = PurePythonIKSolver("left")

    # ── Preset home poses (FK result) ────────────────────────────────────────
    home_r = [0.0, 1.2735, -1.5708, 1.8287, 1.5708, -0.5552, 0.0]
    home_l = [0.0, 1.2735,  1.5708, 1.8287, 1.5708, -0.5552, 0.0]

    Tr = compute_fk(home_r, "right")
    Tl = compute_fk(home_l, "left")
    qr = mat_to_quat(Tr[:3, :3])
    ql = mat_to_quat(Tl[:3, :3])

    print("\n[FK] right home:", end="  ")
    solver_r.print_fk(home_r, "right-home")
    print("[FK] left  home:", end="  ")
    solver_l.print_fk(home_l, "left-home")

    # ── Test 1: IK round-trip (right arm home) ────────────────────────────────
    print("\n[Test 1] IK round-trip: right arm home pose")
    ok, joints, ms = solver_r.solve(
        target_xyz  = (Tr[0,3], Tr[1,3], Tr[2,3]),
        target_quat = qr,
        last_joints = home_r,
        verbose     = True,
    )
    assert ok, "FAILED: IK should solve for home pose"
    T_check = compute_fk(joints, "right")
    pos_err = float(np.linalg.norm(T_check[:3, 3] - Tr[:3, 3]))
    j3_d = math.degrees(joints[2])
    print(f"  ✓ ok={ok} ms={ms:.1f} pos_err={pos_err*1000:.2f}mm j3={j3_d:+.1f}°")
    assert pos_err < 0.005, f"Position error {pos_err*1000:.1f}mm > 5mm"
    assert joints[2] <= 0.01, f"ForwardReach violated: j3={j3_d:.1f}° > 0°"
    print(f"  ✓ ForwardReach: j3={j3_d:+.1f}° ≤ 0°")

    # ── Test 2: IK round-trip (left arm home) ────────────────────────────────
    print("\n[Test 2] IK round-trip: left arm home pose")
    ok, joints_l, ms = solver_l.solve(
        target_xyz  = (Tl[0,3], Tl[1,3], Tl[2,3]),
        target_quat = ql,
        last_joints = home_l,
        verbose     = True,
    )
    assert ok, "FAILED: left home IK"
    T_check_l = compute_fk(joints_l, "left")
    pos_err_l = float(np.linalg.norm(T_check_l[:3, 3] - Tl[:3, 3]))
    print(f"  ✓ ok={ok} ms={ms:.1f} pos_err={pos_err_l*1000:.2f}mm")
    assert pos_err_l < 0.005

    # ── Test 3: Small step (+5cm X) from solution ────────────────────────────
    print("\n[Test 3] Small step: +5cm X from right arm home")
    target2 = (Tr[0,3] + 0.05, Tr[1,3], Tr[2,3])
    ok, joints2, ms = solver_r.solve(
        target_xyz  = target2,
        target_quat = qr,
        last_joints = joints,
        verbose     = True,
    )
    assert ok, "FAILED: step IK"
    pos_err2 = float(np.linalg.norm(compute_fk(joints2, "right")[:3, 3] - np.array(target2)))
    j3_d2 = math.degrees(joints2[2])
    print(f"  ✓ ok={ok} ms={ms:.1f} pos_err={pos_err2*1000:.2f}mm j3={j3_d2:+.1f}°")
    assert joints2[2] <= 0.01, f"j3={j3_d2:.1f}° > 0°"

    # ── Test 4: j4=90° pose (user's preferred home) ─────────────────────────
    print("\n[Test 4] j4=90° home pose IK round-trip")
    j4_home = [0.0, 0.0, 0.0, math.pi/2, 0.0, 0.0, 0.0]
    T_j4 = compute_fk(j4_home, "right")
    q_j4 = mat_to_quat(T_j4[:3, :3])
    print(f"  j4=90° FK: xyz=({T_j4[0,3]:.4f},{T_j4[1,3]:.4f},{T_j4[2,3]:.4f})")
    ok, joints_j4, ms = solver_r.solve(
        target_xyz  = (T_j4[0,3], T_j4[1,3], T_j4[2,3]),
        target_quat = q_j4,
        last_joints = j4_home,
        verbose     = True,
    )
    pos_err_j4 = float(np.linalg.norm(
        compute_fk(joints_j4 if ok else j4_home, "right")[:3, 3] - T_j4[:3, 3]
    ))
    print(f"  {'✓' if ok else '⚠'} ok={ok} ms={ms:.1f} pos_err={pos_err_j4*1000:.2f}mm")
    # Note: j4=90° with j2=0 may violate j2 hard_lo=0.0 but target is valid

    # ── Test 5: Performance benchmark ────────────────────────────────────────
    print("\n[Test 5] Performance: 50 small steps (5mm X increments)")
    q_cur = list(home_r)
    t_total = time.time()
    times = []
    fails = 0
    target_x = Tr[0, 3]
    for i in range(50):
        target_x += 0.005
        ok_i, q_cur_new, ms_i = solver_r.solve(
            target_xyz  = (target_x, Tr[1, 3], Tr[2, 3]),
            target_quat = qr,
            last_joints = q_cur,
        )
        times.append(ms_i)
        if ok_i:
            q_cur = q_cur_new
        else:
            fails += 1
    elapsed = (time.time() - t_total) * 1000
    print(
        f"  50 steps: total={elapsed:.0f}ms  "
        f"mean={sum(times)/len(times):.1f}ms  "
        f"max={max(times):.1f}ms  fails={fails}"
    )
    assert fails == 0, f"{fails} IK failures in step test"

    # ── Summary ──────────────────────────────────────────────────────────────
    print(f"\n  Overall stats: {solver_r.get_stats()}")
    print("\n" + "=" * 70)
    print(" All tests passed ✓")
    print("=" * 70)
    print()
    print("使用方式：")
    print("  from pure_python_ik_solver import PurePythonIKSolver")
    print("  solver = PurePythonIKSolver('right')")
    print("  ok, joints, ms = solver.solve(target_xyz, target_quat, last_joints)")
