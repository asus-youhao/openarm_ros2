#!/usr/bin/env python3
"""
pinocchio_ik_solver.py — Pinocchio-based IK solver for OpenArm O6
==================================================================
★ 使用 Pinocchio 3.x URDF FK + 加權 Damped Jacobian IK — 無需 MoveIt ★

特點：
  · Pinocchio 精確 URDF FK（與 ROS2 robot_state_publisher 一致）
  · 加權 DLS IK: (J^T J + λI + μW) dq = J^T err + μW(q_pref - q)
      J   = 6×7 frame Jacobian（LOCAL_WORLD_ALIGNED）
      λ   = damping (防止奇異點)
      μW  = per-joint naturalness 正則化（elbow-down preference）
  · 多 seed 策略：last_joints → library seeds
  · 取最好的解（最低位置誤差）

安裝（conda pico_teleop_py）：
    pip install pin   # pinocchio 3.x

URDF 路徑：
    /home/asus/Desktop/openarm_description/urdf_transform/urdf/openarm_step10.urdf
    (自動剝除 collision/visual meshes)

Self-test:
    conda run -n pico_teleop_py python3 pinocchio_ik_solver.py
"""

import math
import os
import time
import xml.etree.ElementTree as ET
from typing import List, Optional, Tuple

import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# URDF helper  (same as pybullet_ik_solver.py)
# ─────────────────────────────────────────────────────────────────────────────
# URDF file paths (in priority order, or override with OPENARM_URDF env var)
_URDF_SOURCES = [
    "/home/asus/openArm_leapHand_urdf/src/openarm_description/usd/v10_o6.urdf",
    "/home/asus/Desktop/openarm_description/urdf_transform/urdf/openarm_step10.urdf",
]
_KIN_URDF_CACHE = "/tmp/openarm_kin_pin.urdf"


def _build_kin_urdf(src: str, dst: str = _KIN_URDF_CACHE) -> str:
    tree = ET.parse(src)
    root = tree.getroot()
    for link in root.findall("link"):
        for tag in ("collision", "visual"):
            for el in list(link.findall(tag)):
                link.remove(el)
    tree.write(dst, xml_declaration=True, encoding="unicode")
    return dst


def _find_urdf() -> str:
    """
    Resolve URDF (priority):
      1. OPENARM_URDF env var  2. Local file paths in _URDF_SOURCES
    """
    env = os.environ.get("OPENARM_URDF")
    if env and os.path.isfile(env):
        print(f"[URDF] Using OPENARM_URDF env: {env}")
        return _build_kin_urdf(env)
    for src in _URDF_SOURCES:
        if os.path.isfile(src):
            print(f"[URDF] Using local file: {src}")
            return _build_kin_urdf(src)
    raise FileNotFoundError(
        "Cannot find OpenArm URDF. Ensure either:\n"
        "  1. OPENARM_URDF=/path/to/openarm.urdf  env var is set, OR\n"
        "  2. URDF exists at one of the paths in _URDF_SOURCES"
    )


# ─────────────────────────────────────────────────────────────────────────────
# Human-like joint configuration
# ─────────────────────────────────────────────────────────────────────────────
# (pref, soft_lo, soft_hi, hard_lo, hard_hi, weight)
# weight = μ in the normalisation cost μ * weight * (q - pref)²
_HUMAN_RIGHT = [
#                pref    soft_lo  soft_hi  hard_lo  hard_hi  weight   name
    ( 0.000, -0.400,   1.000,  -1.396,   1.500,   2.5),  # j1 shoulder-yaw  大馬達
    ( 0.700,  0.000,   1.600,   0.000,   1.600,   4.0),  # j2 shoulder-pitch  大馬達
    ( 0.000, -1.571,   0.000,  -1.571,   0.300,  10.0),  # j3 upperarm-yaw j3=0=forward  大馬達
    ( 1.5708, 0.250,   2.200,   0.250,   2.200,   2.0),  # j4 elbow-flex 90°
    ( 0.000, -1.571,   1.571,  -1.571,   1.571,   0.8),  # j5 forearm-roll
    ( 0.000, -0.785,   0.785,  -0.785,   0.785,   0.8),  # j6 wrist-yaw
    ( 0.000, -1.571,   1.571,  -1.571,   1.571,   0.5),  # j7 wrist-pitch palm-fwd
]
_HUMAN_LEFT = [
#                pref    soft_lo  soft_hi  hard_lo  hard_hi  weight   name
    ( 0.000, -1.000,   0.400,  -1.500,   1.396,   2.5),  # j1 mirror RIGHT: soft[-1.000,+0.400] hard[-1.500,+1.396]  大馬達
    (-0.700, -1.600,   0.000,  -1.600,   0.000,   4.0),  # j2 pref=-0.7  mirror RIGHT: hard[-1.600,0.000]  大馬達
    ( 0.000,  0.000,   1.571,  -0.300,   1.571,  10.0),  # j3 mirror RIGHT: soft[0,+1.571] hard[-0.300,+1.571]  大馬達
    ( 1.5708, 0.250,   2.200,   0.250,   2.200,   2.0),  # j4 same as right [0.250,+2.200]
    ( 0.000, -1.571,   1.571,  -1.571,   1.571,   0.8),  # j5 forearm-roll
    ( 0.000, -0.785,   0.785,  -0.785,   0.785,   0.8),  # j6 wrist-yaw
    ( 0.000, -1.571,   1.571,  -1.571,   1.571,   0.5),  # j7 wrist-pitch  axis=0 -1 0 (mirrored vs right)
]
_ARM_HUMAN = {"right": _HUMAN_RIGHT, "left": _HUMAN_LEFT}

_SEEDS_RIGHT = [
    [ 0.00,  0.70,  0.00,  1.5708,  0.00,  0.00,  0.00],  # forward-reach 90°
    [ 0.00,  0.50,  0.00,  1.5708,  0.00,  0.00,  0.00],
    [ 0.00,  1.00,  0.00,  1.5708,  0.00,  0.00,  0.00],
    [ 0.20,  0.70, -0.20,  1.5708,  0.00,  0.00,  0.00],
    [-0.20,  0.70, -0.20,  1.5708,  0.00,  0.00,  0.00],
    [ 0.00,  0.80,  0.00,  2.0000,  0.00,  0.00,  0.00],
    [ 0.00,  0.60,  0.00,  1.2000,  0.00,  0.30,  0.00],
]
_SEEDS_LEFT = [
    # j2 must be NEGATIVE for left arm (URDF mirror: j2_left=-j2_right)
    [ 0.00, -0.70,  0.00,  1.5708,  0.00,  0.00,  0.00],  # forward-reach home 90° (j2=-0.7)
    [ 0.00, -0.50,  0.00,  1.5708,  0.00,  0.00,  0.00],  # shoulder lower
    [ 0.00, -1.00,  0.00,  1.5708,  0.00,  0.00,  0.00],  # shoulder higher
    [-0.20, -0.70,  0.20,  1.5708,  0.00,  0.00,  0.00],  # j1 inward + j3 outward
    [ 0.20, -0.70,  0.20,  1.5708,  0.00,  0.00,  0.00],  # j1 outward + j3 outward
    [ 0.00, -0.80,  0.00,  2.0000,  0.00,  0.00,  0.00],  # elbow ~115°
    [ 0.00, -0.60,  0.00,  1.2000,  0.00,  0.30,  0.00],  # elbow 69° + forearm roll
]
_ARM_SEEDS = {"right": _SEEDS_RIGHT, "left": _SEEDS_LEFT}


def _quat_error(R_cur, R_target) -> np.ndarray:
    """
    Orientation error as 3D rotation vector (R_target @ R_cur^T → axis-angle).
    Returns (3,) array.
    """
    R_err   = R_target @ R_cur.T
    trace   = R_err[0, 0] + R_err[1, 1] + R_err[2, 2]
    angle   = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))
    if abs(angle) < 1e-6:
        return np.zeros(3)
    return angle / (2 * math.sin(angle)) * np.array([
        R_err[2, 1] - R_err[1, 2],
        R_err[0, 2] - R_err[2, 0],
        R_err[1, 0] - R_err[0, 1],
    ])


def _quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
    """Quaternion → 3×3 rotation matrix."""
    x, y, z, w = qx, qy, qz, qw
    return np.array([
        [1-2*(y*y+z*z),  2*(x*y-z*w),    2*(x*z+y*w)  ],
        [2*(x*y+z*w),    1-2*(x*x+z*z),  2*(y*z-x*w)  ],
        [2*(x*z-y*w),    2*(y*z+x*w),    1-2*(x*x+y*y)],
    ])


# ─────────────────────────────────────────────────────────────────────────────
# Solver
# ─────────────────────────────────────────────────────────────────────────────
class PinocchioIKSolver:
    """
    Iterative Jacobian IK using Pinocchio 3.x.

    Cost (per iteration):
        min dq:  || J_arm dq - err_6d ||²  +  μ Σ wᵢ(qᵢ + dqᵢ - qᵢ_pref)²

    Solution (each step):
        H dq = J^T err + μ W (q_pref - q)
        H = J^T J + λ²I + μW

    where:
        J    = 6×7 frame Jacobian (LOCAL_WORLD_ALIGNED, arm columns only)
        err  = [Δxyz; Δω]  (6D pose error)
        λ    = 0.01  (damping — prevents singularity)
        μ    = 0.05  (naturalness scale)
        W    = diag(w₁…w₇)  (per-joint naturalness weights)
    """

    POS_TOL   = 0.003
    ORI_TOL   = 0.15
    POS_RELAX = 0.010
    MAX_ITER  = 300        # per seed
    DT        = 0.3        # step size (0–1)
    LAMBDA    = 0.01       # damping
    # MU must be << 1 to avoid fighting the task objective.
    # With MU=0.05, the naturalness force creates a premature equilibrium
    # before the task is solved. MU=3e-4 keeps naturalness gentle.
    MU        = 3e-4       # naturalness regularisation scale

    def __init__(self, arm: str = "right"):
        import pinocchio as pin
        self._pin = pin
        self.arm  = arm

        urdf      = _find_urdf()
        self.model = pin.buildModelFromUrdf(urdf)
        self.data  = self.model.createData()

        # EE frame id
        self._ee_frame = self.model.getFrameId(f"openarm_{arm}_link7")

        # Dynamically detect q-vector indices for this arm's 7 joints.
        # The URDF has nq=36: left_arm(7) + left_hand(11) + right_arm(7) + right_hand(11)
        # so right arm starts at q[18], NOT q[7].
        _arm_jnames = [f"openarm_{arm}_joint{i}" for i in range(1, 8)]
        _q_idxs = []
        for jn in _arm_jnames:
            jid = self.model.getJointId(jn)
            _q_idxs.append(self.model.joints[jid].idx_q)
        _q_start = _q_idxs[0]
        self._q_slice = slice(_q_start, _q_start + 7)
        self._q_start = _q_start

        cfg = _ARM_HUMAN[arm]
        self._pref   = np.array([c[0] for c in cfg])
        self._lo     = np.array([c[3] for c in cfg])
        self._hi     = np.array([c[4] for c in cfg])
        self._W      = np.diag([c[5] for c in cfg])    # 7×7 weight matrix

        self._seeds = _ARM_SEEDS[arm]
        self._stats = {"total": 0, "ok_first": 0, "ok_retry": 0, "failed": 0}

        print(
            f"[PinocchioIK] {arm} arm ready  "
            f"ee_frame_id={self._ee_frame}  q_slice={self._q_slice}  "
            f"(pinocchio {pin.__version__})"
        )

    def _make_q(self, arm_q: List[float]) -> np.ndarray:
        """Build full pinocchio q vector (nq=14) from 7-DOF arm joints."""
        q = self._pin.neutral(self.model)
        q[self._q_slice] = arm_q
        return q

    def _fk(self, arm_q: List[float]):
        """Return (xyz, R 3×3) for the EE frame."""
        import pinocchio as pin
        q = self._make_q(arm_q)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        T = self.data.oMf[self._ee_frame]
        return np.array(T.translation), np.array(T.rotation)

    def _jacobian(self, arm_q: List[float]) -> np.ndarray:
        """Return 6×7 arm Jacobian (LOCAL_WORLD_ALIGNED)."""
        import pinocchio as pin
        q = self._make_q(arm_q)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        J_full = pin.getFrameJacobian(
            self.model, self.data,
            self._ee_frame,
            pin.LOCAL_WORLD_ALIGNED,
        )  # 6×nv
        # Extract the 7 arm columns using dynamically detected q indices
        return J_full[:, self._q_start:self._q_start + 7]

    def _solve_from_seed(
        self,
        target_xyz: np.ndarray,
        target_R:   np.ndarray,
        seed:       List[float],
        no_rot:     bool,
    ) -> Tuple[np.ndarray, float, float]:
        """Run iterative IK from one seed. Returns (q_arm, pos_err, ori_err)."""
        q_arm = np.array(seed, dtype=float)
        λ = self.LAMBDA
        μ = self.MU
        W = self._W

        for _ in range(self.MAX_ITER):
            xyz, R = self._fk(q_arm)
            err_pos = target_xyz - xyz

            if no_rot:
                err_6d = np.concatenate([err_pos, np.zeros(3)])
            else:
                err_ori = _quat_error(R, target_R)
                err_6d  = np.concatenate([err_pos, err_ori])

            J = self._jacobian(q_arm)  # 6×7 or 3×7 if no_rot
            if no_rot:
                J_use = J[:3, :]           # position-only
                e_use = err_6d[:3]
            else:
                J_use = J
                e_use = err_6d

            # Weighted DLS: H dq = rhs
            H   = J_use.T @ J_use + λ**2 * np.eye(7) + μ * W
            rhs = J_use.T @ e_use + μ * W @ (self._pref - q_arm)
            dq  = np.linalg.solve(H, rhs)

            q_arm = q_arm + self.DT * dq
            q_arm = np.clip(q_arm, self._lo, self._hi)

        xyz_final, R_final = self._fk(q_arm)
        pos_err = float(np.linalg.norm(xyz_final - target_xyz))
        ori_err = math.acos(min(1.0, abs(
            (np.trace(target_R @ R_final.T) - 1.0) / 2.0
        ))) if not no_rot else 0.0

        return q_arm, pos_err, ori_err

    def solve(
        self,
        target_xyz:  Tuple[float, float, float],
        target_quat: Tuple[float, float, float, float],
        last_joints: List[float],
        verbose:     bool = False,
        no_rot:      bool = False,
    ) -> Tuple[bool, Optional[List[float]], float]:
        t0 = time.time()
        self._stats["total"] += 1

        txyz    = np.array(target_xyz)
        tR      = _quat_to_rot(*target_quat)
        seeds   = [list(last_joints)] + self._seeds

        best_q      = None
        best_pos_e  = 1e9

        for attempt, seed in enumerate(seeds):
            try:
                q_arm, pos_e, ori_e = self._solve_from_seed(txyz, tR, seed, no_rot)
            except Exception as ex:
                if verbose:
                    print(f"  [PinIK] attempt {attempt} error: {ex}")
                continue

            if verbose:
                j3d = math.degrees(q_arm[2])
                print(
                    f"  [PinIK] attempt={attempt} "
                    f"pos={pos_e*1000:.1f}mm ori={math.degrees(ori_e):.1f}° "
                    f"j3={j3d:+.1f}° {'✓' if pos_e < self.POS_TOL else '·'}"
                )

            if pos_e < best_pos_e:
                best_pos_e  = pos_e
                best_q      = list(q_arm)

            if pos_e < self.POS_TOL and (no_rot or ori_e < self.ORI_TOL):
                ms = (time.time() - t0) * 1000
                if attempt == 0:
                    self._stats["ok_first"] += 1
                else:
                    self._stats["ok_retry"] += 1
                return True, list(q_arm), ms

        ms = (time.time() - t0) * 1000
        if best_q is not None and best_pos_e < self.POS_RELAX:
            self._stats["ok_retry"] += 1
            return True, best_q, ms

        self._stats["failed"] += 1
        return False, None, ms

    def get_stats(self) -> dict:
        s = self._stats
        t = s["total"]
        ok = s["ok_first"] + s["ok_retry"]
        return {
            "total": t, "ok_first": s["ok_first"],
            "ok_retry": s["ok_retry"], "failed": s["failed"],
            "success_pct": ok / t * 100 if t else 0.0,
            "first_pct":   s["ok_first"] / t * 100 if t else 0.0,
        }


# ─────────────────────────────────────────────────────────────────────────────
# Self-test
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 65)
    print(" pinocchio_ik_solver.py — Self-test")
    print("=" * 65)

    solver = PinocchioIKSolver("right")
    pref   = [_HUMAN_RIGHT[k][0] for k in range(7)]

    # Home pose via FK
    xyz_home, R_home = solver._fk(pref)
    w = math.sqrt(max(0, 1 - sum(x*x for x in [R_home[2,1]-R_home[1,2],
                                                 R_home[0,2]-R_home[2,0],
                                                 R_home[1,0]-R_home[0,1]])/4))
    print(f"\n[Pref FK] xyz={xyz_home}  (j3_pref={math.degrees(pref[2]):.1f}°)")

    print("\n[Test 1] round-trip from preferred home")
    from pure_python_ik_solver import mat_to_quat
    qhome = mat_to_quat(R_home)
    ok, joints, ms = solver.solve(tuple(xyz_home), qhome, pref, verbose=True)
    print(f"  ok={ok} ms={ms:.1f}")
    if ok:
        j3d = math.degrees(joints[2])
        pe = float(np.linalg.norm(solver._fk(joints)[0] - xyz_home))
        print(f"  pos_err={pe*1000:.1f}mm  j3={j3d:+.1f}° (elbow: {'down ✓' if j3d < 1 else 'UP ✗'})")

    print("\n[Test 2] +5cm step X")
    step = (float(xyz_home[0]) + 0.05, float(xyz_home[1]), float(xyz_home[2]))
    ok2, j2, ms2 = solver.solve(step, qhome, joints or pref, verbose=True)
    if ok2:
        pe2 = float(np.linalg.norm(solver._fk(j2)[0] - np.array(step)))
        j3d2 = math.degrees(j2[2])
        print(f"  ok={ok2} ms={ms2:.1f} pos_err={pe2*1000:.1f}mm j3={j3d2:+.1f}°")

    print("\n[Test 3] Performance: 30 steps 5mm each")
    q_cur = list(pref)
    tx = float(xyz_home[0])
    times, fails = [], 0
    for i in range(30):
        tx += 0.005
        ok_i, qn, ms_i = solver.solve((tx, float(xyz_home[1]), float(xyz_home[2])), qhome, q_cur)
        times.append(ms_i)
        if ok_i:
            q_cur = qn
        else:
            fails += 1
    print(f"  30 steps: mean={sum(times)/len(times):.1f}ms max={max(times):.1f}ms fails={fails}")
    print(f"  Stats: {solver.get_stats()}")
    print("=" * 65)
    print(" Test complete ✓")
    print("=" * 65)
    print("\n使用方式：")
    print("  from pinocchio_ik_solver import PinocchioIKSolver")
    print("  solver = PinocchioIKSolver('right')")
    print("  ok, joints, ms = solver.solve(target_xyz, target_quat, last_joints)")
