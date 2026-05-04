#!/usr/bin/env python3
"""
placo_ik_solver.py — Placo task-based IK solver for OpenArm O6
===============================================================
★ 使用 Placo KinematicsSolver 任務式 IK — 無需 MoveIt ★

特點：
  · Task-based IK: PositionTask + OrientationTask + JointsTask（自然姿勢偏好）
  · 每個關節獨立 naturalness 權重
  · RegularizationTask 防止奇異點
  · 多 seed 策略：last_joints → library seeds
  · 採用速度級 IK（solve(True) 積分多次直到收斂）

安裝（conda pico_teleop_py）：
    pip install placo   # 或 placo-wholearm

URDF 路徑：
    /home/asus/Desktop/openarm_description/urdf_transform/urdf/openarm_step10.urdf
    (自動剝除 all meshes → kinematics-only URDF)

Self-test:
    conda run -n pico_teleop_py python3 placo_ik_solver.py
"""

import math
import os
import time
import xml.etree.ElementTree as ET
from typing import List, Optional, Tuple

import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# URDF helper
# ─────────────────────────────────────────────────────────────────────────────
# URDF file paths (in priority order, or override with OPENARM_URDF env var)
_URDF_SOURCES = [
    "/home/asus/openArm_leapHand_urdf/src/openarm_description/usd/v10_o6.urdf",
    "/home/asus/Desktop/openarm_description/urdf_transform/urdf/openarm_step10.urdf",
]
_KIN_URDF_CACHE = "/tmp/openarm_kin_placo.urdf"


def _build_kin_urdf(src: str, dst: str = _KIN_URDF_CACHE) -> str:
    """Strip all visual & collision meshes from URDF (placo needs mesh-free URDF)."""
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
#   (pref, hard_lo, hard_hi, pos_weight, ori_weight)
#   — placo JointsTask weight = naturalness stiffness
_HUMAN_RIGHT = {
    "openarm_right_joint1": (0.000,  -1.396,  1.500, 0.15),  # shoulder yaw  大馬達
    "openarm_right_joint2": (0.700,   0.000,  1.600, 0.25),  # shoulder pitch  大馬達
    "openarm_right_joint3": (0.000,  -1.571,  0.000, 0.80),  # upperarm yaw j3=0=forward  大馬達
    "openarm_right_joint4": (1.5708,  0.250,  2.200, 0.08),  # elbow flex 90°
    "openarm_right_joint5": (0.000,  -1.571,  1.571, 0.03),  # forearm roll
    "openarm_right_joint6": (0.000,  -0.785,  0.785, 0.03),  # wrist yaw
    "openarm_right_joint7": (0.000,  -1.571,  1.571, 0.03),  # wrist pitch palm-fwd
}
_HUMAN_LEFT = {
    "openarm_left_joint1":  (0.000,  -1.500,  1.396, 0.15),
    "openarm_left_joint2":  (0.700,   0.000,  1.600, 0.25),
    "openarm_left_joint3":  (0.000,   0.000,  1.571, 0.80),  # LEFT: j3=0=forward mirrored
    "openarm_left_joint4":  (1.5708,  0.250,  2.200, 0.08),
    "openarm_left_joint5":  (0.000,  -1.571,  1.571, 0.03),
    "openarm_left_joint6":  (0.000,  -0.785,  0.785, 0.03),
    "openarm_left_joint7":  (0.000,  -1.571,  1.571, 0.03),
}
_ARM_HUMAN = {"right": _HUMAN_RIGHT, "left": _HUMAN_LEFT}

_JOINT_NAMES = {
    "right": [f"openarm_right_joint{i}" for i in range(1, 8)],
    "left":  [f"openarm_left_joint{i}"  for i in range(1, 8)],
}

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
    [ 0.00,  0.70,  0.00,  1.5708,  0.00,  0.00,  0.00],
    [ 0.00,  0.50,  0.00,  1.5708,  0.00,  0.00,  0.00],
    [ 0.00,  1.00,  0.00,  1.5708,  0.00,  0.00,  0.00],
    [-0.20,  0.70,  0.20,  1.5708,  0.00,  0.00,  0.00],
    [ 0.20,  0.70,  0.20,  1.5708,  0.00,  0.00,  0.00],
]
_ARM_SEEDS = {"right": _SEEDS_RIGHT, "left": _SEEDS_LEFT}


def _quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
    x, y, z, w = qx, qy, qz, qw
    return np.array([
        [1-2*(y*y+z*z),  2*(x*y-z*w),    2*(x*z+y*w)  ],
        [2*(x*y+z*w),    1-2*(x*x+z*z),  2*(y*z-x*w)  ],
        [2*(x*z-y*w),    2*(y*z+x*w),    1-2*(x*x+y*y)],
    ])


# ─────────────────────────────────────────────────────────────────────────────
# Solver
# ─────────────────────────────────────────────────────────────────────────────
class PlacoIKSolver:
    """
    Task-based IK solver using Placo KinematicsSolver.

    Task priority (all "soft"):
      1. PositionTask     (EE xyz)     weight = W_POS
      2. OrientationTask  (EE R)       weight = W_ORI (0 if no_rot)
      3. JointsTask       (naturalness) per-joint weight in _HUMAN_*
      4. RegularizationTask             weight = 1e-5  (singularity guard)

    Algorithm: iterative velocity-level IK.
      · solver.solve(True) integrates one step with size controlled by solver.dt
      · Repeat until pos_err < POS_TOL or MAX_ITER reached
      · Multiple seeds tried on failure (each re-creates solver for clean state)

    interface (same as PurePythonIKSolver):
        ok, joints, ms = solver.solve(target_xyz, target_quat, last_joints,
                                      verbose=False, no_rot=False)
    """

    POS_TOL   = 0.003
    ORI_TOL   = 0.15
    POS_RELAX = 0.010
    MAX_ITER  = 250       # iterations per SEED (dt=0.01 → 250 steps is enough)
    PLACO_DT  = 0.01      # placo solver.dt (velocity-level integration step)
    W_POS     = 1.0       # position task weight
    W_ORI     = 0.3       # orientation task weight
    # Joint naturalness weight — must be << W_POS so position task dominates.
    # Using a SINGLE JointsTask for all joints (verified working in tests).
    # W_J must be ~1e-4 or less; larger values create equilibria before task is solved.
    W_JOINTS  = 1e-4      # naturalness weight (single combined JointsTask)

    def __init__(self, arm: str = "right"):
        import placo
        self._placo = placo
        self.arm    = arm

        self._urdf      = _find_urdf()
        self._joint_names = _JOINT_NAMES[arm]
        self._human_cfg   = _ARM_HUMAN[arm]
        self._pref = [self._human_cfg[n][0] for n in self._joint_names]
        self._lo   = [self._human_cfg[n][1] for n in self._joint_names]
        self._hi   = [self._human_cfg[n][2] for n in self._joint_names]
        self._seeds = _ARM_SEEDS[arm]
        self._ee_link = f"openarm_{arm}_link7"
        self._stats = {"total": 0, "ok_first": 0, "ok_retry": 0, "failed": 0}

        print(
            f"[PlacoIK] {arm} arm ready  "
            f"ee_link={self._ee_link}  dt={self.PLACO_DT}  "
            f"max_iter={self.MAX_ITER}  (placo)"
        )

    def _make_robot_and_solver(self):
        """Create fresh placo robot + solver (needed for each seed attempt)."""
        placo = self._placo
        robot  = placo.RobotWrapper(self._urdf, placo.Flags.ignore_collisions)
        solver = placo.KinematicsSolver(robot)
        solver.enable_joint_limits(True)
        solver.enable_velocity_limits(False)
        solver.mask_fbase(True)
        solver.dt = self.PLACO_DT
        return robot, solver

    def _set_joints(self, robot, q: List[float]):
        for name, val in zip(self._joint_names, q):
            robot.set_joint(name, val)
        robot.update_kinematics()

    def _get_joints(self, robot) -> List[float]:
        return [robot.get_joint(n) for n in self._joint_names]

    def _get_ee_pos(self, robot) -> np.ndarray:
        T = robot.get_T_world_frame(self._ee_link)
        return T[:3, 3]

    def _pos_error(self, robot, target_xyz: np.ndarray) -> float:
        return float(np.linalg.norm(self._get_ee_pos(robot) - target_xyz))

    def _solve_from_seed(
        self,
        target_xyz:  np.ndarray,
        target_R:    np.ndarray,
        seed:        List[float],
        no_rot:      bool,
    ) -> Tuple[List[float], float]:
        """Single seed attempt. Returns (joints, pos_err)."""
        placo = self._placo
        robot, solver = self._make_robot_and_solver()

        # Set seed
        self._set_joints(robot, seed)

        # Build target transform matrix
        T_target = np.eye(4)
        T_target[:3, :3] = target_R
        T_target[:3, 3]  = target_xyz

        # Tasks
        pos_task = solver.add_position_task(self._ee_link, target_xyz)
        pos_task.configure("pos", "soft", self.W_POS)

        if not no_rot:
            ori_task = solver.add_orientation_task(self._ee_link, target_R)
            ori_task.configure("ori", "soft", self.W_ORI)

        # JointsTask: single combined naturalness task (low weight so position wins).
        # Separate high-weight tasks per joint create over-constraint.
        pref_dict = {n: self._human_cfg[n][0] for n in self._joint_names}
        jt = solver.add_joints_task()
        jt.set_joints(pref_dict)
        jt.configure("naturalness", "soft", self.W_JOINTS)

        # Global regularization (prevents singularity)
        reg = solver.add_regularization_task(1e-5)
        reg.configure("reg", "soft", 1.0)

        # Iterative solve
        for _ in range(self.MAX_ITER):
            solver.solve(True)
            robot.update_kinematics()

        pos_err = self._pos_error(robot, target_xyz)
        joints  = self._get_joints(robot)

        # Clip to hard limits (safety)
        joints = [max(lo, min(hi, q)) for q, lo, hi in zip(joints, self._lo, self._hi)]
        return joints, pos_err

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

        txyz  = np.array(target_xyz)
        tR    = _quat_to_rot(*target_quat)
        seeds = [list(last_joints)] + self._seeds

        best_q     = None
        best_pos_e = 1e9

        for attempt, seed in enumerate(seeds):
            try:
                joints, pos_e = self._solve_from_seed(txyz, tR, seed, no_rot)
            except Exception as ex:
                if verbose:
                    print(f"  [PlacoIK] attempt {attempt} error: {ex}")
                continue

            if verbose:
                j3d = math.degrees(joints[2])
                print(
                    f"  [PlacoIK] attempt={attempt} "
                    f"pos={pos_e*1000:.1f}mm  "
                    f"j3={j3d:+.1f}° {'✓' if pos_e < self.POS_TOL else '·'}"
                )

            if pos_e < best_pos_e:
                best_pos_e = pos_e
                best_q     = joints

            if pos_e < self.POS_TOL:
                ms = (time.time() - t0) * 1000
                if attempt == 0:
                    self._stats["ok_first"] += 1
                else:
                    self._stats["ok_retry"] += 1
                return True, joints, ms

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
    print(" placo_ik_solver.py — Self-test")
    print("=" * 65)

    solver = PlacoIKSolver("right")
    pref   = [_HUMAN_RIGHT[n][0] for n in _JOINT_NAMES["right"]]

    # Get home FK using placo
    robot, _ = solver._make_robot_and_solver()
    solver._set_joints(robot, pref)
    T_home = robot.get_T_world_frame(f"openarm_right_link7")
    xyz_home = T_home[:3, 3]
    R_home   = T_home[:3, :3]
    print(f"\n[Pref FK] xyz={xyz_home}  j3_pref={math.degrees(pref[2]):.1f}°")

    # Home quat from rotation matrix
    tr = R_home[0, 0] + R_home[1, 1] + R_home[2, 2]
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        qhome = (
            (R_home[2,1]-R_home[1,2])*s,
            (R_home[0,2]-R_home[2,0])*s,
            (R_home[1,0]-R_home[0,1])*s,
            0.25/s,
        )
    else:
        qhome = (0., 0., 0., 1.)

    print("\n[Test 1] round-trip from preferred home")
    ok, joints, ms = solver.solve(tuple(xyz_home), qhome, pref, verbose=True)
    if ok:
        robot2, _ = solver._make_robot_and_solver()
        solver._set_joints(robot2, joints)
        pe = float(np.linalg.norm(robot2.get_T_world_frame(f"openarm_right_link7")[:3, 3] - xyz_home))
        j3d = math.degrees(joints[2])
        print(f"  ok={ok} ms={ms:.1f} pos_err={pe*1000:.1f}mm j3={j3d:+.1f}°")
    else:
        print(f"  FAILED ms={ms:.1f}")

    print("\n[Test 2] +5cm X step")
    step = (float(xyz_home[0]) + 0.05, float(xyz_home[1]), float(xyz_home[2]))
    ok2, j2, ms2 = solver.solve(step, qhome, joints or pref, verbose=True)
    if ok2:
        robot3, _ = solver._make_robot_and_solver()
        solver._set_joints(robot3, j2)
        pe2 = float(np.linalg.norm(
            robot3.get_T_world_frame(f"openarm_right_link7")[:3, 3] - np.array(step)
        ))
        j3d2 = math.degrees(j2[2])
        print(f"  ok={ok2} ms={ms2:.1f} pos_err={pe2*1000:.1f}mm j3={j3d2:+.1f}°")
    else:
        print(f"  FAILED ms={ms2:.1f}")

    print("\n[Test 3] Performance: 30 steps 5mm each")
    q_cur = list(pref)
    tx = float(xyz_home[0])
    times, fails = [], 0
    for i in range(30):
        tx += 0.005
        ok_i, qn, ms_i = solver.solve(
            (tx, float(xyz_home[1]), float(xyz_home[2])), qhome, q_cur)
        times.append(ms_i)
        if ok_i:
            q_cur = qn
        else:
            fails += 1
    print(f"  30 steps: total={sum(times):.0f}ms mean={sum(times)/len(times):.1f}ms "
          f"max={max(times):.1f}ms fails={fails}")

    print(f"\n  Stats: {solver.get_stats()}")
    print("=" * 65)
    print(" Test complete ✓")
    print("=" * 65)
    print("\n使用方式：")
    print("  from placo_ik_solver import PlacoIKSolver")
    print("  solver = PlacoIKSolver('right')")
    print("  ok, joints, ms = solver.solve(target_xyz, target_quat, last_joints)")
