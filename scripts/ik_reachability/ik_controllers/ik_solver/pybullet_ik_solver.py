#!/usr/bin/env python3
"""
pybullet_ik_solver.py — PyBullet-based IK solver for OpenArm O6
================================================================
★ 使用 PyBullet DIRECT (headless) 模式解算 IK — 無需 MoveIt ★

特點：
  · PyBullet 內建 DLS IK (Damped Least-Squares)
  · restPoses  = 人體類似首選姿勢（elbow-down, arm-forward）
  · jointDamping = 每個關節的阻尼（等同 naturalness 權重）
  · URDF 關節限制自動套用
  · 多 seed 策略：last_joints → library seeds

安裝（conda pico_teleop_py）：
    pip install pybullet

URDF 路徑（自動搜尋，或設定環境變數 OPENARM_URDF）：
    /home/asus/Desktop/openarm_description/urdf_transform/urdf/openarm_step10.urdf
    (自動剝除 collision/visual meshes 以便載入)

Self-test:
    conda run -n pico_teleop_py python3 pybullet_ik_solver.py
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
_KIN_URDF_CACHE = "/tmp/openarm_kin_pybullet.urdf"


def _build_kin_urdf(src_urdf: str, dst_urdf: str = _KIN_URDF_CACHE) -> str:
    """Strip all visual/collision meshes from URDF for PyBullet loading."""
    tree = ET.parse(src_urdf)
    root = tree.getroot()
    for link in root.findall("link"):
        for tag in ("collision", "visual"):
            for el in list(link.findall(tag)):
                link.remove(el)
    tree.write(dst_urdf, xml_declaration=True, encoding="unicode")
    return dst_urdf


def _find_urdf() -> str:
    """
    Resolve URDF (priority):
      1. OPENARM_URDF env var (file path)
      2. Local file paths in _URDF_SOURCES
    Returns path to mesh-stripped kinematics-only URDF.
    """
    # 1. Env var
    env = os.environ.get("OPENARM_URDF")
    if env and os.path.isfile(env):
        print(f"[URDF] Using OPENARM_URDF env: {env}")
        return _build_kin_urdf(env)

    # 2. Local file paths
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
# preferred angles = "arm hanging ready, elbow bent slightly"
# damping values = naturalness weight (higher = stronger pull to pref)
_HUMAN_CFG = {
    "right": {
        # joint  pref    damping  hard_lo  hard_hi
        "j1": ( 0.000,  0.15,  -1.396,  1.500),  # base-yaw: face forward  大馬達
        "j2": ( 0.700,  0.20,   0.000,  1.600),  # shoulder: slightly raised  大馬達
        "j3": ( 0.000,  0.60,  -1.571,  0.300),  # upperarm yaw: j3=0 forward  大馬達
        "j4": ( 1.5708, 0.05,   0.250,  2.200),  # elbow: 90° forward-reach
        "j5": ( 0.000,  0.02,  -1.571,  1.571),  # forearm roll: neutral
        "j6": ( 0.000,  0.02,  -0.785,  0.785),  # wrist yaw: straight
        "j7": ( 0.000,  0.04,  -1.571,  1.571),  # wrist pitch: palm forward
    },
    "left": {
        # pref=-0.7 for j2: LEFT arm URDF is mirrored (j2_left=-j2_right)
        # Safety limits mirrored from RIGHT: j1[-1.500,+1.396] j2[-1.600,0.000] j3[-0.300,+1.571] j4 same
        "j1": ( 0.000,  0.15,  -1.500,  1.396),  # mirror RIGHT [-1.396,+1.500]
        "j2": (-0.700,  0.20,  -1.600,  0.000),  # mirror RIGHT [0.000,+1.600]
        "j3": ( 0.000,  0.60,  -0.300,  1.571),  # mirror RIGHT [-1.571,+0.300]
        "j4": ( 1.5708, 0.05,   0.250,  2.200),  # same as right
        "j5": ( 0.000,  0.02,  -1.571,  1.571),  # forearm roll: neutral
        "j6": ( 0.000,  0.02,  -0.785,  0.785),  # wrist yaw
        "j7": ( 0.000,  0.04,  -1.571,  1.571),  # wrist pitch  axis=0 -1 0
    },
}

# Library seeds for recovery (right arm, human-like)
_SEEDS_RIGHT = [
    [0.00, 0.70,  0.00, 1.5708, 0.00,  0.00, 0.00],  # forward-reach home 90°
    [0.00, 0.50,  0.00, 1.5708, 0.00,  0.00, 0.00],
    [0.00, 1.00,  0.00, 1.5708, 0.00,  0.00, 0.00],
    [0.20, 0.70, -0.20, 1.5708, 0.00,  0.00, 0.00],
    [-0.20, 0.70,-0.20, 1.5708, 0.00,  0.00, 0.00],
    [0.00, 0.80,  0.00, 2.0000, 0.00,  0.00, 0.00],
    [0.00, 0.60,  0.00, 1.2000, 0.00,  0.30, 0.00],
]
_SEEDS_LEFT = [
    # j2 must be NEGATIVE for left arm (URDF mirror: j2_left=-j2_right)
    [0.00, -0.70,  0.00, 1.5708, 0.00,  0.00, 0.00],  # forward-reach home 90°
    [0.00, -0.50,  0.00, 1.5708, 0.00,  0.00, 0.00],  # shoulder lower
    [0.00, -1.00,  0.00, 1.5708, 0.00,  0.00, 0.00],  # shoulder higher
    [-0.20, -0.70, 0.20, 1.5708, 0.00,  0.00, 0.00],  # j1 inward
    [0.20, -0.70,  0.20, 1.5708, 0.00,  0.00, 0.00],  # j1 outward
    [0.00, -0.80,  0.00, 2.0000, 0.00,  0.00, 0.00],  # elbow ~115°
    [0.00, -0.60,  0.00, 1.2000, 0.00,  0.30, 0.00],  # forearm roll
]
_ARM_SEEDS = {"right": _SEEDS_RIGHT, "left": _SEEDS_LEFT}


# ─────────────────────────────────────────────────────────────────────────────
# Solver
# ─────────────────────────────────────────────────────────────────────────────
class PybulletIKSolver:
    """
    IK solver using PyBullet's built-in DLS IK (calculateInverseKinematics).

    Per-joint naturalness is encoded via:
      · restPoses   — preferred joint angles (home bias)
      · jointDamping — resists deviation from restPoses (higher = stronger)

    Hard joint limits come from the URDF (enforced by PyBullet's IK).

    interface (same as PurePythonIKSolver):
        ok, joints, ms = solver.solve(target_xyz, target_quat, last_joints,
                                      verbose=False, no_rot=False)
    """

    POS_TOL   = 0.003   # m — accept if position error < 3mm
    ORI_TOL   = 0.15    # rad — accept if orientation error < ~9°
    POS_RELAX = 0.010   # m — relaxed accept for best candidate

    def __init__(self, arm: str = "right"):
        import pybullet as p
        self._p = p
        self.arm = arm

        self._urdf = _find_urdf()
        self._client = p.connect(p.DIRECT)

        self._robot = p.loadURDF(
            self._urdf, useFixedBase=True,
            physicsClientId=self._client,
        )

        # Build joint name → pybullet index map
        nj = p.getNumJoints(self._robot, physicsClientId=self._client)
        self._joint_map: dict = {}
        for i in range(nj):
            ji = p.getJointInfo(self._robot, i, physicsClientId=self._client)
            jname = ji[1].decode()
            self._joint_map[jname] = i

        # Arm-specific joint names and human config
        cfg = _HUMAN_CFG[arm]
        self._arm_joint_names = [f"openarm_{arm}_joint{k}" for k in range(1, 8)]
        self._arm_indices     = [self._joint_map[n] for n in self._arm_joint_names]
        self._pref_q          = [cfg[f"j{k}"][0] for k in range(1, 8)]
        self._damping         = [cfg[f"j{k}"][1] for k in range(1, 8)]
        self._hard_lo         = [cfg[f"j{k}"][2] for k in range(1, 8)]
        self._hard_hi         = [cfg[f"j{k}"][3] for k in range(1, 8)]

        # EE link = index of last joint's child link
        self._ee_link = self._arm_indices[-1]  # openarm_{arm}_joint7 → link7

        # Build full-robot restPoses (for all movable joints)
        # PyBullet IK needs restPoses for ALL movable DOF when providing lowerLimits
        self._all_movable = []  # ordered movable joint indices
        for i in range(nj):
            ji = p.getJointInfo(self._robot, i, physicsClientId=self._client)
            if ji[2] != p.JOINT_FIXED:  # not fixed
                self._all_movable.append(i)

        self._seeds = _ARM_SEEDS[arm]
        self._stats = {"total": 0, "ok_first": 0, "ok_retry": 0, "failed": 0}

        print(
            f"[PybulletIK] {arm} arm ready  "
            f"ee_link={self._ee_link}  movable_dof={len(self._all_movable)}  "
            f"arm_joints={self._arm_indices}"
        )

    def _build_full_rest_poses(self, arm_q: List[float]) -> List[float]:
        """Build restPoses array for all movable joints (other arm stays at pref)."""
        other_arm = "left" if self.arm == "right" else "right"
        other_pref = [_HUMAN_CFG[other_arm][f"j{k}"][0] for k in range(1, 8)]
        other_names = [f"openarm_{other_arm}_joint{k}" for k in range(1, 8)]

        rest = []
        arm_q_map  = dict(zip(self._arm_joint_names, arm_q))
        other_q_map = dict(zip(other_names, other_pref))
        for idx in self._all_movable:
            ji   = self._p.getJointInfo(self._robot, idx, physicsClientId=self._client)
            name = ji[1].decode()
            if name in arm_q_map:
                rest.append(arm_q_map[name])
            elif name in other_q_map:
                rest.append(other_q_map[name])
            else:
                rest.append(0.0)
        return rest

    def _set_arm_joints(self, q: List[float]):
        for idx, qi in zip(self._arm_indices, q):
            self._p.resetJointState(
                self._robot, idx, qi, physicsClientId=self._client
            )

    def _get_ee_pose(self):
        """Return EE (xyz, quat_xyzw) from current joint state."""
        ls = self._p.getLinkState(
            self._robot, self._ee_link,
            computeForwardKinematics=True,
            physicsClientId=self._client,
        )
        return ls[4], ls[5]  # worldLinkFramePos, worldLinkFrameOrn

    def _pos_error(self, target_xyz) -> float:
        pos, _ = self._get_ee_pose()
        return float(np.linalg.norm(np.array(pos) - np.array(target_xyz)))

    def _ori_error(self, target_qxyzw) -> float:
        _, q_cur = self._get_ee_pose()
        dot = abs(sum(a * b for a, b in zip(q_cur, target_qxyzw)))
        dot = min(1.0, dot)
        return 2.0 * math.acos(dot)

    def _ik_once(self, target_xyz, target_qxyzw, seed_q: List[float],
                 no_rot: bool) -> List[float]:
        """Run PyBullet IK from seed_q."""
        self._set_arm_joints(seed_q)
        rest = self._build_full_rest_poses(self._pref_q)
        damping = []
        for idx in self._all_movable:
            ji   = self._p.getJointInfo(self._robot, idx, physicsClientId=self._client)
            name = ji[1].decode()
            if name in dict(zip(self._arm_joint_names, range(7))):
                k = self._arm_joint_names.index(name)
                damping.append(self._damping[k])
            else:
                damping.append(0.1)

        ori = None if no_rot else target_qxyzw
        result = self._p.calculateInverseKinematics(
            self._robot,
            self._ee_link,
            target_xyz,
            targetOrientation=ori,
            restPoses=rest,
            jointDamping=damping,
            maxNumIterations=200,
            residualThreshold=1e-5,
            physicsClientId=self._client,
        )
        # Extract arm joints from full-robot result
        arm_result = []
        for arm_idx in self._arm_indices:
            movable_pos = self._all_movable.index(arm_idx)
            arm_result.append(float(result[movable_pos]))

        # Clip to hard bounds
        arm_result = [
            max(lo, min(hi, q))
            for q, lo, hi in zip(arm_result, self._hard_lo, self._hard_hi)
        ]
        return arm_result

    def solve(
        self,
        target_xyz:  Tuple[float, float, float],
        target_quat: Tuple[float, float, float, float],
        last_joints: List[float],
        verbose:     bool = False,
        no_rot:      bool = False,
    ) -> Tuple[bool, Optional[List[float]], float]:
        """
        Returns (ok, joints, elapsed_ms).
        target_quat: (qx, qy, qz, qw)
        """
        t0 = time.time()
        self._stats["total"] += 1

        seeds = [list(last_joints)] + self._seeds

        best_joints  = None
        best_pos_err = 1e9

        for attempt, seed in enumerate(seeds):
            try:
                j_result = self._ik_once(target_xyz, target_quat, seed, no_rot)
            except Exception as e:
                if verbose:
                    print(f"  [BulletIK] attempt {attempt} error: {e}")
                continue

            # Measure FK error
            self._set_arm_joints(j_result)
            pos_err = self._pos_error(target_xyz)
            ori_err = self._ori_error(target_quat) if not no_rot else 0.0

            if verbose:
                j3d = math.degrees(j_result[2])
                print(
                    f"  [BulletIK] attempt={attempt} "
                    f"pos={pos_err*1000:.1f}mm ori={math.degrees(ori_err):.1f}° "
                    f"j3={j3d:+.1f}° {'✓' if pos_err < self.POS_TOL else '·'}"
                )

            if pos_err < best_pos_err:
                best_pos_err = pos_err
                best_joints  = j_result

            if pos_err < self.POS_TOL and (no_rot or ori_err < self.ORI_TOL):
                ms = (time.time() - t0) * 1000
                if attempt == 0:
                    self._stats["ok_first"] += 1
                else:
                    self._stats["ok_retry"] += 1
                return True, j_result, ms

        ms = (time.time() - t0) * 1000
        if best_joints is not None and best_pos_err < self.POS_RELAX:
            self._stats["ok_retry"] += 1
            return True, best_joints, ms

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

    def __del__(self):
        try:
            self._p.disconnect(self._client)
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────────────────
# Self-test
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 65)
    print(" pybullet_ik_solver.py — Self-test")
    print("=" * 65)

    solver = PybulletIKSolver("right")
    pref   = [_HUMAN_CFG["right"][f"j{k}"][0] for k in range(1, 8)]

    # Test 1: solve from preferred pose (round-trip)
    solver._set_arm_joints(pref)
    home_pos, home_q = solver._get_ee_pose()
    print(f"\n[Pref FK] xyz={home_pos}  quat={home_q}")

    print("\n[Test 1] IK round-trip from preferred home")
    ok, joints, ms = solver.solve(home_pos, home_q, pref, verbose=True)
    print(f"  ok={ok} ms={ms:.1f}")
    if ok:
        pe = solver._pos_error(home_pos)
        j3d = math.degrees(joints[2])
        print(f"  pos_err={pe*1000:.1f}mm  j3={j3d:+.1f}° (elbow: {'down ✓' if j3d < 1 else 'UP ✗'})")

    # Test 2: 5cm step in X
    step_target = (home_pos[0] + 0.05, home_pos[1], home_pos[2])
    print(f"\n[Test 2] +5cm X step: target={[f'{v:.3f}' for v in step_target]}")
    ok2, joints2, ms2 = solver.solve(step_target, home_q, joints or pref, verbose=True)
    if ok2:
        pe2 = solver._pos_error(step_target)
        j3d2 = math.degrees(joints2[2])
        print(f"  ok={ok2} ms={ms2:.1f} pos_err={pe2*1000:.1f}mm j3={j3d2:+.1f}°")

    # Test 3: Performance
    print(f"\n[Test 3] Performance: 30 steps (5mm each)")
    q_cur = list(pref)
    t0 = time.time()
    times = []
    fails = 0
    tx = home_pos[0]
    for i in range(30):
        tx += 0.005
        ok_i, q_new, ms_i = solver.solve((tx, home_pos[1], home_pos[2]), home_q, q_cur)
        times.append(ms_i)
        if ok_i:
            q_cur = q_new
        else:
            fails += 1
    total = (time.time() - t0) * 1000
    print(f"  30 steps: total={total:.0f}ms mean={sum(times)/len(times):.1f}ms "
          f"max={max(times):.1f}ms fails={fails}")

    print(f"\n  Stats: {solver.get_stats()}")
    print("=" * 65)
    print(" Test complete ✓")
    print("=" * 65)
    print("\n使用方式：")
    print("  from pybullet_ik_solver import PybulletIKSolver")
    print("  solver = PybulletIKSolver('right')")
    print("  ok, joints, ms = solver.solve(target_xyz, target_quat, last_joints)")
