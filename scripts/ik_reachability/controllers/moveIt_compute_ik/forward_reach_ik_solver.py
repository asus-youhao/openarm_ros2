#!/usr/bin/env python3
"""
forward_reach_ik_solver.py — Forward-reach-only IK for OpenArm O6
==================================================================
★ 手臂只允許向前抓取，完全排除「抓背」/「手臂後扭」類型的 IK 解 ★

目的：
    NaturalIKSolver 已防止肘部朝上（j3 < 0.3 rad）。
    但 7-DOF null-space 仍允許手臂向後伸展（j1 極度負值）+
    j3 旋轉組合 → 機器手臂做出「人體抓背」式的後扭姿勢。

    此 Solver 進一步限縮到「只在身體前方抓取」：
        · j1 (肩膀前後)  hard [-0.50, 1.30] — 防止手臂大幅後擺（抓背起點）
        · j2 (肩膀飛鳥)  hard [0.00, 1.60]  — 防止手臂夾向身體側面
        · j3 (上臂旋轉)  hard [-1.571, 0.00] — 肘部嚴格朝下，比 NaturalIK +0.30 更嚴
        · j4 (手肘彎曲)  hard [0.25, 2.20]  — 防止近乎伸直（配合 j2 大時肘部會高）
        · MAX_RETRIES = 8 — 解空間更小，允許更多嘗試

與 NaturalIKSolver 的差異對照表：
    ┌──────┬────────────────────────────┬─────────────────────────────────────┐
    │ 關節 │ NaturalIKSolver  hard      │ ForwardReachIKSolver  hard          │
    ├──────┼────────────────────────────┼─────────────────────────────────────┤
    │ j1   │ [-1.396,  3.491]           │ [-0.50,  1.30]  ← 防止抓背         │
    │ j2   │ [-0.30,   1.60]            │ [0.00,   1.60]  ← 防止夾身         │
    │ j3   │ [-1.571,  0.30]            │ [-1.571, 0.00]  ← 肘部嚴格不超水平 │
    │      │  soft_hi = 0.0             │  soft_hi = -0.20 ← 更早懲罰        │
    │      │  weight = 4.0              │  weight = 6.0   ← 更嚴格           │
    │ j4   │ [0.20,   2.30]             │ [0.25,   2.20]  ← 略收緊           │
    │ j5   │ [-1.571, 1.571]            │ 同 NaturalIK                        │
    │ j6   │ [-0.785, 0.785]            │ 同 NaturalIK                        │
    │ j7   │ [-1.00,  1.00]             │ 同 NaturalIK                        │
    └──────┴────────────────────────────┴─────────────────────────────────────┘

為什麼 j3 旋轉會造成「抓背」：
    j3 是 7-DOF null-space 的主要自由度（上臂旋轉 / 二頭旋轉）。
    j3 = -1.57 rad (-90°) → 肘部朝下 (home，最自然)        ✓
    j3 =  0.00 rad  (0°)  → 肘部朝側（水平）               →
    j3 = +1.57 rad (+90°) → 肘部朝上                       ← NaturalIK 拒絕
    NaturalIK hard_hi = +0.30 (允許到 17° 上)
    ForwardReach hard_hi = 0.00 (不允許任何朝上）

    此外，當 IK 找不到「肘部朝下 + 手臂向前」的解時，
    KDL solver 可能退回到「j3 負、j1 也負 → 手臂後扭抓背姿勢」。
    → 限制 j1 hard_lo = -0.50 rad 可有效封鎖此退路。

Drop-in replacement for NaturalIKSolver:
    # 只改這兩行即可：
    from forward_reach_ik_solver import ForwardReachIKSolver
    self._natural_ik = ForwardReachIKSolver(arm=args.arm, node=self)

Dependencies:
    · natural_ik_solver.py 必須在相同目錄（提供 NaturalIKSolver, JointCfg, URDF_LIMITS）
"""

import math
import random
from typing import List

from natural_ik_solver import JointCfg, NaturalIKSolver, URDF_LIMITS


# ─────────────────────────────────────────────────────────────────────────────
# Forward-reach-only naturalness configs
# ─────────────────────────────────────────────────────────────────────────────

# ── Right Arm ────────────────────────────────────────────────────────────────
_FWD_RIGHT_CFG: List[JointCfg] = [
    #              pref       soft_lo  soft_hi   hard_lo  hard_hi  weight  name

    # j1: shoulder flex/ext (前後擺)
    #   hard_lo = -0.50 rad (-28°): 防止手臂向後擺（抓背動作的起點）
    #   hard_hi = +1.30 rad (+74°): 正常前向抓取不需要超過此範圍
    JointCfg( 0.0000,  -0.4000,  0.9000,  -0.5000,  1.3000, 1.5, "j1_shoulder_flex_ext"),

    # j2: shoulder abduction (飛鳥 側向)
    #   hard_lo = 0.00 rad: 手臂不得收到身體內側（防止「抱住自己」配置）
    JointCfg( 1.2735,   0.3000,  1.5000,   0.0000,  1.6000, 2.5, "j2_shoulder_abduction"),

    # j3: upper arm rotation / elbow swivel ★ 最核心的前向限制 ★
    #   这是阻止「抓背」的關鍵：
    #   NaturalIK 允許到 +0.30 rad (+17°，肘略朝上)
    #   ForwardReach 嚴格限制到 0.00 rad (肘水平即已至上限）
    #   soft_hi = -0.20 rad: 比 0 更早開始懲罰（防止解趨近硬限）
    #   weight = 6.0 (vs NaturalIK 4.0): 更強的懲罰力道
    # hard_lo = -1.5800: home j3=-1.5708 境界，KDL 微小負偏差也不會被拒絕
    JointCfg(-1.5708,  -1.5708, -0.2000,  -1.5800,  0.0000, 6.0, "j3_upperarm_rot_ELBOW"),

    # j4: elbow flexion (手肘彎曲)
    #   hard_lo = 0.25: 手臂不得近乎伸直（伸直 + j2 大 → 肘位置很高）
    JointCfg( 1.8287,   0.5000,  2.1000,   0.2500,  2.2000, 2.0, "j4_elbow_flex"),

    # j5: forearm roll (前臂旋轉). 同 NaturalIK
    JointCfg( 1.5707,   0.3000,  2.5000,  -1.5708,  1.5708, 0.8, "j5_forearm_roll"),

    # j6: wrist yaw / left-right (手腕左右). 同 NaturalIK
    JointCfg(-0.5552,  -0.7854,  0.4000,  -0.7854,  0.7854, 0.8, "j6_wrist_yaw_lr"),

    # j7: wrist pitch / up-down (手腕上下). 同 NaturalIK, ±1.0 rad
    JointCfg( 0.0000,  -0.9000,  0.9000,  -1.0000,  1.0000, 0.4, "j7_wrist_pitch_ud"),
]

# ── Left Arm (mirrored) ───────────────────────────────────────────────────────
# Left arm: rpy="-1.5708 0 0" → arm Z = world +Y (mirrored from right)
#   j1: forward reach on left side → j1 slightly negative is "forward"
#       hard: [-1.30, 0.50] — mirrored from right arm's [-0.50, 1.30]
#   j3: elbow-down = POSITIVE (+1.5708) on left arm (mirrored)
#       hard_lo = 0.00 rad: j3 must stay positive (elbow cannot point up / backward)
_FWD_LEFT_CFG: List[JointCfg] = [
    JointCfg( 0.0000,  -0.9000,  0.4000,  -1.3000,  0.5000, 1.5, "j1_shoulder_flex_ext"),
    JointCfg( 1.2735,   0.3000,  1.5000,   0.0000,  1.6000, 2.5, "j2_shoulder_abduction"),
    # 左臂 j3 鏡像：elbow-down = +1.5708（正值）
    # hard_lo = 0.00: j3 宇格不得為負值（向負方向 = 肘部朝上 / 後我）
    # hard_hi = +1.5800 → 比 home(+1.5708) 再寬 0.009 的緩衝
    JointCfg( 1.5708,   0.2000,  1.5708,   0.0000,  1.5800, 6.0, "j3_upperarm_rot_ELBOW"),
    JointCfg( 1.8287,   0.5000,  2.1000,   0.2500,  2.2000, 2.0, "j4_elbow_flex"),
    JointCfg( 1.5707,   0.3000,  2.5000,  -1.5708,  1.5708, 0.8, "j5_forearm_roll"),
    JointCfg(-0.5552,  -0.7854,  0.4000,  -0.7854,  0.7854, 0.8, "j6_wrist_yaw_lr"),
    JointCfg( 0.0000,  -0.9000,  0.9000,  -1.0000,  1.0000, 0.4, "j7_wrist_pitch_ud"),
]

_FWD_ARM_CFGS = {"right": _FWD_RIGHT_CFG, "left": _FWD_LEFT_CFG}


# ─────────────────────────────────────────────────────────────────────────────
# Forward-reach seed library
#
# 所有 seed 都是「手臂朝前、肘部嚴格朝下」的配置。
# j1 保持在 [-0.3, 0.5]（手臂指向正前方或略側），
# j3 固定在 -1.5708（肘部完全朝下），
# 沒有任何「手臂後扭」或「肘部抬起」的 seed。
# ─────────────────────────────────────────────────────────────────────────────
_FWD_SEEDS_RIGHT = [
    # j1      j2      j3       j4      j5       j6       j7     # 描述
    [0.0000,  1.2735, -1.5707,  1.8287,  1.5707, -0.5552,  0.0],  # home ✓
    [0.0000,  1.0000, -1.5707,  1.5000,  1.5707, -0.3000,  0.0],  # arm lower-forward
    [0.0000,  1.5000, -1.5707,  1.5000,  1.5707,   -0.7800,  0.0],  # arm higher-forward
    [0.2000,  1.2000, -1.5707,  1.7000,  1.5707, -0.4000,  0.0],  # slight-right forward
    [-0.200,  1.2000, -1.5707,  1.7000,  1.5707, -0.4000,  0.0],  # slight-left forward
    [0.0000,  0.8000, -1.5707,  2.0000,  1.5707,  0.0000,  0.0],  # arm raised forward
    [0.0000,  1.4000, -1.5707,  1.3000,  1.5707, -0.6000,  0.0],  # reach down-front
    [0.3000,  1.2000, -1.5707,  1.8000,  1.3000, -0.4000,  0.1],  # reach right-forward
    [-0.300,  1.2000, -1.5707,  1.8000,  1.5707, -0.4000, -0.1],  # reach left-forward
    [0.0000,  1.3000, -1.4000,  2.0000,  1.5707, -0.5000,  0.0],  # slightly elevated elbow (compromise)
]

_FWD_SEEDS_LEFT = [
    [0.0000,  1.2735,  1.5707,  1.8287,  1.5707, -0.5552,  0.0],  # home ✓
    [0.0000,  1.0000,  1.5707,  1.5000,  1.5707, -0.3000,  0.0],
    [0.0000,  1.5000,  1.5707,  1.5000,  1.5707,   -0.7800,  0.0],
    [-0.200,  1.2000,  1.5707,  1.7000,  1.5707, -0.4000,  0.0],
    [0.2000,  1.2000,  1.5707,  1.7000,  1.5707, -0.4000,  0.0],
    [0.0000,  0.8000,  1.5707,  2.0000,  1.5707,  0.0000,  0.0],
    [0.0000,  1.4000,  1.5707,  1.3000,  1.5707, -0.6000,  0.0],
    [-0.300,  1.2000,  1.5707,  1.8000,  1.3000, -0.4000, -0.1],
    [0.3000,  1.2000,  1.5707,  1.8000,  1.5707, -0.4000,  0.1],
    [0.0000,  1.3000,  1.4000,  2.0000,  1.5707, -0.5000,  0.0],
]

_FWD_SEEDS = {"right": _FWD_SEEDS_RIGHT, "left": _FWD_SEEDS_LEFT}


# ─────────────────────────────────────────────────────────────────────────────
# ForwardReachIKSolver
# ─────────────────────────────────────────────────────────────────────────────
class ForwardReachIKSolver(NaturalIKSolver):
    """
    手臂前向抓取專用 IK solver。排除所有「抓背」/「手臂後扭」配置。

    繼承 NaturalIKSolver 的全部 call_natural() / score() / violates_hard() 邏輯，
    只替換：
        · _cfg : 更嚴格的 JointCfg 約束（見模組頂部的對照表）
        · _seeds: 只有「手臂朝前、肘部嚴格朝下」的 seed 庫
        · perturb_seeds(): j3 的微擾上界夾緊到 0.0（右臂）/ 0.0 下界（左臂）
        · MAX_RETRIES = 8: 解空間更小，允許更多嘗試

    Drop-in replacement for NaturalIKSolver:
        from forward_reach_ik_solver import ForwardReachIKSolver
        self._natural_ik = ForwardReachIKSolver(arm=args.arm, node=self)
    """

    # 更多重試次數，因為解空間比 NaturalIK 更小
    MAX_RETRIES = 8

    def __init__(self, arm: str = "right", node=None):
        # 先呼叫 parent 初始化（設置 self.arm, self.node, self._lock, self._stats）
        super().__init__(arm=arm, node=node)

        # 覆蓋成前向限制版本的 cfg 和 seeds
        self._cfg = _FWD_ARM_CFGS[arm]
        self._seeds = [list(s) for s in _FWD_SEEDS[arm]]

        msg = (
            f"[ForwardReachIK] initialized for {arm} arm  "
            f"j1 hard=[{self._cfg[0].hard_lo:.2f},{self._cfg[0].hard_hi:.2f}]  "
            f"j3 hard=[{self._cfg[2].hard_lo:.2f},{self._cfg[2].hard_hi:.2f}]"
        )
        if node:
            node.get_logger().info(msg)
        else:
            print(msg)

    def perturb_seeds(
        self,
        current_joints: List[float],
        n: int = 4,
        sigma: float = 0.20,   # 比 NaturalIK 的 0.25 更小，避免擾動越界
        rng: random.Random = None,
    ) -> List[List[float]]:
        """
        覆寫 perturb_seeds：
        · 比 NaturalIK 用更小的 sigma（0.20 vs 0.25）—— 解空間更小
        · j1 擾動後夾緊到 hard 限制 [-0.50, 1.30] / [-1.30, 0.50]
        · j3 擾動後額外夾緊：右臂上界 0.0, 左臂下界 0.0（排除後扭方向）
        """
        if rng is None:
            rng = random

        # 使用 ForwardReach 的 hard 限制來 clamp，而非 URDF 原始限制
        # j1 hard limits
        j1_lo = self._cfg[0].hard_lo   # right: -0.50, left: -1.30
        j1_hi = self._cfg[0].hard_hi   # right: +1.30, left: +0.50
        # j3 hard limits
        j3_lo = self._cfg[2].hard_lo   # both: -1.5708 or 0.0
        j3_hi = self._cfg[2].hard_hi   # right:  0.0   / left: 1.5708

        # 其餘關節仍用 URDF limits
        lo_urdf = [URDF_LIMITS[i][0] for i in range(7)]
        hi_urdf = [URDF_LIMITS[i][1] for i in range(7)]

        seeds = []
        bias_j3 = -0.3 if self.arm == "right" else +0.3  # 偏向 elbow-down

        for _ in range(n):
            s = list(current_joints)

            # j1: 小幅隨機，夾緊到 forward-reach hard 限制
            s[0] += rng.gauss(0, sigma * 0.4)
            s[0] = max(j1_lo, min(j1_hi, s[0]))

            # j2: 偏向適度外展
            s[1] += rng.gauss(0, sigma * 0.5)
            s[1] = max(lo_urdf[1], min(hi_urdf[1], s[1]))

            # j3: ★ 偏向 elbow-down，且額外夾緊到 forward-reach 上界 ★
            s[2] += rng.gauss(bias_j3, sigma * 0.6)
            s[2] = max(j3_lo, min(j3_hi, s[2]))  # 右臂上界 0.0，左臂下界 0.0

            # j4~j7: 小幅擾動，保持 wrist 穩定
            s[3] += rng.gauss(0, sigma * 0.4)
            s[4] += rng.gauss(0, sigma * 0.2)
            s[5] += rng.gauss(0, sigma * 0.2)
            s[6] += rng.gauss(0, sigma * 0.1)

            # 最終用 URDF 範圍做最後保護
            s = [max(lo_urdf[i], min(hi_urdf[i], s[i])) for i in range(7)]
            seeds.append(s)

        return seeds


# ─────────────────────────────────────────────────────────────────────────────
# Self-test（直接執行此檔案）
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 70)
    print(" forward_reach_ik_solver.py — Self-test")
    print("=" * 70)

    solver_r = ForwardReachIKSolver("right")
    solver_l = ForwardReachIKSolver("left")

    # ── Test 1: Home pose should be natural ───────────────────────────────
    home_r = [0.0, 1.2735, -1.5708, 1.8287, 1.5708, -0.5552, 0.0]
    home_l = [0.0, 1.2735,  1.5708, 1.8287, 1.5708, -0.5552, 0.0]

    print("\n[Test 1] Home pose should be natural & pass all constraints")
    solver_r.print_diagnosis(home_r, "right-home")
    assert not solver_r.violates_hard(home_r), "Right home should not violate"
    assert solver_r.is_natural(home_r), "Right home should be natural"
    print("  ✓ right home: natural, no hard violation")

    solver_l.print_diagnosis(home_l, "left-home")
    assert not solver_l.violates_hard(home_l), "Left home should not violate"
    print("  ✓ left home: natural, no hard violation")

    # ── Test 2: Backward reach (抓背起點: j1 very negative) ──────────────
    print("\n[Test 2] Backward reach (j1=-1.0, 手臂後擺) should be REJECTED")
    backward = [-1.0, 1.0, -1.5708, 1.8287, 1.5708, -0.5552, 0.0]
    solver_r.print_diagnosis(backward, "backward-j1=-1.0")
    assert solver_r.violates_hard(backward), \
        f"j1={backward[0]} should violate hard_lo=-0.50"
    print("  ✓ backward reach (j1=-1.0) correctly rejected")

    # ── Test 3: j3 = +0.1 (elbow slightly above horizontal) ──────────────
    print("\n[Test 3] j3=+0.1 should be REJECTED (ForwardReach hard_hi=0.0)")
    elbow_slightly_up = [0.0, 1.274, 0.10, 1.829, 1.571, -0.555, 0.0]
    solver_r.print_diagnosis(elbow_slightly_up, "j3=+0.1")
    assert solver_r.violates_hard(elbow_slightly_up), \
        "j3=+0.10 > hard_hi=0.0 should be rejected"
    print("  ✓ j3=+0.1 correctly rejected (stricter than NaturalIK's +0.30)")

    # ── Test 4: NaturalIK allowed j3=+0.25, ForwardReach rejects it ──────
    # Note: j5 must be ≤ 1.5708 (π/2) to stay within NaturalIK j5 hard_hi
    print("\n[Test 4] j3=+0.25 (NaturalIK allows, ForwardReach REJECTS)")
    natural_ok_fwd_no = [0.0, 1.274, 0.25, 1.829, 1.500, -0.555, 0.0]
    from natural_ik_solver import NaturalIKSolver
    nat_solver = NaturalIKSolver("right")
    assert not nat_solver.violates_hard(natural_ok_fwd_no), \
        f"NaturalIK should accept j3=+0.25 (got violates={nat_solver.violates_hard(natural_ok_fwd_no)})"
    assert solver_r.violates_hard(natural_ok_fwd_no), \
        "ForwardReach should REJECT j3=+0.25"
    print("  ✓ j3=+0.25: NaturalIK=accept, ForwardReach=REJECT (correct difference)")

    # ── Test 5: j2=−0.1 (arm slightly inward) should be rejected ─────────
    print("\n[Test 5] j2=-0.1 (arm inward) should be REJECTED (hard_lo=0.0)")
    arm_inward = [0.0, -0.1, -1.5708, 1.8287, 1.5708, -0.5552, 0.0]
    assert solver_r.violates_hard(arm_inward), \
        "j2=-0.1 < hard_lo=0.0 should be rejected"
    print("  ✓ j2=-0.1 correctly rejected (arm pressed against body)")

    # ── Test 6: Score ordering (home better than slightly elevated elbow) ─
    print("\n[Test 6] Score ordering")
    elevated_elbow = [0.0, 1.274, -0.5, 1.829, 1.571, -0.555, 0.0]  # j3=-0.5 (not fully down)
    score_home    = solver_r.score(home_r)
    score_elevated = solver_r.score(elevated_elbow)
    print(f"  home score:              {score_home:.4f}  (j3=-1.5708)")
    print(f"  j3=-0.5 (elevated) score: {score_elevated:.4f}")
    assert score_home < score_elevated, "Home should score better than elevated elbow"
    print("  ✓ home < elevated (correct ordering)")

    # ── Test 7: Forward seeds within URDF limits, correct j3 direction ──────
    # Note: seeds are IK start points, not required to satisfy naturalness hard
    # constraints themselves. We verify: (a) within URDF limits, (b) j3 points
    # in the elbow-down direction (negative right, positive left).
    print("\n[Test 7] Forward-reach seeds: within URDF limits + j3 direction correct")
    all_ok = True
    for arm_label, seed_lib, arm_solver, j3_dir in [
        ("RIGHT", _FWD_SEEDS_RIGHT, solver_r, "<= 0"),
        ("LEFT",  _FWD_SEEDS_LEFT,  solver_l, ">= 0"),
    ]:
        for i, seed in enumerate(seed_lib):
            # Check URDF limits
            urdf_ok = all(URDF_LIMITS[k][0] <= seed[k] <= URDF_LIMITS[k][1] for k in range(7))
            # Check j3 direction (right: ≤ 0, left: ≥ 0)
            j3_ok = (seed[2] <= 0.0) if arm_label == "RIGHT" else (seed[2] >= 0.0)
            if not urdf_ok or not j3_ok:
                print(f"  ✗ {arm_label} seed[{i}]: URDF={urdf_ok} j3_dir={j3_ok} j3={seed[2]:+.3f}")
                all_ok = False
            else:
                print(f"  ✓ {arm_label} seed[{i}]: j1={seed[0]:+.2f} j3={seed[2]:+.3f}")
    assert all_ok, "Some seeds failed URDF/direction check!"

    # ── Test 8: perturb_seeds respects tighter bounds ─────────────────────
    print("\n[Test 8] perturbed seeds should all respect forward-reach bounds")
    perts = solver_r.perturb_seeds(home_r, n=20)
    for i, s in enumerate(perts):
        # j3 must be <= 0.0 for right arm
        assert s[2] <= 0.0, f"Perturbed seed[{i}] j3={s[2]:.4f} > 0.0!"
        # j1 must be in [-0.50, 1.30]
        assert -0.50 <= s[0] <= 1.30, f"Perturbed seed[{i}] j1={s[0]:.4f} out of bounds!"
    print(f"  ✓ 20 perturbed seeds all satisfy j3 ≤ 0.0 and j1 ∈ [-0.50, 1.30]")

    print("\n" + "=" * 70)
    print(" All tests passed ✓")
    print("=" * 70)
    print()
    print("使用方式：在 tracker_ee_delta_ik_natural.py 中只改兩行：")
    print()
    print("  # 原來：")
    print("  from natural_ik_solver import NaturalIKSolver")
    print("  self._natural_ik = NaturalIKSolver(arm=args.arm, node=self)")
    print()
    print("  # 改成：")
    print("  from forward_reach_ik_solver import ForwardReachIKSolver")
    print("  self._natural_ik = ForwardReachIKSolver(arm=args.arm, node=self)")
