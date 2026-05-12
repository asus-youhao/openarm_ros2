#!/usr/bin/env python3
"""
natural_ik_solver.py — Joint-constrained IK for human-like arm teleop
======================================================================

Problem:
    OpenArm O6 (7-DOF) 對任意 6-DOF EE 姿態有無限多 IK 解（1D null-space）。
    MoveIt /compute_ik 只根據「seed 距離」選解 → 有時肘部高於肩膀、手臂往後扭。

    症狀：
        · VR 遙控時手臂突然跳到肘部朝上的配置
        · 手臂做出不像人類的姿勢（肘部高於肩膀、手臂折到後面）
        · 相同 EE 目標、不同初始姿態 → 得到完全不同的 joint 解

Solution implemented here:
    1. **Multi-seed IK**:     嘗試多個偏向自然姿態的 seed，挑最佳解
    2. **Naturalness score**: 根據關節角度對自然程度打分
    3. **Hard constraints**:  硬性拒絕 elbow-above-shoulder（j3 > 閾值）
    4. **Null-space sampling**: 沿 null-space 方向微擾 seed，尋找更自然解

─────────────────────────────────────────────────────────────────────────────
VR Teleop / Motion Capture IK 業界做法對照表
─────────────────────────────────────────────────────────────────────────────
┌────────────────────────────────┬────────┬─────────┬──────────────────────┐
│ 方法                           │ 速度   │ 品質    │ 使用場景              │
├────────────────────────────────┼────────┼─────────┼──────────────────────┤
│ 直接關節映射 (leader-follower) │ <0.1ms │ Best    │ ALOHA, GELLO,        │
│   人體關節 → 機器手臂關節      │        │ (if fit)│ AgiBot, HumanPlus    │
├────────────────────────────────┼────────┼─────────┼──────────────────────┤
│ 解析式 IK + elbow hint vector  │ <1ms   │ Good    │ Franka, UR series,   │
│   明確指定肘部方向向量          │        │         │ 大多數 6-DOF 機械臂   │
├────────────────────────────────┼────────┼─────────┼──────────────────────┤
│ Multi-seed IK + 自然姿態評分   │ 5-30ms │ Good    │ Pico/Quest VR teleop │
│  ← 本文件實作此方法            │        │         │ HoloBot, AvatarXR    │
├────────────────────────────────┼────────┼─────────┼──────────────────────┤
│ Null-space posture control     │10-50ms │ Best    │ NASA Valkyrie,       │
│  Jacobian 投影次要目標         │        │         │ whole-body control   │
├────────────────────────────────┼────────┼─────────┼──────────────────────┤
│ 學習式 IK (neural network)    │ <1ms   │ Great   │ DexHand, DiffIK,     │
│  訓練資料含自然姿態分布         │        │         │ DexMimicHands        │
└────────────────────────────────┴────────┴─────────┴──────────────────────┘

你的情況（20Hz Pico VR teleop, MoveIt /compute_ik, 單臂）：
    → Multi-seed IK + naturalness scoring 是速度與品質的實用平衡點
    → 若仍有解跳動，加入 null-space refinement（本文件 NaturalIKSolver.refine_via_nullspace()）

─────────────────────────────────────────────────────────────────────────────
OpenArm O6 右臂：關節物理意義 & 自然姿態範圍
─────────────────────────────────────────────────────────────────────────────
URDF joint limits（來自 v10/joint_limits.yaml）：
    j1: [-1.396, 3.491]   base yaw rotation
    j2: [-1.745, 1.745]   shoulder pitch（前後）
    j3: [-1.571, 1.571]   shoulder roll  ← 這關節控制肘部高低
    j4: [ 0.000, 2.443]   elbow flexion（彎曲）
    j5: [-1.571, 1.571]   forearm roll
    j6: [-0.785, 0.785]   wrist yaw（左右）
    j7: [-1.571, 1.571]   wrist pitch（上下）

Home 配置（右臂）：[0.0, 1.274, -1.571, 1.829, 1.571, -0.555, 0.0]

★ 已經 URDF 驗證的關節物理意義（右臂）：
    OpenArm 右臂 base 以 rpy="1.5708 0 0" (+90° X-roll) 掛載
    → arm local Z = world -Y;  arm local -X = world -X

    joint1  axis="0 0 1" (arm Z → world -Y):  肩膀前後擺  (flexion/extension)
    joint2  axis="-1 0 0" (arm -X → world -X): 肩膀側向抬  (abduction/adduction, 飛鳥)
    joint3  axis="0 0 1" (link2 Z):            上臂旋轉    (二頭旋轉, upper arm rotation)
                                               ← 7-DOF null-space 參數！
                                               ← 控制 elbow 朝向
    joint4  axis="0 1 0":  肘部彎曲 (elbow flexion)
    joint5  axis="0 0 1":  前臂旋轉 (forearm roll)
    joint6  axis="1 0 0":  手腕左右 (wrist yaw / left-right)
    joint7  axis="0 -1 0": 手腕上下 (wrist pitch / up-down)

    j3 的物理意義（上臂旋轉，null-space 關鍵）：
    j3 ≈ -1.57 rad (-90°)  → 肘部朝下 ← home，自然放鬆 ✓
    j3 ≈  0.00 rad  (0°)   → 肘部朝側（水平）
    j3 ≈ +1.57 rad (+90°)  → 肘部朝上 ← 你的 bug！

    自然姿態軟約束建議（可調整）：
        j2: 偏好區間 [0.2, 1.74]  → 手臂保持外展（不內收超過 -17°）
        j3: 偏好區間 [-1.57, 0.0] → 肘部不高於肩膀（ < 0 即可）
            硬性限制: j3 ≤ +0.3   → 超過 17° 就算肘部過高，直接拒絕 IK 解
    j4: 偏好區間 [0.5, 2.2]   → 手肘保持適度彎曲

─────────────────────────────────────────────────────────────────────────────
Integration 整合方式（3 行加入，不修改原始檔案）
─────────────────────────────────────────────────────────────────────────────

方式 A：最快整合（post-IK filter，加在 call_ik_sync 後）
    ─── 在 tracker_ee_delta_ik_controller_tf.py 的 __init__ 加入 ───
    from natural_ik_solver import NaturalIKSolver
    self._natural_ik = NaturalIKSolver(arm=args.arm, node=self)

    ─── 把 step_and_send() 和 set_target_and_send() 裡的 ──────────
    ok, joints, ik_ms = self.call_ik_sync()
    ─── 換成 ───────────────────────────────────────────────────────
    ok, joints, ik_ms = self._natural_ik.call_natural(
        base_ik_fn=self.call_ik_sync,
        current_joints=self._last_joints,
        set_seed_fn=self._set_ik_seed,
    )

方式 B：獨立來呼叫（搭配 score 印出診斷資訊）
    solver = NaturalIKSolver("right")
    ok, joints, ik_ms = self.call_ik_sync()
    if ok:
        cost = solver.score(joints)
        if solver.violates_hard(joints):
            self.get_logger().warn(f"Unnatural IK! j3={joints[2]:.3f} cost={cost:.2f}")

─────────────────────────────────────────────────────────────────────────────
Dependencies:
    pip install numpy scipy    (scipy 用於 null-space 最佳化，可選)
"""

import math
import time
import random
import threading
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# URDF Joint Limits — OpenArm O6 v10
# ─────────────────────────────────────────────────────────────────────────────
# ────────────────────────────────────────────────────────────────────
# Physical joint meanings (verified from URDF openarm_arm.xacro):
#   Right arm base is mounted at rpy="1.5708 0 0" (+90° roll about world X)
#   → arm local Z  = world -Y direction
#   → arm local -X = world -X direction
#
#   joint1  axis="0 0 1" (arm Z)    → rotates about world -Y
#           = SHOULDER FLEXION/EXTENSION (前後擺, arm swings forward/back)
#
#   joint2  axis="-1 0 0" (arm -X)  → rotates about world -X
#           = SHOULDER ABDUCTION/ADDUCTION (飛鳥, lateral raise/lower)
#
#   joint3  axis="0 0 1" (link2 Z)  → UPPER ARM ROTATION (二頭旋轉)
#           = controls WHERE THE ELBOW POINTS (elbow-down vs elbow-up)
#           This is the 7-DOF null-space parameter!
#           j3 = -90° → elbow points DOWN (natural home) ✓
#           j3 =  0°  → elbow points sideways
#           j3 = +90° → elbow points UP (unnatural bug!)
#
#   joint4  axis="0 1 0" → ELBOW FLEXION (手肘彎曲)
#   joint5  axis="0 0 1" → FOREARM ROLL (前臂旋轉)
#   joint6  axis="1 0 0" → WRIST YAW / LEFT-RIGHT (手腕左右)
#   joint7  axis="0 -1 0" → WRIST PITCH / UP-DOWN (手腕上下)
# ────────────────────────────────────────────────────────────────────
URDF_LIMITS = {
    # joint_idx: (lower_rad, upper_rad)
    0: (-1.396263,  3.490659),   # j1: shoulder flexion/extension (前後, axis Z)
    1: (-1.745329,  1.745329),   # j2: shoulder abduction/adduction (飛鳥, axis -X)
    2: (-1.570796,  1.570796),   # j3: upper arm rotation / elbow swivel (二頭旋轉, axis Z)
    3: ( 0.000000,  2.443461),   # j4: elbow flexion (手肘彎曲, axis Y)
    4: (-1.570796,  1.570796),   # j5: forearm roll (前臂旋轉, axis Z)
    5: (-0.785398,  0.785398),   # j6: wrist yaw / left-right (手腕左右, axis X)
    6: (-1.570796,  1.570796),   # j7: wrist pitch / up-down (手腕上下, axis -Y)
}


# ─────────────────────────────────────────────────────────────────────────────
# Per-joint naturalness configuration
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class JointCfg:
    """
    自然姿態配置：每關節的偏好角度與軟/硬約束範圍。

    Fields:
        pref:        偏好角度 (rad)，越接近越自然
        soft_lo:     軟限下界，超出按二次方懲罰
        soft_hi:     軟限上界，超出按二次方懲罰
        hard_lo:     硬限下界，違反 → 直接拒絕解
        hard_hi:     硬限上界，違反 → 直接拒絕解
        weight:      懲罰強度（越高越嚴格）
        name:        關節名稱（僅供除錯）
    """
    pref:    float
    soft_lo: float
    soft_hi: float
    hard_lo: float
    hard_hi: float
    weight:  float = 1.0
    name:    str   = ""

    def penalty(self, angle: float) -> float:
        """計算此關節角度的不自然懲罰值（越低越好）。"""
        cost = 0.0
        # 軟約束：二次方懲罰
        if angle < self.soft_lo:
            cost += self.weight * (self.soft_lo - angle) ** 2
        elif angle > self.soft_hi:
            cost += self.weight * (angle - self.soft_hi) ** 2
        # 偏好角度：輕量懲罰（吸引到 preferred angle）
        cost += 0.15 * self.weight * (angle - self.pref) ** 2
        return cost

    def is_hard_violated(self, angle: float) -> bool:
        """若角度違反硬約束回傳 True（解應被拒絕）。"""
        return angle < self.hard_lo or angle > self.hard_hi


# ─────────────────────────────────────────────────────────────────────────────
# Arm-specific naturalness configs
#
# 調整這些數值來控制手臂的自然姿態限制。
# 「soft」= 懲罰分數提高但仍接受；「hard」= 直接拒絕整個 IK 解。
# ─────────────────────────────────────────────────────────────────────────────

# ── Right Arm ────────────────────────────────────────────────────────────────
_RIGHT_CFG: List[JointCfg] = [
    #              pref       soft_lo  soft_hi   hard_lo  hard_hi  weight  name
    # j1: shoulder flexion/extension (前後擺). home=0, neutral upright. 大馬達 → 高懲罰
    JointCfg( 0.0000,  -0.7854,  0.7854,  -1.3963,  3.4907, 2.5, "j1_shoulder_flex_ext"),
    # j2: shoulder abduction/adduction (飛鳥, 側向). 大馬達 → 高懲罰
    JointCfg( 1.2735,   0.2000,  1.4000,  -0.3000,  1.6000, 3.5, "j2_shoulder_abduction"),
    # j3: upper arm yaw (肩膀到手軸yaw) ★ 7-DOF null-space 參數 ★ 大馬達 → 最高懲罰
    #   j3=0.0 → 向前 (elbow forward direction / neutral)
    #   j3 < 0 → 手軸向下/後旋轉 (elbow points down/back)
    #   j3 > 0 → 手軸向上旋轉 → 不自然，直接拒絕
    #   soft_hi = -0.20 → 距水平 11° 前就開始懲罰
    #   hard_hi =  0.00 → 不得超水平（拒絕此解）
    JointCfg( 0.0000,  -1.5708, -0.2000,  -1.5800,  0.0000, 7.0, "j3_upperarm_yaw_ELBOW"),
    # j4: elbow flexion (手肘彎曲). 目標: 90° forward-reach home.
    JointCfg( 1.5708,   0.5000,  2.2000,   0.2000,  2.3000, 2.0, "j4_elbow_flex"),
    # j5: forearm roll (前臂旋轉). home=0.
    JointCfg( 0.0000,   0.3000,  2.5000,  -1.5708,  1.5708, 0.8, "j5_forearm_roll"),
    # j6: wrist yaw / left-right (手腕左右). home=0.
    JointCfg( 0.0000,  -0.7854,  0.4000,  -0.7854,  0.7854, 0.8, "j6_wrist_yaw_lr"),
    # j7: wrist pitch / up-down (手腕上下). 限制手掌朝前（不往後翻轉）.
    #   soft_hi = 0.50 → 手腕後仰超過 28° 開始懲罰（防止手掌往後）
    #   hard_hi = 1.5708 → 使用完整 URDF 範圍，soft 保護已足夠
    JointCfg( 0.0000,  -0.8000,  0.5000,  -1.5708,  1.5708, 1.0, "j7_wrist_pitch_ud_PALM_FWD"),
]

# ── Left Arm (mirror j2/j3 sign) ─────────────────────────────────────────────
# Left arm base rpy="-1.5708 0 0" → arm Z maps to world +Y
# j2 abduction sign is mirrored; j3 elbow-down = POSITIVE (+1.5708)
_LEFT_CFG: List[JointCfg] = [
    JointCfg( 0.0000,  -0.7854,  0.7854,  -3.4907,  1.3963, 2.5, "j1_shoulder_flex_ext"),
    # j2 left: same abduction tightening as right arm
    JointCfg( 1.2735,   0.2000,  1.4000,  -0.3000,  1.6000, 3.5, "j2_shoulder_abduction"),
    # 左臂 j3 鏡像：home = 0.0，j3 < 0 在左臂語義上相同（向後旋）
    JointCfg( 0.0000,   0.2000,  1.5708,   0.0000,  1.5800, 7.0, "j3_upperarm_yaw_ELBOW"),
    # j4 left: forward-reach 90° elbow
    JointCfg( 1.5708,   0.5000,  2.2000,   0.2000,  2.3000, 2.0, "j4_elbow_flex"),
    JointCfg( 0.0000,   0.3000,  2.5000,  -1.5708,  1.5708, 0.8, "j5_forearm_roll"),
    JointCfg( 0.0000,  -0.7854,  0.4000,  -0.7854,  0.7854, 0.8, "j6_wrist_yaw_lr"),
    JointCfg( 0.0000,  -0.8000,  0.5000,  -1.5708,  1.5708, 1.0, "j7_wrist_pitch_ud_PALM_FWD"),
]

_ARM_CFGS = {"right": _RIGHT_CFG, "left": _LEFT_CFG}


# ─────────────────────────────────────────────────────────────────────────────
# Natural seed library
#
# 多個預先計算的自然姿態 seed，用來在 IK 解不自然時重試。
# 順序：越前面的越常用（elbow-down, arm-forward 優先）。
# 格式：[j1, j2, j3, j4, j5, j6, j7] (rad)
# ─────────────────────────────────────────────────────────────────────────────
_NATURAL_SEEDS_RIGHT = [
    # j1       j2      j3       j4      j5       j6       j7     # 描述
    [0.0000,  0.7000,  0.0000,  1.5708,  0.0000,  0.0000,  0.0],  # forward-reach 90°
    [0.0000,  1.0000,  0.0000,  1.5708,  0.0000,  0.0000,  0.0],  # arm higher-forward
    [0.0000,  0.5000,  0.0000,  1.5708,  0.0000,  0.0000,  0.0],  # arm lower-forward
    [0.3000,  0.7000, -0.2000,  1.5708,  0.0000,  0.0000,  0.0],  # reach right
    [-0.300,  1.2000, -1.5000,  1.7000,  1.9000, -0.4000, -0.3],  # reach left
    [0.0000,  0.8000, -1.5708,  2.0000,  1.5708,  0.0000,  0.0],  # arm raised
    [0.0000,  1.6000, -1.3000,  1.2000,  1.5708, -1.0000,  0.0],  # reach down-front
    [0.0000,  1.2735, -1.2000,  2.0000,  1.5708, -0.5552,  0.0],  # elbow slightly up (compromise)
    [0.2000,  1.3000, -1.5708,  1.8000,  1.3000, -0.5000,  0.1],  # slight right rotated
    [-0.200,  1.3000, -1.5708,  1.8000,  1.8000, -0.5000, -0.1],  # slight left rotated
]

_NATURAL_SEEDS_LEFT = [
    [0.0000,  0.7000,  0.0000,  1.5708,  0.0000,  0.0000,  0.0],  # forward-reach 90° (mirrored)
    [0.0000,  1.0000,  0.0000,  1.5708,  0.0000,  0.0000,  0.0],
    [0.0000,  0.5000,  0.0000,  1.5708,  0.0000,  0.0000,  0.0],
    [-0.300,  0.7000,  0.2000,  1.5708,  0.0000,  0.0000,  0.0],
    [0.3000,  0.7000,  0.2000,  1.5708,  0.0000,  0.0000,  0.0],
    [0.0000,  0.8000,  0.0000,  2.0000,  0.0000,  0.0000,  0.0],
    [0.0000,  0.9000,  0.0000,  1.2000,  0.0000,  0.0000,  0.0],
    [0.0000,  0.7000,  0.2000,  2.0000,  0.0000,  0.0000,  0.0],
]

_NATURAL_SEEDS = {"right": _NATURAL_SEEDS_RIGHT, "left": _NATURAL_SEEDS_LEFT}


# ─────────────────────────────────────────────────────────────────────────────
# NaturalIKSolver
# ─────────────────────────────────────────────────────────────────────────────
class NaturalIKSolver:
    """
    Drop-in wrapper：讓 /compute_ik 優先回傳肘部朝下的自然解。

    主要方法
    ────────
    score(joints)           → float   計算不自然分數（越低越好）
    violates_hard(joints)   → bool    硬性約束是否被違反（應拒絕）
    is_natural(joints)      → bool    解是否在「可接受自然範圍」
    call_natural(...)       → (ok, joints, ms)   多 seed IK 主入口
    perturb_seeds(joints)   → List    以當前解為中心生成微擾 seeds
    print_diagnosis(joints) → None    印出每關節分析（除錯用）

    快速判斷示例（不改動原 controller）
    ──────────────────────────────────
        solver = NaturalIKSolver("right")
        ok, joints, ms = node.call_ik_sync()
        if ok and solver.violates_hard(joints):
            node.get_logger().warn("Elbow above shoulder! Retrying...")
            # => 加上 multi-seed retry 邏輯
    """

    # 硬約束違反後允許重試的最大 IK 呼叫次數
    MAX_RETRIES = 5

    # 微擾 seed 時加入的隨機偏移範圍 (rad)
    PERTURB_SIGMA = 0.25

    def __init__(self, arm: str = "right", node=None):
        """
        Parameters
        ----------
        arm  : "right" 或 "left"
        node : rclpy.node.Node（可選，僅用於 logging）
        """
        if arm not in ("right", "left"):
            raise ValueError(f"arm must be 'right' or 'left', got '{arm}'")
        self.arm = arm
        self.node = node
        self._cfg: List[JointCfg] = _ARM_CFGS[arm]
        self._seeds: List[List[float]] = [list(s) for s in _NATURAL_SEEDS[arm]]
        self._lock = threading.Lock()
        self._stats = {"total": 0, "natural_first": 0, "retried": 0, "failed": 0}

    # ──────────────────────────────────────────────────────────────────────────
    # Core scoring & constraint checking
    # ──────────────────────────────────────────────────────────────────────────

    def score(self, joints: List[float]) -> float:
        """
        計算解的「不自然分數」，越低越好。
        負值不可能；0 = 完全自然（≈ home）。

        分數由每關節的 JointCfg.penalty() 加總組成。
        """
        total = 0.0
        for i, (angle, cfg) in enumerate(zip(joints, self._cfg)):
            total += cfg.penalty(angle)
        return total

    def violates_hard(self, joints: List[float]) -> bool:
        """
        若任何關節違反硬性約束回傳 True（解應被拒絕）。

        硬性約束最重要的是 j3 (upper arm rotation / 上臂旋轉)：
            右臂：j3 > +0.30 rad → 肘部朝上 → 違反
            左臂：j3 < -0.30 rad → 肘部朝上 → 違反
        """
        for angle, cfg in zip(joints, self._cfg):
            if cfg.is_hard_violated(angle):
                return True
        return False

    def is_natural(self, joints: List[float]) -> bool:
        """
        解是否「自然可接受」：不違反硬約束且分數低於閾值。
        可作為快速 pass/fail 篩選器。
        """
        if self.violates_hard(joints):
            return False
        # 額外軟約束：分數低於合理上限（可調整）
        score = self.score(joints)
        return score < 3.5  # 大約允許 2 個關節各 ~0.6 rad 偏離 soft 範圍

    def print_diagnosis(self, joints: List[float], label: str = "") -> None:
        """
        印出每個關節的分析，方便調整 JointCfg 數值。

        輸出範例：
            [IK Diagnosis] label=retry1
              j1_base_yaw      :  0.000 rad  soft:[ -0.785,  0.785]  penalty: 0.00  ✓
              j2_shoulder_pitch:  1.274 rad  soft:[  0.200,  1.745]  penalty: 0.00  ✓
              j3_shoulder_roll : -1.571 rad  soft:[ -1.571,  0.000]  penalty: 0.00  ✓
              j4_elbow_flex    :  1.829 rad  soft:[  0.400,  2.200]  penalty: 0.00  ✓
              ...
              TOTAL score: 0.00  |  hard_violated: False
        """
        lines = [f"[IK Diagnosis] label={label}"]
        total = 0.0
        hard_violated = False
        for i, (angle, cfg) in enumerate(zip(joints, self._cfg)):
            p = cfg.penalty(angle)
            total += p
            hv = cfg.is_hard_violated(angle)
            if hv:
                hard_violated = True
            flag = "✗ HARD" if hv else ("⚠" if p > 0.5 else "✓")
            lines.append(
                f"  {cfg.name:<22s}: {angle:+.3f} rad  "
                f"soft:[{cfg.soft_lo:+.3f}, {cfg.soft_hi:+.3f}]  "
                f"hard:[{cfg.hard_lo:+.3f}, {cfg.hard_hi:+.3f}]  "
                f"penalty:{p:5.2f}  {flag}"
            )
        lines.append(
            f"  TOTAL score: {total:.2f}  |  hard_violated: {hard_violated}"
        )
        msg = "\n".join(lines)
        if self.node is not None:
            self.node.get_logger().info(msg)
        else:
            print(msg)

    def get_stats(self) -> dict:
        """回傳累積統計資訊。"""
        with self._lock:
            return dict(self._stats)

    # ──────────────────────────────────────────────────────────────────────────
    # Seed generation
    # ──────────────────────────────────────────────────────────────────────────

    def perturb_seeds(
        self,
        current_joints: List[float],
        n: int = 4,
        sigma: float = PERTURB_SIGMA,
        rng: random.Random = None,
    ) -> List[List[float]]:
        """
        以 current_joints 為中心生成 n 個微擾 seed。

        微擾策略：
          · 主要對 j3（shoulder roll）施加偏向負方向的偏移（強制肘部朝下）
          · j1, j2 施加小幅隨機偏移
          · j4~j7 保持接近當前值（wrist 不需調整）
          · 所有值限制在 URDF 關節限制內

        Parameters
        ----------
        current_joints : 當前 joint 角度列表（7 個）
        n              : 生成數量
        sigma          : 偏移標準差（rad）
        rng            : 可選 random.Random 實例（reproducibility）
        """
        if rng is None:
            rng = random
        seeds = []
        lo, hi = zip(*[URDF_LIMITS[i] for i in range(7)])

        for _ in range(n):
            s = list(current_joints)
            # j1: 小幅隨機
            s[0] += rng.gauss(0, sigma * 0.4)
            # j2: 偏向向前（小幅隨機）
            s[1] += rng.gauss(0, sigma * 0.5)
            # j3: ★ 強制偏向負（肘部朝下）★
            # 加入帶負偏的擾動：均值 -0.3 rad，使解往 elbow-down 方向移動
            bias = -0.3 if self.arm == "right" else +0.3
            s[2] += rng.gauss(bias, sigma * 0.6)
            # j4: 中等擾動
            s[3] += rng.gauss(0, sigma * 0.4)
            # j5~j7: 小幅擾動，保持 wrist 穩定
            s[4] += rng.gauss(0, sigma * 0.2)
            s[5] += rng.gauss(0, sigma * 0.2)
            s[6] += rng.gauss(0, sigma * 0.1)
            # Clamp to URDF limits
            s = [max(lo[i], min(hi[i], s[i])) for i in range(7)]
            seeds.append(s)
        return seeds

    def get_prioritized_seeds(self, current_joints: List[float]) -> List[List[float]]:
        """
        回傳按優先順序排列的 seed 列表：
          1. current_joints（最快，通常成功）
          2. 微擾版本（偏向 elbow-down）
          3. 預設自然姿態庫

        在 call_natural() 中按此順序嘗試 IK，拿到自然解就停止。
        """
        seeds = []
        # Priority 1: current solution
        seeds.append(list(current_joints))
        # Priority 2: perturbed seeds (biased toward elbow-down)
        seeds.extend(self.perturb_seeds(current_joints, n=3))
        # Priority 3: natural seed library
        seeds.extend(self._seeds[:4])  # 取前 4 個最常用的
        return seeds

    # ──────────────────────────────────────────────────────────────────────────
    # Multi-seed IK main interface
    # ──────────────────────────────────────────────────────────────────────────

    def call_natural(
        self,
        base_ik_fn: Callable,
        current_joints: List[float],
        set_seed_fn: Callable[[List[float]], None],
        verbose: bool = False,
    ) -> Tuple[bool, Optional[List[float]], float]:
        """
        多 seed IK 主入口：嘗試多個 seeds，回傳最自然的解。

        Algorithm:
            1. 先用 current_joints seed 呼叫 IK（快速路徑）
            2. 若解自然 → 立即回傳
            3. 若解不自然 → 用 MAX_RETRIES 個種子重試
            4. 最終回傳所有候選中分數最低的解
            5. 若全部失敗 → 回傳 (False, None, 0.0)

        Parameters
        ----------
        base_ik_fn  : 原來的 call_ik_sync() function，不帶參數呼叫
                      例：node.call_ik_sync
        current_joints : 上一個成功的 joint 解（用來生成偏移 seeds）
        set_seed_fn : 設定 IK seed 的 callable(List[float])
                      例：node._set_ik_seed
                      → 在 Controller 的 __init__ 補一個 helper:
                        def _set_ik_seed(self, joints):
                            with self._joints_lock:
                                self._last_joints = list(joints)
        verbose     : True 時印出每次嘗試的結果（除錯用）

        Returns
        -------
        (ok, joints, ik_ms)   同 call_ik_sync() 的格式
        """
        with self._lock:
            self._stats["total"] += 1

        t_total = time.time()
        seeds = self.get_prioritized_seeds(current_joints)
        candidates = []  # [(joints, cost, ik_ms)]

        for attempt, seed in enumerate(seeds[: self.MAX_RETRIES + 1]):
            # 將 seed 寫入 controller 的 _last_joints
            set_seed_fn(seed)

            # 呼叫原本的 IK（同步，帶 timeout）
            result = base_ik_fn()

            if result is None:
                if verbose:
                    self._log(f"[NaturalIK] attempt={attempt} seed_set → IK timeout/fail")
                continue

            ok, joints, ik_ms = result

            if not ok or joints is None:
                if verbose:
                    self._log(f"[NaturalIK] attempt={attempt} → IK no solution")
                continue

            cost = self.score(joints)
            hard_fail = self.violates_hard(joints)

            if verbose:
                j3_deg = math.degrees(joints[2])
                self._log(
                    f"[NaturalIK] attempt={attempt} j3={j3_deg:+.1f}° "
                    f"cost={cost:.2f} hard={'FAIL' if hard_fail else 'ok'}"
                )

            # 快速路徑：第一次嘗試就自然 → 直接回傳（最低延遲）
            if attempt == 0 and not hard_fail and cost < 2.5:
                with self._lock:
                    self._stats["natural_first"] += 1
                total_ms = (time.time() - t_total) * 1000
                return True, joints, ik_ms

            candidates.append((joints, cost, ik_ms, hard_fail))

        # 從候選中挑最佳（不違反硬約束 > 違反硬約束，再比 cost）
        if candidates:
            # 先過濾硬約束未違反的
            natural_cands = [(j, c, ms) for j, c, ms, hf in candidates if not hf]
            if natural_cands:
                best_j, best_cost, best_ms = min(natural_cands, key=lambda x: x[1])
                with self._lock:
                    self._stats["retried"] += 1
                if verbose:
                    self._log(f"[NaturalIK] best cost={best_cost:.2f} (from retry)")
                return True, best_j, best_ms

            # 全部違反硬約束 → 取分數最低的（退而求其次）
            all_cands = [(j, c, ms) for j, c, ms, hf in candidates]
            best_j, best_cost, best_ms = min(all_cands, key=lambda x: x[1])
            with self._lock:
                self._stats["retried"] += 1
            if verbose or True:  # always warn on hard constraint fallback
                j3_deg = math.degrees(best_j[2])
                self._log(
                    f"[NaturalIK] ⚠ All {len(candidates)} solutions violate hard "
                    f"constraints! Using best (j3={j3_deg:+.1f}°, cost={best_cost:.2f}). "
                    f"Consider expanding workspace or relaxing constraints."
                )
            return True, best_j, best_ms

        # 全部 IK 失敗
        with self._lock:
            self._stats["failed"] += 1
        return False, None, (time.time() - t_total) * 1000

    def _log(self, msg: str) -> None:
        if self.node is not None:
            self.node.get_logger().info(msg)
        else:
            print(msg)

    # ──────────────────────────────────────────────────────────────────────────
    # Null-space refinement (advanced, optional)
    # ──────────────────────────────────────────────────────────────────────────

    def refine_via_nullspace(
        self,
        joints: List[float],
        set_seed_fn: Callable[[List[float]], None],
        base_ik_fn: Callable,
        n_steps: int = 3,
        step_size: float = 0.08,
    ) -> Tuple[bool, List[float], float]:
        """
        Null-space 精修：沿 null-space 方向微移 j3（shoulder roll）
        找到更自然的姿態，同時保持 EE 位姿不變。

        原理：
            7-DOF 機械臂對 6-DOF EE 姿態有 1D null-space。
            此方向對應 j3（shoulder roll）的連續變化：
                j3 → j3 ± δ  +  IK 重解 → EE 不動，但肘部位置改變。

        參數
        ----
        joints      : 當前 IK 解（可能肘部過高）
        set_seed_fn : Controller 的 seed setter
        base_ik_fn  : Controller 的 call_ik_sync
        n_steps     : 沿 null-space 嘗試幾步
        step_size   : 每步 j3 調整量（rad）

        回傳
        ----
        (ok, best_joints, ik_ms)
        """
        current_cost = self.score(joints)
        best = (True, list(joints), 0.0, current_cost)

        # 決定搜尋方向：j3 向負方向（肘部朝下）
        direction = -1.0 if self.arm == "right" else +1.0
        j3_limit_lo = URDF_LIMITS[2][0]
        j3_limit_hi = URDF_LIMITS[2][1]

        for step in range(1, n_steps + 1):
            candidate = list(joints)
            candidate[2] += direction * step * step_size  # 移動 j3
            candidate[2] = max(j3_limit_lo, min(j3_limit_hi, candidate[2]))

            set_seed_fn(candidate)
            result = base_ik_fn()
            if result is None:
                continue
            ok, j_new, ms = result
            if not ok or j_new is None:
                continue

            cost_new = self.score(j_new)
            if cost_new < best[3]:
                best = (True, list(j_new), ms, cost_new)

        return best[0], best[1], best[2]


# ─────────────────────────────────────────────────────────────────────────────
# Convenience: standalone solution validator (no solver instance needed)
# ─────────────────────────────────────────────────────────────────────────────

def is_elbow_below_shoulder(arm: str, joints: List[float]) -> bool:
    """
    快速檢查：肘部是否低於肩膀（最常用的自然性判斷）。

    使用方式：
        if not is_elbow_below_shoulder("right", joints):
            retry_ik(natural_seed)

    j3 > 0        → 肘部高於肩膀（右臂）
    j3 < 0        → 肘部低於肩膀（右臂）← 自然
    """
    j3 = joints[2]
    if arm == "right":
        return j3 <= 0.3   # 允許到 +17°，超過就是肘過高
    else:
        return j3 >= -0.3  # 左臂鏡像


def score_naturalness(arm: str, joints: List[float]) -> float:
    """
    計算給定 arm + joints 的不自然分數（靜態，無需建立 NaturalIKSolver）。
    """
    solver = NaturalIKSolver(arm)
    return solver.score(joints)


# ─────────────────────────────────────────────────────────────────────────────
# Integration guide & example stub
# ─────────────────────────────────────────────────────────────────────────────

INTEGRATION_EXAMPLE = """
# ═══════════════════════════════════════════════════════════════════════════
# Integration into tracker_ee_delta_ik_controller_tf.py
# ═══════════════════════════════════════════════════════════════════════════

# ── Step 1: import & initialize (in __init__) ─────────────────────────────
from natural_ik_solver import NaturalIKSolver

# 加在 self._last_joints 初始化之後：
self._natural_ik = NaturalIKSolver(arm=args.arm, node=self)

# ── Step 2: add a seed setter helper (new method in Controller) ───────────
def _set_ik_seed(self, joints: list):
    \"\"\"Helper: NaturalIKSolver 用來在重試前切換 seed。\"\"\"
    with self._joints_lock:
        self._last_joints = list(joints)

# ── Step 3: replace call_ik_sync() calls in step_and_send() ──────────────
# 原來：
#   ok, joints, ik_ms = self.call_ik_sync()
#
# 改成：
    ok, joints, ik_ms = self._natural_ik.call_natural(
        base_ik_fn  = self.call_ik_sync,
        current_joints = list(self._last_joints),
        set_seed_fn = self._set_ik_seed,
        verbose     = False,   # 設 True 可看每次嘗試結果
    )

# ── Step 4: 同樣改 set_target_and_send() 裡的 call_ik_sync() ─────────────
# （和上面完全一樣的替換方式）

# ── Step 5: 退出時印出統計 ───────────────────────────────────────────────────
stats = self._natural_ik.get_stats()
print(f"NaturalIK stats: {stats}")
# 輸出範例：
#   {'total': 1234, 'natural_first': 1180, 'retried': 45, 'failed': 9}
#   → natural_first/total ≈ 96% → 自然解幾乎一次就找到


# ═══════════════════════════════════════════════════════════════════════════
# Quick diagnostic: 測試你目前的 home 配置分數
# ═══════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    solver = NaturalIKSolver("right")
    home = [0.0, 1.2735, -1.5708, 1.8287, 1.5708, -0.5552, 0.0]
    solver.print_diagnosis(home, label="home")

    # 模擬一個「肘部過高」的解
    bad_solution = [0.1, 0.8, 0.9, 1.5, 1.2, -0.3, 0.1]
    solver.print_diagnosis(bad_solution, label="elbow_too_high")
    print(f"violates_hard: {solver.violates_hard(bad_solution)}")     # True
    print(f"is_natural:    {solver.is_natural(bad_solution)}")        # False
"""


# ─────────────────────────────────────────────────────────────────────────────
# Self-test（直接執行此檔案）
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import sys

    print("=" * 70)
    print(" natural_ik_solver.py — Self-test")
    print("=" * 70)

    solver_r = NaturalIKSolver("right")
    solver_l = NaturalIKSolver("left")

    # ── Test 1: Home pose should be nearly perfect ─────────────────────────
    home_r = [0.0, 1.2735, -1.5708, 1.8287, 1.5708, -0.5552, 0.0]
    home_l = [0.0, 1.2735,  1.5708, 1.8287, 1.5708, -0.5552, 0.0]

    print("\n[Test 1] Home pose naturalness")
    solver_r.print_diagnosis(home_r, "right-home")
    assert not solver_r.violates_hard(home_r), "Home should not violate hard constraints"
    assert solver_r.is_natural(home_r), "Home should be natural"
    print("  ✓ right home: natural, no hard violation")

    solver_l.print_diagnosis(home_l, "left-home")
    assert not solver_l.violates_hard(home_l), "Left home should not violate hard constraints"
    print("  ✓ left home: natural, no hard violation")

    # ── Test 2: Elbow-up pose should fail ────────────────────────────────-─
    print("\n[Test 2] Elbow-above-shoulder should be rejected")
    elbow_up = [0.0, 1.0, 1.0, 1.5, 1.5, -0.3, 0.0]   # j3=+1.0 (elbow up!)
    solver_r.print_diagnosis(elbow_up, "elbow-up")
    assert solver_r.violates_hard(elbow_up), f"Elbow-up should be hard-rejected (j3={elbow_up[2]:.2f})"
    assert not solver_r.is_natural(elbow_up)
    print("  ✓ elbow-up correctly rejected")

    # ── Test 3: Score ordering ─────────────────────────────────────────────
    print("\n[Test 3] Score ordering")
    slight_elbow_up = [0.0, 1.274, 0.1, 1.829, 1.571, -0.555, 0.0]   # j3 slightly positive
    score_home = solver_r.score(home_r)
    score_slight = solver_r.score(slight_elbow_up)
    print(f"  home score:         {score_home:.4f}")
    print(f"  slight elbow-up score: {score_slight:.4f}")
    assert score_home < score_slight, "Home should score better than elbow-up"
    print("  ✓ home < slight_elbow_up (correct ordering)")

    # ── Test 4: Convenience functions ─────────────────────────────────────
    print("\n[Test 4] Standalone convenience functions")
    assert is_elbow_below_shoulder("right", home_r)
    assert not is_elbow_below_shoulder("right", elbow_up)
    s = score_naturalness("right", home_r)
    print(f"  score_naturalness(right, home) = {s:.4f}")
    print("  ✓ standalone functions work")

    # ── Test 5: Perturb seeds ─────────────────────────────────────────────
    print("\n[Test 5] Perturb seed generation")
    seeds = solver_r.perturb_seeds(home_r, n=4)
    assert len(seeds) == 4
    for i, s in enumerate(seeds):
        assert len(s) == 7
        for ji, (j_val, (lo, hi)) in enumerate(zip(s, [URDF_LIMITS[k] for k in range(7)])):
            assert lo <= j_val <= hi, f"Seed {i} joint {ji} = {j_val:.3f} out of [{lo:.3f}, {hi:.3f}]"
    print(f"  Generated {len(seeds)} valid perturbed seeds (all within URDF limits)")
    print("  ✓ seed clamping works")

    # ── Test 6: Print integration guide ───────────────────────────────────
    print("\n[Integration Guide]")
    print(INTEGRATION_EXAMPLE)

    print("\n" + "=" * 70)
    print(" All tests passed ✓")
    print("=" * 70)
    print()
    print("Usage in tracker_ee_delta_ik_controller_tf.py:")
    print("  See INTEGRATION_EXAMPLE above, or the file docstring for")
    print("  step-by-step instructions.")
