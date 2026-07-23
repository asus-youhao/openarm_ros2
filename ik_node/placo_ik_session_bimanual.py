#!/usr/bin/env python3
"""
placo_ik_session_bimanual.py
============================
雙臂合併單一 QP 的 Placo IK session（v1，2026-06-12）。
純 Python、零 ROS 依賴 — 可在任何地方 import。

與單臂版 (placo_ik_session.py) 的差異
--------------------------------------
本檔是**新檔案**，不修改舊版；設計細節見 docs/bimanual_single_qp.md。

1. 【單一 QP 雙 EE】一個 RobotWrapper（14 joints）+ 一個 KinematicsSolver，
   對 openarm_right_link7 / openarm_left_link7 各加 position + orientation task，
   每 tick solve 一次同時得到兩臂解。
   注意：兩臂是運動學獨立鏈，合併 QP 不改變 IK 解本身；價值在共享約束
   （EE 距離節流、未來的 collision constraint / relative task）與架構簡化。

2. 【per-arm adaptive DLS】舊版的 add_regularization_task(λ) 是全域的——
   合併後一臂進奇異點會把另一臂也阻尼變慢。本版改為：
     - 全域 regularization 固定在 λ_base（數值穩定用）
     - 每臂各算 6×7 Jacobian 的 σ_min；σ_min < 閾值的臂，額外加一個
       「拉向 seed 的 JointsTask」權重 = λ_extra(σ_min)，
       效果等同只阻尼那一臂的關節運動，另一臂不受影響。

3. 【統一 Δq 縮放（roadmap #1）】取代舊版 node 端的逐軸 jump-guard clamp：
   solve 完後對每臂的 Δq = q_sol − seed 做整體比例縮放，
   使 max|Δq| ≤ tick_budget_deg（預設 10°/tick）。
   - 修掉舊版漏洞：奇異點 iter boost ×4 會讓等效限速放寬 4 倍
     （per-iteration cap × max_iter），現在 per-tick 預算與 iteration 數解耦。
   - 統一縮放保持 7 軸耦合方向，EE 沿直線慢走而不是被逐軸 clip 走歪。

4. 【EE 近距節流（防撞減速帶）】每步算兩 EE 距離：
   若 solve 後距離 < min_ee_dist 且**正在接近**（dist_after < dist_before），
   兩臂 Δq 預算乘上 prox ∈ [0,1]（距離到 hard floor 時 → 0）。
   遠離方向不節流，所以不會把操作者鎖死。
   ⚠ 這是減速帶不是防撞保證——真正的 placo self-collision constraint
   需要含 collision geometry 的 URDF（目前 _build_kin_urdf 會剝除），列為 TODO。

5. 【動態 W_ORI（roadmap #6 / docs j3-4）】σ_min < 閾值的臂，姿態權重由 2.0
   依 σ_min/σ_thresh 線性降至下限 0.5，奇異點附近讓位置主導、wrist 不再為
   硬湊姿態而擺動。可用 _DYNAMIC_W_ORI = False 關閉（回到固定 2.0）。

6. 【published EE 的精確 FK】回傳 ee_cmd_xyz = 縮放後實際發布關節的 FK，
   node 端 fail 時的 _pose 簿記用它（舊版用 solver 內部 EE，與實際發布值不符）。

調參常數沿用單臂版（_POS_TOL、_W_ORI、λ_base/λ_max/σ_thresh、velocity caps、
j3/j4 coupling 等數值完全相同），方便 A/B 對照。
"""

import os as _os, sys as _sys
_HERE = _os.path.dirname(_os.path.abspath(__file__))
_ROOT = _os.path.dirname(_HERE)
_sys.path.insert(0, _os.path.join(_ROOT, "ik_solver"))

import math
import time
import numpy as np
from typing import Dict, List, Optional

ARMS = ("right", "left")   # 全檔案統一的臂順序

# ── IK 調參常數（與 placo_ik_session.py 同值，便於 A/B） ─────────────────────
_POS_TOL   = 0.003    # m   — early-exit 位置收斂閾值
_POS_RELAX = 0.010    # m   — success 判定（放寬）
_ORI_TOL   = 0.050    # rad — early-exit 姿態閾值 (~2.9°)
_ORI_RELAX = 0.200    # rad — success 姿態閾值 (~11.5°)
_W_POS     = 1.0
_W_ORI     = 2.0
_W_JOINTS  = 5e-4
_MAX_ITER  = 5        # 非奇異區 iteration 上限

# ── Adaptive DLS（per-arm 版） ────────────────────────────────────────────────
# λ_extra(σ) = λ_max × clamp((σ_thresh − σ)/σ_thresh, 0, 1)²   （該臂專屬阻尼）
# 全域 regularization 固定 λ_base（數值穩定，不再隨 σ 變動）。
_DLS_LAMBDA_BASE  = 6e-5
_DLS_LAMBDA_MAX   = 5e-2
_DLS_SIGMA_THRESH = 0.15
_SINGULAR_ITER_BOOST = 4    # 任一臂 σ_min < 閾值 → max_iter ×4（速度由 Δq 預算管）

# ── 動態 W_ORI（奇異點附近位置主導） ──────────────────────────────────────────
_DYNAMIC_W_ORI = True
_W_ORI_MIN     = 0.5   # σ_min → 0 時姿態權重下限

# ── Velocity caps（與單臂版同值；QP 內 per-iteration 限制） ──────────────────
_WRIST_VEL_CAP_DEFAULT    = 4.0    # rad/s（j5-7）；0 以下 = 用 URDF
_WRIST_JOINT_IDX          = (4, 5, 6)
_ARM_VEL_CAP_DEG_PER_ITER = 1.0    # j1-4 每次 solve() 最多 1°；0 以下 = URDF
_ARM_JOINT_IDX            = (0, 1, 2, 3)

# ── 統一 Δq 縮放（per-tick 總預算；roadmap #1） ──────────────────────────────
_TICK_BUDGET_DEG_DEFAULT = 10.0   # 每 tick 單軸最大移動量（縮放是整臂統一比例）

# ── EE 近距節流 ───────────────────────────────────────────────────────────────
_EE_DIST_SOFT_DEFAULT = 0.10   # m — 低於此距離且接近中 → 開始減速
_EE_DIST_HARD_FLOOR   = 0.04   # m — 到此距離時接近速度 → 0（仍可遠離）

# ── j3/j4 elbow-torso coupling（與單臂版同值，預設由 node 決定開關） ─────────
_J3_J4_COUPLE_RATE  = 1.5
_J4_ELBOW_SAFE_MIN  = 1.30
_J3_J4_COUPLE_START = 0.02
_J3_INWARD_IDX      = 2
_J4_ELBOW_IDX       = 3


def _rot_error_rad(current_R: np.ndarray, target_R: np.ndarray) -> float:
    """兩 3×3 旋轉矩陣間最短測地角（rad）。"""
    R_err = current_R.T @ target_R
    cos_angle = (float(np.trace(R_err)) - 1.0) * 0.5
    return float(np.arccos(np.clip(cos_angle, -1.0, 1.0)))


def _mem_rss_kb() -> int:
    try:
        with open("/proc/self/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1])
    except OSError:
        pass
    return 0


class PlacoBimanualSession:
    """單一 QP 同時解雙臂 IK 的 session（cached RobotWrapper，14 DOF）。

    Parameters
    ----------
    urdf            : 雙臂合一 URDF 絕對路徑（_find_urdf() 的輸出）
    rate_hz         : 控制迴圈頻率；solver.dt = 1/rate_hz
    max_iter        : 非奇異區 iteration 上限
    vel_limits      : 是否啟用 QP 內 velocity limits
    wrist_vel_cap   : j5-7 速度上限 rad/s（兩臂同值）
    j3j4_couple     : j3/j4 elbow-torso 安全耦合開關（兩臂同值）
    tick_budget_deg : per-tick 單軸 Δq 預算（統一縮放）；0 以下 = 關閉
    min_ee_dist     : EE 近距節流的軟閾值（m）；0 以下 = 關閉
    """

    def __init__(
        self,
        urdf:            str,
        rate_hz:         float = 50.0,
        max_iter:        int   = _MAX_ITER,
        vel_limits:      bool  = True,
        wrist_vel_cap:   float = _WRIST_VEL_CAP_DEFAULT,
        j3j4_couple:     bool  = False,
        tick_budget_deg: float = _TICK_BUDGET_DEG_DEFAULT,
        min_ee_dist:     float = _EE_DIST_SOFT_DEFAULT,
    ):
        import placo
        from placo_ik_solver import _HUMAN_RIGHT, _HUMAN_LEFT, _JOINT_NAMES

        self._placo       = placo
        self._urdf        = urdf
        self._dt          = 1.0 / rate_hz
        self._max_iter    = max_iter
        self._vel_limits  = vel_limits
        self._wrist_cap   = float(wrist_vel_cap)
        self._j3j4_couple = bool(j3j4_couple) and (_J3_J4_COUPLE_RATE > 0.0)
        self._budget_rad  = (math.radians(tick_budget_deg)
                             if tick_budget_deg and tick_budget_deg > 0 else 0.0)
        self._min_ee_dist = float(min_ee_dist) if min_ee_dist else 0.0

        human = {"right": _HUMAN_RIGHT, "left": _HUMAN_LEFT}
        self._names   = {a: list(_JOINT_NAMES[a]) for a in ARMS}
        self._ee_link = {a: f"openarm_{a}_link7" for a in ARMS}
        self._lo      = {a: [human[a][n][1] for n in self._names[a]] for a in ARMS}
        self._hi      = {a: [human[a][n][2] for n in self._names[a]] for a in ARMS}
        self._pref    = {a: {n: human[a][n][0] for n in self._names[a]} for a in ARMS}
        # j3 內旋符號：right j3>0 = 內旋，left j3<0 = 內旋
        self._j3_sign = {"right": 1.0, "left": -1.0}

        # 單一 RobotWrapper 載入雙臂模型。
        # ⚠ ignore_collisions：目前的 kin-URDF 已剝除 collision geometry，
        #   placo self-collision constraint 無法啟用 → 用 EE 近距節流代替（TODO 見 docs）。
        self._robot = placo.RobotWrapper(urdf, placo.Flags.ignore_collisions)
        self._apply_velocity_caps(self._robot)

        # Jacobian 欄位（per-arm，URDF 固定 → 算一次）
        self._jac_cols = {
            a: [self._robot.get_joint_v_offset(n) for n in self._names[a]]
            for a in ARMS
        }

        # mem 取樣節流（同單臂版）
        self._mem_sample_every = 50
        self._mem_step_count   = 0
        self._mem_cached_kb    = 0

        cap_str = (f"wrist≤{self._wrist_cap:.1f}rad/s" if self._wrist_cap > 0
                   else "wrist=URDF")
        if _ARM_VEL_CAP_DEG_PER_ITER > 0:
            cap_str += f"  arm(j1-4)≤{_ARM_VEL_CAP_DEG_PER_ITER:.1f}°/it"
        print(f"[PlacoBimanualSession] cached 14-DOF  max_iter={max_iter}"
              f"  dt={self._dt*1000:.1f}ms  vel_limits={vel_limits}  {cap_str}")
        print(f"[PlacoBimanualSession] per-arm DLS σ_thresh={_DLS_SIGMA_THRESH}"
              f"  λ_extra_max={_DLS_LAMBDA_MAX}"
              f"  tick_budget={tick_budget_deg:.1f}°"
              f"  min_ee_dist={self._min_ee_dist*100:.0f}cm"
              f"  dyn_W_ORI={'on' if _DYNAMIC_W_ORI else 'off'}")

    # ── helpers ───────────────────────────────────────────────────────────────
    def _throttled_mem_kb(self) -> int:
        self._mem_step_count += 1
        if self._mem_step_count % self._mem_sample_every == 1:
            self._mem_cached_kb = _mem_rss_kb()
        return self._mem_cached_kb

    def _apply_velocity_caps(self, robot) -> None:
        """兩臂套用同一組 wrist/arm velocity caps（語意同單臂版）。"""
        for a in ARMS:
            if self._wrist_cap > 0.0:
                for i in _WRIST_JOINT_IDX:
                    robot.set_velocity_limit(self._names[a][i], self._wrist_cap)
            if _ARM_VEL_CAP_DEG_PER_ITER > 0.0:
                cap = float(np.radians(_ARM_VEL_CAP_DEG_PER_ITER) / self._dt)
                for i in _ARM_JOINT_IDX:
                    robot.set_velocity_limit(self._names[a][i], cap)

    @staticmethod
    def _lambda_extra(sigma_min: float) -> float:
        ratio = max(0.0, (_DLS_SIGMA_THRESH - sigma_min) / _DLS_SIGMA_THRESH)
        return _DLS_LAMBDA_MAX * ratio * ratio

    @staticmethod
    def _w_ori_dynamic(sigma_min: float) -> float:
        if not _DYNAMIC_W_ORI or sigma_min >= _DLS_SIGMA_THRESH:
            return _W_ORI
        return max(_W_ORI_MIN, _W_ORI * sigma_min / _DLS_SIGMA_THRESH)

    def _j4_dynamic_hi(self, arm: str, j3_val: float) -> List[float]:
        """依 j3 內旋角回傳該臂的動態 hard-hi（j3/j4 coupling 硬保證）。"""
        hi = list(self._hi[arm])
        if not self._j3j4_couple:
            return hi
        j3_in = max(0.0, j3_val * self._j3_sign[arm] - _J3_J4_COUPLE_START)
        if j3_in > 0.0:
            hi[_J4_ELBOW_IDX] = max(_J4_ELBOW_SAFE_MIN,
                                    hi[_J4_ELBOW_IDX] - _J3_J4_COUPLE_RATE * j3_in)
        return hi

    # ── Solve one step（單一 QP，雙 EE） ──────────────────────────────────────
    def solve_step(
        self,
        targets: Dict[str, Dict],          # {"right": {"xyz": np3, "R": 3×3}, "left": {...}}
        seeds:   Dict[str, List[float]],   # {"right": [7], "left": [7]}
        no_rot:  bool = False,
    ) -> Dict:
        """解一步雙臂 IK。

        Returns
        -------
        {
          "timing": {robot_ms, setup_ms, loop_ms, solve_ms, wall_ms,
                     iterations, iter_ms, mem_kb},
          "ee_dist_before_m", "ee_dist_after_m"（QP 解構型，prox 依據）,
          "ee_dist_cmd_m"（實際發布構型，監控用）, "prox_scale",
          "right"/"left": {
              joints            — 實際應發布的關節（統一縮放 + clip 後）
              joints_solved     — QP 原始解（clip 後、縮放前）
              success, pos_err_mm, ori_err_deg     — 以 QP 原始解計
              sigma_min, lambda_extra, w_ori
              budget_scale      — 該臂 Δq 統一縮放係數（已含 prox）
              ee_solved_xyz     — QP 解的 EE 位置
              ee_cmd_xyz        — 縮放後實際發布關節的 FK EE 位置（_pose 簿記用）
          },
        }
        """
        placo   = self._placo
        robot   = self._robot
        t_wall0 = time.perf_counter()
        robot_ms = 0.0   # cached：無 rebuild 成本（欄位保留供與單臂版 CSV 對齊）

        # ── Solver + seed ────────────────────────────────────────────────────
        t0 = time.perf_counter()
        solver = placo.KinematicsSolver(robot)
        solver.enable_joint_limits(True)
        solver.enable_velocity_limits(self._vel_limits)
        solver.mask_fbase(True)
        solver.dt = self._dt

        for a in ARMS:
            for name, val in zip(self._names[a], seeds[a]):
                robot.set_joint(name, val)
        robot.update_kinematics()

        # ── per-arm σ_min / λ_extra / W_ORI + seed EE（近距節流基準） ────────
        sigma, lam_extra, w_ori, ee_seed = {}, {}, {}, {}
        for a in ARMS:
            J = robot.frame_jacobian(self._ee_link[a], "world")[:, self._jac_cols[a]]
            svs = np.linalg.svd(J, compute_uv=False)
            sigma[a]     = float(svs[-1])
            lam_extra[a] = self._lambda_extra(sigma[a])
            w_ori[a]     = self._w_ori_dynamic(sigma[a])
            ee_seed[a]   = np.array(
                robot.get_T_world_frame(self._ee_link[a])[:3, 3])
        dist_before = float(np.linalg.norm(ee_seed["right"] - ee_seed["left"]))

        # ── Tasks（per-arm） ─────────────────────────────────────────────────
        for a in ARMS:
            pos_task = solver.add_position_task(self._ee_link[a], targets[a]["xyz"])
            pos_task.configure(f"pos_{a}", "soft", _W_POS)
            if not no_rot:
                ori_task = solver.add_orientation_task(self._ee_link[a], targets[a]["R"])
                ori_task.configure(f"ori_{a}", "soft", w_ori[a])
            jt = solver.add_joints_task()
            jt.set_joints(self._pref[a])
            jt.configure(f"nat_{a}", "soft", _W_JOINTS)

            # per-arm DLS：只阻尼進奇異點的那一臂（拉向 seed 的 JointsTask）
            if lam_extra[a] > 1e-9:
                damp = solver.add_joints_task()
                damp.set_joints({n: s for n, s in zip(self._names[a], seeds[a])})
                damp.configure(f"damp_{a}", "soft", lam_extra[a])

            # j3/j4 soft 引導（同單臂版，用 seed j3）
            j3_seed = float(seeds[a][_J3_INWARD_IDX]) * self._j3_sign[a]
            if self._j3j4_couple and j3_seed > _J3_J4_COUPLE_START:
                j4_name    = self._names[a][_J4_ELBOW_IDX]
                j4_safe_hi = max(_J4_ELBOW_SAFE_MIN,
                                 self._hi[a][_J4_ELBOW_IDX] - _J3_J4_COUPLE_RATE * j3_seed)
                jt_j4 = solver.add_joints_task()
                jt_j4.set_joints({j4_name: min(self._pref[a][j4_name], j4_safe_hi)})
                jt_j4.configure(f"j4_safety_{a}", "soft", 2e-3)

        # 全域 regularization 固定 λ_base（per-arm 阻尼在上面）
        reg = solver.add_regularization_task(_DLS_LAMBDA_BASE)
        reg.configure("reg", "soft", 1.0)
        setup_ms = (time.perf_counter() - t0) * 1000.0

        # ── 迭代 + 雙臂 early exit ────────────────────────────────────────────
        t0 = time.perf_counter()
        sigma_worst  = min(sigma[a] for a in ARMS)
        max_iter_dyn = (self._max_iter if sigma_worst >= _DLS_SIGMA_THRESH
                        else self._max_iter * _SINGULAR_ITER_BOOST)
        iters_used = 0
        T_ee: Dict[str, Optional[np.ndarray]] = {a: None for a in ARMS}
        for _ in range(max_iter_dyn):
            solver.solve(True)
            robot.update_kinematics()
            iters_used += 1
            all_ok = True
            for a in ARMS:
                T = robot.get_T_world_frame(self._ee_link[a])
                T_ee[a] = T
                if float(np.linalg.norm(T[:3, 3] - targets[a]["xyz"])) >= _POS_TOL:
                    all_ok = False
                    continue
                if not no_rot and \
                        _rot_error_rad(T[:3, :3], targets[a]["R"]) >= _ORI_TOL:
                    all_ok = False
            if all_ok:
                break
        loop_ms = (time.perf_counter() - t0) * 1000.0

        # ── per-arm：取解、clip、誤差、success ────────────────────────────────
        out: Dict[str, Dict] = {}
        for a in ARMS:
            T = T_ee[a] if T_ee[a] is not None else \
                robot.get_T_world_frame(self._ee_link[a])
            q_sol = [robot.get_joint(n) for n in self._names[a]]
            hi_dyn = self._j4_dynamic_hi(a, q_sol[_J3_INWARD_IDX])
            q_sol = [max(l, min(h, q))
                     for q, l, h in zip(q_sol, self._lo[a], hi_dyn)]
            pos_err = float(np.linalg.norm(T[:3, 3] - targets[a]["xyz"]))
            ori_err = 0.0 if no_rot else _rot_error_rad(T[:3, :3], targets[a]["R"])
            out[a] = {
                "joints_solved": q_sol,
                "pos_err_mm":    pos_err * 1000.0,
                "ori_err_deg":   float(np.degrees(ori_err)),
                "success":       int(pos_err < _POS_RELAX
                                     and (no_rot or ori_err < _ORI_RELAX)),
                "sigma_min":     sigma[a],
                "lambda_extra":  lam_extra[a],
                "w_ori":         w_ori[a],
                "ee_solved_xyz": [float(v) for v in T[:3, 3]],
            }
        dist_after = float(np.linalg.norm(
            np.array(out["right"]["ee_solved_xyz"])
            - np.array(out["left"]["ee_solved_xyz"])))

        # ── EE 近距節流：只在「正在接近」時降速 ──────────────────────────────
        # 以「目前實際構型」的距離 (dist_before) 做 ramp，而不是 QP 解構型的
        # dist_after——後者在奇異點 iter boost 時可一步衝很遠，會造成
        # 「解構型已低於 hard floor → prox=0 → seed 永不前進」的死鎖
        # （兩臂相距 30cm 也被凍住）。dist_after 只用來判斷方向（接近/遠離）。
        # 另加每步接近量上限：最多縮小剩餘間距的一半 → 幾何收斂、
        # 不會單步穿越 hard floor。
        prox = 1.0
        if self._min_ee_dist > 0.0 and dist_after < dist_before:
            span = max(1e-6, self._min_ee_dist - _EE_DIST_HARD_FLOOR)
            ramp = (float(np.clip(
                        (dist_before - _EE_DIST_HARD_FLOOR) / span, 0.0, 1.0))
                    if dist_before < self._min_ee_dist else 1.0)
            gap     = max(0.0, dist_before - _EE_DIST_HARD_FLOOR)
            closing = dist_before - dist_after
            cap     = (1.0 if closing <= 1e-9
                       else min(1.0, (0.5 * gap) / closing))
            prox = min(ramp, cap)

        # ── 統一 Δq 縮放（per-arm 預算 × prox） ──────────────────────────────
        for a in ARMS:
            delta = [q - s for q, s in zip(out[a]["joints_solved"], seeds[a])]
            max_d = max(abs(d) for d in delta) if delta else 0.0
            s_budget = 1.0
            if self._budget_rad > 0.0 and max_d > self._budget_rad:
                s_budget = self._budget_rad / max_d
            scale = s_budget * prox
            out[a]["budget_scale"] = scale
            out[a]["joints"] = [s + scale * d for s, d in zip(seeds[a], delta)]

        # ── 縮放後實際發布關節的 FK（精確 ee_cmd，供 node _pose 簿記） ────────
        for a in ARMS:
            for name, val in zip(self._names[a], out[a]["joints"]):
                robot.set_joint(name, val)
        robot.update_kinematics()
        for a in ARMS:
            T = robot.get_T_world_frame(self._ee_link[a])
            out[a]["ee_cmd_xyz"] = [float(v) for v in T[:3, 3]]
        # 實際發布構型的 EE 距離（操作監控用；prox 仍以較保守的 dist_after 計）
        dist_cmd = float(np.linalg.norm(
            np.array(out["right"]["ee_cmd_xyz"])
            - np.array(out["left"]["ee_cmd_xyz"])))

        solve_ms = robot_ms + setup_ms + loop_ms
        result = {
            "timing": {
                "robot_ms":   robot_ms,
                "setup_ms":   setup_ms,
                "loop_ms":    loop_ms,
                "solve_ms":   solve_ms,
                "wall_ms":    (time.perf_counter() - t_wall0) * 1000.0,
                "iterations": iters_used,
                "iter_ms":    loop_ms / iters_used if iters_used else 0.0,
                "mem_kb":     self._throttled_mem_kb(),
            },
            "ee_dist_before_m": dist_before,
            "ee_dist_after_m":  dist_after,   # QP 解構型的距離（prox 依據，較保守）
            "ee_dist_cmd_m":    dist_cmd,     # 實際發布構型的距離（監控用）
            "prox_scale":       prox,
        }
        result.update(out)
        return result

    # ── FK helper（單臂） ─────────────────────────────────────────────────────
    def fk(self, arm: str, joints: List[float]) -> Optional[List[float]]:
        """單臂 FK：回傳 [x,y,z,qx,qy,qz,qw]，失敗回 None。

        只設定該臂 7 軸（另一臂維持上次狀態）；solve_step 每步都會重設
        seed，所以先呼叫 fk 不影響解算。
        """
        try:
            robot = self._robot
            for name, val in zip(self._names[arm], joints):
                robot.set_joint(name, val)
            robot.update_kinematics()
            T   = robot.get_T_world_frame(self._ee_link[arm])
            xyz = T[:3, 3]
            R   = T[:3, :3]
            # Shepperd 4-branch R→quat（同單臂版 fk）
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
