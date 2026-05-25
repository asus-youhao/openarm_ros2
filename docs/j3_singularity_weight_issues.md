# j3 內旋問題 & IK 旋轉權重隱藏 Issue 深度分析

> **觸發問題**：Right arm joint3（上臂 Yaw）無法向內旋轉，導致 EE 靠近胸前時容易進入奇異點。
> **分析對象**：`placo_ik_session.py`, `placo_ik_solver.py`（`_HUMAN_RIGHT`, `_W_ORI`, `_W_JOINTS`）
> **最後更新**：2026-05-25

---

## 目錄

1. [問題根因：j3 hard limit 擋在 0.0](#1-問題根因j3-hard-limit-擋在-00)
2. [隱藏 Issue 全覽](#2-隱藏-issue-全覽)  
   - 2.1 pref 壓在 hard boundary  
   - 2.2 per-joint weight 是 Dead Code  
   - 2.3 Seed 庫缺少胸前構型  
   - 2.4 _W_ORI=2.0 在奇異點附近加劇振盪  
   - 2.5 JointsTask scalar 使所有關節懲罰相同  
   - 2.6 Adaptive DLS 在此情境下失效  
3. [奇異點分析：胸前 EE 為何卡住](#3-奇異點分析胸前-ee-為何卡住)
4. [優化方案（分等級）](#4-優化方案分等級)
5. [程式碼改動清單](#5-程式碼改動清單)

---

## 1. 問題根因：j3 hard limit 擋在 0.0

```python
# placo_ik_solver.py  _HUMAN_RIGHT
"openarm_right_joint3": (0.000, -1.571, 0.000, 0.80)
#                        pref    lo      hi     weight(dead)
#                        ↑       ↑       ↑
#                        0 rad  -90°    0 rad ← hi = pref = 0 !!!
```

**Right arm j3 的物理含義：**

```
j3 = 0 rad     → 上臂正向前（forward-reach 自然姿勢）
j3 = -1.571    → 上臂向外旋轉 90°（手肘向外）
j3 = +x rad    → 上臂向內旋轉（手肘向身體靠近）← 完全禁止！
```

當 EE 靠近胸前 (X 小、Y 靠近身體中線) 的時候，**自然解需要 j3 > 0（內旋）**才能讓手肘轉入身體側邊，讓 EE 抵達胸前位置。但因為 `hard_hi = 0.000`，IK QP solver 把這個方向完全封死了。

---

## 2. 隱藏 Issue 全覽

### Issue 1 — pref 壓在 hard boundary（嚴重）

```
pref = 0.000   hard_hi = 0.000
   │
   └──> pref IS the boundary

       合法範圍                     非法（被擋）
  ────────────────────│  ×  ×  ×  ×  ×  ×  ×
  -90°   ... -10° -5° 0°           +5° +10°
                      ↑
                  pref = hard_hi
```

**問題：**
- 正常情況下 pref 應在合法範圍中央或靠近常用區域中心，讓 IK 有「退行空間」。
- pref 壓在 hard boundary 上時，JointsTask 的 naturalness 恆把 j3 往邊界方向推。
- QP 裡 joint limit 是 hard constraint（inequality），而 naturalness 是 soft cost，兩者衝突時出現**數值顛簸**：soft 推向 0，hard 卡在 ≤ 0，solver 每步都在貼邊震盪。

---

### Issue 2 — per-joint weight 是 Dead Code（已知 Bug 3）

```python
# placo_ik_session.py  solve_step()
jt = solver.add_joints_task()
jt.set_joints(self._pref)
jt.configure("naturalness", "soft", _W_JOINTS)   # ← scalar 5e-4，全部 7 關節共用
```

`_HUMAN_RIGHT` 裡 j3 weight=0.80、j5/j6/j7 weight=0.03 的細緻設定**全部無效**。
實際上每個關節的 naturalness 懲罰是：

$$\text{cost}_{j_i} = 5 \times 10^{-4} \times (q_i - q_i^{pref})^2 \quad \forall i \in \{1..7\}$$

j3 和 wrist 的懲罰強度完全相同。這意味著：

- **j3 偏離 pref 時，IK 不會比偏離 j6 更「在乎」**
- 胸前奇異點時，j3 和 j5/j6/j7 等權重，solver 可能選擇「j3 留在邊界，讓 wrist 瘋狂補償」
- 設計者期望的「j3 優先維持標準姿勢」根本沒有執行

---

### Issue 3 — Seed 庫缺少胸前構型（單 seed 在線版更嚴重）

```python
_SEEDS_RIGHT = [
    [ 0.00,  0.70,  0.00,  1.5708,  ...],  # j3=0
    [ 0.00,  0.50,  0.00,  1.5708,  ...],  # j3=0
    [ 0.00,  1.00,  0.00,  1.5708,  ...],  # j3=0
    [ 0.20,  0.70, -0.20,  1.5708,  ...],  # j3=-0.20 外旋
    [-0.20,  0.70, -0.20,  1.5708,  ...],  # j3=-0.20 外旋
    [ 0.00,  0.80,  0.00,  2.0000,  ...],  # j3=0
    [ 0.00,  0.60,  0.00,  1.2000,  ...],  # j3=0
]
```

**所有 seed 的 j3 都在 0 或 -0.20（外旋）。完全沒有胸前構型（j3 > 0 或 j2 較大 + j1 偏轉）。**

而且在線版 `PlacoSession` 根本不使用 `_SEEDS_RIGHT`（Bug 4 — 為了速度只用 single seed = last_joints）。當手臂剛進入胸前區域，`last_joints` 是靠近邊界的構型，IK 在同一個局部盆地裡跑，**永遠跑不到需要 j3>0 的正確解**。

---

### Issue 4 — `_W_ORI = 2.0` 在奇異點附近加劇振盪（中等嚴重）

```python
_W_POS  = 1.0   # position task weight
_W_ORI  = 2.0   # orientation task weight  ← 比 position 強 ×2
```

胸前奇異點的發生機制：

```
EE 抵達胸前 → j3 被 hard limit 卡在 0 → IK 無法找到正確 position 解
                  ↓
position error 持續存在（pos_task 推 EE 向目標，但 j3 被擋）
                  ↓
orientation task (W=2.0) 仍試圖維持 EE orientation
                  ↓
solver 在每步迭代裡：pos_task 推 → 碰 j3 wall → ori_task 拉 → wrist 補償
                  ↓
wrist j5/j6/j7 瘋狂振盪（這就是 teleop 「胸前凍住＋wrist 抖」的觀察現象）
```

**根本原因：當 j3 構型空間被封死時，orientation 任務（W=2.0）用 wrist 超額補償，反而讓整個解不穩定。**

---

### Issue 5 — 奇異點方向辨識不足，Adaptive DLS 功效有限

`Adaptive DLS` 根據 Jacobian σ_min 動態提高 λ，確實能在**任意奇異點**時保護 solver 穩定。但：

```
j3 = 0（hard boundary）→ rank-deficient 方向 = j3 自己
λ 提高 → 所有 joint delta 都被 damped，包括 j5/j6/j7
```

DLS damping 是**全局的**，它無法識別「j3 這個方向被 hard limit 擋住，其他方向其實還有空間」。所以在 j3 hard constraint 導致的奇異點，DLS 的效果只有讓 wrist 動作變小（damped），但 **j3 本身已被 hard limit 鎖住**，沒有任何機制讓 j3 突破 0 去找正確的胸前解。

---

### Issue 6 — 奇異點時 σ_min 觸發 DLS boost 但方向錯誤

```python
lambda_dls = _DLS_LAMBDA_BASE + _DLS_LAMBDA_MAX * _ratio * _ratio
reg = solver.add_regularization_task(lambda_dls)
```

RegularizationTask 的作用是 $\lambda \| \dot{q} \|^2$，懲罰所有關節速度。但在胸前情境：

- j3 的速度已被 hard limit 歸零（不需要懲罰，它本來就不動）
- 增加 λ 反而懲罰了 j1/j2/j4 想要幫「繞道」的努力

**結果**：DLS boost 讓手臂更「靜止」，加深了「手臂凍住」的感覺，而非幫助找到迂迴的胸前解。

---

## 3. 奇異點分析：胸前 EE 為何卡住

胸前位置的典型關節構型需求（FK 分析）：

```
目標：EE 在 x≈0.20m, y≈-0.10m (靠近胸前)

需要：j1 ≈ -0.3 rad (手臂往身體中線偏)
      j2 ≈  1.0 rad (肩膀往上抬)
      j3 ≈ +0.25 rad (上臂內旋) ← 目前 hard limit = 0，完全禁止！
      j4 ≈  1.8 rad (肘彎更多)
```

由於 j3 被禁止，IK 嘗試的替代解：

```
替代解 (BAD):
      j1 ≈ -0.6 rad (更大偏轉)
      j2 ≈  1.4 rad (接近 limit 1.6)
      j3 =  0.0 rad (被卡在邊界)
      j4 ≈  2.1 rad (肘彎到接近 limit 2.2)
      → 多個關節同時逼近 limit → Jacobian 條件數惡化 → σ_min 趨近 0
```

**這就是為什麼胸前特別容易出現奇異點：不是數學上的奇異姿態，而是人為加的 hard limit 強迫 IK 進入幾何上的惡劣區域。**

---

## 4. 優化方案（分等級）

### 🔴 P0 — 立即可做，低風險，收益最高

#### P0-A：放寬 j3 hard_hi，允許小幅內旋

```python
# placo_ik_solver.py  _HUMAN_RIGHT
# 修改前
"openarm_right_joint3": (0.000, -1.571, 0.000, 0.80),

# 修改後
"openarm_right_joint3": (-0.10, -1.571, 0.350, 0.80),
#                         ↑ pref 移離邊界   ↑ 允許 +20° 內旋
```

- `hard_hi: 0.000 → 0.350 rad (~20°)`：實機確認硬體不碰軀幹的安全範圍（建議先 0.2 rad 測試）
- `pref: 0.000 → -0.10 rad`：把偏好點移到合法範圍中央靠外側，遠離 boundary，消除邊界震盪

> ⚠️ **實機確認**：在執行前，手動把 right arm j3 轉到 +10° / +20°，確認手肘與軀幹的物理間距。OpenArm O6 的機械設計可能允許稍大的內旋範圍。

---

#### P0-B：在 SEEDS_RIGHT 補充胸前構型

```python
# placo_ik_solver.py
_SEEDS_RIGHT = [
    # 原有 seeds ...
    
    # 新增：胸前構型（j3 內旋 + j2 高 + j4 大彎）
    [ -0.30,  1.00,  0.25,  1.90,  0.00,  0.00,  0.00],  # chest-near
    [ -0.15,  1.10,  0.20,  1.80,  0.00,  0.00,  0.00],  # chest-center
]
```

雖然在線版 `PlacoSession` 不使用 seeds，但：
1. `PlacoIKSolver`（離線版） 馬上受益
2. 為後續「在線版首步 / guard_hit 後觸發 seed 重試」做準備（建議同時實作）

---

### 🟡 P1 — 中等工作量，效益明確

#### P1-A：修復 Bug 3 — 實作真正的 per-joint JointsTask

```python
# placo_ik_session.py  solve_step()  修改前
jt = solver.add_joints_task()
jt.set_joints(self._pref)
jt.configure("naturalness", "soft", _W_JOINTS)

# 修改後（每關節獨立 weight）
human_cfg = _HUMAN_RIGHT if self._arm == "right" else _HUMAN_LEFT
_W_JOINTS_BASE = 5e-4
for name in self._joint_names:
    per_w = human_cfg[name][3]   # index[3] 終於被讀取
    jt_i = solver.add_joints_task()
    jt_i.set_joints({name: self._pref[name]})
    jt_i.configure(f"nat_{name}", "soft", per_w * _W_JOINTS_BASE)
```

效果：
- j3 (weight=0.80) 的 naturalness = `0.80 × 5e-4 = 4e-4`，約是 j5/j6/j7 (weight=0.03) 的 **26.7 倍**
- 奇異點時 j3 不偏離 pref，wrist 不需要無謂補償

延遲影響：7 次額外 `configure()` call < 0.01ms，可忽略。

---

#### P1-B：sigma_min 低時動態降低 `_W_ORI`

```python
# placo_ik_session.py  solve_step()

# 現有的 adaptive DLS 計算
sigma_min = float(_svs[-1])
_ratio    = max(0.0, (_DLS_SIGMA_THRESH - sigma_min) / _DLS_SIGMA_THRESH)
lambda_dls = _DLS_LAMBDA_BASE + _DLS_LAMBDA_MAX * _ratio * _ratio

# 新增：奇異點時降低 ORI weight，讓 position 任務有更多空間繞道
_W_ORI_DYNAMIC = _W_ORI * max(0.3, 1.0 - _ratio * 0.7)
# sigma_min 正常  → W_ORI_DYNAMIC ≈ 2.0（不變）
# sigma_min → 0   → W_ORI_DYNAMIC ≈ 0.6（降 70%，讓 pos 主導）

if not no_rot:
    ori_task = solver.add_orientation_task(self._ee_link, target_R)
    ori_task.configure("ori", "soft", _W_ORI_DYNAMIC)   # ← 改用動態值
```

效果：奇異點時不再用 wrist 死命補 orientation，允許 EE 方向有更多誤差以換取 position 解的穩定性。

---

### 🟢 P2 — 進階，架構改動較大

#### P2-A：在線版 PlacoSession 增加 guard_hit 後 seed 重試

```python
# placo_ik_node.py  主迴圈內
if self._guard_streak > 3 or self._fail_streak > 5:
    seeds_to_try = [seed] + _chest_seeds_right[:2]  # 只試胸前構型
else:
    seeds_to_try = [seed]

for s in seeds_to_try:
    r = self._placo_session.solve_step(target_xyz, target_R, s, ...)
    if r["success"]:
        break
```

代價：最多 +1.0ms（2 extra seeds × 0.5ms），只在連續失敗時觸發。

---

#### P2-B：Null-space projection 引導 j3 遠離邊界

在 Placo 的 QP 框架下，可以利用 null-space 加一個額外的 JointsTask，當 j3 接近 0 時主動推向 -0.1（避免邊界）：

```python
j3_cur = robot.get_joint(self._joint_names[2])
j3_repulse_pref = -0.15 if j3_cur > -0.05 else j3_cur  # 接近 0 時推向 -0.15
jt_ns = solver.add_joints_task()
jt_ns.set_joints({self._joint_names[2]: j3_repulse_pref})
jt_ns.configure("j3_repulse", "soft", 2e-3)  # 比一般 naturalness 強 4×
```

---

## 5. 程式碼改動清單

### 立即修（P0）

| 檔案 | 改動 | 影響 |
|------|------|------|
| `ik_solver/placo_ik_solver.py` | j3 `hard_hi: 0.000 → 0.35` | 解放胸前 workspace |
| `ik_solver/placo_ik_solver.py` | j3 `pref: 0.000 → -0.10` | 消除 pref 壓邊界的震盪 |
| `ik_solver/placo_ik_solver.py` | 新增胸前 seeds × 2 | PlacoIKSolver 離線版馬上受益 |

### 短期修（P1，1-3h）

| 檔案 | 改動 | 影響 |
|------|------|------|
| `ik_node/placo_ik_session.py` | 改為 7 個獨立 JointsTask（fix Bug 3） | per-joint weight 真正生效 |
| `ik_node/placo_ik_session.py` | `_W_ORI_DYNAMIC` 奇異點降 ori weight | 胸前不再 wrist 补償振盪 |

### 長期優化（P2，3-8h）

| 檔案 | 改動 | 影響 |
|------|------|------|
| `ik_node/placo_ik_node.py` | guard_hit/fail_streak 後 seed 重試 | 在線版能逃出局部最小 |
| `ik_node/placo_ik_session.py` | j3 null-space repulsion | 主動避免 j3 貼邊 |

---

## 隱藏 Issue 摘要表

| # | Issue | 根因位置 | 嚴重程度 | 是否影響胸前奇異點 |
|:---:|---|---|:---:|:---:|
| 1 | j3 hard_hi = pref = 0.0，完全封死內旋 | `_HUMAN_RIGHT` | 🔴 高 | ✅ 直接原因 |
| 2 | per-joint weight 是 dead code（Bug 3） | `placo_ik_session.py` | 🔴 高 | ✅ 加劇 wrist 振盪 |
| 3 | Seed 庫缺胸前構型 + 在線版不用 seeds | `placo_ik_solver.py`, `placo_ik_session.py` | 🟡 中 | ✅ 無法自動找到逃出路徑 |
| 4 | _W_ORI=2.0 在奇異點時逼 wrist 超補 | `placo_ik_session.py` | 🟡 中 | ✅ 觸發 wrist 振盪 |
| 5 | Adaptive DLS 全局 damping，不識別 hard-limit 方向 | `placo_ik_session.py` | 🟡 中 | ⚠️ DLS 讓手臂「更凍」而非找繞道 |
| 6 | j3 pref 壓在 hard boundary，JointsTask 每步貼邊震盪 | `_HUMAN_RIGHT` | 🟡 中 | ✅ 造成細微震盪 |

---

*相關檔案：`ik_solver/placo_ik_solver.py`, `ik_node/placo_ik_session.py`, `ik_node/placo_ik_node.py`*  
*前置閱讀：[ik_solver_weights.md](ik_solver_weights.md), [ik_seeds_and_joints_task_weight.md](ik_seeds_and_joints_task_weight.md), [adaptive_dls.md](adaptive_dls.md)*
