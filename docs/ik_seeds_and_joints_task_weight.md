# IK Seeds 與 JointsTask Weight — OpenArm Placo IK 深度說明

> 給 IK solver 新手的完整解釋  
> 涵蓋：什麼是 seed、為何要多 seed、以及 Bug 3 / Bug 4 的根本原因

---

## Part 1 — 什麼是 IK Seed？

### 1.1 背景：IK 是一個非線性最佳化問題

逆向運動學（Inverse Kinematics, IK）的任務定義如下：

```
給定目標 end-effector 位置/姿態  →  求解 joint angles q = [q1, q2, q3, q4, q5, q6, q7]
```

這個問題**沒有封閉解（closed-form）**，因此 placo（以及 pinocchio、scipy 等）都用
**迭代式數值方法**求解：

```text
初始猜測 q₀  →  計算誤差 e  →  更新 Δq  →  q₁ = q₀ + Δq  →  重複直到 e < 門檻
```

**這個「初始猜測 q₀」就叫做 seed（種子）。**

---

### 1.2 為什麼 seed 選得不好，IK 會失敗或得到奇怪的解？

數值 IK 是**局部最佳化**，只能找到距離 seed 最近的局部最小值。

機械臂有**多個合法解**（redundancy + multiple configurations），例如：

```
目標：手伸到前方 X=0.40m, Y=-0.15m, Z=0.50m
合法解 A: j3 = -0.8 rad (elbow pointing down)   ← 自然
合法解 B: j3 = +1.2 rad (elbow pointing up)      ← 奇怪
```

如果 seed 恰好在解 B 附近，IK 就會收斂到「動作奇怪」的解。

**視覺化示意：**

```
 cost landscape (簡化成 2D)
      ^
 cost |  局部最小 B        局部最小 A (正確解)
      |    \/                   \/
      |          /\/\/\
      |_______________________________> q space
            ^seed_B              ^seed_A
```

若 seed 在 B 附近 → 落入 B → 手臂反折  
若 seed 在 A 附近 → 落入 A → 正確

---

### 1.3 多 Seed 策略

為了提升成功率，`PlacoIKSolver`（`placo_ik_solver.py`）實作了多 seed 重試：

```python
seeds = [list(last_joints)] + self._seeds   # 第一個 = 上一步的 joint 角度
                                             # 其餘 = 預定義的特徵姿態庫

for attempt, seed in enumerate(seeds):
    joints, pos_e = self._solve_from_seed(target, seed)
    if pos_e < POS_TOL:
        return True, joints, ms   # 成功！立即回傳
```

`_SEEDS_RIGHT` 和 `_SEEDS_LEFT` 就是這個「特徵姿態庫」：

```python
# placo_ik_solver.py
_SEEDS_RIGHT = [
    [ 0.00,  0.70,  0.00,  1.5708,  0.00,  0.00,  0.00],  # forward-reach 90°
    [ 0.00,  0.50,  0.00,  1.5708,  0.00,  0.00,  0.00],  # shoulder lower
    [ 0.00,  1.00,  0.00,  1.5708,  0.00,  0.00,  0.00],  # shoulder higher
    [ 0.20,  0.70, -0.20,  1.5708,  0.00,  0.00,  0.00],  # j1 inward
    [-0.20,  0.70, -0.20,  1.5708,  0.00,  0.00,  0.00],  # j1 outward
    [ 0.00,  0.80,  0.00,  2.0000,  0.00,  0.00,  0.00],  # elbow ~115°
    [ 0.00,  0.60,  0.00,  1.2000,  0.00,  0.30,  0.00],  # forearm roll
]

_SEEDS_LEFT = [
    [ 0.00, -0.70,  0.00,  1.5708,  0.00,  0.00,  0.00],  # forward-reach home 90°
    [ 0.00, -0.50,  0.00,  1.5708,  0.00,  0.00,  0.00],  # shoulder lower
    [ 0.00, -1.00,  0.00,  1.5708,  0.00,  0.00,  0.00],  # shoulder higher
    [-0.20, -0.70,  0.20,  1.5708,  0.00,  0.00,  0.00],  # j1 inward
    [ 0.20, -0.70,  0.20,  1.5708,  0.00,  0.00,  0.00],  # j1 outward
    [ 0.00, -0.80,  0.00,  2.0000,  0.00,  0.00,  0.00],  # elbow ~115°
    [ 0.00, -0.60,  0.00,  1.2000,  0.00,  0.30,  0.00],  # forearm roll
]
```

注意 LEFT 的 j2 全部是負值，對應 LEFT URDF 的鏡像軸方向。

---

## Part 2 — Bug 4：為什麼 PlacoSession 從未使用 Seeds？

### 2.1 兩種 Solver 的架構差異

| 特性 | `PlacoIKSolver` (舊版) | `PlacoSession` (在線版) |
|------|----------------------|----------------------|
| 位置 | `placo_ik_solver.py` | `placo_ik_session.py` |
| Seed 重試 | ✅ 有（最多 8 seeds） | ❌ 沒有（只有 1 seed） |
| RobotWrapper | 每 seed 次重建 | 每 step 重用（cache） |
| 每 solve 時間 | ~25ms | ~0.5ms |
| 線上遙控適用 | ❌ 太慢 | ✅ |
| 使用 `_SEEDS_*` | ✅ 有效 | ❌ 無效（定義了但未用） |

### 2.2 問題代碼路徑

在 `placo_ik_node.py` 的主控制迴路中：

```python
# 只有一個 seed — 上次的 filtered joint 狀態
with self._joints_lock:
    seed = list(self._filt_joints or self._last_joints)

r = self._placo_session.solve_step(
    target_xyz, target_R, seed, no_rot=...)
```

`PlacoSession.solve_step()` 的函式簽名：

```python
def solve_step(self, target_xyz, target_R, seed, no_rot=False) -> Dict:
    # ...
    for name, val in zip(self._joint_names, seed):   # ← 只用傳入的這一個 seed
        robot.set_joint(name, val)
    robot.update_kinematics()
    # IK iterations...
```

**`_SEEDS_LEFT` 和 `_SEEDS_RIGHT` 在 `placo_ik_solver.py` 裡定義，
但 `PlacoSession` 在 `placo_ik_session.py` 裡並沒有 import 或使用它們。**

### 2.3 為什麼在線版故意只用一個 seed？

原因是**速度**：

```
多 seed 策略（PlacoIKSolver）：
  最多 8 seeds × 25ms/seed = 200ms  →  不可能 50Hz

單 seed 策略（PlacoSession）：
  1 seed × 0.5ms  →  輕鬆 200Hz
```

在線遙控的關鍵觀察是：**相鄰兩步的目標位置非常接近（~5mm/step at 50Hz）**，
所以上一步的 joint 角度是本步極優秀的 seed — 幾乎不需要重試。

### 2.4 什麼時候 PlacoSession 的單 seed 會失敗？

| 情境 | 是否失敗 | 原因 |
|------|---------|------|
| 正常遙控（小步） | 很少 | 上步 joints ≈ 最優 seed |
| 接近奇異點（arm 伸直） | 偶爾 | Jacobian ill-conditioned，多 seed 也救不了 |
| **Elbow flip（構型突變）** | **常見** | 兩個解之間跳躍，joint_jump_guard 攔截 |
| 重啟後第一步 | 有時 | `_last_joints = home_joints` 可能不靠近目標 |
| 目標超出 workspace | 有時 | IK 無解，partial solution |

### 2.5 改進方向（供參考）

若要在在線版增加有限 seed 重試，只在首步（`is_new_session=True`）或 `guard_hit` 後觸發：

```python
# 改進草圖（未實作）
if is_new or self._fail_streak > 5:
    seeds_to_try = [seed] + _ARM_SEEDS[self.args.arm][:3]
else:
    seeds_to_try = [seed]

for s in seeds_to_try:
    r = self._placo_session.solve_step(target_xyz, target_R, s, ...)
    if r["success"]:
        break
```

代價：新 session 首步可能多花 3 × 0.5ms = 1.5ms（仍遠低於 20ms budget）。

---

## Part 3 — Bug 3：_W_JOINTS 全局常數，per-joint weight 是 Dead Code

### 3.1 問題定義

`_HUMAN_RIGHT` 和 `_HUMAN_LEFT` 中每個關節的 tuple 格式是：

```python
(pref_angle, hard_lo, hard_hi, naturalness_weight)
#   index[0]   index[1]  index[2]    index[3] ← ⚠️ 從未被讀取
```

例如：
```python
_HUMAN_RIGHT = {
    "openarm_right_joint1": (0.000,  -1.396,  1.500,  0.15),   # weight=0.15
    "openarm_right_joint2": (0.700,   0.000,  1.600,  0.25),   # weight=0.25
    "openarm_right_joint3": (0.000,  -1.571,  0.000,  0.80),   # weight=0.80 ← 大!
    "openarm_right_joint4": (1.5708,  0.250,  2.200,  0.08),   # weight=0.08
    "openarm_right_joint5": (0.000,  -1.571,  1.571,  0.03),   # weight=0.03
    "openarm_right_joint6": (0.000,  -0.785,  0.785,  0.03),   # weight=0.03
    "openarm_right_joint7": (0.000,  -1.571,  1.571,  0.03),   # weight=0.03
}
```

### 3.2 代碼完整追蹤

**步驟 1：`PlacoSession.__init__` 讀取 human_cfg**

```python
# placo_ik_session.py  第 127-130 行
human_cfg = _HUMAN_RIGHT if arm == "right" else _HUMAN_LEFT
self._lo   = [human_cfg[n][1] for n in self._joint_names]   # ✅ index[1] 使用
self._hi   = [human_cfg[n][2] for n in self._joint_names]   # ✅ index[2] 使用
self._pref = {n: human_cfg[n][0] for n in self._joint_names} # ✅ index[0] 使用
#                             ↑ index[3] 從沒出現過
```

**步驟 2：`solve_step` 設定 JointsTask**

```python
# placo_ik_session.py  solve_step() 內
jt = solver.add_joints_task()
jt.set_joints(self._pref)                          # ← dict {name: pref_value}
jt.configure("naturalness", "soft", _W_JOINTS)    # ← 全局常數 5e-4，所有關節共用!
```

**步驟 3：placo JointsTask API 限制**

placo 的 `JointsTask` 只有一個 scalar weight：

```
add_joints_task()
  ├── set_joints(dict)        → 設定所有關節的目標角度
  └── configure(name, type, weight)   → 設定「整個 task」的一個 weight
```

**數學上等同於：**

$$\text{cost} = w \cdot \sum_{i=1}^{7} (q_i - q_i^{pref})^2$$

所有 7 個關節用同樣的 $w = 5 \times 10^{-4}$，
而你在 tuple 裡設定的 `0.80`（j3 高偏好）完全不生效。

### 3.3 若要真正實現 per-joint weight，需要什麼？

必須對每個關節建立獨立的 `JointsTask`：

```python
# 真正的 per-joint weight（目前未實作）
for name in self._joint_names:
    per_joint_weight = human_cfg[name][3]     # index[3] 終於被讀取
    jt_i = solver.add_joints_task()
    jt_i.set_joints({name: self._pref[name]})
    jt_i.configure(f"nat_{name}", "soft", per_joint_weight * _W_JOINTS_BASE)
```

**代價：**
- QP 矩陣從 `(N_task × N_joint)` 增加到 `(N_task + 7) × N_joint`
- 每 step 多 7 次 `configure()` call — 影響極小（< 0.01ms）
- 需驗證多 JointsTask 不會讓 QP ill-conditioned

### 3.4 為何現在 j3 weight=0.80 「感覺有效」但其實沒有？

在 `_HUMAN_RIGHT` 中 j3 pref=0.000（右臂到正前方偏好 j3 在 0 附近），
而且 j3 URDF limit 允許 [-90°, +90°]，solver 自然傾向把 j3 留在中間。
這讓 j3 的行為「看起來符合預期」，但原因是 IK 任務本身的幾何特性，
而非 weight=0.80 的作用。

---

## 總結

| 項目 | 現狀 | 根因 | 影響 |
|------|------|------|------|
| **Per-joint weight** | Dead code（index[3] 未讀） | placo JointsTask 只有一個 scalar w | 無法對不同關節設定差異化的 naturalness |
| **Seeds（在線版）** | `_SEEDS_LEFT/RIGHT` 定義但從未使用 | PlacoSession 為速度只用單 seed | elbow flip 後無自動恢復；新 session 首步有失敗風險 |
| **Seeds（舊版）** | `PlacoIKSolver` 有效使用 | —— | 舊版 25ms/step，不適合在線 50Hz |

---

*最後更新：2026-05-20*  
*相關檔案：`placo_ik_solver.py`, `placo_ik_session.py`, `placo_ik_node.py`*
