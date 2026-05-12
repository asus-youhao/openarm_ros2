# VR 即時控制 IK Solver 深度分析報告

**分析對象：** `placo_ik_online_profiler_ws_mesh.py` + `ik_solver/placo_ik_solver.py`  
**對照指標：** VR 即時控制 IK Solver 關鍵指標（4 大類）

---

## 摘要總評

| 指標類別 | 對齊狀況 | 主要問題 |
|----------|----------|----------|
| 1. 效能與即時性 | ✅ 已大幅優化 | Python overhead 有限，無 C++ 解析導數 |
| 2. 軌跡連續性 | ✅ Warm Start 已實作 | 多解跳變無明確處理 |
| 3. 奇異點與穩定性 | ⚠️ 部分對齊 | DLS 可配置但依賴 Placo 內部，OOR 處理不完整 |
| 4. 約束與任務優化 | ✅ 多任務架構已建立 | 無 Null Space Projection，WBC 不完整 |

---

## 一、效能與即時性 (Real-time Performance)

### 指標要求
- Latency 低（< 5ms 理想，< 20ms 可接受）
- 高確定性（低 Jitter）
- 優先使用有解析導數的 C++ 庫（如 Pinocchio）

### 程式碼實際數值

#### 1.1 IK 時間拆解（`PlacoSession.solve_step()`）

```python
# ── time budget per step ─────────────────────────────────
robot_build   ≈  0.001 ms  (cached 模式 ← __init__ 已建立)
              ≈  5-10  ms  (rebuild 模式 ← 原版行為)
setup_ms      ≈  0.10-0.20 ms   # KinematicsSolver + 4 tasks 初始化
loop_ms       ≈  0.3-1.5  ms   # 1-3 iterations × ~0.08ms/iter (early exit 後)
─────────────────────────────────────────────────────────
total ≈ 0.5-2 ms (cached+early_exit) vs. ≈25ms (rebuild +250iters)
```

| 方式 | robot_ms | setup_ms | loop_ms | total | 可達 Hz |
|------|----------|----------|---------|-------|---------|
| **cached + early exit** (本檔預設) | ~0.001 | ~0.15 | ~0.5 | **~0.7 ms** | **>500 Hz** |
| rebuild + no-early-exit (原版) | ~5-10 | ~0.15 | ~20 | **~25 ms** | **~40 Hz** |

#### 1.2 deadline 監控

```python
_deadline_ms = 1000.0 / self._rate_hz   # = 20ms @ 50Hz

deadline_missed = int(loop_wall_ms > self._deadline_ms)
```

- 控制週期預設：`--rate 50.0 Hz` → budget = **20 ms**
- 在 cached 模式下，每步 ~0.7ms 遠低於 budget

#### 1.3 解析導數 vs 數值導數

| 項目 | 本系統 |
|------|--------|
| 函式庫 | **Placo（Python binding → C++）** |
| 雅可比計算 | Placo 內部計算（C++），Python 層只呼叫 `.solve(True)` |
| 導數類型 | **解析導數**（Placo 使用 Pinocchio 計算精確雅可比） |
| vs. Pinocchio 直接使用 | 相比直接呼叫 Pinocchio 多一層 Python binding overhead |

> ✅ **對齊**：Placo 底層使用 Pinocchio 解析雅可比，非數值差分。  
> ⚠️ **限制**：Python binding 造成固定 overhead（setup ~0.15ms），遠不如 pure C++ pipeline（<0.01ms）。

---

## 二、軌跡連續性 (Continuity & Smoothing)

### 指標要求
- **Warm Start**：必須以上一次關節位置作為初值
- **跳變抑制**：避免多解性造成關節構型突然跳變

### 2.1 Warm Start 實作 ✅

```python
# ── seed = 上一步的關節值 ──────────────────────────────────
with self._joints_lock:
    seed = list(self._last_joints)   # ← 每步使用上一步關節值

r = self._placo_session.solve_step(target_xyz, target_R, seed)

if r["success"]:
    with self._joints_lock:
        self._last_joints = r["joints"]    # ← 成功才更新
```

```python
# 在 solve_step() 中，seed 直接設定為初始關節角
for name, val in zip(self._joint_names, seed):
    robot.set_joint(name, val)
robot.update_kinematics()
```

> ✅ **完全對齊**：強制使用上一步關節值做 warm start，失敗時不更新 seed（保持穩定）。

### 2.2 IK 失敗時的回退策略

```python
if r["success"]:
    self._last_joints = r["joints"]
    self._publish(r["joints"])
else:
    pass    # ← IK 失敗 — 不更新 pose，不發布，保持 last_joints 作為下一輪 seed
```

> ✅ IK 失敗不送指令，不更新 seed，safe fallback。

### 2.3 多解跳變抑制 ⚠️

**目前狀況：**
```python
# 單一 seed（last_joints），沒有多 seed 嘗試
seed = list(self._last_joints)
r = self._placo_session.solve_step(target_xyz, target_R, seed)
```

`PlacoSession.solve_step()` 不做多 seed 嘗試（PlacoIKSolver 才有 `_seeds` 列表）。

| 項目 | 狀態 |
|------|------|
| Warm start from last_joints | ✅ |
| 多 seed 嘗試（防止陷入局部解） | ❌ 本檔未實作 |
| 跳變偵測（關節速度 / 角度差 threshold） | ❌ 未實作 |
| 跳變發生時 fallback | ❌ 未實作 |

**改善建議：**
```python
# 加入跳變量偵測
MAX_JOINT_DELTA_DEG = 15.0   # 單步最大允許關節角變化
prev = np.array(self._last_joints)
new  = np.array(r["joints"])
if np.degrees(np.max(np.abs(new - prev))) > MAX_JOINT_DELTA_DEG:
    # 跳變 → 拒絕這次解，保持舊 joints
    pass
```

---

## 三、奇異點與數值穩定性 (Singularity Handling & Robustness)

### 指標要求
- **阻尼機制（DLS）**：防止奇異點產生無限大速度
- **OOR 處理**：Out of reach 時回傳最接近的物理合理值

### 3.1 正則化任務（RegularizationTask）← DLS 等效 ✅

```python
# 在 solve_step() 中
reg = solver.add_regularization_task(1e-5)
reg.configure("reg", "soft", 1.0)
```

Placo 的 `RegularizationTask` 在雅可比矩陣 QP 中加入 $\lambda \|q\|^2$ 懲罰項，效果等同於 **Damped Least Squares (DLS)**：

$$\dot{q} = J^T(JJ^T + \lambda I)^{-1} \dot{x}$$

| 參數 | 數值 | 說明 |
|------|------|------|
| `reg weight` | `1e-5` | DLS 阻尼係數 λ（越大越保守）|
| `reg.configure` | `"soft", 1.0` | Soft constraint，不強制零速度 |

> ✅ **對齊**：RegularizationTask 提供奇異點附近的阻尼，防止速度爆炸。  
> ⚠️ **限制**：λ=1e-5 是全局固定值，非 **自適應 DLS**（奇異點附近應動態提高 λ）。

### 3.2 OOR（Out of Reach）處理 ⚠️

```python
# 成功判斷（pos_err < 10mm 才算 success）
_POS_RELAX = 0.010   # m = 10mm

success = int(pos_err < _POS_RELAX)
```

**問題**：OOR 時 `success=0`，但仍回傳 `r["joints"]`（最後一次迭代的結果）。在上層：

```python
if r["success"]:
    self._publish(r["joints"])
else:
    pass    # OOR 時什麼都不做 ← 不送指令，保持靜止
```

OOR 時，手臂**靜止不動**（保持上一個有效位置），而非移動到「最接近工作空間邊界的點」。

| 行為 | 本系統 | 指標要求 |
|------|--------|---------|
| OOR 時不送壞指令 | ✅ | ✅ |
| OOR 時移到最近可達點 | ❌（靜止） | ✅ |
| WorkspaceMesh Clamp 提前防止 OOR | ✅ | ✅ |

**WorkspaceMesh 是 OOR 的主要保護：**
```python
if self._ws_mesh is not None:
    # KDTree 查詢最近可達 voxel → 目標永遠在 workspace 內 → 幾乎不會 OOR
    clamped, _inside = self._ws_mesh.clamp(np.array([raw_x, raw_y, raw_z]))
```

> ✅ 前端 clamp 保障目標在 workspace 內，大幅減少 OOR 發生機率。  
> ⚠️ 一旦 OOR 仍發生（方向任務衝突），選擇是靜止而非回傳邊界解。

### 3.3 關節極限 ✅

```python
solver.enable_joint_limits(True)    # Placo 內建 joint limit 軟約束

# 最終後處理 clip（保險層）
joints = [max(l, min(h, q)) for q, l, h in zip(joints, self._lo, self._hi)]
```

| 關節 | 下限 (rad) | 上限 (rad) | 偏好值 (rad) |
|------|------------|------------|--------------|
| joint1（肩 yaw） | -1.396 | +1.500 | 0.000 |
| joint2（肩 pitch） | 0.000 | +1.600 | +0.700 |
| joint3（上臂 yaw） | -1.571 | 0.000 | 0.000 |
| joint4（肘 flex） | 0.250 | +2.200 | +1.5708 |
| joint5（前臂 roll） | -1.571 | +1.571 | 0.000 |
| joint6（腕 yaw） | -0.785 | +0.785 | 0.000 |
| joint7（腕 pitch） | -1.571 | +1.571 | 0.000 |

> ✅ **雙重保護**：Placo enable_joint_limits + Python post-processing clip。

---

## 四、約束與任務優化 (Constraints & Task Priority)

### 指標要求
- **邊界軟約束**：Joint Limits 整合進優化器
- **多任務控制（WBC）**：Null Space Projection 處理主/次任務

### 4.1 多任務架構（Task-based QP）✅

```python
# Task 1: 位置追蹤（主任務）
pos_task = solver.add_position_task(self._ee_link, target_xyz)
pos_task.configure("pos", "soft", _W_POS)           # weight = 1.0

# Task 2: 方向追蹤（次任務）
ori_task = solver.add_orientation_task(self._ee_link, target_R)
ori_task.configure("ori", "soft", _W_ORI)           # weight = 0.3

# Task 3: 關節自然性偏好（tertiary）
jt = solver.add_joints_task()
jt.set_joints(self._pref)
jt.configure("naturalness", "soft", _W_JOINTS)      # weight = 1e-4

# Task 4: 正則化（singularity guard）
reg = solver.add_regularization_task(1e-5)
reg.configure("reg", "soft", 1.0)
```

| Task | 類型 | Weight | 自由度消耗 |
|------|------|--------|-----------|
| PositionTask (EE xyz) | Soft | **1.0** | 3 DoF |
| OrientationTask (EE R) | Soft | **0.3** | 3 DoF |
| JointsTask (naturalness) | Soft | **1e-4** | 7 DoF（全關節）|
| RegularizationTask | Soft | **1.0** | — (λ項) |

**Tasks 有隱式優先順序**：weight 差距 = 1.0 vs 1e-4 ≈ **10000×**，position task 在數值上主導優化。

### 4.2 Weight 設計分析

| 參數 | 數值 | 設計意圖 |
|------|------|---------|
| `_W_POS = 1.0` | 最高優先 | EE 位置精度 |
| `_W_ORI = 0.3` | 中等（位置的 30%）| 允許方向退讓 |
| `_W_JOINTS = 1e-4` | 最低（位置的 0.01%）| 自然性偏好不干擾 IK 主任務 |
| `reg = 1e-5` | DLS 阻尼 | 奇異點保護 |

> ✅ **設計合理**：JointsTask weight 遠小於 PositionTask，不會阻礙 IK 收斂。

### 4.3 Null Space Projection 分析 ❌

**指標要求**：透過 Null Space Projection 處理主追蹤任務與次要避障/姿態任務。

**本系統狀況**：

Placo 的 `soft` 任務架構是 **加權最小二乘（WLS）**，不是嚴格的 Null Space Projection：

```
Null Space Projection（嚴格）:
  q̇ = J⁺ẋ + (I - J⁺J) q̇_secondary
  ← 次任務完全在主任務 null space 執行，不影響主任務

本系統（Placo Weight QP）:
  min  W₁‖J₁q̇ - ẋ₁‖² + W₂‖J₂q̇ - ẋ₂‖² + ...
  ← 所有任務同時競爭，透過 weight 分配優先順序
  ← 當自由度足夠時近似等同 Null Space，但不保證
```

| 特性 | Null Space（嚴格） | 本系統（WLS）|
|------|-------------------|-------------|
| 主任務不受次任務影響 | ✅ 保證 | ❌ 不保證（weight ratio 控制）|
| 次任務在 null space 最大化 | ✅ | 近似（weight 比 >>1 時收斂）|
| 實作複雜度 | 高 | 低 |
| 計算量 | O(n³) SVD | QP（OSQP/qpOASES）|

> ⚠️ **未完全對齊**：使用 WLS，非嚴格 WBC/Null Space Projection。對 7-DoF 臂（冗餘一個自由度），在實際操作中差異較小，但理論上不等效。

### 4.4 Session ref 機制（ee_delta 累積）

```python
# ee_delta session reference（重要：防止累積誤差）
self._ee_delta_gap_sec = 0.35   # 350ms 無訊號 → 強制重設 ref

if is_new:
    ref = self._get_tf(0.3)     # 從 TF2 取得真實 EE 位置
    if ref is not None:
        self._ee_delta_ref_xyz = tuple(ref[:3])   # anchor = 真實 TF 位置
```

> ✅ gap > 350ms 重設 ref 至 TF 讀值，防止 delta 累積漂移。

---

## 五、缺口清單與改善優先順序

### 🔴 高優先（影響安全 / 穩定性）

| # | 問題 | 描述 | 建議改善 |
|---|------|------|---------|
| 1 | **跳變偵測缺失** | 多解 IK 可能造成關節角突變（>30°/step），馬達過電流 | 加入 `MAX_JOINT_DELTA` threshold，超過拒絕解 |
| 2 | **OOR 靜止策略** | OOR 時靜止，但若 WorkspaceMesh 未提前 clamp，連續靜止→delay | OOR 時送「上一步 joints」維持，並輸出 OOR 指標 |

### 🟡 中優先（影響 VR 沉浸感）

| # | 問題 | 描述 | 建議改善 |
|---|------|------|---------|
| 3 | **固定阻尼 λ** | `reg=1e-5` 全局固定，奇異點附近不夠大 | 計算 min singular value，動態調整 λ |
| 4 | **無方向任務 Null Space** | JointsTask 與 OriTask 同時競爭，近奇異點時姿態可能不自然 | 改用 Pinocchio 直接計算 null space basis |
| 5 | **Python loop overhead** | `_IIR2.update()` 等 Python 計算在 main loop 中 | 移至獨立 thread 或 C extension |

### 🟢 低優先（錦上添花）

| # | 問題 | 描述 | 建議改善 |
|---|------|------|---------|
| 6 | **Velocity Limits 未啟用** | `enable_velocity_limits(False)` | 評估是否需要關節速度限制 |
| 7 | **無加速度 / Jerk 限制** | JointTrajectory 只有位置，無速度/加速度約束 | 加入速度控制 horizon |

---

## 六、數值參數速查表

```
═══════════════════════════════════════════════════════════════════════
  IK Solver 參數
═══════════════════════════════════════════════════════════════════════
  _POS_TOL    = 0.003 m   (3mm)  — early-exit 門檻
  _POS_RELAX  = 0.010 m   (10mm) — success 判斷
  _W_POS      = 1.0               — PositionTask weight
  _W_ORI      = 0.3               — OrientationTask weight  (位置的 30%)
  _W_JOINTS   = 1e-4              — JointsTask naturalness  (位置的 0.01%)
  reg         = 1e-5              — RegularizationTask (DLS λ)
  _MAX_ITER   = 250               — 最大迭代數（early exit 通常 1-5）
  _DT         = 0.010 s           — Placo 速度積分步長
═══════════════════════════════════════════════════════════════════════
  控制迴路參數
═══════════════════════════════════════════════════════════════════════
  --rate     = 50  Hz → deadline = 20 ms
  --horizon  = 60  ms → JointTrajectory time_from_start
  gap_sec    = 0.35 s  → ee_delta session ref 重設間隔
  home_tol   = 0.025 m (2.5cm) — home 確認距離
═══════════════════════════════════════════════════════════════════════
  WorkspaceMesh clamp
═══════════════════════════════════════════════════════════════════════
  method     = KDTree 1-NN        — O(log N)
  latency    ≈ 8 µs               — ~0.04% of 20ms budget
  fallback   = box clamp          — O(1), 0.35µs
═══════════════════════════════════════════════════════════════════════
  Joint limits (right arm)
═══════════════════════════════════════════════════════════════════════
  Joint 1  [-1.396, +1.500] rad  pref=0.000  (肩 yaw)
  Joint 2  [ 0.000, +1.600] rad  pref=0.700  (肩 pitch)
  Joint 3  [-1.571,  0.000] rad  pref=0.000  (上臂 yaw)
  Joint 4  [ 0.250, +2.200] rad  pref=1.5708 (肘 flex)
  Joint 5  [-1.571, +1.571] rad  pref=0.000  (前臂 roll)
  Joint 6  [-0.785, +0.785] rad  pref=0.000  (腕 yaw)
  Joint 7  [-1.571, +1.571] rad  pref=0.000  (腕 pitch)
═══════════════════════════════════════════════════════════════════════
  預期效能（cached + early exit）
═══════════════════════════════════════════════════════════════════════
  robot_build  ≈ 0.001 ms  (one-time ~5ms at startup)
  setup_ms     ≈ 0.15  ms  (KinematicsSolver + 4 tasks)
  loop_ms      ≈ 0.3-1 ms  (1-3 iters × ~0.08ms/iter)
  total IK     ≈ 0.5-2 ms  → margin = 18ms at 50Hz
  deadline miss ≈ 0%       (50Hz)
```

---

## 七、指標對齊總結圖

```
指標                              本系統狀態         評分
─────────────────────────────────────────────────────────────
1. 效能與即時性
   ├─ Latency < 5ms              ✅ ~0.7ms cached     ★★★★★
   ├─ 低 Jitter                  ✅ early exit 穩定    ★★★★☆
   └─ 解析導數 (Placo/Pinocchio)  ✅ Placo C++ binding  ★★★★☆

2. 軌跡連續性
   ├─ Warm Start                 ✅ last_joints seed   ★★★★★
   ├─ IK 失敗不送指令             ✅ success guard      ★★★★★
   └─ 跳變偵測/抑制               ❌ 未實作             ★★☆☆☆

3. 奇異點處理
   ├─ DLS (RegularizationTask)   ✅ λ=1e-5             ★★★★☆
   ├─ 自適應阻尼                  ❌ 固定 λ             ★★☆☆☆
   ├─ Joint Limits               ✅ 雙重保護            ★★★★★
   └─ OOR 回傳邊界解              ⚠️ 靜止（WS clamp補償）★★★☆☆

4. 約束與任務優化
   ├─ 多任務 QP                   ✅ 4-task WLS         ★★★★☆
   ├─ Joint Limits in optimizer  ✅ enable_joint_limits ★★★★★
   └─ Null Space Projection      ❌ WLS 近似，非嚴格 WBC ★★☆☆☆
─────────────────────────────────────────────────────────────
整體對齊度                                              ★★★★☆ (3.5/5)
```
