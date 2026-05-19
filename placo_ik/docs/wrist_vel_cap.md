# Wrist Velocity Cap — Joint 5/6/7 Teleop 平滑機制

> **實作檔案**：`ik_node/placo_ik_session.py`
> **相關常數**：`_WRIST_VEL_CAP`, `_WRIST_JOINT_IDX`
> **CLI flag**：`--wrist-vel-cap`

---

## 目錄

1. [問題根因：URDF wrist 速度上限太鬆](#1-問題根因urdf-wrist-速度上限太鬆)
2. [Placo `enable_velocity_limits` 的底層方法](#2-placo-enable_velocity_limits-的底層方法)
3. [實作設計](#3-實作設計)
4. [數值對照表](#4-數值對照表)
5. [調參指南](#5-調參指南)
6. [副作用與權衡](#6-副作用與權衡)
7. [與其他平滑機制的關係](#7-與其他平滑機制的關係)
8. [CLI 使用範例](#8-cli-使用範例)

---

## 1. 問題根因：URDF wrist 速度上限太鬆

OpenArm v10_o6 的 URDF 為 joint 5/6/7（前臂滾、腕偏、腕俯）寫的 `<limit velocity="...">`：

| Joint | URDF `velocity` (rad/s) | (°/s) | 用途 |
|---|---:|---:|---|
| j1 / j2（肩） | 16.75 | 960 | 大馬達快速移動 |
| j3 / j4（上臂 / 肘） | 5.45 | 312 | 中段 |
| **j5 / j6 / j7（wrist）** | **20.94** | **1200** | **廠商給的硬體上限** |

廠商 URDF 寫的是「**馬達硬體可達到的最大速度**」，目的是讓 motion planner 知道物理上限。**這個值不適合直接拿來做 VR teleop 的單步飽和**。

### 為什麼 wrist 抖動最嚴重

1. **慣量小**：wrist 三軸的負載最小，馬達反應最快
2. **冗餘解的 sink**：7-DoF 中，position task 主導，orientation task 權重 0.3 → 剩餘自由度都在 wrist 上
3. **VR tracker quaternion 雜訊大**：rotation 端訊號雜訊本來就比平移高
4. **URDF cap 太大**：vel_limit 形同虛設 → IK 噪聲直接 passthrough 到馬達

在 20Hz 控制（dt=50ms）下，**單步 wrist 允許翻 60°** —— 比人類意圖快數十倍。

---

## 2. Placo `enable_velocity_limits` 的底層方法

### API 介面

```
KinematicsSolver:
  ├── dt                     (float, time step)
  ├── enable_joint_limits    (bool)
  └── enable_velocity_limits (bool)   ← 開關，沒有 method 選項

RobotWrapper:
  ├── set_velocity_limit(name, v)     ← 單關節覆寫
  ├── set_velocity_limits(dict)       ← 多關節覆寫
  └── get_joint_velocity(name)
```

### QP 數學形式

每一步 IK 求解：

$$
\min_{\dot q} \;\;\sum_k W_k\|J_k\dot q - v_k\|^2 + \lambda\|\dot q\|^2
$$

subject to **盒型不等式約束**：

$$
-v_{\max,i} \;\le\; \dot q_i \;\le\; +v_{\max,i}, \quad i = 1\dots n
$$

加上 `solver.dt`：

$$
|\Delta q_i| = |\dot q_i \cdot dt| \le v_{\max,i} \cdot dt
$$

QP solver（OSQP / qpOASES）把 $v_{\max,i}$ 當**硬約束**強制執行 —— 不是後處理 clip，而是在 feasible polytope 上求最佳。

**資料源**：Placo 在 `RobotWrapper` 建構時從 URDF 讀 `<limit velocity="...">`。可以用 `set_velocity_limit(name, v)` runtime 覆寫。

---

## 3. 實作設計

### 常數定義（`placo_ik_session.py`）

```python
# ── Wrist velocity cap (teleop smoothness) ────────────────────────────────────
# URDF default wrist (j5/j6/j7) velocity = 20.94 rad/s ≈ 1200°/s — far too fast
# for VR teleop, lets per-step IK noise pass straight through to the motors.
# At 100Hz dt=10ms: 4.0 rad/s → 2.3°/step  (vs URDF: 12°/step).
_WRIST_VEL_CAP    = 4.0    # rad/s; 0 or negative → disabled (use URDF default)
_WRIST_JOINT_IDX  = (4, 5, 6)   # 0-based indices for joint 5, 6, 7
```

### PlacoSession 整合

```python
def __init__(self, ..., wrist_vel_cap: float = _WRIST_VEL_CAP):
    self._wrist_vel_cap     = float(wrist_vel_cap)
    self._wrist_joint_names = [self._joint_names[i] for i in _WRIST_JOINT_IDX]

    if not rebuild:
        self._robot = placo.RobotWrapper(urdf, ...)
        self._apply_velocity_caps(self._robot)    # ← cached path
    # rebuild path 的 robot 在 solve_step 內建構，那裡也會呼叫
```

### 套用方法

```python
def _apply_velocity_caps(self, robot) -> None:
    """Override URDF wrist velocity limits.  No-op if cap <= 0."""
    if self._wrist_vel_cap <= 0.0:
        return
    for name in self._wrist_joint_names:
        robot.set_velocity_limit(name, self._wrist_vel_cap)
```

### Rebuild 模式也要套

```python
# solve_step():
if self._rebuild or self._robot is None:
    robot  = placo.RobotWrapper(self._urdf, ...)
    self._apply_velocity_caps(robot)    # ← rebuild path
    cached = False
```

兩條建構路徑都套，避免 rebuild 模式下 wrist cap 失效。

### CLI flag（`placo_ik_online_profiler_ws_mesh.py`）

```python
p.add_argument("--wrist-vel-cap", type=float, default=4.0, dest="wrist_vel_cap",
               help="Wrist (joint5-7) velocity cap in rad/s for teleop smoothness. "
                    "URDF default = 20.94 rad/s (1200°/s) lets IK noise pass through. "
                    "Default 4.0 rad/s ≈ 230°/s.  0 or negative = use URDF default.")
```

### 啟動 banner

```
[PlacoSession] cached  arm=right  max_iter=15  dt=10.0ms  vel_limits=True  wrist≤4.0rad/s
```

---

## 4. 數值對照表

### 單步允許 Δq（per-step max joint change）

| 控制週期 | URDF 20.94 rad/s | **cap = 4.0 rad/s** | cap = 2.5 rad/s | cap = 8.0 rad/s |
|---|---:|---:|---:|---:|
| 1 ms (1000Hz) | 1.2° | 0.23° | 0.14° | 0.46° |
| 10 ms (100Hz) | 12.0° | **2.3°** | 1.4° | 4.6° |
| 20 ms (50Hz)  | 24.0° | 4.6° | 2.9° | 9.2° |
| 50 ms (20Hz)  | 60.0° | 11.5° | 7.2° | 22.9° |

### 與其他機制比較（@ 100Hz dt=10ms）

| 機制 | 對單步 Δq_wrist 的影響 | 性質 |
|---|---|---|
| URDF velocity（無覆寫）| 12.0° | QP 硬約束（廠商上限）|
| `_WRIST_VEL_CAP = 4.0` | 2.3° | QP 硬約束（teleop 友善）|
| LPF α=0.5（輸出端）| 雜訊衰減 50% | 後處理濾波（非硬切）|
| Adaptive DLS λ_max=1e-2 | 奇異點附近壓低 Δq | QP regularization（軟）|
| `success` gate (POS_RELAX=10mm) | 失敗時不 publish → 凍結 | 應用層 |

---

## 5. 調參指南

### 推薦範圍

| Cap (rad/s) | (°/s @ wrist) | 適用情境 |
|---:|---:|---|
| 0 (停用) | 1200 | 對照基線、馬達 stress test |
| 8.0 | 460 | 大幅度操作優先（傾倒、揮動）|
| **4.0**（預設）| **230** | **一般 VR teleop** |
| 2.5 | 143 | 細活（縫合、寫字、夾鑷）|
| 1.5 | 86 | 極致平滑，會明顯感覺 wrist lag |

### 怎麼選

1. **先跑預設 4.0** 看是否解掉 wrist 高頻抖動
2. 如果 **CSV `pos_err_mm` 明顯升高**（>5mm 中位數）→ cap 太小，IK 達不到目標 → 放寬到 5–6
3. 如果 **VR 中還能感覺 wrist 雜訊** → cap 縮到 2.5
4. 如果做大動作時 **wrist 明顯跟不上手** → cap 放寬到 6–8

### 觀察指標

```bash
# 1. CSV 中 pos_err_mm 分布
python3 -c "import csv,numpy as np; rows=list(csv.DictReader(open('placo_online_*.csv'))); \
            err=[float(r['pos_err_mm']) for r in rows]; \
            print(f'median={np.median(err):.2f}mm  p95={np.percentile(err,95):.2f}mm  max={max(err):.2f}mm')"

# 2. /right/placo_profile topic 的 pos_err_mm 趨勢
ros2 topic echo /right/placo_profile | jq '.pos_err_mm'
```

---

## 6. 副作用與權衡

### Trade-off 矩陣

| Cap 越小（嚴格）| Cap 越大（寬鬆）|
|---|---|
| ✅ wrist 抖動小 | ✅ wrist tracking 跟得上意圖 |
| ✅ 馬達電流穩定 | ✅ IK 收斂快 |
| ❌ 大幅旋轉 lag | ❌ 高頻雜訊 passthrough |
| ❌ pos_err 可能升高 | ❌ 馬達電流大 spike |
| ❌ 收斂可能需要更多 iter | |

### 可能出現的失敗模式

1. **`pos_err_mm` 持續 > POS_RELAX (10mm)** → success=0 → 軌跡凍結
   - 原因：cap 太小，wrist 達不到目標朝向
   - 解：放寬 cap 或降低 `_W_ORI`
2. **`iterations` 達到 `_MAX_ITER` 上限** → QP 收斂變慢
   - 原因：cap 縮限了 feasible region，需要更多步往 boundary 走
   - 解：提高 `_MAX_ITER` 或放寬 cap
3. **大旋轉時 wrist visible lag**
   - 原因：cap 物理飽和（這是設計上的取捨）
   - 解：放寬 cap，或在 VR app 端對「**意圖旋轉量**」做提示

### 不影響的東西

- 不影響 j1–j4 的反應速度（只動 wrist）
- 不影響 position tracking 精度（只在 wrist 飽和時才會間接影響 ori）
- 不會增加 IK 計算成本（`set_velocity_limit` 只在 robot 建構時呼叫一次）

---

## 7. 與其他平滑機制的關係

### 平滑機制堆疊

```
VR Tracker (40 Hz)
   ↓
 [Input LPF on quat delta]       ← QuaternionEmaFilter, --input-lpf-alpha
   ↓
 [Soft clamp / ws_mesh]          ← SoftClamp, --boundary-margin
   ↓
 IK Solver (Placo)
   ├── enable_joint_limits       ← URDF (硬)
   ├── enable_velocity_limits    ← URDF + 【本機制覆寫 wrist】← 物理層最後防線
   ├── add_regularization_task   ← Adaptive DLS (奇異點)
   └── soft tasks (pos/ori/jt)
   ↓
 [Joint LPF on output]           ← _filter_joints, --lpf-alpha
   ↓
 JointTrajectory / forward_position_controller
   ↓
 ros2_control hardware
```

### 各層職責

| 層 | 機制 | 治什麼 |
|---|---|---|
| 輸入端 | Quaternion LPF | tracker 雜訊源頭 |
| 目標空間 | Soft clamp | 邊界 OOR 跳變 |
| **IK QP** | **Wrist vel cap** | **單步 wrist 爆衝（本文件）** |
| IK QP | Adaptive DLS | 奇異點 wobble |
| 輸出端 | Joint LPF | IK 數值噪聲 |

**特別關係**：
- 和 **Joint LPF 互補**：LPF 平滑連續解之間的數值雜訊（高頻），vel cap 阻止單步爆衝（峰值）
- 和 **Adaptive DLS 互補**：DLS 在奇異點軟化 Δq，vel cap 是硬上限
- **不會疊加破壞**：QP 仍會找滿足 cap 的最佳解；LPF 在 cap 之後再過濾

---

## 8. CLI 使用範例

```bash
# 預設（cap = 4.0 rad/s）
python3 placo_ik_online_profiler_ws_mesh.py --arm right

# 停用 cap，看 baseline
python3 placo_ik_online_profiler_ws_mesh.py --arm right --wrist-vel-cap 0

# 細活模式
python3 placo_ik_online_profiler_ws_mesh.py --arm right --wrist-vel-cap 2.5

# 結合 output LPF（推薦組合）
python3 placo_ik_online_profiler_ws_mesh.py --arm right \
    --wrist-vel-cap 4.0 \
    --lpf-alpha 0.9,0.9,0.9,0.9,0.4,0.4,0.4

# A/B 對照（dry-run 不送指令，看 CSV）
python3 placo_ik_online_profiler_ws_mesh.py --arm right --dry-run --wrist-vel-cap 0
python3 placo_ik_online_profiler_ws_mesh.py --arm right --dry-run --wrist-vel-cap 4.0
```

---

## 附錄：如果想擴充

### 想對其他 joint 也加 cap

修改 `_WRIST_JOINT_IDX` 或新增 `_BASE_VEL_CAP`：
```python
_BASE_VEL_CAP   = 8.0     # j1/j2
_ELBOW_VEL_CAP  = 3.0     # j3/j4
_WRIST_VEL_CAP  = 4.0     # j5/j6/j7
```

### 想用 per-joint comma list（類似 `--lpf-alpha`）

把 `--wrist-vel-cap` 改成 `--vel-caps`，parse 成 list[7]，傳給 `set_velocity_limit` 7 次。模式跟 `_parse_lpf_alpha()` 對齊。

### 想在 runtime 動態調 cap

加 kbd hotkey（e.g. `[` / `]` 加減 cap），呼叫 `self._placo_session.set_wrist_cap(new)` → 內部 `set_velocity_limit()`。但要在 robot lock 之外做，避免 QP 中途改參數。
