# IK Solver Weight Design — OpenArm O6 Right Arm

> 說明三種 IK 後端（pinocchio / pybullet / placo）如何參考 URDF
> 以及各關節懲罰權重（naturalness weight）的設計邏輯。

---

## 1. URDF 參考方式

所有三個 solver 使用同一份 URDF，在載入前先剝除 visual/collision mesh（只保留 kinematics）：

```
OPENARM_URDF (env var)
  └─ 若未設定 → 自動搜尋本地路徑
      1. /home/asus/openArm_leapHand_urdf/src/openarm_description/usd/v10_o6.urdf
      2. /home/asus/Desktop/openarm_description/urdf_transform/urdf/openarm_step10.urdf
```

### 1.1 URDF 關節結構（nq = 36）

| q index | 關節群 | 說明 |
|---------|--------|------|
| `q[0:7]`  | `openarm_left_joint1~7`  | 左臂 7 DOF |
| `q[7:18]` | 左手 11 joints           | LEAP Hand fingers |
| `q[18:25]`| `openarm_right_joint1~7` | 右臂 7 DOF ← **IK 目標範圍** |
| `q[25:36]`| 右手 11 joints           | LEAP Hand fingers |

> ⚠️ 注意：右臂 q_slice = `[18:25]`，**不是** `[7:14]`（那是左手 finger）。  
> pinocchio solver 透過 `model.joints[id].idx_q` 動態查詢避免硬編碼錯誤。

---

### 1.2 各 Solver 載入方式

| Solver | URDF 載入 API | FK/Jacobian |
|--------|--------------|-------------|
| **Pinocchio** | `pin.buildModelFromUrdf(urdf)` | `pin.forwardKinematics()` + `pin.getFrameJacobian()` |
| **PyBullet** | `p.loadURDF(urdf, physicsClientId=...)` | PyBullet 內建 FK（debug state） |
| **Placo** | `placo.RobotWrapper(urdf)` | Placo 內建 FK per FramePosition |

**PyBullet** 使用 `DIRECT` 模式（headless），不顯示 GUI：
```python
p.connect(p.DIRECT)
robot_id = p.loadURDF(kin_urdf, useFixedBase=True)
```

**Placo** 在每次 seed retry 前重建 `RobotWrapper + KinematicsSolver`（保持狀態乾淨）：
```python
robot = placo.RobotWrapper(kin_urdf)
solver = placo.KinematicsSolver(robot)
```

---

## 2. IK 演算法摘要

### Pinocchio — 加權 Damped Jacobian (DLS)

每次迭代解：

$$H \, \Delta q = J^T e + \mu W (q_\text{pref} - q)$$

$$H = J^T J + \lambda^2 I + \mu W$$

| 符號 | 說明 | 值 |
|------|-----|----|
| `J` | 6×7 Frame Jacobian（LOCAL_WORLD_ALIGNED，右臂欄） | — |
| `e` | `[Δxyz; Δω]` 6D 任務誤差 | — |
| `λ` | damping（防奇異點） | **0.01** |
| `μ` | naturalness scale | **3×10⁻⁴** |
| `W` | `diag(w₁…w₇)` 各關節權重矩陣 | 見下節 |
| `DT` | 每步 step size | **0.3** |
| `MAX_ITER` | 最多迭代次數（單一 seed） | 200 |

> μ 設計原則：μ << 1 確保任務目標（J^T e）主導，naturalness 只作微調。  
> μ=0.05 過大會造成求解提前停在自然姿勢而非目標位置。

---

### PyBullet — calculateInverseKinematics (DLS)

PyBullet 內建 DLS IK，naturalness 以 `restPoses` + `jointDamping` 間接編碼：

```python
result = p.calculateInverseKinematics(
    robot_id,
    ee_link_id,
    target_xyz,
    target_quat,
    restPoses=pref,        # 首選角度（等同 q_pref）
    jointDamping=damping,  # 各關節阻尼（等同 naturalness weight）
    lowerLimits=lo,
    upperLimits=hi,
    maxNumIterations=200,
    residualThreshold=1e-5,
)
```

- `restPoses` = 重力（自然偏好）方向的 q_pref  
- `jointDamping` = 數值越大 → 越強迫回到 `restPoses` → 效果等同 naturalness weight

---

### Placo — Task-based IK

Placo 使用任務優先模式（加權 QP / 速度級 IK）：

```
Task stack:
  1. PositionTask     (EE xyz)       weight = W_POS   = 1.0
  2. OrientationTask  (EE rotation)  weight = W_ORI   = 0.3   (no_rot=False 時)
  3. JointsTask       (naturalness)  weight (per-joint) 見下節
  4. RegularizationTask              weight = 1e-5    (防奇異點)
```

每次 `solver.solve(True)` 積分一個速度步長 (`dt = 0.01`)，重複直到 `pos_err < POS_TOL`。

---

## 3. 各關節權重設計

### 3.1 物理分組

```
OpenArm O6 右臂（j1=底座 → j7=手腕末端）

  j1  肩膀 Yaw      ─┐
  j2  肩膀 Pitch     ├─ 大馬達 (high weight)  穩定，不輕易移動
  j3  上臂 Yaw      ─┘

  j4  肘部 Flex        中等馬達 (medium weight)  肘部偏好 90° 伸展

  j5  前臂 Roll     ─┐
  j6  手腕 Yaw       ├─ 小馬達 (low weight)  靈活，讓任務自由調整
  j7  手腕 Pitch    ─┘
```

設計哲學：
- **j1~j3（大馬達）** → weight **高**：慣性大、扭力強，應讓手腕/前臂先移動，肩膀為輔。
- **j4（肘）** → weight **中**：偏好 90°（elbow-down forward-reach），允許輕度偏移。
- **j5~j7（手腕/前臂）** → weight **低**：靈活旋轉以補償精細對準，不需要懲罰。

---

### 3.2 Pinocchio 權重表

格式：`(pref, soft_lo, soft_hi, hard_lo, hard_hi, weight)`

| 關節 | pref (rad) | hard\_lo | hard\_hi | **weight W** | 說明 |
|------|-----------|----------|----------|-------------|------|
| j1 肩膀Yaw   | 0.000 | -1.396 | 1.500 | **2.5** | 大馬達，偏好正前方 |
| j2 肩膀Pitch | 0.700 | 0.000 | 1.600 | **4.0** | 大馬達，偏好微抬（0.7 rad ≈ 40°） |
| j3 上臂Yaw   | 0.000 | -1.571 | 0.300 | **10.0** | 大馬達最大！0=向前，強烈抵抗內旋 |
| j4 肘部      | 1.5708 | 0.250 | 2.200 | 2.0 | 偏好 90°，中等懲罰 |
| j5 前臂Roll  | 0.000 | -1.571 | 1.571 | 0.8 | 小馬達，允許自由旋轉 |
| j6 手腕Yaw   | 0.000 | -0.785 | 0.785 | 0.8 | 小馬達，偏好中立 |
| j7 手腕Pitch | 0.000 | -1.571 | 1.571 | **0.5** | 最靈活，手掌朝前 |

> `μ × W` = 實際正則化強度（μ = 3×10⁻⁴）  
> 相對大小：j3 (10×3e-4=3e-3) >> j2 (4×3e-4) > j1 > j4 > j5=j6 > j7

---

### 3.3 PyBullet 阻尼表 (`jointDamping`)

格式：`(pref, damping, hard_lo, hard_hi)`

| 關節 | pref (rad) | **damping** | hard\_lo | hard\_hi | 說明 |
|------|-----------|------------|----------|----------|------|
| j1 肩膀Yaw   | 0.000 | **0.15** | -1.396 | 1.500 | 大馬達 |
| j2 肩膀Pitch | 0.700 | **0.20** | 0.000 | 1.600 | 大馬達，最高阻尼 |
| j3 上臂Yaw   | 0.000 | **0.60** | -1.571 | 0.000 | 大馬達，最強 pull-to-pref |
| j4 肘部      | 1.5708 | 0.05 | 0.250 | 2.200 | 中等 |
| j5 前臂Roll  | 0.000 | 0.02 | -1.571 | 1.571 | 低阻尼 |
| j6 手腕Yaw   | 0.000 | 0.02 | -0.785 | 0.785 | 低阻尼 |
| j7 手腕Pitch | 0.000 | 0.04 | -1.571 | 1.571 | 稍高（手腕穩定） |

> PyBullet `damping` 無量綱，相對大小即影響 naturalness 強度。

---

### 3.4 Placo JointsTask 權重表

格式：`(pref, hard_lo, hard_hi, joints_weight)`

| 關節 | pref (rad) | hard\_lo | hard\_hi | **joints\_weight** | 說明 |
|------|-----------|----------|----------|-------------------|------|
| joint1 肩膀Yaw   | 0.000 | -1.396 | 1.500 | **0.15** | 大馬達 |
| joint2 肩膀Pitch | 0.700 | 0.000 | 1.600 | **0.25** | 大馬達 |
| joint3 上臂Yaw   | 0.000 | -1.571 | 0.000 | **0.80** | 大馬達最大！偏好向前 |
| joint4 肘部      | 1.5708 | 0.250 | 2.200 | 0.08 | 中等 |
| joint5 前臂Roll  | 0.000 | -1.571 | 1.571 | 0.03 | 低 |
| joint6 手腕Yaw   | 0.000 | -0.785 | 0.785 | 0.03 | 低 |
| joint7 手腕Pitch | 0.000 | -1.571 | 1.571 | 0.03 | 最低 |

> Placo `W_JOINTS = 1e-4`，combined JointsTask 整體 scale。  
> `joints_weight` 是各關節的 per-joint 倍率，最終強度 = `W_JOINTS × joints_weight`。  
> PositionTask (W_POS=1.0) >> RegularizationTask (1e-5) >> JointsTask (~1e-4) 確保任務優先。

---

## 4. 各 Solver 權重對比

| 關節 | Pinocchio W | PyBullet damping | Placo joint_w | 分組 |
|------|------------|-----------------|---------------|------|
| j1 肩膀Yaw   | 2.5  | 0.15 | 0.15 | 大馬達 |
| j2 肩膀Pitch | 4.0  | 0.20 | 0.25 | 大馬達 |
| j3 上臂Yaw   | **10.0** | **0.60** | **0.80** | 大馬達（最強） |
| j4 肘部      | 2.0  | 0.05 | 0.08 | 中 |
| j5 前臂Roll  | 0.8  | 0.02 | 0.03 | 小馬達（靈活） |
| j6 手腕Yaw   | 0.8  | 0.02 | 0.03 | 小馬達 |
| j7 手腕Pitch | 0.5  | 0.04 | 0.03 | 最靈活 |

> 三個 solver 的相對大小保持一致的設計哲學：
> **j3 >> j2 ≈ j1 > j4 >> j5 ≈ j6 > j7**

---

## 5. 為什麼 j3 weight 最高（= 10？）

j3 = **上臂 Yaw**（upperarm yaw，控制肩膀到手肘的內外旋轉）

這個方向有幾個問題：
1. **機械干涉**：j3 過大（內旋超過 −45°）會讓手肘碰到軀幹。
2. **奇異點附近**：j3=0（向前）+ j4=90° 是標準 forward-reach，這點附近 Jacobian conditioning 好；偏移會讓條件數變差。
3. **人體姿勢直覺**：人類手臂自然朝前伸出時，上臂 yaw ≈ 0。

故 j3 使用最高 weight，強迫 IK 在 j3 偏離時付出高代價，引導 IK 找 j3≈0 的解。

---

## 6. seeds 設計

三個 solver 共享相同的 seed library（先試 last_joints，再依序試以下）：

```python
_SEEDS_RIGHT = [
    [ 0.00,  0.70,  0.00,  1.5708,  0.00,  0.00,  0.00],  # home: forward-reach 90°
    [ 0.00,  0.50,  0.00,  1.5708,  0.00,  0.00,  0.00],  # 肩偏低
    [ 0.00,  1.00,  0.00,  1.5708,  0.00,  0.00,  0.00],  # 肩偏高
    [ 0.20,  0.70, -0.20,  1.5708,  0.00,  0.00,  0.00],  # 外側偏轉
    [-0.20,  0.70, -0.20,  1.5708,  0.00,  0.00,  0.00],  # 內側偏轉
    [ 0.00,  0.80,  0.00,  2.0000,  0.00,  0.00,  0.00],  # 肘部伸更直
    [ 0.00,  0.60,  0.00,  1.2000,  0.00,  0.30,  0.00],  # 前臂旋轉
]
```

所有 seeds 都維持 j4=90°（1.5708 rad），因為這是 elbow-down 的最穩定姿勢。

---

## 7. 調參建議

若 IK 成功率低，可按以下方向調整：

### 7.1 提高成功率（SR%）

| 問題 | 診斷 | 調整方向 |
|------|------|---------|
| 超出工作空間（z 過高） | Weight 過重限制 j2 | 降低 j2 soft_hi，增加 reach seeds |
| j3 卡邊界（內旋） | hard_lo 過嚴 | 放寬 j3 hard_lo 至 -π/2 |
| 遠距離目標 seed 沒覆蓋 | seeds 不夠多樣 | 增加肘部彎曲角度的 seeds |
| pinocchio 迭代不夠 | MAX_ITER 不足 | 增加至 300+  |

### 7.2 讓手腕更靈活（提升 j5~j7 自由度）

**Pinocchio**: 降低 j5/j6/j7 的 `weight`：
```python
# 原來 j5=j6=0.8, j7=0.5
( 0.000, -1.571,  1.571,  -1.571,  1.571,   0.3),  # j5 更靈活
( 0.000, -0.785,  0.785,  -0.785,  0.785,   0.3),  # j6 更靈活
( 0.000, -1.571,  1.571,  -1.571,  1.571,   0.2),  # j7 最靈活
```

**PyBullet**: 降低 `damping`：
```python
"j5": ( 0.000, 0.01, ...),  # 原 0.02 → 0.01
"j6": ( 0.000, 0.01, ...),
"j7": ( 0.000, 0.02, ...),
```

**Placo**: 降低 `joints_weight` 或整體降低 `W_JOINTS`：
```python
W_JOINTS = 5e-5  # 原 1e-4，降低整體 naturalness scale
```

### 7.3 讓肩膀更穩定（降低 j1~j3 移動量）

**Pinocchio**: 提高 j1 weight，縮緊 j3 hard 範圍：
```python
( 0.000, -0.300, 0.800, -0.800, 1.200, 3.5),  # j1 weight 2.5→3.5
( 0.000, -1.571, 0.000, -1.200, 0.200, 12.0), # j3 weight 10→12，hard range 縮緊
```

---

## 8. 位置 vs 方向的 trade-off

| Solver | 位置任務 | 方向任務 |
|--------|---------|---------|
| Pinocchio | 全 6D（含方向） | 可選 `no_rot=True` 純位置 |
| PyBullet | 全 6D（含方向） | 可選純位置（omit targetOrientation） |
| Placo | PositionTask W=1.0 | OrientationTask W=0.3（比位置低） |

> Placo 的 `W_ORI=0.3 < W_POS=1.0` 意味著當位置和方向衝突時，**位置優先**，手腕姿勢允許有 0.15 rad 誤差範圍內浮動。
