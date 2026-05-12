# OpenArm EE-Delta IK Controllers

此目錄包含所有 **tracker_ee_delta_ik** 系列控制器。  
所有腳本自動將父目錄 (`ik_reachability/`) 加入 Python path，可直接執行。

---

## 目錄結構

```
ik_controllers/
├── README.md              ← 本文件
├── paths.py               ← 統一輸出路徑 helper（csv_path / png_for / ws_mesh_path）
│
├── docs/                  ← 設計與分析文件（見 docs/README.md）
│
├── trackers/              ← VR tracker → ROS topic（上游）
│   ├── tracker_ee_absolute.py
│   └── tracker_ee_delta_ik_backend.py  ★ 多 IK 後端統一控制器
│
├── ik_node/               ← ROS IK 節點 + 即時 profiler
│   ├── placo_ik_node.py                       核心 ROS Node + AsyncCsvWriter + TfPoller
│   ├── placo_ik_session.py                    純 Python IK session（無 ROS）
│   ├── kbd_controller.py                      鍵盤 daemon
│   ├── placo_ik_online_profiler.py            delta 模式 profiler
│   ├── placo_ik_online_profiler_ws_mesh.py    delta + WorkspaceMesh clamp
│   ├── placo_ik_absolute_profiler.py          絕對座標 profiler
│   └── placo_ik_absolute_profiler_ws_mesh.py  絕對 + WorkspaceMesh clamp
│
├── ws_mesh/               ← WorkspaceMesh 工具集（離線，無 ROS）
│   ├── placo_ws_reachability.py    掃描可達空間
│   ├── placo_ws_analyze.py         建立 WorkspaceMesh + .npz
│   ├── placo_ws_view3d.py          互動 3D 視覺化
│   └── placo_ws_clamp_benchmark.py box vs mesh clamp 速度比較
│
├── offline_profilers/     ← 離線 IK 基準測試（無 ROS）
│   └── placo_ik_profiler.py
│
├── ik_solver/             ← solver 後端
│   └── placo_ik_solver.py
│
├── archive/               ← 舊備份（保留以便回溯）
│   ├── placo_ik_profiler_copy.py
│   └── placo_ik_profiler_copy2.py
│
└── results/               ← 由 paths.py 統一管理
    ├── YYYYMMDD/                                各日 CSV / PNG
    ├── reachability_{arm}_ws.npz                WorkspaceMesh（auto-detect）
    └── legacy/                                  舊散落產物
```

> **路徑 helper（`paths.py`）**：所有 profiler/tracker 不再硬編碼 `results/yyyymmdd/...`，
> 一律 `from paths import csv_path` → `csv_path('placo_online', arm, mode)`。

---

## Home 姿態（forward-reach, j4=90°）

| 關節 | 角度 (rad) | 說明 |
|------|-----------|------|
| j1 | 0.0 | 肩膀前後中立 |
| j2 | 0.7 | 肩膀略微抬起 |
| j3 | 0.0 | 上臂 yaw 中立（j3=0 = 向前） |
| j4 | **1.5708** | 手肘 **90°** 向前抓取標準姿態 |
| j5 | 0.0 | 前臂旋轉中立 |
| j6 | 0.0 | 手腕左右中立 |
| j7 | 0.0 | 手腕上下中立（手掌朝前） |

---

## 1. tracker_ee_delta_ik_backend.py ⭐ 主推薦

**4 種 IK 後端的統一控制器**，不需要 MoveIt。

### 前提條件
```bash
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
```

### 快速開始

```bash
# scipy（預設，最快）
python3 tracker_ee_delta_ik_backend.py --arm right --pattern keyboard

# PyBullet IK
conda run -n pico_teleop_py python3 tracker_ee_delta_ik_backend.py \
  --arm right --pattern keyboard --solver pybullet

# Pinocchio IK
conda run -n pico_teleop_py python3 tracker_ee_delta_ik_backend.py \
  --arm right --pattern ee_delta --solver pinocchio

# Placo IK
conda run -n pico_teleop_py python3 tracker_ee_delta_ik_backend.py \
  --arm right --pattern keyboard --solver placo

# VR tracker + 無旋轉追蹤
conda run -n pico_teleop_py python3 tracker_ee_delta_ik_backend.py \
  --arm right --pattern ee_delta --solver pinocchio --no-rot-tracking

# 雙臂（兩個 terminal）
conda run -n pico_teleop_py python3 tracker_ee_delta_ik_backend.py --arm right --pattern ee_delta --solver pinocchio &
conda run -n pico_teleop_py python3 tracker_ee_delta_ik_backend.py --arm left  --pattern ee_delta --solver pinocchio
```

### 主要選項

| 選項 | 說明 | 預設 |
|------|------|------|
| `--arm` | `left` \| `right` | `right` |
| `--solver` | `scipy` \| `pybullet` \| `pinocchio` \| `placo` | `scipy` |
| `--pattern` | `keyboard` \| `sine` \| `circle` \| `ee_delta` \| `topic` | `keyboard` |
| `--no-rot-tracking` | 只追蹤位置，不追蹤姿態 | off |
| `--home-first` | 啟動時先回到 home 姿態 | off |
| `--dry-run` | 計算 IK 但不發送給機器人 | off |
| `--rate` | 控制循環 Hz | 20 |

### 輸入模式（`--pattern`）

| 模式 | 輸入來源 | 說明 |
|------|----------|------|
| `keyboard` | 鍵盤 | WASD+QE=XYZ, UIOJKL=RPY |
| `sine/circle/lemniscate` | 自動 | 測試軌跡 |
| `ee_delta` | `/ee_delta/{arm}` PoseStamped | VR tracker 累積 delta（推薦） |
| `topic` | `/pico_{arm}/delta_twist` TwistStamped | Pico 橋接節點 |

---

## 2. tracker_ee_delta_ik_controller.py

基礎版控制器，使用 MoveIt `/compute_ik` 服務。

### 前提條件
```bash
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
ros2 launch openarm_bimanual_moveit_config move_group_only.launch.py
```

### 使用方法
```bash
# 右臂鍵盤控制
python3 tracker_ee_delta_ik_controller.py --arm right --pattern keyboard

# VR tracker
python3 tracker_ee_delta_ik_controller.py --arm right --pattern ee_delta

# 雙臂
python3 tracker_ee_delta_ik_controller.py --arm right --pattern ee_delta &
python3 tracker_ee_delta_ik_controller.py --arm left  --pattern ee_delta
```

---

## 3. tracker_ee_delta_ik_controller_tf.py

**TF2 加強版**。從 TF tree 讀取 EE 初始姿態（避免 home pose 偏移問題）。

### 前提條件
```bash
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
ros2 launch openarm_bimanual_moveit_config move_group_only.launch.py
```

### 使用方法
```bash
# 右臂鍵盤控制（含 TF 初始化）
python3 tracker_ee_delta_ik_controller_tf.py --arm right --pattern keyboard

# VR ee_delta 模式
python3 tracker_ee_delta_ik_controller_tf.py --arm right --pattern ee_delta

# 啟動時先回 home
python3 tracker_ee_delta_ik_controller_tf.py --arm right --pattern keyboard --home-first
```

---

## 4. tracker_ee_delta_ik_forward_reach.py

**向前抓取限制版**，加入 ForwardReachIKSolver 防止手臂做出非向前姿態。

### 前提條件
```bash
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
ros2 launch openarm_bimanual_moveit_config move_group_only.launch.py
```

### 使用方法
```bash
# 右臂鍵盤（向前抓取約束）
python3 tracker_ee_delta_ik_forward_reach.py --arm right --pattern keyboard

# 只追蹤位置
python3 tracker_ee_delta_ik_forward_reach.py --arm right --pattern keyboard --no-rot-tracking
```

---

## 5. tracker_ee_delta_ik_forward_reach_tf.py

**最完整舊版**：TF2 + MoveIt + ForwardReach 約束 + ee_delta/topic 雙模式。

### 前提條件
```bash
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
ros2 launch openarm_bimanual_moveit_config move_group_only.launch.py
```

### 使用方法
```bash
# 右臂 VR tracker（推薦用法）
python3 tracker_ee_delta_ik_forward_reach_tf.py --arm right --pattern ee_delta

# 左臂 VR tracker
python3 tracker_ee_delta_ik_forward_reach_tf.py --arm left --pattern ee_delta

# 雙臂同時
python3 tracker_ee_delta_ik_forward_reach_tf.py --arm right --pattern ee_delta &
python3 tracker_ee_delta_ik_forward_reach_tf.py --arm left  --pattern ee_delta

# Yaw 校正（tracker 與手臂方向不對齊時）
python3 tracker_ee_delta_ik_forward_reach_tf.py --arm right --pattern ee_delta --calib-yaw 30

# 完整 RPY 校正
python3 tracker_ee_delta_ik_forward_reach_tf.py --arm right --pattern ee_delta --calib-rpy 0,0,30
```

---

## 6. tracker_ee_delta_ik_natural.py

**NaturalIK 防肘飄版**：整合 `NaturalIKSolver`，防止 7-DOF IK 求解時肘部飄到非自然位置。

### 前提條件
```bash
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
ros2 launch openarm_bimanual_moveit_config move_group_only.launch.py
```

### 使用方法
```bash
# 右臂 VR tracker（含自然姿態過濾）
python3 tracker_ee_delta_ik_natural.py --arm right --pattern ee_delta

# 關閉自然 IK（回退至基礎行為）
python3 tracker_ee_delta_ik_natural.py --arm right --pattern ee_delta --no-natural-ik

# Debug 模式（顯示每次 IK 嘗試）
python3 tracker_ee_delta_ik_natural.py --arm right --pattern keyboard --natural-verbose
```

---

## 7. tracker_ee_delta_ik_pure.py

**純 Python IK（無需 MoveIt）**，使用 `scipy.optimize` + 自訂 FK 解算。

### 前提條件（只需 driver，不需 move_group）
```bash
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
pip install numpy scipy
```

### 使用方法
```bash
# 右臂鍵盤（j4=90° home，無需 move_group）
python3 tracker_ee_delta_ik_pure.py --arm right --pattern keyboard

# 右臂 VR tracker
python3 tracker_ee_delta_ik_pure.py --arm right --pattern ee_delta

# 雙臂同時
python3 tracker_ee_delta_ik_pure.py --arm right --pattern ee_delta &
python3 tracker_ee_delta_ik_pure.py --arm left  --pattern ee_delta

# 只追蹤位置（更快更穩）
python3 tracker_ee_delta_ik_pure.py --arm right --pattern keyboard --no-rot-tracking

# Dry-run（計算 IK 但不發送）
python3 tracker_ee_delta_ik_pure.py --arm right --pattern sine --dry-run
```

---

## 版本對照表

| 檔案 | IK 後端 | 需要 MoveIt | `ee_delta` | TF2 初始化 | ForwardReach | 推薦度 |
|------|---------|------------|-----------|-----------|--------------|--------|
| `backend.py` | scipy/pybullet/pinocchio/placo | ❌ | ✅ | ✅ | ✅ | ⭐⭐⭐ |
| `pure.py` | scipy | ❌ | ✅ | ✅ | ✅ | ⭐⭐⭐ |
| `forward_reach_tf.py` | MoveIt /compute_ik | ✅ | ✅ | ✅ | ✅ | ⭐⭐ |
| `natural.py` | MoveIt /compute_ik | ✅ | ✅ | ✅ | 部分 | ⭐⭐ |
| `controller_tf.py` | MoveIt /compute_ik | ✅ | ✅ | ✅ | ❌ | ⭐ |
| `forward_reach.py` | MoveIt /compute_ik | ✅ | ✅ | ❌ | ✅ | ⭐ |
| `controller.py` | MoveIt /compute_ik | ✅ | ✅ | ❌ | ❌ | ⭐ |

---

## 相依模組（位於 `../` 即 `ik_reachability/`）

| 模組 | 說明 |
|------|------|
| `pure_python_ik_solver.py` | scipy FK+IK（ForwardReach 限制）|
| `pybullet_ik_solver.py` | PyBullet DLS IK |
| `pinocchio_ik_solver.py` | Pinocchio 3.x 梯度 IK |
| `placo_ik_solver.py` | Placo task-based IK |
| `natural_ik_solver.py` | NaturalIK 多 seed 過濾器 |
| `forward_reach_ik_solver.py` | ForwardReach solver（for MoveIt 版） |

---

## 關節定義（OpenArm O6 右臂）

```
URDF limits（v10_o6.urdf）：
  j1: [-1.396, 3.491]  肩膀前後  (shoulder flex/extension)
  j2: [-1.745, 1.745]  肩膀側向  (shoulder abduction)
  j3: [-1.571, 1.571]  上臂yaw j3=0는向前  (upper arm yaw, null-space)
  j4: [ 0.000, 2.443]  手肘彎曲  (elbow flexion) ← home=90°=1.5708
  j5: [-1.571, 1.571]  前臂旋轉  (forearm roll)
  j6: [-0.785, 0.785]  手腕左右  (wrist yaw)
  j7: [-1.571, 1.571]  手腕上下 手掌朝前  (wrist pitch)

⚠ j1~j3 為大型馬達，IK 求解對這些關節設有較高懲罰權重，
  避免快速移動導致系統不穩定。
```

---

## ee_delta 訊息格式

Topic: `/ee_delta/{left|right}` → `geometry_msgs/PoseStamped`

```
pose.position.{x,y,z}   = 距離 reference 的 Δ 位移 [m]
pose.orientation.{x,y,z,w} = 距離 reference 的 Δ 旋轉 (quaternion)
header.frame_id = "tracker_delta"
```
