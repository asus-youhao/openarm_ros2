# OpenArm VR/AR 遙控操作 — MoveIt2 評估計劃

> 目標：系統性地量測、分析並優化 MoveIt2 對 Apple Vision Pro / Pico 等 VR/AR 裝置
> 的即時遙控能力，建立可重現的評估基準。

---

## 目錄

1. [背景：MoveIt2 規劃流程解析](#1-背景moveit2-規劃流程解析)
2. [跳動問題（Jerk / Discontinuity）深入解釋](#2-跳動問題jerk--discontinuity深入解釋)
3. [OMPL 規劃器比較與推薦](#3-ompl-規劃器比較與推薦)
4. [七階段實驗路線圖](#4-七階段實驗路線圖)
   - [階段 1：IK 延遲基準測試 ✅](#階段-1ik-延遲基準測試--完成)
   - [階段 2：分離 OMPL 規劃時間 ✅](#階段-2分離-ompl-規劃時間-vs-機器人移動時間--完成含-spin_once-修復)
   - [階段 2-1：Pilz 規劃器比較 ✅](#階段-2-1pilz-規劃器比較測試--完成--planner-旗標)
   - [階段 2-2：MoveIt Servo 即時控制 🔲](#階段-2-2moveit-servo-即時笛卡爾控制--待測試)
   - [階段 3：realtime_ik_controller.py ✅](#階段-3繞過-ompl-的即時-ik-控制器--完成)
   - [階段 4：端到端延遲測試 🔲](#階段-4端到端延遲測試--待完成)
   - [階段 5：使用者操作實驗 🔲](#階段-5使用者任務操作實驗--待完成)
5. [評估指標清單](#5-評估指標清單)
6. [目前工具清單與狀態](#6-目前工具清單與狀態)
7. [下一步行動](#7-下一步行動)

---

## 1. 背景：MoveIt2 規劃流程解析

### 1.1 `/compute_ik` 服務（純 IK）

```
輸入：EE 位姿 (xyz + quaternion)
輸出：
  ✅ 關節角度解 (joint_state)
  ✅ 成功/失敗 error_code
  ❌ 無路徑
  ❌ 預設不做碰撞檢查（需明確設定 avoid_collisions=true）
典型延遲：1–20 ms
```

### 1.2 MoveGroup 完整規劃管線（不含執行）

```
[EE 位姿輸入]
      │
      ▼
┌─────────────────────┐
│  步驟 1：IK 解算     │  1–20 ms   KDL / TRAC-IK
│  目前關節角 → 目標角  │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  步驟 2：OMPL 路徑規劃│  100–5000 ms  RRT* / BiTRRT...
│  從 q_start → q_goal│  包含碰撞場景採樣、自碰撞檢查
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  步驟 3：時間參數化   │  1–10 ms   TOTG / IPTP
│  路徑 → 速度/加速度  │  確保速度/加速度/jerk 限制
└──────────┬──────────┘
           │（到此為止若不執行，只拿 trajectory 不送出）
           ▼
┌─────────────────────┐
│  步驟 4：實際執行     │  依軌跡長度 500ms–10s
│  JointTrajectoryCtrl│  機器手臂物理移動
└─────────────────────┘
```

### 1.3 `csv_waypoint_runner` 目前量了什麼

| 欄位 | 實際量測內容 | 含碰撞? | 含移動? |
|---|---|---|---|
| `planning_ms` | `move_to_pose()` 回傳時間（約 1–5ms，只是送 ROS goal） | ❌ | ❌ |
| `execution_ms` | `wait_until_executed()` 等待（OMPL規劃 + 機器人移動 混在一起） | ✅ | ✅ |
| `total_ms` | 兩者加總 | ✅ | ✅ |

**缺點**：`execution_ms` 無法分辨「OMPL花了多久」vs「機器人移動花了多久」。

---

## 2. 跳動問題（Jerk / Discontinuity）深入解釋

### 2.1 什麼是跳動？

VR/AR 遙控時，手在空中連續移動，控制端會以固定頻率（如 30 Hz）送出一系列 EE 目標位姿。
「跳動」是指機械臂在執行過程中出現**不連續的突然移動**，有兩種根本原因：

```
原因 A：前一個 trajectory 還沒執行完，新的 goal 就來了
         Robot: ──── traj₁ ────►
         VR 指令: ─ goal₁ ─ goal₂ ─ goal₃ ─ ...
                              ↑↑↑ traj₁ 仍在跑時 goal₂抵達
         後果：控制器搶佔 → 關節速度突然不連續 → 震動

原因 B：連續兩個 OMPL 路徑在關節空間不連續
         IK 解有多組解，next_goal 選了和 prev_goal 完全不同的解
         e.g., joint₁: 30° → 150° （而非繼續 30° → 35°）
         後果：機器手臂瞬間大幅度跳動
```

### 2.2 跳動的量化指標

| 指標 | 定義 | 計算方式 |
|---|---|---|
| **Jerk** | 加速度的導數 | `d³q/dt³` |
| **Joint velocity discontinuity** | 兩段 trajectory 接合處的速度差 | `|v_end_prev - v_start_next|` |
| **Position error** | EE 實際位置 vs 目標位置 | 用 TF 讀取 |
| **Re-planning rate** | 被迫放棄並重規劃的比例 | 統計 |

### 2.3 解決方案

| 方法 | 原理 | 適用場景 |
|---|---|---|
| **Online replanning（增量式）** | 每次規劃從當前狀態出發，保留速度連續性 | MoveIt Servo |
| **MoveIt Servo**（強烈推薦） | 繞過 OMPL，直接做笛卡爾增量 → 關節速度控制 | AR/VR 即時控制 |
| **Trajectory blending** | OMPL + TOTG 產生後，以 spline 混合接縫 | 預先規劃路徑 |
| **Joint seed 固定** | 每次 IK 以上一步解作 seed，避免多解跳動 | 任何 OMPL pipeline |
| **速度/加速度限制收緊** | 降低 `max_velocity_scaling_factor` | 犧牲速度換平滑 |

---

## 3. OMPL 規劃器比較與推薦

### 3.1 目前預設：RRT-Connect

```
特性：
  · 雙向 RRT，速度快，適合高維度關節空間
  · 找到的路徑不是最優的（zigzag）
  · 路徑品質靠後處理 simplify + smooth 改善
  · 典型規劃時間：50–500 ms
```

### 3.2 各規劃器比較

| 規劃器 | 速度 | 路徑品質 | 碰撞處理 | AR/VR 推薦度 |
|---|---|---|---|---|
| **RRT-Connect**（預設） | ⭐⭐⭐⭐ | ⭐⭐ | ✅ 隨機採樣 | ⭐⭐⭐ |
| **BiTRRT** | ⭐⭐⭐ | ⭐⭐⭐ | ✅ | ⭐⭐ |
| **RRT\*** | ⭐⭐ | ⭐⭐⭐⭐ | ✅ | ❌ 太慢 |
| **STOMP** | ⭐⭐ | ⭐⭐⭐⭐⭐ | ✅ 梯度優化 | ❌ 太慢 |
| **CHOMP** | ⭐⭐ | ⭐⭐⭐⭐⭐ | ✅ 梯度 | ❌ 太慢 |
| **Pilz LIN/PTP** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⚠️ 需預先設定 | ⭐⭐⭐⭐ |
| **MoveIt Servo** | ⭐⭐⭐⭐⭐ | N/A | ⚠️ 即時偵測 | ⭐⭐⭐⭐⭐ |

### 3.3 AR/VR 即時控制最推薦：MoveIt Servo

```yaml
# moveit_servo 工作方式
1. 訂閱 /servo_node/delta_twist_cmds  (每 10ms 一次笛卡爾增量)
2. 即時 Jacobian 逆解 → 關節速度指令
3. 直接送 JointVelocity 到控制器
4. 碰撞監測 → 接近障礙物時自動減速

優點：
  - 延遲 < 10 ms（不需要 OMPL）
  - 路徑連續，無跳動
  - AR/VR 送 delta pose → 直接跟蹤

缺點：
  - 不保證找到路徑（局部逆 Jacobian 可能奇異點卡住）
  - 需要 joint velocity controller（非 trajectory controller）
```

### 3.4 Pilz Industrial Motion Planner

```
適合場景：直線/圓弧運動，路徑形狀可預測
命令格式：PTP（point-to-point）、LIN（直線）、CIRC（圓弧）
規劃時間：< 5 ms（不使用隨機採樣）
碰撞：需要手動設定碰撞場景，或搭配 parent planner
```

---

## 4. 七階段實驗路線圖

### 階段 1：IK 延遲基準測試 ✅ 完成

```bash
# 測試 /compute_ik 純計算延遲
python3 ik_timing_benchmark.py --csv right_reachability_.csv --arm right
python3 ik_timing_benchmark.py --csv right_reachability_.csv --arm right --all-orient

# 輸出：ik_timing_right.csv, ik_timing_right.png
```

**期望指標**：
- 中位數 < 5 ms（200 Hz 可行）
- P99 < 20 ms

**目前已知結果**：1–13 ms，大部分滿足 100 Hz 需求 ✅

---

### 階段 2：分離 OMPL 規劃時間 vs 機器人移動時間 ✅ 完成（含 spin_once 修復）

**方法**：`csv_waypoint_runner.py` 新增 `--split-timing` 旗標，
使用 pymoveit2 `plan()` + `execute()` 分開計時。

```bash
python3 csv_waypoint_runner.py --csv right_reachability_.csv --arm right \
    --max-pts 30 --split-timing
# 輸出：right_waypoint_timing.csv 含 ompl_ms / planned_duration_ms / motion_ms
```

**新增欄位說明**：

| 欄位 | 含義 |
|---|---|
| `ompl_ms` | 真正的 OMPL 規劃時間（IK + 碰撞路徑 + 時間參數化） |
| `planned_duration_ms` | TOTG/IPTP 計算出的軌跡預計執行時間 |
| `motion_ms` | 機器人實際移動時間（wall-clock） |
| `n_waypoints` | 軌跡中的關節角度點數量 |
| `joint_path_len` | 關節空間路徑長度 Σ\|Δq\|（rad） |

**實測已知**：
- OMPL 規劃時間 ≈ **150–600 ms**（典型 RRT-Connect） → MoveGroup 約 **1–5 Hz**
- 規劃失敗率 < 10%（視工作空間位置而定）
- `motion_ms` ≈ `planned_duration_ms`（控制器追蹤品質）

**已修復的 pymoveit2 Bug（2026-04-20）**：
1. `wait_until_executed()` 在 EXECUTING 狀態提前返回 False（motion_ms = 0）
2. `plan()` 內部 `rclpy.spin_once()` 污染 `node.executor` → execute_trajectory 回調永遠不觸發
→ 修復：Phase 3 改用 `while query_state() != IDLE: rclpy.spin_once(node, timeout_sec=0.05)`

---

### 階段 2-1：Pilz 規劃器比較測試 ✅ 完成（`--planner` 旗標）

**目的**：量化 Pilz PTP/LIN 相較於 OMPL 的規劃時間差異。

```bash
# OMPL (RRTConnect) — 基準
python3 csv_waypoint_runner.py --csv right_reachability_.csv --arm right \
    --planner ompl --max-pts 30 --split-timing --timing-csv ompl_timing.csv

# Pilz PTP（點對點，不保證 Cartesian 直線）
python3 csv_waypoint_runner.py --csv right_reachability_.csv --arm right \
    --planner pilz_ptp --max-pts 30 --split-timing --timing-csv pilz_ptp_timing.csv

# Pilz LIN（直線 Cartesian 路徑，保持 EE 方向）
python3 csv_waypoint_runner.py --csv right_reachability_.csv --arm right \
    --planner pilz_lin --max-pts 30 --split-timing --timing-csv pilz_lin_timing.csv

# 試所有 6 個 EE 方向 + Pilz PTP
python3 csv_waypoint_runner.py --csv right_reachability_.csv --arm right \
    --planner pilz_ptp --all-orient --max-pts 20 --home-first
```

**預期結果比較**：

| 規劃器 | 規劃時間 (`ompl_ms`) | 特點 |
|---|---|---|
| OMPL RRTConnect | 50–600 ms | 碰撞完整檢查，隨機路徑 |
| Pilz PTP | **1–5 ms** | 確定性，無隨機採樣，需清空工作區 |
| Pilz LIN | **1–5 ms** | 直線笛卡爾路徑，EE 方向鎖定 |

**注意事項**：
- Pilz 規劃器不執行完整隨機碰撞採樣，需手動確保工作空間無障礙
- `joint_limit_margin` 設定過小可能導致 Pilz 規劃失敗
- `pilz_cartesian_limits.yaml` 配置速度/加速度上限必須符合機械臂規格

---

### 階段 2-2：MoveIt Servo 即時笛卡爾控制 🔲 待測試

**原理**：MoveIt Servo 完全繞過 OMPL 和 /compute_ik，
使用 **Jacobian 偽逆** 將笛卡爾速度直接映射到關節速度。
延遲 < 10 ms，控制頻率 100 Hz，被認為是 AR/VR 最佳方案。

```
VR/AR (90 Hz)
      │  TwistStamped (笛卡爾速度增量)
      ▼
servo_node (moveit_servo)
      │  Jacobian^-1 → 關節速度  (< 5 ms)
      │  ButterworthFilter 平滑
      ▼
/right_arm_controller/joint_trajectory  (100 Hz)
      ▼
OpenArm O6 右臂
```

**步驟**：

```bash
# Step 1：啟動基礎設施
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
ros2 launch openarm_bimanual_moveit_config move_group_only.launch.py

# Step 2：啟動 MoveIt Servo（右臂）
ros2 launch openarm_bringup servo_right.launch.py   # → servo_node_main

# Step 3：啟動 Servo（激活服務）
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger

# Step 4a：互動式鍵盤遙控
python3 scripts/ik_reachability/servo_teleop_client.py --arm right

# Step 4b：基準測試（正弦波 X 方向速度，量測 publish 延遲）
python3 scripts/ik_reachability/servo_teleop_client.py --arm right \
    --benchmark --duration 15.0 --rate 50.0

# Step 4c：圓形軌跡（Y-Z 平面，測試連續性）
python3 scripts/ik_reachability/servo_teleop_client.py --arm right \
    --circle --duration 10.0 --rate 50.0

# Step 5：停止 Servo
ros2 service call /servo_node/stop_servo std_srvs/srv/Trigger
```

**監控與調試**：
```bash
# 查看 Servo 狀態（奇異點接近度、碰撞警告）
ros2 topic echo /servo_node/status

# 查看輸出 JointTrajectory
ros2 topic hz /right_arm_controller/joint_trajectory

# 如果 Servo 卡住（奇異點），發零速度解除
ros2 topic pub --once /servo_node/delta_twist_cmds geometry_msgs/TwistStamped \
    "{header: {frame_id: world}, twist: {}}"
```

**設定檔**：
- `openarm_bringup/config/servo_right_config.yaml`：右臂 Servo 設定
- `openarm_bringup/config/servo_left_config.yaml`：左臂 Servo 設定
- Launch: `openarm_bringup/launch/servo_right.launch.py`

**注意事項**：
- Servo 需要 `joint_velocity` 或 `position` command interface
  OpenArm 用 `position` — 輸出為 short-horizon JointTrajectory（可行）
- 奇異點附近 Servo 會自動減速（`hard_stop_singularity_threshold=30`）
- 如果 `is_primary_planning_scene_monitor=false`，需要先啟動 `move_group`

---

### 階段 3：繞過 OMPL 的即時 IK 控制器 ✅ 完成

**檔案**：`scripts/ik_reachability/realtime_ik_controller.py`

與 MoveIt Servo 的差異：
- **Servo**：Jacobian 逆解，處理笛卡爾**速度**（delta Twist），100+ Hz
- **This node**：呼叫 `/compute_ik`，處理絕對**位置** PoseStamped，30–100 Hz

```
VR/AR (30–90 Hz)
      │  /right/target_pose  (PoseStamped — 絕對位姿)
      ▼
realtime_ik_controller.py
      │  /compute_ik service  (1–20 ms, 無 OMPL 隨機採樣)
      │  以上一步解為 seed → 防止多解跳動
      ▼
/right_arm_controller/joint_trajectory
      ▼
OpenArm O6 右臂
```

```bash
# 控制右臂（預設 50 ms 軌跡 horizon）
python3 scripts/ik_reachability/realtime_ik_controller.py --arm right

# 33 ms horizon（配合 30 Hz VR 輸入）
python3 scripts/ik_reachability/realtime_ik_controller.py --arm right --horizon 33

# 停用碰撞檢查（IK 速度最快 ~2–5 ms）
python3 scripts/ik_reachability/realtime_ik_controller.py --arm right --no-collisions

# 僅計算 IK，不執行（dry-run 性能測試）
python3 scripts/ik_reachability/realtime_ik_controller.py --arm right --dry-run

# 離開時回 home
python3 scripts/ik_reachability/realtime_ik_controller.py --arm right --home-on-exit

# 測試：發布一個目標位姿
ros2 topic pub --once /right/target_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: world}, pose: {position: {x: 0.4, y: -0.2, z: 0.3}, \
     orientation: {x: 0.0, y: 0.0, z: -0.707, w: 0.707}}}"

# 監控 IK 延遲（即時）
ros2 topic echo /right/ik_latency_ms
```

**IK Seed 策略（防跳動關鍵）**：
```python
# 每次 /compute_ik 呼叫以上一步成功解作 seed
ik.robot_state.joint_state.position = last_joint_solution
# → 確保選取最近的 IK 解，避免機械臂突然跳到另一個等效解
```

**比較：三種即時控制方案**

| 方案 | 延遲 | 輸入格式 | 碰撞 | 適用場景 |
|---|---|---|---|---|
| `realtime_ik_controller.py` | 1–20 ms | PoseStamped（絕對位姿） | ⚠️ 可選 | VR 絕對位置跟蹤 |
| MoveIt Servo | < 5 ms | TwistStamped（速度增量） | ✅ 即時 | VR 相對速度控制 |
| OMPL MoveGroup | 50–600 ms | PoseStamped | ✅ 完整 | 精確點到點規劃 |

---

### 階段 4：端到端延遲測試 🔲 待完成

```
量測管線：
  [VR 送出 pose stamp]
        │
        ▼
  [ROS2 接收 stamp]
        │  通訊延遲
        ▼
  [IK 完成 stamp]
        │  IK 延遲
        ▼
  [JointCmd 送出 stamp]
        │  控制器延遲
        ▼
  [機器臂到達位置 stamp]  ← 用 TF + 攝影機量測 or /joint_states 比對

總端到端延遲 = 最後 stamp - 第一個 stamp
```

**期望指標**：
- 端到端 < 200 ms（人眼感受不到明顯延遲）
- < 100 ms 為優秀
- > 300 ms 會有明顯「手跟不上」感

---

### 階段 5：使用者任務操作實驗 🔲 待完成

```
實驗設計：
  任務 A：抓取固定位置物體（測試精度）
  任務 B：追蹤移動目標（測試延遲感受）
  任務 C：雙臂協作操作（測試同步性）

評估量表：
  - NASA-TLX（認知負荷）
  - 任務完成率（% success）
  - 操作時間（seconds per task）
  - 使用者主觀評分（1–10）
```

---

## 5. 評估指標清單

### 5.1 延遲類

| 指標 | 單位 | 目標值 | 量測工具 |
|---|---|---|---|
| IK 計算延遲（中位數） | ms | < 5 ms | `ik_timing_benchmark.py` ✅ |
| IK 計算延遲（P99） | ms | < 20 ms | `ik_timing_benchmark.py` ✅ |
| OMPL 規劃延遲（中位數） | ms | < 300 ms | 待開發 🔲 |
| 機器人移動時間 | ms | 依路徑長度 | 待分離 🔲 |
| 端到端延遲 | ms | < 200 ms | 待開發 🔲 |

### 5.2 頻率類

| 指標 | 單位 | 目標值 | 量測工具 |
|---|---|---|---|
| IK 最大吞吐量 | Hz | ≥ 100 Hz | `ik_timing_benchmark.py` ✅ |
| MoveGroup 規劃 + 執行頻率 | Hz | ≥ 5 Hz | `csv_waypoint_runner.py` ✅ |
| MoveIt Servo 控制頻率 | Hz | ≥ 100 Hz | 待建立 🔲 |

### 5.3 可達性類

| 指標 | 單位 | 目標值 | 量測工具 |
|---|---|---|---|
| 工作空間單方向可達率 | % | > 70% | `ik_reachability_sampler.py` ✅ |
| 6 方向任一可達率 | % | > 90% | `ik_timing_benchmark --all-orient` ✅ |
| 碰撞規避成功率 | % | > 95% | 待開發 🔲 |

### 5.4 平滑性類

| 指標 | 單位 | 目標值 | 量測工具 |
|---|---|---|---|
| Joint velocity discontinuity | rad/s | < 0.5 rad/s | 待開發 🔲 |
| Max jerk | rad/s³ | 越小越好 | 待開發 🔲 |
| 規劃失敗率 | % | < 5% | 待開發 🔲 |
| EE 位置誤差 | mm | < 5 mm | 待開發 🔲 |

---

## 6. 目前工具清單與狀態

| 檔案 | 功能 | 狀態 |
|---|---|---|
| `ik_timing_benchmark.py` | IK 純延遲基準測試，支援 6 方向 | ✅ 完成 |
| `ik_reachability_sampler.py` | 工作空間可達率採樣 | ✅ 完成 |
| `csv_waypoint_runner.py` | MoveGroup 完整流程執行 + 計時，支援 `--split-timing` / `--all-orient` / `--planner ompl\|pilz_ptp\|pilz_lin` | ✅ 完成 |
| `plot_ee_orientations.py` | EE 方向 3D 視覺化 | ✅ 完成 |
| `plot_reachability_csv.py` | 工作空間可視化 | ✅ 完成 |
| `test_plan_timing_debug.py` | pymoveit2 state machine 計時 unit tests（19 tests） | ✅ 完成 |
| `realtime_ik_controller.py` | 繞過 OMPL 的即時 IK 控制器（PoseStamped → /compute_ik → JointTrajectory） | ✅ 完成（階段 3） |
| `servo_teleop_client.py` | MoveIt Servo 互動遙控 + 基準測試客戶端 | ✅ 完成（階段 2-2） |
| `openarm_bringup/config/servo_right_config.yaml` | MoveIt Servo 右臂設定（100 Hz） | ✅ 完成（階段 2-2） |
| `openarm_bringup/config/servo_left_config.yaml` | MoveIt Servo 左臂設定（100 Hz） | ✅ 完成（階段 2-2） |
| `openarm_bringup/launch/servo_right.launch.py` | Servo 啟動 launch 檔 | ✅ 完成（階段 2-2） |
| `e2e_latency_tester.py` | 端到端延遲量測 | 🔲 待建立（階段 4） |

---

## 7. 下一步行動

### 優先順序

```
[已完成] 階段 1：IK 延遲基準測試 ✅
[已完成] 階段 2：OMPL plan/execute 分離計時 ✅
[已完成] 階段 2-1：Pilz PTP/LIN 規劃器比較 ✅（--planner 旗標已加入）
[已完成] 階段 2-2：MoveIt Servo 設定 ✅（config + launch + client 已建立）
[已完成] 階段 3：realtime_ik_controller.py ✅

[高優先] 階段 2-2 實際測試：
        → 啟動 servo_right.launch.py，發送 TwistStamped
        → 用 servo_teleop_client.py --benchmark 量測輸出延遲
        → 觀察奇異點處理（servo_node/status）

[高優先] 階段 3 實際測試：
        → 啟動 realtime_ik_controller.py --arm right --dry-run 確認 IK 回應
        → 再移除 --dry-run 實際控制機械臂
        → 觀察 /right/ik_latency_ms 確認 1–20 ms 目標

[中優先] 階段 2-1 實際量測：
        → 執行 Pilz PTP vs OMPL 對比測試
        → 生成兩份 timing CSV 並比較分佈（ompl_ms 差異預計 10–100倍）

[低優先] 階段 4, 5：
        → 需要 VR/AR 裝置連接後才能量測端到端延遲
```

### 指令速查

```bash
cd /home/asus/ros2_ws_yh/src/openarm_ros2

# === 階段 1：IK 延遲測試 ===
cd scripts/ik_reachability
python3 ik_timing_benchmark.py --csv right_reachability_.csv --arm right
python3 ik_timing_benchmark.py --csv right_reachability_.csv --arm right --all-orient --save-png

# === 階段 2：OMPL 規劃+執行計時 ===
python3 csv_waypoint_runner.py --csv right_reachability_.csv --arm right \
    --max-pts 30 --split-timing --home-first

# === 階段 2-1：Pilz 比較 ===
python3 csv_waypoint_runner.py --csv right_reachability_.csv --arm right \
    --planner pilz_ptp --max-pts 30 --split-timing --timing-csv pilz_ptp_timing.csv
python3 csv_waypoint_runner.py --csv right_reachability_.csv --arm right \
    --planner ompl --max-pts 30 --split-timing --timing-csv ompl_timing.csv

# === 階段 2-2：MoveIt Servo ===
# Terminal 1：啟動硬體
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
# Terminal 2：啟動 MoveGroup
ros2 launch openarm_bimanual_moveit_config move_group_only.launch.py
# Terminal 3：啟動 Servo
ros2 launch openarm_bringup servo_right.launch.py
# Terminal 4：互動遙控
python3 scripts/ik_reachability/servo_teleop_client.py --arm right
# 或基準測試
python3 scripts/ik_reachability/servo_teleop_client.py --arm right --benchmark

# === 階段 3：realtime IK 控制器 ===
# Terminal 4（替代 Servo）：
python3 scripts/ik_reachability/realtime_ik_controller.py --arm right
# 測試發布位姿：
ros2 topic pub --once /right/target_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: world}, pose: {position: {x: 0.4, y: -0.2, z: 0.3}, \
     orientation: {x: 0.0, y: 0.0, z: -0.707, w: 0.707}}}"
# 監控 IK 延遲：
ros2 topic echo /right/ik_latency_ms
```

---

*最後更新：2026-04-20 — 階段 2-1/2-2/3 完成*
*平台：OpenArm O6 Bimanual · ROS2 Humble · MoveIt2 · Ubuntu 22.04*
