# OpenArm VR/AR 遙控操作 — MoveIt2 評估計劃

> 目標：系統性地量測、分析並優化 MoveIt2 對 Apple Vision Pro / Pico 等 VR/AR 裝置
> 的即時遙控能力，建立可重現的評估基準。

---

## 目錄

1. [背景：MoveIt2 規劃流程解析](#1-背景moveit2-規劃流程解析)
2. [跳動問題（Jerk / Discontinuity）深入解釋](#2-跳動問題jerk--discontinuity深入解釋)
3. [OMPL 規劃器比較與推薦](#3-ompl-規劃器比較與推薦)
4. [五階段實驗路線圖](#4-五階段實驗路線圖)
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

## 4. 五階段實驗路線圖

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

### 階段 2：分離 OMPL 規劃時間 vs 機器人移動時間 🔲 待完成

**問題**：目前 `execution_ms` 把這兩者混在一起。

**做法**：使用 MoveGroup `plan()` + 分開 `execute()`

```python
# 需要改寫 csv_waypoint_runner 或新建腳本
t0 = time.perf_counter()
plan_result = moveit2.plan(...)     # 只規劃，不執行
t_plan = time.perf_counter()
moveit2.execute(plan_result)        # 只執行
moveit2.wait_until_executed()
t_exec = time.perf_counter()

ompl_planning_ms = (t_plan - t0) * 1000
robot_motion_ms  = (t_exec - t_plan) * 1000
```

**期望指標**：
- OMPL 規劃時間 < 500 ms（P95）
- 規劃失敗率 < 10%

---

### 階段 3：繞過 OMPL 的即時 IK 控制器 🔲 待完成

```
架構：
  VR/AR (90 Hz) → ROS2 Pose topic
        │
        ▼
  IK node（/compute_ik 或 TRAC-IK）
        │  1–20 ms
        ▼
  JointTrajectory publisher
        │
        ▼
  ros2_control JointTrajectoryController
        │
        ▼
  OpenArm 機械臂
```

**目標**：
- 端到端延遲 < 50 ms
- 控制頻率 ≥ 30 Hz（目標 100 Hz）
- 無跳動（使用 joint seed 連續性）

**關鍵實作細節**：
```python
# 每次 IK 以上一步解作 seed，避免多解跳動
ik_request.ik_request.robot_state.joint_state = last_joint_solution
```

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
| `csv_waypoint_runner.py` | MoveGroup 完整流程執行 + 計時 | ✅ 完成（execution_ms 混合） |
| `plot_ee_orientations.py` | EE 方向 3D 視覺化 | ✅ 完成 |
| `plot_reachability_csv.py` | 工作空間可視化 | ✅ 完成 |
| `moveit_plan_only_runner.py` | 只規劃不執行，分離 OMPL 時間 | 🔲 待建立（階段 2） |
| `realtime_ik_controller.py` | 繞過 OMPL 的即時控制器 | 🔲 待建立（階段 3） |
| `e2e_latency_tester.py` | 端到端延遲量測 | 🔲 待建立（階段 4） |

---

## 7. 下一步行動

### 優先順序

```
[高優先] 階段 2：moveit_plan_only_runner.py
        → 分離 OMPL 規劃時間 vs 機器人移動時間
        → 找出 OMPL 的真正瓶頸在哪裡

[高優先] 階段 3：realtime_ik_controller.py
        → 這是 AR/VR 遙控的核心路徑
        → 繞過 OMPL，直接 IK → JointTrajectory
        → 使用 joint seed 連續性解決跳動

[中優先] OMPL 規劃器替換實驗
        → 比較 RRT-Connect vs Pilz PTP vs MoveIt Servo
        → 針對 AR/VR 場景選最佳規劃器

[低優先] 階段 4, 5
        → 需要 VR/AR 裝置連接後才能做
```

### 指令速查

```bash
# 階段 1：IK 延遲測試
cd scripts/ik_reachability
python3 ik_timing_benchmark.py --csv right_reachability_.csv --arm right
python3 ik_timing_benchmark.py --csv right_reachability_.csv --arm right --all-orient --save-png

# 視覺化 EE 方向
python3 plot_ee_orientations.py

# 工作空間可視化
python3 plot_reachability_csv.py --csv right_reachability_.csv

# csv waypoint runner（完整 MoveGroup 流程）
python3 csv_waypoint_runner.py --csv right_reachability_.csv --arm right --max-pts 20
```

---

*最後更新：2026-04-17*
*平台：OpenArm O6 Bimanual · ROS2 Humble · MoveIt2 · Ubuntu 22.04*
