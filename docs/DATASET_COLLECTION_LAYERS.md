# Pico VR Teleop — Dataset Collection 應該從哪一層蒐集？

> 目的：在現有 `pico_vr_bridge → placo_ik → JTC → motor` 的 teleop pipeline 中，
> 釐清「**action**」與「**state**」應該從哪一層 (which topic) 抽取，
> 給未來做 imitation learning / VLA / diffusion policy 的 dataset 蒐集當作 SOP。

---

## TL;DR — 直接告訴我用哪個

| 目的 | Action topic | State topic |
|---|---|---|
| **Joint-space policy（最常見、最穩、推薦）** | `/joint_actions` *(已聚合)* 或 IK output 層 | `/joint_states` |
| **EE-space / 任務空間 policy** | `/ee_delta/{arm}` + `/pico_{side}/grip` | `/joint_states` + TF (EE pose) |
| **End-to-end from VR raw input** | `/pico_{side}/{pose,trigger,grip}` | `/joint_states` |
| **Touch-aware policy** | 同上 + Touch 不放進 action | `/joint_states` + `/cb_{side}_hand_matrix_touch{_mass}` |

> **不要用** JTC `controller_state.reference`（插值後的訊號）當 action — 它已經貼著 state 跑，模型會學到「複製當前位置」這種 trivial policy。

---

## 1. 整個 Pipeline 的層次圖

```
┌──────────────────────────────────────────────────────────────────────────┐
│  LAYER A  ─  Human Intent (Pico controller raw)                          │
│             /pico_{left,right}/pose      (PoseStamped, world-frame)      │
│             /pico_{left,right}/trigger   (Float32 0–1, index trigger)    │
│             /pico_{left,right}/grip      (Float32 0–1, side grip)        │
└──────────────────────────────────────────────────────────────────────────┘
                              │  pico_vr_bridge.py
                              │   - trigger gating（按住才送）
                              │   - 計算 delta = cur − ref_at_press
                              │   - grip → O6 hand pose 線性內插 + LPF
                              ▼
┌──────────────────────────────────────────────────────────────────────────┐
│  LAYER B  ─  Task-space command (EE delta + hand pose)                   │
│             /ee_delta/{arm}                              (PoseStamped)   │
│             /{side}_hand_forward_position_controller/    (Float64Multi)  │
│                 commands                                                 │
└──────────────────────────────────────────────────────────────────────────┘
                              │  placo_ik_online_profiler_ws_mesh.py
                              │   - workspace mesh clamp
                              │   - placo IK solve（cached robot wrapper）
                              ▼
┌──────────────────────────────────────────────────────────────────────────┐
│  LAYER C  ─  Joint-space command (IK output, pre-JTC)                    │
│             /{arm}_joint_trajectory_controller/                          │
│                 joint_trajectory                       (JointTrajectory) │
│             ↑ 7-DoF arm joint targets, 還沒被 JTC 插值                   │
│                                                                          │
│             /{side}_hand_forward_position_controller/                    │
│                 commands                              (Float64MultiArray)│
│             ↑ 手部 6-DoF（Layer B 就是 hand 的最終 command, 直連 motor）│
└──────────────────────────────────────────────────────────────────────────┘
                              │  ros2_control: JointTrajectoryController
                              │   - 在 trajectory points 之間做時間插值
                              ▼
┌──────────────────────────────────────────────────────────────────────────┐
│  LAYER D  ─  JTC interpolated reference (smoothed, dense)                │
│             /{arm}_joint_trajectory_controller/controller_state          │
│                 .reference.positions  (JointTrajectoryControllerState)   │
│             ↑ 真正即時送給 motor 的 setpoint                              │
└──────────────────────────────────────────────────────────────────────────┘
                              │  hardware interface → motor servo loop
                              ▼
┌──────────────────────────────────────────────────────────────────────────┐
│  LAYER E  ─  Hardware feedback (ground-truth state)                      │
│             /joint_states           (JointState, ~500 Hz)                │
│             /follower_joint_states  (JointState, arm only, ~156 Hz)      │
│             /{side}_o6hand_joint_states  (JointState, hand only, ~157Hz) │
│             /cb_{side}_hand_matrix_touch        (String, ~56 Hz)         │
│             /cb_{side}_hand_matrix_touch_mass   (String, ~57 Hz)         │
└──────────────────────────────────────────────────────────────────────────┘

   外加聚合通道 (existing aggregator)：
   /joint_actions   ← LAYER C 的 arm + hand commanded（單一同步 topic）
   /joint_states    ← LAYER E 全身真實 state
```

---

## 2. 每一層當 Action 的取捨

| Layer | Topic | 優點 | 缺點 |
|---|---|---|---|
| **A. Pico raw** | `/pico_*/pose,trigger,grip` | 最接近「人類意圖」；不依賴 robot 任何處理 | • 在 controller world frame，不在 robot frame<br>• Replay 要重做 bridge → IK → JTC 全鏈<br>• 含 ref_pose gating logic，狀態化（需重建 trigger 狀態機）|
| **B. EE delta** | `/ee_delta/{arm}` | 抽象掉 controller 幾何；EE-space 易解釋；replay 容易 | • 是 **相對量** (delta)，需要 session ref 才能還原絕對位姿<br>• Trigger 放開時不發 → 有間斷<br>• 7-DoF arm 對 6-DoF EE 有 redundancy，policy 學不到肘部選擇 |
| **C. IK output (joint cmd)** | `/{arm}_..._/joint_trajectory`<br>`+ /{side}_hand_.../commands` | • 已在 joint space，replay 直接灌 controller<br>• 上游意圖完整保留（IK 唯一解 → 7-DoF arm 全保留）<br>• 與 `/joint_states` 同維度，易做 loss | • Sparse points（每次 solve 一個 point）<br>• 兩個 topic（arm + hand）需要對齊 timestamp |
| **D. JTC reference** | `controller_state.reference` | 插值後 dense, smooth | • **太貼近 state** → policy 容易 collapse 成 identity（學「下一個 state ≈ 當前 state」）<br>• 已經被 JTC 平滑，丟掉 IK 原始 cmd 的高頻意圖 |
| **E. 聚合 `/joint_actions`** | `/joint_actions` | • 單一 topic 含 arm + 雙手 command<br>• 已對齊好（你目前的 recorder 就在用）| • 取決於 aggregator 是抓 Layer C 還是 Layer D — 需要確認來源 |

### 為什麼推薦 Layer C / `/joint_actions`（而不是 Layer A 或 D）

* **Layer A** 的問題：raw VR pose 是「人類想做什麼」，不是「機器要怎麼動」。policy 還要學完整個 bridge + IK，等於把所有 system identification 丟給 model。除非你刻意要做 end-to-end VLA 並且有極大規模 data，否則不划算。
* **Layer D** 的問題：JTC reference 是 motor setpoint，它和 `/joint_states` 的差距只有 servo tracking error（~ms 級延遲 + 補償）。用它當 action，model 等於在學「t+1 state ≈ t state」這種 trivial dynamics。
* **Layer C** 是甜蜜點：上游決策已完成 (IK 解算)，但還沒被 JTC 抹平。它就是「**控制器收到的指令**」，replay 時直接灌回同樣的 controller 就會復現動作。

---

## 3. State 應該抽哪一層

只有一個合理答案：**Layer E**。

| Topic | 內容 | 建議 |
|---|---|---|
| `/joint_states` | 全身（左右 arm + 左右 O6 hand）真實 encoder 位置 | **主 state**，所有 policy 都該用 |
| `/cb_{side}_hand_matrix_touch` | 觸覺矩陣（接觸點分佈）| 接觸密集任務必收 |
| `/cb_{side}_hand_matrix_touch_mass` | 觸覺壓力分布 | 接觸密集任務必收 |
| TF / camera | EE pose / RGB-D | EE-space policy 或 visuomotor policy 必收 |

> Note: 你目前 recorder 同時收 `/follower_joint_states`、`/joint_states`、`/{side}_o6hand_joint_states` 三套是 OK 的（debug 用），但訓練時 **只用 `/joint_states`** 當 state 就好，其他三個是冗餘子集。

---

## 4. 你現有 recorder 對照這個架構

`scripts/gripper/[檔名]_bimanual_o6hand_recorder.py` 目前蒐集到的：

| 你的 topic | 對應 Layer | 角色 |
|---|---|---|
| `/joint_states` | **E** | ✅ State (推薦主訊號) |
| `/joint_actions` | **C → 聚合** | ✅ Action (推薦主訊號) |
| `/follower_joint_states` | E (子集) | 冗餘，可移除或留作 debug |
| `/{side}_o6hand_joint_states` | E (子集) | 冗餘，可移除或留作 debug |
| `/cb_{side}_hand_matrix_touch{,_mass}` | E (sensor) | ✅ State (觸覺) |

**目前架構是正確的**：`/joint_actions` (action) + `/joint_states` (state) 是 standard imitation learning setup。

### 建議補充蒐集（VR pipeline 專屬，給未來想做 hierarchical 或 end-to-end 時保留原始上游訊號）

```python
# Layer A — 人類意圖（給未來 VLA / hierarchical 留路）
self.create_subscription(PoseStamped, '/pico_right/pose',    cb, 10)
self.create_subscription(PoseStamped, '/pico_left/pose',     cb, 10)
self.create_subscription(Float32,     '/pico_right/trigger', cb, 10)
self.create_subscription(Float32,     '/pico_left/trigger',  cb, 10)
self.create_subscription(Float32,     '/pico_right/grip',    cb, 10)
self.create_subscription(Float32,     '/pico_left/grip',     cb, 10)

# Layer B — task-space command（給 EE-space policy 或 hierarchical 高階）
self.create_subscription(PoseStamped, '/ee_delta/right', cb, 10)
self.create_subscription(PoseStamped, '/ee_delta/left',  cb, 10)
```

這幾個 topic **存進 CSV 不影響訓練**（訓練時 policy 只看 state→action），但是有了它們你之後可以：
* 訓練 **hierarchical policy**（高階輸出 EE delta，低階輸出 joint）
* 訓練 **VLA from raw VR**（人類示範 → 直接學 VR controller motion）
* 做 **trajectory replay / debug**（看 IK 失敗時 VR 在做什麼）

---

## 5. 為什麼 `analy_vr_pico_teleop.py` 用 `cmd / ref / state` 三路同步？

那個 script 是 **pipeline debugging tool**，不是 dataset collector。它的三路：

| 別名 | Layer | 用途 |
|---|---|---|
| `cmd`   | C (`joint_trajectory`)        | IK 想要 motor 去哪 |
| `ref`   | D (`controller_state.ref`)    | JTC 插值後實際送給 motor 的 setpoint |
| `state` | E (`/joint_states`)           | motor 真的到了哪 |
| `err`   | `cmd - state`                 | tracking error，給 FFT 找抖動頻率 |

→ 這是為了診斷 **「IK 抖動 vs JTC 平滑 vs servo lag」哪一層是 bottleneck**，
跟 dataset collection 是不同任務。但他剛好示範了「Layer C 和 Layer D 不一樣」這件事 —
**訓練時別用 Layer D 當 action**，用 C。

---

## 6. 蒐集時的時序對齊建議

VR pipeline 各 topic 頻率差很大（Pico ~50Hz, IK output ~100Hz, JTC ~500Hz, touch ~56Hz）。

你目前 recorder 的做法 — **每個 topic 獨立記錄 + timestamp，最後按 timestamp 排序** — 是正確的 (raw, no resample)。
訓練前處理時再做：

1. 選定主時鐘（建議 `/joint_states` 的 stamp，~500Hz 最 dense）
2. 對每一個 sample 時刻，把其他 topic 用 **最近一筆 ≤ t** 的值 join 起來（zero-order hold）
3. Downsample 到 model 訓練頻率（30Hz / 50Hz 視 policy 而定）

**不要在蒐集時 resample** — 一旦 downsample 就拿不回原始訊號，會掩蓋 jitter 問題。

---

## 7. 一句話總結

> 在這個 VR pipeline 裡，**`/joint_actions` 當 action、`/joint_states` 當 state** 就對了。
> 想保留原始人類意圖以後做進階訓練，就「**順便收**」`/pico_*` 和 `/ee_delta/*`，但訓練 loss 別套在它們上。
> JTC 的 `controller_state.reference` 千萬別當 action — 那是 debug 訊號，不是決策訊號。

---

## 相關檔案

| 檔案 | 角色 |
|---|---|
| `scripts/gripper/pico_vr_bridge.py` | Layer A → Layer B 轉換 |
| `placo_ik/ik_node/placo_ik_online_profiler_ws_mesh.py` | Layer B → Layer C 轉換（IK）|
| `scripts/controller_layer/analy_vr_pico_teleop.py` | 抓 cmd/ref/state 三路做 pipeline debug |
| `docs/PICO_O6_HAND_TRIGGER.md` | Pico → O6 hand trigger 詳細說明 |
