# Joint Actions Aggregator

## 概述

`joint_actions_aggregator.py` 是一個 ROS2 節點，用於將多個關節命令來源整合到單一的 `/joint_actions` topic，方便數據採集和模仿學習。

## 架構

```
┌─────────────────┐
│ placo_ik_node   │  (left arm)
│                 │──► /left_arm_ik_commands (JointState, 7 joints)
└─────────────────┘

┌─────────────────┐
│ placo_ik_node   │  (right arm)
│                 │──► /right_arm_ik_commands (JointState, 7 joints)
└─────────────────┘

┌─────────────────┐
│ Hand Controller │  (O6)
│                 │──► /left_hand_forward_position_controller/commands (Float64MultiArray, 6 joints)
│                 │──► /right_hand_forward_position_controller/commands (Float64MultiArray, 6 joints)
└─────────────────┘
         │
         │ (all subscribe by aggregator)
         ▼
┌─────────────────────────┐
│ joint_actions_          │
│ aggregator              │
│                         │
└─────────────────────────┘
         │
         ▼
    /joint_actions (JointState, all joints combined)
```

## 訂閱的 Topics

- `/left_arm_ik_commands` (JointState) — 左臂 IK 解算結果，7 個關節
- `/right_arm_ik_commands` (JointState) — 右臂 IK 解算結果，7 個關節
- `/left_hand_forward_position_controller/commands` (Float64MultiArray) — 左手命令，6 個關節 (O6)
- `/right_hand_forward_position_controller/commands` (Float64MultiArray) — 右手命令，6 個關節 (O6)

## 發佈的 Topics

- `/joint_actions` (JointState) — 整合後的所有關節命令

## 手掌配置選項

使用 `--hand_config` 參數選擇手掌配置：

`--hand_config` 只決定**訂閱哪些手掌**，不決定輸出維度。`/joint_actions` 永遠是
固定的 26 維（7+7+6+6），沒訂閱的手掌一樣佔欄位，用 `/joint_states` 的實測值填。

| 配置 | 說明 | 總關節數 |
|------|------|----------|
| `o6_both` | 雙臂 + O6 雙手（預設） | 26 |
| `o6_left` | 只訂閱左手，右手用實測值填 | 26 |
| `o6_right` | 只訂閱右手，左手用實測值填 | 26 |
| `none` | 兩手都不訂閱，都用實測值填 | 26 |

維度固定是**必要的**：下游 `data_collector` 依關節名稱查表，只要少一個預期的名稱
就會丟棄整筆樣本（`joint_action=None`），結果是整場錄不到任何一幀而且不報錯。
`--arm_config` 同理，閒置的手臂一樣佔 7 個欄位。

## 發布條件

**事件驅動，沒有固定頻率的 timer** —— 每當所有 active 手臂都送來新命令時發布一次，
所以 `/joint_actions` 直接繼承 IK 節點的頻率與相位，不會多做一次重取樣
（下游 collector 還會再依自己的 `control_hz` 取樣一次）。

每個 limb 第一次收到命令時「latch」成 active，之後整場維持。關鍵區別：

| limb 狀態 | 該欄位的值 | 擋不擋整組 |
|---|---|---|
| config 沒選中（不訂閱） | `/joint_states` | 不擋 |
| 有訂閱、**從沒收到過** | `/joint_states` + 持續 warn | 不擋 |
| 有訂閱、收到過、**新鮮** | command | — |
| 有訂閱、收到過、**stale** | — | **擋，整組不發** |
| 兩臂都沒 latch 過 | — | **擋，整組不發** |

- **從沒收到過** = 上游沒開 → 該 limb 物理上靜止，用實測值填是誠實的，不列管。
- **收到過然後靜默** = 上游故障或暫停 → **停止發布**，而不是重發凍住的舊值。

停止發布是刻意的：collector 量的是「最後一則 `/joint_actions` 的年齡」，
持續重發凍住的值會讓它誤判成全程即時；靜默才能讓它正確把該 episode 標記為
非即時（`_episode_stale_ticks`）。停止發布不會在資料裡打洞 —— collector 會沿用
上一筆湊滿 tick，obs/action 陣列仍然對齊。

stale 門檻依各來源**實測**的到達間隔自動推導（3 個週期，下限 30 ms），
所以 50 Hz 的手臂與 100 Hz 的手掌各自有合理的窗口，不需手動設定。
必要時可用 `--stale_timeout` 覆寫成固定值。

**不會**用 `/joint_states` 去頂替故障的來源：collector 的 observation 也是
`/joint_states`，替代進去的 action 會變成 observation 的逐欄位複製，
訓練出「輸出 = 當前狀態」的原地不動 policy。

## 使用方法

### 1. 雙臂 + O6 雙手（預設配置）
```bash
python3 scripts/data_collection/joint_actions_aggregator.py
# 或明確指定
python3 scripts/data_collection/joint_actions_aggregator.py --hand_config o6_both
```

### 2. 雙臂 + O6 左手
```bash
python3 scripts/data_collection/joint_actions_aggregator.py --hand_config o6_left
```

### 3. 雙臂 + O6 右手
```bash
python3 scripts/data_collection/joint_actions_aggregator.py --hand_config o6_right
```

### 4. 僅雙臂（無手掌）
```bash
python3 scripts/data_collection/joint_actions_aggregator.py --hand_config none
```

### 5. 調整 stale 門檻
```bash
# --publish_rate 只用來在量測到實際頻率前預設 stale 窗口，不決定發布頻率
python3 scripts/data_collection/joint_actions_aggregator.py --publish_rate 50.0

# 覆寫成固定的 stale 窗口（秒），停用自動推導
python3 scripts/data_collection/joint_actions_aggregator.py --stale_timeout 0.1
```

## 完整工作流

### 啟動順序

1. **啟動 IK 節點**（雙臂單一 QP，會同時發布兩臂的 `*_arm_ik_commands`）
   ```bash
   python3 placo_ik/ik_node/placo_ik_main_bimanual.py
   ```

   注意：這個入口本身就會 in-process 起一個 aggregator。若同時用 launch 起
   獨立的 aggregator，`/joint_actions` 會有兩個 publisher —— 擇一即可。

2. **啟動機器人與手掌控制器**
   ```bash
   ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
   ```

   這支 launch 預設就會一併帶起 aggregator
   （`launch_joint_actions_aggregator:=true`），組態用 `aggregator_*` 參數調整：
   ```bash
   ros2 launch openarm_bringup openarm_o6_bimanual.launch.py \
       aggregator_hand_config:=o6_right
   ```

3. **單獨啟動聚合節點**（只有在上一步關掉它時才需要）
   ```bash
   python3 scripts/data_collection/joint_actions_aggregator.py --hand_config o6_both
   ```

4. **驗證輸出**
   ```bash
   ros2 topic echo /joint_actions
   ```

## 關節順序

`/joint_actions` 消息中的關節順序：

1. **左臂** (7 joints): `openarm_left_joint1-7`
2. **右臂** (7 joints): `openarm_right_joint1-7`
3. **左手** (6 joints, 如果啟用): `L_index_mcp_pitch`, `L_middle_mcp_pitch`, `L_pinky_mcp_pitch`, `L_ring_mcp_pitch`, `L_thumb_cmc_pitch`, `L_thumb_cmc_yaw`
4. **右手** (6 joints, 如果啟用): `R_index_mcp_pitch`, `R_middle_mcp_pitch`, `R_pinky_mcp_pitch`, `R_ring_mcp_pitch`, `R_thumb_cmc_pitch`, `R_thumb_cmc_yaw`

**注意**：手部關節順序與 `{left|right}_hand_forward_position_controller` YAML 配置一致。

## 故障排除

### 問題：聚合節點一直顯示 "Waiting for arm commands..."

**原因**：還沒有收到左臂或右臂的 IK 命令

**解決方案**：
1. 確認兩個 placo_ik_node 都在運行
2. 檢查 IK 節點是否正確發佈到中間 topics：
   ```bash
   ros2 topic list | grep arm_ik_commands
   # 應該看到：
   # /left_arm_ik_commands
   # /right_arm_ik_commands
   ```
3. 檢查 IK 節點是否有收到 EE delta 命令：
   ```bash
   ros2 topic echo /ee_delta/left
   ros2 topic echo /ee_delta/right
   ```

### 問題：`/joint_actions` 中途停止發布

**原因**：某個已 latch 的來源超時（上游故障、暫停、或斷流）。這是設計行為。

**解決方案**：看 log，會直接指出是哪個 limb、多久沒更新：
```
[WARN] stale: right_hand=140ms — /joint_actions paused until it recovers
```
來源恢復後會自動繼續發布。注意 placo_ik 的 keyboard pause 也會讓手臂停止送
command，因此按 pause 期間 `/joint_actions` 會靜默。

### 問題：一直 warn 某個 limb "no command since startup"

**原因**：該來源訂閱了但從未發布過 —— 上游沒啟動。該 limb 的欄位正在用
`/joint_states` 的實測值填。

**解決方案**：如果是刻意不開，用 `--hand_config` / `--arm_config` 明確宣告即可
（維度不變）。如果是忘了啟動上游，這個警告就是在提醒你 —— 它會持續出現，
因為「刻意不開」和「忘了開」在程式眼裡一模一樣。

### 問題：完全不發布，log 說 withholding

**原因**：某個要用實測值填的 limb，其關節根本不在 `/joint_states` 裡
（例如手掌硬體/控制器沒起來）。此時沒有任何誠實的值可填，因此不發布，
而不是捏造 0.0。

**解決方案**：確認該關節有出現在 `/joint_states`。註：這種情況下 collector
的 observation 也組不出來，本來就錄不到東西。

### 問題：關節數量不符合預期

**原因**：手掌配置選擇錯誤

**解決方案**：
- 檢查啟動日誌中的 "Total joints" 數量
- 確認 `--hand_config` 參數正確
- 如果使用 O6 hand，應該選擇 `o6_left`、`o6_right` 或 `o6_both`（預設）

## 與數據採集的整合

聚合後的 `/joint_actions` topic 可以直接用於：

1. **數據記錄**（用於模仿學習）
   ```bash
   ros2 bag record /joint_actions /joint_states /camera/image_raw
   ```

2. **與現有 recorder 整合**
   您現有的數據採集腳本應該已經訂閱了 `/joint_actions`，現在會自動收到整合後的命令。

3. **VLA 訓練**
   `/joint_actions` 作為 action，`/joint_states` 作為 state，這是標準的模仿學習設置。

## 性能考量

- **發佈頻率**：等於手臂命令的頻率（事件驅動），不是固定 timer。兩臂會 rendezvous
  在同一個 IK solve step 上，因此兩臂的值必定同源，不會跨兩個 step。
- **延遲**：在手臂命令的 callback 裡直接發布，不額外排隊等 timer。
- **手掌 100 Hz**：發布仍由手臂觸發，手掌只取當下最新值（最多 10 ms 舊，比手臂自己的
  20 ms 週期還新）。刻意不由手掌觸發 —— 那會讓 `/joint_actions` 變成 100 Hz，
  其中一半是手臂值的重複幀，而 collector 還是只取自己的 30 Hz。
- **缺失數據處理**：從未啟動的來源用 `/joint_states` 填；已 latch 的來源 stale 則
  停止發布。任何情況都不會捏造數值。
- **訊息驗證**：手臂命令優先依 `msg.name` 對應（發布端改順序不會靜默錯位），
  沒帶 name 時退回位置對應但檢查長度；手掌的 `Float64MultiArray` 檢查長度。
  不合格的訊息會被丟棄並 warn，不會污染輸出。
- **時間戳**：`header.stamp` 填的是**最舊那個來源**的時間，不是發布當下，
  因此讀 `header.stamp` 的消費端看到的是這筆樣本的真實年齡。

## 相關文件

- [DATASET_COLLECTION_LAYERS.md](../docs/DATASET_COLLECTION_LAYERS.md) — 數據採集層次說明
- [placo_ik_node.py](../placo_ik/ik_node/placo_ik_node.py) — IK 解算節點
- [arm_config.py](../placo_ik/config/arm_config.py) — 手臂配置
# 控制架構與延遲說明

## 架構概覽

### 修改前（原始架構）
```
placo_ik_node
    │
    ├─► /left_forward_position_controller/commands  ───► 硬體 (直接控制)
    └─► /right_forward_position_controller/commands ───► 硬體 (直接控制)
```

### 修改後（雙路徑架構）
```
placo_ik_node (left)
    │
    ├─► /left_forward_position_controller/commands ───────────► 硬體 (直接控制，無延遲)
    └─► /left_arm_ik_commands ──┐
                                 │
placo_ik_node (right)            │
    │                            ├──► joint_actions_aggregator
    ├─► /right_forward_position_controller/commands ──────────► 硬體 (直接控制，無延遲)
    └─► /right_arm_ik_commands ─┘               │
                                                 │
Hand controllers                                 │
    ├─► /left_hand_forward_position_controller/commands ──────► 硬體 (直接控制)
    └─► /right_hand_forward_position_controller/commands ─────► 硬體 (直接控制)
         │                        │
         └────────────────────────┘
                  │
                  ▼
            /joint_actions (數據採集用，不影響控制)
```

## 關鍵設計原則

### ✅ 控制迴路完全不受影響

1. **硬體控制路徑保持不變**
   - placo_ik_node 仍然直接發布到 `/left_forward_position_controller/commands`
   - 手掌控制器仍然直接發布到 `/left_hand_forward_position_controller/commands`
   - **沒有額外的中間層**

2. **aggregator 是旁路設計**
   - aggregator 只是「監聽」各個控制命令
   - 然後「複製」並整合到 `/joint_actions`
   - **不參與實際控制迴路**

### 📊 延遲分析

#### 控制命令延遲：**0ms 額外延遲**
```
IK solve → publish 到 controller commands → 硬體
```
這條路徑完全沒有改變，延遲保持原樣（通常 < 1ms）。

#### /joint_actions 數據採集延遲：**最多 20ms**
```
IK solve → /left_arm_ik_commands → aggregator (50Hz timer) → /joint_actions
```
- aggregator 以固定 50Hz (20ms 週期) 讀取最新命令並發布
- 最壞情況：命令剛好在 timer 觸發後到達，需要等下一個週期
- 實際延遲：平均 10ms，最大 20ms

## 為什麼這個設計安全？

### 1. 控制與數據採集分離
- **控制** (control loop)：placo_ik → controller → 硬體，直接、即時
- **數據採集** (data recording)：aggregator → /joint_actions，允許有緩衝

### 2. 數據採集的延遲可接受
- 模仿學習的數據採集通常在 10-50Hz
- 20ms 延遲在這個頻率下是可以接受的
- 重要的是數據的**一致性**和**對齊**，而不是絕對的即時性

### 3. 時間戳保證順序
- aggregator 發布的 `/joint_actions` 帶有時間戳
- 數據記錄時可以根據時間戳對齊不同來源的數據

## 性能影響評估

### CPU 負擔
- **placo_ik_node**：額外發布一個 JointState topic
  - 增加：~0.1-0.2ms per cycle
  - 影響：可忽略（<1%）

- **aggregator**：新增的獨立節點
  - 運行：~0.5ms per cycle (50Hz)
  - 影響：微小（在獨立線程中運行）

### 總體影響
- 原本 IK cycle time：0.5-2ms (cached mode)
- 額外開銷：< 0.3ms
- **結論：對控制頻率幾乎無影響**

## 實際測試建議

### 驗證控制即時性
```bash
# 查看控制命令頻率（應該保持 50Hz）
ros2 topic hz /left_forward_position_controller/commands

# 查看 /joint_actions 頻率（應該是 50Hz）
ros2 topic hz /joint_actions

# 查看延遲
ros2 topic delay /joint_actions
```

### 壓力測試
```bash
# 同時運行雙臂 + aggregator
python3 scripts/ik_reachability/ik_controllers/placo_ik_online_profiler_ws_mesh.py --arm both

# 觀察 IK latency（應該保持在正常範圍）
ros2 topic echo /left/delta_ik_latency_ms
ros2 topic echo /right/delta_ik_latency_ms
```

## 常見問題

### Q1: aggregator 的 50Hz 會拖慢 IK 嗎？
**A:** 不會。aggregator 在獨立線程中以獨立的 timer 運行，不會阻塞 IK 的計算或發布。

### Q2: 如果 IK 跑 100Hz，aggregator 跑 50Hz，會漏數據嗎？
**A:** 不會漏，但會**降採樣**。aggregator 每 20ms 讀取一次最新命令：
- IK 發布：0ms, 10ms, 20ms, 30ms, 40ms, 50ms...
- aggregator 讀取：0ms, 20ms, 40ms, 60ms...
- 結果：數據採集得到 50Hz 的命令流，足夠用於訓練

### Q3: 能不能讓 aggregator 更快？
**A:** 可以，使用 `--publish_rate` 參數：
```bash
python3 scripts/data_collection/joint_actions_aggregator.py --publish_rate 100.0
```
但通常 50Hz 已經足夠，因為：
- VLA 訓練的典型頻率：10-50Hz
- 更高頻率會增加數據量，但不一定提升性能

### Q4: 為什麼不直接讓 placo_ik_node 發布到 /joint_actions？
**A:** 因為需要整合多個來源：
- 左臂 IK (7 joints)
- 右臂 IK (7 joints)
- 左手控制器 (6 joints)
- 右手控制器 (6 joints)

單個 IK 節點無法看到其他來源的數據，需要 aggregator 集中整合。

## 結論

✅ **控制延遲：0ms 額外延遲** - 硬體控制迴路完全不受影響

✅ **數據採集延遲：~10-20ms** - 對模仿學習數據收集完全可接受

✅ **性能影響：< 1%** - CPU 和記憶體開銷可忽略

⚠️ **注意事項**：aggregator 必須在 IK 節點啟動後才能收到數據，建議：
1. 先啟動雙臂 IK：`--arm both`
2. aggregator 自動啟動
3. 或手動啟動手掌控制器

如果使用 `placo_ik_online_profiler_ws_mesh.py --arm both`，aggregator 會自動啟動！
