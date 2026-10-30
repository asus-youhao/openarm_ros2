# `/joint_actions` aggregator

`joint_actions_aggregator.py` 以 50 Hz 發布**固定 26 維**的 `/joint_actions`
（`sensor_msgs/JointState`），給資料收集 / 模仿學習當 action：每個部位有送給
ForwardPositionController（FPC）的命令就記命令，**沒有命令就發 `/joint_states`**。

設計、與 phu 的差異、常數與驗證結果見 [docs/JOINT_ACTIONS.md](../../docs/JOINT_ACTIONS.md)。

## 誰會起它

`openarm_o6_bimanual.launch.py` 預設就會帶起（`with_joint_actions:=true`），它是
`/joint_actions` **唯一**的 publisher，一啟動就以 50 Hz 發布。IK（placo 或其他）**不再**起
aggregator，只要把命令送到 FPC 就會被記錄。

```bash
# 錄資料：FPC 模式（等同 arm_o6_p）
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py \
    robot_controller:=forward_position_controller

# 不要 aggregator
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py with_joint_actions:=false
# 之後單獨起（bringup 沒起的時候才可以，同一時間只能有一個）
ros2 launch openarm_bringup joint_actions.launch.py
```

bringup 參數：

| 參數 | 預設 | 說明 |
|---|---|---|
| `with_joint_actions` | `true` | 是否帶起 aggregator |
| `aggregator_arm_config` | `both` | `both` / `left` / `right`；沒訂閱的手臂用 `/joint_states` |
| `aggregator_hand_config` | `o6_both` | `none` / `o6_left` / `o6_right` / `o6_both`；輸出永遠 26 維 |

JTC 模式（`robot_controller:=joint_trajectory_controller`，等同 `arm_o6_j`）下 FPC 沒有 load，
送到 FPC topic 的命令一律拒收，`/joint_actions` 就是 `/joint_states`（JTC 動手臂時也跟著動）。

## 訂閱 / 發布

| Topic | 型別 | 用途 |
|---|---|---|
| `/left_forward_position_controller/commands` | `Float64MultiArray` (7) | 左臂命令（取最新一筆） |
| `/right_forward_position_controller/commands` | `Float64MultiArray` (7) | 右臂命令 |
| `/left_hand_forward_position_controller/commands` | `Float64MultiArray` (6) | 左手命令 |
| `/right_hand_forward_position_controller/commands` | `Float64MultiArray` (6) | 右手命令 |
| `/joint_states` | `JointState` | 沒有命令的部位用它 |
| **`/joint_actions`**（發布） | `JointState` (26) | 左臂 7 → 右臂 7 → 左手 6 → 右手 6 |

每個 FPC 命令的 slot 對應哪個關節，是向該 controller 讀它的 `joints` 參數得到的，不寫死。
輸出順序固定為 `ALL_JOINT_NAMES`，必須跟 vla_control
`get_joint_order("openarm_linkerhand_o6")` 一致。

## 發布規則

每個 tick（預設 50 Hz，一啟動就開始），每個部位各自決定：

| 這個部位的狀況 | 這一筆的值 |
|---|---|
| 有活著的命令（最後一筆在 `max(3 × 實測週期, 30 ms)` 內到達） | 該命令 |
| 從沒收過命令 / 命令停了 / 命令被拒收 / 沒訂閱 | `/joint_states` |
| 需要補的關節 `/joint_states` 裡沒有 | 這一筆不發（不填 0.0） |

- 完全沒有命令時，`/joint_actions` 就是 `/joint_states`。
- 命令停掉、恢復時各印一行 log（見下面「常見 log」）。
- **拒收**：controller 還沒回報 `joints`（沒 load）、`joints` 跟預期的關節集合不同、長度不對。
- **`header.stamp`**：發布當下。
- 沒有命令時記下的 action 等於 observation（`/joint_states`），episode 起訖請自己控制；
  細節見 [docs/JOINT_ACTIONS.md](../../docs/JOINT_ACTIONS.md)「用 `/joint_states` 補要知道的事」。

## 單獨執行腳本

```bash
python3 scripts/data_collection/joint_actions_aggregator.py
python3 scripts/data_collection/joint_actions_aggregator.py --hand_config none
python3 scripts/data_collection/joint_actions_aggregator.py --stale_timeout 0.2
```

參數：`--arm_config`、`--hand_config`、`--publish_rate`（發布頻率，預設 50 Hz）、
`--stale_timeout`（命令多久沒更新就改用 `/joint_states`；0 = 自動）、
`--{left,right}_{arm,hand}_controller`（換 controller 名稱）。

## 在 fake hardware 上試

```bash
# Terminal 1
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py use_fake_hardware:=true \
    robot_controller:=forward_position_controller
# Terminal 2：假上游（只能用在 fake hardware，偵測到真硬體會拒絕執行）
python3 scripts/data_collection/fake_arm_hand_feed.py
# Terminal 3
ros2 topic hz /joint_actions
ros2 topic echo /joint_actions --once --field name
```

## 自動測試

兩支都只用 fake hardware / 假 controller，各自挑一個沒人在用的 `ROS_DOMAIN_ID`，
discovery 限制在 localhost，不會碰到別人的機器人：

```bash
# 一定要 source 這個 branch build 出來的 workspace（舊的 openarm_ws 沒有 aggregator，
# check_bringup_fake.py 會直接拒絕執行並提示）
source ~/work/openarm/openarm_jact_ws/install/local_setup.bash
python3 scripts/data_collection/test/check_rules.py         # 規則 R1–R14（約 1.5 分鐘）
python3 scripts/data_collection/test/check_bringup_fake.py  # bringup 端對端 F/J/O（約 2 分鐘）

# 想看 RViz：開 RViz，每個 case 測完停 10 秒（F case 停留時手臂持續正弦運動）
python3 scripts/data_collection/test/check_bringup_fake.py --rviz
python3 scripts/data_collection/test/check_bringup_fake.py --rviz --cases F --hold 30
```

## 常見 log

| log | 意思 |
|---|---|
| `<limb> slot order (from /<controller>): [...]` | 讀到 controller 的 `joints`，開始接受該來源的命令 |
| `slot order not known yet for ...` | 那些 controller 還沒 load（或 JTC 模式），它們的命令不會被錄 |
| `<limb>: slot order unknown ... not recorded` | 有命令送到沒 load 的 FPC topic，被拒收 |
| `no command since startup on [...]` | 這些來源從沒送過命令，欄位用 `/joint_states` |
| `<limb>: no command for …ms — filling from /joint_states` | 命令停了，這個部位改發 `/joint_states` |
| `<limb>: commands arriving — recording them` | 命令來了（或恢復），改記命令 |
| `<limb>: no live command and [...] absent from /joint_states — skipping this sample` | 要補的關節 `/joint_states` 裡沒有，這筆不發 |
