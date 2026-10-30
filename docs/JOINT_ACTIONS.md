# `/joint_actions`：由 bringup 帶起、有命令記命令、沒命令發 `/joint_states`

## 一句話

`/joint_actions`（26 維 `JointState`，給資料收集 / 模仿學習當 action）改由
`openarm_o6_bimanual.launch.py` 帶起的**單一** aggregator 以 50 Hz 發布：每個部位有
「還活著的」ForwardPositionController（FPC）命令就記命令，**沒有命令就發 `/joint_states`**。
發布方式與 `phu_controller_ros2` #98 相同（固定頻率 timer），差別在命令停掉之後：#98
繼續發最後一筆命令，這裡改發 `/joint_states`。

## 跟 develop `b170a61` 舊版比

| # | 舊版 | 新版 |
|:-:|---|---|
| 1 | 由 IK 程序在 process 內起（兩個入口各自起），IK 沒跑就沒有 `/joint_actions` | bringup 帶起，`/joint_actions` **只有一個** publisher；IK 不再起 |
| 2 | 手臂來源是 IK 的旁路 topic `/{arm}_arm_ik_commands` | 四個 FPC 實際收到的命令 `/<FPC>/commands` |
| 3 | `list(msg.position)`，不看關節名；手掌順序寫死 | 每個 slot 是哪個關節，向 controller 讀它自己的 `joints` 參數 |
| 4 | `arm_o6_j`（FPC 沒 load）下 IK 照發旁路，手臂沒動卻錄到命令 | FPC 沒 load 就讀不到 `joints`，命令一律拒收 → 發 `/joint_states` |
| 5 | 上游停了繼續重發最後一筆 | 上游停了（超過 stale 門檻）改發 `/joint_states` |
| 6 | `/joint_states` 找不到的關節填 0.0 | 找不到就跳過這一筆，不捏造數值 |
| 7 | 維度隨 `--hand_config` 變（14 / 20 / 26） | 永遠 26 維、固定順序 |
| 8 | Ctrl-C 收到兩次 SIGINT 時 traceback、`exit code -2` | 乾淨結束 |

## 規則

```
openarm_o6_bimanual.launch.py (with_joint_actions:=true，預設)
  └─ joint_actions.launch.py ─ ExecuteProcess ─ joint_actions_aggregator.py   ← /joint_actions 唯一的 publisher

  /left_forward_position_controller/commands         ┐
  /right_forward_position_controller/commands        │ 每個部位的命令（只保留最新一筆）
  /left_hand_forward_position_controller/commands    │
  /right_hand_forward_position_controller/commands   ┘
  /joint_states                                         沒有命令的部位用它
  50 Hz timer ───────────────────────────────────────► /joint_actions（26 維）
```

每個 tick（預設 50 Hz，從啟動開始，不管有沒有命令），每個部位各自決定：

| 這個部位的狀況 | 這一筆的值 |
|---|---|
| 有**活著的命令**（最後一筆在 stale 門檻內到達） | 該命令 |
| 從沒收過命令 | `/joint_states` |
| 收過命令，但已停超過 stale 門檻 | `/joint_states`（log：`<limb>: no command for …ms — filling from /joint_states`） |
| 命令恢復 | 命令（log：`<limb>: commands arriving — recording them`） |
| FPC 沒 load / `joints` 不符 / 長度不對（命令被拒收） | `/joint_states` |
| config 沒訂閱這個部位（`aggregator_arm_config` / `aggregator_hand_config`） | `/joint_states` |
| 需要補的關節 `/joint_states` 裡沒有 | **這一筆不發**（不填 0.0） |

- 完全沒有命令時，`/joint_actions` 就是 `/joint_states`（照固定順序重排成 26 維）。
- stale 門檻 = `max(3 × 該來源實測週期, 30 ms)`：50 Hz 的手臂約 60 ms、30 Hz 的手掌約 100 ms。
  上游比較慢（例如 20 Hz）時門檻跟著變寬，兩筆命令之間仍算「活著」。
- 誰送命令都一樣：placo IK、其他 IK、VR teleop、腳本。aggregator 不依賴 placo IK。
- 輸出永遠是 `ALL_JOINT_NAMES` 的 26 個名稱、固定順序（左臂 7、右臂 7、左手 6、右手 6，
  必須跟 vla_control `get_joint_order("openarm_linkerhand_o6")` 一致）。
- `header.stamp` = 發布當下。

## 用 `/joint_states` 補要知道的事

這是刻意的選擇（沒有命令時也要有連續的資料），代價如下，錄資料時要心裡有數：

- **補進去的 action 等於 observation**：collector 的 observation 也是 `/joint_states`。
  沒有命令的那段時間（例如 teleop 放開 trigger、上游當掉），樣本會是「action = 目前狀態」。
  episode 的起訖要由錄資料的人 / collector 控制，避免大段這種樣本混進訓練資料。
- **真機上數值不等於 FPC 正在 hold 的目標**：上游停了之後 FPC 仍 hold 最後一筆命令，
  手臂在重力下會有追蹤誤差，`/joint_states` 跟那筆命令會差一點。（fake hardware 上兩者相同。）
- **上游卡頓超過 stale 門檻**，中間那幾筆會變成 `/joint_states`。需要時用
  `--stale_timeout`（或 `joint_actions.launch.py stale_timeout:=…`）放寬。
- **上游死掉看不出來**：資料流不會斷。要看上游狀態請看 aggregator 的 log。

## 跟 phu 的比較

| | phu #84～#91（事件驅動） | phu #98（最新） | 這裡 |
|---|---|---|---|
| 發布 | 收到兩臂命令才發 | 50 Hz timer | 50 Hz timer |
| 從沒收過命令 | `/joint_states` | `/joint_states` | `/joint_states` |
| 命令停了 | 停發 | 繼續發最後一筆命令 | **`/joint_states`** |
| 兩臂都沒命令 | 不發 | `/joint_states` | `/joint_states` |
| `/joint_states` 也沒有 | 不發 | 0.0 | 不發 |
| 寬度 | 26（+頭/腰時 29/32） | 永遠 32，沒帶起的部位 NaN | 永遠 26（openarm 沒有頭/腰） |
| slot 順序 | 手掌寫死，頭/腰讀 `joints` | 同左 | 四個 FPC 都讀 `joints` |
| 誰帶起 | bringup（#90） | bringup | bringup |

這個 branch 一開始實作的是 #84～#91 的事件驅動版（`fb07a5e`），後來依需求改成「沒有命令發
`/joint_states`」的 timer 版。

## 常數與調整

| 常數 / 參數 | 值 | 作用 |
|---|---|---|
| `--publish_rate` | 50 Hz | `/joint_actions` 發布頻率 |
| `STALE_PERIODS` | 3.0 | 超過 3 個實測週期沒新命令就改用 `/joint_states` |
| `STALE_FLOOR_SEC` | 0.03 s | stale 門檻下限 |
| EMA α | 0.15 | 每個來源的到達週期估計（≥ 1 s 的間隔不計） |
| `ARM_SEED_RATE` / `HAND_SEED_RATE` | 50 / 30 Hz | 週期估計的初值（placo IK 50 Hz、`pico_o6_hand_trigger.py` 30 Hz） |
| `--stale_timeout` | 0 = 自動 | 給 > 0 就所有來源固定用這個門檻（秒） |
| `--{left,right}_{arm,hand}_controller` | 四個 FPC 名稱 | 換 controller 名稱時用 |

## 真機上怎麼開、怎麼關

```bash
# 錄資料：FPC 模式（arm_o6_p），bringup 會一起帶起 aggregator
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py robot_controller:=forward_position_controller ...

# JTC 模式（arm_o6_j）：aggregator 一樣會起，/joint_actions = /joint_states
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py robot_controller:=joint_trajectory_controller ...

# 不要 aggregator
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py ... with_joint_actions:=false
# 之後要開再單獨起（同一時間只能有一個）
ros2 launch openarm_bringup joint_actions.launch.py
```

## 已知限制

- **home / unfold 的 ramp 也會被記成命令**：IK 的 homing 也是送 FPC 命令。
- **FPC 有 load 但 inactive**（例如切回 JTC 之後）時，送到 FPC topic 的命令仍會被當成命令記錄，
  aggregator 不檢查 controller 狀態（與 phu 相同）。
- **手掌上游要定頻**：只在變化時才發的上游，兩次之間會被判成沒有命令而改發 `/joint_states`。
- **不支援 `arm_prefix` 命名空間**：topic / 參數服務用的是絕對名稱。
- aggregator 腳本在 repo 的 `scripts/`（不屬於任何 package），由 `openarm_bringup` 的
  `CMakeLists.txt` 安裝到 `share/openarm_bringup/scripts/data_collection/`，有沒有
  `--symlink-install` 都找得到。

## 驗證

| 項目 | 內容 | 結果 |
|---|---|---|
| `scripts/data_collection/test/check_rules.py` | 只跑 aggregator + 假 controller（`joints` 故意打亂）、`/joint_states` 用命令不會用到的值：R1–R14（沒命令、只有手掌、有命令、命令停/恢復、手掌停、拒收、20 Hz 上游、`/joint_states` 缺關節、單臂、controller 沒 load / `joints` 不符、config） | 15/15 PASS |
| `scripts/data_collection/test/check_bringup_fake.py` | 真的 bringup（`use_fake_hardware:=true`）：FPC 模式 F1–F7、JTC 模式 J1–J5（含 JTC 軌跡移動時 `/joint_actions` 跟著 `/joint_states`）、`with_joint_actions:=false` 與單獨 launch O1–O3 | 15/15 PASS |
| `openarm_description/test/run_full_test.sh` | develop 既有的完整測試（clean build、lint baseline、fake_smoke 含 O6） | FULL TEST PASSED；openarm_bringup flake8 743 / pep257 20，與 baseline 相同 |

```bash
source ~/work/openarm/openarm_jact_ws/install/local_setup.bash
python3 scripts/data_collection/test/check_rules.py
python3 scripts/data_collection/test/check_bringup_fake.py          # --rviz 可以看畫面
```

## 相關檔案

- `scripts/data_collection/joint_actions_aggregator.py`：aggregator
- `openarm_bringup/launch/joint_actions.launch.py`：單獨 launch / 被 bringup include
- `openarm_bringup/launch/openarm_o6_bimanual.launch.py`：`with_joint_actions`、`aggregator_*`
- `openarm_bringup/CMakeLists.txt`：安裝 aggregator 腳本
- `scripts/data_collection/fake_arm_hand_feed.py`：fake hardware 專用的假上游
- `scripts/data_collection/README.md`：使用說明
