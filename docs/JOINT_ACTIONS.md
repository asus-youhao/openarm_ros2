# `/joint_actions`：由 bringup 帶起、記錄 FPC 命令的事件驅動 aggregator

## 一句話

`/joint_actions`（26 維 `JointState`，給資料收集 / 模仿學習當 action）改由
`openarm_o6_bimanual.launch.py` 帶起的**單一** aggregator 發布，資料來源是四個
ForwardPositionController（FPC）**真正收到的命令**，發布規則與
`phu_controller_ros2` 的 `gallop_joint_actions_aggregator` 相同（事件驅動 + latch +
stale 閘門），另外修掉兩個在 phu / `072aa65` 也存在的問題。

## 為什麼改（develop `b170a61` 的舊版）

| # | 舊版行為 | 問題 |
|:-:|---|---|
| 1 | 50 Hz `create_timer` 定頻發布 | 在 IK 與 collector 之間多一層重取樣，相位漂移 |
| 2 | 上游斷了繼續重發最後的值 | collector 量的是 `/joint_actions` 的年齡，死掉的上游看起來像即時資料 |
| 3 | `/joint_states` 找不到的關節填 0.0 | 捏造的常數欄位混進資料集 |
| 4 | 手臂命令 `list(msg.position)`，不看 `name` | 發布端順序一變就靜默錯位 |
| 5 | 維度隨 `--hand_config` 變（14 / 20 / 26） | 下游依名稱查表，少一個名稱整筆丟掉，整場錄不到也不報錯 |
| 6 | 由 IK 程序在 process 內起（兩個入口各自起） | 跟 IK 共用 executor / GIL；IK 沒跑就沒有 `/joint_actions`；以後若 bringup 也起一個就變兩個 publisher |
| 7 | 手臂來源是 IK 的旁路 topic `/{arm}_arm_ik_commands` | `arm_o6_j`（JTC 模式、FPC 根本沒 load）下 IK 照發旁路，手臂沒動卻錄到 action |
| 8 | `header.stamp` = 發布當下 | 看不出樣本真正多舊 |

## 新機制

### 誰發、什麼時候發

```
openarm_o6_bimanual.launch.py (with_joint_actions:=true，預設)
  └─ joint_actions.launch.py ─ ExecuteProcess ─ joint_actions_aggregator.py   ← /joint_actions 唯一的 publisher

  /left_forward_position_controller/commands        ┐ 手臂：到達即觸發發布
  /right_forward_position_controller/commands       ┘
  /left_hand_forward_position_controller/commands   ┐ 手掌：只更新最新值，不觸發
  /right_hand_forward_position_controller/commands  ┘
  /joint_states                                       只用來補「從沒收到命令」的 limb
```

- **來源 = controller 的輸入**：四個來源都是 `/<FPC>/commands`（沒有名稱的
  `Float64MultiArray`）。每個 slot 是哪個關節，向 controller 讀它自己的 `joints` 參數
  （`GetParameters`，每秒重試直到回應），**不寫死**；讀到之前該來源的命令一律拒收。
  FPC 沒 load（`robot_controller:=joint_trajectory_controller`，也就是 `arm_o6_j`）就
  永遠讀不到 → 什麼都不錄，修掉舊版第 7 點。
- **誰送命令都一樣**：placo IK、其他 IK、VR teleop、腳本都行；aggregator 不依賴 placo IK。
  IK 的兩個入口（`placo_ik_online_profiler_bimanual.py`、`..._ws_mesh.py`）不再起
  aggregator；`/{arm}_arm_ik_commands` 仍照發，只剩監看用途。
- **事件驅動**：所有 active 的手臂都有新命令、而且是同一個 IK 步（見下面「兩臂配對」）
  才發一筆，頻率與相位直接繼承上游。

### 什麼時候用 `/joint_states` 補、什麼時候不發（與 phu 相同）

| 情況 | 欄位的值 | 擋不擋發布 |
|---|---|---|
| ① config 沒訂閱這個 limb | `/joint_states` 實測值 | 不擋 |
| ② 有訂閱、但從開機到現在沒收過命令 | `/joint_states` 實測值（持續 warn） | 不擋 |
| ③ 收過命令、後來斷了（stale） | — | **整組不發**，恢復後印 `resumed … after Xms` |
| ④ 兩臂都沒收過命令 | — | **不發** |
| ①② 但 `/joint_states` 也找不到 | — | **這筆不發**，不填 0.0 |

③④ 不拿 `/joint_states` 頂替：collector 的 observation 本來就是 `/joint_states`，頂替
進去的 action 等於 observation，會訓練出「輸出 = 目前狀態」的原地不動 policy；而且
FPC 斷線時其實還 hold 著最後一筆命令，`/joint_states`（真機有重力下垂）連值都不對。
停發才能讓 collector 把該段標成非即時。

輸出永遠是 `ALL_JOINT_NAMES` 的 26 個名稱、固定順序（左臂 7、右臂 7、左手 6、右手 6，
必須跟 vla_control `get_joint_order("openarm_linkerhand_o6")` 一致）。
`header.stamp` = 參與這筆的命令中**最早到達**的那個的到達時間（FPC 命令沒有 header）。

### 跟 phu 不同的地方

1. **手臂、手掌都讀 controller 的 `joints`**。phu 只有頭 / 腰這樣做，手掌順序仍寫死
   （#86 手動對齊）。openarm 四個來源都是 FPC，統一走同一條路。
2. **兩臂依到達時間配對同一個 IK 步（`PAIR_FRACTION`）**。phu / `072aa65` 只看「所有
   active 的手臂都 advanced」，會**永久錯開一拍**：IK 每步先發左臂再發右臂，第一步的
   左臂到達時右臂還沒 latch，左臂單獨發了一筆；之後每一筆都是「左臂第 k 步 + 右臂第
   k-1 步」（兩臂差 20 ms）。修法：某臂待發的命令若比最新那筆早到超過
   `PAIR_FRACTION × 週期`，就屬於上一步 → 清掉它的 advanced、等它下一筆。
   同步 IK（同一步兩臂間隔 < 1 ms，錯拍時差 20 ms）配到同一步；排程抖動偶爾讓單筆配錯，
   下一步就自己恢復，不會再鎖死。兩臂各自一條執行緒、沒有共同步時，絕大多數配到時間
   最近的那一筆（抖動時偶爾差一步），頻率不降。週期估計沿用 phu 的 EMA（≥ 1 s 的間隔
   不計），所以暫停 0.2–1 s 後視窗會暫時變寬，約 10 筆內恢復。修正前 `check_rules.py`
   R3b 150/150 筆錯開，修正後 0–1 筆。
3. **Ctrl-C 乾淨結束**：`ros2 launch` 底下按 Ctrl-C 會收到兩次 SIGINT（終端機 + launch
   轉送），第二次打在 `destroy_node()` 中間會丟 traceback、launch 報 `exit code -2`。
   清理階段改為忽略後續 SIGINT（phu 與舊版都有這個問題）。
4. **手掌頻率種子 30 Hz**（phu 100 Hz）：`pico_o6_hand_trigger.py` 預設 30 Hz。種子比實際
   頻率低只會讓第一個 stale 視窗變寬，比實際高會讓頭幾筆就誤判 stale。

## 常數與調整

| 常數 / 參數 | 值 | 作用 |
|---|---|---|
| `STALE_PERIODS` | 3.0 | 超過 3 個實測週期沒新命令算 stale |
| `STALE_FLOOR_SEC` | 0.03 s | stale 門檻下限，避免高頻來源被 jitter 誤觸發 |
| EMA α | 0.15 | 每個來源的到達週期估計（≥ 1 s 的間隔不計，與 phu 相同） |
| `PAIR_FRACTION` | 0.5 | 兩臂配對視窗 = 0.5 × 較短的手臂週期 |
| `HAND_SEED_RATE` | 30 Hz | 手掌週期估計的初值 |
| `--publish_rate` | 50 Hz | 手臂週期估計的初值（**不是**發布頻率） |
| `--stale_timeout` | 0 = 自動 | 給 > 0 就所有來源固定用這個 stale 門檻（秒） |
| `--{left,right}_{arm,hand}_controller` | 四個 FPC 名稱 | 換 controller 名稱時用 |

50 Hz 手臂的 stale 門檻約 60 ms、配對視窗約 10 ms；30 Hz 手掌的 stale 門檻約 100 ms。

## 真機上怎麼開、怎麼關

```bash
# 錄資料：FPC 模式（arm_o6_p），bringup 會一起帶起 aggregator
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py robot_controller:=forward_position_controller ...

# 不要 aggregator
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py ... with_joint_actions:=false
# 之後要開再單獨起（同一時間只能有一個）
ros2 launch openarm_bringup joint_actions.launch.py

# 只訂閱右手 / 不訂閱手掌（輸出仍是 26 維）
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py ... aggregator_hand_config:=o6_right
```

`aggregator_arm_config` 預設 `both` 就涵蓋單臂 teleop：只有送過命令的手臂會擋發布，
另一臂用 `/joint_states` 補。

## 已知限制 / 要注意

- **home / unfold 的 ramp 也會被錄**：IK 的 homing 也是送 FPC 命令，action 記的是
  controller 收到的東西。錄 episode 時請在 homing 之後才開始。
- **第一筆可能只有一臂**：第一個左臂命令到達時右臂還沒 latch，會以單臂語意發一筆
  （右臂欄位 = `/joint_states`），之後就兩臂同步。
- **FPC 有 load 但 inactive** 時（例如切回 JTC 之後），往 FPC topic 送的命令仍會被錄，
  aggregator 不檢查 controller 狀態（與 phu 相同）。
- **手掌上游必須定頻**：已 latch 的手掌停發會讓整組停發（規則③）。只在變化時才發的
  上游會一直觸發 stale。
- **不支援 `arm_prefix` 命名空間**：topic / 參數服務用的是絕對名稱。
- aggregator 腳本在 repo 的 `scripts/`（不屬於任何 package），由 `openarm_bringup` 的
  `CMakeLists.txt` 安裝到 `share/openarm_bringup/scripts/data_collection/`，有沒有
  `--symlink-install` 都找得到。

## 驗證

| 項目 | 內容 | 結果 |
|---|---|---|
| `scripts/data_collection/test/check_rules.py` | 只跑 aggregator + 假 controller（`joints` 故意打亂）：R1–R14，含 ①②③④、拒收、單臂、config、兩臂配對、獨立相位 | 18/18 PASS（FastDDS、CycloneDDS 各一次以上） |
| `scripts/data_collection/test/check_bringup_fake.py` | 真的 bringup（`use_fake_hardware:=true`）：FPC 模式 F1–F6、JTC 模式 J1–J4、`with_joint_actions:=false` 與單獨 launch O1–O3 | 13/13 PASS（FastDDS、CycloneDDS） |
| `openarm_description/test/run_full_test.sh` | develop 既有的完整測試（clean build 不加 symlink、lint baseline、fake_smoke 含 O6） | FULL TEST PASSED；openarm_bringup flake8 743 / pep257 20，與 baseline 相同 |
| 真的 placo IK（手動） | fake FPC bringup + `placo_ik_online_profiler_bimanual.py` + 假 `/ee_delta` | `/joint_actions` publisher 始終 1 個；479 筆樣本全部對得上 IK 步，兩臂 IK stamp 差最大 0.36 ms |

兩支檢查都只在 fake hardware、各自未使用的 `ROS_DOMAIN_ID`、localhost discovery 下跑：

```bash
source /opt/ros/jazzy/setup.bash && source <ws>/install/setup.bash
python3 scripts/data_collection/test/check_rules.py
python3 scripts/data_collection/test/check_bringup_fake.py
```

## 相關檔案

- `scripts/data_collection/joint_actions_aggregator.py`：aggregator
- `openarm_bringup/launch/joint_actions.launch.py`：單獨 launch / 被 bringup include
- `openarm_bringup/launch/openarm_o6_bimanual.launch.py`：`with_joint_actions`、`aggregator_*`
- `openarm_bringup/CMakeLists.txt`：安裝 aggregator 腳本
- `scripts/data_collection/fake_arm_hand_feed.py`：fake hardware 專用的假上游
- `scripts/data_collection/README.md`：使用說明
