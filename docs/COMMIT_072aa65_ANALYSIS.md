# Commit `072aa65` 分析：事件驅動的 `/joint_actions` 與 stale 閘門

| 項目 | 內容 |
|---|---|
| Commit | `072aa65dc4d7468e64592e3b3a5f76b848491a88` |
| 標題 | `feat(data_collection): event-driven /joint_actions with a staleness gate` |
| 作者 / 日期 | Ming Li，2026-08-31 |
| 變更檔案 | 3 個檔案，+590 / −243 |

| 檔案 | 變更 | 內容 |
|---|---|---|
| `scripts/data_collection/joint_actions_aggregator.py` | +~350 / −~240（幾乎重寫） | 聚合邏輯從「固定頻率 timer」改成「事件驅動 + stale 閘門」 |
| `openarm_bringup/launch/openarm_o6_bimanual.launch.py` | +96 | launch 預設一併啟動 aggregator，新增 5 個 `aggregator_*` 參數 |
| `scripts/data_collection/README.md` | +145 / −~70 | 文件跟著新行為更新，修正原本跑不起來的 `ros2 run` 指令 |

---

## 1. 一句話總結

`joint_actions_aggregator` 原本用 **50 Hz timer** 不斷重發「各來源最後收到的值」，
現在改成 **收到手臂 IK 命令時才發布**，而且一旦某個已經在跑的來源斷掉，就 **停止發布**，
不再重發凍住的舊值。另外 `/joint_actions` 的維度 **固定為 26**，launch 也 **預設帶起它**。

---

## 2. 為什麼要改（舊版的問題）

| # | 舊版行為 | 造成的問題 |
|---|---|---|
| 1 | 用 `create_timer(1/50)` 固定 50 Hz 發布 | 等於在 IK 節點和 collector（自己的 `control_hz`）之間**多了一層重取樣**；兩個時鐘各跑各的，相位會漂移 |
| 2 | 上游斷了還是繼續以 50 Hz 重發最後的值 | collector 量的是「最後一則 `/joint_actions` 的年齡」，重發舊值讓它**把死掉的上游誤判成即時資料** |
| 3 | 輸出維度隨 `--hand_config` 改變（14 / 20 / 26） | 下游 `data_collector` 依名稱查表，少一個名稱就丟掉整筆樣本 → **整場錄不到東西，而且不報錯** |
| 4 | 手臂命令直接 `list(msg.position)`，不看 `msg.name`、不檢查長度 | 發布端改順序或長度不對時，向量會**靜默錯位** |
| 5 | `/joint_states` 裡找不到的關節填 `0.0` | **捏造的常數欄位**混進資料集後，和真實資料無法區分 |
| 6 | `header.stamp` = 發布當下的時間 | 看不出樣本真正有多舊 |
| 7 | README 寫 `ros2 run openarm_ros2 joint_actions_aggregator.py` | `scripts/` 底下沒有 `package.xml`，**這個指令從來沒辦法執行** |

---

## 3. 新增 / 改變了什麼

### 3.1 事件驅動發布（取代 timer）

- 發布點移到手臂命令的 callback（`_arm_command`）裡。
- 條件：**所有 active 的手臂都送來了新命令**（`advanced == True`）才發布一次，
  發布後把所有來源的 `advanced` 清掉。
- 效果：兩臂會 rendezvous 在同一個 IK solve step，`/joint_actions` 直接繼承 IK 節點的頻率與相位。
- 手掌（約 100 Hz）**不觸發發布**，只在發布時取當下最新值。
  刻意這樣設計：由手掌觸發會讓 `/joint_actions` 變成 100 Hz，其中一半是手臂值的重複幀。
- 原本的 timer 只剩一個 1 Hz 的 `_report_health`，只負責印診斷訊息，不參與發布。

### 3.2 Latch 機制與 limb 狀態

新增 `_LimbSource` 類別，記錄每個 limb 的最新值、到達時間、實測週期、是否 latch 過。
limb **第一次收到命令就 latch 成 active**，之後整場維持。

| limb 狀態 | 欄位的值 | 會不會擋住發布 |
|---|---|---|
| config 沒選（不訂閱） | `/joint_states` 實測值 | 不擋 |
| 有訂閱但**從沒收到過** | `/joint_states` 實測值，並持續 warn | 不擋 |
| 已 latch、**新鮮** | 命令值 | — |
| 已 latch、**stale** | — | **擋，整組不發** |
| 兩臂都沒 latch 過 | — | **擋，整組不發** |

設計邏輯：
- **從沒收到過** = 上游沒開，limb 物理上是靜止的，用實測值填是誠實的。
- **收到過然後沒聲音** = 上游故障或暫停，此時**停止發布**，讓 collector 把該 episode 標記成非即時。
- 刻意**不用** `/joint_states` 頂替故障的來源：collector 的 observation 也是 `/joint_states`，
  頂替進去的 action 會變成 observation 的複製，訓練出「輸出 = 當前狀態」的原地不動 policy。

### 3.3 自動推導的 stale 門檻

```python
STALE_PERIODS   = 3.0    # 3 個週期沒收到就算 stale
STALE_FLOOR_SEC = 0.03   # 下限 30 ms，避免 100 Hz 手掌被正常 jitter 誤觸發
```

- 每個來源用 EMA（α = 0.15）量測自己的到達週期：`period_ema = 0.85·old + 0.15·dt`，
  超過 1 秒的間隔不計入。
- 門檻 = `max(3 × period_ema, 30 ms)`，所以 50 Hz 手臂約 60 ms、100 Hz 手掌約 30 ms。
- 初始值：手臂用 `--publish_rate`（預設 50 Hz），手掌固定用 100 Hz。
- 新增 `--stale_timeout` 參數，可以覆寫成所有來源共用的固定值（秒）。

### 3.4 固定 26 維輸出

```python
ALL_JOINT_NAMES = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + O6_LEFT_JOINTS + O6_RIGHT_JOINTS  # 7+7+6+6
```

- `_build_joint_names()` 被移除，`--hand_config` / `--arm_config` **只決定訂閱哪些 topic**，不影響維度。
- 順序註明必須和 `vla_control` 的 `get_joint_order("openarm_linkerhand_o6")` 一致。

### 3.5 訊息驗證

| 來源 | 驗證方式 | 不合格時 |
|---|---|---|
| 手臂 `JointState` 有 `name` | 依名稱對應，缺任一關節就拒收 | 丟棄 + warn（5 秒節流），累計 `rejects` |
| 手臂 `JointState` 沒 `name` | 退回位置對應，但長度必須等於 7 | 同上 |
| 手掌 `Float64MultiArray` | 長度必須等於 6 | 同上 |

### 3.6 不再捏造 0.0

`_fill_from_state_locked()`：要用實測值填的 limb 如果關節不在 `/joint_states` 裡，
回傳 `None`，**整筆樣本不發布**（log 會出現 `withholding`），而不是填 0.0。

### 3.7 `header.stamp` 語意改變

- 舊版：發布當下的時間。
- 新版：**最舊那個 latch 來源**的時間。手臂用自己訊息的 `header.stamp`
  （為 0 時改用收到的時間）；手掌沒有 header，用收到的時間。
- 消費端讀 `header.stamp` 就能知道這筆樣本實際有多舊。

### 3.8 診斷 log

| 情況 | Log |
|---|---|
| 有訂閱但從沒收到 | `WARN no command since startup on [...] — treating as inactive and filling from /joint_states`（每 5 秒，**刻意持續出現**） |
| 還沒有任何手臂命令 | `INFO waiting for the first arm command before publishing` |
| 已 latch 的來源 stale | `WARN stale: right_hand=140ms — /joint_actions paused until it recovers` |
| 從暫停恢復 | `INFO resumed /joint_actions after XXXms waiting on [...]` |

原本「每 500 則印一次」的心跳 log 被移除。

### 3.9 其他小改動

- `main()` 的例外處理加上 `ExternalShutdownException`，`rclpy.shutdown()` 前先檢查 `rclpy.ok()`。

---

## 4. Launch 檔的變更（`openarm_o6_bimanual.launch.py`）

新增 launch 參數：

| 參數 | 預設值 | 說明 |
|---|---|---|
| `launch_joint_actions_aggregator` | `true` | 是否一併啟動 aggregator |
| `aggregator_script` | `<REPO_ROOT>/scripts/data_collection/joint_actions_aggregator.py` | 腳本路徑 |
| `aggregator_arm_config` | `both` | 對應 `--arm_config` |
| `aggregator_hand_config` | `o6_both` | 對應 `--hand_config` |
| `aggregator_publish_rate` | `50.0` | 對應 `--publish_rate`（只用來設定初始 stale 窗口） |

實作重點：
- aggregator 不屬於任何 ROS package，所以不用 `Node`，而是用 `ExecuteProcess` 以 `sys.executable` 直接執行腳本。
- `REPO_ROOT` 是用 `os.path.realpath(__file__)` 往上三層算出來的，**依賴 `colcon build --symlink-install`**
  讓 launch 檔 symlink 回原始碼目錄。
- 找不到腳本就丟 `FileNotFoundError`，並提示改用 `aggregator_script:=`。
- 用 `TimerAction` 延遲 `2 × LAUNCH_DELAY_SECONDS = 4 s` 才啟動，等 controller 先起來。

使用範例：
```bash
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py aggregator_hand_config:=o6_right
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py launch_joint_actions_aggregator:=false
```

---

## 5. README 的變更

- 所有 `ros2 run openarm_ros2 ...` 改成 `python3 scripts/...`。
- `--hand_config` 表格的「總關節數」全部改為 26。
- 新增「發布條件」章節（latch 狀態表、stale 門檻、為什麼停止發布、為什麼不用實測值頂替）。
- 工作流程改成：`placo_ik_main_bimanual.py` → `ros2 launch ...`（自動帶 aggregator）→ 驗證。
- Troubleshooting 把「手掌關節都是 0.0」換成三個新情境：中途停止發布、一直 warn "no command since startup"、完全不發布（withholding）。
- 「性能考量」改寫成事件驅動、手掌 100 Hz 的取樣方式、訊息驗證、時間戳語意。

---

## 6. 前後對照

| 面向 | 舊版 | 新版 |
|---|---|---|
| 發布觸發 | 50 Hz timer | 手臂命令到達（兩臂都有新值時） |
| 發布頻率 | 固定 `--publish_rate` | = IK 頻率 |
| 上游斷線 | 繼續重發舊值 | 停止發布 |
| 未啟動的來源 | `/joint_states`，找不到填 0.0 | `/joint_states`，找不到就整筆不發 |
| 輸出維度 | 14 / 20 / 26（依 config） | 永遠 26 |
| 手臂訊息對應 | 依位置，不檢查 | 依名稱；沒名稱時依位置 + 檢查長度 |
| `header.stamp` | 發布當下 | 最舊來源的時間 |
| `--publish_rate` | 決定發布頻率 | 只用來設定初始 stale 窗口 |
| 新增參數 | — | `--stale_timeout` |
| 啟動方式 | 文件寫 `ros2 run`（實際無法執行） | `python3 ...` 或 launch 自動帶起 |

---

## 7. 需要注意的地方

以下是讀 code 時發現、值得留意的點（**未在實機上驗證**）：

1. **沒用 `--symlink-install` 時 launch 會直接失敗。**
   `REPO_ROOT` 靠 `realpath(__file__)` 回到原始碼目錄。如果是一般 `colcon build`（複製安裝），
   算出來的路徑會落在 `install/openarm_bringup/share/` 底下，找不到腳本就丟 `FileNotFoundError`，
   而 `launch_joint_actions_aggregator` 預設是 `true`，**整個 launch 都會起不來**。
   這時需要傳 `aggregator_script:=` 或 `launch_joint_actions_aggregator:=false`。

2. **`/joint_actions` 可能出現兩個 publisher。**
   `placo_ik_main_bimanual.py` 和 `placo_ik_main.py --arm both` 都會在 process 內起一個
   `JointActionsAggregator`，launch 預設也會再起一個。README 有寫「擇一即可」，
   但照 README 的工作流程（步驟 1 + 步驟 2）用預設值執行，就會同時有兩個 publisher，
   collector 會收到重複的樣本。

3. **「兩臂在同一個 IK solve step」只對單一 QP 的雙臂節點成立。**
   `placo_ik_node_bimanual.py` 在同一個迴圈裡依序發布兩臂命令，rendezvous 會對齊。
   但 `placo_ik_main.py --arm both` 是兩個獨立節點、兩條執行緒各跑各的，
   此時 rendezvous 只是「等兩邊都有新值」，不保證來自同一個 step。

4. **手掌命令停送會讓整個 `/joint_actions` 停下。**
   手掌一旦 latch，stale 窗口約 30 ms。如果手掌 teleop 只在數值改變時才發布、或暫時停送，
   手臂資料也會一起被擋掉。這是設計行為，但要確認實際的手掌上游是固定頻率發布。
   （本 repo 裡找不到正式的手掌 teleop 發布端，這點無法從 code 確認。）

5. **手臂斷線時不會印 "resumed" log。**
   `_pause_t` 只在 stale 檢查那一段被設定。手臂斷線時，另一臂的 callback 會先在
   「不是所有 active 手臂都 `advanced`」那一步就 return，不會走到 stale 檢查，
   所以恢復時不會出現 `resumed ... after XXXms`。1 Hz 的 `stale:` warn 仍然會正常出現，
   只是少了恢復的那一行。只影響診斷，不影響資料。

6. **EMA 會跟著變慢的來源一起放寬門檻。**
   如果來源是逐漸變慢（不是突然斷掉），`period_ema` 會跟著變大，stale 窗口也會跟著放寬，
   不一定會觸發暫停。需要嚴格門檻時可以用 `--stale_timeout` 固定。

7. **行為改變，可能影響既有的使用方式。**
   - 依賴 `/joint_actions` 固定 50 Hz 的消費端，現在拿到的是 IK 頻率。
   - 依賴 `header.stamp` = 發布時間的消費端（例如 `ros2 topic delay`），數值意義變了。
   - README 的 Q3（「用 `--publish_rate 100.0` 讓 aggregator 更快」）已經不成立，
     因為這個參數不再決定發布頻率，但該段落沒有跟著更新。
