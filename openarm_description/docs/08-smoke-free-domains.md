# 08 fake 啟動測試只用沒人在用的 ROS_DOMAIN_ID

## 問題

`test/fake_smoke.py` 每個 case 用的 domain 是 `60 + pid % 30` 起算，連續三個。如果這時有人正在用其中一個 domain
跑 fake（例如 README 教的 `run_fake_rviz.sh` 用 77），測試的 spawner 和 rclpy probe 就會打到別人的
`/controller_manager`、收到別人的 `/robot_description`：測試結果不可信，還會去動別人的 controller。
3 個 case 時，base 落在 75、76、77 就會碰到 77，機率 3/30 = 10%。

實際發生的情境：使用者在 domain 77 開著 O6 fake 看 RViz，這時跑了一次完整測試（這次剛好沒撞到）。

## 根因

domain 用 pid 取餘數決定，完全沒有看系統上有沒有人在用。

## 改動

`test/fake_smoke.py`：

- `domains_in_use()`：讀所有 `/proc/<pid>/environ`，收集每個 process 設定的 `ROS_DOMAIN_ID`（讀不到的略過）。
- `free_domains(count, 60, 101)`：在 60–101 之間挑 `count` 個沒人用的；不夠就停止並說明。
- 輸出第一行列出被跳過的 domain，每個 case 的標題印出實際用的 domain。

## 行為變化

- 測試永遠不會跟正在跑的 fake（或任何設了 `ROS_DOMAIN_ID` 的 ROS 程式）共用 domain。
- 限制：沒設 `ROS_DOMAIN_ID` 的 process 用的是 domain 0，不在 60–101 範圍內，本來就不會撞到。
- u22 / u24：只讀 `/proc`，兩個版本相同。

## 測試

修正前：上面算出的 10% 碰撞機率；沒有任何避開的機制。

自我檢查（這時使用者的 fake 在 77、另有幾個殘留的 ros2 daemon）：

```
in use: [8, 77, 94, 95, 96, 97]
picked: [60, 61, 62]
```

完整測試（使用者的 O6 fake 仍在 domain 77 執行）：

```
fake_smoke: ROS_DOMAIN_IDs in use by other processes skipped: [77]
[bimanual_default] ...      (domain 60, ...)   S1 3 active（gripper SKIP）/ S2 16 joints at 100 Hz / S3 14 joints at target
[bimanual_o6] ...           (domain 61, ...)   S1 5 active / S2 36 joints at 502 Hz / S3 26 joints at target / S4 10 mimic follow
[real_o6_parse_probe] ...   (domain 62, ...)   H1 PASS
FULL TEST PASSED
```

測試前後使用者 domain 77 的 `ros2_control_node` 都還在，沒有被影響。

## 更正（06、07 的測試紀錄）

06、07 寫「結束後沒有殘留的 ros2 daemon」是錯的。當時用 `pgrep _ros2_daemon` 檢查，但 daemon 的 process 是
`python3 -c ... --name ros2-daemon`，那個 pattern 比對不到。實際上照 README 用 `ros2 topic` / `ros2 control`
指令驗證時，每個 domain 都留下了一個 ros2 daemon（94、95、96、97，以及在 77 上驗證時留下的一個）。
這是 ros2 CLI 的正常行為，不影響功能，已全部停掉。`fake_smoke.py` 只用 rclpy，不會啟動 daemon。

## 後續

無。
