# Go-Home Service（std_srvs/Trigger）— 設計與改動說明

> 日期：2026-06-12
> 把鍵盤 `h` 的回 home 功能曝露成 ROS2 service，雙臂新版與舊版（單臂 node）
> 都已加入。

## Service 介面

| 版本 | Service 名稱 | 行為 |
|------|--------------|------|
| 雙臂單 QP 版（`placo_ik_node_bimanual.py`） | `/bimanual/go_home` | 兩臂同時 ramp 回 home |
| 舊版（`placo_ik_node.py`，每臂一個 node） | `/right/go_home`、`/left/go_home` | 該臂回 home |

型別都是 `std_srvs/srv/Trigger`，呼叫方式：

```bash
ros2 service call /bimanual/go_home std_srvs/srv/Trigger {}
ros2 service call /right/go_home    std_srvs/srv/Trigger {}
```

回覆：`success` = home 是否到位（TF 驗證）；`message` = 說明文字。
homing 進行中重複呼叫會立即回 `success=False, "homing already in progress"`。
等待逾時 20 秒（hot loop 沒在跑時會 timeout）。

---

## 核心設計：service callback 不直接執行 homing

**問題**：`send_home_fwd*()` 是阻塞的（ramp 約 3.5 秒）。service callback 跑在
executor 的 spin thread，與 IK hot loop 是不同執行緒——若 callback 直接執行
homing，期間 hot loop 仍持續發 IK 命令，兩股命令同時餵
ForwardCommandController，手臂會抽動（安全問題）。

鍵盤 `h` 沒這個問題，因為 `_handle_kbd_events()` 是在 hot loop **裡面**呼叫，
homing 阻塞期間整個 IK 迴圈停住。

**做法**：service 與鍵盤共用同一條執行路徑——

```
service callback（executor thread）          hot loop（main/IK thread）
─────────────────────────────────           ──────────────────────────
_home_request.set()                  ──→    每 tick 開頭檢查 _home_request
_home_done.wait(timeout=20s)                  → _execute_home_request():
                                                  send_home_fwd*()（阻塞，IK 暫停）
                                                  清 pending + 強制 reanchor
                                                  填 _home_result
        ←──────────────────────────────────────  _home_done.set()
讀 _home_result 填 response
```

鍵盤 `h` 改成也只是 `_home_request.set()`，執行統一走
`_execute_home_request()`，沒有第二條 homing 路徑。

## 兩個防護

1. **homing 後強制 reanchor**：清掉 homing 期間累積的 tracker `pending`，並設
   `reanchor_pending`。原本靠「gap > 0.8s 自動 reanchor」保護，但 service 可在
   任意時機被外部觸發，顯式重置比依賴時序可靠。
2. **拒絕重入**：`_home_request` 已 set 時直接回失敗，不排隊。

## 連帶修改：舊版單臂入口換 MultiThreadedExecutor

`placo_ik_online_profiler_ws_mesh.py` 的單臂路徑原本用 `rclpy.spin()`
（SingleThreadedExecutor）。service callback 的阻塞等待會把單執行緒 executor
卡死 → `/joint_states`、TF listener 全停 → homing 讀不到關節、TF 驗證必失敗。
已改為 `MultiThreadedExecutor(num_threads=4)`，與 `--arm both` 路徑一致。

## 改動檔案清單

| 檔案 | 改動 |
|------|------|
| `ik_node/placo_ik_node_bimanual.py` | `_home_request/_home_done/_home_result` + `/bimanual/go_home` service + `_home_srv_cb` / `_execute_home_request`；鍵盤 `h` 改走同一路徑；banner 加 service 行 |
| `ik_node/placo_ik_node.py` | 同上（per-arm `/{arm}/go_home`；`--use-traj` 時走 `send_home_confirmed`，否則 `send_home_fwd`） |
| `ik_node/placo_ik_online_profiler_ws_mesh.py` | 單臂路徑 `rclpy.spin` → `MultiThreadedExecutor(4)` |

## 測試順序

1. `--dry-run` 啟動 → `ros2 service call ... go_home ...` → console 出現 ramp
   log、回覆 `success: True`。
2. teleop 中（tracker 持續發訊）呼叫 → homing 期間手臂不抽動；結束後第一筆
   tracker 訊息印 `NEW ref`（reanchor），手臂不跳。
3. homing 進行中重複呼叫 → 回 `already in progress`。
4. 舊版 `--arm both`：分別呼叫 `/right/go_home`、`/left/go_home`，
   可同時呼叫（4 threads 夠用：2 個阻塞 callback + 訂閱）。

## 之後可擴充（v2，未做）

自訂 srv 參數化：`arm: string`（雙臂版選單臂回 home）、`motion_sec: float`、
`do_unfold: bool`（回 home 前先跑 unfold 安全展開）。
