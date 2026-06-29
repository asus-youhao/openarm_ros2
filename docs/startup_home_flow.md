# Placo IK — 啟動同步 & 回 Home 流程圖（2026-05-29）

**對象**：[`placo_ik/ik_node/placo_ik_node.py`](../ik_node/placo_ik_node.py) + [`placo_ik/ik_node/placo_ik_online_profiler_ws_mesh.py`](../ik_node/placo_ik_online_profiler_ws_mesh.py)

本檔說明三件事的程式流程，重點在**沒有 TF / 沒有 joint_states 時各分支的行為**：
1. 程式啟動總流程（home-first → run → `_startup_sync`）
2. `_startup_sync()` 的四層優先級（首步跳動的修法）
3. 執行中按 `h` 回 home：Forward 模式 vs Traj 模式的差異

> Mermaid 圖在 GitHub / VSCode（裝 Markdown Preview Mermaid Support）可直接渲染。

---

## 1. 啟動總流程

profiler 先（可選）回 home，再進 `run()`，`run()` 第一件事是 `_startup_sync()`。

```mermaid
flowchart TD
    A["python3 ./placo_ik_online_profiler_ws_mesh.py"] --> B{"args.home_first?<br/>(預設 True)"}
    B -- "是" --> C{"控制器模式?"}
    C -- "Forward (預設)" --> D["send_home_fwd()<br/>L712"]
    C -- "Traj (--use-traj)" --> E["send_home_confirmed()<br/>L641"]
    D --> F["node.run()  L1262"]
    E --> F
    B -- "否 (--no-home-first)" --> F
    F --> G["_startup_sync()  L862<br/>(同步 _pose / _last_joints)"]
    G --> H["啟動背景服務<br/>TfPoller / CsvWriter"]
    H --> I["主迴圈 while rclpy.ok()<br/>讀 tracker → IK → publish"]
```

---

## 2. `_startup_sync()` 四層優先級（L862-917）

這是「首步跳動」bug 的修法核心。**Step 3 的 FK 回推**是舊版沒有的關鍵新增。

```mermaid
flowchart TD
    S["_startup_sync() 開始"] --> J1["Step 1: 重試 3 秒等 /joint_states<br/>L878-886"]
    J1 --> J2{"7 軸都到齊?"}
    J2 -- "是" --> J3["real_joints = 實機關節<br/>_last_joints = real_joints<br/>L889-893"]
    J2 -- "否 (3s 逾時)" --> J4["⚠ 用 home_joints 當 seed<br/>real_joints = None<br/>L894-897"]

    J3 --> T1["Step 2: TF lookup (1.0s)<br/>L899-900"]
    J4 --> T1
    T1 --> T2{"TF 取得?"}
    T2 -- "是" --> T3["✅ _pose = TF<br/>return  L901-904"]

    T2 -- "否" --> F1{"Step 3: real_joints 存在?<br/>L906-907"}
    F1 -- "是" --> F2["fk_pose = session.fk(real_joints)<br/>L908"]
    F2 --> F3{"FK 成功?"}
    F3 -- "是" --> F4["✅ _pose = FK 回推位置<br/>(貼合實機) return  L909-913"]
    F3 -- "否" --> H1

    F1 -- "否" --> H1["Step 4: ⚠ _pose = home_pose<br/>internal ≠ real arm<br/>L915-917"]

    style T3 fill:#1b5e20,color:#fff
    style F4 fill:#1b5e20,color:#fff
    style H1 fill:#b71c1c,color:#fff
    style J4 fill:#e65100,color:#fff
```

**讀法**：
- 綠色 = 健康路徑（`_pose` 貼合實機）。
- 橘色 = seed 退回 home_joints（首步可能 guard-clamp 爬行，但不暴衝）。
- 紅色 = 最壞情況：TF 與關節都拿不到 → `_pose=home_pose`，內部認知與實機不符（console 明確警告）。
- **只要 joint_states 在，即使 TF 全掛，Step 3 也能用 FK 把 `_pose` 救回實機位置** → 這就是首步跳動被修掉的原因。

---

## 3. 按 `h` 回 home — Forward 模式（裸指令預設）

`send_home_fwd()` 從「目前關節位置」插值 ramp 到 home。**沒有 joint_states 就算不出起點 → 安全中止,手臂不動。**

```mermaid
flowchart TD
    H0["按 h → request_home<br/>_handle_kbd_events L925-931"] --> H1["send_home_fwd()  L712"]
    H1 --> P0{"_fwd_pub 存在?"}
    P0 -- "否" --> FB["fallback → send_home_confirmed()<br/>L728-730"]
    P0 -- "是" --> R1["讀 /joint_states  L737-738"]
    R1 --> R2{"7 軸到齊?"}
    R2 -- "否" --> W["⚠ 等 2 秒再讀一次<br/>L740-743"]
    W --> R3{"仍到齊?"}
    R3 -- "否" --> AB["✗ aborting<br/>return False<br/>不發指令 / 手臂不動 / 狀態不變<br/>L744-746"]
    R3 -- "是" --> RAMP
    R2 -- "是" --> RAMP["start = 目前關節<br/>cos ease-in-out ramp 到 home<br/>≤30°/s 發 Float64MultiArray<br/>L748-782"]
    RAMP --> V1["sleep 0.5 → TF 驗證  L787-788"]
    V1 --> V2{"TF 取得 且 dist≤2.5cm?"}
    V2 -- "是" --> OK["✅ _pose=TF, _last_joints=實機<br/>L791-801"]
    V2 -- "否" --> NV["⚠ 無法驗證,但仍設<br/>_last_joints = home  L808-810"]

    style AB fill:#e65100,color:#fff
    style OK fill:#1b5e20,color:#fff
    style NV fill:#b71c1c,color:#fff
```

**沒 TF + 沒 joint_states 的結果**：走到橘色 `aborting` → **完全不發指令、手臂留在原地、`_pose`/`_last_joints` 不變**（安全失敗）。注意 `_handle_kbd_events` 不檢查回傳值,只會印 `✗ still no joint states — aborting`,迴圈繼續。

---

## 4. 按 `h` 回 home — Traj 模式（`--use-traj`）

`send_home_confirmed()` **不讀 joint_states 算指令**,直接發絕對 home 軌跡(open-loop),再等 TF 確認。

```mermaid
flowchart TD
    C0["按 h → request_home<br/>(--use-traj)"] --> C1["send_home_confirmed()  L641"]
    C1 --> LOOP["for attempt in 1..max_tries"]
    LOOP --> PUB["發絕對 home JointTrajectory<br/>(不看現況)  L656-662"]
    PUB --> WAIT["輪詢 motion_sec 秒<br/>每 0.3s 做 _get_tf(0.3)  L668-677"]
    WAIT --> CK{"TF 取得 且 dist≤2.5cm?"}
    CK -- "是" --> OK["✅ arrived<br/>_pose=TF, _last_joints=實機<br/>L680-692"]
    CK -- "否 (TF=None 一直 continue)" --> RETRY{"還有 attempt?"}
    RETRY -- "是" --> LOOP
    RETRY -- "否 (全逾時)" --> FB["⚠ could not confirm<br/>_pose = home_pose (假設到位)<br/>_last_joints = home_joints<br/>L703-709"]

    style OK fill:#1b5e20,color:#fff
    style FB fill:#b71c1c,color:#fff
```

**沒 TF + 沒 joint_states 的結果**：
- 軌跡**照發**(arm 由 JointTrajectoryController 從自己的內部現況插值執行 → 動作本身安全)。
- TF 永遠確認不到 → 重試耗盡 → 紅色 fallback：**`_pose` 被假設成 home_pose**。
- 若控制堆疊真的掛了(joint_states 也斷),軌跡可能根本沒執行,但內部仍記成 home → **內部認知與實機不符**,下一步 teleop 可能有落差(jump_guard 10° clamp 夾成爬行,不暴衝)。

---

## 5. 兩模式對照表（沒 TF + 沒 joint_states 按 h）

| 模式 | 行為 | 手臂 | 內部狀態 | 安全性 |
|---|---|---|---|---|
| **Forward（預設）** | 等 2s → 仍無 → 中止 | 不動 | 不變 | ✅ 安全失敗 |
| **Traj（`--use-traj`）** | 盲發絕對 home 軌跡 → 確認失敗 → 假設到位 | 由控制器插值(若堆疊在) | `_pose=home_pose`(可能 ≠ 實機) | ⚠ 動作安全但認知可能失準 |

**結論**：裸指令是 Forward 模式 → 最壞只是「按 h 沒反應 + 印 aborting + 手臂不動」,不會出事。要小心的是 `--use-traj` 模式,沒 TF 時會 open-loop 回 home 並假設到位,該模式下建議確保 TF/joint_states 正常再按 h。

---

## 相關文件
- 啟動同步/首步跳動已修的脈絡：[`vr_teleop_issues_20260529.md`](./vr_teleop_issues_20260529.md)（舊 issue #1 已不在清單）
- jump guard clamp 爬行行為：同上 #5
