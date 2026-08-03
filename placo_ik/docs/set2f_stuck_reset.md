# Set 2-F：Stuck-at-Boundary 自動 ref reset

> **實作檔案**：`ik_node/placo_ik_node.py`、`ik_node/ws_boundary.py`
> **相關狀態**：`self._stuck_reset_ms`, `self._stuck_reset_done_eps`
> **CLI flag**：`--stuck-reset-ms`（預設 500，0 停用）
> **設計目標**：使用者推 tracker 越過邊界後再拉回時，**不要 return snap**。

---

## 目錄

1. [問題：「Return Snap」失敗模式](#1-問題return-snap-失敗模式)
2. [為什麼 Set2-A SoftClamp 不夠](#2-為什麼-set2-a-softclamp-不夠)
3. [Set2-F 設計](#3-set2-f-設計)
4. [實作細節](#4-實作細節)
5. [One-shot per episode 邏輯](#5-one-shot-per-episode-邏輯)
6. [與既有 `gap_sec` reset 的關係](#6-與既有-gap_sec-reset-的關係)
7. [調參與測試](#7-調參與測試)
8. [副作用與緩解](#8-副作用與緩解)
9. [與其他改進的協作](#9-與其他改進的協作)

---

## 1. 問題：「Return Snap」失敗模式

### 場景

使用者用 VR tracker 控制 EE：

1. 使用者把 tracker **持續往外推** → 累積 δ 越過 workspace 邊界
2. SoftClamp 把 target 限制在 boundary 上 → 手臂貼牆
3. 但 **tracker 端 δ 仍在累積**（每幀 0.5cm × 持續 2 秒 = 10cm 越界量）
4. 使用者把 tracker 拉回 → δ 反向，但**累積量太大**：例如 -8cm
5. `target = ref + Σδ` 突然從邊界跳回 ref - 8cm（**孔內遠處**）
6. IK 突然 success → 手臂瞬跳 5–10cm

### 為什麼會累積

`_ee_delta_ref_xyz` 在以下情況才會重設：
- **`gap_sec > 0.35s`** — tracker 斷線
- **`is_new` flag** — 首次啟動、kbd reset_ref
- **`send_home_confirmed()`** — home 後

「**持續推著沒斷線**」這條路徑不會 reset → δ 一直加 → return snap。

### 失敗模式分類

| 模式 | 觸發 | 解決機制 |
|---|---|---|
| A. Lock-in（凍結）| OOR + success-freeze | Set2-D ✅ |
| **B. Return snap** | **OOR + 持續推 + 拉回** | **Set2-F（本文件）** |
| C. Singular wobble | 接近奇異點 | Set2-C Adaptive DLS ✅ |
| D. Hidden ori conflict | xyz 可達但 ori 不行 | Set1-G _W_ORI tune ✅ |

---

## 2. 為什麼 Set2-A SoftClamp 不夠

SoftClamp 只處理「**當下的 target**」（投影 / damping 到 reachable 區），它**不知道 tracker 端正在累積多少越界量**。

範例：

```
tracker_pos     = [0, 0, 0] → [0, 0.10m, 0] → [0, 0.20m, 0] → [0, -0.05m, 0]
δ each frame    =  0           +10cm           +10cm           -25cm
ref + Σδ        =  ref         ref+10cm        ref+20cm        ref-5cm
SoftClamp target=  ref         ref+5cm(邊界)   ref+5cm(邊界)   ref-5cm(沒-snap)
IK 實際 EE      =  ref         ref+5cm         ref+5cm         **ref-5cm  ← 跳 10cm**
```

最後一步從 boundary (ref+5cm) 跳到 ref-5cm，**單步 10cm**。即使 vel_limit 也擋不住所有跳變（10cm @ 100Hz 需要 1m/s 末端速度，超出大部分 hardware 限制）。

> SoftClamp 治「**位置上的硬截**」，Set2-F 治「**ref 累積上的漂移**」。兩者正交。

---

## 3. Set2-F 設計

### 核心想法

監聽 `BoundaryState.since_outside_ms`：當「**連續 outside 超過閾值**」時，**強制下一幀 ref reset 到 TF**。

```
outside 計時 = since_outside_ms
   ↑
   │
   │ ──── stuck_reset_ms (500) ────────  trigger reset
   │
   │      [outside episode]
   │ ┌────────────────────────┐
   │ │                        │
   ┌─┘                        └──→  inside again → clear flag
   └─────────────────────────────── 時間
```

### Reset 觸發後

```python
self._ee_delta_ref_xyz = None
self._ee_delta_ref_q   = None
self._ee_delta_last_t  = None
```

下一幀 `is_new = True` → 從 TF 讀 EE 真實位置 → ref 重新 anchor → 累積 δ **歸零**。

使用者拉回 tracker 時，δ 從 0 開始算 → 不會跳。

### One-shot 約束

不能每幀都 reset（會永遠在邊界），需要：
- 一個 episode 只 reset **一次**
- `outside → inside` 過渡時清旗標
- 下次再越界又達閾值才會再 reset

---

## 4. 實作細節

### CLI flag

```python
p.add_argument("--stuck-reset-ms", type=float, default=500.0,
               dest="stuck_reset_ms",
               help="Set2-F: ms outside boundary before forcing ee_delta ref reset. ...")
```

預設 **500 ms**；`0` 停用。

### 狀態變數（`__init__`）

```python
self._stuck_reset_ms       = float(getattr(args, "stuck_reset_ms", 500.0))
self._stuck_reset_done_eps = False    # one-shot per outside episode
```

### 主迴圈邏輯（緊接 BoundaryMonitor publish 後）

```python
if self._stuck_reset_ms > 0:
    if _bs.outside and _bs.since_outside_ms > self._stuck_reset_ms:
        if not self._stuck_reset_done_eps:
            self._ee_delta_ref_xyz = None
            self._ee_delta_ref_q   = None
            self._ee_delta_last_t  = None
            self._stuck_reset_done_eps = True
            print(f"\n  [Set2-F] stuck outside {_bs.since_outside_ms:.0f}ms"
                  f" — ref reset (re-anchor to TF next frame)")
    elif not _bs.outside:
        self._stuck_reset_done_eps = False
```

要點：
- **此幀的 target 已用 stale ref 算完**，不打斷
- 把 ref 設 None → **下一幀**才 re-anchor
- 不在 outside 狀態 → 清旗標（下次 outside 可再觸發）

### Banner 顯示

```
  stuck-rst: 500ms outside → auto ref reset (Set2-F)
```

---

## 5. One-shot per episode 邏輯

### State machine

```
    [INSIDE, done=False]
            │
       outside=True
       since_ms ≤ 500
            ▼
    [OUTSIDE, done=False, waiting]
            │
       since_ms > 500
            ▼
    [OUTSIDE, done=True] ← reset 已觸發，等待回到 inside
            │
       outside=False
            ▼
    [INSIDE, done=False]  (循環)
```

### 為什麼不每幀 reset

如果每幀 outside + over-threshold 都 reset，則：
- ref 永遠是 TF 當前值
- 使用者推 tracker → δ 累積 1 幀就被清掉
- 結果手臂**完全跟不動**（每幀 base_xyz 跳到 TF）

One-shot 保證 reset 只在「**新越界 episode 達閾值**」時發生。

### 邊界 case

| 場景 | 行為 |
|---|---|
| `since_outside_ms < 500` | 不 reset；繼續正常 clamp |
| `since_outside_ms > 500` 第一次 | 觸發 reset，set `done=True` |
| `since_outside_ms > 500` 第 N+1 次 | done=True → 跳過 |
| 短暫 inside 一幀又 outside | done=False 重置 → 下次達 500 又會 reset |
| 沿著邊界 slide（outside=False 大部分時間）| done 不會 set，無 reset |

---

## 6. 與既有 `gap_sec` reset 的關係

| 機制 | 觸發條件 | 用途 |
|---|---|---|
| **`_ee_delta_gap_sec = 0.35s`** | tracker 訊號斷 > 350ms | 防止「斷線重連 ↔ 累積 δ 漂走」 |
| **Set2-F `_stuck_reset_ms = 500ms`** | tracker 持續送但 outside > 500ms | 防止「持續推但 hardware 卡邊界 ↔ return snap」 |

**互補不衝突**：兩個都會把 ref 設 None；只是觸發條件不同。

- 沒 Set2-F → 推著不斷線 = 完全沒有 reset 機會
- 沒 gap_sec → 短暫斷線 = ref 隨意漂

---

## 7. 調參與測試

### `--stuck-reset-ms` 推薦範圍

| 值 | 觸發感 | 適用 |
|---|---|---|
| 0 | 停用 | A/B 對照、原行為 |
| 200 | 很敏感 | 短促觸碰邊界即重設；可能在 slide-along-edge 場景過度觸發 |
| **500**（預設）| 平衡 | 一般 VR teleop |
| 1000 | 寬鬆 | 容忍短時間邊界探索，僅在明顯卡住才重設 |
| 2000 | 幾乎停用 | 只在「使用者已經推牆 2 秒」才介入 |

### 測試流程

```bash
# 啟用 Set2-F（預設）
python3 placo_ik_online_profiler_ws_mesh.py --arm right

# 停用 Set2-F（看 return snap）
python3 placo_ik_online_profiler_ws_mesh.py --arm right --stuck-reset-ms 0

# 更敏感
python3 placo_ik_online_profiler_ws_mesh.py --arm right --stuck-reset-ms 200
```

### 觀察指標

1. **Console**：`[Set2-F] stuck outside Nms — ref reset` 訊息
2. **CSV**：`dx`, `dy`, `dz` 在 reset 時應該瞬間 → 0 附近（base 更新到 TF，δ 重新計算）
3. **`/{arm}/boundary_state`** topic：`since_outside_ms` 計時 + reset 後重置

---

## 8. 副作用與緩解

### 副作用

1. **Sliding along edge 場景** — 使用者沿邊界滑動，可能在邊界 in/out toggle。如果 outside 累計 > 500ms 會被誤觸發
2. **Reset 後仍在邊界** — 如果使用者沒拉回，ref reset 後 base_xyz 變 TF (邊界)，下一幀 δ 累積又會推出去 → 再次達 500ms → 再 reset。最終效果像「失效」但不會崩
3. **Reset 那一幀有微小 EE 跳動** — 因為 `_pose` 從 stale ref 換到 TF；如果 Set2-D 的 `_pose` 漂移防護生效（用 `r["ee_xyz"]` 更新），跳動 < 5mm

### 緩解

| 風險 | 緩解 |
|---|---|
| Sliding 誤觸發 | 加大 `--stuck-reset-ms 1000`；或日後加 `dx · normal > 0` 判斷（只在持續往外推時 reset） |
| Reset 後仍在邊界 | 不是 bug，是使用者拒絕拉回；Set2-A SoftClamp 已避免硬卡 |
| Reset 那幀跳動 | Set2-D 的 `_pose` 漂移防護應接住；output LPF 再多平滑一層 |

### 更聰明的觸發條件（未來改進）

目前只看 `since_outside_ms > threshold`。更細的判斷可以是：

```python
# 只在「持續往外推」時 reset，sliding 不觸發
dx_normal = np.dot(dx_arm_arr, boundary_normal)
if _bs.outside and since_ms > threshold and dx_normal > 0:
    # 使用者真的在嘗試推出去 → reset
```

需要 SoftClamp 回傳 boundary normal（目前內部有算，沒 expose）。

---

## 9. 與其他改進的協作

### Pipeline 位置

```
  ┌──────────────────────────────────────────────┐
  │ Main loop                                    │
  │                                              │
  │  tracker δ → base + δ → raw_xyz              │
  │       ↓                                      │
  │  [SoftClamp] ─→ new_xyz, _bs                 │
  │       ↓               ↓                      │
  │       │       [BoundaryMonitor.publish()]    │
  │       │               ↓                      │
  │       │       ┌─────────────────────────┐    │
  │       │       │ Set2-F:                 │    │
  │       │       │  if _bs.outside &&      │    │
  │       │       │     since_ms > 500 &&   │    │
  │       │       │     !done_eps:          │    │
  │       │       │   ref_xyz = None ←──────┼──── 影響下一幀
  │       │       │   ref_q   = None        │    │
  │       │       │   done_eps = True       │    │
  │       │       └─────────────────────────┘    │
  │       ↓                                      │
  │  IK solve(new_xyz, ...)                      │
  │       ↓                                      │
  │  [Set2-D] always publish                     │
  └──────────────────────────────────────────────┘
```

### 與 Set2 其他項目的關係

| 項目 | 關係 |
|---|---|
| **Set2-A SoftClamp** | 提供 `_bs.outside` 與 `since_outside_ms` 信號（Set2-F 的輸入）|
| **Set2-D Continuous Approach** | 失敗時 `_pose` 用 `r["ee_xyz"]` 防漂；Set2-F reset 後 fallback 到 TF 直接 |
| **Set2-E BoundaryMonitor** | publish 邊界狀態給 VR app；可結合 Set2-F 的 reset 事件做震動提示 |

### 與 ee_delta 機制的整合

```
ee_delta_cb（每收到 PoseStamped）
     ↓
self._pending = msg
     ↓
─── Main loop ───
     │
     ↓
[檢查 gap_sec > 0.35]── True ──→ is_new=True ──→ ref ← TF
     │ False                        ↑
     ↓                              │
[檢查 ref is None]── True ──────────┘
     │ False
     ↓
base = ref
target = ref + Σδ
     ↓
[SoftClamp]
     ↓
[Set2-F check]── 連續 outside > 500ms ──→ ref ← None ← 下一幀 is_new=True
     ↓
[IK solve]
     ↓
[Set2-D publish]
```

---

## 附錄 A：CLI 範例

```bash
# 預設（Set2-D + Set2-F 都開）
python3 placo_ik_online_profiler_ws_mesh.py --arm right

# 停用 Set2-F（保留 Set2-D）
python3 placo_ik_online_profiler_ws_mesh.py --arm right --stuck-reset-ms 0

# 完整回退（停用 Set2-F + 開 success-gate）
python3 placo_ik_online_profiler_ws_mesh.py --arm right --stuck-reset-ms 0 --success-gate

# 更敏感版本（連續 200ms 就 reset）
python3 placo_ik_online_profiler_ws_mesh.py --arm right --stuck-reset-ms 200

# A/B 對照測試
python3 placo_ik_online_profiler_ws_mesh.py --arm right --dry-run --stuck-reset-ms 0    # off
python3 placo_ik_online_profiler_ws_mesh.py --arm right --dry-run --stuck-reset-ms 500  # on
```

---

## 附錄 B：監聽 reset 事件（VR app 端建議）

```python
# 訂閱 /{arm}/boundary_state
def on_boundary_state(msg: String):
    state = json.loads(msg.data)
    if state["since_outside_ms"] > 400 and state["since_outside_ms"] < 600:
        # 即將觸發 stuck reset → 給使用者預警震動
        hmd.haptic_pulse(strength=0.8, duration_ms=100)
    if state["since_outside_ms"] == 0 and prev_since > 500:
        # Reset 剛發生 → EE 轉淡綠色
        hmd.set_ee_color(GREEN_FLASH)
    prev_since = state["since_outside_ms"]
```
