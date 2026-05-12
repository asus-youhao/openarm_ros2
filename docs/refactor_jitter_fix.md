# IK Jitter 修正 & 重構報告

## 問題分析 — 卡頓根因

| 優先 | 根因 | 平均延遲 | 說明 |
|------|------|---------|------|
| 🔴 Fix-1 | `_get_tf(0.3)` in 熱迴圈 | 0–300 ms | tracker `is_new` 觸發時阻塞主執行緒 |
| 🔴 Fix-2 | velocity limits 關閉 + dt 錯誤 | 物理卡頓 | 單步可輸出任意大的 joint delta |
| 🔴 Fix-3 | `csv_fh.flush()` 每步 | 1–10 ms | kernel I/O 調度帶入不確定延遲 |

---

## Fix-1 — TfPoller：非同步 TF 查詢

### 問題

```python
# 舊：每次 is_new 觸發就阻塞 0–300 ms
if is_new:
    ref = self._get_tf(0.3)   # ← lookup_transform timeout=0.3s
```

`is_new` 在以下情況觸發：
- tracker 短暫斷線後恢復
- 按 `r` 鍵重置 session ref
- 啟動後第一幀
- gap > 0.35 s（USB lag、CPU 負載等）

### 解法：`TfPoller` (background daemon thread)

```
TfPoller thread  (50 ms poll interval)
  └── lookup_transform(timeout=50ms)  → 更新 self._pose cache

Main hot-loop
  └── self._tf_poller.get()           → 立即返回 cache，0 阻塞
```

```python
class TfPoller:
    def get(self) -> Optional[Tuple]:
        """Instant non-blocking cache read."""
        with self._lock:
            return self._pose

    def _run(self):
        while True:
            try:
                t = self._buf.lookup_transform(..., timeout=Duration(0.05))
                with self._lock:
                    self._pose = (tr.x, tr.y, tr.z, ro.x, ro.y, ro.z, ro.w)
            except Exception:
                pass
            time.sleep(0.025)   # poll ~40 Hz
```

### 時序對比

| 情境 | 修前 | 修後 |
|------|------|------|
| TF 正常 | 0–5 ms（運氣好） | < 0.01 ms（cache）|
| TF 暫時無效 | **300 ms 阻塞** | < 0.01 ms（None returned）|
| 20 Hz budget | 50 ms | 50 ms（全部留給 IK）|

---

## Fix-2 — Velocity Limits + dt 修正

### 問題

```python
# 舊程式碼
solver.enable_velocity_limits(False)   # ← 無速度限制！
solver.dt = 0.010                      # ← 10 ms，但控制週期 = 50 ms (20 Hz)
```

**效果：** solver 以 10 ms temporal window 計算 joint delta，
但實際每步間隔 50 ms。關閉 velocity limits 後，
solver 可在單步輸出 5× 過大的 joint velocity → 機器手臂急衝。

### 解法

```python
# PlacoSession.__init__
self._dt         = 1.0 / rate_hz   # 自動匹配實際控制週期
self._vel_limits = vel_limits       # 預設 True

# solve_step()
solver.enable_velocity_limits(self._vel_limits)   # ← 啟用
solver.dt = self._dt                              # ← 正確週期
```

CLI 傳入機制：
```bash
# PlacoSession 建立時自動計算 dt
self._placo_session = PlacoSession(
    rate_hz    = self._rate_hz,      # dt = 1/rate_hz
    vel_limits = not args.no_vel_limits,
)
```

| 設定 | 舊值 | 新值 (20 Hz) |
|------|------|-------------|
| `dt` | 10 ms (hardcoded) | 50 ms (= 1/rate_hz) |
| `vel_limits` | False | True |

---

## Fix-3 — AsyncCsvWriter：非阻塞 CSV I/O

### 問題

```python
# 每步都觸發 kernel flush syscall
self._csv_writer.writerow(...)
self._csv_fh.flush()   # ← 1–10 ms 不確定延遲
```

### 解法：`AsyncCsvWriter` (background queue-drain thread)

```
Hot-loop                CSV writer thread
  │                         │
  ├─ put(row)  ──queue──►   ├─ writerow()
  │  (non-blocking)         └─ flush() when queue empties
  │
  └─ 繼續 IK，不等 flush
```

```python
class AsyncCsvWriter:
    def put(self, row_dict):
        try:
            self._q.put_nowait([...])   # O(1)，永不阻塞
        except queue.Full:
            pass   # 極端情況 drop（queue cap = 2000 rows）

    def _run(self):
        while True:
            row = self._q.get()
            if row is None: break
            self._writer.writerow(row)
            if self._q.empty():
                self._fh.flush()   # 只在 queue drain 時才 flush
```

| 指標 | 修前 | 修後 |
|------|------|------|
| hot-loop CSV overhead | 1–10 ms/step | ~0 ms（enqueue O(1)）|
| 資料安全 | 每步 flush | queue drain 時 flush（< 100 ms lag） |

---

## 重構架構

### Before（1 file, ~1100 lines）

```
placo_ik_online_profiler_ws_mesh.py   (1100 lines)
  ├── _KbdController    → 已提取至 kbd_controller.py
  ├── PlacoSession
  ├── PlacoOnlineProfiler
  └── _parse_args / main
```

### After（5 files，各職責獨立）

```
kbd_controller.py                     (240 lines)
  └── KbdController — termios 鍵盤 daemon，零 ROS 依賴

placo_ik_session.py                   (186 lines)  ← NEW
  └── PlacoSession — 純 Python IK session，零 ROS 依賴
      ├── Fix-2: vel_limits=True, dt=1/rate_hz
      └── early-exit T_ee cached（減少 1 次多餘 FK）

placo_ik_node.py                      (813 lines)  ← NEW
  ├── TfPoller     — Fix-1: 背景 TF 快取執行緒
  ├── AsyncCsvWriter — Fix-3: 背景 CSV 佇列執行緒
  └── PlacoOnlineProfiler — ROS2 Node（主控制邏輯）

placo_ik_online_profiler_ws_mesh.py   (151 lines)  ← 改為純 entry point
  ├── _parse_args()
  └── main()
```

### 執行緒架構

```
Main thread
  └── IK hot-loop  (20–50 Hz)
        ├── pop keyboard delta  (O(1) lock-free read)
        ├── tf_poller.get()     (O(1) cache read)     [Fix-1]
        ├── placo_session.solve_step()
        ├── _publish()
        └── csv_writer.put()    (O(1) queue enqueue)  [Fix-3]

Thread: rclpy spin
  └── _js_cb, _ee_delta_cb  (ROS 回呼)

Thread: kbd_controller
  └── termios raw-mode stdin 讀取

Thread: TfPoller                   [Fix-1]
  └── lookup_transform 每 50 ms

Thread: AsyncCsvWriter             [Fix-3]
  └── queue drain → writerow → flush
```

---

## 使用方式（不變）

```bash
# Tracker mode（預設）
conda run -n pico_teleop_py python3 placo_ik_online_profiler_ws_mesh.py --arm right

# 直接啟動 Keyboard mode
conda run -n pico_teleop_py python3 placo_ik_online_profiler_ws_mesh.py --arm right --keyboard

# 停用 velocity limits（不建議）
... --no-vel-limits

# dry-run（IK 計算但不送軌跡）
... --dry-run
```

### 鍵盤快捷鍵

| 鍵 | 功能 |
|----|------|
| `t` | 切換 TRACKER ↔ KEYBOARD mode |
| `w/s` | ±Y（前/後） |
| `a/d` | ±X（左/右） |
| `q/e` | ±Z（上/下） |
| `i/k` | pitch ±（旋轉 Y） |
| `j/l` | yaw ±（旋轉 Z） |
| `u/o` | roll ±（旋轉 X） |
| `1-9` | scale preset 0.10×–5.00× |
| `+/-` | scale ×1.25 / ×0.80 |
| `p` | pause / resume |
| `r` | reset session reference |
| `h` | send home |
| Ctrl-C | quit |

---

## 後續優化建議

| 優先 | 項目 | 說明 |
|------|------|------|
| 🟡 中 | `ee_delta_gap_sec` 調至 1.0 s | 0.35 s 過於敏感，易觸發 ref 跳躍 |
| 🟡 中 | adaptive regularisation | reg 從固定 `1e-5` → 基於 min singular value 動態調整 |
| 🟢 低 | 更多 `_MAX_ITER` 控制 | 近目標用少 iter，遠目標允許更多 |
