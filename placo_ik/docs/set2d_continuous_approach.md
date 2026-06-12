# Set 2-D：Continuous Approach — 移除 Success-Freeze Gate

> **實作檔案**：`ik_node/placo_ik_node.py`
> **CLI flag**：`--success-gate`（restore legacy 行為）
> **設計目標**：teleop 撞 workspace 邊界 / 奇異點時，**手臂持續逼近但走不到**，
> 而非 **整個凍住**。

---

## 目錄

1. [問題：為什麼要拿掉這個 gate](#1-問題為什麼要拿掉這個-gate)
2. [原本的「freeze」邏輯](#2-原本的freeze邏輯)
3. [為什麼移除是安全的](#3-為什麼移除是安全的)
4. [Set 2-D 設計](#4-set-2-d-設計)
5. [`_pose` 漂移防護（關鍵細節）](#5-_pose-漂移防護關鍵細節)
6. [實作 diff](#6-實作-diff)
7. [行為對照](#7-行為對照)
8. [失敗模式 + 回退](#8-失敗模式--回退)
9. [量化指標 + CSV 觀察](#9-量化指標--csv-觀察)
10. [CLI 使用](#10-cli-使用)

---

## 1. 問題：為什麼要拿掉這個 gate

之前的行為（[placo_ik_node.py:637 之前的版本](../ik_node/placo_ik_node.py)）：

```python
if r["success"]:                      # pos_err < POS_RELAX (10 mm)
    self._publish(joints_out)         # 才送軌跡
# else: pass — 完全不送指令，arm 停在原地
```

效果：你用 VR tracker 控手臂，當 `target_xyz` 落在：

- **WorkspaceMesh 外部**（即使有 SoftClamp 也可能殘餘 outside）
- **奇異點附近**（IK 無法在 max_iter 內收斂到 10 mm）
- **orientation + position 衝突**（位置可達但姿態不行）

→ `pos_err > 10mm` → `success = 0` → **整支手臂凍住不動**。

但你**沒有任何反饋**告訴你「我撞牆了」（雖然 Set2-E `BoundaryMonitor` 有 topic，但人感知不一定即時）。
人類預期是：「我推，arm 該往那邊**盡量走**；走不到我自己會收手」 — 而不是「我推，arm 不理我」。

這就是 **continuous approach** 的意義：飽和取代凍結。

---

## 2. 原本的「freeze」邏輯

完整流程（移除前）：

```
tracker δ → target_xyz (clamped) → IK solve → r["joints"], pos_err
                                                  │
                                                  ▼
                                         pos_err < 10mm ?
                                          ┌────┴────┐
                                          │ YES     │ NO
                                          ▼         ▼
                                      update    skip publish
                                      _pose +   skip latency
                                      publish   _pose 不變
                                      latency   ← 手臂凍住
```

設計初衷（[vr_realtime_ik_analysis.md:177-184](vr_realtime_ik_analysis.md#L177-L184)）：
> OOR 時 success=0 ... 上層 pass — 不送指令，**保持靜止**（safe fallback）。

**用意是「安全」** —— 怕送一個 IK 沒收斂的關節值會讓馬達狂跳。

---

## 3. 為什麼移除是安全的

關鍵：**Placo 回傳的 `r["joints"]` 永遠是 feasible 解，就算 `success=0`。**

理由：

1. **`enable_joint_limits(True)`** ([session.py:146](../ik_node/placo_ik_session.py#L146))
   → QP 軟約束 + 後處理 `clip(joints, _lo, _hi)` ([session.py:194](../ik_node/placo_ik_session.py#L194))
   → joint 絕不超過 URDF / human config 範圍

2. **`enable_velocity_limits(True)`** ([session.py:147](../ik_node/placo_ik_session.py#L147)) + **wrist cap 4.0 rad/s**
   → QP 硬不等式約束 `|q̇| ≤ v_max`
   → 單步 Δq 絕不超過 `v_max × dt`
   → **這就是 "saturation"** —— 達不到目標時，IK 自動在約束 polytope 內走最近的方向

3. **Adaptive DLS λ**（接近奇異點時自動增大）
   → 奇異點不會解出無窮大 Δq

4. **Always-update seed** ([node.py:634-635](../ik_node/placo_ik_node.py#L634-L635)) 已經啟用
   → `r["joints"]` 已經是「離 seed 不遠的可行解」（warm start + vel_limit）

綜上：**送 `r["joints"]` 出去 = 送一個 "在約束內、向目標靠近、單步 Δq 受限" 的關節指令**。
不會跳、不會超限、不會出軌道。**就算 pos_err = 30mm，joints 仍然合法**。

---

## 4. Set 2-D 設計

### 行為改變

```
之前：success → publish ; fail → freeze
之後：always publish ; success vs fail 只影響 _pose 怎麼更新 + console warning
```

### 三個小改動

1. **無條件 publish**（除了 `--dry-run` / `--success-gate` 兩個 opt-out 路徑）
2. **`_pose` 漂移防護**：success 時 = target；fail 時 = `r["ee_xyz"]`（solver 實際 EE）
3. **連續失敗計數 + 警告**：每 N 次連續 fail 印一行警告（不致洗版）

### 為什麼還留 `--success-gate` flag

純為 A/B 測試和安全 fallback：
- 第一次跑新行為時，user 可以 `--success-gate` 切回舊行為對照
- 若硬體上發生意料外的失控，立刻 `--success-gate` 就回到 known-safe state

---

## 5. `_pose` 漂移防護（關鍵細節）

`self._pose` 在程式裡有兩個用途：

| 用途 | 在哪 | 失敗時若用 target 會怎樣 |
|---|---|---|
| Keyboard mode：下一個 δ 的起點 | [node.py:554](../ik_node/placo_ik_node.py#L554) `base_xyz = tuple(self._pose[:3])` | 累積誤差：keyboard δ 加在「想去但沒去到的點」上 |
| Delta mode fallback：當 `_ee_delta_ref_xyz` 為 None 時 | [node.py:583-584](../ik_node/placo_ik_node.py#L583) | 較不嚴重（gap > 0.35s 會 resync TF）|
| 顯示 / log | console + CSV `x/y/z` 欄 | 顯示「夢想位置」而非實際位置 |

**Solution：fail 時用 solver 算出的實際 EE 位置 (`r["ee_xyz"]`) 更新 `_pose`，**
這樣下個 keyboard δ 加在「實際位置」上，沒有累積漂移。

### 為什麼 orientation 保留舊值（不用 new_q）

Session 不回傳 `r["ee_R"]`，只有 `r["ee_xyz"]`。若 fail 時 `_pose[3:7] = new_q`：
- new_q = target rotation（可能不可達）
- 下一個 dq 又乘上 target rotation → 累積誤差

**保險做法**：fail 時只更新 `_pose[0:3]`（position），`_pose[3:7]`（orientation）**完全不動**。
等到 IK success 那一步，再一次性更新整個 _pose。

```python
if r["success"]:
    self._pose = [new_x, new_y, new_z, *new_q]   # 整個更新
else:
    ee = r["ee_xyz"]
    self._pose[0:3] = [ee[0], ee[1], ee[2]]      # 只更新 position
    # _pose[3:7] (orientation) intentionally kept
```

---

## 6. 實作 diff

### `placo_ik_node.py::__init__`

```python
# Set2-D state
self._success_gate    = bool(getattr(args, "success_gate", False))
self._fail_streak     = 0
self._fail_warn_every = 20
```

### `placo_ik_node.py::run()` main loop

```python
# Always update seed (existing — unchanged)
with self._joints_lock:
    self._last_joints = r["joints"]

# Set2-D: continuous approach
if r["success"]:
    self._pose = [new_x, new_y, new_z,
                  new_q[0], new_q[1], new_q[2], new_q[3]]
    self._fail_streak = 0
else:
    ee = r["ee_xyz"]
    self._pose[0] = float(ee[0])
    self._pose[1] = float(ee[1])
    self._pose[2] = float(ee[2])
    # _pose[3:7] (orientation) intentionally kept at last value
    self._fail_streak += 1
    if self._fail_streak % self._fail_warn_every == 0:
        print(f"\n  [Set2-D] IK pos_err={r['pos_err_mm']:.1f}mm "
              f"streak={self._fail_streak} — publishing partial solve")

publish_ok = r["success"] or not self._success_gate
joints_out = self._filter_joints(r["joints"])
if publish_ok and not self.args.dry_run:
    self._publish(joints_out)
self._latency_pub.publish(Float32(data=float(ik_ms)))
```

### CLI flag

```python
p.add_argument("--success-gate", action="store_true", dest="success_gate",
               help="Restore legacy 'freeze on IK failure' behaviour. ...")
```

### Banner

```
publish  : always (continuous approach, Set2-D)
publish  : success-gate (legacy freeze)            ← 加 --success-gate 時
```

---

## 7. 行為對照

### 情境：tracker 推到 workspace 外 5cm

| 階段 | 舊行為 (success-gate) | 新行為 (continuous approach) |
|---|---|---|
| t=0 (在 ws 內) | publish ✓ | publish ✓ |
| t=1 (撞邊界, ws_mesh 軟 clamp 收一半 → pos_err 8mm) | publish ✓ | publish ✓ |
| t=2 (繼續推外, pos_err 15mm > 10mm) | **freeze** ✗ | publish 部分解，wrist 受 vel_cap 飽和 → arm 緩慢往邊界滑 ✓ |
| t=3..10 (持續推外) | **freeze** ✗ | 持續逼近，每 20 步印警告 |
| t=11 (使用者收手往回) | success → publish ✓ 但**從 freeze 那一刻的 _pose 起算 δ** → 有可能 snap | success → publish ✓，`_pose` 已隨實際 EE 漂回 → 平滑接續 |

### 情境：奇異點（手臂完全伸直）

| 階段 | 舊行為 | 新行為 |
|---|---|---|
| 接近奇異 | Adaptive DLS λ 升高 | Adaptive DLS λ 升高 |
| 進入奇異 | IK pos_err > 10mm → freeze | 持續 publish，wrist vel_cap 限制 Δq，arm 在奇異點附近平滑 wobble |
| 離開奇異 | freeze 後直接「彈」回 success 解 | 已連續逼近，無 snap |

---

## 8. 失敗模式 + 回退

### 可能新增的失敗模式

| 失敗 | 觸發 | 處理 |
|---|---|---|
| **持續 fail 時 pos_err 累積但 _pose 跟著漂** | 連續 100+ steps 都 fail，`_pose` 已離 target 很遠 | console 每 20 步警告；可以後續加 Set2-F (stuck → auto resync ref) |
| **fail 時 `r["joints"]` 真的不該用** | 罕見：max_iter 太少 + 大 seed jump 時，可能 joints 跑到非 home 邊界 | post-clip + vel_limit 仍會 cap；CLI `--success-gate` 可即時回退 |
| **orientation freeze 違反使用者意圖** | user 想旋轉但 fail；新代碼 `_pose[3:7]` 不更新 → 下個 δq 仍乘舊 q | 等下一次 success 時自然校正；極端情況可手動 `r` reset_ref |

### 回退路徑

```bash
# 一鍵回到舊行為（freeze on fail）
python3 placo_ik_online_profiler_ws_mesh.py --arm right --success-gate
```

---

## 9. 量化指標 + CSV 觀察

CSV 已有的欄位足夠評估：

| 欄位 | 用途 |
|---|---|
| `success` | 0/1 — 全程 success rate 比舊行為應該不變（IK 本身沒變） |
| `pos_err_mm` | fail 段的 pos_err 分布 — 看 saturation 多嚴重 |
| `track_err_mm` | EE actual vs target — 看「實際走了多少」 |
| `x, y, z` | `_pose` xyz — 新行為下這個值在 fail 時會跟著 `r["ee_xyz"]` |
| `sigma_min`, `lambda_dls` | 奇異點旁邊的 fail 應該對應 σ_min 低 + λ 大 |

### 對照測試

```bash
# Run A: legacy
python3 placo_ik_online_profiler_ws_mesh.py --arm right --success-gate \
    --csv /tmp/legacy.csv

# Run B: continuous (Set2-D)
python3 placo_ik_online_profiler_ws_mesh.py --arm right \
    --csv /tmp/set2d.csv

# 比較 success rate / pos_err / track_err 分布
python3 -c "
import csv, numpy as np
for name, path in [('legacy', '/tmp/legacy.csv'), ('set2d', '/tmp/set2d.csv')]:
    rows = list(csv.DictReader(open(path)))
    sr = np.mean([int(r['success']) for r in rows]) * 100
    pe = np.percentile([float(r['pos_err_mm']) for r in rows], [50, 95, 99])
    te = np.percentile([float(r['track_err_mm']) for r in rows], [50, 95, 99])
    print(f'{name:8s} sr={sr:5.1f}%  pos_err p50/p95/p99={pe[0]:.1f}/{pe[1]:.1f}/{pe[2]:.1f}mm  '
          f'track_err p50/p95/p99={te[0]:.1f}/{te[1]:.1f}/{te[2]:.1f}mm')
"
```

預期：
- success rate **幾乎相同**（IK 本身沒變）
- pos_err p99 可能 **略升**（continuous approach 下，pose 跟著漂，IK 對「下一個 target」的 seed 起點不同）
- track_err 在連續 fail 段會明顯**較高**但**連續**（不像 legacy 是 0 + 跳變）

---

## 10. CLI 使用

```bash
# 預設（Set2-D continuous approach）
python3 placo_ik_online_profiler_ws_mesh.py --arm right

# 想對照舊行為
python3 placo_ik_online_profiler_ws_mesh.py --arm right --success-gate

# 完整推薦組合（teleop 平滑 + 邊界軟切 + 連續逼近）
python3 placo_ik_online_profiler_ws_mesh.py --arm right \
    --rate 100 \
    --wrist-vel-cap 4.0 \
    --lpf-alpha 0.9,0.9,0.9,0.9,0.4,0.4,0.4 \
    --boundary-margin 0.05
    # success-gate 不加 = 預設 continuous

# 安全 fallback（出問題時）
python3 placo_ik_online_profiler_ws_mesh.py --arm right --success-gate --wrist-vel-cap 2.5
```

---

## 附錄：和其他機制的關係

| 機制 | 對 fail 的角色 | 與 Set2-D 的關係 |
|---|---|---|
| Joint limits (URDF + post-clip) | fail 時 joints 仍合法 | ✅ 前提，Set2-D 依賴它 |
| Velocity limits (URDF + wrist cap) | fail 時 Δq 受限飽和 | ✅ Set2-D 的「飽和器」 |
| Adaptive DLS | 奇異點時 λ↑ 防 joints 爆衝 | ✅ Set2-D 在奇異點時的保護 |
| SoftClamp (Set2-A) | 邊界前先軟切 target | 配合：減少 OOR 觸發頻率 |
| BoundaryMonitor (Set2-E) | publish 邊界距離 topic | 配合：給 VR app 顯示反饋 |
| Output LPF (Set1-D) | publish 端再平滑 | 配合：抑制 fail 段的 IK 數值雜訊 |
| Stuck detection (Set2-F)  | TODO | 補充：連續 fail 時 auto resync ref |

---

## TL;DR

**舊邏輯**：IK 沒收斂到 10mm 就完全不送指令 → 撞牆瞬間手臂凍住。
**新邏輯**：永遠送 IK 的部分解；joint_limits + vel_limits + DLS 保證它合法且飽和；
`_pose` 跟著實際 EE 漂移防止累積誤差；連續 fail 時 console 警告。
**保險絲**：`--success-gate` 一秒回滾。
