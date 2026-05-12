# WorkspaceMesh 工具集說明

本文件涵蓋最近新增的 5 支 Python 工具，說明各工具的功能、架構與使用方式。

---

## 目錄

| 工具 | 說明 |
|------|------|
| [`placo_ws_reachability.py`](#1-placo_ws_reachabilitypy) | 掃描手臂可達空間，輸出 CSV + 分析圖 |
| [`placo_ws_analyze.py`](#2-placo_ws_analyzepy) | 由 CSV 建立 WorkspaceMesh，輸出 .npz / hull JSON / 分析圖 |
| [`placo_ws_view3d.py`](#3-placo_ws_view3dpy) | 互動 3D 可視化工具（Z slider + 投影面板）|
| [`placo_ik_online_profiler_ws_mesh.py`](#4-placo_ik_online_profiler_ws_meshpy) | 即時 delta IK profiler，以 WorkspaceMesh 取代矩形 clamp |
| [`placo_ik_absolute_profiler_ws_mesh.py`](#5-placo_ik_absolute_profiler_ws_meshpy) | 絕對座標 IK profiler，以 WorkspaceMesh 取代矩形 clamp |
| [`placo_ws_clamp_benchmark.py`](#6-placo_ws_clamp_benchmarkpy) | 比較 Mesh clamp 與 Box clamp 運算速度差異 |

---

## IK 如何「知道」Mesh Workspace？— 核心機制詳解

這是整個工具鏈最關鍵的問題。**重點：IK solver 本身不直接知道 workspace 形狀**；Mesh 的作用是在呼叫 IK 之前，把目標點「夾回（clamp）」到可達空間內，讓 IK 永遠只收到合法的目標。

### 階段一：離線掃描 → 建立 Voxel 點雲（`placo_ws_reachability.py` + `placo_ws_analyze.py`）

```
XYZ 網格 × 6 組方向
    │
    ▼ （對每個 voxel 執行 Placo IK）
CSV：每行 = (x,y,z,rpy, success, pos_err_mm, ...)
    │
    ▼ （placo_ws_analyze.py 讀取 CSV，過濾 success > 0）
reachable_xyz：shape (N, 3)  → 所有可達 voxel 的 XYZ 座標陣列
    │
    ▼ （scipy.spatial.KDTree 建在 reachable_xyz 上）
WorkspaceMesh 物件  →  儲存至 .npz
```

實際程式碼（`placo_ws_analyze.py`，`WorkspaceMesh.__init__`）：

```python
from scipy.spatial import KDTree

self._xyz  = np.array(reachable_xyz, dtype=np.float64)  # (N,3) 可達點雲
self._step = float(step)                                 # 掃描網格解析度（m）
self._tree = KDTree(self._xyz)                           # 建立 KDTree
```

`.npz` 檔內容只有三個欄位：
| 欄位 | 型態 | 說明 |
|------|------|------|
| `reachable_xyz` | `float64 (N,3)` | N 個可達 voxel 中心座標 |
| `step` | `float64` | 掃描解析度（公尺） |
| `min_orient_rate` | `float64` | 建立時的最低方向達率門檻 |

---

### 階段二：KDTree 如何判斷「在不在 workspace 內」

`WorkspaceMesh.contains(point)` 與 `WorkspaceMesh.clamp(point)` 都用同一個 KDTree 1-NN 查詢：

```python
dist, idx = self._tree.query(point)   # 找最近的可達 voxel
inside = dist < self._step * tol_factor   # tol_factor=0.60 → 約半個 voxel 對角線
```

- 若 `dist < step × 0.60`：點落在某個已知可達 voxel 的「鄰近範圍」內 → 視為可達
- 若 `dist ≥ step × 0.60`：點落在 voxel 間隙或完全超出範圍 → 視為不可達

`clamp()` 同時回傳修正後座標與 `was_inside` 旗標：

```python
def clamp(self, point):
    dist, idx = self._tree.query(point)
    if dist < self._step * tol_factor:
        return point.copy(), True          # 已在可達空間，不修改
    else:
        return self._xyz[idx].copy(), False  # 投影至最近可達 voxel
```

---

### 階段三：Runtime 整合（`placo_ik_online_profiler_ws_mesh.py`）

每個控制週期（預設 50 Hz）的執行順序：

```
1. 收到 /ee_delta/{arm} topic（PoseStamped，relative delta）
        │
        ▼
2. 積分計算「原始目標位置」：
   raw_x = base_x + dx_arm
   raw_y = base_y + dy_arm
   raw_z = base_z + dz_arm
        │
        ▼
3. WorkspaceMesh.clamp() 夾回可達空間         ← 這裡 Mesh 發揮作用
   if self._ws_mesh is not None:
       clamped, inside = self._ws_mesh.clamp([raw_x, raw_y, raw_z])
       new_x, new_y, new_z = clamped          # KDTree 1-NN，~8µs
   elif self._ws_clamp:
       new_x = max(ws["x"][0], min(ws["x"][1], raw_x))  # 矩形 fallback
   else:
       new_x, new_y, new_z = raw_x, raw_y, raw_z
        │
        ▼
4. Placo IK（PlacoSession.solve_step）
   target_xyz = np.array([new_x, new_y, new_z])
   target_R   = _quat_to_rot(*new_q)
   result = self._placo_session.solve_step(target_xyz, target_R, seed)
        │
        ▼
5. 發布 JointTrajectory → 機器人執行
```

關鍵點：**IK solver（`PlacoSession.solve_step`）本身完全不知道 workspace**。Mesh 只做「前處理守門員」，確保傳入 IK 的目標點一定在（或非常接近）掃描過的可達區域，避免 IK 在不可達位置掙扎求解。

---

### 初始化：WorkspaceMesh 載入流程

```python
# placo_ik_online_profiler_ws_mesh.py，PlacoOnlineProfiler.__init__()

self._ws_mesh: Optional[WorkspaceMesh] = None
npz_path = getattr(args, "ws_mesh", None)

# 1. 若有 --ws-mesh 參數 → 直接載入指定 .npz
if npz_path:
    self._ws_mesh = WorkspaceMesh.load(npz_path)
    self._ws_clamp = True   # 自動開啟 clamp

# 2. 無 --ws-mesh → 三段式 fallback（auto-detect → box → off）
# （auto-detect: 若 results/reachability_{arm}_ws.npz 存在，自動載入）
```

`WorkspaceMesh.load()` 只做三件事：
1. `np.load(path)` 讀取 `.npz`
2. 恢復 `reachable_xyz` array
3. 重建 `KDTree`（O(N log N)，N≈幾千，約 1–10ms）

---

### 三種 Clamp 模式比較

| 模式 | 觸發條件 | 演算法 | 延遲 | Workspace 形狀 |
|------|----------|--------|------|----------------|
| **WorkspaceMesh clamp** | `--ws-mesh` 指定或 auto-detect | KDTree 1-NN `O(log N)` | ~8 µs | 真實掃描形狀（非凸）|
| **矩形 box clamp** | 無 .npz 但 `--ws-clamp` 開啟 | `max/min` 逐軸夾 `O(1)` | ~0.3 µs | 矩形，過於保守 |
| **不限制** | `--no-ws-clamp` | 直接傳入 IK | 0 | 無限制，IK 可能失敗 |

---

## 完整工具鏈流程

```
掃描可達空間              建立 Mesh / 分析          視覺化（可選）
─────────────            ──────────────────       ─────────────
placo_ws_reachability  →  placo_ws_analyze      →  placo_ws_view3d
         │                      │ (.npz)
         │                      ▼
         │             profiler (online / absolute) 載入 .npz
         │             → WorkspaceMesh.clamp() 取代矩形 box
         │
         └──  placo_ws_clamp_benchmark（速度對比）
```

---

## 1. `placo_ws_reachability.py`

### 功能

以 Placo IK 對整個 XYZ 網格（多組方向）做掃描，record 每個 voxel 的可達率與 IK 殘差。**無需 ROS**，純 Python 可執行。

### 輸出

| 檔案 | 說明 |
|------|------|
| `results/reachability_{arm}_{ts}.csv` | 每行 = 一個 (xyz, rpy) 的 IK 結果 |
| `results/reachability_{arm}_{ts}_plot.png` | 4 列視覺化圖（位置可達性 + 方向覆蓋率）|

### CSV 欄位

```
x, y, z, roll_deg, pitch_deg, yaw_deg,
success, pos_err_mm, solve_ms, n_iters
```

### 4 列圖說明

| 列 | 內容 |
|----|------|
| Row 0 | 3D 位置可達圖 / 3D 方向覆蓋 / 每方向達率長條 |
| Row 1 | XY / XZ / YZ 位置可達性 heatmap（任一方向 OK = 1）|
| Row 2 | XY / XZ / YZ 方向覆蓋率 heatmap（mean n_ok / n_rpy）|
| Row 3 | IK 殘差分布 / solve_ms 分布 / 每 Z 層可達率 |

### 預設掃描方向（6 組 RPY）

```
0,0,0   0,45,0   0,-45,0   0,90,0   90,0,0   180,0,0
```

### 使用範例

```bash
# 右臂，預設 6 方向，step=5cm
conda run -n pico_teleop_py python3 placo_ws_reachability.py --arm right

# 自訂 step 與方向
conda run -n pico_teleop_py python3 placo_ws_reachability.py \
    --arm right --step 0.03 --rpy "0,0,0" "0,45,0" "90,0,0"

# 只跑快速粗掃（step=10cm）
conda run -n pico_teleop_py python3 placo_ws_reachability.py \
    --arm right --step 0.10
```

### 預設工作空間範圍

```python
"right": x(-0.20, 0.50)  y(-0.50, -0.05)  z(0.35, 0.90)
"left":  x(-0.30, 0.30)  y(0.05,  0.45)   z(0.05, 0.65)
```

---

## 2. `placo_ws_analyze.py`

### 功能

讀取 reachability CSV → 建立 `WorkspaceMesh`（可匯入用於 profiler）→ 輸出 .npz / hull JSON / 分析圖。

### 核心類別：`WorkspaceMesh`

```python
from placo_ws_analyze import WorkspaceMesh

ws = WorkspaceMesh.load("results/reachability_right_ws.npz")

# KDTree 鄰近判斷（快速，預設用於 clamp）
inside = ws.contains(np.array([0.2, -0.3, 0.5]))

# clamp 至最近可達 voxel（drop-in 取代矩形 box clamp）
clamped, was_inside = ws.clamp(np.array([0.2, -0.3, 0.5]))
# clamped: np.ndarray shape (3,) — 修正後的 xyz
# was_inside: bool — 原始點是否已在 workspace 內

# 嚴格凸包判斷（scipy Delaunay）
inside_hull = ws.in_convex_hull(np.array([0.2, -0.3, 0.5]))

# 統計摘要
summ = ws.summary()
# → {n_reachable_voxels, step_m, total_volume_cm3, bbox, ...}
```

### 輸出

| 檔案 | 說明 |
|------|------|
| `{stem}_ws.npz` | WorkspaceMesh voxel 點雲（可直接 `WorkspaceMesh.load()` 載入）|
| `{stem}_hull.json` | 凸包頂點 JSON（供外部工具使用）|
| `{stem}_analysis.png` | 3 列聚合總覽 + Z 層切片面板 |

### 使用範例

```bash
# 分析右臂掃描結果
conda run -n pico_teleop_py python3 placo_ws_analyze.py \
    results/reachability_right_20260511.csv --arm right

# 指定最低方向達率（過濾每方向達率 < 0.5 的 voxel）
conda run -n pico_teleop_py python3 placo_ws_analyze.py \
    results/reachability_right_20260511.csv --arm right --min-orient-rate 0.5
```

---

## 3. `placo_ws_view3d.py`

### 功能

互動式暗色主題 3D 可視化工具，顯示掃描的可達空間。

### 互動控制

| 控制項 | 說明 |
|--------|------|
| Z_min / Z_max slider | 篩選顯示高度範圍 |
| Show OK checkbox | 顯示可達點（綠色）|
| Show Fail checkbox | 顯示不可達點（紅色）|
| Convex Hull checkbox | 顯示凸包線框 |
| Reset Z button | 重設 Z 到全範圍 |

右側 4 個面板：XY / XZ / YZ 投影 + Z 方向可達率長條圖。

### 使用範例

```bash
# 從 CSV 啟動互動視圖
conda run -n pico_teleop_py python3 placo_ws_view3d.py \
    results/reachability_right_20260511.csv --arm right

# 非互動，直接存 PNG
conda run -n pico_teleop_py python3 placo_ws_view3d.py \
    results/reachability_right_20260511.csv --arm right --save view.png
```

---

## 4. `placo_ik_online_profiler_ws_mesh.py`

### 功能

`placo_ik_online_profiler.py` 的 WorkspaceMesh 版本。

- **輸入 topic**：`/ee_delta/{arm}` (PoseStamped) — 相對增量訊號
- **WorkspaceMesh clamp**（三段式）：
  1. 有 `--ws-mesh` → KDTree clamp（真實形狀）
  2. 無 mesh 但 ws_clamp 開啟 → 矩形 box clamp（fallback）
  3. `--no-ws-clamp` → 完全不限制

### 與原版差異

| 項目 | 原版 (`online_profiler`) | ws_mesh 版 |
|------|--------------------------|------------|
| workspace 限制 | 矩形 box | WorkspaceMesh KDTree（真實形狀）|
| fallback | 無 | 自動 fallback 至矩形 box |
| 額外參數 | — | `--ws-mesh <path.npz>` |
| auto-detect | — | 若 `results/reachability_{arm}_ws.npz` 存在即自動載入 |
| node 名稱 | `placo_ik_online_profiler` | `placo_ik_online_profiler_ws_mesh` |

### 使用範例

```bash
# 自動偵測 npz
conda run -n pico_teleop_py python3 placo_ik_online_profiler_ws_mesh.py --arm right

# 明確指定 npz
conda run -n pico_teleop_py python3 placo_ik_online_profiler_ws_mesh.py \
    --arm right --ws-mesh results/reachability_right_ws.npz

# Fallback 矩形 clamp（沒有 npz）
conda run -n pico_teleop_py python3 placo_ik_online_profiler_ws_mesh.py --arm right

# dry-run（不送 trajectory）
conda run -n pico_teleop_py python3 placo_ik_online_profiler_ws_mesh.py \
    --arm right --dry-run

# 先回 home
conda run -n pico_teleop_py python3 placo_ik_online_profiler_ws_mesh.py \
    --arm right --home-first
```

### 完整 CLI 參數

| 參數 | 預設 | 說明 |
|------|------|------|
| `--arm` | `right` | 手臂選擇 |
| `--rate` | `50.0` | 控制週期 Hz |
| `--horizon` | `60.0` | JointTrajectory 時間 ms |
| `--max-iter` | `250` | Placo 最大迭代數 |
| `--dt` | `0.010` | Placo solver dt |
| `--rebuild` | off | 每步重建 RobotWrapper（模擬原版 ~25ms）|
| `--ws-mesh` | auto | WorkspaceMesh .npz 路徑 |
| `--no-ws-clamp` | off | 關閉所有 workspace 限制 |
| `--home-first` | off | 先確認到達 home 再開始 |
| `--dry-run` | off | 計算 IK 但不發布 trajectory |
| `--csv` | auto | CSV 輸出路徑 |
| `--plot` | auto | PNG 輸出路徑 |
| `--verbose` | off | 每步印出（預設每 5 步）|

---

## 5. `placo_ik_absolute_profiler_ws_mesh.py`

### 功能

`placo_ik_absolute_profiler.py` 的 WorkspaceMesh 版本。

- **輸入 topic**：`/ee_target/{arm}` (PoseStamped) — **絕對座標**（tracker_ee_absolute.py 發布）
- 啟動時執行 **auto-calibration**：`offset = TF_EE_xyz − first_tracker_xyz`
- WorkspaceMesh clamp 替代矩形 box clamp

### 與絕對版原版差異

| 項目 | 原版 (`absolute_profiler`) | ws_mesh 版 |
|------|---------------------------|------------|
| workspace 限制 | 矩形 box | WorkspaceMesh KDTree（真實形狀）|
| fallback | 無 | 自動 fallback 至矩形 box |
| 額外參數 | — | `--ws-mesh <path.npz>` |
| auto-detect | — | 若 `results/reachability_{arm}_ws.npz` 存在即自動載入 |
| node 名稱 | `placo_ik_absolute_profiler` | `placo_ik_absolute_profiler_ws_mesh` |

### 使用範例

```bash
# Terminal 1: tracker 發布絕對座標
conda run -n pico_teleop_py python3 tracker_ee_absolute.py --tracker right

# Terminal 2: 絕對 IK（WorkspaceMesh clamp，自動偵測 npz）
conda run -n pico_teleop_py python3 placo_ik_absolute_profiler_ws_mesh.py --arm right

# 手動指定 npz + 先回 home
conda run -n pico_teleop_py python3 placo_ik_absolute_profiler_ws_mesh.py \
    --arm right --ws-mesh results/reachability_right_ws.npz --home-first

# 跳過自動校準（直接使用 tracker 原始座標）
conda run -n pico_teleop_py python3 placo_ik_absolute_profiler_ws_mesh.py \
    --arm right --no-auto-calib

# 手動指定偏移量
conda run -n pico_teleop_py python3 placo_ik_absolute_profiler_ws_mesh.py \
    --arm right --offset 0.05,-0.02,0.10
```

### 完整 CLI 參數

原版所有參數都保留，新增：

| 參數 | 預設 | 說明 |
|------|------|------|
| `--ws-mesh` | auto | WorkspaceMesh .npz 路徑 |

---

## 6. `placo_ws_clamp_benchmark.py`

### 功能

**不需要 ROS**，純 Python 執行。量化比較：

| 方式 | 演算法 | 複雜度 |
|------|--------|--------|
| Box clamp | `max(min, min(max, x))` × 3 軸 | O(1) |
| Mesh clamp | KDTree 1-NN query | O(log N) |

### 測試項目

1. **單點 clamp 延遲**（µs 級）— mean / median / p95 / max
2. **Mesh/Box 比率**（每次呼叫的時間倍數）
3. **Inside vs Outside** 分組延遲（box 外的點 mesh 差異）
4. **KDTree 大小 vs 延遲** scaling（從 step=10cm 到 step=2cm）

### 輸出

- 終端機統計表
- `results/clamp_benchmark_{timestamp}_{arm}.png`（6 個面板）

### 圖表面板

| 面板 | 內容 |
|------|------|
| 左上 | 每次 query 的 latency trace（Box vs Mesh overlaid）|
| 中上 | CDF 比較 |
| 右上 | Mesh/Box 比率直方圖 |
| 左下 | Mean & p95 統計長條 |
| 中下 | Mesh: Inside vs Outside 延遲比較 |
| 右下 | KDTree 大小 vs 延遲（log X scale）|

### 使用範例

```bash
# 使用真實 npz（推薦）
conda run -n pico_teleop_py python3 placo_ws_clamp_benchmark.py \
    --npz results/reachability_right_ws.npz

# 自動偵測 npz
conda run -n pico_teleop_py python3 placo_ws_clamp_benchmark.py --arm right

# 不需要 npz，使用合成 mesh 做速度比較
conda run -n pico_teleop_py python3 placo_ws_clamp_benchmark.py --synthetic --arm right

# 5000 次查詢，跳過 scaling sub-benchmark
conda run -n pico_teleop_py python3 placo_ws_clamp_benchmark.py \
    --arm right --n 5000 --no-scaling
```

### 典型結果（供參考）

```
╒═══════════════════════════════════════════════════════════════╕
│  Clamp Benchmark   arm=right   n=2000   inside_frac=48%      │
│═══════════════════════════════════════════════════════════════│
│  Method            mean    median     p95     max  [µs/call] │
│  ──────────────    ─────   ──────   ─────   ─────            │
│  Box clamp         0.35     0.30     0.58    3.12             │
│  Mesh clamp        8.20     7.80    14.50   45.30             │
│  ───────────────────────────────────────────────────────────  │
│  Mesh / Box ratio:  mean=23.4×                               │
│                                                               │
│  At 50Hz loop (20ms budget):                                  │
│    Box  overhead: 0.0004 ms  = 0.002% of budget              │
│    Mesh overhead: 0.0082 ms  = 0.041% of budget              │
╘═══════════════════════════════════════════════════════════════╛
```

> **結論**：Mesh clamp 約比 Box clamp 慢 20–30×（µs 級），但在 50Hz 20ms budget 下仍只佔 <0.05%，對 IK 效能**無實質影響**。

---

## 典型完整工作流程

```bash
# Step 1: 掃描右臂可達空間（~20min，step=5cm）
conda run -n pico_teleop_py python3 placo_ws_reachability.py --arm right --step 0.05

# Step 2: 建立 WorkspaceMesh（分析 + 存 .npz）
conda run -n pico_teleop_py python3 placo_ws_analyze.py \
    results/reachability_right_20260511_123456.csv --arm right

# Step 3（可選）: 互動視覺化
conda run -n pico_teleop_py python3 placo_ws_view3d.py \
    results/reachability_right_20260511_123456.csv --arm right

# Step 4（可選）: 速度 benchmark
conda run -n pico_teleop_py python3 placo_ws_clamp_benchmark.py --arm right

# Step 5a: 啟動 delta 模式 IK（WorkspaceMesh clamp）
conda run -n pico_teleop_py python3 placo_ik_online_profiler_ws_mesh.py --arm right

# Step 5b: 啟動絕對座標模式 IK（WorkspaceMesh clamp）
conda run -n pico_teleop_py python3 tracker_ee_absolute.py --tracker right &
conda run -n pico_teleop_py python3 placo_ik_absolute_profiler_ws_mesh.py --arm right
```

---

## 檔案相依圖

```
placo_ws_reachability.py
  └── placo_ik_solver  (PlacoSession 底層)

placo_ws_analyze.py
  └── scipy (KDTree / ConvexHull / Delaunay)
  └── matplotlib

placo_ws_view3d.py
  └── placo_ws_analyze       (WorkspaceMesh, _aggregate_by_xyz)
  └── matplotlib (interactive)

placo_ik_online_profiler_ws_mesh.py
  └── placo_ws_analyze       (WorkspaceMesh.load / clamp)
  └── placo_ik_solver        (PlacoSession 底層)
  └── ROS2 rclpy

placo_ik_absolute_profiler_ws_mesh.py
  └── placo_ws_analyze       (WorkspaceMesh.load / clamp)
  └── placo_ik_solver        (PlacoSession 底層)
  └── ROS2 rclpy

placo_ws_clamp_benchmark.py
  └── placo_ws_analyze       (WorkspaceMesh)
  └── matplotlib
```
