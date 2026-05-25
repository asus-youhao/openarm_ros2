# `placo_ik/offline_profilers/` — Teleop Analysis Toolkit

五個離線分析工具，從 `placo_ik_online_profiler_ws_mesh.py` 跑出來的 CSV 計算量化指標。

| 工具 | 回答的問題 | 輸出 |
|---|---|---|
| **[run_all.py](#0-run_allpy-one-shot-orchestrator)** | **一鍵跑全部** | bundle 資料夾（含 4 份 MD + 5 張 PNG + index） |
| [quantify_teleop.py](#1-quantify_teleoppy) | Solver 整體**健康度**好不好？ | 4 大類指標表 (🟢🟡🔴) + 4-panel PNG |
| [spatial_failure_map.py](#2-spatial_failure_mappy) | xyz 空間**哪裡**容易出事？ | top-N voxel 表 + 3-projection heatmap |
| [spatial_grid_3d.py](#3-spatial_grid_3dpy) | 不同 failure mode 在空間中是否**重疊**？ | 4×3 heatmap grid + 3D scatter |
| [spatial_compare.py](#4-spatial_comparepy) | 哪個 orientation / 哪隻手臂在某 voxel 比較糟？ | ori-slice rpy scatter 或 L−R diff heatmap |

---

## 共通設計

### 必要 CSV 欄位（最少）
`t, x, y, z, success, ik_ms, pos_err_mm, ori_err_deg, sigma_min, joint_jump_guard, deadline_missed`

### 進階欄位（解鎖更多指標）
- `q_cmd_0..6` → wrist/base FFT band ratio
- `tf_x, tf_y, tf_z, tf_qx..tf_qw` → closed-loop err + orientation slice
- `tracker_t_recv` → tracker→IK lag

→ 這些欄位由更新後的 `placo_ik_node.py::_write_step` 自動寫入。舊 CSV 缺欄位時對應指標顯示 `—`，不會炸。

### 互動 picker（四個工具共用）
不給檔案參數時，自動進入：
```
  Results root: /home/asus/ros2_ws_yh/src/openarm_ros2/placo_ik/results
  Pick a folder:
     1. 20260504
     ...
    11. 20260521
  folder # > 11

  CSVs in .../20260521:
     1. placo_online_..._right_cached.csv  (1271 KB)
     ...
    36. placo_online_20260521_162338_left_cached.csv  (1271 KB)
    37. placo_online_20260521_162338_right_cached.csv  (1281 KB)
  file # (comma/space-separated, or 'all') > 36 37
```

### 預設輸出
都寫到 **`<csv 所在資料夾>/<工具名>_<YYYYMMDD_HHMMSS>.{md,png}`**，不需指定 `--out`。

### 環境
全部用 `conda run -n pico_teleop_py --no-capture-output python3 ...`。`--no-capture-output` **必要** — `conda run` 預設會吞 stdin 害互動 picker 卡死。

建議 alias：
```bash
alias pp='cd /home/asus/ros2_ws_yh/src/openarm_ros2/placo_ik/offline_profilers'
alias runall='conda run -n pico_teleop_py --no-capture-output python3 /home/asus/ros2_ws_yh/src/openarm_ros2/placo_ik/offline_profilers/run_all.py'
alias qtl='conda run -n pico_teleop_py --no-capture-output python3 /home/asus/ros2_ws_yh/src/openarm_ros2/placo_ik/offline_profilers/quantify_teleop.py'
alias smap='conda run -n pico_teleop_py --no-capture-output python3 /home/asus/ros2_ws_yh/src/openarm_ros2/placo_ik/offline_profilers/spatial_failure_map.py'
alias sgrid='conda run -n pico_teleop_py --no-capture-output python3 /home/asus/ros2_ws_yh/src/openarm_ros2/placo_ik/offline_profilers/spatial_grid_3d.py'
alias scmp='conda run -n pico_teleop_py --no-capture-output python3 /home/asus/ros2_ws_yh/src/openarm_ros2/placo_ik/offline_profilers/spatial_compare.py'
```

---

## 0. `run_all.py` — One-shot orchestrator

**選一次 CSV，跑完全部四個工具，所有輸出進到同一個資料夾。**

### Bundle 資料夾命名
從 CSV 檔名 (`placo_online_YYYYMMDD_HHMMSS_<arm>_<mode>.csv`) 抽 HHMM + arm：

| 輸入 | 資料夾名 | 範例 |
|---|---|---|
| 1 CSV | `<HHMM>_<arm>` | `162338` → `1623_right` |
| 2 CSVs 同時間，left+right | `<HHMM>_lr` | `1623_lr` |
| 不同時間多 CSVs | `<HHMM_lo>-<HHMM_hi>_<arms>` | `1408-1623_lr` |

可用 `--folder-name` 覆寫。

### Bundle 內容
```
results/20260521/1623_lr/
├── index.md              ← 入口：列出所有 reports + 連結
├── quantify.md   .png
├── spatial_failure.md   .png
├── spatial_grid.md   .png   _3d.png
└── spatial_compare.md   .png
```

### `spatial_compare` 模式自動選擇
- 1 CSV → `ori-slice`（topn 自動壓到 ≤5 防 panel 太密）
- 2 CSVs 且檔名含 `_left_` + `_right_` → `lr-diff --mirror-left-y`
- 其他情況 → 退回對第一個 CSV 跑 `ori-slice`

### 用法
```bash
# 互動（最常用）
runall

# 直接給檔
runall left.csv right.csv
runall single.csv --voxel-size 0.03 --metric inv_sigma --topn 15

# 自訂資料夾名
runall left.csv right.csv --folder-name "baseline_v1"
```

### 用例 — 162338 雙臂
```bash
runall placo_online_20260521_162338_left_cached.csv \
       placo_online_20260521_162338_right_cached.csv
# → results/20260521/1623_lr/
#   index.md  quantify.md+png  spatial_failure.md+png
#   spatial_grid.md+png+_3d.png  spatial_compare.md+png  (auto lr-diff)
```

每個 sub-report 還是有自己的 `--out` 可以單獨跑，run_all 只是把參數編好幫你 batch 起來。

---

## 1. `quantify_teleop.py`

**用途**：把一個（或多個）session 壓成一張 4 區塊指標表，每個指標自動標 🟢🟡🔴 對照 pass/fail 門檻。

### 輸出
- **MD**：4 大類（Realtime / Tracking / Smoothness / Robustness）× N 個指標的表格，多 session 自動 side-by-side
- **PNG**：4-panel（ik_ms CDF / pos_err CDF / Δq hist / σ_min vs t）

### 4 大類指標 + 門檻
| 類別 | 主要指標 | 🟢 門檻 | 🔴 門檻 |
|---|---|---|---|
| Realtime | `ik_ms p95`, `deadline_miss%`, `loop_dt std` | <3 ms, 0%, <2 ms | >10 ms, >0.5%, >10 ms |
| Tracking | `pos_err p95`, `ori_err p95`, `track_err p95`, `tracker→IK lag p95` | <5 mm, <3°, <8 mm, <60 ms | >10 mm, >5°, >15 mm, >100 ms |
| Smoothness | `max\|Δq\| p99`, `guard hit%`, `q_cmd hi band wrist/base` | <8°, <0.1%, <3× | >15°, >1%, >10× |
| Robustness | `success%`, `σ_min<0.05%`, `cl_err p95` | ≥99%, <5%, <10 mm | <95%, >15%, >30 mm |

完整門檻表在報告底部自動列出。要調就直接編 `THRESHOLDS` dict。

### 用法
```bash
# 互動（推薦）
qtl

# 直接給檔
qtl session_a.csv session_b.csv --out report.md

# 不畫 PNG
qtl session.csv --no-plot
```

### 何時用
- 跑完一段 session 想知道整體有沒有過 baseline
- A/B 比較兩種設定（例如 `--ori-lpf-alpha 0.35` vs `1.0`）
- Regression 監控（新 commit 後跑同樣 scenario，看數字有沒有退步）

---

## 2. `spatial_failure_map.py`

**用途**：把 EE workspace 切成 5 cm voxel，找出**哪個位置**最容易爆 pos_err / σ_min / fail。

### 輸出
- **MD**：top-10 worst voxel（含 xyz 中心、n、dwell、各 metric）+ per-metric top-3 leaderboard
- **PNG**：1×3 heatmap（XY top / XZ side / YZ front），單一 metric，max-projection

### 可選 metric (`--metric`)
- `pos_err_p95` (default) — IK 解的位置誤差 tail
- `pos_err_p50` — 中位數版
- `ori_err_p95` — 方向誤差
- `fail_rate` — `success==0` 比例
- `guard_rate` — `joint_jump_guard==1` 比例
- `inv_sigma` — 接近奇異點程度（1/σ_min，越大越糟）

### 用法
```bash
# 互動 + 預設 pos_err_p95
smap

# 換成找奇異點熱區
smap --metric inv_sigma --topn 15

# 改 voxel 大小（3 cm 更精細）
smap session.csv --voxel-size 0.03
```

### 何時用
- 主觀感覺「手在某個方向會卡」→ 用客觀數字確認位置
- 想決定 `ws_mesh` 該不該剔除某個區域
- 想知道「奇異點熱區」對不對應「pos_err 熱區」（用換 metric 比對）

---

## 3. `spatial_grid_3d.py`

**用途**：`spatial_failure_map` 的進階版 — 同時把 4 種 metric 在 3 個視角畫出來，**一張圖看完所有 failure mode 的空間分佈**。再加 3D scatter，能直接看到 EE 真正走過的空間殼。

### 輸出
- **MD**：每個 metric 都列 top-10 voxel
- **PNG ×2**：
  - `*_grid.png` — 4 metric × 3 projection = 12-panel heatmap
  - `*_3d.png` — 3D scatter，所有 qualifying voxel 上色 + top-N 用黑圈標號

### 用法
```bash
sgrid                                   # 互動
sgrid session.csv --topn 15             # 直接給檔，多列幾個
sgrid session.csv --metric inv_sigma    # 3D 用奇異點上色
```

### 何時用
- 想知道「pos_err 熱區跟奇異點熱區是不是同一塊」→ 對比 grid 第 1 row 跟第 4 row
- Demo / report 需要一張「總覽圖」
- 3D scatter 用於判斷「EE 走過的殼形狀」是不是合理

---

## 4. `spatial_compare.py`

兩個 mode 同檔，自動偵測：

### Mode A: `--mode ori-slice`（單 CSV）

在最爛的 N 個 voxel 內，把每個 row 的 EE orientation（從 `tf_qx..qw` 解出 rpy）畫成 scatter，色塊 = pos_err。

→ 回答：**「同樣這個 xyz 點，是某個朝向特別爛、還是整個區域都爛？」**

額外 print：roll/pitch/yaw 跟 pos_err 的 correlation，找主導軸。

**需要**：CSV 有 `tf_qx..tf_qw` 欄位（新版 placo_ik_node 才有）。

### Mode B: `--mode lr-diff`（2 個 CSV，一左一右）

兩 CSV 各自聚成 voxel，找**兩臂都到過的共同 voxel**，畫 `left_metric − right_metric` heatmap：
- **紅色**塊 = 左臂在這 voxel 表現比較差
- **藍色**塊 = 右臂表現比較差

### 用法
```bash
# ori-slice（自動）
scmp session.csv --topn 3

# lr-diff（自動 — 給左右各一個 CSV，檔名含 _left_ / _right_）
scmp left_cached.csv right_cached.csv

# lr-diff with mirror（重要！）
scmp left_cached.csv right_cached.csv --mirror-left-y
```

### `--mirror-left-y` 重點
雙臂正常操作時左手在 base-frame y<0、右手 y>0，**兩臂幾乎沒有共同 voxel**。
加 `--mirror-left-y` 把左臂 y 軸翻轉，等於 **「兩臂在 arm-local frame 比較」**。多數時候要加這個 flag。

### 何時用
- 想找出「**這個 voxel 在 yaw 接近 ±90° 時才壞**」這種具體控制訊號（ori-slice）
- 想知道「**左右臂硬體 / IK config 是否對稱**」（lr-diff + mirror）

---

## 三種 spatial 工具怎麼選

```
要知道哪裡最壞 → spatial_failure_map      （簡單，單 metric，1 張 heatmap）
要看 failure mode 共現 → spatial_grid_3d  （4 metric 全擺出來 + 3D）
要研究某點 / 比較兩臂 → spatial_compare   （ori-slice 或 lr-diff）
```

時間預算：
- `quantify_teleop` ~1 秒（每個 CSV）
- `spatial_failure_map` ~1 秒
- `spatial_grid_3d` ~3 秒（12 panel + 3D）
- `spatial_compare ori-slice` ~2 秒
- `spatial_compare lr-diff` ~1 秒

---

## 典型工作流

### 預設：一鍵跑全部
```bash
runall
  → 選 folder/file
  → 自動建立 bundle 資料夾 + 跑全部 4 個工具
  → 開 index.md 從那裡 navigate
```

### 進階：個別跑 / 換參數
```bash
# 1. 跑一段 session 後先看整體
qtl
  → 開 quantify_<ts>.md, 看哪些 🔴

# 2. 如果 pos_err / σ_min / fail 有紅 → 找哪裡發生
smap --metric pos_err_p95

# 3. 想看 failure 模式是否相關 → 用 grid 對齊
sgrid

# 4. 鎖定熱區後想知道為什麼壞
#    (a) 單臂 → 看是哪個朝向有問題
scmp single_arm.csv --mode ori-slice --topn 3

#    (b) 雙臂 → 看是不是只有一臂壞
scmp left.csv right.csv --mode lr-diff --mirror-left-y
```

---

## 跟 IK node 端的關係

```
placo_ik_node.py
    ↓ writes
results/<date>/placo_online_<HHMMSS>_<arm>_<mode>.csv
    ↓ analysed by
offline_profilers/{quantify, spatial_failure, spatial_grid_3d, spatial_compare}.py
    ↓ outputs (gitignored)
results/<date>/{quantify, spatial_*}_<ts>.{md,png}
```

CSV 欄位由 `placo_ik_node.py::CSV_FIELDS` 定義（[L66](../ik_node/placo_ik_node.py#L66)）。新增欄位時，這四個工具要在 `_load_csv` 之後加對應的 `d.get("new_col", default)` 兼容處理。

---

## 開發 / Debug

```bash
# 語法檢查
python3 -c "import ast; ast.parse(open('quantify_teleop.py').read())"

# 模組 import 測試（檢查 sibling import 沒壞）
python3 -c "from spatial_grid_3d import _plot_grid; print('OK')"

# 不用 conda env 跑（numpy-only 部分；matplotlib 會 fail 但 MD 還是會生）
python3 quantify_teleop.py session.csv --no-plot
```
