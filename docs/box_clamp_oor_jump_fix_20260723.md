# Box workspace clamp — OOR 大範圍跳變修復

**日期**：2026-07-23
**檔案**：`placo_ik/ik_node/ws_boundary.py` — `SoftClamp._apply_box()`
**相關**：`placo_ik/config/arm_config.py`（box 範圍）、`placo_ik/ik_node/placo_ik_node.py:1144`（clamp 套用點）、`BoundaryMonitor.publish()`（`[BOX]/[OOR]/[OK]` 彩色提示）

---

## 1. 症狀

啟用矩形 box clamp（`--ws-clamp`，無 `--ws-mesh`）後，末端目標一旦接近或超出
workspace 邊界（OOR, out-of-range），下令位置會在「邊界面」與「session 參考點」
之間**大範圍來回跳變**，VR tracker 只要在邊界附近輕微抖動就會被放大成劇烈跳動。

---

## 2. 根因

舊版 `_apply_box` 用「縮放整段增量位移」的方式做 soft clamp，含兩個獨立缺陷，疊加後
在邊界附近產生跳變。

### 缺陷 A — `dx *= g` 把整段位移往 base 收，而非停在邊界

舊碼核心：

```python
dx[i] *= g                       # g = (d_near / margin)²
new_xyz = (raw_xyz - dx_arm_raw) + dx     # = base + g·dx
```

`new = base + g·dx`。越靠近邊界 `g → 0`，下令位置不是停在邊界，而是**塌回
`base`（session 參考點，可能離邊界很遠）**。

以右手 x 軸為例（base=0.30, hi=0.50, margin=0.05）：

| tracker 推到 raw_x | d_near_hi | g=(d/0.05)² | 舊版下令 new_x = 0.30 + g·dx |
|---|---|---|---|
| 0.45 | 0.05 | 1.00 | 0.450 |
| 0.48 | 0.02 | 0.16 | 0.329 |
| 0.50（貼邊界） | 0.00 | 0.00 | 0.300 |

手往外多推 3cm（0.45→0.48），下令位置反而往回彈 12cm；推到邊界直接彈回 base（離邊界 20cm）。

### 缺陷 B — 平方 gain 非單調，出界後「重新打開」

`g = (d/margin)²` 對 signed distance 是對稱拋物線（V 形），最小值 0 在邊界，
但越過邊界後 `d` 變負、平方又變大 → clip 回 1.0：

| 距邊界 d | −margin | 0（邊界） | +margin |
|---|---|---|---|
| g | 1.0（clip） | 0.0 | 1.0 |

一旦單步 delta 夠大、直接跳到「界外 > margin」，`g` 回到 1.0 → 完全不衰減 →
`new = base + dx`（遠在界外）→ 再被 `np.clip` **硬 snap 回邊界面**。

### 兩層疊加 = 跳變

邊界附近，下令位置在兩種極端之間切換：

- raw 貼在**邊界內側**（g≈0）→ 塌回 **base**
- raw 越過 margin 到**界外**（g→1 clip）→ dx 全放行 → `np.clip` snap 回**邊界面**

tracker 抖動讓 raw 在「界內 g≈0」和「界外 g=1」之間切換，下令位置就在
**base ↔ 邊界面**之間大幅來回。box 版界外又沒有 mesh 版的「投影到邊界、沿邊滑動」，
只靠 `np.clip` 硬 snap，進一步放大。

---

## 3. 修復：改為位置式（position-based）平滑飽和

不再縮放增量位移，改成把 **raw 目標位置**經過一個平滑「飽和」映射。每軸獨立處理，
上界為例（下界對稱）：

```
over = raw - (hi - margin)               # >0 一旦進入 margin 帶
enc  = margin · over / (over + margin)   # ∈ [0, margin)，帶入口斜率 1 → 遠處 → 0
new  = (hi - margin) + enc               # 嚴格 < hi
```

性質：

- **不依賴 base / dx** → 不會塌回參考點（消除缺陷 A）。
- **對 raw 單調遞增** → 界外越遠只是越貼近邊界，不會反向跳（消除缺陷 B）。
- **C¹ 連續**：帶入口斜率 = 1，與 passthrough 銜接，無速度不連續。
- **恆在界內**：`enc < margin` 恆成立 → `new < hi`，**不需要 `np.clip` 硬 snap**。
- **切向自由**：軸正交，未貼面的軸原樣通過 → 手可沿邊界面滑動，不黏死。

退化保護：若某軸 `hi − lo ≤ 2·margin`（兩側 margin 帶重疊，本專案 box 不會發生），
退回硬 clamp 至中段。

---

## 4. 修復前後對照（右手 x 軸，hi=0.62, margin=0.05）

| raw_x | 舊版 new_x | 新版 new_x | outside |
|---|---|---|---|
| 0.500 | 0.500 | 0.5000 | False |
| 0.570 | ~0.30（塌回 base 起） | 0.5700 | False |
| 0.610 | 塌回 base 附近 | 0.5922 | False |
| 0.620（貼邊界） | ≈ base | 0.5950 | False |
| 0.650（界外 3cm） | snap → 0.620 | 0.6008 | True |
| 0.900（界外 28cm） | snap → 0.620 | 0.6134 | True |
| 1.500（界外 88cm） | snap → 0.620 | 0.6174 | True |

新版：下令位置 0.500 → 0.6174 **單調、連續、永遠 < 0.62**，界外越深只是越貼邊界，
無任何跳變。

---

## 5. 驗證

```bash
python3 -m py_compile placo_ik/ik_node/ws_boundary.py     # 語法
# + 掃描 raw_x ∈ [0.50, 1.50]，確認 new_x 單調、恆 < hi、無 snap-back（已通過）
```

---

## 6. 使用方式與參數

啟用 box clamp（arm_config 的 workspace 範圍生效）：

```bash
python3 placo_ik/ik_node/placo_ik_online_profiler_ws_mesh.py --arm right --ws-clamp
```

`--boundary-margin`（預設 `0.05` = 5cm）：飽和帶寬度。調大 → 更早、更緩開始減速；
調小 → 幾乎到邊界才減速。

觸發時 console 會亮彩色提示（`BoundaryMonitor`）：

- 🟡 `[BOX]`：soft clamp 正在減速（顯示 `d`=距邊界、`cut`=被砍幾mm、`gain`）
- 🔴 `[OOR]`：raw 目標已出界（下令位置仍安全貼在界內）
- 🟢 `[OK]`：脫離邊界、clamp 解除

---

## 7. 已知後續（未在本次改動範圍）

`_apply_mesh`（`--ws-mesh` 路徑）在數學上有**類似的潛在問題**：near-boundary 時
`new = base + tangent + g·normal_proj`，當 `g→0` 法向分量同樣會收向 base 的法向座標。
目前無 `.npz` mesh 被自動偵測（canonical 路徑 `results/reachability_<arm>_ws.npz`
不存在），故未觸發。若日後改走 mesh，建議比照本次以位置式飽和重寫。
