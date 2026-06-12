# Adaptive DLS Damping（方案 C）— Wrist Wobble 解決方案

> **實作檔案**：`ik_node/placo_ik_session.py`
> **相關常數**：`_DLS_LAMBDA_BASE`, `_DLS_LAMBDA_MAX`, `_DLS_SIGMA_THRESH`

---

## 目錄

1. [問題根因：奇異點與 Wrist Wobble](#1-問題根因奇異點與-wrist-wobble)
2. [DLS 原理](#2-dls-原理)
3. [固定 λ 的缺陷](#3-固定-λ-的缺陷)
4. [Adaptive DLS 設計（方案 C）](#4-adaptive-dls-設計方案-c)
5. [Placo API 確認與實作細節](#5-placo-api-確認與實作細節)
6. [常數調整指南](#6-常數調整指南)
7. [量化指標：CSV 欄位與 Plot 說明](#7-量化指標csv-欄位與-plot-說明)
8. [典型量化結果](#8-典型量化結果)
9. [與其他方案的關係](#9-與其他方案的關係)

---

## 1. 問題根因：奇異點與 Wrist Wobble

7-DOF 機械臂在某些姿態下 Jacobian 矩陣 $J \in \mathbb{R}^{6 \times 7}$ 會接近「奇異（singular）」—— 即某個方向的運動對應的關節速度趨向無限大。

典型觸發姿態：
- **Wrist singularity**：joint 5（wrist pitch）接近 0，joint 6 和 joint 7 軸線共線
- **Shoulder singularity**：arm 完全伸直
- **Elbow singularity**：joint 3 ≈ 0

在這些位置，QP solver 解出的 $\Delta q$ 非常大，造成：
1. 關節在兩側狂跳（**wobble** / **jitter**）
2. 速度限制被觸發 → 目標追蹤失敗
3. CSV 中 `pos_err_mm` 急升、`success = 0`

---

## 2. DLS 原理

Damped Least Squares（DLS）的核心：在 QP 的正則化項加上 λ，等效於在 pseudo-inverse 中引入 damping：

$$\Delta q = J^T (J J^T + \lambda I)^{-1} \Delta x$$

| λ 大小 | 效果 |
|--------|------|
| λ → 0  | 精準追蹤，奇異點附近 Δq → ∞ |
| λ → ∞  | Δq 被抑制，追蹤精度下降 |
| λ 自適應 | 遠離奇異點精準，靠近奇異點穩定 ✓ |

Placo 透過 `solver.add_regularization_task(λ)` 實現等效 DLS 正則化。

---

## 3. 固定 λ 的缺陷

原版 `_W_REG = 6e-5`（固定）的問題：

| 場景 | σ_min | Δq 量 | 結果 |
|------|-------|-------|------|
| 日常追蹤 | ~0.3~1.5 | 正常 | ✓ |
| 靠近 wrist singularity | ~0.01 | 爆炸性 | ✗ wobble |
| 完全奇異 | ~0 | 數值無窮 | ✗ crash |

固定小 λ 能保持遠離奇異時精準，但無法防護奇異時不穩定。

---

## 4. Adaptive DLS 設計（方案 C）

### 公式

$$\lambda = \lambda_\text{base} + \lambda_\text{max} \times \max\!\left(0,\;\frac{\sigma_\text{thresh} - \sigma_\text{min}}{\sigma_\text{thresh}}\right)^2$$

- $\sigma_\text{min}$：Jacobian 最小奇異值（即時計算）
- $\sigma_\text{thresh}$：開始觸發阻尼的門檻值
- $\lambda_\text{base}$：非奇異時的基準 λ（= 原 `_W_REG`，行為不變）
- $\lambda_\text{max}$：奇異時可升到的最大額外阻尼

### 行為曲線

```
σ_min ≥ 0.15  →  ratio = 0    →  λ = 6e-5   （原始行為，精準追蹤）
σ_min = 0.10  →  ratio = 0.33 →  λ = 1.2e-3
σ_min = 0.05  →  ratio = 0.67 →  λ = 4.5e-3
σ_min = 0.00  →  ratio = 1.0  →  λ = 1.0e-2  （最大阻尼，防止 wobble）
```

### 二次曲線的優勢

- 靠近 σ_thresh 時阻尼平滑升起（不突變）
- 完全奇異才達到最大（不浪費阻尼能力）
- 微分連續 → 不引入額外抖動

---

## 5. Placo API 確認與實作細節

### API 發現（v0.x Placo Python binding）

```python
# 原生 Jacobian（不需數值差分）
J_full = robot.frame_jacobian("openarm_right_link7", "world")
# → shape (6, 42)：全機器人 DOF（含 base + 兩臂 + 雙手）

# 取得某關節在 velocity DOF 空間的欄位索引
col = robot.get_joint_v_offset("openarm_right_joint5")   # → 28
```

### 實作：列索引預計算（`__init__`）

```python
# v_offset 只與 URDF 結構有關，不隨姿態改變 → 計算一次即可
self._jac_cols = [
    robot.get_joint_v_offset(n) for n in self._joint_names
]
# 右臂結果：[24, 25, 26, 27, 28, 29, 30]
```

### 實作：每步 solve 前計算（`solve_step`）

```python
J_full    = robot.frame_jacobian(self._ee_link, "world")   # (6, 42)
J_arm     = J_full[:, self._jac_cols]                      # (6, 7)
_svs      = np.linalg.svd(J_arm, compute_uv=False)        # 只算奇異值，更快
sigma_min = float(_svs[-1])

_ratio     = max(0.0, (_DLS_SIGMA_THRESH - sigma_min) / _DLS_SIGMA_THRESH)
lambda_dls = _DLS_LAMBDA_BASE + _DLS_LAMBDA_MAX * _ratio * _ratio

reg = solver.add_regularization_task(lambda_dls)   # 取代固定 _W_REG
```

### 效能開銷

| 操作 | 時間 |
|------|------|
| `frame_jacobian()` | ~0.005 ms |
| `np.linalg.svd(6×7)` | ~0.005 ms |
| **合計額外開銷** | **~0.01 ms** |

> 相比原版 ~25 ms 的 IK，開銷 < 0.05%，可以忽略。

---

## 6. 常數調整指南

```python
# placo_ik_session.py
_DLS_LAMBDA_BASE  = 6e-5   # 非奇異基準（原 _W_REG）
_DLS_LAMBDA_MAX   = 1e-2   # 奇異時最大增益
_DLS_SIGMA_THRESH = 0.15   # σ_min 低於此開始升阻尼
```

| 參數 | 調大效果 | 調小效果 | 建議範圍 |
|------|---------|---------|---------|
| `_DLS_LAMBDA_BASE` | 全程更平滑，精度略降 | 全程更精準，風險升高 | `1e-5` ~ `1e-4` |
| `_DLS_LAMBDA_MAX` | 奇異點更穩，but 誤差升高 | 奇異點仍可能 wobble | `5e-3` ~ `5e-2` |
| `_DLS_SIGMA_THRESH` | 更早觸發阻尼（更多步受影響） | 更晚觸發（只在深奇異點起作用） | `0.05` ~ `0.25` |

**診斷建議**：
- 若 `sigma_min` 常低於 `sigma_thresh`（plot 顯示長時間紅線以下）→ 適當調高 `_DLS_SIGMA_THRESH`
- 若 `lambda_dls` 常處於最大值但仍有 wobble → 調高 `_DLS_LAMBDA_MAX`（最高建議 `5e-2`）
- 若追蹤誤差上升但 wobble 已消失 → 可調低 `_DLS_LAMBDA_MAX`

---

## 7. 量化指標：CSV 欄位與 Plot 說明

### 新增 CSV 欄位

| 欄位 | 型別 | 說明 |
|------|------|------|
| `sigma_min` | float | 每步計算的 $\sigma_\text{min}(J_\text{arm})$，奇異點接近時趨近 0 |
| `lambda_dls` | float | 本步使用的實際 λ 值，範圍 `[6e-5, ~1e-2]` |

### Plot Row 4（新增）

最終輸出的 PNG 圖表（4×2，共 8 個子圖）：

```
Row 1:  [IK breakdown (ms)]      [Iterations (early exit)]
Row 2:  [IK residual (mm)]       [Tracking error (mm)]
Row 3:  [Deadline bar chart]     [Rolling success rate %]
Row 4:  [σ_min over time]  ←new  [λ_dls over time (log)]  ←new
```

#### 子圖 [3, 0]：σ_min（Jacobian 最小奇異值）

- **藍線**：每步的 σ_min 值
- **紅虛線**：`σ_thresh = 0.15`（觸發阻尼的門檻）
- **標題**：顯示 near-singular 步數比例（σ_min < σ_thresh 的步佔比）
- **解讀**：
  - σ_min 接近 0 = 接近奇異點 = λ_dls 升高
  - 長時間低於紅線 → 手臂常處於 wrist singularity 附近 → 考慮調整工作空間

#### 子圖 [3, 1]：λ_dls（Adaptive DLS 阻尼值，log 刻度）

- **紫線**：每步的 λ_dls 值（log scale）
- **灰虛線**：`λ_base = 6e-5`（非奇異基準）
- **紅虛線**：`λ_base + λ_max ≈ 1e-2`（最大阻尼上界）
- **解讀**：
  - 線靠近灰線 = 遠離奇異點，正常追蹤
  - 線靠近紅線 = 接近奇異點，阻尼啟動
  - 線長時間靠近紅線但 pos_err 不超過 10mm = 方案運作正常

---

## 8. 典型量化結果

以下為預期的對比數據（需實際運行驗證）：

| 指標 | 固定 λ（原版）| Adaptive DLS |
|------|------------|--------------|
| 正常追蹤 pos_err | ~2-5 mm | ~2-5 mm（不變）|
| Wrist singularity pos_err | 20+ mm（不穩定）| <10 mm（穩定）|
| λ 平均值 | 6e-5（固定）| 6e-5 ~ 2e-3（動態）|
| Wobble 觸發次數 | 多（無保護）| 少（自動阻尼）|
| 額外計算開銷 | 0 ms | ~0.01 ms |

---

## 9. 與其他方案的關係

| 方案 | 目標 | 與本方案關係 |
|------|------|-------------|
| 方案 A — SoftClamp | 防止 EE 超出工作空間 | 互補：SoftClamp 防止觸發奇異點的去向，DLS 處理已在奇異點的情況 |
| Fix-2 — velocity limits | 防止單步大跳 | 第一道防線；DLS 是第二道，處理 vel limit 無法覆蓋的連續小幅 wobble |
| Fix-D — output LPF | 關節指令平滑濾波 | 第三道防線：對已輸出的指令加平滑，但不能修正 IK 解的不穩定性 |
| 方案 C — Adaptive DLS | 在 IK 求解過程中動態調整阻尼 | **根源修正**：在 solver 層面防止 wobble，其他方案不需要 |

**建議組合**：SoftClamp（A） + velocity limits（Fix-2） + Adaptive DLS（C）+ 輕量 LPF（Fix-D，α=0.8）
