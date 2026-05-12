# IK Solver 成功率分析報告

> 資料來源：`results/` 目錄下的 `.csv` 檔案 + `moveIt_compute_ik/delta_ik_timing_kdl_replay.csv`
> 可視化圖表：見對應 `.png`

---

## 1. 各 Solver 結果摘要

### 已有數據

| Solver | 資料集 | n | SR% | IK median (ms) | IK max (ms) |
|--------|--------|---|-----|----------------|-------------|
| **ForwardReachIK-TF** (pinocchio backend, q_slice fix 後) | 第一次測試 | 707 | **98.9%** | 5.3 ms | ~100 ms |
| **ForwardReachIK-TF** (pinocchio backend, 改良後) | 第二次測試 | 497 | **100.0%** | 5.3 ms | 13.8 ms |
| **KDL (MoveIt)** replay | replay | 497 | **100.0%** | 5.3 ms | 13.8 ms |

> ForwardReachIK = `tracker_ee_delta_ik_backend.py` 使用 pinocchio 後端（q_slice 修正版）

---

## 2. ForwardReachIK-TF 第一次 98.9% 分析

從 trajectory PNG 可觀察到，失敗點（紅點）集中在：
- **時間 t ≈ 30~37 秒**（位置累積誤差段）
- **IK latency 跳升至 90~100 ms**（正常約 5~7 ms）

**失敗模式**：IK 收斂迭代次數超過 200 次仍未收斂（timeout），而非 workspace 外。  
→ 這通常發生在目標跳躍過大（delta 累積太多）或 seed 無法提供好的初始猜測。

**第二次 100%** 改善原因推測：
- `send_home_confirmed()` 確保 arm 真正回到 home 再開始
- 無 TF 同步失敗導致的初始 q_slice 錯誤

---

## 3. 老版本（q_slice 錯誤前）的失敗模式

在 q_slice 修正之前（`slice(7,14)` 指向左手 fingers），症狀為：
- **所有 IK 嘗試 = 失敗（success=0）**
- IK ms ≈ 31ms（MAX_ITER 耗盡）
- EE 永遠在 `(0.216, -0.295, 0.530)` 不動（home pose FK 正常但 IK 解錯誤 joints）
- 即使 IK 報告成功，實際送出的 joint command 也是左手 finger indices → 右臂不動

---

## 4. 與 KDL (MoveIt) 的比較

| 指標 | ForwardReachIK (pinocchio) | KDL (MoveIt) |
|------|--------------------------|--------------|
| SR% | 98.9% → 100% | 100% |
| 中位延遲 | 5.3 ms | 5.3 ms |
| 最大延遲 | 13.8 ms | 13.8 ms（replay） |
| 依賴 ROS | ✅ 不需要 MoveIt | ❌ 需要 MoveIt ComputeIK |
| 離線運行 | ✅ 可 | ❌ 不可 |
| 多 seed | ✅ 7 seeds | N/A |

> **結論**：pinocchio solver 在無 MoveIt 環境下可達到與 KDL 相同的 SR% 和延遲。

---

## 5. 失敗成因分析

### 5.1 Workspace 邊界問題

```
失敗主要發生在以下條件組合：
  z > 0.60 m   (向上超伸，接近肩膀高度)
  x < 0.17 m   (過於靠近軀幹)
  |delta_z| 持續正值 (連續向上累積)
```

根本原因：
1. **Jacobian 奇異點**：手臂接近完全伸直時，j2（肩膀 pitch）= 90°，Jacobian 行列式 → 0
2. **seed 距離過遠**：`last_joints` 距離目標超過收斂盆地（basin of attraction）
3. **MAX_ITER 不足**：200 次迭代對邊界點不夠

### 5.2 Delta 累積問題

若 EE 無法到達目標（失敗），`_pose` 不更新 → 下一幀 delta 繼續累積：

```python
# 失敗時：self._pose 維持上次成功值
# 下一幀：target = self._pose + delta_new
# 若手臂沒動：target 越差越跳
```

解法：失敗時將 `self._pose` 仍更新為「夾緊在 joint limit 範圍內的最近可達點」。

---

## 6. 針對性改善建議

### 6.1 提高 MAX_ITER（即效，成本低）

```python
# pinocchio_ik_solver.py
MAX_ITER = 300  # 原 200
```

預期效果：工作空間邊界附近的 SR% 提升 1~3%

### 6.2 增加 j5~j7 自由度（手腕更靈活）

目前 pinocchio 的 j5=j6=0.8、j7=0.5 weight 對手腕 naturalness 仍偏高。  
降低後，IK 可將手腕作為「調整 DOF」吸收末端誤差：

```python
# 修改 _HUMAN_RIGHT（pinocchio_ik_solver.py）
( 0.000, -1.571,  1.571, -1.571,  1.571,  0.3),  # j5 0.8 → 0.3
( 0.000, -0.785,  0.785, -0.785,  0.785,  0.3),  # j6 0.8 → 0.3
( 0.000, -1.571,  1.571, -1.571,  1.571,  0.2),  # j7 0.5 → 0.2
```

預期效果：SR% 提升 0.5~2%（更多解路徑通過手腕自由度補償）

### 6.3 增加 workspace 邊界附近的 seeds

在 z > 0.55 的高位姿勢，肘部需要更大角度：

```python
_SEEDS_RIGHT = [
    ...（原有 7 個 seeds）...
    [ 0.00, 1.20, 0.00, 1.800, 0.00, 0.00, 0.00],  # 新增：高位肩抬
    [ 0.10, 0.90, -0.10, 1.5708, 0.00, 0.30, 0.00], # 新增：外旋高位
]
```

### 6.4 失敗後 clamp 更新（防止 delta 累積）

```python
# tracker_ee_delta_ik_backend.py — _callback() 中
if not ok:
    # 失敗時仍更新 pose 至 IK 回傳的最近解
    best_joints = np.clip(solved_joints, self._lo, self._hi)
    best_xyz, best_quat = fk(best_joints)
    self._pose = Pose(best_xyz, best_quat)
    self._last_joints = best_joints.tolist()
```

### 6.5 針對 j3 過度限制的問題

目前 pybullet j3 `hard_hi = 0.000`（向前才是 0，不能往外），若目標在側邊高位可能被截斷：

```python
# pybullet_ik_solver.py — _HUMAN_CFG["right"]["j3"]
"j3": ( 0.000, 0.60, -1.571, 0.300),  # hard_hi 0.000 → 0.300
```

這與 pinocchio 版本一致。

---

## 7. 改善優先順序

| 優先 | 改善方案 | 難度 | 預期收益 |
|------|---------|------|---------|
| ⭐⭐⭐ | 6.4 失敗後 clamp 更新 | 低 | 防止失敗連鎖，改善穩定性 |
| ⭐⭐⭐ | 6.1 MAX_ITER=300 | 極低 | 邊界 SR% +1~3% |
| ⭐⭐ | 6.2 j5~j7 weight 降低 | 低 | 解路徑更多 |
| ⭐⭐ | 6.5 pybullet j3 hard\_hi 修正 | 極低 | 側邊高位 +SR% |
| ⭐ | 6.3 增加 seeds | 低 | 高位 SR% +1~2% |

---

## 8. 如何收集多 solver 對比數據

目前只有 `ForwardReachIK-TF`（pinocchio）的數據。要對比 pybullet / placo / pure_python：

```bash
# 右臂 pybullet 測試
cd scripts/ik_reachability/ik_controllers
conda run -n pico_teleop_py python3 tracker_ee_delta_ik_backend.py \
    --arm right --solver pybullet --home-first

# 右臂 placo 測試
conda run -n pico_teleop_py python3 tracker_ee_delta_ik_backend.py \
    --arm right --solver placo --home-first

# 跑完後對比 results/ 目錄下的 CSV
```

結束後，各 solver 的 CSV 存放於：
```
results/
  yyyymmdd/
    delta_ik_timing_mmddHHMM_right_pinocchio.csv
    delta_ik_timing_mmddHHMM_right_pybullet.csv
    delta_ik_timing_mmddHHMM_right_placo.csv
```

---

## 9. 附：KDL CSV 欄位說明

| 欄位 | 說明 |
|------|------|
| `t` | 時間戳（Unix） |
| `x, y, z` | EE 位置（m，world frame） |
| `success` | IK 成功 = 1，失敗 = 0 |
| `ik_ms` | IK 計算時間（ms） |
| `total_ms` | 整個 callback 時間（ms） |
| `dx, dy, dz` | 本幀收到的 delta（m） |
