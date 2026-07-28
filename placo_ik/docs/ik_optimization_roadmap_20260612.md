# Placo IK 優化路線圖（2026-06-12 分析）

> 來源：對 `placo_ik_node.py`、`placo_ik_session.py`、`placo_ik_online_profiler_ws_mesh.py`
> 的完整 code review，並比對 `docs/vr_teleop_issues_20260529.md`、
> `docs/vr_teleop_remaining_issues.md`、`DEBUG_REPORT_20260525.md` 既有 issue 清單。
>
> 總結：solver 效能已達標（p95 ~2.5ms @ 50Hz、deadline miss 0%）。瓶頸不在解算速度，
> 而在控制管線語意：速度上限定義方式、開迴路漂移、固定參數濾波。

---

## 1. 速度上限語意錯誤 + 奇異點越限漏洞（最優先，安全相關）

**根因鏈**：

- `placo_ik_session.py` 把 arm cap 定義成 `radians(1°)/dt`，即「每次 `solver.solve()`
  最多 1°」。但每 tick 呼叫 `solve()` 最多 `max_iter` 次 → 實際預算 = 5°/tick。
- 接近奇異點時 `max_iter_dyn = 5 × 4 = 20`（`_SINGULAR_ITER_BOOST`），等效速度上限
  變成 **20°/tick——在最該慢的地方反而放寬 4 倍**。
- 然後 node 端 10° jump guard 逐軸獨立 clamp（`placo_ik_node.py` `_run_ik_pipeline`），
  破壞 IK 解的 7 軸耦合性 → 末端軌跡偏移（= docs issue #5 的 wrist 亂飄）。

**修法**：post-solve 統一比例縮放，取代逐軸 clamp（約 10 行）：

```python
delta = q_sol - seed
s = min(1.0, budget_rad / max(abs(d) for d in delta))
q_pub = [si + s * d for si, d in zip(seed, delta)]
```

一次解三件事：(a) 速度上限與 iteration 數解耦；(b) 保持解方向一致性（EE 沿直線
慢走而非走歪）；(c) 預算天然變 rad/tick，改 rate 不影響安全（docs #11）。

## 2. 固定 α LPF → One-Euro filter，並搬到 task space（體感改善最大）

- 現況：輸入端 SLERP EMA（預設關閉）+ 輸出端 per-joint LPF α=0.25。
  固定截止頻率的兩難：α 小 → 快速動作拖延遲；α 大 → 靜止時抖。
- **One-Euro filter**（~30 行）依速度自適應截止頻率：靜止重濾波、快動低延遲。
- 濾波點從關節空間搬到 **task space**（target xyz + quat 各一個 One-Euro）：
  per-joint 不同相位濾波會讓 EE 路徑走弧線（docs issue #3 根因），task space 濾
  則路徑形狀不變。輸出端 LPF 之後可拿掉或留極輕的一層。

## 3. `_pose` 開迴路漂移 → 加慢速 TF 閉迴路校正

- `_run_ik_pipeline`：success 時 `_pose ← target`、fail 時 ← IK 內部 FK，
  從未使用量測值。vel-cap 飽和 / guard 觸發 / 控制器跟不上時偏差累積，
  reanchor 那刻手臂跳一下（docs #2/#6）。
- TfPoller cache 已存在，只差回授：

```python
_pose = _pose + k * (tf_measured - _pose)   # k ≈ 0.02–0.05，姿態用 SLERP
```

- 配套：reanchor 那一次改用 blocking TF lookup（取代最舊 ~75ms 的 cache，docs #10）。

## 4. teleop 中途改 scale 會瞬跳（本次新發現）

- tracker delta 是「相對 anchor 的累積位移」，`_build_target` 做 `dx = delta × kbd.scale`。
- `kbd_controller.py` 改 scale（1-9/+/-）時不觸發 reanchor → 整段累積 offset 被
  瞬間重新縮放，offset 越大跳越大。
- 修法二選一：scale 改變時自動 reanchor（最簡單），或改成對增量套 scale。

## 5. Solver/任務每步重建 → 持久化 + 救活 per-joint weight

- `solve_step` 每步重建 `KinematicsSolver` + 全部 task。placo 支援直接更新
  task 目標（`pos_task.target_world = ...`），solver 建一次即可。
- 順帶修 docs 已確認的 **Bug 3**：per-joint weight 是 dead code（j3 的 0.80 沒被讀），
  拆成多個 joints_task 各自 configure 權重。

## 6. 奇異點：被動阻尼 → 主動降階

- **動態 W_ORI**（docs j3-4，幾行）：σ_min < 閾值時把姿態權重 2.0 → ~0.5，
  位置主導、wrist 不再為硬湊姿態而擺動。奇異方向幾乎都在姿態子空間，比全域 DLS 對症。
- **nullspace 可操作度梯度** task（低權重）：還沒進奇異區就繞開（中期）。
- seed retry library（fail streak 時換構型重解，docs j3-3）。

## 7. 延遲補償（欄位已有，未使用）

- `tracker_t_stamp` 已進 CSV 但未參與控制；端到端 ~60–100ms。
- 機制：用 One-Euro 的導數當速度估測，外推 `latency × v̂`，外推量上限 ~2cm。
- 依賴第 2 點先落地，否則外推放大雜訊。

## 8. 輸出端：50Hz 位置階梯 → 控制器側插值 / OTG

- ForwardCommandController 收 50Hz 位置階梯 = 馬達吃方波；現在靠 LPF+cap+guard
  三層事後磨平。
- 正規解：IK 輸出與命令之間放 **jerk-limited OTG（如 Ruckig）**，
  以 200–500Hz 輸出有 vel/acc/jerk 上限保證的插值。抖動的根治版（中期）。

## 9. 雙臂防撞（安全缺口，docs #13）

- 預設 `--arm both` 但 RobotWrapper 用 `ignore_collisions` 建，兩臂互撞無防護。
- 最低成本第一步：node 層做兩 EE/前臂線段最小距離檢查，逼近時降速。
- 正解見下方第 10 節（合併 QP + self-collision 約束）。

## 10. 雙臂合併單一 QP（兩個 end-effector）——✅ v1 已實作（2026-06-12）

> 實作與設計細節見 **`docs/bimanual_single_qp.md`**。新增檔案：
> `ik_node/placo_ik_session_bimanual.py` / `placo_ik_node_bimanual.py` /
> `placo_ik_online_profiler_bimanual.py`（未修改既有檔案）。
> v1 一併實作了本路線圖的 #1（統一 Δq 縮放）、#4（scale 改變 reanchor）、
> #6 的動態 W_ORI、N1（左臂 calib 預設 180°）、reanchor 進 CSV，
> 以及 per-arm adaptive DLS 與 EE 近距節流。
> 離線 smoke test 通過（穩態 0.14ms/step）；實機 A/B 待測。
> 以下為原始可行性分析（保留參考）。

**現況**：兩個 `PlacoSession` 各載入同一份雙臂 URDF（已確認 `v10_o6.urdf` 含
左右共 14 joints、兩個 link7），各只對自己 7 軸加 task，另一臂凍結。

**關鍵認知**：兩臂是運動學獨立鏈（共用 base、無共用關節），合併 QP **不改變 IK 解
本身**——價值在共享約束與架構：

1. **雙臂防撞的正解**：合併後 `solver.add_avoid_self_collisions_constraint()`
   讓 QP 解算時主動避開另一臂。⚠️ 前提：不能再用 `Flags.ignore_collisions`，
   且 URDF 要有合理 collision geometry——**最大未知數，要先驗證**。
2. **雙手協調任務**：placo 的 relative position/orientation task，
   之後做「雙手捧物」「保持相對位姿」只有合併 QP 做得到。
3. **架構簡化**：一個 hot loop 取代兩 thread；一份 RobotWrapper（記憶體減半）；
   兩臂 timing 不互漂。

**代價 / 設計點**：

- **奇異點耦合**：adaptive DLS 的 regularization 是全域的——一臂進奇異點拉高 λ
  會阻尼到另一臂。需 per-arm 處理：兩條 6×7 Jacobian 各算 σ_min，
  damping 分臂套用（如 joint-space 加權）。合併版最需要設計的一塊。
- **失敗耦合**：soft task 權重競爭，一臂不可達可能輕微影響另一臂；
  success 判定維持 per-arm。
- **工作量分布**：session 層改雙 EE ~1–2h；node 層（reanchor/LPF/guard/CSV
  全是單臂設計）重構成單 node 收兩個 `/ee_delta/{arm}` ~1 天。

**建議**：先做第 1 點（統一 Δq 縮放），合併版會用到同一機制；
合併時把 per-arm σ_min + per-arm 速度預算一起設計進去。

---

## 快速小修（各 <30 分鐘）

| 項目 | 說明 |
|------|------|
| N1 | `--arm both` 無 `--config` 時左臂 calib_yaw=0（應 180°）方向相反 → 直接 `sys.exit` 要求 config |
| reanchor 進 CSV | 加 `event` 欄，事後切片分析跳動 |
| help 字串 | `--lpf-alpha` 等 help 與實際 default 不一致；`bimanual.yaml` LPF α 仍是舊非統一值 |
| wrist cap 不一致 | session 內預設 `_WRIST_VEL_CAP=1.0`、CLI 預設 4.0——其他 import 方拿到不同行為，建議統一 |
| ~~DLS λ_max~~ | ✅ 已完成：`_DLS_LAMBDA_MAX=5e-2` 已套用（DEBUG_REPORT 建議） |

## 建議施工順序

1. **#1 統一 Δq 縮放**（安全 + 一致性一次解，~10 行）
2. **#4 scale 瞬跳** + 快速小修
3. **#3 TF 閉迴路校正**
4. **#2 One-Euro（task space）**
5. **#5 solver 持久化 + per-joint weight** / **#6 動態 W_ORI**
6. **#10 雙臂合併 QP**（先驗證 URDF collision geometry）
7. **#7 延遲補償** / **#8 Ruckig OTG** / **#9 雙臂防撞**（若 #10 先做則由其涵蓋）
