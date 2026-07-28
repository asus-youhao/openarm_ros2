# Placo IK 應用於 VR Teleop — 剩餘問題深度分析（2026-05-29 更新版）

**對象**：[`placo_ik/ik_node/placo_ik_node.py`](../ik_node/placo_ik_node.py) + [`placo_ik/ik_node/placo_ik_session.py`](../ik_node/placo_ik_session.py) + [`placo_ik/ik_node/placo_ik_online_profiler_ws_mesh.py`](../ik_node/placo_ik_online_profiler_ws_mesh.py)

**本檔取代** [`vr_teleop_remaining_issues.md`](./vr_teleop_remaining_issues.md)（2026-05-27 版）。
舊版分析基於 `_MAX_ITER=5` 無 adaptive boost、jump_guard=15、λ_max=1e-2、per-joint LPF 不一致、單右臂預設等**已過時**的程式碼狀態。本檔依 **2026-05-29 當前程式碼**重新核對每一條 issue 的行號與是否仍存在，並標出已解決項目。

---

## 0. 當前裸指令的實際生效設定（核對自原始碼）

> 指令：`python3 ./placo_ik_online_profiler_ws_mesh.py`（不帶任何參數）

| 項目 | 生效值 | 來源 | 備註 |
|---|---|---|---|
| `--arm` | **both 雙臂** | profiler L101 `default="both"` | ⚠ 預設就跑雙臂 |
| `--config` | 無 | L102 `default=None` | ⚠ 雙臂用相同 CLI 預設，左臂 calib_yaw 不會是 180（見 N1） |
| `--rate` | 50 Hz | L105 | solver dt = 0.02s |
| `--home-first` | **ON** | L148 `default=True` | 啟動先把兩臂送 home（見 N4） |
| ws_clamp | **OFF** | L152 `--no-ws-clamp default=True` | 預設關閉工作空間夾取 |
| j3j4 coupling | **OFF** | L166 `--no-j3j4-couple default=True` | 預設關閉肘-軀幹耦合 |
| `--lpf-alpha` | **0.25 ×7（統一）** | L119 | ⚠ help 字串寫的 `0.25,0.25,0.25,0.4,0.6,0.6,0.6` 已過時（見 N3） |
| `--ori-lpf-alpha` | **1.0（關閉）** | L126 | 方向不做 SLERP 平滑 |
| `--joint-jump-guard-deg` | **10°** | L132 | guard clamp 模式（爬行） |
| `--wrist-vel-cap` | 4.0 rad/s | L115 | 覆寫 session 預設 1.0 |
| arm cap (j1-4) | **1.0°/iter** | session L90 `_ARM_VEL_CAP_DEG_PER_ITER=1.0` | 無 CLI flag，固定生效（精密夾取設定） |
| `--success-gate` | off | — | IK 失敗仍發 partial（Set2-D） |
| adaptive max_iter | **ON** | session L312-314 | 5 → 20 when σ_min<0.15 |
| λ_max | 5e-2 | session L65 | DLS 奇異點阻尼 |

**每 tick（20ms）關節位移上限換算**（dt=0.02s）：
- arm（j1-4）：1°/iter × 5 iter = **5°/tick**（非奇異）；奇異時 ×20=20°/tick，但被 jump_guard 夾到 10°
- wrist（j5-7）：4 rad/s → 4.58°/iter × 5 = **22.9°/tick** → **超過 jump_guard 10°**

> **重要結論（與舊版相反）**：現在 arm 的速度 cap（5°/tick）< jump_guard（10°），所以
> **jump_guard 主要夾的是 WRIST（22.9°），不是肩肘**。快速轉手腕時才會頻繁觸發 guard。

---

## 一、相較舊版「已解決 / 已改變」的項目

| 舊 issue | 狀態 | 說明 |
|---|---|---|
| #4 `_MAX_ITER=5` 奇異點不收斂 | ✅ **已修** | adaptive max_iter 已實作（session L312-314）：σ_min<0.15 時 ×4=20 iter。搭配 λ_max=5e-2，深奇異點有迭代預算收斂 |
| #3 per-joint LPF α 不一致 → EE 彎路徑 | 🟡 **預設已緩解** | CLI 預設改成統一 0.25 ×7 → 不再有人為去同步。**但** bimanual.yaml 的 common 仍是 `0.25,0.25,0.25,0.4,0.6,0.6,0.6` → 載 config 時 issue 復現（見下方 #3 重述） |
| jump_guard=15 | 改為 **10** | profiler L132 預設改 10 |
| λ_max=1e-2 | 改為 **5e-2** | session L65 |
| arm 速度無 cap | 新增 **1°/iter cap** | session L90，j1-4 在 QP 內限速 |
| 單右臂預設 | 改為 **both 預設** | profiler L101 |

---

## 二、嚴重度速覽（2026-05-29 重排）

| #  | 問題 | 嚴重度 | VR 徵兆 | 修正成本 |
|----|-----|-------|---------|---------|
| N1 | `--arm both` 無 config → 左臂 calib_yaw=0（應為 180） | 🔴 高 | 裸指令下左臂方向鏡像/反向 | 低 |
| 1  | `_pending` 覆蓋 → 訊息聚合錯誤 | 🔴 高 | 快移時手臂跟不上、delta 累積 | 中 |
| 2  | `target_R` / `_pose` 在 IK 失敗時方向不更新 | 🟠 中-高 | 鬆開再按下方向感不對 | 低 |
| 5  | jump guard 逐軸獨立 clamp → 破壞 IK 一致性 | 🟠 中-高 | 快轉手腕時 wrist 軌跡偏離指令 | 中 |
| 6  | `_pose` 開迴路 → vel-limit 飽和時積壓 | 🟠 中 | 飽和落後、鬆手位置跳 | 中-高 |
| 3  | LPF 各關節 α 不一（僅載 config 時） | 🟡 中 | 載 bimanual.yaml 後動態 EE 走弧 | 低 |
| N3 | profiler help 字串 / 預設文件漂移 | 🟡 中-低 | 看 help 以為 LPF 非統一、誤判 | 極低 |
| 9  | tracker header.stamp 有記錄但無延遲補償 | 🟡 中-低 | 總延遲 ~60-100ms 無自我感知 | 中-高 |
| 10 | TF poller cache 在 reanchor 最舊 ~75ms | 🟡 中-低 | session 啟動瞬間 1-3mm 誤差 | 低 |
| 12 | 沒有 VR 端失敗回饋 | 🟡 中 | fail/guard 時操作者無感 | 中 |
| 13 | bimanual 兩臂無 collision 感知（預設跑雙臂） | 🟠 中 | 兩臂可互撞，且現在預設雙臂 | 高 |
| N4 | home-first 預設 ON → 啟動兩臂自動運動 | 🟡 中-低 | 啟動瞬間機械臂動，需確認淨空 | 低 |
| 7  | 方向不受 ws clamp 限制（僅 `--ws-clamp` 時） | 🟢 低 | 邊界處 wrist wobble | 中 |
| 8  | SoftClamp 用意圖位置（僅 `--ws-clamp` 時） | 🟢 低 | 邊界外推可無限延伸 | 低 |
| 11 | jump guard 是 deg/step → 跟 rate 耦合 | 🟢 低 | 切 rate 後安全性變 | 低 |
| 14 | `_filt_joints` 讀寫未全納鎖 | 🟢 低 | 極罕見 torn-read | 低 |
| 15 | reanchor 事件未進 CSV | 🟢 低 | 事後切片困難 | 低 |
| N2 | argparse `store_true default=True` 正向 flag 失效 | 🟢 低 | `--home-first`/`--no-ws-clamp` 是 no-op | 低 |

---

## N1. `--arm both` 無 config → 左臂校正錯誤 🔴（裸指令最關鍵）

**程式碼**：[profiler L101](../ik_node/placo_ik_online_profiler_ws_mesh.py#L101)（`--arm default="both"`）、[L256-257](../ik_node/placo_ik_online_profiler_ws_mesh.py#L256-L257)
```python
elif not args.config:
    print("  ⚠  --arm both without --config: both arms use identical CLI defaults")
```

**問題**：
裸指令預設 `--arm both`，但 `--config=None`。兩臂都套用相同的 CLI 預設，其中 `--calib-yaw default=0.0`。
正確設定（bimanual.yaml）裡 **左臂 calib_yaw 必須是 180°** 才能把 tracker 座標映射到左臂 base frame。
→ 裸指令下左臂的 tracker→arm 映射差 180° yaw → **左臂往反方向/鏡像移動**，操作者推右、左臂往左。

**修正**：
- 裸指令應視為「不可直接用於雙臂」，必須帶 `--config ../config/bimanual.yaml`。
- 或把 `--arm` 預設改回 `right`（單臂裸測安全），雙臂強制要求 config（程式可在 `args.arm=="both" and not args.config` 時 `sys.exit` 而非只印警告）。

---

## 1. `_pending` 覆蓋 → 訊息聚合錯誤 🔴

**程式碼**：[placo_ik_node.py:424-436](../ik_node/placo_ik_node.py#L424-L436)
```python
def _ee_delta_cb(self, msg: PoseStamped):
    ...
    with self._delta_lock:
        self._pending        = msg          # ← 直接覆蓋，前一筆丟棄
        self._pending_t_recv = t_recv
```

**現況**：publisher（XRoboToolkit ~40-60 Hz）> IK rate（50 Hz）時，tick 間多筆訊息只留最後一筆。
position delta 是相對 publisher 端 `ref_pos`（單次 trigger 內固定）→ 丟中間筆對「位置」尚可（最新即正確）。
**真正 bug** 在 publisher 端 dead-zone「累積後才發」：靜止久後一次送大 delta → IK 一次收到大位移 → vel_limit 飽和 → 多 frame 才追到 → 視覺 lag。現在 arm cap 只有 1°/iter（5°/tick），**飽和更容易發生**，此 issue 反而比舊版更明顯。

**建議**：callback 端累加 position delta（quaternion 取最新）；或 publisher 改 on-transition 發送；長期 IK/publisher rate 對齊。

---

## 2. IK 失敗時方向不更新（`target_R`/`_pose` 漂移）🟠

**程式碼**：[placo_ik_node.py:1145-1151](../ik_node/placo_ik_node.py#L1145-L1151)
```python
if r["success"]:
    self._pose = [nx, ny, nz, new_q[0], new_q[1], new_q[2], new_q[3]]  # 寫 target_q（非 FK 實際）
else:
    self._pose[0] = float(ee[0])     # 只更新位置
    self._pose[1] = float(ee[1])
    self._pose[2] = float(ee[2])     # ← _pose[3:7] 方向完全沒更新
```

**問題**：success 時寫的是 target 方向（非實際 FK）；fail 時方向殘留上一輪。
tracker 模式下 `base_q = self._ee_delta_ref_q`（session ref 固定，[L1039](../ik_node/placo_ik_node.py#L1039)）→ 對「下一步目標計算」影響有限；但連續 IK 失敗期間 `_pose` 的方向與實機 FK 偏離，鬆手 reanchor 時 `_pose=TF` 會突然校正 → 重新按下時方向感不對。

**修正**：success 分支也用 FK 的實際 q 寫 `_pose[3:7]`（需 session 回傳 `ee_R`），或每 success step 直接從 `_tf_poller.get()` 讀回（同時修 #6）。

---

## 5. jump guard 逐軸獨立 clamp → 破壞 IK 一致性 🟠

**程式碼**：[placo_ik_node.py:1116-1122](../ik_node/placo_ik_node.py#L1116-L1122)
```python
guard_rad = math.radians(self._joint_jump_guard_deg)
joints_clamped = [
    max(s - guard_rad, min(s + guard_rad, j))   # ← 每軸獨立 clip
    for s, j in zip(seed, r["joints"])
]
```

**問題（已隨速度 cap 改變）**：
IK 解的 7 軸是配對的；獨立 clip 某一軸後 FK 不再對應任何合理 EE pose。
**新重點**：現在 arm 5°/tick < guard 10° → 肩肘幾乎不觸發 guard；**真正會被 clamp 的是 wrist（22.9°/tick > 10°）**。
→ 快速轉手腕時 wrist 三軸被逐一夾、彼此比例被打亂 → **末端姿態（orientation）軌跡偏離指令**，操作者覺得「轉手腕時方向歪掉」。

**修正**：用統一 scale 保方向：
```python
delta_q = [j - s for s, j in zip(seed, r["joints"])]
max_abs = max(abs(d) for d in delta_q)
if max_abs > guard_rad:
    scale = guard_rad / max_abs
    joints_clamped = [s + d * scale for s, d in zip(seed, delta_q)]
```

---

## 6. `_pose` 開迴路 → vel-limit 飽和時積壓 🟠

**程式碼**：[placo_ik_node.py:1039](../ik_node/placo_ik_node.py#L1039)、[1145-1151](../ik_node/placo_ik_node.py#L1145-L1151)
`_pose` success=target、fail=IK 內部 FK，**都不是實際 TF**。arm cap 現在僅 1°/iter（5°/tick），飽和比舊版更頻繁 → 內部 `_pose` 與 TF 偏差更易累積 → 鬆手 reanchor 時位置跳。

**修正**：每 success step 從 TF 寫回 `_pose`（代價：cache 最舊 75ms，見 #10）。

---

## 3. 各關節 LPF α 不一（僅在載 config 時）🟡

**現況**：CLI 預設已改統一 `0.25 ×7`（[profiler L119](../ik_node/placo_ik_online_profiler_ws_mesh.py#L119)）→ 裸指令**不再有**此問題。
**但** [bimanual.yaml](../config/bimanual.yaml) common 仍是 `0.25,0.25,0.25,0.4,0.6,0.6,0.6`，且雙臂必須帶 config → **正式雙臂跑法仍會復現**：肩 25%、肘 40%、腕 60% 不同步 → 動態時 EE 走歪斜弧、停下才準。

**修正方向**：(A) task-space 濾波（在 target_xyz/R 端，不在關節端，根治）；(B) bimanual.yaml 也改統一 α，靠 wrist_vel_cap + DLS 抑 wrist noise；(C) 量化 `q_cmd_*` vs `tf_*` 動態誤差後決定。

---

## N3. profiler help 字串 / 文件漂移 🟡

[L125](../ik_node/placo_ik_online_profiler_ws_mesh.py#L125) help 寫 `Default: 0.25,0.25,0.25,0.4,0.6,0.6,0.6`，但 [L119](../ik_node/placo_ik_online_profiler_ws_mesh.py#L119) 實際 default 是統一 `0.25 ×7`。
[L126-128](../ik_node/placo_ik_online_profiler_ws_mesh.py#L126-L128) ori-lpf help 寫「0.35 = moderate (default)」，實際 default=1.0（關閉）。
→ 看 `--help` 會誤判實際行為。**修正**：對齊 help 字串與真實 default（極低成本）。

---

## 9. tracker 延遲未補償 🟡

`tracker_t_stamp`/`tracker_t_recv` 有記錄無使用。總延遲鏈 ~60-100ms（VR→client→ROS→IK loop→controller→馬達物理響應）。可用 `now - tracker_t_stamp` 做前向預測，但需平滑速度估測，過頭會 overshoot。

---

## 10. TF poller cache 在 reanchor 最舊 ~75ms 🟡

**程式碼**：[placo_ik_node.py:183-195](../ik_node/placo_ik_node.py#L183-L195)（poll_sec=0.05，timeout 50ms + sleep 25ms）。
reanchor 時 `_pose=list(ref)` 寫入 cache 值；若機械手仍有殘餘移動 → session 啟動瞬間 1-3mm 誤差。
**修正**：reanchor 時改 blocking `_get_tf(0.05)`（只發生一次）。

---

## 12. 無 VR 端失敗回饋 🟡

fail/guard/boundary 只進 console，頭盔內看不到。可擴充 BoundaryMonitor 發狀態 topic，VR app 訂閱做震動/UI。

---

## 13. bimanual 無 collision 感知 🟠（預設雙臂後升級）

每臂獨立 PlacoSession，互不知對方位置，URDF 載入用 `ignore_collisions`。**現在裸指令預設就是雙臂** → 兩手交叉可互撞，安全層面缺口比舊版（單臂預設）更需注意。修正成本高（共享 RobotWrapper 或外掛 collision checker）。

---

## N4. home-first 預設 ON → 啟動兩臂自動運動 🟡

**程式碼**：[profiler L148](../ik_node/placo_ik_online_profiler_ws_mesh.py#L148)（`default=True`）→ `run()` 內 `_startup_sync()` 後送 home。
裸指令一啟動就把兩臂送 home（forward 模式 `send_home_fwd` 或 traj `send_home_confirmed`）。
**風險**：若機械臂周邊未淨空 / TF 未就緒，啟動瞬間運動需人留意。**建議**：實機首次啟動前確認淨空，或用 `--no-home-first`（[L150](../ik_node/placo_ik_online_profiler_ws_mesh.py#L150)）關閉。

---

## 7 / 8. workspace clamp 相關（預設 OFF，僅 `--ws-clamp` 時相關）🟢

裸指令 ws_clamp 預設 OFF（[L152](../ik_node/placo_ik_online_profiler_ws_mesh.py#L152)），以下僅在加 `--ws-clamp` 時生效：
- **#7**：[L1049](../ik_node/placo_ik_node.py#L1049) 只 clamp xyz，方向不限 → 邊界處 wrist wobble。
- **#8**：clamp 基準是意圖位置（`base+delta`）非 TF → 配 #6 漂移時操作者「推不動仍累積」。
**修正**：ws_mesh 存 orient cone；clamp 基準改 TF。

---

## 11. jump guard 是 deg/step → 跟 rate 耦合 🟢

[profiler L132](../ik_node/placo_ik_online_profiler_ws_mesh.py#L132) `--joint-jump-guard-deg`。10° 在 50Hz=500°/s，在 20Hz=200°/s，切 rate 後物理含義變。**修正**：改 `rad/s`，內部 `guard_per_step = rad_per_s / rate_hz`。

---

## 14. `_filt_joints` 讀寫未全納鎖 🟢

[placo_ik_node.py:845-858](../ik_node/placo_ik_node.py#L845-L858) `_filter_joints` 在無鎖區讀寫 `self._filt_joints`；[L690/799/811](../ik_node/placo_ik_node.py#L690) 重置也未鎖。實際單寫多讀、靠 GIL，torn-read 風險小但形式不嚴謹。**修正**：全納 `_joints_lock`。

---

## 15. reanchor 事件未進 CSV 🟢

[placo_ik_node.py:1036](../ik_node/placo_ik_node.py#L1036) reanchor/首幀 `return None` → 跳過 `_write_step` → CSV 看不出 session 邊界。**修正**：reanchor 寫一行 `event="reanchor"`。

---

## N2. argparse `store_true default=True` 正向 flag 失效 🟢

[L148/152/166](../ik_node/placo_ik_online_profiler_ws_mesh.py#L148) 三個旗標用 `action="store_true", default=True`：
```python
p.add_argument("--home-first",   action="store_true", default=True, ...)  # 永遠 True
p.add_argument("--no-home-first", action="store_false", dest="home_first")  # 這個才有效
```
→ 正向 flag（`--home-first`/`--no-ws-clamp`/`--no-j3j4-couple`）是 **no-op**（不寫也 True）；只有反向旗標（`--no-home-first`/`--ws-clamp`/`--j3j4-couple`）能改值。功能可用，但語意混亂、易誤解。**修正**：正向旗標改 `default=False` 或移除（保留反向即可）。

---

## 整合建議：優先級路線（2026-05-29）

**P0（影響感知、低成本）**
1. **N1** — 雙臂無 config 時 `sys.exit` 或預設改回 right（避免左臂反向）
2. #2 — IK 成功也用 FK 寫方向（或一律從 TF 寫回，順修 #6）
3. #5 — guard clamp 改統一 scale（現在主要救 wrist 姿態）
4. #10 — reanchor 用 blocking TF
5. #15 — reanchor 寫 CSV 標記
6. N3 — 對齊 help 字串與真實 default

**P1（穩定性、中成本）**
7. #1 — callback 端 delta 累加 / publisher rate 對齊（arm cap 變小後更迫切）
8. #6 — `_pose` 改用 TF 寫回（順修 #8）
9. #11 — guard 改 rad/s

**P2（高階體驗、高成本）**
10. #3 — bimanual.yaml 改統一 α 或 LPF 移 task-space
11. #9 — 延遲補償
12. #12 — VR 端失敗回饋
13. #13 — bimanual collision（預設雙臂後升級）

**雜項**：N2、N4（文件/旗標清理）、#14（鎖）

---

## 量化驗證方法（資料皆在現行 CSV，免改碼）

1. **#1 / #6 漂移**：畫 `(tf_x,y,z) − (x,y,z)` 時序，連續多 frame >10mm 即確認
2. **#4 已修驗證**：σ_min<0.05 段看 `iterations` 是否升到 ~20、`pos_err_mm` 是否 <10（修好應收斂）
3. **#5 wrist clamp**：`joint_jump_guard==1` 的 frame 看是哪幾軸（應集中 j5-7）+ `ori_err_deg` 是否變大
4. **#3 弧線（載 config 時）**：選斜向位移段畫 `tf_(x,z)` 是否成弧
5. **N1 左臂方向**：左臂 CSV 的 `dx` 方向 vs `tf` 實際移動方向是否反

---

## 不在此檔範圍

- placo solver 內部演算法（見 [`placo_solver_analysis.md`](./placo_solver_analysis.md)）
- DLS 調參（見 [`adaptive_dls.md`](./adaptive_dls.md)）
- j3 奇異點本身（見 [`j3_singularity_weight_issues.md`](./j3_singularity_weight_issues.md)）
- 已修的 session-start gap timing（reanchor sentinel，[placo_ik_node.py 約 L1000-1036](../ik_node/placo_ik_node.py#L1000-L1036)）
- 已修的 `_MAX_ITER` 奇異點收斂（adaptive max_iter，session L312-314）

本檔聚焦：**子系統都正常運作、整合進真實 VR teleop（2026-05-29 預設組態）後仍會看到的行為缺陷**。
