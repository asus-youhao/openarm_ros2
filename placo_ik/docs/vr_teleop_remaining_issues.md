> ⚠ **已過時（2026-05-27 版）— 請改看 [`vr_teleop_issues_20260529.md`](./vr_teleop_issues_20260529.md)**
> 本檔的行號、嚴重度與 #3/#4/#11 結論基於舊程式碼狀態（`_MAX_ITER=5` 無 adaptive boost、jump_guard=15、λ_max=1e-2、per-joint LPF 不一致、單右臂預設）。
> #4 已修（adaptive max_iter）、#3 預設已緩解（LPF 統一 0.25）、預設已改為雙臂等變更見新檔。保留本檔僅供歷史對照。

# Placo IK 應用於 VR Teleop — 剩餘問題深度分析

**對象**：[`placo_ik/ik_node/placo_ik_node.py`](../ik_node/placo_ik_node.py) + [`placo_ik/ik_node/placo_ik_session.py`](../ik_node/placo_ik_session.py) + [`placo_ik/ik_solver/placo_ik_solver.py`](../ik_solver/placo_ik_solver.py)
**互補對象**：本檔接續 [`vr_realtime_ik_analysis.md`](./vr_realtime_ik_analysis.md) — 那邊已詳述「達成了什麼」，本檔只列「**還沒解決、目前仍會造成 VR 體驗劣化**」的具體問題，並標註是否為 blocker。
**最近一次修正基準**：reanchor sentinel（publisher → IK 明示 session-start，避免靠 gap timing 推測）已合併進 [placo_ik_node.py:951-988](../ik_node/placo_ik_node.py#L951-L988)。本檔之後的所有「session 邊界」相關討論假設此 fix 已套用。

---

## 嚴重度速覽

| #  | 問題 | 嚴重度 | 在 VR 中的徵兆 | 修正成本 |
|----|-----|-------|---------------|---------|
| 1  | `_pending` 覆蓋 → 訊息聚合錯誤 | 🔴 高 | 操作者覺得「手快移動時手臂跟不上」、Δ 累積不到位 | 中 |
| 2  | `target_R` 在 IK 失敗時繼續推進，`base_q` 漂移 | 🔴 高 | 鬆開 trigger 後再按下，方向感不對 | 低 |
| 3  | LPF 各關節 α 不一 → 在動態時 EE 軌跡偏離 IK 解 | 🟠 中-高 | EE 在斜向位移時走出「歪斜的弧」，非直線 | 低（須權衡 jitter） |
| 4  | `_MAX_ITER=5` 在 σ_min→0 + λ 拉到 1e-2 時不夠收斂 | 🟠 中-高 | 進入奇異點時手臂「黏住」、無視小幅移動 | 低 |
| 5  | 關節跳變 guard 獨立 clamp → 破壞 IK 一致性 | 🟠 中 | jump guard 命中時 EE 軌跡瞬間偏離指令 | 中 |
| 6  | `_pose` 為意圖姿態而非閉迴路 → vel-limit 飽和時持續積壓 | 🟠 中 | 速度限制下手臂落後，鬆手後位置跳到實際 TF | 中-高 |
| 7  | 方向不受 workspace clamp 限制 → 邊界處方向奇異 | 🟡 中-低 | 邊界處手腕翻轉、wrist wobble | 中 |
| 8  | SoftClamp 用意圖位置 `_pose+delta`，不用 TF | 🟡 中-低 | 邊界外推時操作手可以「無限延伸」 | 低 |
| 9  | tracker 訊息 header.stamp 已記錄但沒做延遲補償 | 🟡 中-低 | 整體系統 ~50-80ms 延遲不會自我感知 | 中-高 |
| 10 | TF poller cache 在 reanchor 時最舊可達 ~75 ms | 🟡 中-低 | session 啟動瞬間有 1-3 mm 位置誤差 | 低 |
| 11 | jump guard 閾值是 deg/step（非 rad/s）→ 跟 rate 耦合 | 🟢 低 | 切換 rate 後相同 deg 變得太鬆/太緊 | 低 |
| 12 | 沒有 VR 端的失敗回饋（震動/視覺） | 🟡 中 | 操作者在 IK fail/guard hit 時毫無感知 | 中 |
| 13 | bimanual 兩臂無 collision/coupling 感知 | 🟢 低 | 兩臂可互撞 | 高 |
| 14 | `_joints_lock` 使用不對稱、warm-start seed 讀取未鎖 | 🟢 低 | 極罕見的 torn-read（非觀察到的 bug） | 低 |
| 15 | reanchor 事件未進 CSV → 事後切片困難 | 🟢 低 | 分析時看不出 session 邊界 | 低 |

---

## 1. `_pending` 覆蓋 → 訊息聚合錯誤 🔴

**程式碼**：[placo_ik_node.py:416-428](../ik_node/placo_ik_node.py#L416-L428)
```python
def _ee_delta_cb(self, msg: PoseStamped):
    ...
    with self._delta_lock:
        self._pending        = msg          # ← 直接覆蓋
        self._pending_t_recv = t_recv
```

**問題**：
publisher 端通常在 40-60 Hz（XRoboToolkit 預設 `pub_hz=40`，可拉到 60），IK 端常用 20-50 Hz。
當 publisher > IK rate 時，每個 IK tick 之間會到 2-3 筆訊息 — **只有最後一筆會被處理，前面的 delta 全部丟掉**。

但**這些 delta 是相對於 publisher 端的 `ref_pos`**（單次 trigger session 不變），不是 incremental，所以「丟掉中間幾筆」表面上是對的（用最新的就好）。

**真正的 bug** 在於 **rate-limit + dead-zone**（[side_state.py:236-238](../../../XRoboToolkit-Teleop-ROS/tracker_pose/controller_ee_delta_v1.0/side_state.py#L236)）：
publisher 用 `_pub_interval = 1/pub_hz` 做 rate limit，但**只在 `exceeds` 突破 dead-zone 時才發**。
快速移動時 dead-zone 連續突破 → publisher 用滿 pub_hz；緩慢移動時可能 0.5-1s 才發一筆。
IK 端 fetch 每個 tick 必拉最新一筆，**等於把「publisher 沉默期間操作者的全部位移」累積到一筆 large delta 上一次性處理**，造成：
- 操作者覺得「我輕輕移就馬上停，怎麼手臂還在動？」
- 一次 IK 收到大 delta → vel_limits saturate → 多 frame 才追到 → 視覺上是 lag
- 若這時剛好 guard_hit，arm 進入「clamp 慢爬」模式

**為何沒人發現**：profiler 看到的 `loop_ms`、`pos_err` 都正常；現象只在「快速短促位移」時出現，且只能用閉迴路 TF vs target 的時間序列看出。

**建議修正**：
- 短期：在 callback 端**累加** position delta（quaternion 取最新即可），IK tick 一次處理累加值
- 中期：把 publisher 的 dead-zone 改成「on transition」而非「on accumulated」，讓訊息穩定 ~50 Hz 流入
- 長期：IK rate 跟 publisher rate 對齊（例如雙方都 60 Hz）

---

## 2. `target_R` 漂移：IK 失敗時 base_q 持續推進 🔴

**程式碼**：[placo_ik_node.py:1090-1107](../ik_node/placo_ik_node.py#L1090-L1107)
```python
if r["success"]:
    nx, ny, nz = target["new_x"], target["new_y"], target["new_z"]
    self._pose = [nx, ny, nz, new_q[0], new_q[1], new_q[2], new_q[3]]  # ← 全部更新
else:
    ee = r["ee_xyz"]
    self._pose[0] = float(ee[0])    # ← 只更新位置
    self._pose[1] = float(ee[1])
    self._pose[2] = float(ee[2])
    # ↑ 沒寫 self._pose[3:7]！
```

**問題**：
- success=1 → `_pose[3:7]` 寫成 `new_q`（target 方向，**不是實際 FK 方向**）
- success=0 → `_pose[3:7]` 完全沒更新 → 上一輪的 target_q 殘留

下個 frame 計算 `base_q = self._ee_delta_ref_q` 仍是 session ref 沒問題；但 **連續多次 IK 失敗**期間：
- 操作者的 delta_q 繼續累積在 `base_q` 之上 → 計算的 target_R 持續推進
- 實際 EE 旋轉因 vel_limit / DLS 阻尼跟不上
- 操作者鬆手 → publisher gate → IK 收到 reanchor sentinel → `_pose = TF`（含實際方向）→ **下一次按下時 ref 是 TF，但操作者腦中的「初始方向」是上次鬆手前的視覺方向**
- 結果：第一次重新按下後，操作者覺得「方向錯了」、手臂往不對的方向轉

**修正**：success 分支也應該寫實際 FK 的 q，而非 target q
```python
if r["success"]:
    nx, ny, nz = target["new_x"], target["new_y"], target["new_z"]
    # 從 FK 取實際 q（placo session 已有 T_ee）
    R = r.get("ee_R")            # 需要 session 多回傳一個欄位
    qx, qy, qz, qw = _rot_to_quat(R) if R is not None else new_q
    self._pose = [nx, ny, nz, qx, qy, qz, qw]
```
或乾脆**只信 TF**：每個 success step 都從 `_tf_poller.get()` 讀 FK 寫回 `_pose`。這也順便修了 issue #6（開迴路漂移）。

---

## 3. 各關節 LPF α 不一 → 動態時 EE 偏離 🟠

**程式碼**：[placo_ik_online_profiler_ws_mesh.py:111-117](../ik_node/placo_ik_online_profiler_ws_mesh.py#L111-L117) 預設
```
--lpf-alpha 0.25,0.25,0.25,0.4,0.6,0.6,0.6
```
意思：肩膀三軸用 25% follow（強平滑、滯後）；肘 40%；腕 60%（弱平滑）。

**問題**：
IK 解出來的 7 個關節是**耦合的** — 在 task-space 等價於一個 Jacobian 同時動。把每軸獨立打不同 α，等於**人為地把同步動作打散到不同時刻**：
- 肩膀拖到後面才動 → 在前 30 ms 內，腕指方向已轉了但肘還沒就位 → EE 走出**歪斜的弧**而非直線
- 操作者期待「跟著我手」，但實際 EE 軌跡跟 IK 給出的 q 軌跡都對不上
- 在 task-space 看是 **systematic offset during motion**，靜止時又消失 → 操作者形容「移動時手臂有點漂、停下來就準了」

**為什麼這樣設計**：腕關節是高頻 noise 來源（IK 解微擾被放大），所以給它弱平滑（α 大）讓 noise 過；肩膀大馬達慣性大，給強平滑減震。**邏輯是對的，副作用沒被檢驗**。

**修正方向**（任一）：
- (A) 把 LPF 移到 task-space（在 target_xyz / target_R 端 LPF），不要在關節端
- (B) 統一 α，靠 wrist_vel_cap + DLS 抑制 wrist noise（已有，可調強）
- (C) 維持現狀但量化副作用：用 CSV `q_cmd_*` vs `tf_x/y/z` 量化動態誤差，決定權衡

---

## 4. `_MAX_ITER=5` 在重度奇異點時不夠 🟠

**程式碼**：[placo_ik_session.py:54](../ik_node/placo_ik_session.py#L54)
```python
_MAX_ITER = 5
```
搭配 Adaptive DLS（[placo_ik_session.py:56-62](../ik_node/placo_ik_session.py#L56-L62)）：σ_min → 0 時 λ 從 6e-5 拉到 ~1e-2，IK 變「重阻尼小步走」。

**問題**：
λ 拉到 1e-2 時，每個 iter 走的步長被打到約原本的 1/10。5 iters 在非奇異區可能 1-2 iter 就收斂（POS_TOL=3mm），但在 σ_min < 0.05 的深度奇異時，**5 iters 走的距離可能不到 1mm**，pos_err 卡在 10-30 mm。

結果：
- `success = pos_err < POS_RELAX(10mm)` → 0
- Set2-D 模式繼續發 partial solution → arm 慢慢爬，但每 frame 都 fail
- 操作者覺得手臂「黏在那」、推不動

**驗證方法**：CSV `sigma_min` 低於 0.05 的段，看 `iterations` 是不是頂滿 5 + `pos_err_mm` 是不是 > 10。

**修正**：σ_min 低時動態提高 max_iter
```python
max_iter_dyn = self._max_iter if sigma_min > _DLS_SIGMA_THRESH else self._max_iter * 4
for _ in range(max_iter_dyn): ...
```
代價：奇異點時 `loop_ms` 從 ~0.5ms 升到 ~2ms（20 iter × 0.1ms），仍遠低於 20ms deadline。

---

## 5. Jump guard 獨立 clamp 破壞 IK 一致性 🟠

**程式碼**：[placo_ik_node.py:1064-1071](../ik_node/placo_ik_node.py#L1064-L1071)
```python
guard_rad = math.radians(self._joint_jump_guard_deg)
joints_clamped = [
    max(s - guard_rad, min(s + guard_rad, j))   # ← 每軸獨立 clip
    for s, j in zip(seed, r["joints"])
]
```

**問題**：
IK 解 `r["joints"]` 是 7 個關節的**配對**（共同達成 EE target）。若 j1 想動 +20°、j3 想動 -5°、其他在 ±15° 內 → 只有 j1 被 clamp 到 +15°。
**clamp 後的 q 不再對應任何合理的 EE pose** — 是「j1=+15 + j3=-5 + 其他不變」的 FK 結果，跟 target 完全無關。
結果：
- EE 跑到一個操作者沒指定的位置
- 連續多 frame 命中 guard → arm 沿一條沒人指定的軌跡爬行
- 操作者覺得「明明往前推、手臂卻偏向旁邊」

**修正**：用**統一 scale**保持方向
```python
delta_q = [j - s for s, j in zip(seed, r["joints"])]
max_abs = max(abs(d) for d in delta_q)
if max_abs > guard_rad:
    scale = guard_rad / max_abs
    joints_clamped = [s + d * scale for s, d in zip(seed, delta_q)]
else:
    joints_clamped = list(r["joints"])
```
這樣 EE 跟著 IK 解的方向走，只是每步走得更短。

---

## 6. `_pose` 開迴路 → vel-limit 飽和時持續積壓 🟠

**程式碼**：[placo_ik_node.py:973-974](../ik_node/placo_ik_node.py#L973), [1090-1107](../ik_node/placo_ik_node.py#L1090-L1107)
```python
base_xyz = self._ee_delta_ref_xyz or tuple(self._pose[:3])
...
self._pose[0] = float(ee[0])    # IK FK 結果（不是 TF 實際）
```

**問題**：
`_pose` 在 success 時 = target，fail 時 = IK 內部 FK。**兩者都不是實際 TF**。實際機械手因 wrist_vel_cap (~4 rad/s) 和 joint vel_limits 飽和時，TF 會落後 IK 內部 FK：
- 操作者推手 5 cm
- IK 算出 target，但每 step 只能走 1 cm（vel limit）
- 5 frame 後 IK 內部認為手臂走了 5 cm；TF 上實際走了 5 cm（沒問題）...
- **但若 publisher 累加 delta 又進來 3 cm**（issue #1），IK 內部以為已經到 5 cm，target 變成 8 cm。實際 TF 只走到 5 cm
- 後續 base_xyz = `_pose[:3]` = IK 內部位置 = 5 cm（不是 TF 的 5cm）→ 短期內勉強重合
- 但只要再有 IK fail 或 saturate，**內部 `_pose` 和 TF 的偏差會累積**
- 操作者鬆手 → reanchor sentinel → `_pose = TF`（突然回到實際位置）→ **視覺上手臂位置會跳**（或下次按下時感覺 ref 不對）

**修正**：每個 success step 都從 TF 寫回 `_pose`（替代 IK FK）。代價：TF cache 最多 75 ms 舊（issue #10），但只要 publisher rate 與 IK rate 對齊，此延遲可吸收。

---

## 7. 方向不受 workspace clamp 限制 🟡

**程式碼**：[placo_ik_node.py:1000-1004](../ik_node/placo_ik_node.py#L1000-L1004)
```python
if self._ws_clamp or self._ws_mesh is not None:
    new_xyz_arr, _bs = self._soft_clamp.apply(raw_xyz_arr, dx_arm_arr)
    # ↑ 只 clamp xyz，沒處理 orientation
```

**問題**：
邊界處工作空間有效但**腕關節在該位置可能逼近奇異**（如 j5/j7 接近 ±90° 或對齊）。SoftClamp 把位置鎖在可達區內，但操作者持續扭轉手腕 → target_R 帶 EE 進入該位置的方向死角 → wrist wobble、IK 收斂變慢。

**修正方向**：
- 在 ws_mesh 裡也存 reachable orient cone，clamp 時順便把 target_R 投影回該 cone
- 或在 `solve_step` 偵測 σ_min<thresh 時，降低 W_ORI（允許方向妥協換位置正確）

---

## 8. SoftClamp 基準是意圖位置而非 TF 🟡

**程式碼**：[placo_ik_node.py:996-1001](../ik_node/placo_ik_node.py#L996-L1001)
```python
raw_xyz_arr = np.array([base_xyz[0] + dx_arm[0], ...])   # base = _pose 或 ref
new_xyz_arr, _bs = self._soft_clamp.apply(raw_xyz_arr, dx_arm_arr)
```

**問題**：
clamp 是對「IK 想要去哪」做投影，不是對「機械手已經在哪」做投影。配合 issue #6（`_pose` 開迴路漂移），當 arm 被 vel limit 卡住時：
- `_pose` 一直加 delta，逼近 ws 邊界
- SoftClamp 開始 damping → 推不動
- 操作者繼續往外推 → SoftClamp 持續 damping，內部 `_pose` 卡在邊界
- 操作者沒感覺到 arm 已經停了，繼續累積腦中的位置 → 鬆手後感受到「歸位偏移」

**修正**：clamp 基準改為 `tf_poller.get()`，這樣 clamp 是對實際位置生效，操作者「手在哪推就在哪止」。

---

## 9. tracker 訊息延遲沒做補償 🟡

**程式碼**：[placo_ik_node.py:81-82, 1240-1244](../ik_node/placo_ik_node.py#L81-L82)
```python
"tracker_t_stamp", "tracker_t_recv",   # ← 已記錄
...
tracker_t_stamp = float(s.sec) + float(s.nanosec) * 1e-9
```
**有記錄、但沒用**。

**問題**：
總延遲鏈：
| 階段 | 估計 | 累積 |
|---|---|---|
| VR controller → XR client | ~5-15 ms | 15 ms |
| XR client → publisher → ROS | ~5-10 ms | 25 ms |
| ROS → IK callback | ~1-3 ms | 28 ms |
| IK loop wait + solve | ~10-25 ms | 50 ms |
| 控制器 → 馬達 | ~5-10 ms | 60 ms |
| 馬達物理響應 | ~20-50 ms | **~100 ms** |

操作者主觀感受到的延遲 ~60-100 ms。VR 一般要求 <50 ms 才不會 motion sickness 或感到「跟不上」。

**可做但未做**：用 `tracker_t_stamp` 計算 `(now - stamp)` 當作 input lag，在 target 上做**前向預測**：
```python
input_lag = t_wall - tracker_t_stamp
target_xyz += controller_velocity * input_lag    # 需要 publisher 端帶 velocity 或這邊估
```
代價：需要平滑的速度估測；過頭會 overshoot。

---

## 10. TF poller cache 在 reanchor 時最舊 ~75 ms 🟡

**程式碼**：[placo_ik_node.py:183-195](../ik_node/placo_ik_node.py#L183-L195)
```python
def _run(self) -> None:
    timeout = rclpy.duration.Duration(seconds=self._poll_sec)   # 50 ms
    while True:
        try:
            t = self._buf.lookup_transform(..., timeout=timeout)
            ...
        time.sleep(self._poll_sec * 0.5)   # 25 ms
```
平均 cache age = 12.5 ms，最壞 = 75 ms（50 ms timeout + 25 ms sleep）。

**問題**：
reanchor 時 `_pose = list(ref)` 寫入 cache 值。若機械手在那 75 ms 內仍有移動（前一個 session 鬆手後仍有殘餘運動）→ 寫入的 ref 是 75 ms 前的舊位置 → session 啟動瞬間有 1-3 mm 位置誤差，操作者輕推就「跳一小步」。

**修正**：reanchor 時不依賴 cache，直接 blocking `_get_tf(0.05)`（最多 50 ms，但只發生一次）。

---

## 11. Jump guard 是 deg/step，跟 rate 耦合 🟢

**程式碼**：[placo_ik_online_profiler_ws_mesh.py:124-127](../ik_node/placo_ik_online_profiler_ws_mesh.py#L124-L127)
```python
--joint-jump-guard-deg 15.0
```
20 Hz 時 = 300 °/s，50 Hz 時 = 750 °/s — **同樣參數安全性差 2.5×**。切換 rate 後 guard 的「物理含義」變了，但人通常忘記重調。

**修正**：改成 `--joint-jump-guard-rad-per-s`，內部換算成 per-step：
```python
guard_per_step = guard_rad_per_s / self._rate_hz
```

---

## 12. 沒有 VR 端的失敗回饋 🟡

當前 fail/guard hit 的訊號只進 console。操作者**在頭盔裡看不見 console**。

**問題**：
- IK fail streak、guard clamp、boundary hit 都靜默
- 操作者只感覺「手臂變慢/變鈍」但不知為何
- 嚴重時可能繼續用力推，加深 saturation 或 guard 抑制

**修正方向**：
- 在 IK 端發 `Bool` 或 `Float32` 狀態 topic（已有 `boundary_state` via BoundaryMonitor — 擴充）
- VR app 端訂閱，做 controller 震動或 UI 顯示

---

## 13. Bimanual 無 collision/coupling 感知 🟢

**程式碼**：[placo_ik_online_profiler_ws_mesh.py:226-249](../ik_node/placo_ik_online_profiler_ws_mesh.py#L226-L249)
每臂獨立 PlacoSession + 獨立 IK，**完全互不知道對方位置**。雙手交叉時可能撞在一起，也沒有共用 self-collision check（URDF 載入時 `placo.Flags.ignore_collisions`）。

**問題**：blocker 程度低（操作者通常會避免），但安全層面缺。修正成本高（需共享 RobotWrapper 或加 ext collision checker）。

---

## 14. `_joints_lock` 使用不對稱 🟢

**程式碼**：[placo_ik_node.py:1047-1093](../ik_node/placo_ik_node.py#L1047-L1093)
```python
with self._joints_lock:
    seed = list(self._filt_joints or self._last_joints)   # ← 鎖內讀

# ... IK 運算（無鎖） ...

with self._joints_lock:
    self._last_joints = list(joints_out)                  # ← 鎖內寫
```
讀寫分別在鎖內 — OK。**但** `_filter_joints` 在無鎖區域讀寫 `self._filt_joints`（[placo_ik_node.py:840-847](../ik_node/placo_ik_node.py#L840-L847)）。
另外 `_handle_kbd_events` 重置 `_filt_joints = None` 也沒鎖。

**問題**：
單一 IK loop thread 寫 `_filt_joints`、`_js_cb`/spin thread 不寫它 — 實際單寫多讀，torn-read 風險小。但 list 賦值的原子性靠 GIL，更新整個 list 中途 spin thread 若讀 `_filt_joints[i]` 可能拿到混合值。實務上沒觀察到 bug 但形式上不嚴謹。

**修正**：把 `_filt_joints` 的讀寫一律納入 `_joints_lock`。

---

## 15. Reanchor 事件未進 CSV 🟢

**程式碼**：[placo_ik_node.py:988](../ik_node/placo_ik_node.py#L988) — reanchor 後 `return None` → `_build_target` 返回 None → 跳過 `_write_step`。

**問題**：
- CSV 看不出 session 邊界
- 事後切片（如「每個 session 的 success rate」「session 啟動瞬間是否大跳」）只能用時間 gap 推測
- 整合 multi-session 分析時麻煩

**修正**：reanchor 時也寫一行 CSV，新增欄位 `event = "reanchor"`（其他欄位空）。

---

## 整合建議：優先級路線

**P0（影響感知體驗、低成本）**
1. Issue #2 — IK 失敗時也用 FK 寫 q（5 行）
2. Issue #4 — σ_min 低時提高 max_iter（3 行）
3. Issue #5 — guard clamp 改成統一 scale（8 行）
4. Issue #10 — reanchor 用 blocking TF（2 行）
5. Issue #15 — reanchor 寫 CSV 標記（5 行）

**P1（影響穩定性、中成本）**
6. Issue #1 — callback 端 delta 累加 / publisher rate 對齊
7. Issue #6 — `_pose` 改用 TF 寫回（同時修 #8）
8. Issue #11 — guard 改成 rad/s

**P2（影響高階體驗、高成本）**
9. Issue #3 — LPF 移到 task space（需重寫 motion pipe）
10. Issue #9 — delay compensation（需 velocity 估測）
11. Issue #12 — VR 端失敗回饋（跨 repo 改動）

**P3（規模/安全擴充）**
12. Issue #7 — orient cone clamp
13. Issue #13 — bimanual collision

---

## 量化驗證方法

要驗證以上問題是否真的影響你的工況，建議跑：

1. **issue #1 → #6**：開 CSV `tf_x/y/z` 對 `x/y/z`（`_pose`）的時序，畫 `delta_t = (TF − pose).norm()`。穩態應該 < 2mm；若連續多 frame > 10mm 就確認漂移
2. **issue #4**：CSV `sigma_min` < 0.05 的 frame，看 `iterations` 是否頂滿 5、`pos_err_mm` > 10
3. **issue #3**：選一個斜向位移段（如 +X+Z），畫 `tf_(x,z)` 是直線還是弧
4. **issue #5**：`joint_jump_guard==1` 的 frame，看 `track_err_mm`（IK target vs TF）大不大
5. **issue #9**：CSV `t - tracker_t_stamp` 的分佈

所有資料都在現行 CSV，不需要改任何程式碼就能跑出來。

---

## 不在此檔的範圍

- placo solver 內部演算法品質（已在 [`placo_solver_analysis.md`](./placo_solver_analysis.md)）
- DLS 調參（已在 [`adaptive_dls.md`](./adaptive_dls.md)）
- j3 奇異點本身（已在 [`j3_singularity_weight_issues.md`](./j3_singularity_weight_issues.md)）
- 已修正的 session-start gap timing 問題（已在新 sentinel 機制；見 [placo_ik_node.py:951-988](../ik_node/placo_ik_node.py#L951-L988)）

本檔聚焦於：**這些子系統都正常運作，整合進真實 VR teleop 後仍會看到的行為缺陷**。
