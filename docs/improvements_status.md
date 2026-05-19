# Improvements Status — A–F 改進進度總表

> 兩組 A–F 改進建議的實作狀態，跟對應的程式位置 + CLI flag。
> 最後更新：2026-05-19

---

## 目錄

1. [Set 1 — Jitter / 抖動改進 (A–G)](#set-1--jitter--抖動改進-ag)
2. [Set 2 — Boundary / Singularity / Saturation (A–F)](#set-2--boundary--singularity--saturation-af)
3. [整體 Pipeline 全圖](#整體-pipeline-全圖)
4. [尚未完成的優先序](#尚未完成的優先序)
5. [CLI flags 全表](#cli-flags-全表)

---

## Set 1 — Jitter / 抖動改進 (A–G)

> 對應原始分析：**duration 200ms/100ms/60ms 旋轉低頻抖** vs **duration 1ms wrist 高頻抖**

| ID | 項目 | 狀態 | 位置 / CLI | 備註 |
|---|---|---|---|---|
| **A** | 量測：per-joint cmd 入 CSV + 7-joint 時序圖 | ⚠️ 部分 | CSV 含 `pos_err_mm`, `track_err_mm`, `iterations`, `iter_ms`, `sigma_min`, `lambda_dls`；但**沒有 per-joint cmd 7 欄** | 看 IK 內部夠用，但 motor 端要鎖頻譜還缺 |
| **B** | JT 模式調參：rate=50/100, horizon = period × 1.2 | ✅ 完成 | CLI `--rate 100 --horizon 1.0` 已調 | 目前 rate=100Hz dt=10ms |
| **C** | 輸入端濾波：tracker quat SLERP LPF | ❌ 移除 | （曾實作 `QuaternionEmaFilter` / `_q_slerp`，後來移除 — 被 wrist vel cap + DLS 取代）| 若仍需可重新加 flag |
| **D** | 輸出端濾波：joint cmd 1st-order LPF | ✅ 完成 | `placo_ik_node.py::_filter_joints()` / CLI `--lpf-alpha` | 支援 single 或 per-joint 7 值 |
| **E** | 跳變偵測：`MAX_JOINT_DELTA_DEG` 拒絕解 | ❌ 移除 | （CSV 2026-05-15 留有 `max_joint_delta_deg`/`joint_jump_guard` 欄位，code 已移除） | 被 wrist vel cap 取代（QP 硬約束更早就擋下）|
| **F** | forward_position_controller 模式 | ❌ 未做 | — | 仍用 `right_joint_trajectory_controller`（action）|
| **G** | IK weight tuning | ✅ 完成 | `placo_ik_session.py`：`_W_ORI 0.3→2.0`、`_W_JOINTS 1e-4→5e-4`、`_W_REG 1e-5→6e-5`、`_MAX_ITER 5→15` | 大幅加重 orientation 任務以壓 wrist null-space 漂 |

---

## Set 2 — Boundary / Singularity / Saturation (A–F)

> 對應原始分析：**teleop 撞 workspace 邊界時手臂「凍住」感**

| ID | 項目 | 狀態 | 位置 / CLI | 備註 |
|---|---|---|---|---|
| **A** | Soft clamp（軟邊界 / 彈性場）取代 hard snap | ✅ 完成 | `ik_node/ws_boundary.py::SoftClamp` / CLI `--boundary-margin 0.05` | 支援 ws_mesh + box 兩種底層；徑向 damping 取代離散 voxel snap |
| **B** | Decoupled position/orientation handling | ❌ 未做 | — | 目前仍是單一 QP；若再有「位置已到、旋轉卡 wrist」問題再做 |
| **C** | Adaptive DLS damping（奇異點 λ 動態調整）| ✅ 完成 | `placo_ik_session.py`：`_DLS_LAMBDA_BASE/MAX/SIGMA_THRESH` + Jacobian SVD 每步算 σ_min | CSV 新增 `sigma_min`、`lambda_dls` 兩欄 |
| **D** | Velocity-limited target 取代 success-freeze | ❌ 未做 | `placo_ik_node.py:637` 仍是 `if r["success"]: publish` | 目前靠 success guard 防止壞指令；改 D 後手臂會「持續逼近但走不到」更直覺，但要先確認 vel_limits 行為 |
| **E** | OOR / boundary distance 反饋 topic | ✅ 完成 | `ik_node/ws_boundary.py::BoundaryMonitor` 發 `/{arm}/boundary_dist_mm` (Float32) + `/{arm}/boundary_state` (JSON String) | VR app 訂閱即可加震動 / 顏色反饋 |
| **F** | Stuck detection + ref auto-resync | ❌ 未做 | — | 目前只靠 `_ee_delta_gap_sec=0.35s` 斷線重設；OOR 持續推但未斷線會累積 delta |

---

## 額外完成（不在原始 A–F 之內）

| 項目 | 位置 / CLI | 說明 |
|---|---|---|
| **Wrist velocity cap** | `placo_ik_session.py::_WRIST_VEL_CAP` / CLI `--wrist-vel-cap 4.0` | URDF 20.94 rad/s 太鬆 → 覆寫到 4.0 rad/s。詳見 [wrist_vel_cap.md](wrist_vel_cap.md) |
| **TfPoller (Fix-1)** | `placo_ik_node.py::TfPoller` | 背景 50ms TF 快取執行緒，消除主 loop 0–300ms TF 阻塞 |
| **AsyncCsvWriter (Fix-3)** | `placo_ik_node.py::AsyncCsvWriter` | 背景 CSV queue 寫入，主 loop 0 I/O 等待 |
| **Velocity limits 啟用 (Fix-2)** | `placo_ik_session.py:147` `enable_velocity_limits(True)` + `dt=1/rate_hz` | 從原本 False 改 True + dt 對齊實際週期 |
| **Always update seed** | `placo_ik_node.py:621-622` | `_last_joints = r["joints"]` 不再 gate by success，防止 left-arm cascade failure |

---

## 整體 Pipeline 全圖

```
VR Tracker (40 Hz)
   ↓                                                  ← Set1-C input LPF：❌ 移除
 PoseStamped /ee_delta/{arm}
   ↓
─────────────────────────────────────────────────────────
 IK Node (placo_ik_node.py) Main Loop @ rate_hz
   ↓
 _ee_delta_cb → pending msg
   ↓
 Compute dx_arm = rotate_vec(δ_xyz, calib_q)
   ↓
 [SoftClamp.apply()]                                  ← Set2-A ✅
   ├── ws_mesh 或 box 邊界
   ├── 軟 damping (margin 內)
   └── BoundaryState(distance, outside, since_outside_ms)
   ↓
 [BoundaryMonitor.publish()]                          ← Set2-E ✅
   ├── /{arm}/boundary_dist_mm  (Float32)
   └── /{arm}/boundary_state    (JSON String)
   ↓
 target_xyz, target_R
   ↓
 [PlacoSession.solve_step()]
   ├── enable_joint_limits=True   ← URDF + post-clip
   ├── enable_velocity_limits=True ← URDF + 【wrist 覆寫 4.0】 ← Wrist cap ✅
   ├── Adaptive DLS λ              ← SVD(J) σ_min → λ 動態  ← Set2-C ✅
   ├── pos_task  W=1.0
   ├── ori_task  W=2.0             ← Set1-G tune ✅
   ├── jt_task   W=5e-4            ← Set1-G tune ✅
   ├── max_iter  15                ← Set1-G tune ✅
   └── pos_err < POS_RELAX=10mm → success
   ↓
 _last_joints = r["joints"]   ← always update (no success gate)
   ↓
 if r["success"]: ←━━━━━━━━━━━━━━ Set2-D success-gate freeze ❌ 仍在
   ↓
 [_filter_joints()]                                   ← Set1-D ✅
   └── per-joint LPF (α default 1.0 = off)
   ↓
 _publish(JointTrajectory, time_from_start=horizon_ms)
   ↓
─────────────────────────────────────────────────────────
 ros2_control @ 100Hz
   ↓
 right_joint_trajectory_controller (action) ← Set1-F forward_position_controller 未試 ❌
   ↓
 Hardware (DC motors)
```

---

## 尚未完成的優先序

| 優先 | 項目 | 估計收益 | 風險 | 估計工時 |
|---|---|---|---|---|
| 🔴 高 | **Set2-D**：移除 success-freeze gate | 消除最後一種卡頓來源；OOR/邊界時手臂「逼近但走不到」比凍結直覺 | 中（先驗證 vel_limits 真的會擋住極端解）| 1–2h |
| 🟡 中 | **Set2-F**：stuck-at-boundary detection + auto resync ref | 防止 "return snap" 模式 B；連續 OOR 推時自動重設 ref | 低 | 1h |
| 🟡 中 | **Set1-F**：forward_position_controller 切換 | 跳過 JT 插值，看是否真的是 JT 插值造成 200ms 旋轉抖 | 中（無 JTC 平滑，必須配合 output LPF）| 2h |
| 🟢 低 | **Set1-A 補完**：CSV 加 7-joint cmd 欄位 + plot per-joint 時序 | 直接看 motor 端頻譜，分辨 IK 抖 vs JTC 抖 vs 馬達抖 | 0 | 30min |
| 🟢 低 | **Set1-C 重做**：tracker quat input LPF (帶 CLI flag) | 從源頭壓 VR rotation 雜訊；目前是靠下游機制間接消化 | 低 | 1h |
| 🟢 低 | **Set2-B**：decoupled pos/ori task | 治「位置已到但 wrist 還在猶豫」的特殊 case；目前 _W_ORI=2.0 應該已大幅減緩 | 中（架構改動較大）| 3h |

---

## CLI flags 全表

| Flag | 預設 | 控制機制 | 對應 |
|---|---|---|---|
| `--arm` | right | 選臂 | — |
| `--rate` | 100 | 控制週期 (Hz) | Set1-B |
| `--horizon` | 1.0 | JT duration (ms) | Set1-B |
| `--max-iter` | _MAX_ITER (15) | IK 迭代上限 | Set1-G |
| `--rebuild` | False | 每步 rebuild RobotWrapper | — |
| `--no-vel-limits` | False | 關閉 QP 速度約束（不建議） | — |
| `--wrist-vel-cap` | 4.0 | Wrist QP 速度硬約束 (rad/s) | Extra |
| `--lpf-alpha` | 1.0 | 輸出端 joint cmd LPF | Set1-D |
| `--boundary-margin` | 0.05 | Soft clamp damping 起始距離 (m) | Set2-A |
| `--ws-mesh` | (auto) | WorkspaceMesh .npz 路徑 | Set2-A 底層 |
| `--no-ws-clamp` | False | 完全停用 workspace clamp | — |
| `--dry-run` | False | 算 IK 但不送軌跡 | — |
| `--keyboard` | False | KEYBOARD 模式啟動 | — |
| `--home-first` | False | 啟動先 home + TF confirm | — |
| `--calib-yaw` / `--calib-rpy` | 0 | Tracker → arm frame 校正 | — |
| `--verbose` | False | 每步印（vs 每 5 步）| — |

---

## ROS Topics（IK 端 publish）

| Topic | Type | 來源 | 對應 |
|---|---|---|---|
| `/{arm}_joint_trajectory_controller/joint_trajectory` | trajectory_msgs/JointTrajectory | `_publish()` | — |
| `/{arm}/delta_ik_latency_ms` | std_msgs/Float32 | `_latency_pub` | profiling |
| `/{arm}/placo_profile` | std_msgs/String (JSON) | `_profile_pub` | profiling |
| `/{arm}/boundary_dist_mm` | std_msgs/Float32 | `BoundaryMonitor` | Set2-E ✅ |
| `/{arm}/boundary_state` | std_msgs/String (JSON) | `BoundaryMonitor` | Set2-E ✅ |

---

## 相關 docs（同目錄）

- [refactor_jitter_fix.md](refactor_jitter_fix.md) — Fix-1/2/3 三大根因（TfPoller / vel_limits / AsyncCsvWriter）
- [vr_realtime_ik_analysis.md](vr_realtime_ik_analysis.md) — VR 即時控制 IK 4 大指標對齊度評估
- [adaptive_dls.md](adaptive_dls.md) — Set2-C 詳細設計
- [wrist_vel_cap.md](wrist_vel_cap.md) — Wrist velocity cap 詳細設計
- [placo_solver_analysis.md](placo_solver_analysis.md) — Placo IK 架構分析
- [ik_solver_weights.md](ik_solver_weights.md) — 4 種 solver 的權重設計
- [ws_mesh_tools.md](ws_mesh_tools.md) — WorkspaceMesh 工具集

---

## 推薦下一步

**做完 Set2-D (移除 success-gate) + Set2-F (stuck detection)** 應該能徹底解決你說的「邊界卡頓」感。

之後如果 wrist 旋轉抖動仍有殘餘：
1. 先把 **Set1-A 的 per-joint CSV** 補上，量化抖動頻譜
2. 若是 JTC 插值問題 → 試 **Set1-F (forward_position_controller)**
3. 若是 tracker 源頭問題 → 重做 **Set1-C (input LPF)**
