# Improvements Status — A–F 改進進度總表

> 兩組 A–F 改進建議的實作狀態，跟對應的程式位置 + CLI flag。
> 最後更新：2026-05-19（merge `feature/jitter_test3` 進 placo_ik 後）

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
| **A** | 量測：per-joint cmd 入 CSV + 7-joint 時序圖 | ⚠️ 部分 | CSV 含 `pos_err_mm`, `ori_err_deg`, `track_err_mm`, `iterations`, `iter_ms`, `sigma_min`, `lambda_dls`, `max_joint_delta_deg`, `joint_jump_guard`；但**沒有 per-joint cmd 7 欄** | 內部診斷夠用；motor 頻譜分析還缺；另有 `results/analyze_vibration.py` 補助 |
| **B** | JT 模式調參：rate=50/100, horizon = period × 1.2 | ✅ 完成 | CLI `--rate 100 --horizon 1.0` 已調 | 目前 rate=100Hz dt=10ms |
| **C** | 輸入端濾波：tracker quat SLERP LPF | ✅ 完成（merge） | `placo_ik_node.py::_qslerp` + `_filter_orientation` / CLI `--ori-lpf-alpha 0.35` | jitter_test3 重新引入 quaternion SLERP EMA；預設 α=0.35 中等平滑 |
| **D** | 輸出端濾波：joint cmd 1st-order LPF | ✅ 完成 | `placo_ik_node.py::_filter_joints()` / CLI `--lpf-alpha 0.25,0.25,0.25,0.4,0.6,0.6,0.6` | 預設 per-joint（base 強 / wrist 輕；merge 後改用 test3 預設）|
| **E** | 跳變偵測：`MAX_JOINT_DELTA_DEG` 拒絕解 | ✅ 完成（merge） | `placo_ik_node.py::_joint_jump_guard()` / CLI `--joint-jump-guard-deg 15` | 兩層防護的**反應層**；與 wrist_vel_cap（**預防層**）並存 |
| **F** | forward_position_controller 模式 | ✅ 完成（merge） | `placo_ik_node.py` dual publisher / CLI `--use-traj`（回退 JTC） | **預設改 ForwardCommandController (topic mode)**；繞過 JTC spline re-plan |
| **G** | IK weight tuning | ✅ 完成 | `placo_ik_session.py`：`_W_ORI 0.3→2.0`、`_W_JOINTS 1e-4→5e-4`、`_W_REG 1e-5→6e-5`、`_MAX_ITER 5→15` | placo_ik 值（搭配 Adaptive DLS）；jitter_test3 alternative `_W_ORI=1.0, _W_REG=1e-4, _MAX_ITER=20` 保留為註解 |

---

## Set 2 — Boundary / Singularity / Saturation (A–F)

> 對應原始分析：**teleop 撞 workspace 邊界時手臂「凍住」感**

| ID | 項目 | 狀態 | 位置 / CLI | 備註 |
|---|---|---|---|---|
| **A** | Soft clamp（軟邊界 / 彈性場）取代 hard snap | ✅ 完成 | `ik_node/ws_boundary.py::SoftClamp` / CLI `--boundary-margin 0.05` | 支援 ws_mesh + box 兩種底層；徑向 damping 取代離散 voxel snap |
| **B** | Decoupled position/orientation handling | ❌ 未做 | — | 目前仍是單一 QP；若再有「位置已到、旋轉卡 wrist」問題再做 |
| **C** | Adaptive DLS damping（奇異點 λ 動態調整）| ✅ 完成 | `placo_ik_session.py`：`_DLS_LAMBDA_BASE/MAX/SIGMA_THRESH` + Jacobian SVD 每步算 σ_min | CSV 新增 `sigma_min`、`lambda_dls` 兩欄 |
| **D** | Velocity-limited target 取代 success-freeze | ✅ 完成 | `placo_ik_node.py::run()` always publish + `_pose` 漂移防護 / CLI `--success-gate`（回退） | 詳見 [set2d_continuous_approach.md](set2d_continuous_approach.md) — 「持續逼近但走不到」取代凍結 |
| **E** | OOR / boundary distance 反饋 topic | ✅ 完成 | `ik_node/ws_boundary.py::BoundaryMonitor` 發 `/{arm}/boundary_dist_mm` (Float32) + `/{arm}/boundary_state` (JSON String) | VR app 訂閱即可加震動 / 顏色反饋 |
| **F** | Stuck detection + ref auto-resync | ⚪ 可忽略 | （`--stuck-reset-ms` flag 仍在；stash@{0} 為未完成版） | **實機改用 manual reset**：tracker start 訊號（鍵盤 `r/R`）會 reset ref。Set2-F auto-reset 測試效果不佳，**視為可忽略**；`--stuck-reset-ms 0` 即可關掉。詳見 [Manual reset workflow](#manual-reset-workflow-取代-set2-f) |

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
   ↓
 PoseStamped /ee_delta/{arm}
   ↓
─────────────────────────────────────────────────────────
 IK Node (placo_ik_node.py) Main Loop @ 100 Hz
   ↓
 _ee_delta_cb → pending msg
   ↓
 Compute dx_arm = rotate_vec(δ_xyz, calib_q)
 Compute dq_arm = rotate_quat(dq, calib_q)            ← Fix-8 (merge)
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
 [Stuck reset check]                                  ← Set2-F ⚪ 可忽略
   └── 預設仍開 (500ms)；建議 --stuck-reset-ms 0
       Manual reset via 鍵盤 'r' 取代 (見下節)
   ↓
 new_q_raw = qmul(dq_arm, base_q)
 new_q     = _qslerp(prev_q, new_q_raw, ori_lpf_α)    ← Set1-C ✅ (merge)
   ↓
 target_xyz, target_R
   ↓
 [PlacoSession.solve_step()]
   ├── enable_joint_limits=True    ← URDF + post-clip
   ├── enable_velocity_limits=True ← URDF + 【wrist override 4.0 rad/s】 ← Wrist cap (預防層)
   ├── Adaptive DLS λ              ← SVD(J) σ_min → λ 動態  ← Set2-C ✅
   ├── pos_task  W=1.0
   ├── ori_task  W=2.0             ← Set1-G tune ✅
   ├── jt_task   W=5e-4            ← Set1-G tune ✅
   ├── reg_task  W=6e-5 (baseline) ← Set1-G; DLS 動態 boost
   ├── max_iter  15                ← Set1-G tune ✅
   └── early-exit: pos_err<3mm AND ori_err<2.9° → break (orientation-aware)
   ↓
 [_joint_jump_guard()]                                ← Set1-E ✅ (merge, 反應層)
   ├── max |Δjoint| > 15° → guard_hit=True
   └── guard_hit → reject 整步（不送、不 update seed/filter/_pose）
   ↓
 [_filter_joints()]                                   ← Set1-D ✅
   └── per-joint LPF α=[0.25,0.25,0.25,0.4,0.6,0.6,0.6] (base 重 / wrist 輕)
   ↓
 _last_joints = filtered                              ← Fix-6 (merge): seed from filtered
   ↓
 publish_ok = !guard_hit && (success || !success_gate)
                            ← Set2-D continuous approach ✅
                              (_pose 漂移防護: fail 時 _pose ← r["ee_xyz"])
   ↓
 _publish(joints_out)
   ├── (預設) ForwardCommandController topic mode    ← Set1-F ✅ (merge)
   │      → /{arm}/forward_position_controller/commands (Float64MultiArray)
   └── (--use-traj) JointTrajectoryController action mode (legacy)
          → /{arm}_joint_trajectory_controller/joint_trajectory
   ↓
─────────────────────────────────────────────────────────
 ros2_control @ 100Hz
   ↓
 ForwardCommandController (topic, default)            ← 跳過 JTC spline re-plan
   ↓
 Hardware (DC motors)
```

---

## Manual reset workflow (取代 Set2-F)

實機 teleop 流程：

1. **使用者按下 tracker start**（鍵盤 `r/R`；VR app 可將 trigger 映射到此）
   - `kbd_controller.py:218` 設 `self.reset_ref = True`
   - 主迴圈下一步把 `_ee_delta_ref_xyz / _ref_q / _last_t` 全部歸 None
   - 下一幀 `is_new=True` → ref 從 TF re-anchor
2. **Tracker delta 從 0 開始累積**
3. **越過 workspace 邊界** → SoftClamp 軟阻尼；BoundaryMonitor 發 outside 訊號
4. **拉回 / 結束** → 下一段操作前必定再按 `r`，**不會 return snap**
5. (可選) 想暫停按 `p`；或停止 tracker 訊號（`_ee_delta_gap_sec=0.35s` 後自動斷線重設）

→ **Set2-F auto-reset 沒明顯價值**：人類本來就會在每段開頭按 `r`。
→ 推薦 `--stuck-reset-ms 0` 直接停用，避免 sliding-along-edge 場景被誤觸發。

---

## 尚未完成的優先序

| 優先 | 項目 | 估計收益 | 風險 | 估計工時 |
|---|---|---|---|---|
| 🟢 低 | **Set1-A 補完**：CSV 加 7-joint cmd 欄位 + plot per-joint 時序 | 直接看 motor 端頻譜（`results/analyze_vibration.py` 已部分補上）| 0 | 30min |
| 🟢 低 | **Set2-B**：decoupled pos/ori task | 治「位置已到但 wrist 還在猶豫」的特殊 case；目前 _W_ORI=2.0 已大幅減緩 | 中（架構改動較大）| 3h |
| ⚪ skip | ~~**Set2-F**：auto stuck-reset~~ | 已決定不採用 — manual reset (`r`/space) 取代 | — | — |

---

## CLI flags 全表

| Flag | 預設 | 控制機制 | 對應 |
|---|---|---|---|
| `--arm` | right | 選臂 | — |
| `--rate` | 100 | 控制週期 (Hz) | Set1-B |
| `--horizon` | 1.0 | JT duration (ms;`--use-traj` 時生效) | Set1-B |
| `--max-iter` | _MAX_ITER (15) | IK 迭代上限 | Set1-G |
| `--rebuild` | False | 每步 rebuild RobotWrapper | — |
| `--no-vel-limits` | False | 關閉 QP 速度約束（不建議） | — |
| `--wrist-vel-cap` | 4.0 | Wrist QP 速度硬約束 (rad/s) — 預防層 | Extra |
| `--lpf-alpha` | `0.25,0.25,0.25,0.4,0.6,0.6,0.6` | 輸出端 per-joint LPF（base 重 / wrist 輕） | Set1-D |
| `--ori-lpf-alpha` | 0.35 | 輸入端 SLERP EMA on target quat；1.0 = off | Set1-C (merge) |
| `--joint-jump-guard-deg` | 15.0 | 單關節單步 Δ > N° → reject 整步 — 反應層；0 = off | Set1-E (merge) |
| `--ee-delta-gap-sec` | 0.8 | tracker 斷線後重設 ref 的容忍時間 | Extra (merge) |
| `--no-rot-tracking` | False | 關閉 orientation tracking（position-only） | merge |
| `--use-traj` | False | 切回 JointTrajectoryController action（legacy） | Set1-F (merge, default = topic mode) |
| `--boundary-margin` | 0.05 | Soft clamp damping 起始距離 (m) | Set2-A |
| `--success-gate` | False | 回到舊 freeze on IK fail 行為（A/B 用） | Set2-D |
| `--stuck-reset-ms` | 500 | ~~連續 outside N ms 自動 ref reset~~ ⚪ 推薦 `0` 停用 | Set2-F (可忽略) |
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
| `/{arm}/forward_position_controller/commands` | std_msgs/Float64MultiArray | `_publish()` 預設 | Set1-F (default) |
| `/{arm}_joint_trajectory_controller/joint_trajectory` | trajectory_msgs/JointTrajectory | `_publish()` (`--use-traj`) | Set1-F legacy |
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
- [set2d_continuous_approach.md](set2d_continuous_approach.md) — Set2-D 詳細設計（移除 success-freeze gate）
- [anti_vibration_fixes_test3.md](anti_vibration_fixes_test3.md) — jitter_test3 的 8 大修正（SLERP / jump guard / forward_pos_controller / IK seed tracking 等）
- [placo_solver_analysis.md](placo_solver_analysis.md) — Placo IK 架構分析
- [ik_solver_weights.md](ik_solver_weights.md) — 4 種 solver 的權重設計
- [ws_mesh_tools.md](ws_mesh_tools.md) — WorkspaceMesh 工具集

---

## 推薦下一步

✅ **Set1 (B/C/D/E/F/G) + Set2 (A/C/D/E) 全部到位**。Set2-F 列為 ⚪ 可忽略，Set2-B 保留待之後再評估。

實機測試重點：
1. **預設 ForwardCommandController 模式**跑 — 不要加 `--use-traj`，看 wrist 抖動是否徹底消除（JTC spline re-plan 主因之一被繞過）
2. **`--stuck-reset-ms 0`** 停用 Set2-F auto reset，改用 keyboard `r` 手動重置 ref
3. 觀察 `pos_err_mm` / `ori_err_deg` / `max_joint_delta_deg` p95，跟 baseline 對比
4. 若 wrist 還有殘餘抖：先試 `--ori-lpf-alpha 0.2` 加重輸入端平滑，再試 `--wrist-vel-cap 2.5` 加緊預防層

完整推薦命令：
```bash
python3 placo_ik_online_profiler_ws_mesh.py --arm right \
    --stuck-reset-ms 0 \
    --joint-jump-guard-deg 15 \
    --wrist-vel-cap 4.0 \
    --ori-lpf-alpha 0.35
```
