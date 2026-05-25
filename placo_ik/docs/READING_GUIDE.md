# placo_ik/docs — 導讀地圖

> **目的**：讓任何新加入者、或是自己過了一段時間回來看，能在最短時間內理解整個系統的演進脈絡。
> 依閱讀順序標號，並說明「為什麼要在這個時間點讀這個檔案」。

---

## 閱讀順序一覽

| # | 檔案 | 定位 | 為什麼在此順序 |
|:---:|---|---|---|
| 1 | [placo_solver_analysis.md](placo_solver_analysis.md) | **架構起點** | 最先讀：理解 Placo IK 為何慢（20–30ms）、架構是什麼、cache 與 early exit 怎麼運作。沒這個基礎後面都看不懂。 |
| 2 | [vr_realtime_ik_analysis.md](vr_realtime_ik_analysis.md) | **問題診斷** | 接著讀：評估 VR 即時控制下 4 大指標哪裡不達標（效能、軌跡連續、奇異點、約束）。定義了問題的優先序。 |
| 3 | [csv_analysis.md](csv_analysis.md) | **數據佐證** | 看完定性分析，補上數據：solver 成功率、IK latency p95 是多少，佐證問題嚴重度。 |
| 4 | [ik_solver_weights.md](ik_solver_weights.md) | **設計基礎** | 必讀設計文件：3 種 solver（pinocchio/pybullet/placo）的關節權重邏輯、j1–j7 分組哲學。後面所有調參都要引用這裡。 |
| 5 | [left_right_ik_diff.md](left_right_ik_diff.md) | **左右差異** | 左右臂鏡像問題，j2/j3/j7 軸向反向，URDF limits 鏡像翻轉。理解後才不會把左臂 bug 當右臂 bug 修。 |
| 6 | [ik_seeds_and_joints_task_weight.md](ik_seeds_and_joints_task_weight.md) | **IK 深度概念** | 深入理解 seed 是什麼、Bug 3（per-joint weight 是 dead code）、Bug 4（在線版 PlacoSession 不使用 seeds）。是後續優化的關鍵背景。 |
| 7 | [refactor_jitter_fix.md](refactor_jitter_fix.md) | **Set1 根因修正** | Fix-1/2/3 三大根因（TfPoller / vel_limits / AsyncCsvWriter），說明為什麼最初的系統有 0–300ms 不定期卡頓。 |
| 8 | [anti_vibration_fixes_test3.md](anti_vibration_fixes_test3.md) | **Set1 實作全記錄** | `jitter_test3` 分支的 8 大修正（SLERP LPF、jump guard、ForwardCommandController、IK seed tracking etc.）。讀完能理解 merge 後的程式為何長這樣。 |
| 9 | [adaptive_dls.md](adaptive_dls.md) | **Set2-C 詳設** | 奇異點 wrist wobble 的根本解：Adaptive DLS，Jacobian SVD → σ_min → λ 動態調整。 |
| 10 | [wrist_vel_cap.md](wrist_vel_cap.md) | **Extra 詳設** | wrist 速度硬約束（URDF 20.94 rad/s → 4.0 rad/s 覆寫）。與 Adaptive DLS 並列，一個是預防層一個是反應層。 |
| 11 | [set2d_continuous_approach.md](set2d_continuous_approach.md) | **Set2-D 詳設** | 移除 success-freeze gate — 邊界外時手臂「持續逼近但走不到」，取代凍結行為。 |
| 12 | [set2f_stuck_reset.md](set2f_stuck_reset.md) | **Set2-F（不採用）** | Auto stuck-reset 的設計草案，最終決定不採用（manual `r` 鍵取代）。讀讀是非了解為什麼放棄它。 |
| 13 | [ws_mesh_tools.md](ws_mesh_tools.md) | **工具集手冊** | WorkspaceMesh 6 個工具。可隨時查閱，不需要按順序；需要跑 reachability scan 或 3D 可視化時翻這裡。 |
| ★ 14 | [improvements_status.md](improvements_status.md) | **★ 整合彙總中心** | 全部讀完後必看。Set1 A–G × Set2 A–F 全狀態表、完整 Pipeline 圖、CLI flags 總覽、決策記錄、下一步優先序。是整個系統的 source of truth。 |

---

## 為什麼順序是這樣

```
理解為什麼慢 → 理解問題在哪 → 有數據 → 理解設計邏輯
(#1)           (#2)             (#3)       (#4,5,6)

          ↓
修根本（Fix-1/2/3） → 修抖動（Set1 A–G） → 修邊界/奇異點（Set2 A–F）
(#7)                  (#8)                  (#9,10,11,12)

          ↓
工具 → 最終狀態總覽
(#13)   (★#14)
```

---

## 快速查閱地圖（按問題分類）

| 你在問什麼 | 直接跳到 |
|---|---|
| IK 為什麼慢 / 卡頓根因 | [placo_solver_analysis.md](placo_solver_analysis.md), [refactor_jitter_fix.md](refactor_jitter_fix.md) |
| wrist 為什麼抖 | [anti_vibration_fixes_test3.md](anti_vibration_fixes_test3.md), [adaptive_dls.md](adaptive_dls.md), [wrist_vel_cap.md](wrist_vel_cap.md) |
| 靠近邊界手臂凍住 | [set2d_continuous_approach.md](set2d_continuous_approach.md), [ws_mesh_tools.md](ws_mesh_tools.md) |
| j3 內旋 / 胸前奇異點 | [ik_solver_weights.md](ik_solver_weights.md) §5, [j3_singularity_weight_issues.md](j3_singularity_weight_issues.md) |
| 左右臂 IK 行為不一致 | [left_right_ik_diff.md](left_right_ik_diff.md) |
| per-joint weight 怎麼沒用？ | [ik_seeds_and_joints_task_weight.md](ik_seeds_and_joints_task_weight.md) Part 3 |
| Seed 是什麼 / 在線版為何不用 seed | [ik_seeds_and_joints_task_weight.md](ik_seeds_and_joints_task_weight.md) Part 1, 2 |
| CLI flags 全覽 | [improvements_status.md](improvements_status.md) §CLI flags 全表 |
| 哪些功能還沒做 | [improvements_status.md](improvements_status.md) §尚未完成的優先序 |
| WorkspaceMesh 怎麼跑 | [ws_mesh_tools.md](ws_mesh_tools.md) |
| IK 成功率數據 | [csv_analysis.md](csv_analysis.md) |

---

## 各檔案一句話定位

| 分類 | 檔案 |
|---|---|
| **讀一遍就夠的基礎** | `placo_solver_analysis` · `ik_solver_weights` · `left_right_ik_diff` |
| **問題診斷數據** | `vr_realtime_ik_analysis` · `csv_analysis` |
| **IK 深度概念** | `ik_seeds_and_joints_task_weight` |
| **實作細節參考** | `refactor_jitter_fix` · `anti_vibration_fixes_test3` · `adaptive_dls` · `wrist_vel_cap` · `set2d_continuous_approach` |
| **設計但放棄的** | `set2f_stuck_reset`（⚪ 不採用） |
| **工具手冊** | `ws_mesh_tools` |
| **★ 必讀總覽** | `improvements_status` |
| **奇異點權重深度分析** | `j3_singularity_weight_issues` |

---

*最後更新：2026-05-25*
