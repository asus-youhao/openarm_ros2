# Documentation Index

設計與分析文件，按主題分組：

## 總覽
- [../README.md](../README.md) — 專案總覽（tracker_ee_delta_ik 控制器、Home 姿態、CLI 範例）
- [READING_GUIDE.md](READING_GUIDE.md) — **文件導讀地圖**（閱讀順序 + 快速查閱索引）← 新手從這裡開始
- [improvements_status.md](improvements_status.md) — 兩組 A–F 改進進度總表 + pipeline 全圖 + CLI flags 一覽

## Solver 設計
- [ik_solver_weights.md](ik_solver_weights.md) — 4 種 solver（pinocchio / pybullet / placo / pure_python）的關節權重設計邏輯
- [left_right_ik_diff.md](left_right_ik_diff.md) — 左右臂 IK 權重與 limit 差異分析

## Placo 深度分析
- [placo_solver_analysis.md](placo_solver_analysis.md) — Placo IK 架構、為什麼測到 20–30 ms、cache + early exit 優化原理
- [vr_realtime_ik_analysis.md](vr_realtime_ik_analysis.md) — VR 即時控制 IK Solver 4 大指標對齊度評估
- [refactor_jitter_fix.md](refactor_jitter_fix.md) — Jitter 修正（TfPoller / vel_limits / AsyncCsvWriter）三大根因與重構

## Teleop 平滑機制
- [adaptive_dls.md](adaptive_dls.md) — Adaptive DLS damping（方案 C）— 奇異點 wrist wobble 解法
- [wrist_vel_cap.md](wrist_vel_cap.md) — Joint 5/6/7 velocity cap，QP 硬約束限制 wrist 單步爆衝
- [set2d_continuous_approach.md](set2d_continuous_approach.md) — 方案 D：移除 success-freeze gate，邊界時「持續逼近但走不到」
- [anti_vibration_fixes_test3.md](anti_vibration_fixes_test3.md) — jitter_test3 分支的 8 大修正：SLERP / 跳變 guard / forward_position_controller / IK seed tracking 等

## 工具集
- [ws_mesh_tools.md](ws_mesh_tools.md) — WorkspaceMesh 6 個工具（reachability / analyze / view3d / clamp benchmark / profiler ws_mesh 版）
- [csv_analysis.md](csv_analysis.md) — IK Solver 成功率分析（results/*.csv）

## 奇異點 & 權重問題深度分析
- [j3_singularity_weight_issues.md](j3_singularity_weight_issues.md) — j3 內旋封死問題、IK 旋轉權重 6 個隱藏 Issue、分等級優化方案（P0/P1/P2）
