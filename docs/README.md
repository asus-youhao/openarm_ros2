# Documentation Index

設計與分析文件，按主題分組：

## 總覽
- [../README.md](../README.md) — 專案總覽（tracker_ee_delta_ik 控制器、Home 姿態、CLI 範例）

## Solver 設計
- [ik_solver_weights.md](ik_solver_weights.md) — 4 種 solver（pinocchio / pybullet / placo / pure_python）的關節權重設計邏輯
- [left_right_ik_diff.md](left_right_ik_diff.md) — 左右臂 IK 權重與 limit 差異分析

## Placo 深度分析
- [placo_solver_analysis.md](placo_solver_analysis.md) — Placo IK 架構、為什麼測到 20–30 ms、cache + early exit 優化原理
- [vr_realtime_ik_analysis.md](vr_realtime_ik_analysis.md) — VR 即時控制 IK Solver 4 大指標對齊度評估
- [refactor_jitter_fix.md](refactor_jitter_fix.md) — Jitter 修正（TfPoller / vel_limits / AsyncCsvWriter）三大根因與重構

## 工具集
- [ws_mesh_tools.md](ws_mesh_tools.md) — WorkspaceMesh 6 個工具（reachability / analyze / view3d / clamp benchmark / profiler ws_mesh 版）
- [csv_analysis.md](csv_analysis.md) — IK Solver 成功率分析（results/*.csv）
