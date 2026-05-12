#!/usr/bin/env python3
"""
placo_ik_online_profiler_ws_mesh.py
===================================
基於 placo_ik_online_profiler.py，但將矩形 workspace clamp
替換為由 placo_ws_analyze.WorkspaceMesh (.npz) 驅動的真實形狀 clamp。

  --ws-mesh <path.npz>   載入掃描產生的 WorkspaceMesh（取代矩形 box）
  --ws-mesh-or  0.0      最低 orient_rate 門檻（若 npz 缺少時從 CSV rebuild）

原有深度 profiling 完全保留：每次 IK 計算都拆解出 setup_ms / loop_ms / iterations，
並揭露為何原版測到 20-30 ms。

【為什麼原版是 20-30 ms？】
  原版 PlacoIKSolver._solve_from_seed() 有兩個問題：
  1. 每次 solve 都 rebuild RobotWrapper（~5ms overhead）
  2. _solve_from_seed 跑滿 MAX_ITER=250 iterations，沒有 early exit
     → 250 iters × ~0.08ms/iter ≈ 20ms + robot_build ≈ 5ms = ~25ms

【online 要 rebuild 還是 cache？→ 一定要 cache！】
  rebuild (原版行為):
    每步 = robot_build(~5ms) + solver_setup(~0.1ms) + 250_iters(~20ms) ≈ 25ms
    → 20Hz 已在 deadline 邊緣，50Hz 不可能 (預算=20ms)
  cached (本檔預設):
    每步 = solver_setup(~0.15ms) + early_exit_iters(~0.3ms) ≈ 0.5ms
    → 200Hz 輕鬆，50Hz deadling miss = 0%
    原理：RobotWrapper 建立成本昂貴且只需建一次；
           KinematicsSolver 輕量，每步重建沒問題。

【Early exit（本檔新增，原版缺少）】
  每 iteration 後檢查 pos_err < POS_TOL (3mm)，達到即跳出。
  near-target 步通常 1-3 iters 即收斂，比跑滿 250 fast 100×。

Usage（需要 ROS2 + robot driver 執行中）：
  # 右臂，ee_delta tracker 模式，right 手把
  conda run -n pico_teleop_py python3 placo_ik_online_profiler.py --arm right

  # 左臂
  conda run -n pico_teleop_py python3 placo_ik_online_profiler.py --arm left

  # 對比原版行為（rebuild + 無 early exit）
  conda run -n pico_teleop_py python3 placo_ik_online_profiler.py --arm right --rebuild

  # 自訂輸出路徑
  conda run -n pico_teleop_py python3 placo_ik_online_profiler.py \\
      --arm right --csv ~/my_profile.csv --plot ~/my_profile.png

  # dry-run（計算 IK 但不送 trajectory）
  conda run -n pico_teleop_py python3 placo_ik_online_profiler.py --arm right --dry-run

Prerequisites（和 tracker_ee_delta_ik_backend.py 相同）：
  ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
  # tracker 需要在另一個 terminal 發布 /ee_delta/{arm} (PoseStamped)

Published topics（完全相容 tracker_ee_delta_ik_backend.py）：
  /right_joint_trajectory_controller/joint_trajectory   (JointTrajectory)
  /right/delta_ik_latency_ms                            (Float32) — total ik_ms
  /right/placo_profile                                  (String, JSON) — detailed breakdown

CSV columns（延伸自原版）：
  t, x, y, z, success, ik_ms, total_ms, dx, dy, dz,
  robot_ms, setup_ms, loop_ms, iterations, iter_ms,
  pos_err_mm, track_err_mm, mem_kb, deadline_missed
"""

# ── Path setup ───────────────────────────────────────────────────────────────
import os as _os, sys as _sys
_DIR    = _os.path.dirname(_os.path.abspath(__file__))
_PARENT = _os.path.dirname(_DIR)
_sys.path.insert(0, _PARENT)
_sys.path.insert(0, _DIR)
_sys.path.insert(0, _os.path.join(_DIR, "ik_solver"))

import argparse
import os
import threading

import rclpy

from paths import ws_mesh_path as _ws_mesh_path
from placo_ik_session import _MAX_ITER
from placo_ik_node import PlacoOnlineProfiler


# ── CLI ───────────────────────────────────────────────────────────────────────
def _parse_args():
    p = argparse.ArgumentParser(
        description="Placo IK online profiler with WorkspaceMesh clamp (ROS2)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--arm",       default="right", choices=["right", "left"])
    p.add_argument("--rate",      type=float, default=20.0,
                   help="Control-loop Hz  (default: 20).  Also sets solver dt.")
    p.add_argument("--horizon",   type=float, default=60.0,
                   help="JointTrajectory duration ms  (default: 60)")
    p.add_argument("--max-iter",  type=int, default=_MAX_ITER, dest="max_iter",
                   help=f"Solver iteration cap  (default: {_MAX_ITER})")
    p.add_argument("--rebuild",   action="store_true",
                   help="Rebuild RobotWrapper every step — original ~25 ms behaviour")
    p.add_argument("--no-vel-limits", action="store_true", dest="no_vel_limits",
                   help="Disable joint velocity limits in IK solver  (not recommended)")
    p.add_argument("--calib-yaw", type=float, default=0.0, dest="calib_yaw",
                   help="Tracker→arm yaw offset in degrees  (default: 0)")
    p.add_argument("--calib-rpy", default=None, dest="calib_rpy",
                   help="'roll,pitch,yaw' degrees — overrides --calib-yaw")
    p.add_argument("--tracker-side", default=None, dest="tracker_side",
                   choices=["left", "right", None])
    p.add_argument("--csv",       default="", help="Output CSV path")
    p.add_argument("--plot",      default="", help="Output plot PNG path")
    p.add_argument("--dry-run",   action="store_true", dest="dry_run",
                   help="Compute IK but do not publish trajectory")
    p.add_argument("--home-first", action="store_true", dest="home_first",
                   help="Send arm to home (with TF confirmation) before starting")
    p.add_argument("--no-ws-clamp", action="store_true", dest="no_ws_clamp",
                   help="Disable workspace XYZ clamping")
    p.add_argument("--ws-mesh",   default=None, dest="ws_mesh",
                   help="WorkspaceMesh .npz path (from placo_ws_analyze.py)")
    p.add_argument("--verbose",   action="store_true",
                   help="Print every IK step  (default: every 5th)")
    p.add_argument("--keyboard",  action="store_true",
                   help="Start in KEYBOARD mode  (w/s/a/d/q/e/i/k/j/l/u/o)")
    return p.parse_args()


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    args = _parse_args()

    # Auto-detect WorkspaceMesh if not specified
    if not args.ws_mesh:
        _auto = _ws_mesh_path(args.arm)
        if os.path.isfile(_auto):
            args.ws_mesh = _auto
            print(f"  [ws_mesh] auto-detected: {_auto}")

    rclpy.init()
    node = PlacoOnlineProfiler(args)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        if args.home_first:
            print("  Moving to home (with TF confirmation)...")
            node.send_home_confirmed(pos_tol=0.025, motion_sec=3.5, max_tries=5)
        node.run()
    except KeyboardInterrupt:
        print("\n\n  Ctrl-C — stopping...")
    finally:
        node.print_final_stats()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
