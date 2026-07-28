#!/usr/bin/env python3
"""
placo_ik_main_bimanual.py
=========================
雙臂合併單一 QP 的 IK 入口：一個 node、一個 14-DOF QP 同時解兩個 end-effector。

與單臂入口（placo_ik_main.py --arm both，兩個獨立 node / 兩個 7-DOF QP）的差別：
兩臂的 task 掛在同一個 KinematicsSolver 上，共用 iteration 預算、共用 Δq 縮放、
並可做 EE-EE 近距節流。運動學上兩臂仍獨立，價值在共享約束與單一 tick 時序。

Usage（需要 ROS2 + robot driver 執行中）：
  # 裸跑即可（兩臂 calib 預設 0°，實測方向正確）
  python3 placo_ik_main_bimanual.py

  # dry-run（解 IK 但不發命令）
  python3 placo_ik_main_bimanual.py --dry-run

  # 關掉 workspace clamp（預設開，見 ws_boundary.SoftClamp）
  python3 placo_ik_main_bimanual.py --no-ws-clamp

Published topics：
  /{right,left}_forward_position_controller/commands  (Float64MultiArray)
  /{right,left}/delta_ik_latency_ms                   (Float32)
  /{right,left}_arm_ik_commands                       (JointState)
  /{right,left}_eef_pose                              (PoseStamped) — waypoint
  /{right,left}_eef_pose_fk                           (PoseStamped) — FK(命令關節)
  /{right,left}/boundary_dist_mm                      (Float32)
  /{right,left}/boundary_state                        (String JSON)
  /bimanual/placo_profile                             (String JSON, 合併 profile)
  /joint_actions                    (經 joint_actions_aggregator，若可載入)

Subscribed topics：
  /ee_delta/right + /ee_delta/left  (PoseStamped)   /joint_states

Services：
  /bimanual/go_home     (Trigger)  兩臂一起回 home
  /reset_robot_pose     (Trigger)  先 disable teleop 再 home

限制：tracker 模式 only（鍵盤僅全域控制鍵）、ForwardCommandController only。
"""

import os as _os, sys as _sys
_HERE = _os.path.dirname(_os.path.abspath(__file__))
_ROOT = _os.path.dirname(_HERE)
_sys.path.insert(0, _ROOT)
_sys.path.insert(0, _HERE)
_sys.path.insert(0, _os.path.join(_ROOT, "ik_solver"))
_sys.path.insert(0, _os.path.join(_os.path.dirname(_ROOT), "scripts"))  # data_collection

import argparse
import threading

import rclpy

from placo_ik_session_bimanual import (
    ARMS, _MAX_ITER, _TICK_BUDGET_DEG_DEFAULT, _EE_DIST_SOFT_DEFAULT,
)
from placo_ik_node_bimanual import PlacoBimanualNode

try:
    from data_collection import JointActionsAggregator
    _AGGREGATOR_AVAILABLE = True
except ImportError:
    _AGGREGATOR_AVAILABLE = False
    print("  ⚠  joint_actions_aggregator not available — /joint_actions off")


def _parse_args():
    p = argparse.ArgumentParser(
        description="Placo bimanual single-QP IK (ROS2)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--rate",    type=float, default=50.0,
                   help="控制迴圈 Hz（同時是 solver dt 與 EE pose 發佈率；"
                        "default: 50）")
    p.add_argument("--max-iter", type=int, default=_MAX_ITER, dest="max_iter",
                   help=f"非奇異區 iteration 上限（default: {_MAX_ITER}；"
                        f"奇異區自動 ×4，速度由 Δq 預算管）")
    p.add_argument("--no-vel-limits", action="store_true", dest="no_vel_limits",
                   help="關閉 QP 內 velocity limits（不建議）")
    p.add_argument("--wrist-vel-cap", type=float, default=4.0,
                   dest="wrist_vel_cap",
                   help="j5-7 速度上限 rad/s（default: 4.0；0 以下=URDF）")
    p.add_argument("--tick-budget-deg", type=float,
                   default=_TICK_BUDGET_DEG_DEFAULT, dest="tick_budget_deg",
                   help=f"每 tick 單軸 Δq 預算（整臂統一比例縮放，"
                        f"取代逐軸 jump guard；default: "
                        f"{_TICK_BUDGET_DEG_DEFAULT:.0f}°；0=關閉）")
    p.add_argument("--min-ee-dist", type=float,
                   default=_EE_DIST_SOFT_DEFAULT, dest="min_ee_dist",
                   help=f"兩 EE 近距節流軟閾值 m（default: "
                        f"{_EE_DIST_SOFT_DEFAULT}；接近中才減速；0=關閉）")
    p.add_argument("--lpf-alpha", default="0.25", dest="lpf_alpha",
                   help="輸出端 per-joint LPF α；單值=統一、逗號列=J1..J7"
                        "（default: 0.25 統一，兩臂相同）")
    p.add_argument("--ori-lpf-alpha", type=float, default=1.0,
                   dest="ori_lpf_alpha",
                   help="輸入端 target 姿態 SLERP EMA（default: 1.0=off）")
    p.add_argument("--ee-delta-gap-sec", type=float, default=0.8,
                   dest="ee_delta_gap_sec",
                   help="tracker 訊息中斷多少秒後 re-anchor（default: 0.8）")
    p.add_argument("--no-rot-tracking", action="store_true",
                   dest="no_rot_tracking",
                   help="只追位置不追姿態")
    p.add_argument("--calib-yaw-right", type=float, default=None,
                   dest="calib_yaw_right",
                   help="右臂 tracker→arm yaw 校正 deg（default: 0；實測正常）")
    p.add_argument("--calib-yaw-left", type=float, default=None,
                   dest="calib_yaw_left",
                   help="左臂 tracker→arm yaw 校正 deg（default: 0；"
                        "曾用過的 180° 會造成 X/Y 反向，勿沿用）")
    p.add_argument("--ws-clamp", action="store_true", dest="ws_clamp",
                   default=True,
                   help="啟用 workspace soft clamp（default: on；範圍取自 "
                        "ARM_CONFIG[arm][\"workspace\"]）")
    p.add_argument("--no-ws-clamp", action="store_false", dest="ws_clamp",
                   help="關閉 workspace clamp")
    p.add_argument("--boundary-margin", type=float, default=0.05,
                   dest="boundary_margin",
                   help="SoftClamp 飽和帶寬度 m（default: 0.05）")
    p.add_argument("--j3j4-couple", action="store_true", dest="j3j4_couple",
                   help="啟用 j3/j4 elbow-torso 安全耦合（default: off，"
                        "與單臂入口預設一致）")
    p.add_argument("--csv",  default="", help="輸出 CSV 路徑")
    p.add_argument("--plot", default="", help="輸出 PNG 路徑")
    p.add_argument("--dry-run", action="store_true", dest="dry_run",
                   help="解 IK 但不發命令")
    p.add_argument("--home-first", action="store_true", default=True,
                   dest="home_first",
                   help="啟動先 unfold + home 兩臂（default: on）")
    p.add_argument("--no-home-first", action="store_false", dest="home_first")
    p.add_argument("--verbose", action="store_true",
                   help="每步都印（default: 每 5 步）")
    return p.parse_args()


def _build_arm_overrides(args):
    """整理 per-arm 參數：目前只有 calib_yaw（來自 CLI）。

    舊版此處還會合併 config/bimanual.yaml 的 common/{arm} 區塊，但那份 YAML 的
    11 個 key 中大半是被靜默忽略的（rate / wrist_vel_cap / success_gate /
    use_traj … 實際只能走 CLI），且 left.calib_yaw=180° 與實測結論相反 ——
    帶上 --config 反而會重新引入左臂 X/Y 反向的 bug。移植時整條移除，只留 CLI。

    node 端仍支援 per-arm 的 calib_rpy / lpf_alpha override（見
    PlacoBimanualNode.__init__ 讀 arm_overrides 的地方），只是現在沒有輸入
    管道會填它們；要 per-arm LPF 就加對應的 CLI flag，不要再開 YAML。
    """
    ov = {}
    for arm in ARMS:
        o = {}
        cli_yaw = getattr(args, f"calib_yaw_{arm}")
        if cli_yaw is not None:
            o["calib_yaw"] = cli_yaw
        ov[arm] = o
    return ov


def main():
    args = _parse_args()
    args.arm_overrides = _build_arm_overrides(args)

    rclpy.init()
    node = PlacoBimanualNode(args)

    # joint_actions_aggregator：把兩臂的 /{arm}_arm_ik_commands 併成
    # /joint_actions 供資料收集用（住在 openarm_ros2/scripts/data_collection）。
    aggregator = None
    if _AGGREGATOR_AVAILABLE and not args.dry_run:
        try:
            aggregator = JointActionsAggregator(
                hand_config="o6_both", publish_rate=50.0)
            print("  [✓] joint_actions_aggregator started (o6_both, 50Hz)")
        except Exception as e:
            print(f"  ⚠  aggregator failed to start: {e}")
            aggregator = None

    # MultiThreadedExecutor 是必須的（不能用 rclpy.spin）：go_home /
    # reset_robot_pose 的 callback 會阻塞等 hot loop 執行完 homing，單執行緒
    # executor 會連 /joint_states 與 TF 一起卡死。EePosePublisher 的 timer
    # 也在這個 executor 上跑。
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    if aggregator:
        executor.add_node(aggregator)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if args.home_first:
            print("  Running bimanual pre-home unfold sequence...")
            node.joint_unfold_sequence_both()
            print("  Moving both arms to home (ForwardCommand ramp)...")
            node.send_home_fwd_both(pos_tol=0.025, motion_sec=3.5)
        node.run()
    except KeyboardInterrupt:
        print("\n\n  Ctrl-C — stopping...")
    finally:
        node.print_final_stats()
        node.destroy_node()
        if aggregator:
            aggregator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
