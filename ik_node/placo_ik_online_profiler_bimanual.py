#!/usr/bin/env python3
"""
placo_ik_online_profiler_bimanual.py
====================================
雙臂合併單一 QP 版的 online profiler 入口（v1，2026-06-12）。
取代「placo_ik_online_profiler_ws_mesh.py --arm both」的雙 node 架構，
改為一個 node、一個 14-DOF QP 同時解兩個 end-effector。
設計與差異說明見 docs/bimanual_single_qp.md。

Usage（需要 ROS2 + robot driver 執行中）：
  # 裸跑即可（兩臂 calib 預設 0°，實測方向正確）
  conda run -n pico_teleop_py python3 placo_ik_online_profiler_bimanual.py

  # 帶 YAML（沿用 config/bimanual.yaml 的 common/right/left 結構）
  python3 placo_ik_online_profiler_bimanual.py --config ../config/bimanual.yaml

  # dry-run（解 IK 但不發命令）
  python3 placo_ik_online_profiler_bimanual.py --dry-run

Published topics：
  /right_forward_position_controller/commands   (Float64MultiArray)
  /left_forward_position_controller/commands    (Float64MultiArray)
  /{right,left}/delta_ik_latency_ms             (Float32)
  /{right,left}_arm_ik_commands                 (JointState, aggregator 用)
  /bimanual/placo_profile                       (String JSON, 合併 profile)

Subscribed topics：
  /ee_delta/right + /ee_delta/left  (PoseStamped)   /joint_states

v1 限制：tracker 模式 only（鍵盤僅全域控制鍵）、ForwardCommand only。
"""

import os as _os, sys as _sys
_HERE = _os.path.dirname(_os.path.abspath(__file__))
_ROOT = _os.path.dirname(_HERE)
_sys.path.insert(0, _ROOT)
_sys.path.insert(0, _HERE)
_sys.path.insert(0, _os.path.join(_ROOT, "ik_solver"))
_sys.path.insert(0, _os.path.join(_ROOT, "ws_mesh"))
_sys.path.insert(0, _os.path.join(_os.path.dirname(_ROOT), "scripts"))

import argparse
import os
import threading

import rclpy

from paths import ws_mesh_path as _ws_mesh_path
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
        description="Placo bimanual single-QP online profiler (ROS2)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--config",  default=None,
                   help="bimanual YAML（common/right/left；選填，"
                        "裸跑時兩臂 calib 預設 0°）")
    p.add_argument("--rate",    type=float, default=50.0,
                   help="控制迴圈 Hz（同時是 solver dt；default: 50）")
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
                        "（default: 0.25 統一）")
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
                        "2026-06-15 修正：舊值 180° 造成 X/Y 反向）")
    p.add_argument("--ws-clamp", action="store_true", dest="ws_clamp",
                   help="啟用 workspace clamp（default: off；有 mesh 自動開）")
    p.add_argument("--ws-mesh-right", default=None, dest="ws_mesh_right",
                   help="右臂 WorkspaceMesh .npz（default: 自動偵測）")
    p.add_argument("--ws-mesh-left",  default=None, dest="ws_mesh_left",
                   help="左臂 WorkspaceMesh .npz（default: 自動偵測）")
    p.add_argument("--boundary-margin", type=float, default=0.05,
                   dest="boundary_margin",
                   help="SoftClamp 軟邊界 margin m（default: 0.05）")
    p.add_argument("--j3j4-couple", action="store_true", dest="j3j4_couple",
                   help="啟用 j3/j4 elbow-torso 安全耦合（default: off，"
                        "與單臂版 profiler 預設一致）")
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


def _load_yaml(path):
    try:
        import yaml
    except ImportError:
        raise SystemExit("pyyaml not installed — pip install pyyaml")
    with open(path) as f:
        return yaml.safe_load(f) or {}


def _build_arm_overrides(args):
    """整理 per-arm 參數：CLI > YAML(arm) > YAML(common) > 內建預設。

    認得的 per-arm key：calib_yaw / calib_rpy / ws_mesh / lpf_alpha。
    （YAML 的 common 區也可放這些 key，套用到兩臂。）
    """
    yaml_cfg = _load_yaml(args.config) if args.config else {}
    if args.config:
        print(f"  [config] Loaded: {args.config}")
    ov = {}
    for arm in ARMS:
        merged = dict(yaml_cfg.get("common", {}))
        merged.update(yaml_cfg.get(arm, {}))
        o = {k: merged[k] for k in
             ("calib_yaw", "calib_rpy", "ws_mesh", "lpf_alpha") if k in merged}
        # CLI 覆寫
        cli_yaw = getattr(args, f"calib_yaw_{arm}")
        if cli_yaw is not None:
            o["calib_yaw"] = cli_yaw
            o.pop("calib_rpy", None)
        cli_mesh = getattr(args, f"ws_mesh_{arm}")
        if cli_mesh:
            o["ws_mesh"] = cli_mesh
        # mesh 自動偵測
        if not o.get("ws_mesh"):
            auto = _ws_mesh_path(arm)
            if os.path.isfile(auto):
                o["ws_mesh"] = auto
                print(f"  [{arm}][ws_mesh] auto-detected: {auto}")
        ov[arm] = o
    return ov


def main():
    args = _parse_args()
    args.arm_overrides = _build_arm_overrides(args)

    rclpy.init()
    node = PlacoBimanualNode(args)

    aggregator = None
    if _AGGREGATOR_AVAILABLE and not args.dry_run:
        try:
            aggregator = JointActionsAggregator(
                hand_config="o6_both", publish_rate=50.0)
            print("  [✓] joint_actions_aggregator started (o6_both, 50Hz)")
        except Exception as e:
            print(f"  ⚠  aggregator failed to start: {e}")
            aggregator = None

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
