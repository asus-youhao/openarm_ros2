#!/usr/bin/env python3
"""
placo_ik_main.py
================
Placo IK online profiler。預設雙手 (--arm both)、開機先 home (--home-first)，
裸跑即可。

深度 profiling 完全保留：每次 IK 計算都拆解出 setup_ms / loop_ms / iterations。

【RobotWrapper 一律 cache（本檔固定行為，無切換）】
  RobotWrapper 建立成本昂貴且只需建一次；KinematicsSolver 輕量，每步重建沒問題。
  每步 = solver_setup(~0.15ms) + early_exit_iters(~0.3ms) ≈ 0.5ms
  → 200Hz 輕鬆，50Hz deadline miss = 0%。
  （對比：每步 rebuild RobotWrapper(~5ms) + 跑滿 250 iters(~20ms) ≈ 25ms，
    50Hz 必爆 deadline——所以本檔不提供 rebuild 模式。）

【Early exit】
  每 iteration 後檢查 pos_err < POS_TOL (3mm)，達到即跳出。
  near-target 步通常 1-3 iters 即收斂，比跑滿 250 fast 100×。

Usage（需要 ROS2 + robot driver 執行中）：
  # 預設：雙手 + home-first，裸跑即可
  python3 placo_ik_main.py

  # 單臂（右 / 左）
  python3 placo_ik_main.py --arm right
  python3 placo_ik_main.py --arm left

  # dry-run（計算 IK 但不送指令）
  python3 placo_ik_main.py --dry-run

Prerequisites：
  ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
  # tracker 需要在另一個 terminal 發布 /ee_delta/{arm} (PoseStamped)

Published topics（每隻手 {arm} ∈ {right,left}）：
  /{arm}_forward_position_controller/commands           (Float64MultiArray)
  /{arm}/delta_ik_latency_ms                            (Float32) — total ik_ms
  /{arm}/placo_profile                                  (String, JSON) — detailed breakdown

CSV columns：
  t, x, y, z, success, ik_ms, total_ms, dx, dy, dz,
  robot_ms, setup_ms, loop_ms, iterations, iter_ms,
  pos_err_mm, track_err_mm, mem_kb, deadline_missed
"""

# ── Path setup ───────────────────────────────────────────────────────────────
import os as _os, sys as _sys
_HERE = _os.path.dirname(_os.path.abspath(__file__))           # ik_node/
_ROOT = _os.path.dirname(_HERE)                                # project root
_sys.path.insert(0, _ROOT)                                     # paths.py
_sys.path.insert(0, _HERE)                                     # siblings: session, node
_sys.path.insert(0, _os.path.join(_ROOT, "ik_solver"))         # placo_ik_solver
_sys.path.insert(0, _os.path.join(_os.path.dirname(_ROOT), "scripts"))  # joint_actions_aggregator

import argparse
import copy
import threading

import rclpy

from placo_ik_session import _MAX_ITER
from placo_ik_node import PlacoOnlineProfiler

try:
    from data_collection import JointActionsAggregator
    _AGGREGATOR_AVAILABLE = True
except ImportError:
    _AGGREGATOR_AVAILABLE = False
    print("  ⚠  joint_actions_aggregator not available — /joint_actions will not be published")


# ── CLI ───────────────────────────────────────────────────────────────────────
def _parse_args():
    p = argparse.ArgumentParser(
        description="Placo IK online profiler (ROS2) — bimanual",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--arm",       default="both", choices=["right", "left", "both"])
    p.add_argument("--config",    default=None,
                   help="Path to bimanual YAML config (e.g. ../config/bimanual.yaml). "
                        "Required when --arm both; optional for single-arm use.")
    p.add_argument("--rate",      type=float, default=50.0,
                   help="Control-loop Hz  (default: 50).  Also sets solver dt.")
    p.add_argument("--max-iter",  type=int, default=_MAX_ITER, dest="max_iter",
                   help=f"Solver iteration cap  (default: {_MAX_ITER})")
    p.add_argument("--wrist-vel-cap", type=float, default=4.0, dest="wrist_vel_cap",
                   help="Wrist (joint5-7) velocity cap in rad/s for teleop smoothness. "
                        "URDF default = 20.94 rad/s (1200°/s) lets IK noise pass through. "
                        "Default 4.0 rad/s ≈ 230°/s.  0 or negative = use URDF default.")
    p.add_argument("--lpf-alpha", default="0.25,0.25,0.25,0.25,0.25,0.25,0.25",
                   dest="lpf_alpha",
                   help="Output-side 1st-order LPF on joint cmd. "
                        "Single value '0.5' = uniform; "
                        "comma list '0.3,0.3,0.3,0.3,0.6,0.6,0.6' = per-joint J1..J7. "
                        "Smaller α = heavier smoothing + more lag. "
                        "Default: 0.25,0.25,0.25,0.4,0.6,0.6,0.6")
    p.add_argument("--ori-lpf-alpha", type=float, default=1.0, dest="ori_lpf_alpha",
                   help="Input-side SLERP EMA on target orientation. "
                        "0.35 = moderate smoothing (default). "
                        "1.0 = off. Smaller = heavier smoothing.")
    p.add_argument("--ee-delta-gap-sec", type=float, default=0.8, dest="ee_delta_gap_sec",
                   help="Seconds without tracker msg before resetting session ref (default: 0.8)")
    p.add_argument("--joint-jump-guard-deg", type=float, default=10.0,
                   dest="joint_jump_guard_deg",
                   help="Max single-joint delta per step in degrees. "
                        "IK output exceeding this is rejected. 0 = off. (default: 10)")
    p.add_argument("--no-rot-tracking", action="store_true", dest="no_rot_tracking",
                   help="Disable orientation tracking (position-only IK)")
    p.add_argument("--calib-yaw", type=float, default=0.0, dest="calib_yaw",
                   help="Tracker→arm yaw offset in degrees  (default: 0)")
    p.add_argument("--calib-rpy", default=None, dest="calib_rpy",
                   help="'roll,pitch,yaw' degrees — overrides --calib-yaw")
    p.add_argument("--ws-clamp",  action="store_true", default=True, dest="ws_clamp",
                   help="Enable the rectangular workspace soft clamp; box comes "
                        "from ARM_CONFIG[arm]['workspace'] (default: on). "
                        "Position-saturating, not a hard clip — see "
                        "ws_boundary.SoftClamp")
    p.add_argument("--no-ws-clamp", action="store_false", dest="ws_clamp",
                   help="Disable the workspace clamp")
    p.add_argument("--boundary-margin", type=float, default=0.05,
                   dest="boundary_margin",
                   help="SoftClamp saturation band width in metres (default: 0.05)")
    p.add_argument("--csv",       default="", help="Output CSV path")
    p.add_argument("--plot",      default="", help="Output plot PNG path")
    p.add_argument("--dry-run",   action="store_true", dest="dry_run",
                   help="Compute IK but do not publish trajectory")
    p.add_argument("--home-first", action="store_true", default=True, dest="home_first",
                   help="Send arm to home (with TF confirmation) before starting (default: on)")
    p.add_argument("--no-home-first", action="store_false", dest="home_first",
                   help="Skip homing before starting")
    p.add_argument("--verbose",   action="store_true",
                   help="Print every IK step  (default: every 5th)")
    p.add_argument("--keyboard",  action="store_true",
                   help="Start in KEYBOARD mode  (w/s/a/d/q/e/i/k/j/l/u/o)")
    p.add_argument("--no-j3j4-couple", action="store_true", default=True, dest="no_j3j4_couple",
                   help="Disable j3/j4 elbow-torso safety coupling (both soft QP guidance "
                        "and hard post-solve clip). Use when testing chest-reach without "
                        "elbow restriction, or to diagnose coupling behaviour. (default: on)")
    p.add_argument("--j3j4-couple", action="store_false", dest="no_j3j4_couple",
                   help="Enable j3/j4 elbow-torso safety coupling")
    return p.parse_args()


# ── Helpers ──────────────────────────────────────────────────────────────────
def _load_yaml_config(config_path):
    """Load bimanual YAML. Returns dict (keys: 'common', 'right', 'left')."""
    try:
        import yaml
    except ImportError:
        raise SystemExit("pyyaml not installed — run: pip install pyyaml")
    with open(config_path) as f:
        return yaml.safe_load(f) or {}


def _apply_yaml_to_args(args, yaml_cfg, arm):
    """Merge YAML common + arm-specific values onto an args Namespace copy."""
    merged = dict(yaml_cfg.get("common", {}))
    merged.update(yaml_cfg.get(arm, {}))
    for key, val in merged.items():
        dest = key.replace("-", "_")
        if hasattr(args, dest):
            setattr(args, dest, val)
    return args


def _add_reset_pose_service(host_node, nodes):
    """Create a single /reset_robot_pose (std_srvs/Trigger) on host_node that
    first disables teleop, then homes every node in `nodes`.

    Flow:
      1. Call /set_teleop_arm_enabled {data: false} as a client so the tracker
         stops feeding ee_delta (best-effort — skipped if the service is offline).
      2. Reuse each node's hot-loop homing mechanism (_home_request /
         _home_done), the same path as /{arm}/go_home and keyboard 'h'.

    Requires a MultiThreadedExecutor + ReentrantCallbackGroup (host_node._cbg),
    otherwise the client wait / home wait would deadlock the executor.
    """
    import time
    from std_srvs.srv import Trigger, SetBool

    teleop_cli = host_node.create_client(
        SetBool, "/set_teleop_arm_enabled", callback_group=host_node._cbg)

    def _disable_teleop(timeout_sec=3.0):
        if not teleop_cli.wait_for_service(timeout_sec=1.0):
            return "SKIPPED (service /set_teleop_arm_enabled offline)"
        sb = SetBool.Request()
        sb.data = False
        fut = teleop_cli.call_async(sb)
        t0 = time.time()
        while not fut.done() and (time.time() - t0) < timeout_sec:
            time.sleep(0.02)
        if not fut.done():
            return "TIMEOUT (no response)"
        res = fut.result()
        return f"disabled ({res.message})" if res.success else f"FAILED ({res.message})"

    def _cb(req, resp):
        arms = ", ".join(n.args.arm for n in nodes)
        print("\n  [srv] /reset_robot_pose → step1: disable teleop")
        disable_msg = _disable_teleop()
        print(f"  [srv] /reset_robot_pose → step2: homing: {arms}  ({disable_msg})")
        for n in nodes:
            n._home_done.clear()
        for n in nodes:
            n._home_request.set()
        ok_all = True
        msgs = []
        for n in nodes:
            if n._home_done.wait(timeout=25.0):
                ok_all = ok_all and n._home_result["ok"]
                msgs.append(f"{n.args.arm}: {n._home_result['msg']}")
            else:
                ok_all = False
                msgs.append(f"{n.args.arm}: timeout — hot loop not running?")
        resp.success = ok_all
        resp.message = f"teleop: {disable_msg} | home: " + " | ".join(msgs)
        return resp

    host_node.create_service(
        Trigger, "/reset_robot_pose", _cb,
        callback_group=host_node._cbg)
    print("  [✓] service /reset_robot_pose (std_srvs/Trigger) — disable teleop → home all arms")


def _run_arm(node):
    """Run one arm's home sequence + IK hot-loop. Designed to run in a thread."""
    try:
        if node.args.home_first:
            arm = node.args.arm
            print(f"  [{arm}] Running pre-home unfold sequence...")
            node.joint_unfold_sequence()
            print(f"  [{arm}] Moving to home (ForwardCommand ramp)...")
            node.send_home_fwd(pos_tol=0.025, motion_sec=3.5)
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.print_final_stats()


def _run_bimanual(args):
    """Launch right + left IK nodes in one process with MultiThreadedExecutor.
    
    If aggregator is available, also starts joint_actions_aggregator to combine
    arm commands into /joint_actions topic for data collection.
    """
    from rclpy.executors import MultiThreadedExecutor

    yaml_cfg = {}
    if args.config:
        yaml_cfg = _load_yaml_config(args.config)
        print(f"  [config] Loaded: {args.config}")
    elif not args.config:
        print("  ⚠  --arm both without --config: both arms use identical CLI defaults")

    args_right = _apply_yaml_to_args(copy.deepcopy(args), yaml_cfg, "right")
    args_right.arm = "right"
    args_left  = _apply_yaml_to_args(copy.deepcopy(args), yaml_cfg, "left")
    args_left.arm  = "left"

    rclpy.init()
    node_right = PlacoOnlineProfiler(args_right)
    node_left  = PlacoOnlineProfiler(args_left)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node_right)
    executor.add_node(node_left)

    # Single /reset_robot_pose that disables teleop then homes BOTH arms.
    _add_reset_pose_service(node_right, [node_right, node_left])

    # Start joint_actions_aggregator if available (default: o6_both)
    aggregator_node = None
    if _AGGREGATOR_AVAILABLE:
        try:
            aggregator_node = JointActionsAggregator(hand_config="o6_both", publish_rate=50.0)
            executor.add_node(aggregator_node)
            print("  [✓] joint_actions_aggregator started (o6_both, 50Hz)")
        except Exception as e:
            print(f"  ⚠  Failed to start aggregator: {e}")
            aggregator_node = None
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    t_right = threading.Thread(target=_run_arm, args=(node_right,), name="ik-right", daemon=True)
    t_left  = threading.Thread(target=_run_arm, args=(node_left,),  name="ik-left",  daemon=True)
    t_right.start()
    t_left.start()

    try:
        t_right.join()
        t_left.join()
    except KeyboardInterrupt:
        print("\n  Ctrl-C — stopping both arms...")
    finally:
        node_right.destroy_node()
        node_left.destroy_node()
        if aggregator_node:
            aggregator_node.destroy_node()
        rclpy.shutdown()


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    args = _parse_args()

    if args.arm == "both":
        _run_bimanual(args)
        return

    # ── Single-arm path ──────────────────────────────────────────────────────
    # MultiThreadedExecutor (not rclpy.spin): the go_home / reset_robot_pose
    # service callbacks block waiting on the hot loop, so a single-threaded
    # executor would deadlock (TF / joint_states / client responses stall).
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init()
    node = PlacoOnlineProfiler(args)

    # /reset_robot_pose in single-arm mode homes just this arm.
    _add_reset_pose_service(node, [node])

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if args.home_first:
            # Safety unfold sequence: joint3 0→90°, joint4 0→90°, joint3 90→0°
            print("  Running pre-home unfold sequence...")
            node.joint_unfold_sequence()
            print("  Moving to home (ForwardCommand ramp)...")
            node.send_home_fwd(pos_tol=0.025, motion_sec=3.5)
        node.run()
    except KeyboardInterrupt:
        print("\n\n  Ctrl-C — stopping...")
    finally:
        node.print_final_stats()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
