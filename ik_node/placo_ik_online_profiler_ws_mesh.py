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
_HERE = _os.path.dirname(_os.path.abspath(__file__))           # ik_node/
_ROOT = _os.path.dirname(_HERE)                                # project root
_sys.path.insert(0, _ROOT)                                     # paths.py
_sys.path.insert(0, _HERE)                                     # siblings: session, node
_sys.path.insert(0, _os.path.join(_ROOT, "ik_solver"))         # placo_ik_solver
_sys.path.insert(0, _os.path.join(_ROOT, "ws_mesh"))           # placo_ws_analyze (via node)
_sys.path.insert(0, _os.path.join(_os.path.dirname(_ROOT), "scripts"))  # joint_actions_aggregator

import argparse
import copy
import os
import threading

import rclpy

from paths import ws_mesh_path as _ws_mesh_path
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
        description="Placo IK online profiler with WorkspaceMesh clamp (ROS2)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--arm",       default="both", choices=["right", "left", "both"])
    p.add_argument("--config",    default=None,
                   help="Path to bimanual YAML config (e.g. ../config/bimanual.yaml). "
                        "Required when --arm both; optional for single-arm use.")
    p.add_argument("--rate",      type=float, default=50.0,
                   help="Control-loop Hz  (default: 50).  Also sets solver dt.")
    p.add_argument("--horizon",   type=float, default=None,
                   help="JointTrajectory duration ms  (default: auto = 1.5× period)")
    p.add_argument("--max-iter",  type=int, default=_MAX_ITER, dest="max_iter",
                   help=f"Solver iteration cap  (default: {_MAX_ITER})")
    p.add_argument("--rebuild",   action="store_true",
                   help="Rebuild RobotWrapper every step — original ~25 ms behaviour")
    p.add_argument("--no-vel-limits", action="store_true", dest="no_vel_limits",
                   help="Disable joint velocity limits in IK solver  (not recommended)")
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
    p.add_argument("--tracker-side", default=None, dest="tracker_side",
                   choices=["left", "right", None])
    p.add_argument("--csv",       default="", help="Output CSV path")
    p.add_argument("--plot",      default="", help="Output plot PNG path")
    p.add_argument("--dry-run",   action="store_true", dest="dry_run",
                   help="Compute IK but do not publish trajectory")
    p.add_argument("--home-first", action="store_true", default=True, dest="home_first",
                   help="Send arm to home (with TF confirmation) before starting (default: on)")
    p.add_argument("--no-home-first", action="store_false", dest="home_first",
                   help="Skip homing before starting")
    p.add_argument("--no-ws-clamp", action="store_true", default=False, dest="no_ws_clamp",
                   help="Disable workspace XYZ clamping (clamp is ON by default)")
    p.add_argument("--ws-clamp", action="store_false", dest="no_ws_clamp",
                   help="Enable workspace XYZ clamping (default; kept for compatibility)")
    p.add_argument("--ws-mesh",   default=None, dest="ws_mesh",
                   help="WorkspaceMesh .npz path (from placo_ws_analyze.py). "
                        "Default: None → auto-detect results/reachability_<arm>_ws.npz")
    p.add_argument("--verbose",   action="store_true",
                   help="Print every IK step  (default: every 5th)")
    p.add_argument("--keyboard",  action="store_true",
                   help="Start in KEYBOARD mode  (w/s/a/d/q/e/i/k/j/l/u/o)")
    p.add_argument("--boundary-margin", type=float, default=0.05,
                   dest="boundary_margin",
                   help="Soft-clamp margin in metres  (default: 0.05 = 5 cm). "
                        "Distance from workspace boundary where damping begins.")
    p.add_argument("--no-j3j4-couple", action="store_true", default=True, dest="no_j3j4_couple",
                   help="Disable j3/j4 elbow-torso safety coupling (both soft QP guidance "
                        "and hard post-solve clip). Use when testing chest-reach without "
                        "elbow restriction, or to diagnose coupling behaviour. (default: on)")
    p.add_argument("--j3j4-couple", action="store_false", dest="no_j3j4_couple",
                   help="Enable j3/j4 elbow-torso safety coupling")
    p.add_argument("--success-gate", action="store_true", dest="success_gate",
                   help="Restore legacy 'freeze on IK failure' behaviour. "
                        "Default (off) = Set2-D continuous approach: always publish, "
                        "let velocity_limits saturate the partial solution. "
                        "Set this flag to revert to the old behaviour where IK "
                        "pos_err > 10mm causes the arm to stop moving.")
    p.add_argument("--use-traj",  action="store_true", dest="use_traj",
                   help="Use JointTrajectoryController (legacy, may vibrate) "
                        "instead of the default ForwardCommandController. "
                        "Topic mode is preferred for VR teleop (no JT spline re-plan).")
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


def _prepare_args(args):
    """Auto-detect ws_mesh and compute horizon in-place for one arm."""
    if not args.ws_mesh:
        _auto = _ws_mesh_path(args.arm)
        if os.path.isfile(_auto):
            args.ws_mesh = _auto
            print(f"  [{args.arm}][ws_mesh] auto-detected: {_auto}")
    period_ms = 1000.0 / args.rate
    if args.horizon is None:
        args.horizon = round(1.5 * period_ms, 1)
        print(f"  [{args.arm}][horizon] auto → {args.horizon:.1f}ms  (1.5× period @ {args.rate:.0f}Hz)")
    elif args.horizon < period_ms:
        print(f"  ⚠ [{args.arm}] --horizon {args.horizon:.1f}ms < period {period_ms:.1f}ms "
              f"→ clamped to {period_ms:.1f}ms")
        args.horizon = period_ms


def _start_aggregator(executor, arm):
    """Start JointActionsAggregator for the given arm scope ('both'/'left'/'right').

    Single-arm scope publishes /joint_actions without waiting for the idle
    arm; output is always the full 26 joints — command-less joints (idle arm,
    idle hand) are filled from /joint_states.
    Returns the node (added to executor) or None if unavailable/failed.
    """
    if not _AGGREGATOR_AVAILABLE:
        return None
    try:
        node = JointActionsAggregator(hand_config="o6_both", publish_rate=50.0,
                                      arm_config=arm)
        executor.add_node(node)
        print(f"  [✓] joint_actions_aggregator started (arm={arm}, o6_both, 50Hz)")
        return node
    except Exception as e:
        print(f"  ⚠  Failed to start aggregator: {e}")
        return None


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
            if node.args.use_traj:
                print(f"  [{arm}] Moving to home (JointTrajectory)...")
                node.send_home_confirmed(pos_tol=0.025, motion_sec=3.5, max_tries=5)
            else:
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

    _prepare_args(args_right)
    _prepare_args(args_left)

    rclpy.init()
    node_right = PlacoOnlineProfiler(args_right)
    node_left  = PlacoOnlineProfiler(args_left)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node_right)
    executor.add_node(node_left)

    # Single /reset_robot_pose that homes BOTH arms at once (2026-07-01).
    _add_reset_pose_service(node_right, [node_right, node_left])

    # Start joint_actions_aggregator if available (both arms + both hands)
    aggregator_node = _start_aggregator(executor, "both")

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
    _prepare_args(args)

    rclpy.init()
    node = PlacoOnlineProfiler(args)

    # 2026-06-12: the /{arm}/go_home service callback blocks waiting for homing
    # to finish; a single-threaded executor (rclpy.spin) would get stuck
    # (/joint_states and TF would stall), so use a MultiThreadedExecutor
    # (consistent with the --arm both path).
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    # /reset_robot_pose alias — single arm mode homes just this arm.
    _add_reset_pose_service(node, [node])
    # Single-arm aggregator: publishes /joint_actions with only this arm's
    # joints (+ its hand), without waiting for the idle arm.
    aggregator_node = _start_aggregator(executor, args.arm)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if args.home_first:
            # Safety unfold sequence: joint3 0→90°, joint4 0→90°, joint3 90→0°
            print("  Running pre-home unfold sequence...")
            node.joint_unfold_sequence()
            if args.use_traj:
                print("  Moving to home (JointTrajectory)...")
                node.send_home_confirmed(pos_tol=0.025, motion_sec=3.5, max_tries=5)
            else:
                print("  Moving to home (ForwardCommand ramp)...")
                node.send_home_fwd(pos_tol=0.025, motion_sec=3.5)
        node.run()
    except KeyboardInterrupt:
        print("\n\n  Ctrl-C — stopping...")
    finally:
        node.print_final_stats()
        node.destroy_node()
        if aggregator_node:
            aggregator_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
