#!/usr/bin/env python3
"""
placo_ik_profiler.py — Deep performance profiler for the Placo IK solver
=========================================================================

【靜態 Benchmark 模式】  測量每次 solve 的時間拆解與資源用量：
  robot_ms    : placo.RobotWrapper + KinematicsSolver 建立時間
  setup_ms    : Task 設定時間
  solve_ms    : 完整 solve 時間（robot build + 求解迴圈）
  iterations  : 實際收斂所需迭代次數
  iter_ms     : 每次迭代平均耗時
  cpu_time_ms : 本次 solve 消耗的 CPU 時間（user+sys）
  mem_kb      : solve 開始時的常駐記憶體（VmRSS）
  pos_err_mm  : 最終位置誤差（mm）

【Realtime ee_delta 模式】（--realtime）  模擬真實 20/50/100 Hz 控制迴圈：
  以上一步 joints 作為 warm-start seed（模擬 tracker 連續追蹤）
  軌跡模式：sine / circle / lemniscate
  對比兩種機器人建立策略：
    rebuild  : 每步重建 RobotWrapper（現有行為，~5ms overhead）
    cached   : 只重建 KinematicsSolver，複用 RobotWrapper（--cache-robot）
  額外統計：deadline_missed（超時次數）、track_err_mm（軌跡追蹤誤差）

Usage:
  # 靜態 100 次 benchmark（右臂）
  conda run -n pico_teleop_py python3 placo_ik_profiler.py --benchmark 100

  # 左臂，隨機目標，詳細輸出
  conda run -n pico_teleop_py python3 placo_ik_profiler.py \
      --arm left --benchmark 200 --random --profile

  # 掃描 dt 值
  conda run -n pico_teleop_py python3 placo_ik_profiler.py \
      --benchmark 50 --sweep-dt 0.005,0.008,0.010,0.015,0.020

  # ── Realtime 模式（不需要 ROS）──
  # 預設 20Hz，circle 軌跡，30 秒，右臂
  conda run -n pico_teleop_py python3 placo_ik_profiler.py --realtime

  # 50Hz + lemniscate + 快取 robot（對比優化）
  conda run -n pico_teleop_py python3 placo_ik_profiler.py \
      --realtime --rate 50 --traj lemniscate --cache-robot

  # 左臂 20Hz，sine，存 CSV + plot
  conda run -n pico_teleop_py python3 placo_ik_profiler.py \
      --realtime --arm left --traj sine --csv rt_left.csv

  # 同時對比 rebuild vs cached（兩個 terminal）
  conda run -n pico_teleop_py python3 placo_ik_profiler.py --realtime --rate 50
  conda run -n pico_teleop_py python3 placo_ik_profiler.py --realtime --rate 50 --cache-robot
"""

import argparse
import csv
import math
import os
import random
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

# ── Path setup ────────────────────────────────────────────────────────────────
_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_DIR, "ik_solver"))   # placo_ik_solver.py

sys.path.insert(0, _DIR)   # for paths.py
from paths import today_dir as _today_dir, timestamp as _timestamp  # noqa: E402
from placo_ik_solver import (  # noqa: E402
    _find_urdf,
    _HUMAN_RIGHT,
    _HUMAN_LEFT,
    _JOINT_NAMES,
    _ARM_SEEDS,
    _SEEDS_RIGHT,
    _SEEDS_LEFT,
    _quat_to_rot,
)

# ── Resource helpers (no psutil required) ─────────────────────────────────────

def _mem_rss_kb() -> int:
    """Read current process RSS (kB) from /proc/self/status."""
    try:
        with open("/proc/self/status") as fh:
            for line in fh:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1])
    except OSError:
        pass
    return 0


def _cpu_seconds() -> float:
    """Return (user + system) CPU seconds consumed by this process."""
    t = os.times()
    return t[0] + t[1]          # user + system (indices 0, 1)


# ── Placo solver constants (mirrors placo_ik_solver.py) ───────────────────────
_DEFAULT_MAX_ITER = 250
_DEFAULT_DT       = 0.010
_POS_TOL          = 0.003    # m  — convergence threshold
_POS_RELAX        = 0.010    # m  — relaxed acceptance (counts as success)
_W_POS            = 1.0
_W_ORI            = 0.3
_W_JOINTS         = 1e-4

# ── Arm workspace bounds (used for random target generation) ──────────────────
_WORKSPACE = {
    "right": {"x": (0.10, 0.45), "y": (-0.50, -0.05), "z": (0.20, 0.75)},
    "left":  {"x": (0.10, 0.45), "y": ( 0.05,  0.50), "z": (0.20, 0.75)},
}

# ── Home targets ──────────────────────────────────────────────────────────────
_HOME = {
    "right": {
        "xyz":  (0.2160, -0.2952, 0.5297),
        "quat": (0.6642,  0.2425, 0.6642, 0.2425),
        "joints": [0.0,  0.7, 0.0, 1.5708, 0.0, 0.0, 0.0],
    },
    "left": {
        "xyz":  (0.2160,  0.2952, 0.5297),
        "quat": (0.6642, -0.2425, 0.6642, -0.2425),
        "joints": [0.0, -0.7, 0.0, 1.5708, 0.0, 0.0, 0.0],
    },
}


# ── Core profiled solve ────────────────────────────────────────────────────────

def solve_profiled(
    urdf:        str,
    arm:         str,
    target_xyz:  np.ndarray,
    target_R:    np.ndarray,
    seed:        List[float],
    max_iter:    int = _DEFAULT_MAX_ITER,
    dt:          float = _DEFAULT_DT,
    no_rot:      bool = False,
) -> Dict:
    """
    Run one full Placo IK solve from a single seed with detailed profiling.

    Returns a dict with keys:
      robot_ms, setup_ms, solve_ms, iterations, iter_ms,
      cpu_time_ms, mem_kb, pos_err_mm, success, joints
    """
    import placo

    joint_names = _JOINT_NAMES[arm]
    human_cfg   = _HUMAN_RIGHT if arm == "right" else _HUMAN_LEFT
    ee_link     = f"openarm_{arm}_link7"

    mem_kb_start = _mem_rss_kb()
    cpu_t0       = _cpu_seconds()
    wall_t0      = time.perf_counter()

    # ── Robot construction ────────────────────────────────────────────────────
    t_robot0 = time.perf_counter()
    robot    = placo.RobotWrapper(urdf, placo.Flags.ignore_collisions)
    t_robot1 = time.perf_counter()
    robot_ms = (t_robot1 - t_robot0) * 1000.0

    # ── Solver + task setup ───────────────────────────────────────────────────
    t_setup0 = time.perf_counter()
    solver   = placo.KinematicsSolver(robot)
    solver.enable_joint_limits(True)
    solver.enable_velocity_limits(False)
    solver.mask_fbase(True)
    solver.dt = dt

    # Seed
    for name, val in zip(joint_names, seed):
        robot.set_joint(name, val)
    robot.update_kinematics()

    # Tasks
    pos_task = solver.add_position_task(ee_link, target_xyz)
    pos_task.configure("pos", "soft", _W_POS)

    if not no_rot:
        ori_task = solver.add_orientation_task(ee_link, target_R)
        ori_task.configure("ori", "soft", _W_ORI)

    pref_dict = {n: human_cfg[n][0] for n in joint_names}
    jt = solver.add_joints_task()
    jt.set_joints(pref_dict)
    jt.configure("naturalness", "soft", _W_JOINTS)

    reg = solver.add_regularization_task(1e-5)
    reg.configure("reg", "soft", 1.0)
    t_setup1 = time.perf_counter()
    setup_ms = (t_setup1 - t_setup0) * 1000.0

    # ── Iterative solve loop (with per-iteration timing) ─────────────────────
    lo = [human_cfg[n][1] for n in joint_names]
    hi = [human_cfg[n][2] for n in joint_names]

    t_loop0    = time.perf_counter()
    iters_used = 0

    for i in range(max_iter):
        solver.solve(True)
        robot.update_kinematics()
        iters_used += 1

        # Check convergence
        T_ee   = robot.get_T_world_frame(ee_link)
        ee_pos = T_ee[:3, 3]
        pos_e  = float(np.linalg.norm(ee_pos - target_xyz))
        if pos_e < _POS_TOL:
            break

    t_loop1  = time.perf_counter()
    loop_ms  = (t_loop1 - t_loop0) * 1000.0

    # Final joints
    joints  = [robot.get_joint(n) for n in joint_names]
    joints  = [max(l, min(h, q)) for q, l, h in zip(joints, lo, hi)]

    # Final error
    T_ee    = robot.get_T_world_frame(ee_link)
    pos_err = float(np.linalg.norm(T_ee[:3, 3] - target_xyz))

    # ── Resource metrics ──────────────────────────────────────────────────────
    cpu_dt       = _cpu_seconds() - cpu_t0
    cpu_time_ms  = cpu_dt * 1000.0

    total_wall_ms = (time.perf_counter() - wall_t0) * 1000.0
    solve_ms      = robot_ms + setup_ms + loop_ms   # total per solve attempt
    iter_ms       = loop_ms / iters_used if iters_used > 0 else 0.0
    success       = int(pos_err < _POS_RELAX)

    return {
        "robot_ms":    robot_ms,
        "setup_ms":    setup_ms,
        "loop_ms":     loop_ms,
        "solve_ms":    solve_ms,
        "total_wall_ms": total_wall_ms,
        "iterations":  iters_used,
        "iter_ms":     iter_ms,
        "cpu_time_ms": cpu_time_ms,
        "mem_kb":      mem_kb_start,
        "pos_err_mm":  pos_err * 1000.0,
        "success":     success,
        "joints":      joints,
    }


# ── Target generators ──────────────────────────────────────────────────────────

def _make_home_targets(arm: str, n: int) -> List[Tuple]:
    """Return n copies of the home target."""
    h = _HOME[arm]
    R = _quat_to_rot(*h["quat"])
    return [(np.array(h["xyz"]), R, list(h["joints"]))] * n


def _make_random_targets(arm: str, n: int, seed: int = 42) -> List[Tuple]:
    """Return n random (xyz, R, seed_joints) tuples within workspace."""
    rng = random.Random(seed)
    ws  = _WORKSPACE[arm]
    home_R  = _quat_to_rot(*_HOME[arm]["quat"])
    seeds   = _ARM_SEEDS[arm]
    targets = []
    for _ in range(n):
        x = rng.uniform(*ws["x"])
        y = rng.uniform(*ws["y"])
        z = rng.uniform(*ws["z"])
        seed_j = rng.choice(seeds)
        targets.append((np.array([x, y, z]), home_R, list(seed_j)))
    return targets


def _make_grid_targets(arm: str) -> List[Tuple]:
    """Return a structured grid of targets for systematic coverage."""
    ws = _WORKSPACE[arm]
    home_R = _quat_to_rot(*_HOME[arm]["quat"])
    home_j = list(_HOME[arm]["joints"])
    pts = []
    for x in np.linspace(ws["x"][0], ws["x"][1], 3):
        for y in np.linspace(ws["y"][0], ws["y"][1], 4):
            for z in np.linspace(ws["z"][0], ws["z"][1], 3):
                pts.append((np.array([x, y, z]), home_R, home_j))
    return pts


# ── Sweep helpers ──────────────────────────────────────────────────────────────

def _run_sweep_dt(
    urdf:     str,
    arm:      str,
    dt_vals:  List[float],
    n:        int,
    max_iter: int,
    warmup:   int,
    random_targets: bool,
) -> Dict:
    """Sweep across multiple dt values. Returns dict[dt] = aggregate stats."""
    targets = (
        _make_random_targets(arm, n + warmup)
        if random_targets
        else _make_home_targets(arm, n + warmup)
    )
    seed_j = list(_HOME[arm]["joints"])
    results = {}
    for dt in dt_vals:
        print(f"\n── dt={dt:.4f} ─────────────────────────────────────────")
        rows = []
        for i, (xyz, R, _) in enumerate(targets):
            r = solve_profiled(urdf, arm, xyz, R, seed_j, max_iter, dt)
            if i >= warmup:
                rows.append(r)
                _print_row(i - warmup + 1, dt, r, verbose=False)
        results[dt] = _aggregate(rows)
    return results


# ── Aggregation & printing ─────────────────────────────────────────────────────

def _aggregate(rows: List[Dict]) -> Dict:
    """Compute aggregate stats from a list of profiled rows."""
    if not rows:
        return {}
    keys = ["robot_ms", "setup_ms", "loop_ms", "solve_ms", "iter_ms",
            "cpu_time_ms", "iterations", "pos_err_mm"]
    agg = {"n": len(rows), "success_pct": sum(r["success"] for r in rows) / len(rows) * 100}
    for k in keys:
        vals = [r[k] for r in rows]
        agg[f"{k}_mean"] = float(np.mean(vals))
        agg[f"{k}_median"] = float(np.median(vals))
        agg[f"{k}_p95"]  = float(np.percentile(vals, 95))
        agg[f"{k}_max"]  = float(max(vals))
        agg[f"{k}_std"]  = float(np.std(vals))
    agg["mem_kb_mean"] = float(np.mean([r["mem_kb"] for r in rows]))
    return agg


def _print_row(run_i: int, dt: float, r: Dict, verbose: bool):
    """Print one solve row to stdout."""
    tag = "✓" if r["success"] else "✗"
    if verbose:
        print(
            f"  [{run_i:4d}] {tag} "
            f"solve={r['solve_ms']:6.1f}ms  "
            f"robot={r['robot_ms']:5.1f}ms  "
            f"setup={r['setup_ms']:5.1f}ms  "
            f"loop={r['loop_ms']:5.1f}ms  "
            f"iters={r['iterations']:3d}  "
            f"iter_ms={r['iter_ms']:5.3f}  "
            f"cpu={r['cpu_time_ms']:5.1f}ms  "
            f"mem={r['mem_kb']/1024:.1f}MB  "
            f"err={r['pos_err_mm']:5.2f}mm"
        )
    else:
        print(
            f"  [{run_i:4d}] {tag} "
            f"solve={r['solve_ms']:6.1f}ms "
            f"iters={r['iterations']:3d} "
            f"iter={r['iter_ms']:.3f}ms "
            f"err={r['pos_err_mm']:5.2f}mm "
            f"cpu={r['cpu_time_ms']:.1f}ms "
            f"mem={r['mem_kb']/1024:.0f}MB"
        )


def _print_aggregate(agg: Dict, label: str = ""):
    """Print aggregate stats table."""
    print(f"\n{'─'*65}")
    if label:
        print(f"  {label}")
    print(f"  n={agg['n']}  success={agg['success_pct']:.1f}%  mem={agg.get('mem_kb_mean',0)/1024:.1f}MB")
    print(f"{'─'*65}")
    for metric, unit in [
        ("solve_ms",   "ms"),
        ("robot_ms",   "ms"),
        ("setup_ms",   "ms"),
        ("loop_ms",    "ms"),
        ("iter_ms",    "ms/iter"),
        ("iterations", "iters"),
        ("cpu_time_ms","ms"),
        ("pos_err_mm", "mm"),
    ]:
        mn = agg.get(f"{metric}_mean",   0)
        md = agg.get(f"{metric}_median", 0)
        p9 = agg.get(f"{metric}_p95",    0)
        mx = agg.get(f"{metric}_max",    0)
        sd = agg.get(f"{metric}_std",    0)
        print(f"  {metric:<14} mean={mn:7.3f}  median={md:7.3f}"
              f"  p95={p9:7.3f}  max={mx:7.3f}  std={sd:6.3f}  [{unit}]")
    print(f"{'─'*65}")


def _print_sweep_summary(sweep: Dict[float, Dict]):
    """Print comparison table for dt sweep."""
    print(f"\n{'═'*75}")
    print("  dt sweep summary")
    print(f"{'═'*75}")
    header = f"  {'dt':>6}  {'success%':>8}  {'solve_ms':>9}  {'iters':>6}  {'iter_ms':>8}  {'cpu_ms':>7}"
    print(header)
    print(f"  {'─'*6}  {'─'*8}  {'─'*9}  {'─'*6}  {'─'*8}  {'─'*7}")
    for dt_val in sorted(sweep.keys()):
        s = sweep[dt_val]
        print(
            f"  {dt_val:6.4f}  {s.get('success_pct',0):8.1f}  "
            f"{s.get('solve_ms_mean',0):9.2f}  "
            f"{s.get('iterations_mean',0):6.1f}  "
            f"{s.get('iter_ms_mean',0):8.4f}  "
            f"{s.get('cpu_time_ms_mean',0):7.2f}"
        )
    print(f"{'═'*75}")


# ── CSV writing ────────────────────────────────────────────────────────────────

_CSV_FIELDS = [
    "run", "seed_idx", "dt",
    "target_x", "target_y", "target_z",
    "success", "pos_err_mm",
    "robot_ms", "setup_ms", "loop_ms", "solve_ms",
    "iterations", "iter_ms",
    "cpu_time_ms", "mem_kb",
    "j1", "j2", "j3", "j4", "j5", "j6", "j7",
]


def _write_csv_header(writer):
    writer.writerow(_CSV_FIELDS)


def _write_csv_row(writer, run_i: int, seed_idx: int, dt: float,
                   target_xyz: np.ndarray, r: Dict):
    joints = r.get("joints") or [0.0] * 7
    row = {
        "run":       run_i,
        "seed_idx":  seed_idx,
        "dt":        dt,
        "target_x":  round(float(target_xyz[0]), 6),
        "target_y":  round(float(target_xyz[1]), 6),
        "target_z":  round(float(target_xyz[2]), 6),
        "success":   r["success"],
        "pos_err_mm": round(r["pos_err_mm"], 4),
        "robot_ms":  round(r["robot_ms"],   3),
        "setup_ms":  round(r["setup_ms"],   3),
        "loop_ms":   round(r["loop_ms"],    3),
        "solve_ms":  round(r["solve_ms"],   3),
        "iterations": r["iterations"],
        "iter_ms":   round(r["iter_ms"],    5),
        "cpu_time_ms": round(r["cpu_time_ms"], 3),
        "mem_kb":    r["mem_kb"],
        "j1": round(joints[0], 6), "j2": round(joints[1], 6),
        "j3": round(joints[2], 6), "j4": round(joints[3], 6),
        "j5": round(joints[4], 6), "j6": round(joints[5], 6),
        "j7": round(joints[6], 6),
    }
    writer.writerow([row[f] for f in _CSV_FIELDS])


# ── Optional matplotlib plot ───────────────────────────────────────────────────

def _make_plot(rows: List[Dict], out_path: str, arm: str, dt: float):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[plot] matplotlib not available — skipping plot")
        return

    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle(f"Placo IK Profiler  arm={arm}  dt={dt}  n={len(rows)}", fontsize=13)

    metrics = [
        ("solve_ms",    "Total solve time (ms)",    axes[0, 0]),
        ("robot_ms",    "Robot build time (ms)",    axes[0, 1]),
        ("loop_ms",     "Solve loop time (ms)",     axes[0, 2]),
        ("iter_ms",     "Per-iteration time (ms)",  axes[1, 0]),
        ("iterations",  "Iterations used",          axes[1, 1]),
        ("pos_err_mm",  "Position error (mm)",      axes[1, 2]),
    ]

    run_idx = list(range(len(rows)))
    for key, label, ax in metrics:
        vals = [r[key] for r in rows]
        ax.plot(run_idx, vals, linewidth=0.8, color="steelblue")
        ax.axhline(float(np.mean(vals)),   color="red",    linestyle="--",
                   linewidth=0.8, label=f"mean={np.mean(vals):.2f}")
        ax.axhline(float(np.median(vals)), color="orange", linestyle=":",
                   linewidth=0.8, label=f"med={np.median(vals):.2f}")
        ax.set_title(label, fontsize=9)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_path, dpi=120)
    plt.close()
    print(f"[plot] Saved → {out_path}")


# ── Realtime: cached-robot solve ──────────────────────────────────────────────

def _build_cached_robot(urdf: str, arm: str):
    """
    Build and return a placo RobotWrapper once — reuse across realtime steps.
    This eliminates the ~5ms robot_build cost per step.
    """
    import placo
    robot = placo.RobotWrapper(urdf, placo.Flags.ignore_collisions)
    return robot


def solve_realtime_step(
    robot,           # pre-built cached RobotWrapper (or None → rebuild each step)
    urdf:       str,
    arm:        str,
    target_xyz: np.ndarray,
    target_R:   np.ndarray,
    seed:       List[float],
    max_iter:   int   = _DEFAULT_MAX_ITER,
    dt:         float = _DEFAULT_DT,
    no_rot:     bool  = False,
) -> Dict:
    """
    One realtime IK step — identical to solve_profiled but separates robot build
    from solver setup so that a cached robot can skip the build cost.

    If robot is None → always rebuild (same as solve_profiled).
    Returns same dict as solve_profiled + 'robot_was_cached' flag.
    """
    import placo

    joint_names = _JOINT_NAMES[arm]
    human_cfg   = _HUMAN_RIGHT if arm == "right" else _HUMAN_LEFT
    ee_link     = f"openarm_{arm}_link7"
    lo = [human_cfg[n][1] for n in joint_names]
    hi = [human_cfg[n][2] for n in joint_names]

    mem_kb_start = _mem_rss_kb()
    cpu_t0       = _cpu_seconds()
    wall_t0      = time.perf_counter()

    # ── Robot (maybe build, maybe reuse) ──────────────────────────────────────
    t_robot0 = time.perf_counter()
    if robot is None:
        robot = placo.RobotWrapper(urdf, placo.Flags.ignore_collisions)
        robot_was_cached = False
    else:
        robot_was_cached = True
    t_robot1  = time.perf_counter()
    robot_ms  = (t_robot1 - t_robot0) * 1000.0

    # ── Solver + task setup ───────────────────────────────────────────────────
    t_setup0 = time.perf_counter()
    solver   = placo.KinematicsSolver(robot)
    solver.enable_joint_limits(True)
    solver.enable_velocity_limits(False)
    solver.mask_fbase(True)
    solver.dt = dt

    for name, val in zip(joint_names, seed):
        robot.set_joint(name, val)
    robot.update_kinematics()

    pos_task = solver.add_position_task(ee_link, target_xyz)
    pos_task.configure("pos", "soft", _W_POS)
    if not no_rot:
        ori_task = solver.add_orientation_task(ee_link, target_R)
        ori_task.configure("ori", "soft", _W_ORI)
    pref_dict = {n: human_cfg[n][0] for n in joint_names}
    jt = solver.add_joints_task()
    jt.set_joints(pref_dict)
    jt.configure("naturalness", "soft", _W_JOINTS)
    reg = solver.add_regularization_task(1e-5)
    reg.configure("reg", "soft", 1.0)
    t_setup1 = time.perf_counter()
    setup_ms = (t_setup1 - t_setup0) * 1000.0

    # ── Iterative solve ───────────────────────────────────────────────────────
    t_loop0    = time.perf_counter()
    iters_used = 0
    for _ in range(max_iter):
        solver.solve(True)
        robot.update_kinematics()
        iters_used += 1
        T_ee  = robot.get_T_world_frame(ee_link)
        pos_e = float(np.linalg.norm(T_ee[:3, 3] - target_xyz))
        if pos_e < _POS_TOL:
            break
    t_loop1  = time.perf_counter()
    loop_ms  = (t_loop1 - t_loop0) * 1000.0

    joints  = [robot.get_joint(n) for n in joint_names]
    joints  = [max(l, min(h, q)) for q, l, h in zip(joints, lo, hi)]
    T_ee    = robot.get_T_world_frame(ee_link)
    pos_err = float(np.linalg.norm(T_ee[:3, 3] - target_xyz))

    cpu_time_ms   = (_cpu_seconds() - cpu_t0) * 1000.0
    solve_ms      = robot_ms + setup_ms + loop_ms
    iter_ms       = loop_ms / iters_used if iters_used > 0 else 0.0
    success       = int(pos_err < _POS_RELAX)

    return {
        "robot_ms":        robot_ms,
        "setup_ms":        setup_ms,
        "loop_ms":         loop_ms,
        "solve_ms":        solve_ms,
        "total_wall_ms":   (time.perf_counter() - wall_t0) * 1000.0,
        "iterations":      iters_used,
        "iter_ms":         iter_ms,
        "cpu_time_ms":     cpu_time_ms,
        "mem_kb":          mem_kb_start,
        "pos_err_mm":      pos_err * 1000.0,
        "success":         success,
        "joints":          joints,
        "robot_was_cached": robot_was_cached,
        "ee_xyz":          list(T_ee[:3, 3]),   # actual EE position after solve
    }


# ── Realtime: trajectory generators ───────────────────────────────────────────

def _traj_sine(t: float, t0: float, amp: float, freq: float = 0.3):
    """Sine wave along X axis. Returns absolute xyz offset from home."""
    ph = 2 * math.pi * freq * (t - t0)
    return np.array([amp * math.sin(ph), 0.0, 0.0])


def _traj_circle(t: float, t0: float, r: float, period: float = 6.0):
    """Circle in Y-Z plane. Returns absolute xyz offset from home."""
    a = 2 * math.pi / period * (t - t0)
    return np.array([0.0, r * math.cos(a) - r, r * math.sin(a)])


def _traj_lemniscate(t: float, t0: float, sc: float, period: float = 10.0):
    """Lemniscate (figure-8) in X-Y plane. Returns absolute xyz offset from home."""
    om    = 2 * math.pi / period
    theta = om * (t - t0)
    d     = 1.0 + math.sin(theta) ** 2
    x     = sc * math.cos(theta) / d
    y     = sc * math.sin(theta) * math.cos(theta) / d
    return np.array([x, y, 0.0])


_TRAJ_FN = {
    "sine":       _traj_sine,
    "circle":     _traj_circle,
    "lemniscate": _traj_lemniscate,
}


# ── Realtime: main simulation loop ────────────────────────────────────────────

_RT_CSV_FIELDS = [
    "step", "t_sec",
    "target_x", "target_y", "target_z",
    "actual_x",  "actual_y",  "actual_z",
    "track_err_mm",
    "success", "pos_err_mm",
    "solve_ms", "robot_ms", "setup_ms", "loop_ms",
    "iterations", "iter_ms",
    "cpu_time_ms", "mem_kb",
    "deadline_ms", "deadline_missed",
    "loop_wall_ms",
    "j1", "j2", "j3", "j4", "j5", "j6", "j7",
]


def run_realtime_profiled(
    urdf:        str,
    arm:         str,
    traj_name:   str   = "circle",
    rate_hz:     float = 20.0,
    duration_s:  float = 30.0,
    radius:      float = 0.04,
    max_iter:    int   = _DEFAULT_MAX_ITER,
    dt:          float = _DEFAULT_DT,
    no_rot:      bool  = False,
    cache_robot: bool  = False,
    warmup_s:    float = 1.0,
    csv_writer   = None,
    verbose:     bool  = False,
) -> List[Dict]:
    """
    Simulate a real-time IK control loop (no ROS required).

    - 每步使用上一步的 joints 作為 warm-start seed（模擬連續追蹤）
    - 軌跡在 home xyz 周圍以 radius 振盪
    - cache_robot=True: 只建立 RobotWrapper 一次（優化模式）
    - 返回每步的詳細 profiling dict list
    """
    import datetime

    deadline_ms  = 1000.0 / rate_hz
    step_sec     = 1.0 / rate_hz
    traj_fn      = _TRAJ_FN.get(traj_name, _traj_circle)
    home         = _HOME[arm]
    home_xyz     = np.array(home["xyz"])
    home_R       = _quat_to_rot(*home["quat"])
    last_joints  = list(home["joints"])

    # Optional cached robot (reused every step)
    cached_robot = _build_cached_robot(urdf, arm) if cache_robot else None

    mode_str = "cached" if cache_robot else "rebuild"
    print(f"\n  Realtime  arm={arm}  traj={traj_name}  {rate_hz:.0f}Hz  "
          f"radius={radius*100:.1f}cm  duration={duration_s:.0f}s  mode={mode_str}")
    print(f"  deadline={deadline_ms:.1f}ms  dt={dt}  max_iter={max_iter}")
    print(f"  {'step':>5}  {'t':>5}  {'solve':>7}  {'iters':>5}  "
          f"{'track':>7}  {'ik_err':>7}  {'cpu':>6}  {'ddl?':>5}")
    print(f"  {'─'*5}  {'─'*5}  {'─'*7}  {'─'*5}  "
          f"{'─'*7}  {'─'*7}  {'─'*6}  {'─'*5}")

    rows      = []
    n_missed  = 0
    n_fail    = 0
    t0        = time.perf_counter()
    step_idx  = 0

    while True:
        t_step_start = time.perf_counter()
        t_elapsed    = t_step_start - t0
        if t_elapsed > duration_s + warmup_s:
            break

        # Target: home + trajectory offset
        traj_off  = traj_fn(t_step_start, t0 + warmup_s, radius)
        target_xyz = home_xyz + traj_off

        # Solve IK with warm-start seed
        r = solve_realtime_step(
            cached_robot, urdf, arm,
            target_xyz, home_R,
            last_joints, max_iter, dt, no_rot,
        )

        # Update last_joints only on success → warm start for next step
        if r["success"] and r["joints"]:
            last_joints = r["joints"]
        else:
            n_fail += 1
            # Don't update last_joints — keep previous valid config as seed

        # Trajectory tracking error: actual EE vs desired target
        actual_xyz = np.array(r["ee_xyz"])
        track_err  = float(np.linalg.norm(actual_xyz - target_xyz)) * 1000.0

        is_warmup      = (t_elapsed < warmup_s)
        loop_wall_ms   = (time.perf_counter() - t_step_start) * 1000.0
        deadline_missed = int(loop_wall_ms > deadline_ms)
        if deadline_missed and not is_warmup:
            n_missed += 1

        if not is_warmup:
            step_idx += 1
            row_data = {
                "step":       step_idx,
                "t_sec":      round(t_elapsed - warmup_s, 4),
                "target_x":   round(float(target_xyz[0]), 5),
                "target_y":   round(float(target_xyz[1]), 5),
                "target_z":   round(float(target_xyz[2]), 5),
                "actual_x":   round(float(actual_xyz[0]), 5),
                "actual_y":   round(float(actual_xyz[1]), 5),
                "actual_z":   round(float(actual_xyz[2]), 5),
                "track_err_mm": round(track_err, 3),
                "success":    r["success"],
                "pos_err_mm": round(r["pos_err_mm"], 4),
                "solve_ms":   round(r["solve_ms"], 3),
                "robot_ms":   round(r["robot_ms"], 3),
                "setup_ms":   round(r["setup_ms"], 3),
                "loop_ms":    round(r["loop_ms"], 3),
                "iterations": r["iterations"],
                "iter_ms":    round(r["iter_ms"], 5),
                "cpu_time_ms": round(r["cpu_time_ms"], 3),
                "mem_kb":     r["mem_kb"],
                "deadline_ms": round(deadline_ms, 2),
                "deadline_missed": deadline_missed,
                "loop_wall_ms": round(loop_wall_ms, 3),
            }
            joints = r.get("joints") or [0.0] * 7
            for k, v in enumerate(joints[:7], 1):
                row_data[f"j{k}"] = round(v, 6)
            rows.append(row_data)

            if csv_writer:
                csv_writer.writerow([row_data.get(f, "") for f in _RT_CSV_FIELDS])

            # Print every step (or every 5th in non-verbose)
            if verbose or (step_idx % 5 == 0):
                tag = "✓" if r["success"] else "✗"
                dlm = "!" if deadline_missed else " "
                print(
                    f"  {step_idx:5d}  {t_elapsed-warmup_s:5.1f}  "
                    f"{r['solve_ms']:7.2f}  {r['iterations']:5d}  "
                    f"{track_err:7.2f}  {r['pos_err_mm']:7.2f}  "
                    f"{r['cpu_time_ms']:6.1f}  {dlm}{tag}"
                )

        # Rate-limit: sleep remainder of step budget
        elapsed_step = (time.perf_counter() - t_step_start)
        sleep_sec    = step_sec - elapsed_step
        if sleep_sec > 0:
            time.sleep(sleep_sec)

    total_steps = len(rows)
    sr  = sum(r["success"]          for r in rows) / max(total_steps, 1) * 100
    dlr = sum(r["deadline_missed"]  for r in rows) / max(total_steps, 1) * 100
    te  = float(np.mean([r["track_err_mm"] for r in rows])) if rows else 0.0
    sm  = float(np.median([r["solve_ms"] for r in rows])) if rows else 0.0

    print(f"\n  Done  steps={total_steps}  success={sr:.1f}%  "
          f"deadline_miss={dlr:.1f}%  track_err_mean={te:.2f}mm  "
          f"solve_median={sm:.2f}ms")
    return rows


def _make_plot_realtime(rows: List[Dict], out_path: str, arm: str,
                        rate_hz: float, traj_name: str, cache_robot: bool):
    """4-row plot specialized for realtime profiling results."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[plot] matplotlib not available — skipping")
        return

    if not rows:
        return

    t      = [r["t_sec"]      for r in rows]
    sm     = [r["solve_ms"]   for r in rows]
    iters  = [r["iterations"] for r in rows]
    track  = [r["track_err_mm"] for r in rows]
    cpu    = [r["cpu_time_ms"] for r in rows]
    ddl    = [r["deadline_ms"] for r in rows]
    missed = [r["deadline_missed"] for r in rows]
    succ   = [r["success"] for r in rows]
    tx     = [r["target_x"] for r in rows]
    ty     = [r["target_y"] for r in rows]
    tz     = [r["target_z"] for r in rows]
    ax_    = [r["actual_x"] for r in rows]
    ay_    = [r["actual_y"] for r in rows]
    az_    = [r["actual_z"] for r in rows]

    fig, axes = plt.subplots(4, 2, figsize=(15, 16))
    mode = "cached" if cache_robot else "rebuild"
    fig.suptitle(
        f"Placo Realtime Profiler  arm={arm}  {rate_hz:.0f}Hz  "
        f"traj={traj_name}  mode={mode}  n={len(rows)}",
        fontsize=12,
    )

    def _hline(ax, vals, **kw):
        ax.axhline(float(np.mean(vals)), linestyle="--", linewidth=0.9, **kw)

    # Row 0: Solve time + deadline
    ax = axes[0, 0]
    ax.plot(t, sm, color="steelblue", linewidth=0.7, label="solve_ms")
    ax.plot(t, ddl, color="red", linewidth=0.8, linestyle="--", label=f"deadline={ddl[0]:.0f}ms")
    ax.fill_between(t, sm, [d if m else s for s, d, m in zip(sm, ddl, missed)],
                    where=missed, color="red", alpha=0.25, label="deadline missed")
    _hline(ax, sm, color="orange", label=f"mean={np.mean(sm):.2f}ms")
    ax.set_title("Solve time vs deadline (ms)", fontsize=9)
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # Row 0: Iterations
    ax = axes[0, 1]
    ax.plot(t, iters, color="purple", linewidth=0.7)
    _hline(ax, iters, color="red", label=f"mean={np.mean(iters):.1f}")
    ax.set_title("Iterations per step", fontsize=9)
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # Row 1: Trajectory tracking error
    ax = axes[1, 0]
    ax.plot(t, track, color="tomato", linewidth=0.8)
    _hline(ax, track, color="darkred", label=f"mean={np.mean(track):.2f}mm")
    ax.set_title("Trajectory tracking error (mm)", fontsize=9)
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # Row 1: Success rate (rolling window)
    ax = axes[1, 1]
    win = max(1, len(succ) // 20)
    sr_roll = [
        sum(succ[max(0, i-win):i+1]) / min(i+1, win+1) * 100
        for i in range(len(succ))
    ]
    ax.plot(t, sr_roll, color="green", linewidth=0.8)
    ax.set_ylim(-5, 105)
    ax.set_title(f"Rolling success rate % (window={win})", fontsize=9)
    ax.grid(True, alpha=0.3)

    # Row 2: CPU time
    ax = axes[2, 0]
    ax.plot(t, cpu, color="darkorange", linewidth=0.7)
    _hline(ax, cpu, color="red", label=f"mean={np.mean(cpu):.1f}ms")
    ax.set_title("CPU time per step (ms)", fontsize=9)
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # Row 2: Deadline miss histogram
    ax = axes[2, 1]
    ax.bar(["hit", "miss"],
           [sum(1 for v in missed if not v), sum(missed)],
           color=["steelblue", "red"])
    ax.set_title(f"Deadline: {sum(missed)}/{len(missed)} missed ({sum(missed)/len(missed)*100:.1f}%)", fontsize=9)

    # Row 3: XYZ trajectory (target vs actual)
    for col, (tvals, avals, lbl, color) in enumerate([
        (tx, ax_, "X", "royalblue"),
        (ty, ay_, "Y", "seagreen"),
    ]):
        ax = axes[3, col]
        ax.plot(t, tvals, "--", color=color, linewidth=0.8, label=f"target {lbl}")
        ax.plot(t, avals,  "-", color=color, linewidth=0.8, alpha=0.6, label=f"actual {lbl}")
        ax.set_title(f"Trajectory {lbl} (m)", fontsize=9)
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_path, dpi=120)
    plt.close()
    print(f"[plot] Saved → {out_path}")


# ── Main ───────────────────────────────────────────────────────────────────────

def _parse_args():
    p = argparse.ArgumentParser(
        description="Placo IK deep profiler",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--arm",       default="right", choices=["right", "left"])
    p.add_argument("--benchmark", type=int, default=100, metavar="N",
                   help="Number of solves to profile (default: 100)")
    p.add_argument("--warmup",    type=int, default=10, metavar="N",
                   help="Warmup solves discarded before recording (default: 10)")
    p.add_argument("--max-iter",  type=int, default=_DEFAULT_MAX_ITER, dest="max_iter",
                   help=f"Max iterations per seed attempt (default: {_DEFAULT_MAX_ITER})")
    p.add_argument("--dt",        type=float, default=_DEFAULT_DT,
                   help=f"Placo solver dt (default: {_DEFAULT_DT})")
    p.add_argument("--random",    action="store_true",
                   help="Use random workspace targets (default: home target)")
    p.add_argument("--grid",      action="store_true",
                   help="Use structured grid targets (overrides --benchmark count)")
    p.add_argument("--profile",   action="store_true",
                   help="Print verbose per-solve output (robot/setup/loop breakdown)")
    p.add_argument("--no-rot",    action="store_true", dest="no_rot",
                   help="Position-only IK (skip orientation task)")
    p.add_argument("--csv",       default="", metavar="PATH",
                   help="Write results CSV to this path")
    p.add_argument("--plot",      default="", metavar="PATH",
                   help="Write timing plot PNG to this path")
    p.add_argument("--sweep-dt",  default="", dest="sweep_dt", metavar="LIST",
                   help="Comma-separated dt values to sweep, e.g. 0.005,0.010,0.020")
    p.add_argument("--seed-rand", type=int, default=42, dest="seed_rand",
                   help="Random seed for target generation (default: 42)")
    # ── Realtime mode ─────────────────────────────────────────────────────────
    p.add_argument("--realtime",    action="store_true",
                   help="Enable realtime ee_delta simulation mode (no ROS needed)")
    p.add_argument("--traj",        default="circle",
                   choices=["sine", "circle", "lemniscate"], dest="traj",
                   help="Trajectory shape for realtime mode (default: circle)")
    p.add_argument("--rate",        type=float, default=20.0,
                   help="Control loop Hz for realtime mode (default: 20)")
    p.add_argument("--duration",    type=float, default=30.0,
                   help="Duration seconds for realtime mode (default: 30)")
    p.add_argument("--radius",      type=float, default=0.04,
                   help="Trajectory amplitude/radius m (default: 0.04)")
    p.add_argument("--cache-robot", action="store_true", dest="cache_robot",
                   help="Realtime: reuse RobotWrapper across steps (optimization test)")
    return p.parse_args()


def main():
    args = _parse_args()

    # ── Find URDF ──────────────────────────────────────────────────────────────
    try:
        urdf = _find_urdf()
    except FileNotFoundError as e:
        print(f"[ERROR] {e}")
        sys.exit(1)

    arm        = args.arm
    n_bench    = args.benchmark
    n_warmup   = args.warmup
    max_iter   = args.max_iter
    dt         = args.dt
    verbose    = args.profile

    print(f"{'═'*65}")
    print(f"  Placo IK Profiler")
    print(f"  arm={arm}  dt={dt}  max_iter={max_iter}  warmup={n_warmup}")
    print(f"  urdf={os.path.basename(urdf)}")
    print(f"{'═'*65}")

    # ── Realtime mode ─────────────────────────────────────────────────────────
    if args.realtime:
        import datetime
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        mode_str = "cached" if args.cache_robot else "rebuild"

        csv_fh = csv_writer = None
        if args.csv:
            csv_fh     = open(args.csv, "w", newline="")
            csv_writer = csv.writer(csv_fh)
            csv_writer.writerow(_RT_CSV_FIELDS)
            print(f"  CSV → {args.csv}")

        rows = run_realtime_profiled(
            urdf         = urdf,
            arm          = arm,
            traj_name    = args.traj,
            rate_hz      = args.rate,
            duration_s   = args.duration,
            radius       = args.radius,
            max_iter     = max_iter,
            dt           = dt,
            no_rot       = args.no_rot,
            cache_robot  = args.cache_robot,
            warmup_s     = 1.0,
            csv_writer   = csv_writer,
            verbose      = args.profile,
        )

        if csv_fh:
            csv_fh.close()
            print(f"[done] Realtime CSV → {args.csv}")

        # Aggregate
        if rows:
            agg = _aggregate(rows)
            _print_aggregate(
                agg,
                label=f"REALTIME  arm={arm}  {args.rate:.0f}Hz  traj={args.traj}  mode={mode_str}",
            )
            # Extra realtime stats
            n_miss    = sum(r["deadline_missed"] for r in rows)
            dlr       = n_miss / len(rows) * 100
            te_mean   = float(np.mean([r["track_err_mm"] for r in rows]))
            te_p95    = float(np.percentile([r["track_err_mm"] for r in rows], 95))
            print(f"  deadline_miss : {n_miss}/{len(rows)} = {dlr:.1f}%  "
                  f"(budget={1000/args.rate:.1f}ms  solve_mean={agg.get('solve_ms_mean',0):.2f}ms)")
            print(f"  track_err_mm  : mean={te_mean:.2f}  p95={te_p95:.2f}")
            robot_pct = agg.get("robot_ms_mean", 0) / max(agg.get("solve_ms_mean", 1), 1) * 100
            print(f"  Time breakdown: robot_build={robot_pct:.0f}%  "
                  f"loop={100-robot_pct:.0f}%  mode={mode_str}")
            if not args.cache_robot and robot_pct > 60:
                print("  Tip: 試試 --cache-robot，可省去 robot_build 時間。")
            if dlr > 10:
                print(f"  Tip: deadline miss 率 {dlr:.0f}% 過高，"
                      f"考慮降低 --rate（目前 {args.rate:.0f}Hz）或改用 --cache-robot。")

        # Plot
        plot_path = args.plot or os.path.join(
            _today_dir(), f"placo_rt_{arm}_{args.traj}_{mode_str}_{ts}.png")
        _make_plot_realtime(rows, plot_path, arm, args.rate, args.traj, args.cache_robot)
        return

    # ── dt sweep mode ─────────────────────────────────────────────────────────
    if args.sweep_dt:
        dt_vals = [float(v.strip()) for v in args.sweep_dt.split(",")]
        print(f"  Sweeping dt values: {dt_vals}  n={n_bench}/each  warmup={n_warmup}")
        sweep = _run_sweep_dt(
            urdf, arm, dt_vals, n_bench, max_iter, n_warmup, args.random
        )
        _print_sweep_summary(sweep)
        return

    # ── Target selection ──────────────────────────────────────────────────────
    if args.grid:
        all_targets = _make_grid_targets(arm)
        print(f"  Grid targets: {len(all_targets)}")
    elif args.random:
        all_targets = _make_random_targets(arm, n_bench + n_warmup, args.seed_rand)
        print(f"  Random targets: {len(all_targets)}  (seed={args.seed_rand})")
    else:
        all_targets = _make_home_targets(arm, n_bench + n_warmup)
        home = _HOME[arm]["xyz"]
        print(f"  Home target: xyz={home}  n={len(all_targets)}")

    # ── CSV setup ──────────────────────────────────────────────────────────────
    csv_fh = csv_writer = None
    if args.csv:
        csv_fh     = open(args.csv, "w", newline="")
        csv_writer = csv.writer(csv_fh)
        _write_csv_header(csv_writer)
        print(f"  CSV → {args.csv}")

    # ── Main benchmark loop ────────────────────────────────────────────────────
    rows = []
    total     = len(all_targets)
    seeds_lib = _ARM_SEEDS[arm]
    home_seed = list(_HOME[arm]["joints"])

    print(f"\n  {'run':>4}  {'ms':>6}  {'iters':>5}  {'err_mm':>7}  {'cpu_ms':>6}  {'mem_MB':>6}")
    print(f"  {'─'*4}  {'─'*6}  {'─'*5}  {'─'*7}  {'─'*6}  {'─'*6}")

    for i, (xyz, R, _) in enumerate(all_targets):
        # Use home seed for first attempt (mirrors production use)
        r = solve_profiled(urdf, arm, xyz, R, home_seed, max_iter, dt, args.no_rot)

        is_warmup = (i < n_warmup)
        run_no    = i - n_warmup + 1

        if verbose:
            tag = "(warmup) " if is_warmup else ""
            _print_row(i + 1, dt, r, verbose=True)
        else:
            if not is_warmup or (i % 5 == 0):
                tag = "W" if is_warmup else " "
                print(
                    f"  [{tag}{run_no if not is_warmup else i+1:4d}]"
                    f"  {r['solve_ms']:6.1f}"
                    f"  {r['iterations']:5d}"
                    f"  {r['pos_err_mm']:7.2f}"
                    f"  {r['cpu_time_ms']:6.1f}"
                    f"  {r['mem_kb']/1024:6.1f}"
                )

        if not is_warmup:
            rows.append(r)
            if csv_writer:
                _write_csv_row(csv_writer, run_no, 0, dt, xyz, r)

        # Print progress every 50 runs
        if not is_warmup and run_no % 50 == 0:
            partial_agg = _aggregate(rows)
            ok_pct = partial_agg.get("success_pct", 0)
            ms_med = partial_agg.get("solve_ms_median", 0)
            print(f"\n  [progress {run_no}/{n_bench}]  success={ok_pct:.1f}%  solve_med={ms_med:.1f}ms\n")

    # ── Final aggregate ────────────────────────────────────────────────────────
    agg = _aggregate(rows)
    target_type = "grid" if args.grid else ("random" if args.random else "home")
    _print_aggregate(agg, label=f"arm={arm}  dt={dt}  targets={target_type}  n={len(rows)}")

    # ── Plot ───────────────────────────────────────────────────────────────────
    if args.plot:
        _make_plot(rows, args.plot, arm, dt)
    elif rows:
        # Auto-generate dated plot name (results/yyyymmdd/ when no --csv given)
        ts  = _timestamp()
        out_dir = os.path.dirname(args.csv) if args.csv else _today_dir()
        png = os.path.join(out_dir, f"placo_profile_{arm}_{ts}.png")
        _make_plot(rows, png, arm, dt)

    # ── Close CSV ──────────────────────────────────────────────────────────────
    if csv_fh:
        csv_fh.close()
        print(f"\n[done] CSV saved → {args.csv}")

    # ── Quick optimisation hint ────────────────────────────────────────────────
    robot_pct = agg.get("robot_ms_mean", 0) / max(agg.get("solve_ms_mean", 1), 1) * 100
    loop_pct  = agg.get("loop_ms_mean",  0) / max(agg.get("solve_ms_mean", 1), 1) * 100
    print(f"\n  Time breakdown:  robot_build={robot_pct:.0f}%  solve_loop={loop_pct:.0f}%")
    if robot_pct > 50:
        print("  Tip: robot build dominates — consider caching RobotWrapper between solves.")
    if agg.get("iterations_mean", 0) >= max_iter * 0.9:
        print(f"  Tip: most solves hit max_iter={max_iter} — try --max-iter {max_iter+100} or --dt {dt*1.5:.3f}")
    elif agg.get("iterations_mean", 0) < max_iter * 0.3:
        print(f"  Tip: solves converge fast (avg {agg.get('iterations_mean',0):.0f} iters) — try --max-iter {max(50, max_iter//2)}")


if __name__ == "__main__":
    main()
