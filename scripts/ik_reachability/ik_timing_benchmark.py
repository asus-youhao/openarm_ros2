#!/usr/bin/env python3
"""
IK Timing Benchmark
===================
Benchmarks MoveIt's /compute_ik service for every reachable point in a CSV.
**No robot motion** — only IK computation latency is measured.

Goal: evaluate whether the MoveIt IK solver is fast enough for real-time
teleoperation (AR/VR -> EE xyz/rpy).

Real-time budgets
-----------------
  @ 60 Hz  each IK must finish in  <16.7 ms
  @ 30 Hz  each IK must finish in  <33.3 ms
  @ 10 Hz  each IK must finish in <100   ms

Prerequisites (must be running):
    ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
    ros2 launch openarm_bimanual_moveit_config move_group_only.launch.py

Usage examples:
    # Right arm (uses natural -Y approach orientation)
    python3 ik_timing_benchmark.py --csv right_reachability_.csv --arm right

    # Left arm, 5 repeated warmup calls to stabilise ROS infrastructure
    python3 ik_timing_benchmark.py --csv left_reachability_.csv --arm left --warmup 5

    # Custom quaternion, save CSV + PNG automatically
    python3 ik_timing_benchmark.py --csv right_reachability_.csv --arm right \\
        --quat 0.0 0.0 -0.707 0.707 --out ik_times_right.csv

    # Bimanual: benchmark both CSVs, combine plot
    python3 ik_timing_benchmark.py \\
        --left  left_reachability_.csv  \\
        --right right_reachability_.csv

Options:
    --csv        CSV path (single-arm)
    --arm        left | right  (single-arm, default: right)
    --left       Left-arm CSV  (bimanual mode)
    --right      Right-arm CSV (bimanual mode)
    --quat       EE quaternion  qx qy qz qw  (overrides arm default)
    --warmup     Warmup calls before timing begins (default: 3)
    --max-pts    Limit number of points benchmarked (default: all)
    --out        Output CSV filename (default: ik_timing_<arm>.csv)
    --no-plot    Skip matplotlib plots
    --save-png   Save plots to PNG instead of showing interactively
    --no-sort    Do not sort by Z (default: sort low-to-high)
"""

import argparse
import csv
import sys
import threading
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")   # changed to non-interactive for servers; swap to "TkAgg" if on desktop
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
IK_SUCCESS = 1

_ARM_NATURAL_QUAT = {
    "right": (0.0, 0.0, -0.707, 0.707),   # EE -> -Y  (natural for right arm)
    "left":  (0.0, 0.0,  0.707, 0.707),   # EE -> +Y  (natural for left arm)
}

# ---------------------------------------------------------------------------
# Dexterous-hand grasp approach orientations
# These simulate realistic wrist orientations during AR/VR teleoperation.
# A dexterous hand needs to approach from many directions depending on object.
# ---------------------------------------------------------------------------
GRASP_ORIENTATIONS = {
    "side_neg_y":   (0.0,    0.0,   -0.707,  0.707),  # side approach -Y (natural right)
    "side_pos_y":   (0.0,    0.0,    0.707,  0.707),  # side approach +Y (natural left)
    "top_down":     (0.0,    0.707,  0.0,    0.707),  # top-down grasp (EE -> -Z)
    "front_pos_x":  (0.0,    0.0,    0.0,    1.0),    # frontal approach +X
    "front_neg_x":  (0.0,    0.0,    1.0,    0.0),    # frontal approach -X
    "bottom_up":    (0.0,   -0.707,  0.0,    0.707),  # bottom-up (shelf pickup)
}

RT_BUDGETS_MS = {
    "500 Hz": 1000.0 / 500.0,   #  2.0 ms  ← high-frequency real-time
    "200 Hz": 1000.0 / 200.0,   #  5.0 ms
    "100 Hz": 1000.0 / 100.0,   # 10.0 ms
    # "60 Hz":  1000.0 / 60.0,  # 16.7 ms  (commented out)
    # "30 Hz":  1000.0 / 30.0,  # 33.3 ms  (commented out)
    # "10 Hz":  1000.0 / 10.0,  # 100.0 ms (commented out)
}

_ARM_COLOR = {
    "right": "#E91E63",   # pink/red
    "left":  "#2196F3",   # blue
}


# ---------------------------------------------------------------------------
# CSV loader
# ---------------------------------------------------------------------------
def load_reachable_pts(csv_path: str, sort_by_z: bool = True):
    pts = []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            if row["reachable"] == "1":
                pts.append((float(row["x"]), float(row["y"]), float(row["z"])))
    if sort_by_z:
        pts.sort(key=lambda p: p[2])
    return pts


# ---------------------------------------------------------------------------
# IK benchmarker node
# ---------------------------------------------------------------------------
class IKBenchmarker(Node):
    """Wraps the /compute_ik service and times each call."""

    def __init__(self, arm: str, ik_timeout: float = 0.5):
        super().__init__(f"ik_timing_benchmark_{arm}")
        self.arm = arm
        self.ik_timeout = ik_timeout
        self.ee_link = f"openarm_{arm}_link7"
        self.group = f"{arm}_arm"
        self.joint_names = [f"openarm_{arm}_joint{i}" for i in range(1, 8)]
        self._latest_js: JointState | None = None

        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        self.client = self.create_client(GetPositionIK, "/compute_ik")
        self.get_logger().info("Waiting for /compute_ik service …")
        if not self.client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error(
                "/compute_ik not available — is move_group running?"
            )
            sys.exit(1)
        self.get_logger().info("IK service ready.")

    def _js_cb(self, msg: JointState):
        if msg.name:
            self._latest_js = msg

    def _seed_state(self) -> RobotState:
        rs = RobotState()
        js = self._latest_js
        if js is not None and js.name:
            rs.joint_state.name = list(js.name)
            rs.joint_state.position = list(js.position)
        else:
            rs.joint_state.name = self.joint_names
            rs.joint_state.position = [0.0] * 7
        return rs

    def query_timed(self, x: float, y: float, z: float,
                    quat: tuple) -> tuple[bool, float]:
        """
        Send one IK request and return (success, elapsed_ms).
        Wall-clock time measured around the full round-trip
        (call_async -> future.done).
        """
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group
        req.ik_request.ik_link_name = self.ee_link
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = int(self.ik_timeout * 1e9)
        req.ik_request.robot_state = self._seed_state()

        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.x = quat[0]
        ps.pose.orientation.y = quat[1]
        ps.pose.orientation.z = quat[2]
        ps.pose.orientation.w = quat[3]
        req.ik_request.pose_stamped = ps

        t_start = time.perf_counter()
        future = self.client.call_async(req)

        deadline = time.time() + self.ik_timeout + 2.0
        while not future.done():
            if time.time() > deadline:
                elapsed_ms = (time.perf_counter() - t_start) * 1000.0
                return False, elapsed_ms
            time.sleep(0.001)

        elapsed_ms = (time.perf_counter() - t_start) * 1000.0
        result = future.result()
        if result is None:
            return False, elapsed_ms
        return result.error_code.val == IK_SUCCESS, elapsed_ms


# ---------------------------------------------------------------------------
# Benchmark runner
# ---------------------------------------------------------------------------
def benchmark_arm(arm: str, pts: list, quat: tuple,
                  warmup: int, max_pts: int,
                  ik_timeout: float = 0.5) -> list[dict]:
    """
    Run the IK benchmark for one arm.
    Returns list of dicts: {x, y, z, success, elapsed_ms}
    """
    node = IKBenchmarker(arm, ik_timeout=ik_timeout)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    log = node.get_logger()

    # --- warmup ---
    if warmup > 0 and pts:
        wx, wy, wz = pts[0]
        log.info(f"[{arm}] Warming up with {warmup} calls …")
        for _ in range(warmup):
            node.query_timed(wx, wy, wz, quat)

    # --- benchmark ---
    total = min(len(pts), max_pts) if max_pts > 0 else len(pts)
    log.info(f"[{arm}] Benchmarking {total} waypoints …")

    results = []
    for i, (x, y, z) in enumerate(pts[:total]):
        ok, ms = node.query_timed(x, y, z, quat)
        results.append({"x": x, "y": y, "z": z, "success": ok, "elapsed_ms": ms})
        status = "OK  " if ok else "FAIL"
        log.info(f"[{arm}] [{i+1:4d}/{total}] ({x:+.3f},{y:+.3f},{z:+.3f}) "
                 f"{status}  {ms:6.1f} ms")

    executor.shutdown()
    node.destroy_node()
    return results


# ---------------------------------------------------------------------------
# Multi-orientation benchmark (dexterous hand: test all grasp directions)
# ---------------------------------------------------------------------------
def benchmark_arm_all_orientations(
    arm: str, pts: list, warmup: int, max_pts: int,
    ik_timeout: float = 0.5
) -> dict[str, list[dict]]:
    """
    For each grasp orientation, benchmark IK over all waypoints.
    Returns dict: {orientation_name: [result_dicts]}
    Each result dict has: {x, y, z, success, elapsed_ms, orientation}
    """
    node = IKBenchmarker(arm, ik_timeout=ik_timeout)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    log = node.get_logger()

    total_pts = min(len(pts), max_pts) if max_pts > 0 else len(pts)
    all_results = {}

    for orient_name, quat in GRASP_ORIENTATIONS.items():
        log.info(f"[{arm}] Orientation: {orient_name}  quat={quat}")

        # warmup
        if warmup > 0 and pts:
            wx, wy, wz = pts[0]
            for _ in range(warmup):
                node.query_timed(wx, wy, wz, quat)

        results = []
        for i, (x, y, z) in enumerate(pts[:total_pts]):
            ok, ms = node.query_timed(x, y, z, quat)
            results.append({
                "x": x, "y": y, "z": z,
                "success": ok, "elapsed_ms": ms,
                "orientation": orient_name,
            })
        ok_n   = sum(1 for r in results if r["success"])
        fail_n = total_pts - ok_n
        log.info(f"[{arm}]   {orient_name}: {ok_n} ok / {fail_n} fail")
        all_results[orient_name] = results

    executor.shutdown()
    node.destroy_node()
    return all_results


# ---------------------------------------------------------------------------
# Statistics helper
# ---------------------------------------------------------------------------
def print_stats(arm: str, results: list[dict]):
    times = [r["elapsed_ms"] for r in results if r["success"]]
    fails = sum(1 for r in results if not r["success"])
    total = len(results)

    if not times:
        print(f"\n[{arm}]  ALL {total} calls FAILED — no timing data.")
        return

    arr = np.array(times)
    print(f"\n{'='*50}")
    print(f" IK Timing Summary — {arm} arm  ({len(times)} ok / {fails} fail / {total} total)")
    print(f"{'='*50}")
    print(f"  Min      : {arr.min():.2f} ms")
    print(f"  Mean     : {arr.mean():.2f} ms")
    print(f"  Median   : {np.median(arr):.2f} ms")
    print(f"  P90      : {np.percentile(arr, 90):.2f} ms")
    print(f"  P95      : {np.percentile(arr, 95):.2f} ms")
    print(f"  P99      : {np.percentile(arr, 99):.2f} ms")
    print(f"  Max      : {arr.max():.2f} ms")
    print(f"  Std Dev  : {arr.std():.2f} ms")
    print()
    for label, budget_ms in RT_BUDGETS_MS.items():
        pct = np.mean(arr <= budget_ms) * 100.0
        print(f"  Within {label} budget ({budget_ms:.1f} ms) : {pct:.1f}%  of calls")
    print(f"{'='*50}\n")


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
# Multi-orientation statistics & plot (dexterous hand use-case)
# ---------------------------------------------------------------------------
def print_orient_stats(arm: str, all_results: dict[str, list[dict]]):
    """Print a comparison table: one row per grasp orientation."""
    print(f"\n{'='*72}")
    print(f" Dexterous-Hand IK Summary — {arm} arm  (all grasp orientations)")
    print(f"{'='*72}")
    header = f"  {'Orientation':<16} {'ok%':>5} {'mean':>7} {'P95':>7} {'max':>7}  RT-30Hz"
    print(header)
    print(f"  {'-'*65}")
    for orient_name, results in all_results.items():
        times = np.array([r["elapsed_ms"] for r in results if r["success"]])
        total = len(results)
        if len(times) == 0:
            print(f"  {orient_name:<16}  ALL FAIL")
            continue
        ok_pct = len(times) / total * 100
        within_100 = np.mean(times <= RT_BUDGETS_MS["100 Hz"]) * 100
        print(f"  {orient_name:<16} {ok_pct:5.1f}% "
              f"{times.mean():7.1f}ms {np.percentile(times,95):7.1f}ms "
              f"{times.max():7.1f}ms  {within_100:5.1f}% within 100Hz")
    print(f"{'='*72}\n")


def plot_orient_results(arm: str, all_results: dict[str, list[dict]],
                        save_png: str):
    """
    Multi-orientation plot for dexterous-hand AR/VR teleoperation.
    Shows: per-orientation box-plot of IK time + success rate bar chart.
    """
    orient_names  = list(all_results.keys())
    orient_colors = plt.cm.tab10(np.linspace(0, 0.9, len(orient_names)))

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle(
        f"Dexterous-Hand Grasp IK Benchmark — {arm} arm\n"
        "All 6 approach orientations (AR/VR teleoperation)",
        fontsize=14, fontweight="bold"
    )

    # ---- Panel 1: Box-plot of IK time per orientation ----
    ax = axes[0]
    data  = [np.array([r["elapsed_ms"] for r in all_results[o] if r["success"]])
             for o in orient_names]
    data  = [d if len(d) > 0 else np.array([np.nan]) for d in data]
    bp = ax.boxplot(data, patch_artist=True, vert=True, showfliers=True,
                    flierprops=dict(marker=".", markersize=3, alpha=0.4))
    for patch, color in zip(bp["boxes"], orient_colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    for label, ms in RT_BUDGETS_MS.items():
        rt_colors = {"500 Hz": "#7B1FA2", "200 Hz": "#E53935", "100 Hz": "#FB8C00"}
        # rt_colors = {"60 Hz": "#E53935", "30 Hz": "#FB8C00", "10 Hz": "#43A047"}  # old
        ax.axhline(ms, color=rt_colors[label], linestyle="--",
                   linewidth=1.2, alpha=0.85, label=f"{label} ({ms:.0f}ms)")
    ax.set_xticks(range(1, len(orient_names) + 1))
    ax.set_xticklabels(orient_names, rotation=30, ha="right", fontsize=8)
    ax.set_ylabel("IK time (ms)")
    ax.set_title("IK Time per Grasp Orientation")
    ax.legend(fontsize=7)
    ax.set_ylim(bottom=0)

    # ---- Panel 2: Success rate bar chart ----
    ax2 = axes[1]
    ok_pcts = []
    for o in orient_names:
        res = all_results[o]
        ok_pcts.append(sum(1 for r in res if r["success"]) / len(res) * 100)
    bars = ax2.bar(orient_names, ok_pcts, color=orient_colors, alpha=0.8,
                   edgecolor="white")
    ax2.set_ylim(0, 110)
    ax2.axhline(100, color="gray", linestyle=":", linewidth=0.8)
    for bar, pct in zip(bars, ok_pcts):
        ax2.text(bar.get_x() + bar.get_width() / 2, pct + 1,
                 f"{pct:.0f}%", ha="center", va="bottom", fontsize=8)
    ax2.set_xticklabels(orient_names, rotation=30, ha="right", fontsize=8)
    ax2.set_ylabel("IK success rate (%)")
    ax2.set_title("Reachability per Grasp Orientation")

    # ---- Panel 3: CDF overlay for all orientations ----
    ax3 = axes[2]
    for o, color in zip(orient_names, orient_colors):
        times = np.array([r["elapsed_ms"] for r in all_results[o] if r["success"]])
        if len(times) < 2:
            continue
        sorted_t = np.sort(times)
        cdf = np.arange(1, len(sorted_t) + 1) / len(sorted_t)
        ax3.plot(sorted_t, cdf * 100, color=color, linewidth=1.8, label=o)
    for label, ms in RT_BUDGETS_MS.items():
        rt_colors = {"500 Hz": "#7B1FA2", "200 Hz": "#E53935", "100 Hz": "#FB8C00"}
        # rt_colors = {"60 Hz": "#E53935", "30 Hz": "#FB8C00", "10 Hz": "#43A047"}  # old
        ax3.axvline(ms, color=rt_colors[label], linestyle="--",
                    linewidth=1.2, alpha=0.85, label=f"{label}")
    ax3.set_xlabel("IK time (ms)")
    ax3.set_ylabel("Cumulative % of successful calls")
    ax3.set_title("CDF — all orientations")
    ax3.legend(fontsize=7, ncol=2)
    ax3.set_ylim(0, 102)
    ax3.yaxis.set_major_formatter(
        matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:.0f}%")
    )
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_png, dpi=150, bbox_inches="tight")
    print(f"[plot] Saved → {save_png}")
    plt.close(fig)

    import subprocess, shutil
    for viewer in ("eog", "feh", "display", "xdg-open"):
        if shutil.which(viewer):
            subprocess.Popen([viewer, save_png],
                             stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
            print(f"[plot] Opened with {viewer}")
            break


# ---------------------------------------------------------------------------
def _add_budget_lines(ax, orientation="vertical"):
    """Draw vertical/horizontal dashed lines for each real-time budget."""
    colors = {"500 Hz": "#7B1FA2", "200 Hz": "#E53935", "100 Hz": "#FB8C00"}
    # colors = {"60 Hz": "#E53935", "30 Hz": "#FB8C00", "10 Hz": "#43A047"}  # old
    for label, ms in RT_BUDGETS_MS.items():
        if orientation == "vertical":
            ax.axvline(ms, color=colors[label], linestyle="--", linewidth=1.2,
                       alpha=0.8, label=f"{label}  ({ms:.1f} ms)")
        else:
            ax.axhline(ms, color=colors[label], linestyle="--", linewidth=1.2,
                       alpha=0.8, label=f"{label}  ({ms:.1f} ms)")


def plot_results(arm_results: dict[str, list[dict]],
                 save_png: str | None = None):
    """
    arm_results: {"right": [...], "left": [...]}  (may contain one or both)
    """
    fig = plt.figure(figsize=(18, 13))
    fig.suptitle("MoveIt IK Solver — Computation Time Benchmark\n"
                 "Real-time Teleoperation Evaluation", fontsize=15, fontweight="bold")

    gs = gridspec.GridSpec(2, 3, figure=fig,
                           left=0.07, right=0.97,
                           top=0.91, bottom=0.08,
                           wspace=0.35, hspace=0.40)

    ax_hist  = fig.add_subplot(gs[0, 0])      # Histogram
    ax_time  = fig.add_subplot(gs[0, 1:])     # Timeline
    ax_3d    = fig.add_subplot(gs[1, 0:2],    # 3-D scatter (first arm)
                               projection="3d")
    ax_cdf   = fig.add_subplot(gs[1, 2])      # CDF

    # ---- Gather all timing arrays ----
    for arm, results in arm_results.items():
        color = _ARM_COLOR.get(arm, "gray")
        times  = [r["elapsed_ms"] for r in results if r["success"]]
        f_idxs = [i for i, r in enumerate(results) if not r["success"]]
        if not times:
            continue
        arr = np.array(times)

        # ---- 1. Histogram ----
        ax_hist.hist(arr, bins=40, color=color, alpha=0.6, edgecolor="white",
                     label=f"{arm}  (n={len(arr)})")

        # ---- 2. Timeline ----
        ok_idx = [i for i, r in enumerate(results) if r["success"]]
        ax_time.plot(ok_idx, times, color=color, linewidth=0.7, alpha=0.6)
        # rolling mean (window=20)
        if len(times) >= 20:
            win = 20
            rm = np.convolve(arr, np.ones(win) / win, mode="valid")
            ax_time.plot(ok_idx[win - 1:], rm, color=color, linewidth=2.0,
                         label=f"{arm}  rolling-mean")
        # mark failures
        fail_times = [results[i]["elapsed_ms"] for i in f_idxs]
        if f_idxs:
            ax_time.scatter(f_idxs, fail_times, color="black", s=15,
                            zorder=5, label=f"{arm}  fail")

        # ---- 3. CDF ----
        sorted_t = np.sort(arr)
        cdf = np.arange(1, len(sorted_t) + 1) / len(sorted_t)
        ax_cdf.plot(sorted_t, cdf * 100.0, color=color, linewidth=2.0,
                    label=arm)

        # ---- 4. 3-D scatter (use the first arm encountered) ----
    first_arm = next(iter(arm_results))
    first_results = arm_results[first_arm]
    xs = [r["x"] for r in first_results if r["success"]]
    ys = [r["y"] for r in first_results if r["success"]]
    zs = [r["z"] for r in first_results if r["success"]]
    ts = [r["elapsed_ms"] for r in first_results if r["success"]]
    if xs:
        sc = ax_3d.scatter(xs, ys, zs, c=ts, cmap="plasma",
                           s=18, alpha=0.75, vmin=0,
                           vmax=np.percentile(ts, 98))
        fig.colorbar(sc, ax=ax_3d, shrink=0.55, pad=0.01,
                     label="IK time (ms)")
        ax_3d.set_xlabel("X (m)", labelpad=6, fontsize=9)
        ax_3d.set_ylabel("Y (m)", labelpad=6, fontsize=9)
        ax_3d.set_zlabel("Z (m)", labelpad=6, fontsize=9)
        ax_3d.set_title(f"3-D Workspace — IK time heatmap\n({first_arm} arm)",
                        fontsize=10)

    # ---- Decorate histogram ----
    _add_budget_lines(ax_hist, "vertical")
    ax_hist.set_xlabel("IK computation time (ms)", fontsize=10)
    ax_hist.set_ylabel("Count", fontsize=10)
    ax_hist.set_title("IK Time Distribution", fontsize=11)
    ax_hist.legend(fontsize=8)

    # ---- Decorate timeline ----
    _add_budget_lines(ax_time, "horizontal")
    ax_time.set_xlabel("Waypoint index", fontsize=10)
    ax_time.set_ylabel("IK time (ms)", fontsize=10)
    ax_time.set_title("IK Time per Waypoint  (with rolling mean)", fontsize=11)
    ax_time.legend(fontsize=8, loc="upper right")
    ax_time.set_ylim(bottom=0)

    # ---- Decorate CDF ----
    for label, budget_ms in RT_BUDGETS_MS.items():
        colors_rt = {"500 Hz": "#7B1FA2", "200 Hz": "#E53935", "100 Hz": "#FB8C00"}
        # colors_rt = {"60 Hz": "#E53935", "30 Hz": "#FB8C00", "10 Hz": "#43A047"}  # old
        ax_cdf.axvline(budget_ms, color=colors_rt[label], linestyle="--",
                       linewidth=1.2, alpha=0.85,
                       label=f"{label}  ({budget_ms:.1f} ms)")
    ax_cdf.set_xlabel("IK time (ms)", fontsize=10)
    ax_cdf.set_ylabel("Cumulative % of calls", fontsize=10)
    ax_cdf.set_title("CDF — real-time budget analysis", fontsize=11)
    ax_cdf.legend(fontsize=8)
    ax_cdf.set_ylim(0, 102)
    ax_cdf.grid(True, alpha=0.3)
    ax_cdf.yaxis.set_major_formatter(
        matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:.0f}%")
    )

    # Shared annotation in histogram
    for arm, results in arm_results.items():
        times = [r["elapsed_ms"] for r in results if r["success"]]
        if times:
            arr = np.array(times)
            txt = (f"{arm}: mean={arr.mean():.1f}ms  "
                   f"P95={np.percentile(arr,95):.1f}ms  "
                   f"max={arr.max():.1f}ms")
            ax_hist.text(0.98, 0.98 - list(arm_results.keys()).index(arm) * 0.08,
                         txt, transform=ax_hist.transAxes,
                         ha="right", va="top", fontsize=7,
                         bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.7))

    if save_png:
        plt.savefig(save_png, dpi=150, bbox_inches="tight")
        print(f"[plot] Saved to {save_png}")
    else:
        plt.show()
    plt.close(fig)


# ---------------------------------------------------------------------------
# Save timing CSV
# ---------------------------------------------------------------------------
def save_timing_csv(results: list[dict], path: str):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["x", "y", "z", "success", "elapsed_ms"])
        w.writeheader()
        for r in results:
            w.writerow({
                "x": f"{r['x']:.4f}",
                "y": f"{r['y']:.4f}",
                "z": f"{r['z']:.4f}",
                "success": int(r["success"]),
                "elapsed_ms": f"{r['elapsed_ms']:.3f}",
            })
    print(f"[csv] Timing results saved to {path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="IK Timing Benchmark — evaluate MoveIt IK latency for real-time teleoperation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    # Single-arm
    parser.add_argument("--csv",  default=None, help="Reachable CSV path (single-arm)")
    parser.add_argument("--arm",  default="right", choices=["left", "right"])
    # Bimanual
    parser.add_argument("--left",  default=None, metavar="CSV")
    parser.add_argument("--right", default=None, metavar="CSV")
    # Options
    parser.add_argument("--quat",     type=float, nargs=4, default=None,
                        metavar=("QX", "QY", "QZ", "QW"))
    parser.add_argument("--warmup",   type=int,   default=3,
                        help="Warmup calls before timing (default: 3)")
    parser.add_argument("--max-pts",  type=int,   default=0,
                        help="Limit waypoints benchmarked (default: all)")
    parser.add_argument("--out",      default=None,
                        help="Output timing CSV (default: ik_timing_<arm>.csv)")
    parser.add_argument("--no-plot",  action="store_true")
    parser.add_argument("--save-png", default=None,
                        help="Save plot PNG to this path instead of showing")
    parser.add_argument("--no-sort",  action="store_true")
    parser.add_argument("--ik-timeout", type=float, default=0.5,
                        help="IK service timeout per call in seconds (default: 0.5)")
    parser.add_argument("--all-orient", action="store_true",
                        help="Test ALL 6 dexterous-hand grasp orientations (overrides --quat)")
    args, ros_args = parser.parse_known_args()

    sort_by_z = not args.no_sort
    bimanual  = args.left is not None and args.right is not None

    if not bimanual and args.csv is None:
        print("[ERROR] Specify --csv (single-arm) or --left + --right (bimanual)")
        sys.exit(1)

    rclpy.init(args=ros_args)

    arm_results = {}

    try:
        if bimanual:
            for arm, csv_path in [("left", args.left), ("right", args.right)]:
                if not Path(csv_path).exists():
                    print(f"[ERROR] File not found: {csv_path}")
                    sys.exit(1)
                pts = load_reachable_pts(csv_path, sort_by_z=sort_by_z)
                quat = tuple(args.quat) if args.quat else _ARM_NATURAL_QUAT[arm]
                print(f"\n[{arm}] {len(pts)} reachable points  quat={quat}")
                results = benchmark_arm(arm, pts, quat,
                                        warmup=args.warmup,
                                        max_pts=args.max_pts,
                                        ik_timeout=args.ik_timeout)
                arm_results[arm] = results
                print_stats(arm, results)
                out_csv = args.out or f"ik_timing_{arm}.csv"
                save_timing_csv(results, out_csv)
        else:
            arm      = args.arm
            csv_path = args.csv
            if not Path(csv_path).exists():
                print(f"[ERROR] File not found: {csv_path}")
                sys.exit(1)
            pts = load_reachable_pts(csv_path, sort_by_z=sort_by_z)
            if not pts:
                print("[ERROR] CSV has no reachable points")
                sys.exit(1)

            if args.all_orient:
                # ---- Multi-orientation mode for dexterous hand ----
                print(f"[{arm}] {len(pts)} pts — testing all {len(GRASP_ORIENTATIONS)} grasp orientations")
                all_or = benchmark_arm_all_orientations(
                    arm, pts,
                    warmup=args.warmup,
                    max_pts=args.max_pts,
                    ik_timeout=args.ik_timeout,
                )
                print_orient_stats(arm, all_or)
                # save one CSV per orientation
                for o_name, res in all_or.items():
                    csv_out = f"ik_timing_{arm}_{o_name}.csv"
                    save_timing_csv(res, csv_out)
                if not args.no_plot:
                    png_path = args.save_png or f"ik_timing_{arm}_all_orient.png"
                    plot_orient_results(arm, all_or, save_png=png_path)
            else:
                # ---- Single-orientation mode ----
                quat = tuple(args.quat) if args.quat else _ARM_NATURAL_QUAT[arm]
                print(f"[{arm}] {len(pts)} reachable points  quat={quat}")
                results = benchmark_arm(arm, pts, quat,
                                        warmup=args.warmup,
                                        max_pts=args.max_pts,
                                        ik_timeout=args.ik_timeout)
                arm_results[arm] = results
                print_stats(arm, results)
                out_csv = args.out or f"ik_timing_{arm}.csv"
                save_timing_csv(results, out_csv)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        rclpy.shutdown()

    # --- Plot (single-orientation mode only) ---
    if not args.no_plot and arm_results and not getattr(args, 'all_orient', False):
        png_path = args.save_png
        if png_path is None:
            arms_str = "_".join(arm_results.keys())
            png_path = f"ik_timing_{arms_str}.png"

        print(f"\n[plot] Generating plots → {png_path}")
        plot_results(arm_results, save_png=png_path)

        import subprocess, shutil
        for viewer in ("eog", "feh", "display", "xdg-open"):
            if shutil.which(viewer):
                subprocess.Popen([viewer, png_path],
                                  stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL)
                print(f"[plot] Opened with {viewer}")
                break


if __name__ == "__main__":
    main()
