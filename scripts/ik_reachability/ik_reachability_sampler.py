#!/usr/bin/env python3
"""
EE Reachability Sampler using /compute_ik

Samples a 3-D XYZ grid and calls MoveIt's /compute_ik service for each point.
Saves reachable points to a CSV and optionally shows a 3-D scatter plot.

Prerequisites (must be running):
  ros2 launch openarm_bringup openarm_o6_bimanual.launch.py
  ros2 launch openarm_bimanual_moveit_config move_group_only.launch.py

Arm geometry (world frame, all joints at 0):
  Right arm base: (0.000, -0.031, 0.698)  roll=-90deg  EE home: (0, -0.153, 0.262)
  Left  arm base: (0.000,  0.031, 0.698)  roll=+90deg  EE home: (0,  0.153, 0.262)

Usage:
  python3 ik_reachability_sampler.py                           # right arm, auto range
  python3 ik_reachability_sampler.py --arm left  --step 0.06
  python3 ik_reachability_sampler.py --arm both  --step 0.08  # both arms, combined plot
  python3 ik_reachability_sampler.py --arm right --step 0.05 --out right_reach.csv
  python3 ik_reachability_sampler.py --arm right --step 0.08 --no-plot
  # Custom range override (single arm only):
  python3 ik_reachability_sampler.py --xrange -0.5 0.5 --yrange -0.9 0.1 --zrange -0.3 0.8

Options:
  --arm        left | right | both     (default: right)
  --step       grid resolution in m    (default: 0.07)
  --xrange     min max                 (single arm only; override auto range)
  --yrange     min max                 (single arm only; override auto range)
  --zrange     min max                 (single arm only; override auto range)
  --out        output CSV filename     (single arm only; default: <arm>_reachability.csv)
  --no-plot    skip matplotlib
  --timeout    IK service timeout (s)  (default: 0.5)
"""

import argparse
import csv
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

# IK error codes (moveit_msgs/MoveItErrorCodes)
IK_SUCCESS = 1

# Strict colour assignment: left=blue, right=red
_ARM_COLOR = {"left": "#2196F3", "right": "#E91E63"}

# Per-arm default workspace ranges (world frame)
# Derived from TF: right base (0,-0.031,0.698) roll=-90°, left base (0,0.031,0.698) roll=+90°
# Arm reach ~0.7m.  Ranges are generous to capture full workspace.
_DEFAULT_RANGES = {
    "right": {"x": (-0.65, 0.65), "y": (-0.85,  0.15), "z": (-0.45, 0.75)},
    "left":  {"x": (-0.65, 0.65), "y": (-0.15,  0.85), "z": (-0.45, 0.75)},
}

# Human-readable error code table for diagnostics
_IK_ERROR_NAMES = {
    1:  "SUCCESS",
    -1: "FAILURE",
    -2: "PLANNING_FAILED",
    -3: "INVALID_MOTION_PLAN",
    -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -10: "CONTROL_FAILED",
    -11: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -12: "TIMED_OUT",
    -13: "PREEMPTED",
    -14: "START_STATE_IN_COLLISION",
    -15: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -16: "GOAL_IN_COLLISION",
    -17: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -18: "GOAL_CONSTRAINTS_VIOLATED",
    -19: "INVALID_GROUP_NAME",
    -20: "INVALID_GOAL_CONSTRAINTS",
    -21: "INVALID_ROBOT_STATE",
    -22: "INVALID_LINK_NAME",
    -23: "INVALID_OBJECT_NAME",
    -31: "NO_IK_SOLUTION",
}

# 6 axis-aligned EE orientations for position-only reachability.
# A point is "reachable" if IK succeeds with ANY of these approach directions.
# Format: (qx, qy, qz, qw)
_ORIENTATIONS_6 = [
    ( 0.0,    0.0,    0.0,    1.0  ),  # EE -> +X (identity)
    ( 0.0,    0.0,    1.0,    0.0  ),  # EE -> -X (180 around Z)
    ( 0.0,    0.0,    0.707,  0.707),  # EE -> +Y (+90 around Z)
    ( 0.0,    0.0,   -0.707,  0.707),  # EE -> -Y (-90 around Z, natural for right arm)
    ( 0.0,   -0.707,  0.0,    0.707),  # EE -> +Z (-90 around Y)
    ( 0.0,    0.707,  0.0,    0.707),  # EE -> -Z (+90 around Y)
]

# Per-arm natural approach orientation (single best direction for fast scan)
_ARM_NATURAL_ORIENT = {
    "right": (0.0, 0.0, -0.707, 0.707),  # EE -> -Y
    "left":  (0.0, 0.0,  0.707, 0.707),  # EE -> +Y
}


class IKSampler(Node):
    def __init__(self, arm: str, ik_timeout: float, avoid_collisions: bool = True):
        # Unique node name per arm so --arm both doesn't create a name conflict
        super().__init__(f"ik_reachability_sampler_{arm}")
        self.arm = arm
        self.ik_timeout = ik_timeout
        self.avoid_collisions = avoid_collisions

        # EE link: arm's joint7 output link
        self.ee_link = f"openarm_{arm}_link7"
        self.group = f"{arm}_arm"

        # Joint names for seed state (home = all zeros)
        self.joint_names = [
            f"openarm_{arm}_joint1",
            f"openarm_{arm}_joint2",
            f"openarm_{arm}_joint3",
            f"openarm_{arm}_joint4",
            f"openarm_{arm}_joint5",
            f"openarm_{arm}_joint6",
            f"openarm_{arm}_joint7",
        ]
        self._latest_joint_state: JointState | None = None
        self._diag_count = 0  # number of calls printed for diagnostics

        # TF listener for arm-chain visualisation
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to /joint_states to get current positions as IK seed
        self.create_subscription(
            JointState, "/joint_states", self._js_cb, 10
        )

        self.client = self.create_client(GetPositionIK, "/compute_ik")
        self.get_logger().info("Waiting for /compute_ik service...")
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/compute_ik not available — is move_group running?")
            sys.exit(1)
        self.get_logger().info("IK service ready.")

    def _js_cb(self, msg: JointState):
        if msg.name:  # ignore empty messages
            self._latest_joint_state = msg

    def _make_seed_state(self) -> RobotState:
        """Build a RobotState seed from ALL latest /joint_states.

        Passing ALL joints (both arms, hands, etc.) lets move_group correctly
        evaluate self-collisions between the arms when avoid_collisions=True.
        Order of joints in the message does not matter — dict lookup is used.
        """
        rs = RobotState()
        js = self._latest_joint_state
        if js is not None and js.name:
            rs.joint_state.name = list(js.name)
            rs.joint_state.position = list(js.position)
        else:
            # Fallback: zero out only the arm we know about
            rs.joint_state.name = self.joint_names
            rs.joint_state.position = [0.0] * len(self.joint_names)
        return rs

    def query(self, x: float, y: float, z: float,
              quat: tuple = (0.0, 0.0, 0.0, 1.0)) -> bool:
        """Return True if IK solution found for (x,y,z) with the given (qx,qy,qz,qw) orientation."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group
        req.ik_request.ik_link_name = self.ee_link
        req.ik_request.avoid_collisions = self.avoid_collisions
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = int(self.ik_timeout * 1e9)
        req.ik_request.robot_state = self._make_seed_state()

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

        # Use call_async + polling loop so the background executor can process
        # the service response callback without contention.
        future = self.client.call_async(req)
        deadline = time.time() + self.ik_timeout + 1.5
        while not future.done():
            if time.time() > deadline:
                self.get_logger().warn(f"IK timeout ({x:.2f},{y:.2f},{z:.2f})")
                return False
            time.sleep(0.005)

        result = future.result()
        if result is None:
            return False

        val = result.error_code.val
        # Print raw error code for the first 8 calls (diagnostics)
        if self._diag_count < 8:
            name = _IK_ERROR_NAMES.get(val, f"code={val}")
            reachable_str = "REACH" if val == IK_SUCCESS else "FAIL "
            print(f"  [diag #{self._diag_count+1}] ({x:.2f},{y:.2f},{z:.2f})  "
                  f"error_code={val} ({name})  -> {reachable_str}")
            self._diag_count += 1

        return val == IK_SUCCESS

    def query_any(self, x: float, y: float, z: float,
                  orientations=None) -> bool:
        """Position-only reachability: True if IK succeeds with ANY of the given orientations.
        If orientations is None, uses the module-level _ORIENTATIONS_6.
        """
        orients = orientations if orientations is not None else _ORIENTATIONS_6
        for q in orients:
            if self.query(x, y, z, quat=q):
                return True
        return False

    def get_arm_chain(self):
        """Return list of (x, y, z) for the arm link chain in world frame (for plotting)."""
        links = [
            "openarm_body_link0",
            f"openarm_{self.arm}_link0",
            f"openarm_{self.arm}_link1",
            f"openarm_{self.arm}_link2",
            f"openarm_{self.arm}_link3",
            f"openarm_{self.arm}_link4",
            f"openarm_{self.arm}_link5",
            f"openarm_{self.arm}_link6",
            f"openarm_{self.arm}_link7",
        ]
        pts = [(0.0, 0.0, 0.0)]  # world origin
        for link in links:
            try:
                tf = self.tf_buffer.lookup_transform(
                    "world", link,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=1.0),
                )
                t = tf.transform.translation
                pts.append((t.x, t.y, t.z))
            except Exception:
                pass
        return pts


def build_grid(xrange, yrange, zrange, step):
    xs = np.arange(xrange[0], xrange[1] + step * 0.5, step)
    ys = np.arange(yrange[0], yrange[1] + step * 0.5, step)
    zs = np.arange(zrange[0], zrange[1] + step * 0.5, step)
    pts = [(x, y, z) for x in xs for y in ys for z in zs]
    return pts


def build_refine_grid(coarse_csv: str, fine_step: float) -> list:
    """Coarse-to-fine grid: only sample within coarse_step/2 of each reachable point.

    Strategy:
      1. Load coarse CSV, collect reachable (x0, y0, z0) and infer coarse step.
      2. Around each reachable point, generate a fine sub-grid of radius=coarse_step/2.
      3. Deduplicate to avoid re-querying the same voxel.

    Speed-up factor: ~(coarse_step/fine_step)^3 * reachable_fraction
    Example: coarse=0.08, fine=0.01 -> 8^3=512 pts per seed, vs full grid of ~1.5M
    """
    import csv as _csv
    reachable_seeds = []
    xs_all, ys_all, zs_all = [], [], []
    with open(coarse_csv, newline="") as f:
        for row in _csv.DictReader(f):
            x, y, z = float(row["x"]), float(row["y"]), float(row["z"])
            xs_all.append(x); ys_all.append(y); zs_all.append(z)
            if row["reachable"] == "1":
                reachable_seeds.append((x, y, z))

    if not reachable_seeds:
        print(f"[WARN] --refine: no reachable points found in {coarse_csv}")
        return []

    # Infer coarse step from unique sorted X values spacing
    uniq_x = sorted(set(round(v, 6) for v in xs_all))
    if len(uniq_x) >= 2:
        coarse_step = round(uniq_x[1] - uniq_x[0], 6)
    else:
        # fallback: estimate from Z
        uniq_z = sorted(set(round(v, 6) for v in zs_all))
        coarse_step = round(uniq_z[1] - uniq_z[0], 6) if len(uniq_z) >= 2 else 0.08

    half = coarse_step / 2.0
    offsets = np.arange(-half, half + fine_step * 0.5, fine_step)

    seen = set()
    pts = []
    for x0, y0, z0 in reachable_seeds:
        for dx in offsets:
            for dy in offsets:
                for dz in offsets:
                    x = round(x0 + dx, 6)
                    y = round(y0 + dy, 6)
                    z = round(z0 + dz, 6)
                    key = (round(x / fine_step), round(y / fine_step), round(z / fine_step))
                    if key not in seen:
                        seen.add(key)
                        pts.append((x, y, z))

    print(f"[refine] coarse step inferred: {coarse_step:.4f} m  "
          f"seeds: {len(reachable_seeds)}  fine pts: {len(pts)}  "
          f"(vs full grid: too large)")
    return pts


def save_csv(path: str, reachable, unreachable):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "z", "reachable"])
        for x, y, z in reachable:
            w.writerow([f"{x:.4f}", f"{y:.4f}", f"{z:.4f}", "1"])
        for x, y, z in unreachable:
            w.writerow([f"{x:.4f}", f"{y:.4f}", f"{z:.4f}", "0"])
    print(f"Saved {len(reachable)} reachable + {len(unreachable)} unreachable -> {path}")


# ---------------------------------------------------------------------------
# Shared chain-drawing helper
# ---------------------------------------------------------------------------
_JOINT_LABELS = ["world", "body", "link0", "J1", "J2", "J3", "J4", "J5", "J6", "EE"]


def _draw_chain(ax, chain_pts, color, arm):
    if not chain_pts or len(chain_pts) < 2:
        return
    cx = [p[0] for p in chain_pts]
    cy = [p[1] for p in chain_pts]
    cz = [p[2] for p in chain_pts]
    ax.plot(cx, cy, cz, "-o", color=color, linewidth=3,
            markersize=6, label=f"{arm} TF chain", zorder=10)
    for i, (x, y, z) in enumerate(chain_pts):
        lbl = _JOINT_LABELS[i] if i < len(_JOINT_LABELS) else str(i)
        ax.text(x, y, z, f"  {lbl}", fontsize=7, color=color)
    ax.scatter([cx[-1]], [cy[-1]], [cz[-1]], s=180, marker="*",
               color=color, zorder=11, label=f"{arm} EE (TF)")


# ---------------------------------------------------------------------------
# Single-arm plot
# ---------------------------------------------------------------------------
def plot_results(arm: str, reachable, unreachable, step: float, chain_pts=None):
    try:
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError:
        print("[WARN] matplotlib not installed, skipping plot.")
        return

    arm_color = _ARM_COLOR[arm]  # left=blue, right=red
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    s = max(10, int(200 * step))

    if unreachable:
        ux, uy, uz = zip(*unreachable)
        ax.scatter(ux, uy, uz, s=s, c="#BDBDBD", alpha=0.15, label="unreachable", depthshade=False)
    if reachable:
        rx, ry, rz = zip(*reachable)
        ax.scatter(rx, ry, rz, s=s, c=arm_color, alpha=0.8, label="reachable", depthshade=True)

    _draw_chain(ax, chain_pts, arm_color, arm)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    side_color_note = "blue" if arm == "left" else "red"
    ax.set_title(
        f"OpenArm V10 – {arm.capitalize()} Arm EE Reachability  [{side_color_note}]\n"
        f"EE = openarm_{arm}_link7  |  grid step = {step:.3f} m  |  "
        f"{len(reachable)} / {len(reachable)+len(unreachable)} reachable"
    )
    ax.legend()
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Combined bimanual plot
# ---------------------------------------------------------------------------
def plot_both_results(results: dict, step: float):
    """Both arms on one axes.  left=blue  right=red."""
    try:
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError:
        print("[WARN] matplotlib not installed, skipping plot.")
        return

    fig = plt.figure(figsize=(13, 9))
    ax = fig.add_subplot(111, projection="3d")
    s = max(10, int(200 * step))

    total_reach = 0
    total_pts = 0

    for arm, data in results.items():
        color       = _ARM_COLOR[arm]  # left=blue, right=red
        reachable   = data["reachable"]
        unreachable = data["unreachable"]
        chain_pts   = data["chain"]
        total_reach += len(reachable)
        total_pts   += len(reachable) + len(unreachable)

        if unreachable:
            ux, uy, uz = zip(*unreachable)
            ax.scatter(ux, uy, uz, s=s, c="#BDBDBD", alpha=0.06, depthshade=False)
        if reachable:
            rx, ry, rz = zip(*reachable)
            ax.scatter(rx, ry, rz, s=s, c=color, alpha=0.75,
                       label=f"{arm} reachable ({len(reachable)})", depthshade=True)
        _draw_chain(ax, chain_pts, color, arm)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(
        f"OpenArm V10 – Bimanual EE Reachability  [left=blue  right=red]\n"
        f"grid step = {step:.3f} m  |  "
        f"{total_reach} / {total_pts} total reachable"
    )
    ax.legend()
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Per-arm scan (shared by single and bimanual modes)
# ---------------------------------------------------------------------------
def _run_single_arm(arm: str, xrange, yrange, zrange, step: float,
                    out_file: str, ik_timeout: float, avoid_collisions: bool,
                    show_diag: bool = True, refine_csv: str = None,
                    n_orientations: int = 6):
    """Scan one arm and return (reachable, unreachable, chain_pts).
    Assumes rclpy.init() has already been called."""
    node = IKSampler(arm=arm, ik_timeout=ik_timeout, avoid_collisions=avoid_collisions)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    print(f"\n[{arm.upper()}] Waiting for /joint_states (up to 5s)...")
    deadline = time.time() + 5.0
    while time.time() < deadline and node._latest_joint_state is None:
        time.sleep(0.1)
    if node._latest_joint_state is not None:
        js = node._latest_joint_state
        print(f"  Got {len(js.name)} joints. First 4: {list(js.name[:4])} ...")
        # Warn if arm joints are missing from seed
        missing = [j for j in node.joint_names if j not in js.name]
        if missing:
            print(f"  [WARN] Missing from /joint_states: {missing} (will use 0.0 as seed)")
    else:
        print("  No joint states — using home (all-zeros) as IK seed.")
    collision_note = "ON" if avoid_collisions else "OFF (debug mode)"
    if show_diag:
        print(f"  Collision checking: {collision_note}")
        print("  Diagnostic: first 8 IK calls will print raw error codes.")

    # Build orientation list (subset of _ORIENTATIONS_6 for speed)
    n_orientations = max(1, min(n_orientations, len(_ORIENTATIONS_6)))
    if n_orientations == 1:
        orientations = [_ARM_NATURAL_ORIENT[arm]]
        print(f"  Orientations: 1 (natural approach for {arm} arm — fastest)")
    else:
        orientations = _ORIENTATIONS_6[:n_orientations]
        print(f"  Orientations: {n_orientations}/6 axis-aligned")

    ee_home = {"right": (0.0, -0.153, 0.262), "left": (0.0, 0.153, 0.262)}
    hx, hy, hz = ee_home[arm]
    print(f"\n[{arm.upper()}] Pre-flight: EE home ({hx:.3f},{hy:.3f},{hz:.3f}) x6 orientations ...")
    ok_home = node.query_any(hx, hy, hz, orientations=orientations)
    if ok_home:
        print(f"  Pre-flight PASSED")
    else:
        print(f"  Pre-flight FAILED — NO_IK_SOLUTION for all 6 orientations.")
        if avoid_collisions:
            print(f"  Try --no-collision to check if collision detection is blocking IK.")
        print(f"  Continuing anyway.")
    print()

    # Grid selection: refine mode or normal mode
    if refine_csv:
        grid = build_refine_grid(refine_csv, step)
        mode_label = f"REFINE mode  (seed: {refine_csv})"
    else:
        grid = build_grid(xrange, yrange, zrange, step)
        mode_label = "full grid"
    total = len(grid)
    print(f"[{arm.upper()}] Sampling {total} points  (step={step} m, {mode_label}) ...")
    if not refine_csv:
        print(f"  X:{xrange}  Y:{yrange}  Z:{zrange}")
    print(f"  EE:openarm_{arm}_link7")

    reachable: list = []
    unreachable: list = []
    t0 = time.time()
    for i, (x, y, z) in enumerate(grid):
        ok = node.query_any(x, y, z, orientations=orientations)
        (reachable if ok else unreachable).append((x, y, z))
        if (i + 1) % 20 == 0 or (i + 1) == total:
            elapsed = time.time() - t0
            eta = elapsed / (i + 1) * (total - i - 1)
            pct = 100 * (i + 1) / total
            print(f"  [{i+1:>5}/{total}]  {pct:5.1f}%  "
                  f"reach={len(reachable)}  fail={len(unreachable)}  "
                  f"elapsed={elapsed:.0f}s  ETA={eta:.0f}s")

    print(f"\n[{arm.upper()}] Done. {len(reachable)}/{total} reachable "
          f"({100*len(reachable)/total:.1f}%)  in {time.time()-t0:.1f}s")
    save_csv(out_file, reachable, unreachable)

    chain_pts = node.get_arm_chain()
    if len(chain_pts) < 2:
        chain_pts = None

    executor.shutdown()
    node.destroy_node()
    return reachable, unreachable, chain_pts


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="EE Reachability Sampler (/compute_ik)")
    parser.add_argument("--arm",     default="right", choices=["left", "right", "both"],
                        help="Which arm(s) to sample (default: right)")
    parser.add_argument("--step",    type=float, default=0.07,  metavar="M")
    parser.add_argument("--xrange",  type=float, nargs=2, default=None, metavar=("MIN", "MAX"),
                        help="Override X range (single arm only)")
    parser.add_argument("--yrange",  type=float, nargs=2, default=None, metavar=("MIN", "MAX"),
                        help="Override Y range (single arm only)")
    parser.add_argument("--zrange",  type=float, nargs=2, default=None, metavar=("MIN", "MAX"),
                        help="Override Z range (single arm only)")
    parser.add_argument("--out",     type=str,   default=None,
                        help="CSV output path (single arm only)")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--no-collision", action="store_true",
                        help="Disable collision checking in IK (debug)")
    parser.add_argument("--timeout", type=float, default=0.5, metavar="SEC",
                        help="IK call timeout per point (default 0.5s)")
    parser.add_argument("--refine", type=str, default=None, metavar="COARSE_CSV",
                        help="Coarse-to-fine mode: only refine around reachable points in COARSE_CSV. "
                             "Use --step for fine resolution (e.g. --refine right_reachability.csv --step 0.01). "
                             "Speed-up: ~100x vs full fine grid.")
    parser.add_argument("--orientations", type=int, default=6, metavar="N",
                        help="Number of EE orientations to test per point (1~6, default 6). "
                             "1=fastest (arm natural direction), 6=most complete.")
    args, ros_args = parser.parse_known_args()

    arms = ["left", "right"] if args.arm == "both" else [args.arm]
    avoid_collisions = not args.no_collision
    if not avoid_collisions:
        print("[INFO] Collision checking DISABLED (--no-collision).  Results show geometry-only reachability.")

    if args.arm == "both" and (args.xrange or args.yrange or args.zrange or args.out):
        print("[WARN] --xrange/--yrange/--zrange/--out are ignored for --arm both")

    rclpy.init(args=ros_args)

    results = {}
    for arm in arms:
        dr = _DEFAULT_RANGES[arm]
        if args.arm == "both":
            xrange = list(dr["x"])
            yrange = list(dr["y"])
            zrange = list(dr["z"])
            out_file = f"{arm}_reachability.csv"
        else:
            xrange = args.xrange if args.xrange is not None else list(dr["x"])
            yrange = args.yrange if args.yrange is not None else list(dr["y"])
            zrange = args.zrange if args.zrange is not None else list(dr["z"])
            out_file = args.out or f"{arm}_reachability.csv"

        reachable, unreachable, chain_pts = _run_single_arm(
            arm, xrange, yrange, zrange, args.step, out_file, args.timeout,
            avoid_collisions=avoid_collisions,
            show_diag=(arm == arms[0]),
            refine_csv=args.refine,
            n_orientations=args.orientations,
        )
        results[arm] = {"reachable": reachable, "unreachable": unreachable, "chain": chain_pts}

    if not args.no_plot:
        if args.arm == "both":
            plot_both_results(results, args.step)
        else:
            arm = arms[0]
            d = results[arm]
            plot_results(arm, d["reachable"], d["unreachable"], args.step, d["chain"])

    rclpy.shutdown()


if __name__ == "__main__":
    main()
