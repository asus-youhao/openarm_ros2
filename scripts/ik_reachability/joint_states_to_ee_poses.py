#!/usr/bin/env python3
"""
joint_states_to_ee_poses.py — Offline FK: recorded joint states → EE poses

Reads joint-state CSVs produced by record_joint_states_sync.py (rows with
topic=/follower_joint_states contain the 7 arm joint angles).
Computes forward kinematics using the OpenArm right-arm URDF chain and writes:

 1. Full poses CSV  (stamp, joints, ee xyz, quaternion, RPY)
 2. Waypoints CSV   (x y z reachable=1) — compatible with csv_waypoint_runner.py
                    and realtime_ik_controller.py --csv mode

FK chain (world → openarm_right_link7):
  world → openarm_body_link0       : xyz=0,0,0       rpy=0,0,0
  body  → openarm_right_link0      : xyz=0,-0.031,0.698  rpy=π/2,0,0
  J1 (origin xyz=0,0,0.0625      rpy=0,0,0     axis z)
  J2 (origin xyz=-0.0301,0,0.06  rpy=π/2,0,0  axis -x)
  J3 (origin xyz=0.0301,0,0.06625 rpy=0,0,0   axis z)
  J4 (origin xyz=0,0.0315,0.15375 rpy=0,0,0   axis y)
  J5 (origin xyz=0,-0.0315,0.0955 rpy=0,0,0   axis z)
  J6 (origin xyz=0.0375,0,0.1205  rpy=0,0,0   axis x)
  J7 (origin xyz=-0.0375,0,0      rpy=0,0,0   axis y)

Usage:
  # Single file → outputs beside the input file
  python3 joint_states_to_ee_poses.py path/to/recording.csv

  # Entire folder
  python3 joint_states_to_ee_poses.py 20260410/

  # Custom output names
  python3 joint_states_to_ee_poses.py rec.csv --out full.csv --waypoints wpts.csv

  # Sample every 10th row (reduce density)
  python3 joint_states_to_ee_poses.py 20260410/ --every 10

  # Left arm
  python3 joint_states_to_ee_poses.py rec.csv --arm left
"""

import argparse
import csv
import os
import sys
import math
import glob
import numpy as np


# ---------------------------------------------------------------------------
# FK geometry — derived from openarm_bimanual_control.urdf
# ---------------------------------------------------------------------------
def _rpy_matrix(roll, pitch, yaw):
    """URDF RPY convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cx, sx = math.cos(roll),  math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw),   math.sin(yaw)
    Rx = np.array([[1, 0,   0  ],
                   [0, cx, -sx ],
                   [0, sx,  cx ]])
    Ry = np.array([[ cy, 0, sy],
                   [  0, 1,  0],
                   [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0],
                   [sz,  cz, 0],
                   [ 0,   0, 1]])
    return Rz @ Ry @ Rx


def _tf(xyz, rpy):
    """Build 4×4 homogeneous transform from (xyz, rpy)."""
    T = np.eye(4)
    T[:3, :3] = _rpy_matrix(*rpy)
    T[:3, 3]  = xyz
    return T


def _axis_rot(axis, angle):
    """4×4 pure rotation around unit-vector *axis* by *angle* (Rodrigues)."""
    ax, ay, az = axis
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    R = np.array([
        [t*ax*ax + c,     t*ax*ay - s*az,  t*ax*az + s*ay],
        [t*ax*ay + s*az,  t*ay*ay + c,     t*ay*az - s*ax],
        [t*ax*az - s*ay,  t*ay*az + s*ax,  t*az*az + c   ],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    return T


# Each entry: (origin_xyz, origin_rpy, joint_axis)
_RIGHT_JOINT_PARAMS = [
    ([0.0,     0.0,    0.0625 ], [0,          0, 0], [0,  0,  1]),  # J1
    ([-0.0301, 0.0,    0.06   ], [math.pi/2,  0, 0], [-1, 0,  0]),  # J2
    ([0.0301,  0.0,    0.06625], [0,          0, 0], [0,  0,  1]),  # J3
    ([0.0,     0.0315, 0.15375], [0,          0, 0], [0,  1,  0]),  # J4
    ([0.0,    -0.0315, 0.0955 ], [0,          0, 0], [0,  0,  1]),  # J5
    ([0.0375,  0.0,    0.1205 ], [0,          0, 0], [1,  0,  0]),  # J6
    ([-0.0375, 0.0,    0.0    ], [0,          0, 0], [0,  1,  0]),  # J7
]

_LEFT_JOINT_PARAMS = [
    ([0.0,     0.0,    0.0625 ], [0,          0, 0], [0,  0,  1]),  # J1
    ([-0.0301, 0.0,    0.06   ], [math.pi/2,  0, 0], [-1, 0,  0]),  # J2
    ([0.0301,  0.0,    0.06625], [0,          0, 0], [0,  0,  1]),  # J3
    ([0.0,     0.0315, 0.15375], [0,          0, 0], [0,  1,  0]),  # J4
    ([0.0,    -0.0315, 0.0955 ], [0,          0, 0], [0,  0,  1]),  # J5
    ([0.0375,  0.0,    0.1205 ], [0,          0, 0], [1,  0,  0]),  # J6
    ([-0.0375, 0.0,    0.0    ], [0,          0, 0], [0,  1,  0]),  # J7
]

# Fixed transforms: world → body_link0 → arm_link0
_RIGHT_BASE_TF = (
    _tf([0.0, 0.0,    0.0  ], [0,          0, 0])  # world → body
    @ _tf([0.0, -0.031, 0.698], [math.pi/2,  0, 0])  # body → right_link0
)
_LEFT_BASE_TF = (
    _tf([0.0, 0.0,    0.0  ], [0,          0, 0])  # world → body
    @ _tf([0.0,  0.031, 0.698], [-math.pi/2, 0, 0])  # body → left_link0
)


def compute_fk(q, arm: str = "right") -> np.ndarray:
    """
    Compute FK for 7 joint angles q → 4×4 world-to-link7 transform.
    """
    if arm == "right":
        T = _RIGHT_BASE_TF.copy()
        params = _RIGHT_JOINT_PARAMS
    else:
        T = _LEFT_BASE_TF.copy()
        params = _LEFT_JOINT_PARAMS

    for (xyz, rpy, axis), qi in zip(params, q):
        T = T @ _tf(xyz, rpy) @ _axis_rot(axis, qi)
    return T


def mat_to_rpy(R: np.ndarray):
    """Extract ZYX Euler angles (roll, pitch, yaw) from 3×3 rotation matrix."""
    pitch = math.atan2(-R[2, 0], math.sqrt(R[2, 1]**2 + R[2, 2]**2))
    yaw   = math.atan2(R[1, 0], R[0, 0])
    roll  = math.atan2(R[2, 1], R[2, 2])
    return roll, pitch, yaw


def mat_to_quat(R: np.ndarray):
    """Convert 3×3 rotation matrix to quaternion (qx, qy, qz, qw)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


# ---------------------------------------------------------------------------
# CSV reading
# ---------------------------------------------------------------------------
FOLLOWER_TOPIC = "/follower_joint_states"


def process_file(input_path: str, arm: str, every: int,
                 out_path: str, waypoints_path: str):
    """Process one recording CSV, write full poses and waypoints CSVs."""
    rows = []
    with open(input_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    # Filter rows for the arm joint states topic
    arm_rows = [r for r in rows if r["topic"] == FOLLOWER_TOPIC]
    if not arm_rows:
        print(f"  [WARN] No '{FOLLOWER_TOPIC}' rows in {input_path} — skipping")
        return 0

    # Apply stride sampling
    arm_rows = arm_rows[::every]

    full_records = []
    waypoint_records = []

    for r in arm_rows:
        sec  = int(r["stamp_sec"])
        nsec = int(r["stamp_nanosec"])
        pos_str = r["position"].strip().strip('"')
        try:
            q = [float(v) for v in pos_str.split(",")]
        except ValueError:
            continue
        if len(q) < 7:
            continue
        q = q[:7]

        T = compute_fk(q, arm)
        px, py, pz = T[0, 3], T[1, 3], T[2, 3]
        qx, qy, qz, qw = mat_to_quat(T[:3, :3])
        roll, pitch, yaw = mat_to_rpy(T[:3, :3])

        full_records.append({
            "stamp_sec":   sec,
            "stamp_nsec":  nsec,
            "j1": q[0], "j2": q[1], "j3": q[2],
            "j4": q[3], "j5": q[4], "j6": q[5], "j7": q[6],
            "ee_x": px, "ee_y": py, "ee_z": pz,
            "ee_qx": qx, "ee_qy": qy, "ee_qz": qz, "ee_qw": qw,
            "ee_roll": roll, "ee_pitch": pitch, "ee_yaw": yaw,
        })
        waypoint_records.append({
            "x": px, "y": py, "z": pz,
            "qx": qx, "qy": qy, "qz": qz, "qw": qw,
            "reachable": 1,
        })

    # Write full poses CSV
    full_fields = [
        "stamp_sec", "stamp_nsec",
        "j1", "j2", "j3", "j4", "j5", "j6", "j7",
        "ee_x", "ee_y", "ee_z",
        "ee_qx", "ee_qy", "ee_qz", "ee_qw",
        "ee_roll", "ee_pitch", "ee_yaw",
    ]
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=full_fields)
        w.writeheader()
        w.writerows(full_records)

    # Write waypoints CSV (compatible with csv_waypoint_runner.py)
    wpt_fields = ["x", "y", "z", "qx", "qy", "qz", "qw", "reachable"]
    with open(waypoints_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=wpt_fields)
        w.writeheader()
        w.writerows(waypoint_records)

    print(f"  {len(full_records):5d} poses  →  {out_path}")
    print(f"  {len(waypoint_records):5d} waypts →  {waypoints_path}")
    return len(full_records), waypoint_records


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _derive_output_paths(input_path: str, out_arg, waypoints_arg):
    base = input_path
    if base.endswith(".csv"):
        base = base[:-4]
    out_path  = out_arg       if out_arg       else base + "_ee_poses.csv"
    wpts_path = waypoints_arg if waypoints_arg else base + "_waypoints.csv"
    return out_path, wpts_path


def main():
    parser = argparse.ArgumentParser(
        description="Compute FK from recorded joint states → EE pose CSV",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument("input",
        help="Input CSV file or folder containing CSV files")
    parser.add_argument("--out",      default=None,
        help="Output full-poses CSV path (single-file mode)")
    parser.add_argument("--waypoints", default=None,
        help="Output waypoints CSV path (single-file mode)")
    parser.add_argument("--arm",      default="right",
        choices=["right", "left"],
        help="Which arm FK to compute (default: right)")
    parser.add_argument("--every",    default=1, type=int,
        help="Sample every Nth row (default: 1 = all rows)")
    parser.add_argument("--topic",    default=FOLLOWER_TOPIC,
        help=f"Topic to read joint states from (default: {FOLLOWER_TOPIC})")
    parser.add_argument("--merge",    default=None, metavar="PATH",
        help="(folder mode) Also write one combined waypoints CSV from all files")
    args = parser.parse_args()

    if args.topic != FOLLOWER_TOPIC:
        globals()["FOLLOWER_TOPIC"] = args.topic

    if os.path.isdir(args.input):
        # Only process original recording files, not already-generated output files
        all_files = sorted(glob.glob(os.path.join(args.input, "*.csv")))
        files = [f for f in all_files
                 if not f.endswith("_ee_poses.csv")
                 and not f.endswith("_waypoints.csv")]
        if not files:
            sys.exit(f"No CSV files found in {args.input}")
        print(f"Processing {len(files)} file(s) in {args.input}  "
              f"(arm={args.arm} every={args.every})")
        total = 0
        all_waypoints = []
        for f in files:
            out_path, wpts_path = _derive_output_paths(f, None, None)
            n, wpts = process_file(f, args.arm, args.every, out_path, wpts_path)
            total += n
            all_waypoints.extend(wpts)
        print(f"\nDone. Total poses computed: {total}")
        if args.merge:
            wpt_fields = ["x", "y", "z", "qx", "qy", "qz", "qw", "reachable"]
            with open(args.merge, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=wpt_fields)
                w.writeheader()
                w.writerows(all_waypoints)
            print(f"Merged waypoints CSV ({len(all_waypoints)} pts) → {args.merge}")
    else:
        if not os.path.isfile(args.input):
            sys.exit(f"File not found: {args.input}")
        out_path, wpts_path = _derive_output_paths(
            args.input, args.out, args.waypoints)
        print(f"Processing {args.input}  (arm={args.arm} every={args.every})")
        process_file(args.input, args.arm, args.every, out_path, wpts_path)

if __name__ == "__main__":
    main()
