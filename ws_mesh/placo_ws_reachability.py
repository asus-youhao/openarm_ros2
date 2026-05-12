#!/usr/bin/env python3
"""
placo_ws_reachability.py
========================
Sweep EE XYZ workspace in a grid, run Placo IK for each point with the given
RPY orientation(s), record success/failure, and produce:
  · CSV  — every sampled point with IK result
  · Plot — 3D scatter + 2D projections (XY / XZ / YZ success-rate heatmaps)

No ROS needed — pure Placo.

Usage examples
--------------
# right arm, default workspace, 5 cm step, orientation = identity (facing forward)
conda run -n pico_teleop_py python3 placo_ws_reachability.py --arm right

# left arm, 7 cm step, wrist-down orientation (roll=180°)
conda run -n pico_teleop_py python3 placo_ws_reachability.py --arm left --step 0.07 --rpy 180,0,0

# multiple orientations in one run
conda run -n pico_teleop_py python3 placo_ws_reachability.py --arm right \\
    --rpy 0,0,0  0,45,0  0,-45,0  0,90,0

# custom XYZ search range (wider than default workspace)
conda run -n pico_teleop_py python3 placo_ws_reachability.py --arm right \\
    --x -0.50 0.50  --y -0.60 -0.01  --z 0.01 0.80

CSV columns
-----------
  x, y, z          — target EE position (m, world frame)
  roll, pitch, yaw — target EE orientation (deg)
  success          — 1 = IK converged within pos_relax
  pos_err_mm       — final IK residual (mm)
  iterations       — number of placo iterations used
  solve_ms         — total wall time (ms)
  seed_used        — 0-based index of the seed that succeeded (-1 = all failed)
"""

# ── path setup ────────────────────────────────────────────────────────────────
import os as _os, sys as _sys
_HERE = _os.path.dirname(_os.path.abspath(__file__))           # ws_mesh/
_ROOT = _os.path.dirname(_HERE)                                # project root
_sys.path.insert(0, _ROOT)                                     # paths.py
_sys.path.insert(0, _os.path.join(_ROOT, "ik_solver"))         # placo_ik_solver

import argparse
import csv
import datetime
import math
import time
from typing import List, Optional, Tuple

import numpy as np

from paths import reachability_csv as _reachability_csv, png_for as _png_for
from placo_ik_solver import (
    _find_urdf,
    _HUMAN_RIGHT,
    _HUMAN_LEFT,
    _JOINT_NAMES,
    _ARM_SEEDS,
    _quat_to_rot,
)

# ── arm workspace defaults (mirrors _ARM_CONFIG in placo_ik_online_profiler) ──
_WS_DEFAULT = {
    "right": {"x": (-0.20, 0.50), "y": (-0.50, -0.05), "z": (0.35, 0.80)},
    "left":  {"x": (-0.30, 0.30), "y": (0.05,  0.45),  "z": (0.05, 0.65)},
}

# ── IK constants ──────────────────────────────────────────────────────────────
_POS_TOL   = 0.003   # m  — early-exit threshold
_POS_RELAX = 0.010   # m  — acceptance threshold
_MAX_ITER  = 100
_DT        = 0.010
_W_POS     = 1.0
_W_ORI     = 0.3
_W_JOINTS  = 1e-4


# ── RPY → rotation matrix ─────────────────────────────────────────────────────
def _rpy_to_rot(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """ZYX Euler convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr,  cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy,  cy, 0], [0,    0, 1]])
    return Rz @ Ry @ Rx


# ── Cached robot session ──────────────────────────────────────────────────────
class _ReachabilitySolver:
    """
    Single cached RobotWrapper.  For each target point we:
      1. try each seed in order
      2. set joints to seed, create fresh KinematicsSolver, run early-exit IK
      3. return first seed that converges within _POS_RELAX
    """

    def __init__(self, arm: str, max_iter: int = _MAX_ITER, dt: float = _DT):
        import placo
        self._placo      = placo
        self._arm        = arm
        self._max_iter   = max_iter
        self._dt         = dt
        self._joint_names = _JOINT_NAMES[arm]
        self._human_cfg   = _HUMAN_RIGHT if arm == "right" else _HUMAN_LEFT
        self._ee_link     = f"openarm_{arm}_link7"
        self._lo   = [self._human_cfg[n][1] for n in self._joint_names]
        self._hi   = [self._human_cfg[n][2] for n in self._joint_names]
        self._pref = {n: self._human_cfg[n][0] for n in self._joint_names}
        self._seeds = _ARM_SEEDS[arm]

        urdf = _find_urdf()
        self._robot = placo.RobotWrapper(urdf, placo.Flags.ignore_collisions)
        print(f"[Solver] {arm} arm ready  ee_link={self._ee_link}  "
              f"max_iter={max_iter}  dt={dt}  seeds={len(self._seeds)}")

    def solve(
        self,
        target_xyz: np.ndarray,
        target_R:   np.ndarray,
    ) -> dict:
        """
        Try all seeds.  Returns dict with success, pos_err_mm, iterations,
        seed_used, joints, solve_ms.
        """
        placo  = self._placo
        robot  = self._robot
        t0     = time.perf_counter()

        best_err   = 9999.0
        best_joints = list(self._pref.values())
        total_iters = 0
        seed_used   = -1

        for s_idx, seed in enumerate(self._seeds):
            # Reset robot to this seed
            for name, val in zip(self._joint_names, seed):
                robot.set_joint(name, val)
            robot.update_kinematics()

            # Fresh solver each attempt (clean task list)
            solver = placo.KinematicsSolver(robot)
            solver.enable_joint_limits(True)
            solver.enable_velocity_limits(False)
            solver.mask_fbase(True)
            solver.dt = self._dt

            pos_task = solver.add_position_task(self._ee_link, target_xyz)
            pos_task.configure("pos", "soft", _W_POS)
            ori_task = solver.add_orientation_task(self._ee_link, target_R)
            ori_task.configure("ori", "soft", _W_ORI)
            jt = solver.add_joints_task()
            jt.set_joints(self._pref)
            jt.configure("naturalness", "soft", _W_JOINTS)
            reg = solver.add_regularization_task(1e-5)
            reg.configure("reg", "soft", 1.0)

            iters = 0
            for _ in range(self._max_iter):
                solver.solve(True)
                robot.update_kinematics()
                iters += 1
                T_ee   = robot.get_T_world_frame(self._ee_link)
                pos_err = float(np.linalg.norm(T_ee[:3, 3] - target_xyz))
                if pos_err < _POS_TOL:
                    break

            total_iters += iters
            T_ee    = robot.get_T_world_frame(self._ee_link)
            pos_err = float(np.linalg.norm(T_ee[:3, 3] - target_xyz))
            joints  = [robot.get_joint(n) for n in self._joint_names]
            joints  = [max(l, min(h, q)) for q, l, h in zip(joints, self._lo, self._hi)]

            if pos_err < best_err:
                best_err    = pos_err
                best_joints = joints
                seed_used   = s_idx

            if pos_err < _POS_RELAX:
                # success
                return {
                    "success":    1,
                    "pos_err_mm": pos_err * 1000.0,
                    "iterations": total_iters,
                    "seed_used":  s_idx,
                    "joints":     joints,
                    "solve_ms":   (time.perf_counter() - t0) * 1000.0,
                }

        return {
            "success":    0,
            "pos_err_mm": best_err * 1000.0,
            "iterations": total_iters,
            "seed_used":  -1,
            "joints":     best_joints,
            "solve_ms":   (time.perf_counter() - t0) * 1000.0,
        }


# ── Grid sweep ────────────────────────────────────────────────────────────────
def sweep(
    arm:       str,
    x_range:   Tuple[float, float],
    y_range:   Tuple[float, float],
    z_range:   Tuple[float, float],
    step:      float,
    rpy_list:  List[Tuple[float, float, float]],
    max_iter:  int  = _MAX_ITER,
    dt:        float = _DT,
) -> List[dict]:
    """
    Sweep XYZ grid × RPY orientations, return list of result dicts.
    """
    solver = _ReachabilitySolver(arm, max_iter=max_iter, dt=dt)

    xs = np.arange(x_range[0], x_range[1] + step * 0.5, step)
    ys = np.arange(y_range[0], y_range[1] + step * 0.5, step)
    zs = np.arange(z_range[0], z_range[1] + step * 0.5, step)

    total = len(xs) * len(ys) * len(zs) * len(rpy_list)
    print(f"\n  Grid: x={len(xs)}  y={len(ys)}  z={len(zs)}  "
          f"rpy={len(rpy_list)}  → {total} points")

    results = []
    done    = 0
    n_ok    = 0
    t_start = time.perf_counter()

    for rpy in rpy_list:
        roll, pitch, yaw = rpy
        target_R = _rpy_to_rot(roll, pitch, yaw)

        for x in xs:
            for y in ys:
                for z in zs:
                    target_xyz = np.array([x, y, z])
                    r = solver.solve(target_xyz, target_R)

                    row = {
                        "x":          round(float(x), 4),
                        "y":          round(float(y), 4),
                        "z":          round(float(z), 4),
                        "roll":       roll,
                        "pitch":      pitch,
                        "yaw":        yaw,
                        "success":    r["success"],
                        "pos_err_mm": round(r["pos_err_mm"], 3),
                        "iterations": r["iterations"],
                        "solve_ms":   round(r["solve_ms"], 2),
                        "seed_used":  r["seed_used"],
                    }
                    results.append(row)
                    done += 1
                    if r["success"]:
                        n_ok += 1

                    # Progress every 100 points
                    if done % 100 == 0 or done == total:
                        elapsed = time.perf_counter() - t_start
                        eta     = elapsed / done * (total - done) if done < total else 0
                        pct     = done / total * 100
                        sr_pct  = n_ok / done * 100
                        print(
                            f"\r  {done:>{len(str(total))}}/{total} ({pct:5.1f}%)"
                            f"  sr={sr_pct:5.1f}%"
                            f"  elapsed={elapsed:5.1f}s  eta={eta:5.1f}s",
                            end="", flush=True,
                        )

    print(f"\n  Done.  success={n_ok}/{total} ({n_ok/total*100:.1f}%)"
          f"  total_time={time.perf_counter()-t_start:.1f}s")
    return results


# ── CSV save ──────────────────────────────────────────────────────────────────
_CSV_FIELDS = ["x", "y", "z", "roll", "pitch", "yaw",
               "success", "pos_err_mm", "iterations", "solve_ms", "seed_used"]

def save_csv(results: List[dict], path: str):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=_CSV_FIELDS)
        w.writeheader()
        w.writerows(results)
    print(f"  CSV → {path}")


# ── Aggregate by XYZ (position-level) ────────────────────────────────────────
def _aggregate_by_xyz(
    results:  List[dict],
    rpy_list: List[Tuple[float, float, float]],
) -> List[dict]:
    """
    Group per-(x,y,z,rpy) rows by position only.
    Returns one dict per unique (x,y,z):
      pos_reachable  — 1 if ANY orientation succeeded, 0 if ALL failed
      n_orient_ok    — how many orientations succeeded at this xyz
      orient_rate    — n_orient_ok / n_rpy  (0.0 – 1.0)
      best_err_mm    — minimum pos_err across all orientations
      mean_solve_ms  — mean solve time across all orientations
    """
    from collections import defaultdict
    groups: dict = defaultdict(list)
    for r in results:
        groups[(r["x"], r["y"], r["z"])].append(r)
    agg = []
    for (x, y, z), rows in groups.items():
        n_ok = sum(r["success"] for r in rows)
        agg.append({
            "x":             x,
            "y":             y,
            "z":             z,
            "pos_reachable": 1 if n_ok > 0 else 0,
            "n_orient_ok":   n_ok,
            "orient_rate":   n_ok / len(rows),
            "best_err_mm":   min(r["pos_err_mm"] for r in rows),
            "mean_solve_ms": sum(r["solve_ms"] for r in rows) / len(rows),
        })
    return agg


# ── Plot ──────────────────────────────────────────────────────────────────────
def save_plot(
    results:   List[dict],
    arm:       str,
    rpy_list:  List[Tuple[float, float, float]],
    step:      float,
    plot_path: str,
):
    """
    4-row layout:
      Row 0 — 3D position reachability | 3D orientation coverage | per-RPY bar
      Row 1 — XY/XZ/YZ heatmaps: POSITION reachability (any orient. succeeds)
      Row 2 — XY/XZ/YZ heatmaps: ORIENTATION coverage  (avg n_ok/n_rpy)
      Row 3 — IK residual dist | solve_ms dist | pos reachability per Z
    CSV stays per-(xyz,rpy) — plot uses position-aggregated view.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap

    # ── Aggregate to position level ───────────────────────────────────────────
    agg   = _aggregate_by_xyz(results, rpy_list)
    ax_   = np.array([a["x"]             for a in agg])
    ay_   = np.array([a["y"]             for a in agg])
    az_   = np.array([a["z"]             for a in agg])
    pos_r = np.array([a["pos_reachable"] for a in agg], dtype=float)
    n_ok_ = np.array([a["n_orient_ok"]   for a in agg], dtype=float)
    or_   = np.array([a["orient_rate"]   for a in agg], dtype=float)

    pos_ok_mask   = pos_r == 1
    pos_fail_mask = pos_r == 0
    n_pos_ok    = int(pos_ok_mask.sum())
    n_pos_total = len(agg)

    # raw per-(xyz,rpy) arrays (for stats panels and per-RPY bar)
    succ = np.array([r["success"]    for r in results], dtype=float)
    err  = np.array([r["pos_err_mm"] for r in results])
    ms_  = np.array([r["solve_ms"]   for r in results])
    fail_mask = succ == 0
    ok_mask   = succ == 1

    n_rpy   = len(rpy_list)
    rpy_str = "  ".join(f"({roll:.0f},{pitch:.0f},{yaw:.0f})°"
                        for roll, pitch, yaw in rpy_list)

    cmap_sr = LinearSegmentedColormap.from_list("sr", ["#cc2222", "#22aa22"])
    cmap_n  = plt.cm.RdYlGn  # for orientation coverage

    fig = plt.figure(figsize=(20, 22))
    fig.suptitle(
        f"Workspace Reachability  arm={arm}  step={step*100:.0f}cm\n"
        f"RPY(deg): {rpy_str}\n"
        f"Position reachable (any orient): {n_pos_ok}/{n_pos_total}"
        f" ({n_pos_ok/n_pos_total*100:.1f}%)   "
        f"Total IK calls: {int(succ.sum())}/{len(results)}"
        f" ({succ.mean()*100:.1f}% success)",
        fontsize=10, y=0.99,
    )
    gs = fig.add_gridspec(4, 3, hspace=0.52, wspace=0.38)

    # ── Row 0: 3D position reachability | 3D orient coverage | per-RPY bar ───
    ax3d = fig.add_subplot(gs[0, 0], projection="3d")
    if pos_ok_mask.any():
        ax3d.scatter(ax_[pos_ok_mask], ay_[pos_ok_mask], az_[pos_ok_mask],
                     c="green", s=5, alpha=0.25, label="pos reachable")
    if pos_fail_mask.any():
        ax3d.scatter(ax_[pos_fail_mask], ay_[pos_fail_mask], az_[pos_fail_mask],
                     c="red", s=9, alpha=0.70, label="all orient. FAIL")
    ax3d.set_xlabel("X(m)", fontsize=6); ax3d.set_ylabel("Y(m)", fontsize=6)
    ax3d.set_zlabel("Z(m)", fontsize=6)
    ax3d.set_title("Position reachability 3D\ngreen=any RPY OK  red=ALL RPY fail", fontsize=8)
    ax3d.legend(fontsize=6); ax3d.tick_params(labelsize=5)

    ax3d2 = fig.add_subplot(gs[0, 1], projection="3d")
    sc = ax3d2.scatter(ax_, ay_, az_, c=n_ok_ / n_rpy, cmap=cmap_n,
                       s=5, alpha=0.50, vmin=0, vmax=1)
    plt.colorbar(sc, ax=ax3d2, shrink=0.6, label=f"orient. success rate (/{n_rpy})")
    ax3d2.set_xlabel("X(m)", fontsize=6); ax3d2.set_ylabel("Y(m)", fontsize=6)
    ax3d2.set_zlabel("Z(m)", fontsize=6)
    ax3d2.set_title(f"Orientation coverage 3D\ncolour = n_ok / {n_rpy} orientations", fontsize=8)
    ax3d2.tick_params(labelsize=5)

    # per-RPY success rate bar
    ax_bar = fig.add_subplot(gs[0, 2])
    rpy_labels = [f"({r:.0f},{p:.0f},{y:.0f})" for r, p, y in rpy_list]
    rpy_sr = []
    for rpy in rpy_list:
        r_r, p_r, y_r = rpy
        rows_rpy = [r for r in results
                    if abs(r["roll"]  - r_r) < 0.01
                    and abs(r["pitch"] - p_r) < 0.01
                    and abs(r["yaw"]   - y_r) < 0.01]
        rpy_sr.append(sum(r["success"] for r in rows_rpy) / max(len(rows_rpy), 1) * 100)
    ax_bar.barh(range(len(rpy_labels)), rpy_sr,
                color=[cmap_sr(v / 100) for v in rpy_sr])
    ax_bar.set_yticks(range(len(rpy_labels)))
    ax_bar.set_yticklabels(rpy_labels, fontsize=7)
    ax_bar.set_xlabel("IK success rate (%)", fontsize=8)
    ax_bar.set_xlim(0, 112)
    ax_bar.set_title("Per-orientation IK success rate\n(across all xyz points)", fontsize=8)
    ax_bar.axvline(100, color="gray", linestyle=":", linewidth=0.8)
    ax_bar.grid(True, alpha=0.3, axis="x")
    for i, v in enumerate(rpy_sr):
        ax_bar.text(v + 1, i, f"{v:.1f}%", va="center", fontsize=7)

    # ── generic heatmap (works on aggregated position data) ───────────────────
    def _hm(ax_obj, data_x, data_y, val,
            a_label, b_label, title,
            cmap=cmap_sr, vmin=0.0, vmax=1.0, cb_label="rate"):
        """
        data_x / data_y / val are parallel arrays (one entry per xyz point).
        Bins into (a,b) cells and averages val within each cell.
        For pos_reachable (0/1): gives fraction of reachable positions per cell.
        For orient_rate (0-1):   gives mean orientation coverage per cell.
        """
        a_bins = np.unique(np.round(data_x, 6))
        b_bins = np.unique(np.round(data_y, 6))
        a_idx  = {v: i for i, v in enumerate(a_bins)}
        b_idx  = {v: i for i, v in enumerate(b_bins)}
        counts = np.zeros((len(b_bins), len(a_bins)))
        totals = np.zeros((len(b_bins), len(a_bins)))
        for xi, yi, vi in zip(data_x, data_y, val):
            ai = a_idx.get(round(float(xi), 6))
            bi = b_idx.get(round(float(yi), 6))
            if ai is not None and bi is not None:
                totals[bi, ai] += 1
                counts[bi, ai] += vi
        with np.errstate(invalid="ignore"):
            grid = np.where(totals > 0, counts / totals, np.nan)
        im = ax_obj.imshow(
            grid, origin="lower", aspect="auto",
            extent=[a_bins[0]-step/2, a_bins[-1]+step/2,
                    b_bins[0]-step/2, b_bins[-1]+step/2],
            vmin=vmin, vmax=vmax, cmap=cmap,
        )
        plt.colorbar(im, ax=ax_obj, shrink=0.8, label=cb_label)
        ax_obj.set_xlabel(a_label, fontsize=8)
        ax_obj.set_ylabel(b_label, fontsize=8)
        ax_obj.set_title(title, fontsize=8)
        ax_obj.tick_params(labelsize=7)

    # ── Row 1: Position reachability heatmaps (any-orientation binary) ────────
    _hm(fig.add_subplot(gs[1, 0]), ax_, ay_, pos_r,
        "X (m)", "Y (m)", "XY — position reachability\n(1 = any orient. succeeds)",
        cb_label="pos reachable rate")
    _hm(fig.add_subplot(gs[1, 1]), ax_, az_, pos_r,
        "X (m)", "Z (m)", "XZ — position reachability\n(1 = any orient. succeeds)",
        cb_label="pos reachable rate")
    _hm(fig.add_subplot(gs[1, 2]), ay_, az_, pos_r,
        "Y (m)", "Z (m)", "YZ — position reachability\n(1 = any orient. succeeds)",
        cb_label="pos reachable rate")

    # ── Row 2: Orientation coverage heatmaps (mean n_ok/n_rpy per cell) ───────
    _hm(fig.add_subplot(gs[2, 0]), ax_, ay_, or_,
        "X (m)", "Y (m)", f"XY — orientation coverage\n(mean n_ok / {n_rpy})",
        cmap=cmap_n, cb_label=f"orient. rate (/{n_rpy})")
    _hm(fig.add_subplot(gs[2, 1]), ax_, az_, or_,
        "X (m)", "Z (m)", f"XZ — orientation coverage\n(mean n_ok / {n_rpy})",
        cmap=cmap_n, cb_label=f"orient. rate (/{n_rpy})")
    _hm(fig.add_subplot(gs[2, 2]), ay_, az_, or_,
        "Y (m)", "Z (m)", f"YZ — orientation coverage\n(mean n_ok / {n_rpy})",
        cmap=cmap_n, cb_label=f"orient. rate (/{n_rpy})")

    # ── Row 3: Stats panels ───────────────────────────────────────────────────
    ax_err = fig.add_subplot(gs[3, 0])
    if fail_mask.any():
        ax_err.hist(err[fail_mask], bins=40, color="tomato",
                    edgecolor="white", linewidth=0.5)
        ax_err.axvline(_POS_RELAX * 1000, color="red", linestyle="--",
                       label=f"relax={_POS_RELAX*1000:.0f}mm")
        ax_err.legend(fontsize=7)
    ax_err.set_xlabel("pos_err_mm", fontsize=8)
    ax_err.set_title(f"IK residual (failed IK calls)\nn={fail_mask.sum()}", fontsize=8)
    ax_err.tick_params(labelsize=7)
    ax_err.grid(True, alpha=0.3)

    ax_ms = fig.add_subplot(gs[3, 1])
    ax_ms.hist(ms_[ok_mask],   bins=40, color="steelblue", alpha=0.7,
               edgecolor="white", linewidth=0.5, label="success")
    ax_ms.hist(ms_[fail_mask], bins=40, color="tomato", alpha=0.7,
               edgecolor="white", linewidth=0.5, label="fail")
    ax_ms.set_xlabel("solve_ms", fontsize=8)
    ax_ms.set_title("Solve time distribution (ms)", fontsize=8)
    ax_ms.legend(fontsize=7)
    ax_ms.tick_params(labelsize=7)
    ax_ms.grid(True, alpha=0.3)

    ax_z = fig.add_subplot(gs[3, 2])
    z_levels = np.unique(np.round(az_, 6))
    sr_per_z = []
    for zl in z_levels:
        mask_z = np.abs(az_ - zl) < 1e-5
        sr_per_z.append(pos_r[mask_z].mean() * 100.0)
    ax_z.barh(z_levels, sr_per_z, height=step * 0.85,
              color=[cmap_sr(v / 100) for v in sr_per_z])
    ax_z.set_xlabel("pos reachable (%)", fontsize=8)
    ax_z.set_ylabel("Z (m)", fontsize=8)
    ax_z.set_xlim(0, 105)
    ax_z.axvline(100, color="gray", linestyle=":", linewidth=0.8)
    ax_z.set_title("Position reachability per Z level\n(any orientation)", fontsize=8)
    ax_z.tick_params(labelsize=7)

    plt.savefig(plot_path, dpi=130, bbox_inches="tight")
    plt.close()
    print(f"  Plot → {plot_path}")


# ── CLI ───────────────────────────────────────────────────────────────────────
def _parse_args():
    p = argparse.ArgumentParser(
        description="Placo IK workspace reachability sweeper (no ROS needed)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--arm",      default="right",  choices=["right", "left"])
    p.add_argument("--step",     type=float, default=0.05,
                   help="Grid step in metres (default: 0.05 = 5 cm)")
    p.add_argument("--rpy",      type=str,   nargs="+",
                   default=["0,0,0", "0,45,0", "0,-45,0", "0,90,0", "90,0,0", "180,0,0"],
                   help="EE orientation(s) as 'roll,pitch,yaw' in degrees. "
                        "Multiple allowed, e.g. --rpy 0,0,0  0,45,0  0,90,0\n"
                        "Default (6 orientations): forward / pitch-down45 / pitch-up45 / "
                        "pointing-down / side-grasp / wrist-down")
    p.add_argument("--x",        type=float, nargs=2, default=None,
                   metavar=("X_MIN", "X_MAX"),
                   help="Override X range (m). Default: arm workspace")
    p.add_argument("--y",        type=float, nargs=2, default=None,
                   metavar=("Y_MIN", "Y_MAX"),
                   help="Override Y range (m). Default: arm workspace")
    p.add_argument("--z",        type=float, nargs=2, default=None,
                   metavar=("Z_MIN", "Z_MAX"),
                   help="Override Z range (m). Default: arm workspace")
    p.add_argument("--max-iter", type=int,   default=_MAX_ITER, dest="max_iter",
                   help=f"Max placo iterations per seed (default: {_MAX_ITER})")
    p.add_argument("--dt",       type=float, default=_DT,
                   help=f"Placo solver dt (default: {_DT})")
    p.add_argument("--csv",      default="",
                   help="Output CSV path (auto-generated if omitted)")
    p.add_argument("--plot",     default="",
                   help="Output plot PNG path (auto-generated if omitted)")
    return p.parse_args()


def main():
    args = _parse_args()

    # Parse RPY list
    rpy_list = []
    for s in args.rpy:
        parts = [float(v) for v in s.split(",")]
        if len(parts) != 3:
            raise ValueError(f"--rpy expects 'roll,pitch,yaw', got: {s!r}")
        rpy_list.append(tuple(parts))

    # Workspace ranges
    ws_def = _WS_DEFAULT[args.arm]
    x_range = tuple(args.x) if args.x else ws_def["x"]
    y_range = tuple(args.y) if args.y else ws_def["y"]
    z_range = tuple(args.z) if args.z else ws_def["z"]

    # Output paths
    csv_path  = args.csv  or _reachability_csv(args.arm)
    plot_path = args.plot or _png_for(csv_path)

    print(f"\n{'═'*65}")
    print(f"  Placo Workspace Reachability Sweeper")
    print(f"  arm={args.arm}  step={args.step*100:.0f}cm")
    print(f"  X=[{x_range[0]:.2f}, {x_range[1]:.2f}]  "
          f"Y=[{y_range[0]:.2f}, {y_range[1]:.2f}]  "
          f"Z=[{z_range[0]:.2f}, {z_range[1]:.2f}]")
    rpy_str = "  ".join(f"({r},{pi},{y})" for r, pi, y in rpy_list)
    print(f"  RPY(deg): {rpy_str}")
    print(f"  max_iter={args.max_iter}  dt={args.dt}")
    print(f"  CSV:  {csv_path}")
    print(f"  Plot: {plot_path}")
    print(f"{'═'*65}")

    results = sweep(
        arm=args.arm,
        x_range=x_range,
        y_range=y_range,
        z_range=z_range,
        step=args.step,
        rpy_list=rpy_list,
        max_iter=args.max_iter,
        dt=args.dt,
    )

    save_csv(results, csv_path)
    save_plot(results, args.arm, rpy_list, args.step, plot_path)

    # Summary
    n_ok    = sum(r["success"] for r in results)
    n_total = len(results)
    n_fail  = n_total - n_ok
    fail_pts = [(r["x"], r["y"], r["z"], r["roll"], r["pitch"], r["yaw"],
                 r["pos_err_mm"])
                for r in results if not r["success"]]
    print(f"\n{'─'*65}")
    print(f"  Total: {n_total}  Success: {n_ok} ({n_ok/n_total*100:.1f}%)"
          f"  Fail: {n_fail}")
    if fail_pts:
        print(f"  First 10 failed points (x,y,z,r,p,y_deg,err_mm):")
        for pt in fail_pts[:10]:
            print(f"    x={pt[0]:+.3f}  y={pt[1]:+.3f}  z={pt[2]:+.3f}"
                  f"  rpy=({pt[3]:.0f},{pt[4]:.0f},{pt[5]:.0f})"
                  f"  err={pt[6]:.1f}mm")
    print(f"{'─'*65}\n")


if __name__ == "__main__":
    import os
    main()
