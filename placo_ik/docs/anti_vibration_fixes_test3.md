# Anti-Vibration Fixes for PlaCo IK Teleoperation

**Date:** 2026-05-15
**Branch:** `feature/placo_ik` (built on top of commit `069be5d`)
**Problem:** 2–10 Hz rotational vibration during PICO VR tracker teleoperation

---

## Root Cause Summary

| Cause | Impact | Location |
|-------|--------|----------|
| IK early-exit checked **only position**, not orientation | High | `placo_ik_session.py` |
| `_MAX_ITER = 5` too low for orientation convergence | High | `placo_ik_session.py` |
| `_W_ORI = 0.8` below position weight → ori under-determined | High | `placo_ik_session.py` |
| No orientation smoothing on input side | High | `placo_ik_node.py` |
| `--horizon 1.0 ms` → instant velocity demands on controller | High | Entry point defaults |
| No calibration on orientation delta (`dq`) | Medium | `placo_ik_node.py` |
| IK seeded from raw joints, not filtered output | Medium | `placo_ik_node.py` |
| Uniform output LPF on all joints | Medium | Entry point defaults |
| `_W_REG = 1e-5` too low singularity damping | Low | `placo_ik_session.py` |
| `ee_delta_gap_sec = 0.35` → spurious ref resets | Low | `placo_ik_node.py` |

---

## 8 Fixes Applied

### Fix 1 — SLERP-Based Orientation Filter (Input Side)

**File:** `placo_ik_node.py`
**CLI:** `--ori-lpf-alpha 0.35` (default)

Applies SLERP interpolation (exponential moving average) on the target orientation
**before** the IK solver, so sensor noise is reduced at the source.

Pipeline: `dq_raw → _rotate_quat(dq, calib_q) → dq_arm * base_q → SLERP filter → target_R`

- Handles quaternion sign-flipping (`dot < 0`)
- Normalizes all quaternions
- Resets on: home, session reset, new tracker reference
- `alpha=1.0` to disable

---

### Fix 2 — Increase IK Iterations + Orientation Convergence Check

**File:** `placo_ik_session.py`

| Parameter | Before | After |
|-----------|--------|-------|
| `_MAX_ITER` | 5 | **20** |
| Early-exit | Position only (`_POS_TOL`) | Position **AND** orientation |
| `_ORI_TOL` | (none) | **0.050 rad** (~2.9°) |
| `_ORI_RELAX` | (none) | **0.200 rad** (~11.5°) |

At ~0.08 ms/iter, 20 iters = ~1.6 ms — well within 20 ms budget.
Most steps still exit in 3–8 iterations for small movements.

---

### Fix 3 — Per-Joint Output LPF

**CLI:** `--lpf-alpha 0.25,0.25,0.25,0.4,0.6,0.6,0.6` (new default)

| Joints | Motor Size | α | Effect |
|--------|-----------|---|--------|
| J1–J3 (shoulder) | Large | 0.25 | Heavy smoothing (slow, stable) |
| J4 (elbow) | Medium | 0.4 | Moderate smoothing |
| J5–J7 (wrist) | Small | 0.6 | Responsive (orientation-critical) |

Previously: uniform `α=1.0` (disabled by default).

---

### Fix 4 — IK Task Weight Rebalancing

**File:** `placo_ik_session.py`

| Weight | Before | After | Rationale |
|--------|--------|-------|-----------|
| `_W_ORI` | 0.8 | **1.0** | Equal priority with position for teleop |
| `_W_REG` | 1e-5 | **1e-4** | Stronger singularity damping |

7-DOF arm: 3 DOF position + 3 DOF orientation + 1 DOF null-space → equal weights work.

---

### Fix 5 — Joint Jump Guard

**File:** `placo_ik_node.py`
**CLI:** `--joint-jump-guard-deg 15` (default)

Hard rejects IK solutions where any single joint moves > 15°/step.
Protects against IK branch jumps (elbow flip, wrist singularity).

- `0` = disabled
- Reports `max_joint_delta_deg` and `joint_jump_guard` in CSV

---

### Fix 6 — Seed IK from Filtered Output

**File:** `placo_ik_node.py`

| Before | After |
|--------|-------|
| `seed = self._last_joints` (raw IK output) | `seed = self._filt_joints or self._last_joints` |
| `self._last_joints = r["joints"]` (raw) | `self._last_joints = joints_out` (filtered) |

This prevents divergence between the seed (what IK thinks is current) and what the
robot is actually tracking (the filtered values).

---

### Fix 7 — Horizon / Rate Sanity

**File:** `placo_ik_online_profiler_ws_mesh.py`

| Parameter | Before | After |
|-----------|--------|-------|
| `--rate` | 100 Hz | **50 Hz** |
| `--horizon` | 1.0 ms | **auto (30.0 ms)** |

Auto-compute: `horizon = 1.5 × (1000 / rate)`. Clamps if user sets horizon < period.

**Why this matters:** `--horizon 1.0` at 100 Hz means the `joint_trajectory_controller`
tries to reach each new position in 1 ms → instantaneous velocity demands → motor PID
oscillation → 2–10 Hz vibration.

Also fixed `_publish()` to correctly split horizon into `sec` + `nanosec` fields.

---

### Fix 8 — Orientation Calibration on Delta Quaternion

**File:** `placo_ik_node.py`

Added `dq = _rotate_quat(dq, self._calib_q)` after extracting the tracker delta.

Implements the similarity transform: `dq_arm = calib_q * dq * conj(calib_q)`

Previously, only the position delta was calibrated (`dx_arm = _rotate_vec(dx, calib_q)`)
while the orientation delta was applied in the raw tracker frame → frame mismatch → RPY jitter.

---

## New CLI Arguments

```
--ori-lpf-alpha 0.35        # SLERP EMA on target orientation (0.0–1.0)
--ee-delta-gap-sec 0.8      # Gap before resetting tracker reference
--joint-jump-guard-deg 15   # Max joint delta per step (0 = off)
--no-rot-tracking           # Position-only IK (disable orientation)
--use-traj                  # Revert to JointTrajectoryController (legacy)
```

## New CSV / Profiling Columns

| Column | Description |
|--------|-------------|
| `ori_err_deg` | Orientation error after IK solve (degrees) |
| `max_joint_delta_deg` | Largest single-joint change this step |
| `joint_jump_guard` | 1 if jump guard rejected this step |

---

## Fix 9 — ForwardCommandController (Actual Root Cause Fix)

**Date:** 2026-05-17
**Problem:** Fixes 1–8 reduced jitter but vibration persisted.

### Root Cause Analysis

CSV analysis (`analyze_vibration.py`) revealed:

| Metric | test3 (Fixes 1-8) | test1 (heavy LPF) | Healthy |
|--------|-------|-------|---------|
| cmd/state ratio | **1.5–3.6×** | 1.2–1.4× | ≈ 1.0× |
| Direction reversal | **60–68%** | 7–9% | < 20% |
| Dominant FFT peak | **13–14 Hz** | ~1 Hz | no peak |

The `JointTrajectoryController` was the root cause. It receives a single-point
trajectory every 26 ms, aborts the previous interpolation, and starts a new
"reach this position in 30 ms" step response — creating a staircase of step
inputs at the motor PID.

`send_home_confirmed()` is smooth because it sends ONE trajectory over 3 seconds.

### Solution

Switch the hot-loop publisher from `JointTrajectoryController` to
`ForwardCommandController`. The forward controller passes positions **directly**
to the hardware interface with no interpolation/PID — the motor firmware handles
tracking. This is the standard approach for real-time teleoperation.

### Changes

1. **ARM_CONFIG**: Split `cmd_topic` into `traj_topic` (for home) + `fwd_cmd_topic` (for streaming)
2. **`_publish()`**: Sends `Float64MultiArray` to `/right_forward_position_controller/commands`
3. **`send_home_confirmed()`**: Still uses `JointTrajectory` via `_traj_pub` (for `--use-traj` mode)
4. **`send_home_fwd()`**: New method for homing in ForwardCmd mode — ramps to home via cosine-interpolated position streaming at 50 Hz with conservative 30°/s speed limit
5. **`--use-traj`**: CLI flag to revert to legacy JointTrajectoryController behavior

### Launch Prerequisite

The robot must be launched with `robot_controller:=forward_position_controller`:

```bash
# Bimanual
ros2 launch openarm_bringup openarm.bimanual.launch.py \
    robot_controller:=forward_position_controller

# Or O6 bimanual
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py \
    robot_controller:=forward_position_controller
```

`--home-first` now works in both modes:
- **ForwardCmd mode**: uses `send_home_fwd()` (cosine ramp at 50Hz, 30°/s max)
- **Legacy mode** (`--use-traj`): uses `send_home_confirmed()` (JointTrajectory)

---

## Recommended Test Commands

```bash
# Step 1: Launch robot with ForwardCommandController
ros2 launch openarm_bringup openarm.bimanual.launch.py \
    robot_controller:=forward_position_controller

# Step 2: Run teleoperation with homing
python3 ./placo_ik_online_profiler_ws_mesh.py --home-first --arm right

# Step 3: Record control CSV for analysis
python3 scripts/controller_layer/analy_arm_controller.py \
    --mode topic --arm right

# Step 4: Analyze vibration
python3 results/analyze_vibration.py results/<csv_file>.csv
```

### If ForwardCommandController is not feasible

```bash
# Legacy mode with JointTrajectoryController (may vibrate)
python3 ./placo_ik_online_profiler_ws_mesh.py --home-first --arm right --use-traj
```

## Verification Checklist

1. **ori_err_deg** in CSV should consistently be < 3° during rotation
2. **max_joint_delta_deg** should stay < 5° during normal tracking
3. **joint_jump_guard** should fire rarely (< 1% of steps)
4. **iterations** median should be 3–8 (near-target), up to 15–20 for large moves
5. No audible motor buzz or visible arm vibration during RPY rotation
6. **cmd/state ratio** ≈ 1.0× (verify with `analyze_vibration.py`)
7. **Direction reversal** < 20% during smooth motion

## Vibration Analysis Tool

```bash
# Single CSV analysis with plot
python3 results/analyze_vibration.py results/<csv>.csv

# Compare two runs
python3 results/analyze_vibration.py results/test3.csv results/test1.csv \
    --labels "ForwardCmd" "JointTraj"

# Text-only (no matplotlib needed)
python3 results/analyze_vibration.py results/<csv>.csv --no-plot
```

