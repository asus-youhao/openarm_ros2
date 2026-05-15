# Orientation Jitter Fixes

This note records the first-pass changes for reducing right-arm vibration during
VR roll/pitch/yaw teleoperation with `placo_ik_online_profiler_ws_mesh.py`.

## What Changed

1. Safer trajectory timing defaults
   - `--rate`: `50 Hz`
   - `--horizon`: `60 ms`
   - This avoids sending 1 ms trajectory points at 100 Hz.

2. Orientation delta calibration
   - XYZ deltas were already rotated by `calib_q`.
   - Quaternion deltas are now also mapped through calibration:
     `dq_arm = calib_q * dq * conj(calib_q)`.

3. Input-side orientation smoothing
   - Added SLERP EMA on the target quaternion after calibration.
   - Default: `--ori-lpf-alpha 0.35`
   - Use `--ori-lpf-alpha 1.0` to disable.

4. Orientation-aware IK convergence
   - Early exit now requires both position and orientation convergence.
   - Default solver cap is now `20` iterations.
   - Orientation task weight was reduced from `0.8` to `0.3`.
   - Regularization was increased from `1e-5` to `1e-4`.

5. Output joint LPF tuning
   - Default joint LPF is now:
     `0.3,0.3,0.3,0.3,0.6,0.6,0.6`
   - Shoulders/elbow are smoother; wrist joints remain more responsive.
   - IK now seeds from the filtered/commanded joint state instead of the raw IK
     solution, reducing filter-vs-seed mismatch.

6. Joint jump guard
   - Added `--joint-jump-guard-deg`.
   - Default: reject an IK output if any joint changes more than `15 deg` in
     one step.
   - Use `--joint-jump-guard-deg 0` to disable.

7. Less sensitive tracker reference reset
   - `ee_delta` reference reset gap is now `0.8 s`.
   - CLI: `--ee-delta-gap-sec`.

## Suggested Command

```bash
python3 ./placo_ik_online_profiler_ws_mesh.py \
  --home-first \
  --arm right
```

The command above uses the new defaults. For lighter output smoothing:

```bash
python3 ./placo_ik_online_profiler_ws_mesh.py \
  --home-first \
  --arm right \
  --lpf-alpha 0.4,0.4,0.4,0.4,0.7,0.7,0.7
```

## Tuning Order

1. If RPY still jitters, lower `--ori-lpf-alpha` to `0.2`.
2. If RPY feels too laggy, raise `--ori-lpf-alpha` to `0.5` or `0.7`.
3. If IK becomes sluggish, try `_W_REG = 1e-5` again.
4. If orientation tracking is too loose, try `_W_ORI = 0.4`.
5. If the jump guard rejects normal motion, raise `--joint-jump-guard-deg` to
   `20` or disable it for testing.

## Debug Checks

- Watch `/right/placo_profile`.
- Check `ori_err_deg`, `max_joint_delta_deg`, and `joint_jump_guard`.
- If `joint_jump_guard` frequently becomes `1`, the target orientation is still
  too noisy or the guard threshold is too low.
