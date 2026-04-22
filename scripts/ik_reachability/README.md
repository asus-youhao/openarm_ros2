# IK Reachability Analysis — Scripts & Data Guide

## Folder Layout

```
ik_reachability/
├── [Python scripts]               ← run from this directory
├── moveit_client/                 ← MoveIt2 client examples & guides
├── tf_chains.json                 ← saved TF arm chains (offline plotting)
├── VR_AR_MOVEIT_EVALUATION_PLAN.md
│
├── data/
│   ├── reachability/              ← INPUT: reachability maps (from ik_reachability_sampler.py)
│   │   ├── right_reachability_.csv   coarse 0.1m grid, right arm
│   │   ├── left_reachability_.csv    coarse 0.1m grid, left arm
│   │   ├── right_fine.csv            refined 0.05m grid, right arm
│   │   └── left_fine.csv             refined 0.05m grid, left arm
│   ├── waypoints/                 ← merged / curated waypoint sets
│   │   ├── 20260410_merged_waypoints.csv
│   │   └── 20260410_merged_waypoints_1.csv
│   └── (recordings ignored by .gitignore)
│       └── 20260410/              ← raw joint-state recordings + derived EE poses
│
└── results/
    ├── ik_compute_timing/         ← /compute_ik service benchmark (ik_timing_benchmark.py)
    │   ├── ik_timing_right_*.csv     per-orientation timing (6 orientations)
    │   ├── ik_timing_right_*.png     histogram plots
    │   ├── EE_orientations.png       EE frame visualisation diagram
    │   ├── plot_ee_orientations.py   script that produced EE_orientations.png
    │   └── legacy/                   older one-off timing files
    ├── waypoint_runner/           ← MoveIt2 full-plan timing (csv_waypoint_runner.py)
    │   ├── ompl/                     OMPL, 0.1m grid, ~30 pts
    │   ├── ompl_3000/                OMPL, 3000 pts
    │   ├── pilz_ptp/                 Pilz PTP, 0.1m grid
    │   ├── pilz_lin/                 Pilz LIN, 0.1m grid
    │   ├── pilz_ptp_3000/            Pilz PTP, 3000 pts
    │   ├── pilz_lin_3000/            Pilz LIN, 3000 pts
    │   ├── ompl_legacy/              early OMPL run (before planner folders)
    │   └── legacy/                   misc older root-level outputs
    └── reachability_plots/        ← 3-D reachability scatter plots
        ├── bimanual/                 bimanual overlap plots
        └── ik/                       single-arm IK scatter plots
```

---

## Data Flow

### Pipeline A — Build reachability map → run waypoints

```
ik_reachability_sampler.py
    INPUT : none (samples a 3-D grid via MoveIt /compute_ik)
    OUTPUT: data/reachability/<arm>_reachability_.csv
              columns: x, y, z, reachable (1/0)
            (optional) 3-D scatter plot shown interactively

         ↓  INPUT CSV

csv_waypoint_runner.py  --csv data/reachability/right_reachability_.csv  --arm right
    INPUT : data/reachability/<arm>_reachability_.csv   (or _fine.csv / _waypoints.csv)
    OUTPUT: results/waypoint_runner/<arm>_waypoint_timing_<YYYYMMDD_HHMMSS>.csv
              columns: x, y, z, success, planning_ms, execution_ms, total_ms
                       [split-timing] ompl_ms, planned_duration_ms, motion_ms,
                                      n_waypoints, joint_path_len
                       [all-orient]   orient_name, qx, qy, qz, qw
            results/waypoint_runner/<arm>_waypoint_timing_<stamp>_hist.png
            results/waypoint_runner/<arm>_waypoint_timing_<stamp>_timeline.png
            results/waypoint_runner/<arm>_waypoint_timing_<stamp>_3d_heatmap.png
            results/waypoint_runner/<arm>_waypoint_timing_<stamp>_cdf.png
```

### Pipeline B — /compute_ik raw speed benchmark

```
ik_timing_benchmark.py
    INPUT : data/reachability/<arm>_reachability_.csv
    OUTPUT: results/ik_compute_timing/ik_timing_<arm>_<orient>.csv
              columns: x, y, z, ik_ms, success
            results/ik_compute_timing/ik_timing_right_*.png   (histogram + 3-D heatmap)
```

### Pipeline C — Plot reachability maps

```
plot_reachability_csv.py
    INPUT : data/reachability/left_reachability_.csv
            data/reachability/right_reachability_.csv
            (optional) tf_chains.json   for overlaying the arm skeleton
    OUTPUT: results/reachability_plots/  (PNG files when --save-root is passed)
    
  Usage:
    python3 plot_reachability_csv.py \
        --left  data/reachability/left_reachability_.csv \
        --right data/reachability/right_reachability_.csv \
        --save-root results/reachability_plots \
        --tf-file tf_chains.json
```

### Pipeline D — Convert joint-state recordings to EE waypoints

```
joint_states_to_ee_poses.py
    INPUT : 20260410/joint_states_sync_o6hand_<timestamp>.csv
              columns: timestamp, joint1…joint7 (from hardware recording)
    OUTPUT: 20260410/joint_states_sync_o6hand_<timestamp>_ee_poses.csv
              columns: stamp, j1…j7, x, y, z, qx, qy, qz, qw, roll, pitch, yaw
            20260410/joint_states_sync_o6hand_<timestamp>_waypoints.csv
              columns: x, y, z, reachable=1, qx, qy, qz, qw
              → compatible with csv_waypoint_runner.py (replays recorded EE path)
```

---

## Quick Reference — which script reads what

| Script | Reads | Writes |
|--------|-------|--------|
| `ik_reachability_sampler.py` | — (grid sampler) | `data/reachability/<arm>_reachability_.csv` |
| `ik_timing_benchmark.py` | `data/reachability/<arm>_reachability_.csv` | `results/ik_compute_timing/ik_timing_<arm>_<orient>.csv` + PNG |
| `csv_waypoint_runner.py` | `data/reachability/*.csv` or `20260410/*_waypoints.csv` | `results/waypoint_runner/<arm>_waypoint_timing_<stamp>_{hist,timeline,3d_heatmap,cdf}.png` + CSV |
| `plot_reachability_csv.py` | `data/reachability/<arm>_reachability_.csv`, `tf_chains.json` | `results/reachability_plots/*.png` |
| `joint_states_to_ee_poses.py` | `20260410/joint_states_sync_*.csv` | `20260410/*_ee_poses.csv`, `20260410/*_waypoints.csv` |
| `results/ik_compute_timing/plot_ee_orientations.py` | — (no CSV input) | `results/ik_compute_timing/EE_orientations.png` |

---

## Re-plotting from a saved timing CSV (no robot required)

```bash
python3 csv_waypoint_runner.py \
    --plot-csv results/waypoint_runner/ompl/ompl_recorded_timing.csv \
    --arm right --planner ompl
```

---

## Notes

- `results/` is auto-created by `csv_waypoint_runner.py` on first run.
- All timestamped outputs use `YYYYMMDD_HHMMSS` so runs never overwrite each other.
- `tf_chains.json` is saved by `plot_reachability_csv.py --live-tf --no-plot` when hardware is running; use `--tf-file tf_chains.json` for offline replay.
- `20260410/` contents are gitignored (see `.gitignore`). Only the derived `_waypoints.csv` files you explicitly add will be tracked.
