#!/usr/bin/env python3
"""
Offline CSV plotter for Right Arm combined CSV produced by `analy_right_arm_topic.py`.

Plots one multi-subplot figure (2 columns) for the right arm joints. Each subplot
has a left y-axis for cmd/state and a right y-axis for error (independent scale).

Usage:
  python3 scripts/plot_right_arm_csv.py --csv path/to/right_arm_combined_*.csv --out out.png --error-scale deg --show

If --csv is omitted the script will look for the most recent CSV in
`analy_right_arm/csv/<YYYYMMDD>/`.
"""

import argparse
import csv
import os
import sys
import math
from pathlib import Path
import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MPL = True
except Exception:
    HAS_MPL = False

# Default joint list (must match CSV columns produced earlier)
RIGHT_ARM_JOINTS = [
    'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3',
    'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6',
    'openarm_right_joint7'
]


def find_latest_csv(base_dir='analy_right_arm/csv'):
    base = Path(base_dir)
    if not base.exists():
        return None
    # find date subdirs
    date_dirs = [d for d in base.iterdir() if d.is_dir()]
    if not date_dirs:
        return None
    latest_date_dir = max(date_dirs, key=lambda p: p.name)
    csv_files = sorted(latest_date_dir.glob('*.csv'), key=lambda p: p.stat().st_mtime)
    if not csv_files:
        return None
    return str(csv_files[-1])


def read_csv_simple(path):
    """Read CSV into a dict of columns (lists)."""
    data = {}
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        # initialize lists
        for col in header:
            data[col] = []
        for row in reader:
            # pad row if short
            if len(row) < len(header):
                row = row + [''] * (len(header) - len(row))
            for col, val in zip(header, row):
                try:
                    if val == '' or val.lower() == 'nan':
                        data[col].append(math.nan)
                    else:
                        data[col].append(float(val))
                except Exception:
                    # non-numeric (unlikely), store nan
                    data[col].append(math.nan)
    return data


def plot_combined(data, joints, out_path, error_scale='rad', show=False):
    if not HAS_MPL:
        print('Matplotlib not available. Install matplotlib to plot.')
        return

    n = len(joints)
    cols = 2
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(12, 3 * rows), squeeze=False)
    axes_flat = axes.flatten()

    # Determine time vector
    if 'time' not in data:
        print('CSV missing "time" column')
        return
    t = np.array(data['time'], dtype=float)
    if len(t) == 0:
        print('No samples in CSV')
        return
    t0 = t[0]
    t = t - t0

    for idx, joint in enumerate(joints):
        ax = axes_flat[idx]
        cmd_col = f'{joint}_cmd'
        state_col = f'{joint}_state'
        err_col = f'{joint}_err'

        cmd = np.array(data.get(cmd_col, [math.nan] * len(t)), dtype=float)
        state = np.array(data.get(state_col, [math.nan] * len(t)), dtype=float)
        err = np.array(data.get(err_col, [math.nan] * len(t)), dtype=float)

        # Scale conversion
        if error_scale == 'deg':
            factor = 180.0 / math.pi
            cmd_plot = cmd * factor
            state_plot = state * factor
            err_plot = err * factor
            unit = 'deg'
        else:
            cmd_plot = cmd
            state_plot = state
            err_plot = err
            unit = 'rad'

        # Plot cmd/state on left axis
        lines = []
        if np.any(~np.isnan(cmd_plot)):
            l_cmd = ax.plot(t, cmd_plot, label='cmd', linestyle='--', color='r')
            lines += l_cmd
        if np.any(~np.isnan(state_plot)):
            l_state = ax.plot(t, state_plot, label='state', linestyle='-', color='b')
            lines += l_state

        # Error on right axis
        err_lines = []
        if np.any(~np.isnan(err_plot)):
            ax_err = ax.twinx()
            l_err = ax_err.plot(t, err_plot, label='err', linestyle=':', color='g')
            err_lines += l_err
            try:
                ax_err.fill_between(t, np.nan_to_num(err_plot, nan=0.0), 0, color='g', alpha=0.06)
            except Exception:
                pass
            ax_err.set_ylabel(f'Error ({unit})', color='g')
            ax_err.tick_params(axis='y', colors='g')

            # Steady-state error: only calculate when cmd velocity ≈ 0 (robot stopped)
            steady_val = math.nan
            steady_std = math.nan
            steady_mask = np.zeros(len(cmd_plot), dtype=bool)
            fallback_mode = False
            
            if len(cmd_plot) >= 10:
                # Calculate cmd velocity (rate of change)
                cmd_vel = np.zeros_like(cmd_plot)
                cmd_vel[1:] = np.abs(np.diff(cmd_plot))
                
                # Adaptive threshold based on cmd range
                cmd_valid = cmd_plot[~np.isnan(cmd_plot)]
                if len(cmd_valid) >= 10:
                    cmd_range = np.ptp(cmd_valid)
                    # Velocity threshold: 0.5% of range per sample (strict for steady-state)
                    vel_threshold = max(0.005 * cmd_range, 0.001)  # fallback to 0.001 unit
                    
                    # Mark steady regions: velocity below threshold for consecutive samples
                    for i in range(5, len(cmd_vel) - 5):  # need margin
                        # Check if stable in a window
                        window_vel = cmd_vel[max(0, i-3):min(len(cmd_vel), i+4)]
                        if np.nanmax(window_vel) < vel_threshold:
                            steady_mask[i] = True
                    
                    # Extract steady-state errors
                    steady_errors = err_plot[steady_mask]
                    valid_steady = steady_errors[~np.isnan(steady_errors)]
                    
                    if len(valid_steady) >= 5:  # need sufficient samples
                        steady_val = float(np.mean(valid_steady))
                        steady_std = float(np.std(valid_steady))
                        
                        # Shade steady regions on the plot
                        for i in range(len(steady_mask)):
                            if steady_mask[i] and i < len(t):
                                ax.axvspan(t[i], t[i] + (t[1] - t[0]) if i+1 < len(t) else t[i], 
                                          color='yellow', alpha=0.1, linewidth=0)
                    else:
                        # Fallback: no steady regions found, use slowest motion window
                        fallback_mode = True
                        window_size = max(10, int(0.1 * len(cmd_vel)))  # 10% window
                        min_avg_vel = float('inf')
                        best_idx = 0
                        
                        for i in range(len(cmd_vel) - window_size + 1):
                            window_vel = cmd_vel[i:i+window_size]
                            avg_vel = np.nanmean(window_vel)
                            if avg_vel < min_avg_vel:
                                min_avg_vel = avg_vel
                                best_idx = i
                        
                        # Use errors from slowest window
                        fallback_errors = err_plot[best_idx:best_idx+window_size]
                        valid_fallback = fallback_errors[~np.isnan(fallback_errors)]
                        if len(valid_fallback) >= 3:
                            steady_val = float(np.mean(valid_fallback))
                            steady_std = float(np.std(valid_fallback))
                            # Mark fallback region
                            if best_idx < len(t):
                                end_idx = min(best_idx + window_size, len(t))
                                ax.axvspan(t[best_idx], t[end_idx-1], 
                                          color='orange', alpha=0.15, linewidth=0)
            
            # Plot steady-error mean and ±1σ band
            if not math.isnan(steady_val):
                label_prefix = 'quasi-steady' if fallback_mode else 'steady-err'
                l_steady = ax_err.axhline(steady_val, color='m', linestyle='--', linewidth=2, 
                                         label=f'{label_prefix}: {steady_val:.4f}')
                err_lines.append(l_steady)
                
                if not math.isnan(steady_std) and steady_std > 1e-9:
                    # ±1σ band
                    ax_err.axhspan(steady_val - steady_std, steady_val + steady_std, 
                                  color='m', alpha=0.15, linewidth=0)
                    # Add std to legend via dummy line
                    l_std = ax_err.plot([], [], ' ', label=f'±1σ: {steady_std:.4f}')[0]
                    err_lines.append(l_std)

        ax.set_title(joint)
        ax.set_xlabel('time (s)')
        ax.set_ylabel('cmd/state')
        ax.grid(True, linestyle=':', alpha=0.6)

        # Combine legends
        all_lines = lines + err_lines
        if all_lines:
            labels = [ln.get_label() for ln in all_lines]
            ax.legend(all_lines, labels, fontsize='small')

    # Hide unused axes
    for j in range(len(joints), len(axes_flat)):
        fig.delaxes(axes_flat[j])

    plt.suptitle(f'Right Arm Commands vs States (error_scale={error_scale})')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    try:
        plt.savefig(out_path, dpi=150)
        print(f'Saved plot: {out_path}')
    except Exception as e:
        print(f'Failed to save plot: {e}')

    if show:
        backend = matplotlib.get_backend().lower()
        if backend != 'agg' and os.environ.get('DISPLAY'):
            plt.show()
        else:
            print('Non-interactive backend or no DISPLAY; skipping plt.show()')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', help='Path to combined CSV file')
    parser.add_argument('--out', help='Output PNG path', default=None)
    parser.add_argument('--error-scale', choices=['rad','deg'], default='rad')
    parser.add_argument('--show', action='store_true', help='Show plot if interactive')
    args = parser.parse_args()

    csv_path = args.csv
    if csv_path is None:
        csv_path = find_latest_csv()
        if csv_path is None:
            print('No CSV found in analy_right_arm/csv, please provide --csv')
            sys.exit(1)

    csv_path = str(csv_path)
    if args.out is None:
        out_name = Path(csv_path).stem + '.png'
        out_path = str(Path('analy_right_arm/img') / out_name)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
    else:
        out_path = args.out

    data = read_csv_simple(csv_path)
    plot_combined(data, RIGHT_ARM_JOINTS, out_path, error_scale=args.error_scale, show=args.show)


if __name__ == '__main__':
    main()
