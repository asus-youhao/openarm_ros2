#!/usr/bin/env python3
"""
Offline CSV plotter for Right Arm combined CSV produced by `analy_right_arm_topic.py`.

Enhanced with:
 - Maximum Tracking Error (ep)
 - RMSE
 - Phase Lag (Delay) estimation
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

def interactive_select_csv(base_dir='analy_right_arm/csv'):
    base = Path(base_dir)
    if not base.exists():
        print(f"Error: Base directory {base_dir} not found.")
        return None

    # 1. 選擇日期資料夾
    date_dirs = sorted([d for d in base.iterdir() if d.is_dir()], key=lambda x: x.name, reverse=True)
    if not date_dirs:
        print("No date directories found.")
        return None

    print("\n--- Available Dates ---")
    for i, d in enumerate(date_dirs):
        print(f"[{i}] {d.name}")
    
    try:
        idx = int(input(f"Select date index (default 0): ") or 0)
        selected_date_dir = date_dirs[idx]
    except (ValueError, IndexError):
        print("Invalid selection.")
        return None

    # 2. 選擇該日期下的 CSV 檔案
    csv_files = sorted(list(selected_date_dir.glob('*.csv')), key=lambda x: x.stat().st_mtime, reverse=True)
    if not csv_files:
        print(f"No CSV files found in {selected_date_dir.name}")
        return None

    print(f"\n--- CSV Files in {selected_date_dir.name} ---")
    for i, f in enumerate(csv_files):
        print(f"[{i}] {f.name}")
    
    try:
        idx = int(input(f"Select CSV index (default 0): ") or 0)
        return str(csv_files[idx])
    except (ValueError, IndexError):
        print("Invalid selection.")
        return None

def find_latest_csv(base_dir='analy_right_arm/csv'):
    base = Path(base_dir)
    if not base.exists():
        return None
    date_dirs = [d for d in base.iterdir() if d.is_dir()]
    if not date_dirs:
        return None
    latest_date_dir = max(date_dirs, key=lambda p: p.name)
    csv_files = sorted(latest_date_dir.glob('*.csv'), key=lambda p: p.stat().st_mtime)
    if not csv_files:
        return None
    return str(csv_files[-1])

def read_csv_simple(path):
    data = {}
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        for col in header:
            data[col] = []
        for row in reader:
            if len(row) < len(header):
                row = row + [''] * (len(header) - len(row))
            for col, val in zip(header, row):
                try:
                    if val == '' or val.lower() == 'nan':
                        data[col].append(math.nan)
                    else:
                        data[col].append(float(val))
                except Exception:
                    data[col].append(math.nan)
    return data

def estimate_phase_lag(t, cmd, state):
    """Estimate time delay in ms using cross-correlation."""
    if np.any(np.isnan(cmd)) or np.any(np.isnan(state)) or len(cmd) < 20:
        return 0.0
    
    # Remove DC offset and normalize
    c = cmd - np.nanmean(cmd)
    s = state - np.nanmean(state)
    
    # Compute cross-correlation
    correlation = np.correlate(c, s, mode='full')
    lags = np.arange(-len(c) + 1, len(c))
    lag_idx = np.argmax(correlation)
    actual_lag = lags[lag_idx]
    
    # Convert sample lag to time (ms)
    dt = np.nanmean(np.diff(t))
    return actual_lag * dt * 1000.0

def plot_combined(data, joints, out_path, error_scale='rad', show=False):
    if not HAS_MPL:
        print('Matplotlib not available. Install matplotlib to plot.')
        return

    n = len(joints)
    cols = 2
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(14, 3.5 * rows), squeeze=False)
    axes_flat = axes.flatten()

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

        if error_scale == 'deg':
            factor = 180.0 / math.pi
            cmd_plot, state_plot, err_plot = cmd * factor, state * factor, err * factor
            unit = 'deg'
        else:
            cmd_plot, state_plot, err_plot = cmd, state, err
            unit = 'rad'

        # --- Performance Metrics Calculation ---
        valid_mask = ~np.isnan(err_plot)
        v_err = err_plot[valid_mask]
        
        ep = np.max(np.abs(v_err)) if len(v_err) > 0 else 0.0
        rmse = np.sqrt(np.mean(v_err**2)) if len(v_err) > 0 else 0.0
        delay_ms = estimate_phase_lag(t, cmd_plot, state_plot)

        # Plot cmd/state on left axis
        lines = []
        if np.any(~np.isnan(cmd_plot)):
            lines += ax.plot(t, cmd_plot, label='cmd', linestyle='--', color='r', alpha=0.8)
        if np.any(~np.isnan(state_plot)):
            lines += ax.plot(t, state_plot, label='state', linestyle='-', color='b', alpha=0.8)

        # Error on right axis
        err_lines = []
        if np.any(~np.isnan(err_plot)):
            ax_err = ax.twinx()
            l_err = ax_err.plot(t, err_plot, label='err', linestyle=':', color='g', lw=1.2)
            err_lines += l_err
            ax_err.fill_between(t, np.nan_to_num(err_plot, nan=0.0), 0, color='g', alpha=0.07)
            ax_err.set_ylabel(f'Error ({unit})', color='g')
            ax_err.tick_params(axis='y', colors='g')

            # --- Steady State Analysis (Reuse logic) ---
            steady_val = math.nan
            if len(cmd_plot) >= 10:
                cmd_vel = np.abs(np.diff(cmd_plot, prepend=cmd_plot[0]))
                vel_threshold = max(0.005 * (np.nanmax(cmd_plot) - np.nanmin(cmd_plot)), 0.001)
                steady_mask = cmd_vel < vel_threshold
                
                v_steady = err_plot[steady_mask & ~np.isnan(err_plot)]
                if len(v_steady) > 5:
                    steady_val = np.mean(v_steady)
                    ax_err.axhline(steady_val, color='m', ls='--', alpha=0.6)
                    # Shade steady regions
                    ax.fill_between(t, ax.get_ylim()[0], ax.get_ylim()[1], where=steady_mask, 
                                    color='yellow', alpha=0.05, transform=ax.get_xaxis_transform())

        # Add Metrics Card
        stats_text = (f"ep: {ep:.3f} {unit}\n"
                      f"RMSE: {rmse:.3f} {unit}\n"
                      f"Delay: {delay_ms:.1f} ms")
        ax.text(0.02, 0.96, stats_text, transform=ax.transAxes, va='top', fontsize=8,
                family='monospace', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        ax.set_title(f"{joint}", fontweight='bold')
        ax.set_xlabel('time (s)')
        ax.set_ylabel(f'Pos ({unit})')
        ax.grid(True, linestyle=':', alpha=0.5)

        all_lines = lines + err_lines
        ax.legend(all_lines, [l.get_label() for l in all_lines], loc='lower right', fontsize='x-small')

    for j in range(len(joints), len(axes_flat)):
        fig.delaxes(axes_flat[j])

    plt.suptitle(f'OpenArm Right Performance Analysis (Scale: {error_scale})', fontsize=14)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(out_path, dpi=150)
    print(f'Saved analysis plot: {out_path}')

    if show:
        if os.environ.get('DISPLAY'):
            plt.show()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', help='Path to combined CSV file')
    parser.add_argument('--out', help='Output PNG path', default=None)
    parser.add_argument('--error-scale', choices=['rad','deg'], default='rad')
    parser.add_argument('--show', action='store_true', help='Show plot')
    args = parser.parse_args()

    csv_path = args.csv
    if not csv_path:
        csv_path = interactive_select_csv()
    
    if not csv_path:
        print("No file selected. Exiting.")
        sys.exit(1)
    if args.out is None:
        out_path = str(Path('analy_right_arm/img') / (Path(csv_path).stem + '_analysis.png'))
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
    else:
        out_path = args.out

    data = read_csv_simple(csv_path)
    plot_combined(data, RIGHT_ARM_JOINTS, out_path, error_scale=args.error_scale, show=args.show)

if __name__ == '__main__':
    main()