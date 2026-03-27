#!/usr/bin/env python3
"""
Offline CSV plotter for Right Arm combined CSV.
Features:
 - Interactive directory/file selection
 - Performance metrics: ep, RMSE, Delay, and Settling Time (Ts)
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

def calculate_settling_time(t, cmd, err, tol=0.01):
    """計算從指令停止到誤差穩定在 tol 以內的時間 (Settling Time)"""
    if len(t) < 10 or np.all(np.isnan(cmd)): return 0.0
    
    cmd_vel = np.abs(np.diff(cmd, prepend=cmd[0]))
    # 找到最後一個明顯運動的點 (速度大於門檻)
    motion_indices = np.where(cmd_vel > 1e-4)[0]
    if len(motion_indices) == 0: return 0.0
    
    last_motion_idx = motion_indices[-1]
    
    # 從最後運動點往後找，看何時誤差完全進入容許範圍且不再出來
    ts = 0.0
    for i in range(last_motion_idx, len(err)):
        if np.all(np.abs(err[i:]) < tol):
            ts = t[i] - t[last_motion_idx]
            break
    return max(0.0, ts)

def estimate_phase_lag(t, cmd, state):
    if np.any(np.isnan(cmd)) or np.any(np.isnan(state)) or len(cmd) < 20:
        return 0.0
    c = cmd - np.nanmean(cmd)
    s = state - np.nanmean(state)
    correlation = np.correlate(c, s, mode='full')
    lags = np.arange(-len(c) + 1, len(c))
    lag_idx = np.argmax(correlation)
    dt = np.nanmean(np.diff(t))
    return lags[lag_idx] * dt * 1000.0

def read_csv_simple(path):
    data = {}
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        for col in header: data[col] = []
        for row in reader:
            if len(row) < len(header): row = row + [''] * (len(header) - len(row))
            for col, val in zip(header, row):
                try:
                    data[col].append(math.nan if val == '' or val.lower() == 'nan' else float(val))
                except: data[col].append(math.nan)
    return data

def plot_combined(data, joints, out_path, error_scale='rad', show=False):
    if not HAS_MPL: return
    n = len(joints)
    cols = 2
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(14, 3.5 * rows), squeeze=False)
    axes_flat = axes.flatten()

    t = np.array(data['time'], dtype=float)
    t = t - t[0]

    for idx, joint in enumerate(joints):
        ax = axes_flat[idx]
        cmd = np.array(data.get(f'{joint}_cmd', [math.nan]*len(t)), dtype=float)
        state = np.array(data.get(f'{joint}_state', [math.nan]*len(t)), dtype=float)
        err = np.array(data.get(f'{joint}_err', [math.nan]*len(t)), dtype=float)

        if error_scale == 'deg':
            f = 180.0 / math.pi
            cmd_p, state_p, err_p, unit = cmd*f, state*f, err*f, 'deg'
            tol = 0.5 # 0.5 deg tolerance for Ts
        else:
            cmd_p, state_p, err_p, unit = cmd, state, err, 'rad'
            tol = 0.01 # 0.01 rad tolerance for Ts

        # Metrics
        valid_err = err_p[~np.isnan(err_p)]
        ep = np.max(np.abs(valid_err)) if len(valid_err)>0 else 0
        rmse = np.sqrt(np.mean(valid_err**2)) if len(valid_err)>0 else 0
        delay = estimate_phase_lag(t, cmd_p, state_p)
        ts = calculate_settling_time(t, cmd_p, err_p, tol=tol)

        # Plotting
        ax.plot(t, cmd_p, 'r--', label='cmd', alpha=0.7)
        ax.plot(t, state_p, 'b-', label='state', alpha=0.7)
        ax_err = ax.twinx()
        ax_err.plot(t, err_p, 'g:', label='err', lw=1)
        ax_err.fill_between(t, np.nan_to_num(err_p), 0, color='g', alpha=0.05)
        
        stats = f"ep: {ep:.3f}\nRMSE: {rmse:.3f}\nDelay: {delay:.1f}ms\nTs: {ts:.2f}s"
        ax.text(0.02, 0.95, stats, transform=ax.transAxes, va='top', fontsize=8, 
                family='monospace', bbox=dict(facecolor='white', alpha=0.7))
        
        ax.set_title(joint, fontweight='bold')
        ax.grid(True, ls=':', alpha=0.5)

    plt.suptitle(f"Analysis: {Path(out_path).stem}\nScale: {error_scale}", fontsize=12)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(out_path, dpi=150)
    print(f"Saved: {out_path}")
    if show and os.environ.get('DISPLAY'): plt.show()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', help='Path to CSV')
    parser.add_argument('--error-scale', choices=['rad','deg'], default='rad')
    parser.add_argument('--show', action='store_true')
    args = parser.parse_args()

    csv_path = args.csv
    if not csv_path:
        csv_path = interactive_select_csv()
    
    if not csv_path:
        print("No file selected. Exiting.")
        sys.exit(1)

    out_name = Path(csv_path).stem + "_full_analys.png"
    # out_path = str(Path('analy_right_arm/img') / out_name)
    out_path = str(Path('analy_arm/img') / out_name)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    data = read_csv_simple(csv_path)
    plot_combined(data, RIGHT_ARM_JOINTS, out_path, args.error_scale, args.show)

if __name__ == '__main__':
    main()