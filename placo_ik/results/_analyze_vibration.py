#!/usr/bin/env python3
"""Deep analysis of command vs state vibration patterns."""
import csv, sys, numpy as np
from collections import defaultdict

def analyze(path, label):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    
    n = len(rows)
    ts = np.array([float(r['time']) for r in rows])
    dt = np.diff(ts)
    
    print(f"\n{'='*75}")
    print(f"  {label}  ({n} rows)")
    print(f"{'='*75}")
    print(f"  Duration: {ts[-1]:.1f}s  dt: mean={np.mean(dt)*1000:.1f}ms  "
          f"std={np.std(dt)*1000:.1f}ms  min={np.min(dt)*1000:.1f}ms  max={np.max(dt)*1000:.1f}ms")
    
    joints = ['openarm_right_joint' + str(i) for i in range(1, 8)]
    
    # Per-joint analysis
    for j in joints:
        cmd = np.array([float(r[j+'_cmd']) for r in rows])
        state = np.array([float(r[j+'_state']) for r in rows])
        err = np.array([float(r[j+'_err']) for r in rows])
        
        # Command step sizes (inter-step deltas)
        cmd_delta = np.diff(cmd)
        cmd_delta_deg = np.degrees(cmd_delta)
        
        # State step sizes
        state_delta = np.diff(state)
        state_delta_deg = np.degrees(state_delta)
        
        # Tracking error
        err_deg = np.degrees(err)
        
        # Count "sign changes" in command deltas (staircase indicator)
        sign_changes = np.sum(np.diff(np.sign(cmd_delta)) != 0)
        direction_reversals = sign_changes / max(1, len(cmd_delta)) * 100
        
        # Command acceleration (2nd derivative) — jerk indicator
        cmd_accel = np.diff(cmd_delta)
        cmd_accel_deg = np.degrees(cmd_accel)
        
        # FFT on command signal to find dominant frequencies
        if len(cmd) > 64:
            fs = 1.0 / np.mean(dt)  # sample rate
            # Detrend
            cmd_detrend = cmd - np.linspace(cmd[0], cmd[-1], len(cmd))
            fft_vals = np.abs(np.fft.rfft(cmd_detrend))
            freqs = np.fft.rfftfreq(len(cmd_detrend), d=1.0/fs)
            # Top 3 peaks above 1 Hz
            mask = freqs > 1.0
            if np.any(mask):
                peak_idx = np.argsort(fft_vals[mask])[-3:]
                peak_freqs = freqs[mask][peak_idx]
                peak_amps = fft_vals[mask][peak_idx]
                fft_str = "  ".join(f"{f:.1f}Hz({a:.4f})" for f, a in zip(peak_freqs, peak_amps))
            else:
                fft_str = "none"
        else:
            fft_str = "too short"
        
        short = j.replace('openarm_right_', '')
        print(f"\n  {short}:")
        print(f"    cmd   range={np.degrees(np.ptp(cmd)):6.2f}°  "
              f"delta_deg: mean={np.mean(np.abs(cmd_delta_deg)):.4f}  "
              f"max={np.max(np.abs(cmd_delta_deg)):.4f}  "
              f"std={np.std(cmd_delta_deg):.4f}")
        print(f"    state range={np.degrees(np.ptp(state)):6.2f}°  "
              f"delta_deg: mean={np.mean(np.abs(state_delta_deg)):.4f}  "
              f"max={np.max(np.abs(state_delta_deg)):.4f}  "
              f"std={np.std(state_delta_deg):.4f}")
        print(f"    err   mean={np.mean(err_deg):+.4f}°  "
              f"std={np.std(err_deg):.4f}°  max={np.max(np.abs(err_deg)):.4f}°")
        print(f"    dir_reversals={direction_reversals:.1f}%  "
              f"cmd_accel_max={np.max(np.abs(cmd_accel_deg)):.4f}°/step²")
        print(f"    cmd FFT peaks: {fft_str}")
    
    # Summary: which joints have the most vibration?
    print(f"\n  ── Vibration ranking (by cmd direction reversal %) ──")
    rankings = []
    for j in joints:
        cmd = np.array([float(r[j+'_cmd']) for r in rows])
        cmd_delta = np.diff(cmd)
        sign_changes = np.sum(np.diff(np.sign(cmd_delta)) != 0)
        pct = sign_changes / max(1, len(cmd_delta)) * 100
        short = j.replace('openarm_right_', '')
        rankings.append((pct, short, np.degrees(np.std(np.diff(cmd)))))
    
    rankings.sort(reverse=True)
    for pct, name, std in rankings:
        bar = '█' * int(pct / 2)
        print(f"    {name:7s}  {pct:5.1f}%  σ={std:.4f}°/step  {bar}")

# Analyze both files
analyze(sys.argv[1], "test3 (current fixes)")
if len(sys.argv) > 2:
    analyze(sys.argv[2], "test1 (jitter_test1, heavier LPF)")
