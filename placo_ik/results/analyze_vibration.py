#!/usr/bin/env python3
"""
Vibration Diagnostic Tool for OpenArm Teleoperation
====================================================

Analyzes CSV files recorded by analy_arm_controller.py to:
1. Compare command vs state signals (cmd_delta_σ / state_delta_σ ratio)
2. Measure direction reversal rate (zigzag detection)
3. Perform FFT to find dominant vibration frequencies
4. Generate per-joint diagnostic plots

Usage:
  python3 analyze_vibration.py <csv_file>
  python3 analyze_vibration.py <csv_file_A> <csv_file_B>  --labels "test3" "test1"
  python3 analyze_vibration.py <csv_file> --no-plot        # text-only mode

The CSV must have columns: time, <joint>_cmd, <joint>_state, <joint>_err
(as produced by analy_arm_controller.py)
"""

import csv, sys, os, argparse
import numpy as np

# ── Plotting (optional) ──────────────────────────────────────────────────────
try:
    import matplotlib
    for backend in ['TkAgg', 'Qt5Agg', 'Agg']:
        try:
            matplotlib.use(backend)
            break
        except Exception:
            continue
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


def load_csv(path):
    """Load CSV into dict of numpy arrays."""
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames
        for r in reader:
            rows.append(r)
    data = {}
    for k in fields:
        data[k] = np.array([float(r[k]) for r in rows])
    return data, fields


def detect_joints(fields):
    """Find joint base names from CSV columns."""
    joints = []
    for f in fields:
        if f.endswith('_cmd'):
            jname = f[:-4]
            if jname + '_state' in fields and jname + '_err' in fields:
                joints.append(jname)
    return joints


def _rolling_reversal(cmd_delta, win=50):
    """
    Compute rolling direction reversal rate.

    A "reversal" = consecutive nonzero deltas have opposite signs.
    For a smooth trajectory: <20%. For a staircase: >50%.

    Returns an array of the same length as cmd_delta.
    """
    n = len(cmd_delta)
    result = np.zeros(n)
    for i in range(n):
        lo = max(0, i - win + 1)
        window = cmd_delta[lo:i+1]
        # Only count nonzero deltas (ignore standstill)
        nz = window[np.abs(window) > 1e-9]
        if len(nz) < 2:
            result[i] = 0.0
            continue
        signs = np.sign(nz)
        # Count how many consecutive pairs have opposite signs
        reversals = np.sum(signs[1:] * signs[:-1] < 0)
        result[i] = reversals / (len(nz) - 1) * 100.0
    return result


def analyze(data, joints, label=""):
    """Compute all vibration metrics. Returns a dict per joint."""
    ts = data['time']
    dt = np.diff(ts)
    fs = 1.0 / np.mean(dt)
    n = len(ts)

    print(f"\n{'='*80}")
    print(f"  {label}  ({n} rows, {fs:.1f} Hz sample rate)")
    print(f"{'='*80}")
    print(f"  Duration: {ts[-1]:.1f}s  dt: mean={np.mean(dt)*1000:.1f}ms  "
          f"std={np.std(dt)*1000:.1f}ms  min={np.min(dt)*1000:.1f}ms  max={np.max(dt)*1000:.1f}ms")

    results = {}
    for j in joints:
        cmd   = data[j + '_cmd']
        state = data[j + '_state']
        err   = data[j + '_err']

        cmd_delta   = np.diff(cmd)
        state_delta = np.diff(state)
        cmd_delta_deg   = np.degrees(cmd_delta)
        state_delta_deg = np.degrees(state_delta)
        err_deg = np.degrees(err)

        # ── Direction reversal rate ──────────────────────────────────────
        # A "reversal" = consecutive nonzero cmd deltas have opposite signs.
        # For smooth motion: <20%. For a zigzagging staircase: ~50-60%.
        nz_delta = cmd_delta[np.abs(cmd_delta) > 1e-9]
        if len(nz_delta) >= 2:
            nz_signs = np.sign(nz_delta)
            reversals = np.sum(nz_signs[1:] * nz_signs[:-1] < 0)
            reversal_pct = reversals / (len(nz_signs) - 1) * 100
        else:
            reversal_pct = 0.0

        # Rolling reversal for plotting
        roll_rev = _rolling_reversal(cmd_delta)

        # ── cmd / state oscillation ratio ────────────────────────────────
        cmd_sigma   = np.std(cmd_delta_deg)
        state_sigma = np.std(state_delta_deg)
        ratio = cmd_sigma / max(0.001, state_sigma)

        # Tracking error RMS
        err_rms = np.degrees(np.sqrt(np.mean(err**2)))

        # State oscillation when command is constant
        cmd_const_mask = np.abs(cmd_delta) < np.radians(0.01)
        if np.sum(cmd_const_mask) > 10:
            state_osc_const = np.std(np.degrees(state_delta[cmd_const_mask]))
        else:
            state_osc_const = 0.0

        # ── FFT on BOTH cmd and state deltas ─────────────────────────────
        cmd_fft_peaks = []
        state_fft_peaks = []
        cmd_fft_data = None
        state_fft_data = None
        fft_freqs = None

        if len(cmd_delta) > 64:
            fft_freqs = np.fft.rfftfreq(len(cmd_delta_deg), d=1.0/fs)
            mask = fft_freqs > 1.0  # only > 1 Hz

            # CMD FFT
            cmd_fft_data = np.abs(np.fft.rfft(cmd_delta_deg - np.mean(cmd_delta_deg)))
            if np.any(mask):
                peak_idx = np.argsort(cmd_fft_data[mask])[-3:][::-1]
                for idx in peak_idx:
                    cmd_fft_peaks.append((fft_freqs[mask][idx], cmd_fft_data[mask][idx]))

            # STATE FFT
            state_fft_data = np.abs(np.fft.rfft(state_delta_deg - np.mean(state_delta_deg)))
            if np.any(mask):
                peak_idx = np.argsort(state_fft_data[mask])[-3:][::-1]
                for idx in peak_idx:
                    state_fft_peaks.append((fft_freqs[mask][idx], state_fft_data[mask][idx]))

        short = j.split('_')[-1] if 'joint' in j else j
        results[j] = {
            'cmd_sigma': cmd_sigma, 'state_sigma': state_sigma,
            'ratio': ratio, 'reversal_pct': reversal_pct,
            'err_rms': err_rms, 'state_osc_const': state_osc_const,
            'cmd_fft_peaks': cmd_fft_peaks, 'state_fft_peaks': state_fft_peaks,
            'cmd_fft_data': cmd_fft_data, 'state_fft_data': state_fft_data,
            'fft_freqs': fft_freqs,
            'short': short, 'roll_rev': roll_rev,
            'cmd': cmd, 'state': state, 'err': err,
            'cmd_delta': cmd_delta, 'state_delta': state_delta,
            'cmd_delta_deg': cmd_delta_deg, 'state_delta_deg': state_delta_deg,
        }

        cmd_fft_str = "  ".join(f"{f:.1f}Hz({a:.1f})" for f, a in cmd_fft_peaks) if cmd_fft_peaks else "none"
        state_fft_str = "  ".join(f"{f:.1f}Hz({a:.1f})" for f, a in state_fft_peaks) if state_fft_peaks else "none"
        print(f"\n  {short}:")
        print(f"    cmd_delta_σ  = {cmd_sigma:.4f}°/step   (IK output jitter)")
        print(f"    state_delta_σ= {state_sigma:.4f}°/step   (motor actual jitter)")
        print(f"    ratio (cmd/state) = {ratio:.2f}×   {'⚠ CMD JITTER' if ratio > 1.5 else '✓ OK'}")
        print(f"    err_rms      = {err_rms:.4f}°")
        print(f"    dir_reversals= {reversal_pct:.1f}%    {'⚠ ZIGZAG' if reversal_pct > 40 else '✓ SMOOTH'}")
        print(f"    state_osc_const= {state_osc_const:.4f}°/step  "
              f"{'⚠ RINGS' if state_osc_const > 0.1 else '✓ QUIET'}")
        print(f"    FFT cmd      : {cmd_fft_str}")
        print(f"    FFT state    : {state_fft_str}")

    # Summary ranking
    print(f"\n  ── Vibration ranking (by cmd/state ratio) ──")
    ranking = sorted(results.items(), key=lambda x: -x[1]['ratio'])
    for jname, r in ranking:
        bar = '█' * int(min(r['ratio'], 5) * 10)
        print(f"    {r['short']:7s}  ratio={r['ratio']:.2f}×  "
              f"rev={r['reversal_pct']:.0f}%  σ_cmd={r['cmd_sigma']:.3f}°  {bar}")

    return results, fs


def plot_diagnostics(ts, results, fs, label="", save_path=None):
    """Generate comprehensive diagnostic plots."""
    if not HAS_MPL:
        print("  ⚠ matplotlib not available — skipping plots")
        return

    joints = list(results.keys())
    n_joints = len(joints)

    fig = plt.figure(figsize=(22, 4 * n_joints), constrained_layout=True)
    fig.suptitle(f"Vibration Diagnostics — {label}", fontsize=14, fontweight='bold')

    gs = GridSpec(n_joints, 4, figure=fig, width_ratios=[3, 1.5, 2, 1.5])

    for idx, jname in enumerate(joints):
        r = results[jname]
        t = ts[:len(r['cmd'])]
        cmd_deg   = np.degrees(r['cmd'])
        state_deg = np.degrees(r['state'])

        # ── Panel 1: CMD vs STATE time series ────────────────────────────
        ax1 = fig.add_subplot(gs[idx, 0])
        ax1.plot(t, cmd_deg, 'r-', linewidth=0.5, alpha=0.8, label='cmd')
        ax1.plot(t, state_deg, 'b-', linewidth=0.5, alpha=0.8, label='state')
        ax1.set_ylabel(f"{r['short']} (°)")
        ax1.legend(loc='upper right', fontsize=7)
        ax1.grid(True, alpha=0.3)
        # Add ratio annotation
        ax1.text(0.02, 0.95, f"ratio={r['ratio']:.2f}×  rev={r['reversal_pct']:.0f}%",
                 transform=ax1.transAxes, fontsize=7, verticalalignment='top',
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.5))
        if idx == 0:
            ax1.set_title('Command (red) vs State (blue)')
        if idx == n_joints - 1:
            ax1.set_xlabel('time (s)')

        # ── Panel 2: Delta histogram ─────────────────────────────────────
        ax2 = fig.add_subplot(gs[idx, 1])
        bins = np.linspace(-5, 5, 100)
        ax2.hist(r['cmd_delta_deg'], bins=bins, alpha=0.7, color='red',
                 label=f'cmd σ={r["cmd_sigma"]:.2f}°', density=True)
        ax2.hist(r['state_delta_deg'], bins=bins, alpha=0.7, color='blue',
                 label=f'state σ={r["state_sigma"]:.2f}°', density=True)
        ax2.legend(fontsize=6, loc='upper right')
        ax2.set_xlim(-5, 5)
        if idx == 0:
            ax2.set_title('Step Δ distribution (°/step)')

        # ── Panel 3: FFT of BOTH cmd and state delta ─────────────────────
        ax3 = fig.add_subplot(gs[idx, 2])
        if r['cmd_fft_data'] is not None and r['fft_freqs'] is not None:
            freqs = r['fft_freqs']
            mask = freqs > 0.5
            # Plot both cmd and state FFT
            ax3.plot(freqs[mask], r['cmd_fft_data'][mask], 'r-',
                     linewidth=0.7, alpha=0.8, label='cmd')
            ax3.plot(freqs[mask], r['state_fft_data'][mask], 'b-',
                     linewidth=0.7, alpha=0.8, label='state')
            ax3.set_xlim(0.5, fs/2)
            ax3.legend(fontsize=6, loc='upper right')
            # Mark cmd peaks
            for f, a in r['cmd_fft_peaks'][:2]:
                ax3.axvline(f, color='darkred', linewidth=0.5, linestyle='--', alpha=0.5)
                ax3.annotate(f'{f:.1f}Hz', (f, a), fontsize=6, color='darkred')
        if idx == 0:
            ax3.set_title('FFT: cmd (red) vs state (blue)')
        if idx == n_joints - 1:
            ax3.set_xlabel('Frequency (Hz)')

        # ── Panel 4: Rolling direction reversal ──────────────────────────
        ax4 = fig.add_subplot(gs[idx, 3])
        roll_rev = r['roll_rev']
        t_delta = t[:len(roll_rev)]
        if len(roll_rev) > 0:
            ax4.plot(t_delta, roll_rev, 'orange', linewidth=0.7)
            ax4.axhline(50, color='red', linewidth=0.5, linestyle='--', alpha=0.5)
            ax4.axhline(20, color='green', linewidth=0.5, linestyle='--', alpha=0.5)
            ax4.set_ylim(0, 100)
            ax4.set_ylabel('%')
            # Add overall average
            avg = r['reversal_pct']
            ax4.axhline(avg, color='darkorange', linewidth=1.0, linestyle='-', alpha=0.5)
            ax4.text(0.98, 0.95, f"avg={avg:.0f}%",
                     transform=ax4.transAxes, fontsize=7, ha='right', va='top',
                     bbox=dict(boxstyle='round,pad=0.3', facecolor='orange', alpha=0.3))
        if idx == 0:
            ax4.set_title('Dir reversal % (win=50)')
        if idx == n_joints - 1:
            ax4.set_xlabel('time (s)')

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\n  [plot] saved → {save_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Vibration diagnostics for OpenArm teleoperation CSVs",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("csv_files", nargs='+', help="One or two CSV files to analyze")
    parser.add_argument("--labels", nargs='*', default=None,
                        help="Labels for each CSV file (default: filenames)")
    parser.add_argument("--no-plot", action="store_true", dest="no_plot",
                        help="Text-only analysis (no matplotlib)")
    parser.add_argument("--save-plot", default=None, dest="save_plot",
                        help="Save plot to this path instead of showing")
    parser.add_argument("--arm", default="right", choices=["right", "left"],
                        help="Which arm's joints to analyze (default: right)")
    args = parser.parse_args()

    if args.labels and len(args.labels) != len(args.csv_files):
        print("Error: --labels count must match number of CSV files")
        sys.exit(1)

    for i, path in enumerate(args.csv_files):
        if not os.path.isfile(path):
            print(f"Error: file not found: {path}")
            sys.exit(1)

        label = args.labels[i] if args.labels else os.path.basename(path)
        data, fields = load_csv(path)
        joints = detect_joints(fields)

        if not joints:
            print(f"  ⚠ No joint columns found in {path}")
            continue

        # Filter to requested arm
        arm_joints = [j for j in joints if args.arm in j]
        if not arm_joints:
            arm_joints = joints  # fallback: use all

        results, fs = analyze(data, arm_joints, label)

        if not args.no_plot and HAS_MPL:
            save = None
            if args.save_plot:
                base, ext = os.path.splitext(args.save_plot)
                save = f"{base}_{i}{ext}" if len(args.csv_files) > 1 else args.save_plot
            elif len(args.csv_files) == 1:
                save = os.path.splitext(path)[0] + '_vibration_diag.png'
            plot_diagnostics(data['time'], results, fs, label, save_path=save)

    # Interpretation guide
    print(f"\n{'─'*80}")
    print("  INTERPRETATION GUIDE:")
    print("  • ratio > 1.5× → command jitters MORE than motor (staircase from controller)")
    print("  • reversal > 40% → command zigzags (step-response artifact)")
    print("  • state_osc_const > 0.1° → motor rings when command holds (PID overshoot)")
    print("  • If ratio ≈ 1.0 AND reversal < 20% → controller is tracking well")
    print("  • FFT: if cmd (red) >> state (blue) → controller is damping the jitter")
    print(f"{'─'*80}")


if __name__ == '__main__':
    main()
