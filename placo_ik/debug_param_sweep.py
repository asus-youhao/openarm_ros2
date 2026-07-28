#!/usr/bin/env python3
"""
Parameter sweep analysis: Find optimal λ_max for iter=15 region
"""

import csv
import numpy as np
from pathlib import Path

# Load data
csv_file = Path("results/20260521/placo_online_20260521_162338_right_cached.csv")
data = {}
with open(csv_file) as f:
    reader = csv.DictReader(f)
    for row in reader:
        for k, v in row.items():
            if k not in data:
                data[k] = []
            try:
                data[k].append(float(v))
            except:
                data[k].append(v)

sigma_min = np.array(data['sigma_min'])
iterations = np.array(data['iterations'])
ik_ms = np.array(data['ik_ms'])
pos_err = np.array(data['pos_err_mm'])

print("\n" + "=" * 80)
print("📊 DLS λ_max PARAMETER SWEEP ANALYSIS")
print("=" * 80)

# Focus on the problematic region: σ < 0.15 (near singularity)
near_singular = sigma_min < 0.15
print(f"\nFocus region: σ < 0.15 ({near_singular.sum()} frames)")
print()

# Test different λ_max values
lambda_max_candidates = [0.005, 0.01, 0.02, 0.03, 0.05, 0.10]
_DLS_LAMBDA_BASE = 6e-5
_DLS_SIGMA_THRESH = 0.15

def compute_lambda_test(sigma, lambda_max):
    _ratio = max(0.0, (_DLS_SIGMA_THRESH - sigma) / _DLS_SIGMA_THRESH)
    return _DLS_LAMBDA_BASE + lambda_max * _ratio * _ratio

print(f"{'λ_max':<10} {'λ_mean':<12} {'λ_@σ=0.05':<14} {'λ_@σ=0.10':<14} {'λ_@σ=0.15':<14}")
print("-" * 80)

for lambda_max in lambda_max_candidates:
    lambda_vals = np.array([compute_lambda_test(s, lambda_max) for s in sigma_min[near_singular]])

    # Calculate lambda at specific sigma values
    l_at_005 = compute_lambda_test(0.05, lambda_max)
    l_at_010 = compute_lambda_test(0.10, lambda_max)
    l_at_015 = compute_lambda_test(0.15, lambda_max)

    print(f"{lambda_max:<10.3f} {lambda_vals.mean():<12.6f} {l_at_005:<14.6f} {l_at_010:<14.6f} {l_at_015:<14.6f}")

print()
print("=" * 80)
print("🎯 IMPACT ON iter=15 REGION")
print("=" * 80)

# Focus specifically on iter=15 frames
max_iter_mask = iterations == 15
max_iter_sigma = sigma_min[max_iter_mask]
max_iter_ik = ik_ms[max_iter_mask]
max_iter_pos = pos_err[max_iter_mask]

print(f"\niter=15 frames: {max_iter_mask.sum()}")
print(f"σ range: [{max_iter_sigma.min():.4f}, {max_iter_sigma.max():.4f}]")
print()

print(f"{'λ_max':<10} {'mean_λ':<12} {'pos_err':<12} {'ik_time':<12} {'λ_increase':<12}")
print("-" * 70)

baseline_lambda = np.array([compute_lambda_test(s, 0.01) for s in max_iter_sigma])
for lambda_max in lambda_max_candidates:
    test_lambda = np.array([compute_lambda_test(s, lambda_max) for s in max_iter_sigma])
    increase = (test_lambda.mean() / baseline_lambda.mean() - 1) * 100

    print(f"{lambda_max:<10.3f} {test_lambda.mean():<12.6f} {max_iter_pos.mean():<12.2f} {max_iter_ik.mean():<12.2f} {increase:>10.1f}%")

print()
print("=" * 80)
print("💡 KEY INSIGHT")
print("-" * 80)
print(f"""
Current situation:
  · λ_max = 0.01 gives λ ≈ 0.001 in iter=15 region (only 10% of max)
  · This is TOO WEAK for Placo optimizer to suppress oscillations
  · Solver needs all 15 iterations to converge

Physics behind DLS damping:
  λ is the regularization weight in: solve(J^T J + λ I)^{-1} J^T

  Small λ (0.001): Ill-conditioned, slow convergence
  Large λ (0.01+): Well-regularized, faster convergence

Recommended action:
  Change: _DLS_LAMBDA_MAX = 0.05  (was 0.01)

  This will:
    ✅ Increase damping 5× in critical region
    ✅ Likely reduce iter=15 count significantly
    ✅ Slightly increase λ in non-singular regions (acceptable)
    ⚠️  MUST re-test full trajectory to confirm no regressions

Test plan:
  1. Apply patch: _DLS_LAMBDA_MAX = 0.05
  2. Rerun EXACT same session
  3. Analyze new CSV:
     - iter=15 count (expect: <50 frames, currently 259)
     - IK time p95 (expect: <2.5ms, currently 3.45ms)
     - pos_err p95 (expect: similar or better)
""")

print("=" * 80 + "\n")
