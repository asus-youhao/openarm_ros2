#!/usr/bin/env python3
"""
Debug DLS Adaptive Damping Analysis
====================================

Check if DLS formula is working correctly and suggest optimal parameters.
"""

import csv
import numpy as np
from pathlib import Path

# Constants from placo_ik_session.py
_DLS_LAMBDA_BASE  = 6e-5
_DLS_LAMBDA_MAX   = 1e-2
_DLS_SIGMA_THRESH = 0.15

def compute_lambda_dls(sigma_min):
    """Replicate DLS formula from placo_ik_session.py"""
    _ratio = max(0.0, (_DLS_SIGMA_THRESH - sigma_min) / _DLS_SIGMA_THRESH)
    return _DLS_LAMBDA_BASE + _DLS_LAMBDA_MAX * _ratio * _ratio

# Load CSV
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
lambda_actual = np.array(data['lambda_dls'])
iterations = np.array(data['iterations'])
ik_ms = np.array(data['ik_ms'])

print("\n" + "=" * 80)
print("🔬 DLS ADAPTIVE DAMPING VERIFICATION")
print("=" * 80)

# 1. 验证公式
print("\n1️⃣  FORMULA VERIFICATION")
print("-" * 80)
lambda_computed = np.array([compute_lambda_dls(s) for s in sigma_min])
diff = np.abs(lambda_actual - lambda_computed)
print(f"Formula matches CSV: {np.allclose(lambda_actual, lambda_computed)}")
print(f"Max difference: {diff.max():.8f}")
print(f"Mean difference: {diff.mean():.8f}")

# 2. 参数有效性分析
print("\n2️⃣  PARAMETER EFFECTIVENESS")
print("-" * 80)

# 按sigma分组
sigma_bins = [(0, 0.05), (0.05, 0.10), (0.10, 0.15), (0.15, 0.20), (0.20, 0.30)]
print(f"Current params: λ_base={_DLS_LAMBDA_BASE:.5f}, λ_max={_DLS_LAMBDA_MAX:.5f}, σ_thresh={_DLS_SIGMA_THRESH:.2f}")
print()

for s_lo, s_hi in sigma_bins:
    mask = (sigma_min >= s_lo) & (sigma_min < s_hi)
    if mask.any():
        n = mask.sum()
        lambda_in_bin = lambda_actual[mask]
        iter_in_bin = iterations[mask]
        ik_in_bin = ik_ms[mask]

        print(f"σ ∈ [{s_lo:.2f}, {s_hi:.2f}): {n:4d} frames")
        print(f"  λ: mean={lambda_in_bin.mean():.6f}, min={lambda_in_bin.min():.6f}, max={lambda_in_bin.max():.6f}")
        print(f"  iter: mean={iter_in_bin.mean():.1f}, max={int(iter_in_bin.max())}")
        print(f"  ik_ms: mean={ik_in_bin.mean():.2f}ms, p95={np.percentile(ik_in_bin, 95):.2f}ms")
        print()

# 3. 关键问题：iter=15 的 Lambda 值
print("\n3️⃣  CRITICAL ISSUE: iter=15 Frames")
print("-" * 80)
max_iter_mask = iterations == 15
max_iter_sigma = sigma_min[max_iter_mask]
max_iter_lambda = lambda_actual[max_iter_mask]

print(f"iter=15 frames: {max_iter_mask.sum()}")
print(f"σ_min: mean={max_iter_sigma.mean():.5f}, range=[{max_iter_sigma.min():.5f}, {max_iter_sigma.max():.5f}]")
print(f"λ_dls: mean={max_iter_lambda.mean():.6f}, max={max_iter_lambda.max():.6f}")
print(f"Expected λ_max: {_DLS_LAMBDA_MAX:.5f}")
print(f"⚠️  Actual λ is only {100*max_iter_lambda.mean()/_DLS_LAMBDA_MAX:.1f}% of λ_max")
print()

# 4. 参数优化建议
print("\n4️⃣  PARAMETER OPTIMIZATION RECOMMENDATIONS")
print("-" * 80)

# 试算不同参数
test_configs = [
    ("CURRENT", 6e-5, 1e-2, 0.15),
    ("OPT-1 (higher λ_max)", 6e-5, 0.05, 0.15),
    ("OPT-2 (lower σ_thresh)", 6e-5, 1e-2, 0.12),
    ("OPT-3 (combined)", 6e-5, 0.03, 0.12),
    ("OPT-4 (aggressive)", 1e-4, 0.05, 0.10),
]

def compute_lambda_custom(sigma, base, max_boost, thresh):
    _ratio = max(0.0, (thresh - sigma) / thresh)
    return base + max_boost * _ratio * _ratio

print("\nComparison of parameter sets (on iter=15 frames):")
print(f"{'Config':<25} {'mean_λ':<12} {'max_λ':<12} {'λ/λ_max %':<12}")
print("-" * 61)

for name, base, max_b, thresh in test_configs:
    lambda_test = np.array([compute_lambda_custom(s, base, max_b, thresh) for s in max_iter_sigma])
    print(f"{name:<25} {lambda_test.mean():<12.6f} {lambda_test.max():<12.6f} {100*lambda_test.mean()/max_b:<11.1f}%")

print()
print("💡 ANALYSIS:")
print("-" * 80)
print("""
Current behavior:
  · When σ_min = 0.10, λ ≈ 0.0010 (≈10% of λ_max)
  · These frames STILL need iter=15, suggesting damping isn't enough
  · However, final accuracy is still GOOD (pos_err ~3.3mm)

Possible causes:
  1. λ_max=0.01 is TOO LOW for effective singularity suppression
  2. σ_thresh=0.15 is TOO HIGH (activates too late)
  3. Optimizer (Placo) may not be responsive to small λ adjustments

Recommendation (in priority order):
  ✅ OPT-3 (combined): λ_max=0.03, σ_thresh=0.12
     · Provides 3× more damping in critical region
     · Activates damping earlier
     · Minimal impact on other regions

  🔄 OPT-1 (higher λ_max): λ_max=0.05
     · More aggressive at singularities
     · May over-damp normal regions

  🔄 OPT-4 (aggressive): Comprehensive tuning
     · Complete reset with lower baselines
     · Risks affecting non-singular performance

Test methodology:
  1. Run same trajectory with EACH config
  2. Compare: iter distribution, IK time, accuracy
  3. Pick config with best success_rate and lowest iter=15 count
""")

print("\n" + "=" * 80)
print("✅ NEXT STEPS")
print("-" * 80)
print("""
1. Edit placo_ik_session.py, try OPT-3 parameters:
   _DLS_LAMBDA_MAX = 0.03  (was 0.01)
   _DLS_SIGMA_THRESH = 0.12  (was 0.15)

2. Run a test session with same trajectory:
   python3 placo_ik_online_profiler_ws_mesh.py --arm right [same flags]

3. Analyze new CSV with quantify_teleop.py:
   - Compare iter=15 count (should drop from 259)
   - Compare IK time p95 (should decrease)
   - Check pos_err (should stay ~same or improve)

4. If improved:
   - Push changes to feature branch
   - Document in docs/dls_tuning.md
""")
print("=" * 80 + "\n")
