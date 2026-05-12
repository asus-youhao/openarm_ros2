#!/usr/bin/env python3
"""
placo_ws_clamp_benchmark.py
===========================
比較 WorkspaceMesh.clamp() 與矩形 box clamp 的運算速度差異。

【兩種方式說明】
  ─ Box clamp（矩形）：O(1), 純數學運算
    new_x = max(x_min, min(x_max, raw_x))   × 3 軸
    適合範圍不複雜、不需精確邊界的情況。

  ─ Mesh clamp（WorkspaceMesh KDTree）：O(log N)
    從掃描資料建立 KDTree，每步查詢最鄰近可達 voxel。
    能反映手臂真實工作空間形狀，但需要提前掃描並載入 .npz。

【測試項目】
  1. 微秒級單點 clamp 延遲（Box vs Mesh）
  2. 隨機 1000 個 query points 的 throughput（Box vs Mesh）
  3. Inside / Outside 比例不同時的延遲差異
  4. KDTree 大小（voxel 數量）對延遲的影響

【使用方式】
  # 使用已掃描的 .npz 測試（建議）
  python3 placo_ws_clamp_benchmark.py --npz results/reachability_right_ws.npz

  # 自動偵測 right 臂的 npz
  python3 placo_ws_clamp_benchmark.py --arm right

  # 合成假 mesh（不需要真實 npz，用來測速度比較）
  python3 placo_ws_clamp_benchmark.py --synthetic

  # 指定迭代次數
  python3 placo_ws_clamp_benchmark.py --npz results/reachability_right_ws.npz --n 5000

Output：
  終端機統計表 + results/clamp_benchmark_{timestamp}.png
"""

import argparse
import os
import sys
import time

import numpy as np

# ── path setup ────────────────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))             # ws_mesh/
_ROOT = os.path.dirname(_HERE)                                 # project root
sys.path.insert(0, _ROOT)                                      # paths.py
sys.path.insert(0, _HERE)                                      # placo_ws_analyze sibling

from paths import ws_mesh_path as _ws_mesh_path, clamp_benchmark_png as _clamp_benchmark_png
from placo_ws_analyze import WorkspaceMesh

# ── default workspace boxes (mirrors _ARM_CONFIG) ─────────────────────────────
_BOX = {
    "right": {"x": (-0.20, 0.40), "y": (-0.45, -0.05), "z": (0.05, 0.65)},
    "left":  {"x": (-0.20, 0.40), "y": (0.05,  0.45),  "z": (0.05, 0.65)},
}

# ── box clamp (inline, mirrors profiler code) ─────────────────────────────────
def box_clamp(point: np.ndarray, ws: dict) -> np.ndarray:
    return np.array([
        max(ws["x"][0], min(ws["x"][1], point[0])),
        max(ws["y"][0], min(ws["y"][1], point[1])),
        max(ws["z"][0], min(ws["z"][1], point[2])),
    ])


# ── synthetic WorkspaceMesh builder ──────────────────────────────────────────
def _build_synthetic_mesh(arm: str, step: float = 0.05) -> WorkspaceMesh:
    """
    Build a synthetic mesh from the rectangular workspace bounds, optionally
    with a spherical exclusion zone to make it non-rectangular (more realistic).
    """
    ws = _BOX[arm]
    xs = np.arange(ws["x"][0], ws["x"][1] + step, step)
    ys = np.arange(ws["y"][0], ws["y"][1] + step, step)
    zs = np.arange(ws["z"][0], ws["z"][1] + step, step)
    gx, gy, gz = np.meshgrid(xs, ys, zs, indexing="ij")
    pts = np.column_stack([gx.ravel(), gy.ravel(), gz.ravel()])

    # Remove a sphere near origin to simulate unreachable zone
    center = np.array([0.0, float(np.mean([ws["y"][0], ws["y"][1]])),
                       float(np.mean([ws["z"][0], ws["z"][1]]))])
    r = 0.15
    mask = np.linalg.norm(pts - center, axis=1) > r
    pts = pts[mask]

    print(f"[synthetic] {len(pts)} voxels  arm={arm}  step={step}m")
    return WorkspaceMesh(pts, step=step, min_orient_rate=0.0)


# ── timing helper ─────────────────────────────────────────────────────────────
def _time_calls(fn, queries: np.ndarray, warmup: int = 200) -> np.ndarray:
    """
    Call fn(point) for each row in queries.
    Returns array of per-call durations in microseconds.
    """
    # warmup
    for i in range(min(warmup, len(queries))):
        fn(queries[i])

    times = np.empty(len(queries))
    for i, q in enumerate(queries):
        t0 = time.perf_counter()
        fn(q)
        times[i] = (time.perf_counter() - t0) * 1e6  # µs
    return times


# ── benchmark runner ──────────────────────────────────────────────────────────
def _run_benchmark(
    mesh: WorkspaceMesh,
    arm:  str,
    n:    int,
    rng:  np.random.Generator,
) -> dict:
    ws = _BOX[arm]

    # Query points: 50% inside workspace box, 50% slightly outside
    lo = np.array([ws["x"][0], ws["y"][0], ws["z"][0]])
    hi = np.array([ws["x"][1], ws["y"][1], ws["z"][1]])
    spread = (hi - lo) * 0.30   # 30% overshoot zone

    pts_inside  = rng.uniform(lo,          hi,          (n // 2, 3))
    pts_outside = rng.uniform(lo - spread, hi + spread, (n - n // 2, 3))
    all_pts     = np.vstack([pts_inside, pts_outside])
    rng.shuffle(all_pts)

    # Inside fraction (mesh)
    inside_flags = np.array([mesh.contains(p) for p in all_pts])
    inside_frac  = inside_flags.mean()

    # ── Box clamp timing ──────────────────────────────────────────────────────
    box_fn  = lambda p: box_clamp(p, ws)
    box_us  = _time_calls(box_fn, all_pts)

    # ── Mesh clamp timing ─────────────────────────────────────────────────────
    mesh_fn = lambda p: mesh.clamp(p)
    mesh_us = _time_calls(mesh_fn, all_pts)

    # ── Inside-only vs Outside-only sub-timing ────────────────────────────────
    pts_in_sub  = all_pts[inside_flags][:min(500, inside_flags.sum())]
    pts_out_sub = all_pts[~inside_flags][:min(500, (~inside_flags).sum())]

    mesh_in_us  = _time_calls(mesh_fn, pts_in_sub)  if len(pts_in_sub)  > 0 else np.array([0.])
    mesh_out_us = _time_calls(mesh_fn, pts_out_sub) if len(pts_out_sub) > 0 else np.array([0.])

    return {
        "n":          n,
        "inside_frac": float(inside_frac),
        "box": {
            "mean_us":  float(box_us.mean()),
            "median_us":float(np.median(box_us)),
            "p95_us":   float(np.percentile(box_us, 95)),
            "max_us":   float(box_us.max()),
            "all_us":   box_us,
        },
        "mesh": {
            "mean_us":   float(mesh_us.mean()),
            "median_us": float(np.median(mesh_us)),
            "p95_us":    float(np.percentile(mesh_us, 95)),
            "max_us":    float(mesh_us.max()),
            "all_us":    mesh_us,
        },
        "mesh_inside": {
            "mean_us": float(mesh_in_us.mean()),
            "n":       len(pts_in_sub),
        },
        "mesh_outside": {
            "mean_us": float(mesh_out_us.mean()),
            "n":       len(pts_out_sub),
        },
    }


# ── size-scaling sub-benchmark ───────────────────────────────────────────────
def _run_size_scaling(arm: str, n_query: int = 500, rng=None) -> list:
    """Return latency vs mesh size (voxel count)."""
    ws = _BOX[arm]
    lo = np.array([ws["x"][0], ws["y"][0], ws["z"][0]])
    hi = np.array([ws["x"][1], ws["y"][1], ws["z"][1]])
    queries = rng.uniform(lo, hi, (n_query, 3))

    results = []
    for step in [0.10, 0.07, 0.05, 0.04, 0.03, 0.02]:
        m = _build_synthetic_mesh(arm, step=step)
        fn = lambda p: m.clamp(p)
        us = _time_calls(fn, queries, warmup=50)
        results.append({
            "step":    step,
            "n_voxels": len(m._xyz),
            "mean_us": float(us.mean()),
            "p95_us":  float(np.percentile(us, 95)),
        })
        print(f"  step={step:.2f}m  n_voxels={len(m._xyz):6d}"
              f"  clamp_mean={us.mean():.2f}µs  p95={np.percentile(us,95):.2f}µs")
    return results


# ── print summary table ───────────────────────────────────────────────────────
def _print_table(res: dict, arm: str):
    b = res["box"]
    m = res["mesh"]
    ratio = m["mean_us"] / b["mean_us"] if b["mean_us"] > 0 else float("inf")

    print(f"\n{'═'*65}")
    print(f"  Clamp Benchmark   arm={arm}   n={res['n']}   "
          f"inside_frac={res['inside_frac']*100:.0f}%")
    print(f"{'═'*65}")
    print(f"  {'Method':<16}  {'mean':>7}  {'median':>7}  {'p95':>7}  {'max':>7}  [µs/call]")
    print(f"  {'─'*16}  {'─'*7}  {'─'*7}  {'─'*7}  {'─'*7}")
    print(f"  {'Box clamp':<16}  {b['mean_us']:>7.2f}  {b['median_us']:>7.2f}  "
          f"{b['p95_us']:>7.2f}  {b['max_us']:>7.2f}")
    print(f"  {'Mesh clamp':<16}  {m['mean_us']:>7.2f}  {m['median_us']:>7.2f}  "
          f"{m['p95_us']:>7.2f}  {m['max_us']:>7.2f}")
    print(f"{'─'*65}")
    print(f"  Mesh / Box ratio:  mean={ratio:.1f}×  (overhead per step)")
    print(f"\n  Mesh clamp breakdown by location:")
    print(f"    Inside  workspace : {res['mesh_inside']['mean_us']:.2f} µs/call"
          f"  (n={res['mesh_inside']['n']})")
    print(f"    Outside workspace : {res['mesh_outside']['mean_us']:.2f} µs/call"
          f"  (n={res['mesh_outside']['n']})")
    print(f"\n  At 50Hz loop (20ms budget):")
    print(f"    Box  overhead: {b['mean_us']/1000:.4f} ms  = {b['mean_us']/20000*100:.3f}% of budget")
    print(f"    Mesh overhead: {m['mean_us']/1000:.4f} ms  = {m['mean_us']/20000*100:.3f}% of budget")
    print(f"{'═'*65}")


# ── plot ──────────────────────────────────────────────────────────────────────
def _save_plot(res: dict, scaling: list, arm: str, out_path: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    b_us   = res["box"]["all_us"]
    m_us   = res["mesh"]["all_us"]
    ratio  = m_us / np.maximum(b_us, 0.01)
    t      = np.arange(len(b_us))

    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    fig.suptitle(
        f"Clamp Benchmark: Mesh vs Box  (arm={arm}, n={res['n']})\n"
        f"inside_frac={res['inside_frac']*100:.0f}%",
        fontsize=12
    )

    # ── Row 0: timing traces ──────────────────────────────────────────────────
    ax = axes[0, 0]
    ax.plot(t, b_us, color="steelblue", linewidth=0.5, alpha=0.8, label="Box")
    ax.plot(t, m_us, color="tomato",    linewidth=0.5, alpha=0.8, label="Mesh")
    ax.axhline(np.mean(b_us), color="steelblue", linestyle="--", linewidth=1,
               label=f"Box mean={np.mean(b_us):.2f}µs")
    ax.axhline(np.mean(m_us), color="tomato",    linestyle="--", linewidth=1,
               label=f"Mesh mean={np.mean(m_us):.2f}µs")
    ax.set_title("Clamp latency over time (µs)", fontsize=9)
    ax.set_xlabel("Query index"); ax.set_ylabel("µs")
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    ax.set_ylim(0, np.percentile(np.concatenate([b_us, m_us]), 99) * 1.5)

    # ── Row 0: CDF ────────────────────────────────────────────────────────────
    ax = axes[0, 1]
    for vals, label, color in [
        (b_us, "Box",  "steelblue"),
        (m_us, "Mesh", "tomato"),
    ]:
        sv = np.sort(vals)
        cdf = np.arange(1, len(sv) + 1) / len(sv)
        ax.plot(sv, cdf, color=color, linewidth=1.5, label=label)
    ax.set_title("CDF of clamp latency", fontsize=9)
    ax.set_xlabel("µs"); ax.set_ylabel("CDF")
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    ax.set_xlim(left=0)
    ax.axvline(20, color="red", linestyle=":", linewidth=0.8, label="20µs")  # 0.02ms

    # ── Row 0: Overhead ratio ─────────────────────────────────────────────────
    ax = axes[0, 2]
    ax.hist(ratio, bins=60, color="purple", alpha=0.75)
    ax.axvline(np.mean(ratio), color="red", linestyle="--", linewidth=1.2,
               label=f"mean={np.mean(ratio):.1f}×")
    ax.set_title("Mesh/Box ratio per call", fontsize=9)
    ax.set_xlabel("Ratio (×)"); ax.set_ylabel("Count")
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # ── Row 1: Bar comparison ─────────────────────────────────────────────────
    ax = axes[1, 0]
    methods = ["Box\nmean", "Box\np95", "Mesh\nmean", "Mesh\np95"]
    vals    = [
        res["box"]["mean_us"],  res["box"]["p95_us"],
        res["mesh"]["mean_us"], res["mesh"]["p95_us"],
    ]
    colors = ["steelblue", "dodgerblue", "tomato", "orangered"]
    bars = ax.bar(methods, vals, color=colors, edgecolor="white", linewidth=0.5)
    for bar, v in zip(bars, vals):
        ax.text(bar.get_x() + bar.get_width() / 2, v * 1.02,
                f"{v:.2f}", ha="center", va="bottom", fontsize=8)
    ax.set_title("Summary: mean & p95 latency (µs)", fontsize=9)
    ax.set_ylabel("µs"); ax.grid(True, alpha=0.3, axis="y")

    # ── Row 1: Inside vs Outside mesh clamp ──────────────────────────────────
    ax = axes[1, 1]
    groups = ["Inside", "Outside"]
    vals   = [res["mesh_inside"]["mean_us"], res["mesh_outside"]["mean_us"]]
    ns     = [res["mesh_inside"]["n"],       res["mesh_outside"]["n"]]
    colors = ["seagreen", "salmon"]
    bars   = ax.bar(groups, vals, color=colors, edgecolor="white")
    for bar, v, n in zip(bars, vals, ns):
        ax.text(bar.get_x() + bar.get_width() / 2, v * 1.02,
                f"{v:.2f}µs\n(n={n})", ha="center", va="bottom", fontsize=8)
    ax.set_title("Mesh clamp: Inside vs Outside workspace", fontsize=9)
    ax.set_ylabel("µs"); ax.grid(True, alpha=0.3, axis="y")

    # ── Row 1: Scaling (voxel count vs latency) ───────────────────────────────
    ax = axes[1, 2]
    if scaling:
        nvox   = [s["n_voxels"] for s in scaling]
        mu_us  = [s["mean_us"]  for s in scaling]
        p95_us = [s["p95_us"]   for s in scaling]
        ax.plot(nvox, mu_us,  "o-", color="tomato",    linewidth=1.5,
                markersize=5, label="mean")
        ax.plot(nvox, p95_us, "s-", color="orangered", linewidth=1.0,
                linestyle="--", markersize=4, label="p95")
        ax.set_xscale("log")
        for x, y, s in zip(nvox, mu_us, scaling):
            ax.annotate(f"{s['step']:.2f}m", (x, y),
                        textcoords="offset points", xytext=(4, 2), fontsize=7)
    ax.set_title("Mesh clamp latency vs KDTree size (log scale)", fontsize=9)
    ax.set_xlabel("n_voxels (log)"); ax.set_ylabel("µs")
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_path, dpi=130)
    plt.close()
    print(f"\n  [plot] Saved → {out_path}")


# ── CLI ───────────────────────────────────────────────────────────────────────
def _parse_args():
    p = argparse.ArgumentParser(
        description="Benchmark: WorkspaceMesh.clamp() vs rectangular box clamp",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--arm",       default="right", choices=["right", "left"])
    p.add_argument("--npz",       default=None,
                   help="Path to WorkspaceMesh .npz (from placo_ws_analyze.py). "
                        "If omitted, auto-detect from results/")
    p.add_argument("--synthetic", action="store_true",
                   help="Use synthetic mesh (no real npz needed)")
    p.add_argument("--n",         type=int, default=2000,
                   help="Number of query points (default: 2000)")
    p.add_argument("--step",      type=float, default=0.05,
                   help="Grid step for synthetic mesh (m, default: 0.05)")
    p.add_argument("--no-scaling", action="store_true", dest="no_scaling",
                   help="Skip KDTree-size scaling sub-benchmark")
    p.add_argument("--plot",      default="",
                   help="Output plot PNG path (auto-named if omitted)")
    p.add_argument("--seed",      type=int, default=42,
                   help="Random seed (default: 42)")
    return p.parse_args()


def main():
    args = _parse_args()
    rng  = np.random.default_rng(args.seed)

    # ── Load or build mesh ────────────────────────────────────────────────────
    if args.synthetic:
        print(f"\n  [mode] synthetic mesh  arm={args.arm}  step={args.step}m")
        mesh = _build_synthetic_mesh(args.arm, step=args.step)
    else:
        npz = args.npz
        if not npz:
            _auto = _ws_mesh_path(args.arm)
            if os.path.isfile(_auto):
                npz = _auto
                print(f"  [auto] found npz: {_auto}")
            else:
                print(f"  [WARN] no npz found at {_auto} — falling back to synthetic")
                mesh = _build_synthetic_mesh(args.arm, step=args.step)
                npz  = None
        if npz:
            print(f"\n  [mode] real WorkspaceMesh npz: {npz}")
            mesh = WorkspaceMesh.load(npz)

    # ── Main benchmark ────────────────────────────────────────────────────────
    print(f"\n  Running main benchmark  n={args.n} queries ...")
    res = _run_benchmark(mesh, args.arm, args.n, rng)
    _print_table(res, args.arm)

    # ── Size-scaling sub-benchmark ────────────────────────────────────────────
    scaling = []
    if not args.no_scaling:
        print(f"\n  Running KDTree size-scaling sub-benchmark ...")
        scaling = _run_size_scaling(args.arm, n_query=500, rng=rng)

    # ── Plot ──────────────────────────────────────────────────────────────────
    out = args.plot or _clamp_benchmark_png(args.arm)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    try:
        _save_plot(res, scaling, args.arm, out)
    except Exception as e:
        print(f"  [plot] error: {e}")


if __name__ == "__main__":
    main()
