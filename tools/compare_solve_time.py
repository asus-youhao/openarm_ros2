#!/usr/bin/env python3
"""
compare_solve_time.py
=====================
比較 bimanual 單一 QP 版 vs ws_mesh 雙 node 版的 placo IK solve 時間,
回答「為何 bimanual 的 ik_ms 較長」是 #1(一次解兩臂)還是 #2(迭代耦合)主導。

bimanual CSV 欄位:ik_ms / setup_ms / loop_ms / iterations / r_sigma_min / l_sigma_min
ws_mesh  CSV 欄位:ik_ms / robot_ms / setup_ms / loop_ms / iterations / sigma_min

用法:
  python3 compare_solve_time.py \
    --bi    results/.../placo_bimanual_*_both_qp.csv \
    --right results/.../placo_online_*_right_cached.csv \
    --left  results/.../placo_online_*_left_cached.csv \
    [--out compare.png]
"""

import argparse
import csv
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def _load(path, fields):
    """讀 CSV 指定欄位,回傳 {field: np.array(float)}。空值跳過。"""
    cols = {f: [] for f in fields}
    with open(path) as fh:
        rd = csv.DictReader(fh)
        for row in rd:
            try:
                vals = {f: float(row[f]) for f in fields if row.get(f, "") != ""}
            except (ValueError, KeyError):
                continue
            if len(vals) != len(fields):
                continue
            for f in fields:
                cols[f].append(vals[f])
    return {f: np.array(v) for f, v in cols.items()}


def _stat(name, arr):
    if len(arr) == 0:
        return f"  {name:<28} (no data)"
    return (f"  {name:<28} n={len(arr):5d}  mean={np.mean(arr):7.3f}  "
            f"median={np.median(arr):7.3f}  p95={np.percentile(arr,95):7.3f}  "
            f"max={np.max(arr):7.3f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bi",    required=True, help="bimanual both_qp CSV")
    ap.add_argument("--right", required=True, help="ws_mesh right_cached CSV")
    ap.add_argument("--left",  required=True, help="ws_mesh left_cached CSV")
    ap.add_argument("--out",   default=None,  help="輸出 PNG（default: 與 --bi 同目錄）")
    args = ap.parse_args()

    bi = _load(args.bi, ["ik_ms", "setup_ms", "loop_ms", "iterations",
                         "r_sigma_min", "l_sigma_min"])
    R  = _load(args.right, ["ik_ms", "setup_ms", "iterations", "sigma_min"])
    L  = _load(args.left,  ["ik_ms", "setup_ms", "iterations", "sigma_min"])

    # ── 文字統計 ──────────────────────────────────────────────────────────────
    print("=" * 78)
    print("  Solve-time comparison: bimanual single-QP  vs  ws_mesh dual-node")
    print("=" * 78)
    print("\n【ik_ms — 單次 solve_step 時間】")
    print(_stat("bimanual (雙臂一次解)",  bi["ik_ms"]))
    print(_stat("ws_mesh right (單臂)",    R["ik_ms"]))
    print(_stat("ws_mesh left  (單臂)",    L["ik_ms"]))
    rl_sum = (np.mean(R["ik_ms"]) + np.mean(L["ik_ms"])) if len(R["ik_ms"]) and len(L["ik_ms"]) else 0
    print(f"\n  → ws_mesh 兩臂 ik_ms 平均相加 ≈ {rl_sum:.3f} ms"
          f"   vs  bimanual {np.mean(bi['ik_ms']):.3f} ms")
    print(f"     若 bimanual ≈ (right+left) → #1（一次解兩臂）主導")
    print(f"     若 bimanual ≫ (right+left) → #2（迭代耦合 / setup ×2）額外加成")

    print("\n【iterations — 每步迭代數】")
    print(_stat("bimanual",        bi["iterations"]))
    print(_stat("ws_mesh right",   R["iterations"]))
    print(_stat("ws_mesh left",    L["iterations"]))
    if len(R["iterations"]) and len(L["iterations"]):
        rl_max_mean = np.mean([max(a, b) for a, b in
                               zip(R["iterations"][:min(len(R['iterations']),len(L['iterations']))],
                                   L["iterations"][:min(len(R['iterations']),len(L['iterations']))])])
        print(f"\n  → 單臂 max(right,left) 迭代平均 ≈ {rl_max_mean:.2f}"
              f"   vs  bimanual {np.mean(bi['iterations']):.2f}")
        print(f"     bimanual 因『兩臂都收斂才 exit』+『σ_worst 觸發 boost』,"
              f"理論上 ≈ max(兩臂) 或更高")

    print("\n【sigma_min — 越小越接近奇異點(觸發 ×4 boost 的門檻 0.15)】")
    print(_stat("bimanual right (r_sigma_min)", bi["r_sigma_min"]))
    print(_stat("bimanual left  (l_sigma_min)", bi["l_sigma_min"]))
    print(_stat("ws_mesh right",                 R["sigma_min"]))
    print(_stat("ws_mesh left",                  L["sigma_min"]))
    thr = 0.15
    for nm, arr in [("bi r", bi["r_sigma_min"]), ("bi l", bi["l_sigma_min"]),
                    ("ws R", R["sigma_min"]), ("ws L", L["sigma_min"])]:
        if len(arr):
            pct = 100.0 * np.mean(arr < thr)
            print(f"    {nm}: {pct:5.1f}% 的步在奇異區 (σ<{thr})")
    print("=" * 78)

    # ── 畫圖（6 panel）────────────────────────────────────────────────────────
    fig, ax = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle("Placo solve-time: bimanual single-QP  vs  ws_mesh dual-node",
                 fontsize=13)

    # (0,0) ik_ms 時間序列
    a = ax[0, 0]
    a.plot(bi["ik_ms"], lw=0.6, color="crimson", label="bimanual (2 arms)")
    a.plot(R["ik_ms"], lw=0.5, color="steelblue", alpha=0.7, label="ws_mesh right")
    a.plot(L["ik_ms"], lw=0.5, color="seagreen", alpha=0.7, label="ws_mesh left")
    a.set_title("ik_ms time series"); a.set_ylabel("ms")
    a.legend(fontsize=8); a.grid(alpha=0.3)

    # (0,1) ik_ms 分佈（histogram）
    a = ax[0, 1]
    bins = np.linspace(0, max(np.percentile(bi["ik_ms"], 99),
                              np.percentile(R["ik_ms"], 99)), 60)
    a.hist(bi["ik_ms"], bins=bins, alpha=0.5, color="crimson", label="bimanual")
    a.hist(R["ik_ms"], bins=bins, alpha=0.5, color="steelblue", label="ws right")
    a.hist(L["ik_ms"], bins=bins, alpha=0.5, color="seagreen", label="ws left")
    a.axvline(np.mean(bi["ik_ms"]), color="crimson", ls="--", lw=1)
    a.axvline(np.mean(R["ik_ms"]) + np.mean(L["ik_ms"]), color="black", ls=":",
              lw=1.2, label="ws(R+L) mean")
    a.set_title("ik_ms distribution"); a.set_xlabel("ms")
    a.legend(fontsize=8); a.grid(alpha=0.3)

    # (1,0) iterations 時間序列
    a = ax[1, 0]
    a.plot(bi["iterations"], lw=0.6, color="crimson", label="bimanual")
    a.plot(R["iterations"], lw=0.5, color="steelblue", alpha=0.7, label="ws right")
    a.plot(L["iterations"], lw=0.5, color="seagreen", alpha=0.7, label="ws left")
    a.set_title("iterations per step"); a.legend(fontsize=8); a.grid(alpha=0.3)

    # (1,1) iterations 分佈
    a = ax[1, 1]
    maxit = int(max(bi["iterations"].max() if len(bi["iterations"]) else 1,
                    R["iterations"].max() if len(R["iterations"]) else 1)) + 1
    ibins = np.arange(0, maxit + 1) - 0.5
    a.hist(bi["iterations"], bins=ibins, alpha=0.5, color="crimson", label="bimanual")
    a.hist(R["iterations"], bins=ibins, alpha=0.5, color="steelblue", label="ws right")
    a.hist(L["iterations"], bins=ibins, alpha=0.5, color="seagreen", label="ws left")
    a.set_title("iterations distribution"); a.set_xlabel("iters")
    a.legend(fontsize=8); a.grid(alpha=0.3)

    # (2,0) sigma_min 時間序列
    a = ax[2, 0]
    a.plot(bi["r_sigma_min"], lw=0.5, color="crimson", label="bi right")
    a.plot(bi["l_sigma_min"], lw=0.5, color="orange", label="bi left")
    a.plot(R["sigma_min"], lw=0.4, color="steelblue", alpha=0.6, label="ws right")
    a.plot(L["sigma_min"], lw=0.4, color="seagreen", alpha=0.6, label="ws left")
    a.axhline(0.15, color="red", ls="--", lw=1, label="σ_thresh=0.15 (boost)")
    a.set_title("sigma_min (lower = nearer singularity → ×4 iters)")
    a.legend(fontsize=8); a.grid(alpha=0.3)

    # (2,1) setup_ms 比較（驗證 #3 setup ×2）
    a = ax[2, 1]
    sb = [("bimanual\nsetup", bi["setup_ms"]),
          ("ws right\nsetup", R["setup_ms"]),
          ("ws left\nsetup", L["setup_ms"])]
    labels = [x[0] for x in sb]
    means  = [np.mean(x[1]) if len(x[1]) else 0 for x in sb]
    p95s   = [np.percentile(x[1], 95) if len(x[1]) else 0 for x in sb]
    xp = np.arange(len(labels))
    a.bar(xp - 0.18, means, 0.36, color="slateblue", label="mean")
    a.bar(xp + 0.18, p95s, 0.36, color="lightsteelblue", label="p95")
    a.set_xticks(xp); a.set_xticklabels(labels, fontsize=8)
    a.set_title("setup_ms (Jacobian+SVD+task assembly)")
    a.set_ylabel("ms"); a.legend(fontsize=8); a.grid(alpha=0.3, axis="y")

    plt.tight_layout()
    out = args.out or os.path.join(os.path.dirname(args.bi), "compare_solve_time.png")
    plt.savefig(out, dpi=120)
    print(f"\n  [plot] → {out}")


if __name__ == "__main__":
    main()
