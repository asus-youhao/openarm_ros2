#!/usr/bin/env python3
"""量測 FK vs IK 的實際 CPU 耗時（雙臂 PlacoBimanualSession）。

在你的 runtime conda env 執行：
  cd placo_ik
  python3 tools/bench_fk_vs_ik.py
  # 或指定次數：python3 tools/bench_fk_vs_ik.py --n 5000
"""
import argparse
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "ik_node"))
sys.path.insert(0, os.path.join(_HERE, "..", "ik_solver"))
sys.path.insert(0, os.path.join(_HERE, "..", "config"))

from placo_ik_solver import _find_urdf                    # noqa: E402
from placo_ik_session_bimanual import PlacoBimanualSession, ARMS  # noqa: E402


def _pct(a, p):
    return float(np.percentile(a, p))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=3000, help="量測迭代次數")
    ap.add_argument("--rate", type=float, default=250.0)
    args = ap.parse_args()

    sess = PlacoBimanualSession(urdf=_find_urdf(), rate_hz=args.rate)

    # home 附近的 seed（j4=90°，其餘 0）
    seed = [0.0, 0.0, 0.0, 1.5708, 0.0, 0.0, 0.0]
    seeds = {a: list(seed) for a in ARMS}
    # 各臂 FK 目標當 IK target
    fk0 = {a: sess.fk(a, seed) for a in ARMS}
    targets = {}
    for a in ARMS:
        p = fk0[a]
        R = np.eye(3)  # 只要合法即可，成本與姿態無關
        targets[a] = {"xyz": np.array(p[:3]), "R": R}

    # ── warmup ──
    for _ in range(50):
        for a in ARMS:
            sess.fk(a, seed)
        sess.solve_step(targets, seeds)

    # ── FK：雙臂各一次（模擬 _publish_ee_pose_all）──
    fk_ms = []
    for _ in range(args.n):
        t0 = time.perf_counter()
        for a in ARMS:
            sess.fk(a, seed)
        fk_ms.append((time.perf_counter() - t0) * 1000.0)

    # ── IK：一次雙臂單 QP solve ──
    ik_ms = []
    for _ in range(args.n):
        t0 = time.perf_counter()
        sess.solve_step(targets, seeds)
        ik_ms.append((time.perf_counter() - t0) * 1000.0)

    fk_ms = np.array(fk_ms)
    ik_ms = np.array(ik_ms)
    budget = 1000.0 / args.rate

    print(f"\n  n={args.n}  rate={args.rate:.0f}Hz  budget={budget:.2f}ms/tick\n")
    print(f"  {'':22s}{'median':>10s}{'p95':>10s}{'max':>10s}")
    print(f"  {'FK ×2 (雙臂/ tick)':22s}"
          f"{np.median(fk_ms):>9.3f}m{_pct(fk_ms,95):>9.3f}m{fk_ms.max():>9.3f}m")
    print(f"  {'IK solve_step':22s}"
          f"{np.median(ik_ms):>9.3f}m{_pct(ik_ms,95):>9.3f}m{ik_ms.max():>9.3f}m")
    ratio = np.median(ik_ms) / max(np.median(fk_ms), 1e-9)
    print(f"\n  IK / FK 中位數比值 ≈ {ratio:.1f}×")
    print(f"  FK 佔 tick budget：{np.median(fk_ms)/budget*100:.2f}%")
    print(f"  IK 佔 tick budget：{np.median(ik_ms)/budget*100:.2f}%\n")


if __name__ == "__main__":
    main()
