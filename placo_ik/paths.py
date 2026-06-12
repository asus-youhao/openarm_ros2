"""Centralized output-path helpers — import to skip hand-typing paths.

Usage:
    from paths import csv_path, png_for, ws_mesh_path, today_dir

    self._csv_path = args.csv or csv_path("placo_online", args.arm, mode)
    self._png_path = args.plot or png_for(self._csv_path)
    npz            = args.ws_mesh or ws_mesh_path(args.arm)

Layout produced:
    results/
      YYYYMMDD/                                       <- today_dir()
        placo_online_YYYYMMDD_HHMMSS_right_cached.csv <- csv_path(...)
        placo_online_YYYYMMDD_HHMMSS_right_cached.png <- png_for(csv)
      reachability_{arm}_ws.npz                       <- ws_mesh_path(arm)
      legacy/                                         <- pre-refactor artifacts
"""

import datetime
import os

# Project root = directory holding this file.
# When paths.py moves with the source tree (or scripts live in subfolders),
# this stays correct because PROJECT_ROOT is anchored to paths.py itself.
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR  = os.path.join(PROJECT_ROOT, "results")


def today_dir(create: bool = True) -> str:
    """Return ``results/YYYYMMDD`` for today, creating it if needed."""
    d = os.path.join(RESULTS_DIR, datetime.datetime.now().strftime("%Y%m%d"))
    if create:
        os.makedirs(d, exist_ok=True)
    return d


def timestamp() -> str:
    """``YYYYMMDD_HHMMSS`` — used as a filename component."""
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def csv_path(prefix: str, arm: str, mode: str = "", ts: str | None = None) -> str:
    """Standard CSV path under today's dated folder.

    Example::

        csv_path("placo_online", "right", "cached")
        -> results/20260512/placo_online_20260512_133605_right_cached.csv

    ``ts`` lets the caller share a single timestamp across paired files.
    """
    ts = ts or timestamp()
    suffix = f"_{mode}" if mode else ""
    return os.path.join(today_dir(), f"{prefix}_{ts}_{arm}{suffix}.csv")


def png_for(csv_pathlike: str) -> str:
    """Return ``.csv`` path with ``.png`` extension (paired plot file)."""
    if csv_pathlike.endswith(".csv"):
        return csv_pathlike[:-4] + ".png"
    return csv_pathlike + ".png"


def ws_mesh_path(arm: str) -> str:
    """Canonical workspace mesh location for an arm.

    Profilers auto-detect this path when ``--ws-mesh`` is not given.
    """
    return os.path.join(RESULTS_DIR, f"reachability_{arm}_ws.npz")


def reachability_csv(arm: str, ts: str | None = None) -> str:
    """``results/YYYYMMDD/reachability_<ts>_<arm>.csv``."""
    ts = ts or timestamp()
    return os.path.join(today_dir(), f"reachability_{ts}_{arm}.csv")


def clamp_benchmark_png(arm: str, ts: str | None = None) -> str:
    """``results/clamp_benchmark_<ts>_<arm>.png`` (top-level of results/)."""
    ts = ts or timestamp()
    return os.path.join(RESULTS_DIR, f"clamp_benchmark_{ts}_{arm}.png")


__all__ = [
    "PROJECT_ROOT",
    "RESULTS_DIR",
    "today_dir",
    "timestamp",
    "csv_path",
    "png_for",
    "ws_mesh_path",
    "reachability_csv",
    "clamp_benchmark_png",
]
