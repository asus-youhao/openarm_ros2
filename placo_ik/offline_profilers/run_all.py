#!/usr/bin/env python3
"""
run_all.py
==========
One-shot orchestrator: pick CSV(s) once, run all four analyzers, drop every
output into a single folder named after the CSV(s).

Folder naming (created next to the picked CSVs):
  • 1 CSV   →  <HHMM>_<arm>                e.g. 162338_right
  • 2 CSVs same time, both arms →  <HHMM>_lr           e.g. 162338_lr
  • Multiple times              →  <HHMM-lo>-<HHMM-hi>_<arms>

Contents of the folder:
  quantify.md            quantify.png              ← quantify_teleop
  spatial_failure.md     spatial_failure.png       ← spatial_failure_map
  spatial_grid.md        spatial_grid.png  spatial_grid_3d.png   ← spatial_grid_3d
  spatial_compare.md     spatial_compare.png       ← spatial_compare (auto mode)
  index.md               ← short summary + links to the above

Usage:
  python3 run_all.py                       # interactive picker
  python3 run_all.py session.csv           # direct
  python3 run_all.py left.csv right.csv    # direct, two arms
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from quantify_teleop import _pick_int_range, RESULTS_ROOT                  # noqa: E402
import quantify_teleop                                                      # noqa: E402
import spatial_failure_map                                                  # noqa: E402
import spatial_grid_3d                                                      # noqa: E402
import spatial_compare                                                      # noqa: E402


# ── Run-all specific picker (mode first, then file(s)) ───────────────────────
def _pick_folder(root: str) -> str:
    subdirs = sorted(
        d for d in os.listdir(root)
        if os.path.isdir(os.path.join(root, d)) and not d.startswith(".")
    )
    csvs_in_root = sorted(f for f in os.listdir(root) if f.endswith(".csv"))
    options: List[Tuple[str, str]] = []
    if csvs_in_root:
        options.append(("(this folder)", root))
    for d in subdirs:
        options.append((d, os.path.join(root, d)))
    if not options:
        print(f"  no CSV files or subfolders under {root}")
        sys.exit(1)
    print(f"\n  Results root: {root}")
    print("  Pick a folder:")
    for i, (lbl, _) in enumerate(options, 1):
        print(f"    {i:>2}. {lbl}")
    idx = _pick_int_range("  folder # > ", 1, len(options))
    return options[idx - 1][1]


def _pick_one_csv(folder: str, prompt: str, prefer_arm: Optional[str] = None,
                  already_picked: Optional[str] = None) -> str:
    """List CSVs (numbering stable across picks). Marks matching arm with '←'
    and the already-picked CSV with '✓' so the user knows not to repeat it.
    """
    csvs = sorted(f for f in os.listdir(folder) if f.endswith(".csv"))
    if not csvs:
        print(f"  no CSVs in {folder}")
        sys.exit(1)
    picked_base = os.path.basename(already_picked) if already_picked else None
    print(f"\n  {prompt}:")
    for i, name in enumerate(csvs, 1):
        size_kb = os.path.getsize(os.path.join(folder, name)) // 1024
        flags = []
        if prefer_arm and f"_{prefer_arm}_" in name:
            flags.append("←")
        if picked_base and name == picked_base:
            flags.append("✓ already picked")
        flag = ("  " + " ".join(flags)) if flags else ""
        print(f"    {i:>2}. {name}  ({size_kb} KB){flag}")
    while True:
        idx = _pick_int_range("  file # > ", 1, len(csvs))
        chosen = csvs[idx - 1]
        if picked_base and chosen == picked_base:
            print("  ! same file as already picked — choose a different one")
            continue
        return os.path.join(folder, chosen)


def _runall_pick(root: str) -> Tuple[List[str], str]:
    """Folder → mode (single / dual) → file(s). Returns (csv_paths, out_dir)."""
    folder = _pick_folder(root)

    print("\n  Analysis mode:")
    print("    1. Single arm   (1 CSV)")
    print("    2. Dual arm L+R (pick LEFT then RIGHT — runs lr-diff)")
    mode = _pick_int_range("  mode # > ", 1, 2)

    if mode == 1:
        path = _pick_one_csv(folder, "Pick the CSV")
        return [path], folder

    left  = _pick_one_csv(folder, "Pick LEFT  CSV", prefer_arm="left")
    right = _pick_one_csv(folder, "Pick RIGHT CSV", prefer_arm="right",
                          already_picked=left)
    return [left, right], folder


# ── Folder name derivation ────────────────────────────────────────────────────
def _parse_csv_name(path: str) -> Optional[Tuple[str, str]]:
    """placo_online_YYYYMMDD_HHMMSS_<arm>_<mode>.csv → (HHMM, arm). Else None."""
    base = os.path.basename(path).replace(".csv", "")
    parts = base.split("_")
    if len(parts) < 5:
        return None
    stamp, arm = parts[3], parts[4]
    if len(stamp) < 4 or not stamp.isdigit():
        return None
    return stamp[:4], arm


def _derive_folder_name(csv_paths: List[str]) -> str:
    infos = [info for info in (_parse_csv_name(p) for p in csv_paths) if info]
    if not infos:
        # Fallback: use first CSV's basename (sans extension)
        return os.path.basename(csv_paths[0]).replace(".csv", "")[:32]

    hhmms = sorted({h for h, _ in infos})
    arms  = sorted({a for _, a in infos})

    if set(arms) == {"left", "right"}:
        arm_str = "lr"
    elif len(arms) == 1:
        arm_str = arms[0]
    else:
        arm_str = "+".join(arms)

    if len(hhmms) == 1:
        time_str = hhmms[0]
    else:
        time_str = f"{hhmms[0]}-{hhmms[-1]}"

    return f"{time_str}_{arm_str}"


# ── Tool runners (call each script's main with synthetic argv) ────────────────
def _run_tool(name: str, fn, argv: List[str]) -> bool:
    """Invoke a sibling tool's main() with the given argv. Returns True on success."""
    print(f"\n  ─── [{name}] ───")
    try:
        rc = fn(argv)
        return rc == 0
    except SystemExit as e:
        return int(getattr(e, "code", 0) or 0) == 0
    except Exception as e:
        print(f"  [{name}] failed: {type(e).__name__}: {e}")
        return False


def _index_md(folder: str, csv_paths: List[str], successes: dict) -> str:
    out = ["# Analysis bundle\n"]
    out.append("**Inputs:**")
    for p in csv_paths:
        out.append(f"- `{os.path.basename(p)}`")
    out.append("")
    out.append("**Reports** (open the `.md` files; `.png` next to each):\n")
    items = [
        ("quantify.md",        "Aggregate health (4-category metric table w/ 🟢🟡🔴 grading)"),
        ("spatial_failure.md", "Top-10 worst voxels + 3-projection heatmap (primary: pos_err_p95)"),
        ("spatial_grid.md",    "4-metric × 3-projection grid + 3-D scatter"),
        ("spatial_compare.md", "Orientation slice OR L−R diff (auto-selected by input count)"),
    ]
    for fname, desc in items:
        ok = successes.get(fname, False)
        mark = "✅" if ok else "⚠️ (failed or skipped)"
        out.append(f"- [{fname}]({fname}) — {desc}  {mark}")
    out.append("")
    out.append(f"Bundle folder: `{folder}`")
    return "\n".join(out)


# ── Main ──────────────────────────────────────────────────────────────────────
def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("csv", nargs="*",
                   help="CSV file(s). Omit → interactive picker.")
    p.add_argument("--root", default=RESULTS_ROOT)
    p.add_argument("--voxel-size", type=float, default=0.05,
                   help="Voxel edge length for spatial tools (default 0.05 = 5cm)")
    p.add_argument("--metric", default="pos_err_p95",
                   help="Primary metric for spatial tools (default pos_err_p95)")
    p.add_argument("--topn", type=int, default=10)
    p.add_argument("--folder-name", default="",
                   help="Override the auto-derived bundle folder name")
    args = p.parse_args(argv)

    # 1. Resolve inputs.
    if args.csv:
        csv_paths = [os.path.abspath(p) for p in args.csv]
        base_dir = os.path.dirname(csv_paths[0])
    else:
        csv_paths, base_dir = _runall_pick(args.root)
        csv_paths = [os.path.abspath(p) for p in csv_paths]

    # 2. Create bundle folder.
    name = args.folder_name or _derive_folder_name(csv_paths)
    out_dir = os.path.join(base_dir, name)
    os.makedirs(out_dir, exist_ok=True)
    print(f"\n  Bundle folder: {out_dir}\n  Inputs:")
    for p in csv_paths:
        print(f"    - {os.path.basename(p)}")

    successes: dict = {}

    # 3. quantify_teleop  — all CSVs side-by-side.
    qt_md = os.path.join(out_dir, "quantify.md")
    successes["quantify.md"] = _run_tool(
        "quantify_teleop", quantify_teleop.main,
        list(csv_paths) + ["--out", qt_md],
    )

    # 4. spatial_failure_map — all CSVs merged.
    sf_md = os.path.join(out_dir, "spatial_failure.md")
    successes["spatial_failure.md"] = _run_tool(
        "spatial_failure_map", spatial_failure_map.main,
        list(csv_paths) + [
            "--out", sf_md,
            "--voxel-size", str(args.voxel_size),
            "--metric", args.metric,
            "--topn", str(args.topn),
        ],
    )

    # 5. spatial_grid_3d — all CSVs merged.
    sg_md = os.path.join(out_dir, "spatial_grid.md")
    successes["spatial_grid.md"] = _run_tool(
        "spatial_grid_3d", spatial_grid_3d.main,
        list(csv_paths) + [
            "--out", sg_md,
            "--voxel-size", str(args.voxel_size),
            "--metric", args.metric,
            "--topn", str(args.topn),
        ],
    )

    # 6. spatial_compare — route by CSV count.
    sc_md = os.path.join(out_dir, "spatial_compare.md")
    sc_argv: List[str]
    arms = {info[1] for info in (_parse_csv_name(p) for p in csv_paths) if info}
    if len(csv_paths) == 2 and arms == {"left", "right"}:
        sc_argv = list(csv_paths) + [
            "--mode", "lr-diff",
            "--mirror-left-y",
            "--out", sc_md,
            "--voxel-size", str(args.voxel_size),
            "--metric", args.metric,
            "--topn", str(args.topn),
        ]
    else:
        # Use the first CSV for ori-slice; warn if more were given.
        if len(csv_paths) > 1:
            print(f"  [spatial_compare] >1 CSV but not (left,right) — "
                  f"running ori-slice on {os.path.basename(csv_paths[0])}")
        sc_argv = [csv_paths[0]] + [
            "--mode", "ori-slice",
            "--out", sc_md,
            "--voxel-size", str(args.voxel_size),
            "--metric", args.metric,
            "--topn", str(min(args.topn, 5)),    # ori-slice readable up to ~5
        ]
    successes["spatial_compare.md"] = _run_tool(
        "spatial_compare", spatial_compare.main, sc_argv)

    # 7. Index file.
    idx_path = os.path.join(out_dir, "index.md")
    with open(idx_path, "w") as fh:
        fh.write(_index_md(out_dir, csv_paths, successes))
    print(f"\n  [bundle] wrote {idx_path}")

    # 8. Listing.
    print(f"\n  ─── Done. Files in {out_dir}:")
    for f in sorted(os.listdir(out_dir)):
        size = os.path.getsize(os.path.join(out_dir, f))
        print(f"    {f:<28s}  {size:>10,} B")
    return 0 if all(successes.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
