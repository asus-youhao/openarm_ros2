#!/usr/bin/env python3
"""
placo_ik_online_profiler.py
===========================
[DEPRECATED — redirect shim]

This file is superseded by placo_ik_online_profiler_ws_mesh.py which is a
strict superset:

  • All original CLI flags are still accepted unchanged.
  • When --ws-mesh is omitted, a rectangular workspace BOX clamp is used
    (values from ARM_CONFIG["workspace"] in arm_config.py) — identical to
    what this file previously did with its hard box clamp.
  • Additional flags available: --ws-mesh, --lpf-alpha, --wrist-vel-cap,
    --joint-jump-guard-deg, --ori-lpf-alpha, --boundary-margin, etc.

Migration (no changes needed to existing scripts):
  # Before:
  python3 placo_ik_online_profiler.py --arm right
  # After (identical behaviour):
  python3 placo_ik_online_profiler_ws_mesh.py --arm right

This shim forwards all invocations to placo_ik_online_profiler_ws_mesh.main()
so existing launch files and shell scripts continue to work.
"""

import sys as _sys, os as _os

_HERE = _os.path.dirname(_os.path.abspath(__file__))
_sys.path.insert(0, _HERE)

_sys.stderr.write(
    "[placo_ik_online_profiler] DEPRECATED: forwarding to "
    "placo_ik_online_profiler_ws_mesh.py\n"
    "  All args are accepted. Omit --ws-mesh to use default rectangular box clamp.\n"
)

from placo_ik_online_profiler_ws_mesh import main  # noqa: E402

if __name__ == "__main__":
    main()
