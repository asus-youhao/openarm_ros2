#!/usr/bin/env bash
# verify_deps.sh — smoke-test that all deps import and the profiler script loads.
# Exits non-zero on the first failure. Safe to run anywhere ROS is sourced.
set -euo pipefail

HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
PLACO_IK_ROOT="$(dirname "$HERE")"                 # .../placo_ik
SCRIPT="$PLACO_IK_ROOT/ik_node/placo_ik_main.py"
ROS_DISTRO="${ROS_DISTRO:-humble}"

# Source ROS if not already on the path. (set +u: ROS setup.bash trips 'set -u'.)
if ! python3 -c "import rclpy" >/dev/null 2>&1; then
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi

echo "==> [verify] python: $(command -v python3)  ($(python3 -V 2>&1))"

echo "==> [verify] importing third-party deps"
python3 - <<'PY'
import importlib, sys
mods = ["numpy", "scipy", "matplotlib", "yaml", "placo",
        "rclpy", "geometry_msgs.msg", "sensor_msgs.msg",
        "std_msgs.msg", "trajectory_msgs.msg", "tf2_ros"]
bad = []
for m in mods:
    try:
        importlib.import_module(m)
        print(f"   ok   {m}")
    except Exception as e:
        bad.append((m, repr(e)))
        print(f"   FAIL {m}: {e!r}")
if bad:
    print(f"\n{len(bad)} import(s) failed", file=sys.stderr)
    sys.exit(1)
PY

echo "==> [verify] loading the profiler script (--help, exercises full import chain)"
# --help parses args and exits 0 *after* every top-level import runs, but before
# rclpy.init() / hardware / URDF — so it validates imports without needing a robot.
python3 "$SCRIPT" --help >/dev/null
echo "   ok   $SCRIPT --help"

echo "==> [verify] ALL CHECKS PASSED"
