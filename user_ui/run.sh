#!/usr/bin/env bash
# Entry point for the OpenArm Launcher GUI.
# Refuses to start if ros2 isn't on PATH — source your workspace first.

set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v ros2 >/dev/null 2>&1; then
  cat >&2 <<'MSG'
ERROR: 'ros2' not found in PATH.
Source your ROS 2 setup before launching, e.g.:
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws_yh/install/setup.bash
MSG
  exit 1
fi

cd "$HERE"
exec python3 -m openarm_launcher "$@"
