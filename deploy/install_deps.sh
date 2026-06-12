#!/usr/bin/env bash
# install_deps.sh — install everything placo_ik_online_profiler_ws_mesh.py needs
# on a fresh x86_64 machine that already has ROS 2 Humble installed.
#
# What it installs:
#   * ROS 2 Humble message/tf packages used by the node (apt)
#   * Python deps: placo, numpy<2, scipy, matplotlib, PyYAML (pip, see requirements.txt)
#
# What it does NOT do:
#   * Install ROS 2 Humble itself  (must already exist at /opt/ros/humble)
#   * Provide the OpenArm URDF       (set OPENARM_URDF=/path/to/openarm.urdf at runtime)
#
# Usage:
#   ./install_deps.sh                 # apt (sudo) + pip into active interpreter
#   PIP="pip install --user" ./install_deps.sh
#   SKIP_APT=1 ./install_deps.sh      # pip only (e.g. apt already done)
#   SKIP_PIP=1 ./install_deps.sh      # apt only
set -euo pipefail

HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
REQ="$HERE/requirements.txt"
ROS_DISTRO_DEFAULT="humble"
ROS_DISTRO="${ROS_DISTRO:-$ROS_DISTRO_DEFAULT}"

# sudo only when not already root (containers run as root, no sudo present).
if [ "$(id -u)" -eq 0 ]; then SUDO=""; else SUDO="sudo"; fi

ROS_PKGS=(
  "ros-${ROS_DISTRO}-rclpy"
  "ros-${ROS_DISTRO}-geometry-msgs"
  "ros-${ROS_DISTRO}-sensor-msgs"
  "ros-${ROS_DISTRO}-std-msgs"
  "ros-${ROS_DISTRO}-trajectory-msgs"
  "ros-${ROS_DISTRO}-tf2-ros-py"
)

echo "==> placo_ik dependency installer  (ROS_DISTRO=${ROS_DISTRO})"

if [ "${SKIP_APT:-0}" != "1" ]; then
  echo "==> [apt] installing ROS message/tf packages + pip"
  $SUDO apt-get update
  $SUDO apt-get install -y --no-install-recommends python3-pip "${ROS_PKGS[@]}"
else
  echo "==> [apt] skipped (SKIP_APT=1)"
fi

if [ "${SKIP_PIP:-0}" != "1" ]; then
  PIP="${PIP:-python3 -m pip install}"
  echo "==> [pip] upgrading pip (old resolvers pick source-only matplotlib)"
  python3 -m pip install --upgrade pip
  echo "==> [pip] installing Python deps from requirements.txt"
  echo "         ($PIP)"
  $PIP -r "$REQ"
else
  echo "==> [pip] skipped (SKIP_PIP=1)"
fi

echo "==> Done. Verify with:  bash $HERE/verify_deps.sh"
