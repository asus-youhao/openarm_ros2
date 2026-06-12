#!/usr/bin/env bash
# test_docker_humble.sh — prove install_deps.sh works on a clean ROS 2 Humble box.
#
# Spins up the official ros:humble-ros-base image (no host deps leaked in),
# mounts this repo read-only, runs install_deps.sh, then verify_deps.sh.
# If this passes, the install script is good to ship to another x86 PC.
#
#   ./test_docker_humble.sh           # full run: install + verify
#   KEEP=1 ./test_docker_humble.sh    # drop into a shell afterwards for poking
set -euo pipefail

HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
PLACO_IK_ROOT="$(dirname "$HERE")"                          # .../placo_ik
OPENARM_ROOT="$(dirname "$PLACO_IK_ROOT")"                  # .../openarm_ros2
IMAGE="ros:humble-ros-base"
MNT="/work/openarm_ros2"                                    # mount point in container
DEPLOY="$MNT/placo_ik/deploy"

echo "==> Using image: $IMAGE"
echo "==> Mounting:    $OPENARM_ROOT  ->  $MNT"

# Run as host UID so any pip --user / cache files aren't left root-owned, and
# point HOME at a writable in-container dir for the pip cache.
read -r -d '' INNER <<INNER_EOF || true
export DEBIAN_FRONTEND=noninteractive
echo "----- inside ros:humble container -----"
# ROS setup.bash trips 'set -u', so source it first, then go strict.
source /opt/ros/humble/setup.bash
set -euo pipefail
bash "$DEPLOY/install_deps.sh"
bash "$DEPLOY/verify_deps.sh"
INNER_EOF

CMD=(bash -lc "$INNER")
if [ "${KEEP:-0}" = "1" ]; then
  CMD=(bash -lc "$INNER"$'\n''echo; echo "[KEEP=1] dropping into shell"; exec bash')
  TTY=(-it)
else
  TTY=()
fi

exec docker run --rm "${TTY[@]}" \
  -v "$OPENARM_ROOT":"$MNT" \
  -w "$MNT/placo_ik" \
  -e HOME=/tmp/ctr-home \
  "$IMAGE" "${CMD[@]}"
