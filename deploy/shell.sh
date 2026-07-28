#!/usr/bin/env bash
# shell.sh — enter the placo_ik ROS 2 Humble container, wired to talk to the
# HOST ROS 2 graph (--network host + --ipc host, matching ROS_DOMAIN_ID / RMW).
#
# Deps are baked into the image, so this is instant after the first build.
#
#   ./shell.sh                                  # interactive shell at ik_node/
#   ./shell.sh ros2 topic list                  # one-off command
#   ./shell.sh python3 ./placo_ik_main.py --help
#
# Env overrides:
#   ROS_DOMAIN_ID=5 ./shell.sh        # default 10 (matches your ~/.bashrc)
#   RMW=rmw_cyclonedds_cpp ./shell.sh # default rmw_fastrtps_cpp (Humble default)
#   REBUILD=1 ./shell.sh              # force-rebuild the image first
#   ROOT=1 ./shell.sh                 # run as root (breaks host ROS comms, see below)
#
# Why --user $(id -u):$(id -g):  FastDDS (Humble's default RMW) discovers peers
# through per-UID shared-memory segments in /dev/shm. A root container creates
# those segments mode 0644, so the host user (uid 1000) cannot write to them and
# discovery silently fails — `ros2 topic echo` just times out. Running the
# container as the host UID makes the SHM segments writable on both sides.
set -euo pipefail

IMAGE="placo-ik-humble:latest"
HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"        # .../placo_ik/deploy
PLACO_IK_ROOT="$(dirname "$HERE")"                            # .../placo_ik
OPENARM_ROOT="$(dirname "$PLACO_IK_ROOT")"                    # .../openarm_ros2
MNT="/work/openarm_ros2"

# Build the image on first use (or when REBUILD=1).
if [ "${REBUILD:-0}" = "1" ] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "[shell.sh] building $IMAGE (one-time; bakes in all deps)..."
  docker build -t "$IMAGE" -f "$HERE/Dockerfile" "$HERE"
fi

ROS_DOMAIN_ID_VAL="${ROS_DOMAIN_ID:-10}"
RMW_VAL="${RMW:-rmw_fastrtps_cpp}"

# Run as the host UID so FastDDS SHM discovery works across the boundary (see
# header). ROOT=1 opts out (e.g. to apt-install something), at the cost of host
# ROS comms. HOME=/tmp gives the uid a writable home for ~/.ros logs.
USER_ARGS=(--user "$(id -u):$(id -g)" -e HOME=/tmp)
[ "${ROOT:-0}" = "1" ] && USER_ARGS=()

# Interactive TTY only when attached to one (so pipes / one-off cmds still work).
TTY_ARGS=(-i)
[ -t 1 ] && TTY_ARGS=(-it)

ARGS=("$@")
[ ${#ARGS[@]} -eq 0 ] && ARGS=(bash)

echo "[shell.sh] ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VAL  RMW=$RMW_VAL  user=$( [ "${ROOT:-0}" = "1" ] && echo root || id -un )  (host network)"
echo "[shell.sh] OPENARM_URDF defaults to repo URDF; run your script from ik_node/"

exec docker run --rm "${TTY_ARGS[@]}" \
  --network host \
  --ipc host \
  "${USER_ARGS[@]}" \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VAL" \
  -e RMW_IMPLEMENTATION="$RMW_VAL" \
  -v "$OPENARM_ROOT":"$MNT" \
  -w "$MNT/placo_ik/ik_node" \
  "$IMAGE" "${ARGS[@]}"
