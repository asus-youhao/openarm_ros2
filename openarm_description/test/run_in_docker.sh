#!/usr/bin/env bash
# Run the full test inside a ROS 2 container, e.g. to test u22 (Humble) on a u24 machine.
#
#   openarm_description/test/run_in_docker.sh            # humble
#   openarm_description/test/run_in_docker.sh jazzy
#
# Builds openarm-<distro>-test:local from test/docker/Dockerfile (dependencies via
# rosdep from the two package.xml files, as in the README), then runs
# run_full_test.sh with this repo mounted read-only; build/install/log stay in the
# container. The container uses Docker's default bridge network: ament_xmllint
# fetches the package.xml schema, and DDS stays inside the container's own
# network namespace.

set -euo pipefail

DISTRO=${1:-humble}
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG=$(cd "$HERE/.." && pwd)
REPO=$(cd "$PKG/.." && pwd)
IMAGE=openarm-$DISTRO-test:local

ctx=$(mktemp -d)
trap 'rm -rf "$ctx"' EXIT
mkdir -p "$ctx/deps/openarm_description" "$ctx/deps/openarm_bringup"
cp "$PKG/package.xml" "$ctx/deps/openarm_description/"
cp "$REPO/openarm_bringup/package.xml" "$ctx/deps/openarm_bringup/"
cp "$HERE/docker/Dockerfile" "$ctx/"

echo "building $IMAGE ..."
docker build -q --build-arg ROS_DISTRO="$DISTRO" -t "$IMAGE" "$ctx" >/dev/null
exec docker run --rm -v "$REPO:/ws/src/openarm_ros2:ro" "$IMAGE" \
  /ws/src/openarm_ros2/openarm_description/test/run_full_test.sh
