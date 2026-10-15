# Test image for run_in_docker.sh: ROS 2 <distro> base, the dependencies of
# openarm_description and openarm_bringup installed the same way the README
# does (rosdep from their package.xml), plus the tools the full test needs.
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
COPY deps /deps
RUN apt-get update \
 && rosdep update \
 && rosdep install --from-paths /deps --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && apt-get install -y --no-install-recommends git libxml2-utils liburdfdom-tools python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/* \
 && git config --global --add safe.directory '*'
