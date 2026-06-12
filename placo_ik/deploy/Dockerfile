# placo_ik runtime image — ROS 2 Humble + placo + the profiler's Python deps,
# all baked in so re-entering the container is instant (no per-run pip/apt).
#
# Build:  docker build -t placo-ik-humble:latest -f Dockerfile .   (run from deploy/)
# Use:    ../deploy/shell.sh   (handles --network host + ROS_DOMAIN_ID for you)
FROM ros:humble-ros-base

# ROS message/tf packages the node imports (rclpy ships with the base image,
# but the message packages + tf2 python bindings do not).
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        ros-humble-rclpy \
        ros-humble-geometry-msgs \
        ros-humble-sensor-msgs \
        ros-humble-std-msgs \
        ros-humble-trajectory-msgs \
        ros-humble-tf2-ros-py \
    && rm -rf /var/lib/apt/lists/*

# Python deps (placo + its pinned cmeel subtree, numpy 2.2, scipy, matplotlib).
COPY requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --upgrade pip \
    && python3 -m pip install -r /tmp/requirements.txt \
    && rm -rf /root/.cache/pip

# Default URDF = the one shipped in the repo (overridable with -e OPENARM_URDF).
# Points at the in-container mount path used by shell.sh.
ENV OPENARM_URDF=/work/openarm_ros2/placo_ik/ik_solver/v10_o6.urdf

# The base image's /ros_entrypoint.sh already sources /opt/ros/humble/setup.bash.
CMD ["bash"]
