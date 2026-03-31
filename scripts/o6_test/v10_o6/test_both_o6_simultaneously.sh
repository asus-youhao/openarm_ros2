#!/bin/bash

# Test both O6 hands simultaneously
echo "Testing both O6 hands at the same time..."

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/asus/ros2_ws_yh/install/setup.bash

echo "Sending trajectory to both hands simultaneously..."

# Send to left hand in background
ros2 action send_goal /left_hand_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names:
    - L_thumb_cmc_pitch
    - L_thumb_cmc_yaw
    - L_index_mcp_pitch
    - L_middle_mcp_pitch
    - L_ring_mcp_pitch
    - L_pinky_mcp_pitch
  points:
    - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      time_from_start: {sec: 0, nanosec: 0}
    - positions: [0.3, 0.5, 0.8, 0.8, 0.8, 0.8]
      time_from_start: {sec: 0, nanosec: 500000000}
    - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      time_from_start: {sec: 1, nanosec: 0}
" &

# Wait a moment then send to right hand
sleep 0.5

ros2 action send_goal /right_hand_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names:
    - R_thumb_cmc_pitch
    - R_thumb_cmc_yaw
    - R_index_mcp_pitch
    - R_middle_mcp_pitch
    - R_ring_mcp_pitch
    - R_pinky_mcp_pitch
  points:
    - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      time_from_start: {sec: 0, nanosec: 0}
    - positions: [0.3, 0.5, 0.8, 0.8, 0.8, 0.8]
      time_from_start: {sec: 0, nanosec: 500000000}
    - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      time_from_start: {sec: 1, nanosec: 0}
"

echo "Both hands should be moving simultaneously now!"
