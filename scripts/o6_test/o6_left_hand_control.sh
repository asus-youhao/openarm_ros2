#!/bin/bash
# O6 Left Hand Control Commands
# Usage: ./o6_left_hand_control.sh [launch|open|grasp|close]

HAND_PREFIX="L_"
ACTION_SERVER="/o6_hand_controller/follow_joint_trajectory"

case "$1" in
    launch)
        echo "Launching left hand hardware (can1)..."
        ros2 launch openarm_bringup o6_hand_hardware_test.launch.py \
            can_interface:=can1 \
            hand_type:=left \
            hand_prefix:=L_ \
            use_action_server:=true
        ;;
    open)
        echo "Opening left hand..."
        ros2 action send_goal ${ACTION_SERVER} control_msgs/action/FollowJointTrajectory "{
          trajectory: {
            joint_names: [${HAND_PREFIX}thumb_cmc_yaw, ${HAND_PREFIX}thumb_cmc_pitch, ${HAND_PREFIX}index_mcp_pitch, ${HAND_PREFIX}middle_mcp_pitch, ${HAND_PREFIX}ring_mcp_pitch, ${HAND_PREFIX}pinky_mcp_pitch],
            points: [
              {
                positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                time_from_start: {sec: 2}
              }
            ]
          }
        }" --feedback
        ;;
    grasp)
        echo "Grasping with left hand (medium grip)..."
        ros2 action send_goal ${ACTION_SERVER} control_msgs/action/FollowJointTrajectory "{
          trajectory: {
            joint_names: [${HAND_PREFIX}thumb_cmc_yaw, ${HAND_PREFIX}thumb_cmc_pitch, ${HAND_PREFIX}index_mcp_pitch, ${HAND_PREFIX}middle_mcp_pitch, ${HAND_PREFIX}ring_mcp_pitch, ${HAND_PREFIX}pinky_mcp_pitch],
            points: [
              {
                positions: [0.3, 0.7, 0.8, 0.8, 0.8, 0.8],
                time_from_start: {sec: 2}
              }
            ]
          }
        }" --feedback
        ;;
    close)
        echo "Closing left hand (full grip)..."
        ros2 action send_goal ${ACTION_SERVER} control_msgs/action/FollowJointTrajectory "{
          trajectory: {
            joint_names: [${HAND_PREFIX}thumb_cmc_yaw, ${HAND_PREFIX}thumb_cmc_pitch, ${HAND_PREFIX}index_mcp_pitch, ${HAND_PREFIX}middle_mcp_pitch, ${HAND_PREFIX}ring_mcp_pitch, ${HAND_PREFIX}pinky_mcp_pitch],
            points: [
              {
                positions: [0.5, 1.2, 1.5, 1.5, 1.5, 1.5],
                time_from_start: {sec: 2}
              }
            ]
          }
        }" --feedback
        ;;
    *)
        echo "Usage: $0 {launch|open|grasp|close}"
        echo ""
        echo "Commands:"
        echo "  launch - Launch left hand hardware on can1"
        echo "  open   - Open left hand (all joints to 0 rad)"
        echo "  grasp  - Medium grip for grasping objects"
        echo "  close  - Full grip (maximum closure)"
        exit 1
        ;;
esac
