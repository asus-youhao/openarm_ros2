#!/bin/bash
# O6 Right Hand Control Commands
# Usage: ./o6_right_hand_control.sh [launch|open|grasp|close]

HAND_PREFIX="R_"
ACTION_SERVER="/o6_hand_controller/follow_joint_trajectory"

case "$1" in
    launch)
        echo "Launching right hand hardware (can0)..."
        ros2 launch openarm_bringup o6_hand_hardware_test.launch.py \
            can_interface:=can0 \
            hand_type:=right \
            hand_prefix:=R_ \
            use_action_server:=true
        ;;
    open)
        echo "Opening right hand..."
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
        echo "Grasping with right hand (medium grip)..."
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
        echo "Closing right hand (full grip)..."
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
        echo "  launch - Launch right hand hardware on can0"
        echo "  open   - Open right hand (all joints to 0 rad)"
        echo "  grasp  - Medium grip for grasping objects"
        echo "  close  - Full grip (maximum closure)"
        exit 1
        ;;
esac
