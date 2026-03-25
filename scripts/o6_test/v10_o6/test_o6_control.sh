#!/bin/bash

# Test script for OpenArm V10 + O6 bimanual system
# Usage:
#   ./test_o6_control.sh fake              # Launch with fake hardware
#   ./test_o6_control.sh real              # Launch with real hardware
#   ./test_o6_control.sh list              # List all controllers
#   ./test_o6_control.sh test_arms         # Test arm movements
#   ./test_o6_control.sh test_hands        # Test hand movements (11 joints)
#   ./test_o6_control.sh test_hands_active # Test hand with position controller (6 joints)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Workspace setup
WORKSPACE_DIR="/home/asus/ros2_ws_yh"
URDF_PATH="/home/asus/openArm_leapHand_urdf/src"

setup_environment() {
    echo -e "${YELLOW}Setting up environment...${NC}"
    cd ${WORKSPACE_DIR}
    source install/setup.bash
    export ROS_PACKAGE_PATH=${URDF_PATH}:$ROS_PACKAGE_PATH
    echo -e "${GREEN}Environment ready${NC}"
}

launch_fake() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Launching with FAKE hardware${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo -e "${YELLOW}Using joint_trajectory_controller (default)${NC}"
    setup_environment
    ros2 launch openarm_bringup openarm_o6_bimanual.launch.py \
        use_fake_hardware:=true
}

launch_fake_position() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Launching with FAKE hardware${NC}"
    echo -e "${GREEN}Using FORWARD POSITION CONTROLLER${NC}"
    echo -e "${GREEN}========================================${NC}"
    setup_environment
    ros2 launch openarm_bringup openarm_o6_bimanual.launch.py \
        use_fake_hardware:=true \
        robot_controller:=forward_position_controller
}

launch_real() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Launching with REAL hardware${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo -e "${YELLOW}Make sure CAN interfaces are configured:${NC}"
    echo -e "${BLUE}  sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up${NC}"
    echo -e "${BLUE}  sudo ip link set can1 type can bitrate 1000000 && sudo ip link set can1 up${NC}"
    echo -e "${BLUE}  sudo ip link set can2 type can bitrate 1000000 && sudo ip link set can2 up${NC}"
    echo -e "${BLUE}  sudo ip link set can3 type can bitrate 1000000 && sudo ip link set can3 up${NC}"
    echo ""
    echo -e "${YELLOW}Using joint_trajectory_controller (default)${NC}"
    read -p "Press Enter to continue or Ctrl+C to cancel..."
    
    setup_environment
    ros2 launch openarm_bringup openarm_o6_bimanual.launch.py \
        right_can_interface:=can0 \
        left_can_interface:=can1 \
        right_o6_can_interface:=can2 \
        left_o6_can_interface:=can3
}

launch_real_position() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Launching with REAL hardware${NC}"
    echo -e "${GREEN}Using FORWARD POSITION CONTROLLER${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo -e "${YELLOW}Make sure CAN interfaces are configured:${NC}"
    echo -e "${BLUE}  sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up${NC}"
    echo -e "${BLUE}  sudo ip link set can1 type can bitrate 1000000 && sudo ip link set can1 up${NC}"
    echo -e "${BLUE}  sudo ip link set can2 type can bitrate 1000000 && sudo ip link set can2 up${NC}"
    echo -e "${BLUE}  sudo ip link set can3 type can bitrate 1000000 && sudo ip link set can3 up${NC}"
    echo ""
    read -p "Press Enter to continue or Ctrl+C to cancel..."
    
    setup_environment
    ros2 launch openarm_bringup openarm_o6_bimanual.launch.py \
        right_can_interface:=can0 \
        left_can_interface:=can1 \
        right_o6_can_interface:=can2 \
        left_o6_can_interface:=can3 \
        robot_controller:=forward_position_controller
}

list_controllers() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Listing all controllers${NC}"
    echo -e "${GREEN}========================================${NC}"
    setup_environment
    ros2 control list_controllers
}

test_arms() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Testing arm movements${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo -e "${YELLOW}Sending trajectory to both arms (7 joints each)${NC}"
    
    setup_environment
    
    # Test left arm
    echo -e "${BLUE}>>> Moving left arm...${NC}"
    ros2 action send_goal /left_joint_trajectory_controller/follow_joint_trajectory \
        control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names:
    - openarm_left_joint1
    - openarm_left_joint2
    - openarm_left_joint3
    - openarm_left_joint4
    - openarm_left_joint5
    - openarm_left_joint6
    - openarm_left_joint7
  points:
    - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      time_from_start: {sec: 0, nanosec: 0}
    - positions: [0.3, 0.2, 0.2, 0.3, 0.0, 0.2, 0.0]
      time_from_start: {sec: 3, nanosec: 0}
    - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      time_from_start: {sec: 6, nanosec: 0}
" --feedback
    
    echo ""
    sleep 1
    
    # Test right arm
    echo -e "${BLUE}>>> Moving right arm...${NC}"
    ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
        control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names:
    - openarm_right_joint1
    - openarm_right_joint2
    - openarm_right_joint3
    - openarm_right_joint4
    - openarm_right_joint5
    - openarm_right_joint6
    - openarm_right_joint7
  points:
    - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      time_from_start: {sec: 0, nanosec: 0}
    - positions: [-0.3, 0.2, -0.2, 0.3, 0.0, 0.2, 0.0]
      time_from_start: {sec: 3, nanosec: 0}
    - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      time_from_start: {sec: 6, nanosec: 0}
" --feedback
}

test_hands() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Testing O6 hand movements (6 ACTIVE joints)${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo -e "${YELLOW}Sending trajectory to both O6 hands${NC}"
    echo -e "${YELLOW}Only controlling 6 active joints${NC}"
    echo -e "${YELLOW}Passive joints will follow via mechanical coupling${NC}"
    
    setup_environment
    
    # Test left O6 hand
    echo -e "${BLUE}>>> Moving left O6 hand...${NC}"
    ros2 action send_goal /left_hand_controller/follow_joint_trajectory \
        control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names:
    - L_thumb_cmc_yaw
    - L_thumb_cmc_pitch
    - L_index_mcp_pitch
    - L_middle_mcp_pitch
    - L_ring_mcp_pitch
    - L_pinky_mcp_pitch
  points:
    - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      time_from_start: {sec: 0, nanosec: 0}
    - positions: [0.3, 0.5, 0.8, 0.8, 0.8, 0.8]
      time_from_start: {sec: 3, nanosec: 0}
    - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      time_from_start: {sec: 6, nanosec: 0}
" --feedback
    
    echo ""
    sleep 1
    
    # Test right O6 hand
    echo -e "${BLUE}>>> Moving right O6 hand...${NC}"
    ros2 action send_goal /right_hand_controller/follow_joint_trajectory \
        control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names:
    - R_thumb_cmc_yaw
    - R_thumb_cmc_pitch
    - R_index_mcp_pitch
    - R_middle_mcp_pitch
    - R_ring_mcp_pitch
    - R_pinky_mcp_pitch
  points:
    - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      time_from_start: {sec: 0, nanosec: 0}
    - positions: [0.3, 0.5, 0.8, 0.8, 0.8, 0.8]
      time_from_start: {sec: 3, nanosec: 0}
    - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      time_from_start: {sec: 6, nanosec: 0}
" --feedback
}

test_hands_active() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Testing O6 hand movements (6 ACTIVE joints)${NC}"
    echo -e "${GREEN}Using FORWARD POSITION CONTROLLER${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo -e "${YELLOW}NOTE: This requires launching with 'fake_position' or 'real_position'${NC}"
    echo -e "${YELLOW}Passive joints will follow automatically via hardware mimic${NC}"
    
    setup_environment
    
    # Test left O6 hand - open and close sequence
    echo -e "${BLUE}>>> Testing left O6 hand (open -> close -> open)...${NC}"
    
    echo "  Position 1: Open (all 0.0)"
    ros2 topic pub --once /left_hand_forward_position_controller/commands std_msgs/msg/Float64MultiArray \
        "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
    sleep 2
    
    echo "  Position 2: Close/Grasp"
    ros2 topic pub --once /left_hand_forward_position_controller/commands std_msgs/msg/Float64MultiArray \
        "{data: [0.3, 0.5, 0.8, 0.8, 0.8, 0.8]}"
    sleep 2
    
    echo "  Position 3: Open again"
    ros2 topic pub --once /left_hand_forward_position_controller/commands std_msgs/msg/Float64MultiArray \
        "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
    
    echo ""
    sleep 1
    
    # Test right O6 hand
    echo -e "${BLUE}>>> Testing right O6 hand (open -> close -> open)...${NC}"
    
    echo "  Position 1: Open (all 0.0)"
    ros2 topic pub --once /right_hand_forward_position_controller/commands std_msgs/msg/Float64MultiArray \
        "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
    sleep 2
    
    echo "  Position 2: Close/Grasp"
    ros2 topic pub --once /right_hand_forward_position_controller/commands std_msgs/msg/Float64MultiArray \
        "{data: [0.3, 0.5, 0.8, 0.8, 0.8, 0.8]}"
    sleep 2
    
    echo "  Position 3: Open again"
    ros2 topic pub --once /right_hand_forward_position_controller/commands std_msgs/msg/Float64MultiArray \
        "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
    
    echo -e "${GREEN}Done!${NC}"
}

# Main script
interactive_menu() {
  echo "Select an option by number:"
  echo "  1) Launch with fake hardware"
  echo "  2) Launch with fake hardware (forward position controller)"
  echo "  3) Launch with real hardware"
  echo "  4) Launch with real hardware (forward position controller)"
  echo "  5) List controllers"
  echo "  6) Test arms (send trajectories)"
  echo "  7) Test O6 hands (trajectory, 6 active joints)"
  echo "  8) Test O6 hands (forward position controller sequence)"
  read -p "Enter choice [1-8]: " choice
  case "$choice" in
    1) launch_fake ;;
    2) launch_fake_position ;;
    3) launch_real ;;
    4) launch_real_position ;;
    5) list_controllers ;;
    6) test_arms ;;
    7) test_hands ;;
    8) test_hands_active ;;
    *) echo "Invalid choice"; exit 1 ;;
  esac
}

if [ -z "$1" ]; then
  interactive_menu
  exit 0
fi

case "$1" in
  1|fake)
    launch_fake
    ;;
  2|fake_position)
    launch_fake_position
    ;;
  3|real)
    launch_real
    ;;
  4|real_position)
    launch_real_position
    ;;
  5|list)
    list_controllers
    ;;
  6|test_arms)
    test_arms
    ;;
  7|test_hands)
    test_hands
    ;;
  8|test_hands_active)
    test_hands_active
    ;;
  *)
    echo -e "${RED}Usage: $0 {fake|fake_position|real|real_position|list|test_arms|test_hands|test_hands_active}${NC}"
    exit 1
    ;;
esac
