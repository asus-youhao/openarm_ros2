#!/bin/bash
# OpenArm V10 Bimanual with O6 Hands Test Script
# This script provides commands to launch and test the bimanual system

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== OpenArm V10 Bimanual + O6 Hands Test Script ===${NC}"
echo ""

# Setup environment
source /home/asus/ros2_ws_yh/install/setup.bash
export ROS_PACKAGE_PATH=/home/asus/openArm_leapHand_urdf/src:$ROS_PACKAGE_PATH

# Function to show usage
show_usage() {
    echo "Usage: $0 [fake|real|list|test_arms|test_hands]"
    echo ""
    echo "Commands:"
    echo "  fake          - Launch with fake hardware"
    echo "  real          - Launch with real hardware (CAN interfaces)"
    echo "  list          - List all controllers"
    echo "  test_arms     - Send test trajectory to both arms"
    echo "  test_hands    - Send test trajectory to both O6 hands"
    echo ""
}

# Function to launch with fake hardware
launch_fake() {
    echo -e "${YELLOW}Launching with FAKE hardware...${NC}"
    ros2 launch openarm_bringup openarm_o6_bimanual.launch.py \
        use_fake_hardware:=true \
        launch_rviz:=true
}

# Function to launch with real hardware
launch_real() {
    echo -e "${YELLOW}Launching with REAL hardware...${NC}"
    echo -e "${RED}Make sure CAN interfaces (can0, can1, can2, can3) are configured!${NC}"
    ros2 launch openarm_bringup openarm_o6_bimanual.launch.py \
        right_can_interface:=can0 \
        left_can_interface:=can1 \
        right_o6_can_interface:=can2 \
        left_o6_can_interface:=can3 \
        launch_rviz:=true
}

# Function to list controllers
list_controllers() {
    echo -e "${YELLOW}Listing all controllers...${NC}"
    ros2 control list_controllers
}

# Function to test arm movement
test_arms() {
    echo -e "${YELLOW}Testing ARM movement...${NC}"
    echo "Sending trajectory to left arm..."
    
    # Create test trajectory for left arm (7 joints)
    ros2 action send_goal /left_joint_trajectory_controller/follow_joint_trajectory \
        control_msgs/action/FollowJointTrajectory \
        "{
            trajectory: {
                joint_names: [
                    'openarm_left_joint1',
                    'openarm_left_joint2', 
                    'openarm_left_joint3',
                    'openarm_left_joint4',
                    'openarm_left_joint5',
                    'openarm_left_joint6',
                    'openarm_left_joint7'
                ],
                points: [
                    {
                        positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        time_from_start: {sec: 0, nanosec: 0}
                    },
                    {
                        positions: [0.5, 0.3, 0.2, 0.4, 0.1, 0.3, 0.2],
                        time_from_start: {sec: 3, nanosec: 0}
                    },
                    {
                        positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        time_from_start: {sec: 6, nanosec: 0}
                    }
                ]
            }
        }" &
    
    echo "Sending trajectory to right arm..."
    
    # Create test trajectory for right arm (7 joints)
    ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
        control_msgs/action/FollowJointTrajectory \
        "{
            trajectory: {
                joint_names: [
                    'openarm_right_joint1',
                    'openarm_right_joint2',
                    'openarm_right_joint3',
                    'openarm_right_joint4',
                    'openarm_right_joint5',
                    'openarm_right_joint6',
                    'openarm_right_joint7'
                ],
                points: [
                    {
                        positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        time_from_start: {sec: 0, nanosec: 0}
                    },
                    {
                        positions: [-0.5, -0.3, -0.2, -0.4, -0.1, -0.3, -0.2],
                        time_from_start: {sec: 3, nanosec: 0}
                    },
                    {
                        positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        time_from_start: {sec: 6, nanosec: 0}
                    }
                ]
            }
        }"
}

# Function to test O6 hand movement
test_hands() {
    echo -e "${YELLOW}Testing O6 HAND movement...${NC}"
    echo "Sending trajectory to left O6 hand..."
    
    # Create test trajectory for left O6 hand (11 joints: 6 active + 5 passive)
    ros2 action send_goal /left_o6_hand_controller/follow_joint_trajectory \
        control_msgs/action/FollowJointTrajectory \
        "{
            trajectory: {
                joint_names: [
                    'L_thumb_cmc_yaw',
                    'L_thumb_cmc_pitch',
                    'L_index_mcp_pitch',
                    'L_middle_mcp_pitch',
                    'L_ring_mcp_pitch',
                    'L_pinky_mcp_pitch',
                    'L_thumb_dip',
                    'L_index_dip',
                    'L_middle_dip',
                    'L_ring_dip',
                    'L_pinky_dip'
                ],
                points: [
                    {
                        positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        time_from_start: {sec: 0, nanosec: 0}
                    },
                    {
                        positions: [0.3, 0.5, 0.8, 0.8, 0.8, 0.8, 0.4, 0.7, 0.7, 0.7, 0.7],
                        time_from_start: {sec: 2, nanosec: 0}
                    },
                    {
                        positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        time_from_start: {sec: 4, nanosec: 0}
                    }
                ]
            }
        }" &
    
    echo "Sending trajectory to right O6 hand..."
    
    # Create test trajectory for right O6 hand (11 joints)
    ros2 action send_goal /right_o6_hand_controller/follow_joint_trajectory \
        control_msgs/action/FollowJointTrajectory \
        "{
            trajectory: {
                joint_names: [
                    'R_thumb_cmc_yaw',
                    'R_thumb_cmc_pitch',
                    'R_index_mcp_pitch',
                    'R_middle_mcp_pitch',
                    'R_ring_mcp_pitch',
                    'R_pinky_mcp_pitch',
                    'R_thumb_dip',
                    'R_index_dip',
                    'R_middle_dip',
                    'R_ring_dip',
                    'R_pinky_dip'
                ],
                points: [
                    {
                        positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        time_from_start: {sec: 0, nanosec: 0}
                    },
                    {
                        positions: [0.3, 0.5, 0.8, 0.8, 0.8, 0.8, 0.4, 0.7, 0.7, 0.7, 0.7],
                        time_from_start: {sec: 2, nanosec: 0}
                    },
                    {
                        positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        time_from_start: {sec: 4, nanosec: 0}
                    }
                ]
            }
        }"
}

# Parse command line arguments
case "$1" in
    fake)
        launch_fake
        ;;
    real)
        launch_real
        ;;
    list)
        list_controllers
        ;;
    test_arms)
        test_arms
        ;;
    test_hands)
        test_hands
        ;;
    *)
        show_usage
        exit 1
        ;;
esac
