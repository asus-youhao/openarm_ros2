#!/usr/bin/env python3
"""
Test script for O6 bimanual hands.

This script sends test trajectories to both left and right O6 hands.

Usage:
    # Open both hands
    python3 test_o6_bimanual.py open

    # Close both hands
    python3 test_o6_bimanual.py close

    # Grasp with both hands (medium grip)
    python3 test_o6_bimanual.py grasp

    # Point with both index fingers
    python3 test_o6_bimanual.py point

    # Mirror test - right closes, left opens
    python3 test_o6_bimanual.py mirror
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class BimanualHandTester(Node):
    def __init__(self):
        super().__init__('bimanual_hand_tester')
        
        # Action clients for both hands
        self.right_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/right_hand_controller/follow_joint_trajectory'
        )
        
        self.left_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/left_hand_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Waiting for action servers...')
        self.right_client.wait_for_server()
        self.left_client.wait_for_server()
        self.get_logger().info('Action servers ready!')

    def send_trajectory(self, right_positions, left_positions, duration_sec=2.0):
        """Send trajectory to both hands."""
        
        # Joint names
        right_joints = [
            'R_thumb_cmc_yaw', 'R_thumb_cmc_pitch', 
            'R_index_mcp_pitch', 'R_middle_mcp_pitch', 
            'R_ring_mcp_pitch', 'R_pinky_mcp_pitch'
        ]
        
        left_joints = [
            'L_thumb_cmc_yaw', 'L_thumb_cmc_pitch', 
            'L_index_mcp_pitch', 'L_middle_mcp_pitch', 
            'L_ring_mcp_pitch', 'L_pinky_mcp_pitch'
        ]
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        
        # Right hand goal
        right_goal = FollowJointTrajectory.Goal()
        right_goal.trajectory.joint_names = right_joints
        point.positions = right_positions
        right_goal.trajectory.points = [point]
        
        # Left hand goal
        left_goal = FollowJointTrajectory.Goal()
        left_goal.trajectory.joint_names = left_joints
        point.positions = left_positions
        left_goal.trajectory.points = [point]
        
        # Send goals
        self.get_logger().info(f'Sending trajectories...')
        self.get_logger().info(f'  Right: {right_positions}')
        self.get_logger().info(f'  Left:  {left_positions}')
        
        right_future = self.right_client.send_goal_async(right_goal)
        left_future = self.left_client.send_goal_async(left_goal)
        
        rclpy.spin_until_future_complete(self, right_future)
        rclpy.spin_until_future_complete(self, left_future)
        
        self.get_logger().info('Trajectories sent!')


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    
    command = sys.argv[1].lower()
    
    rclpy.init()
    tester = BimanualHandTester()
    
    try:
        if command == 'open':
            # Open both hands (all zeros)
            tester.send_trajectory(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
        
        elif command == 'close':
            # Close both hands (max grip)
            tester.send_trajectory(
                [0.5, 1.2, 1.5, 1.5, 1.5, 1.5],
                [0.5, 1.2, 1.5, 1.5, 1.5, 1.5]
            )
        
        elif command == 'grasp':
            # Medium grip for grasping
            tester.send_trajectory(
                [0.3, 0.7, 0.8, 0.8, 0.8, 0.8],
                [0.3, 0.7, 0.8, 0.8, 0.8, 0.8]
            )
        
        elif command == 'point':
            # Point with index fingers
            tester.send_trajectory(
                [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],  # Right: index open, others closed
                [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]   # Left: index open, others closed
            )
        
        elif command == 'mirror':
            # Right closes, left opens
            tester.send_trajectory(
                [0.5, 1.2, 1.5, 1.5, 1.5, 1.5],  # Right: closed
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # Left: open
            )
        
        else:
            tester.get_logger().error(f'Unknown command: {command}')
            print(__doc__)
    
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
