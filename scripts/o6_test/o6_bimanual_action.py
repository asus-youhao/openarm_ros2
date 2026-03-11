#!/usr/bin/env python3
"""
Test script for O6 bimanual hands.

This script sends test trajectories to both left and right O6 hands.
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
            '/right_o6_hand_controller/follow_joint_trajectory'
        )
        
        self.left_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/left_o6_hand_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Waiting for action servers...')
        self.right_client.wait_for_server()
        self.left_client.wait_for_server()
        self.get_logger().info('Action servers ready!')

    def send_trajectory(self, right_positions, left_positions, duration_sec=0.5):
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
    menu = [
        ('open', 'Open both hands'),
        ('close', 'Close both hands'),
        ('grasp', 'Medium grip'),
        ('point', 'Point with index fingers'),
        ('mirror', 'Right closes, left opens'),
        ('cycle', 'Open and grasp cycle (2s interval)'),
    ]
    print('O6 Bimanual Hands Test Menu:')
    for idx, (cmd, desc) in enumerate(menu, 1):
        print(f'  {idx}. {cmd:<8} - {desc}')
    try:
        sel = input('Enter option number (1-6): ').strip()
        if not sel.isdigit() or not (1 <= int(sel) <= len(menu)):
            print('Invalid option, please rerun.')
            sys.exit(1)
        command = menu[int(sel)-1][0]
    except Exception:
        print('Input error, please rerun.')
        sys.exit(1)

    rclpy.init()
    tester = BimanualHandTester()
    try:
        if command == 'open':
            tester.send_trajectory(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
        elif command == 'close':
            tester.send_trajectory(
                [0.5, 1.2, 1.5, 1.5, 1.5, 1.5],
                [0.5, 1.2, 1.5, 1.5, 1.5, 1.5]
            )
        elif command == 'grasp':
            tester.send_trajectory(
                [0.3, 0.7, 0.8, 0.8, 0.8, 0.8],
                [0.3, 0.7, 0.8, 0.8, 0.8, 0.8]
            )
        elif command == 'point':
            tester.send_trajectory(
                [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]
            )
        elif command == 'mirror':
            tester.send_trajectory(
                [0.5, 1.2, 1.5, 1.5, 1.5, 1.5],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
        elif command == 'cycle':
            import time
            print('Starting open-grasp cycle (Press Ctrl+C to stop)...')
            try:
                cycle_count = 0
                while True:
                    cycle_count += 1
                    print(f'\n--- Cycle {cycle_count} ---')
                    # Open
                    print('Opening hands...')
                    tester.send_trajectory(
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        duration_sec=0.1
                    )
                    time.sleep(2.0)
                    
                    # Grasp
                    print('Grasping...')
                    tester.send_trajectory(
                        [0.3, 0.7, 0.8, 0.8, 0.8, 0.8],
                        [0.3, 0.7, 0.8, 0.8, 0.8, 0.8],
                        duration_sec=0.1
                    )
                    time.sleep(2.0)
            except KeyboardInterrupt:
                print(f'\nStopped after {cycle_count} cycles.')
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
