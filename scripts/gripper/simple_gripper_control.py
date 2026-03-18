#!/usr/bin/env python3
"""
Simple Gripper Control Script

Basic trajectory control for right gripper only.
Simple open and close commands.

Usage:
    python3 simple_gripper_control.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class SimpleGripperClient(Node):
    def __init__(self):
        super().__init__('simple_gripper_client')
        
        # Gripper positions (meters)
        self.GRIPPER_OPEN = 0.044
        self.GRIPPER_CLOSE = 0.0
        
        # Initialize action client for right gripper
        self.client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/right_gripper_trajectory_controller/follow_joint_trajectory'
        )
        
        self.joint_name = 'openarm_right_finger_joint1'
        
        self.get_logger().info('Simple gripper client initialized for RIGHT hand')
    
    def send_position(self, position, duration=0.5):
        """Send gripper to target position"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [self.joint_name]
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        goal_msg.trajectory.points.append(point)
        
        self.client.wait_for_server()
        self.client.send_goal_async(goal_msg)
    
    def open_gripper(self):
        """Open gripper to maximum position (0.044m)"""
        self.send_position(self.GRIPPER_OPEN)
        self.get_logger().info('Opening gripper...')
    
    def close_gripper(self):
        """Close gripper completely (0.0m)"""
        self.send_position(self.GRIPPER_CLOSE)
        self.get_logger().info('Closing gripper...')
    
    def test_16_steps_open_close_open(self):
        """Test 16-step trajectory: Open -> Close -> Open"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [self.joint_name]
        
        # Create 16 waypoints for open-close-open sequence
        # Steps 1-6: Open (stay open)
        # Steps 7-10: Close (transition to closed)
        # Steps 11-16: Open (transition back to open)
        positions = [
            self.GRIPPER_OPEN,   # Step 1: Open
            self.GRIPPER_OPEN,   # Step 2: Open
            self.GRIPPER_OPEN,   # Step 3: Open
            self.GRIPPER_OPEN,   # Step 4: Open
            self.GRIPPER_OPEN,   # Step 5: Open
            self.GRIPPER_OPEN,   # Step 6: Open
            0.033,               # Step 7: 75% open (transition)
            0.022,               # Step 8: 50% open
            0.011,               # Step 9: 25% open
            self.GRIPPER_CLOSE,  # Step 10: Closed
            self.GRIPPER_CLOSE,  # Step 11: Closed
            0.011,               # Step 12: 25% open (transition)
            0.022,               # Step 13: 50% open
            0.033,               # Step 14: 75% open
            self.GRIPPER_OPEN,   # Step 15: Open
            self.GRIPPER_OPEN,   # Step 16: Open
        ]
        
        # Time for each step (0.3s per step = 4.8s total)
        base_time = 0.3
        for i, pos in enumerate(positions):
            point = JointTrajectoryPoint()
            point.positions = [pos]
            time_sec = base_time * (i + 1)
            point.time_from_start = Duration(sec=int(time_sec), nanosec=int((time_sec % 1) * 1e9))
            goal_msg.trajectory.points.append(point)
        
        self.client.wait_for_server()
        self.client.send_goal_async(goal_msg)
        self.get_logger().info('Executing 16-step trajectory: Open → Close → Open')
        self.get_logger().info(f'Total duration: {len(positions) * base_time}s')


def print_menu():
    """Print menu"""
    print("\n" + "="*50)
    print("Simple Gripper Control - RIGHT Hand")
    print("="*50)
    print("  1 - Open gripper")
    print("  2 - Close gripper")
    print("  3 - Test 16-step sequence (Open→Close→Open)")
    print("  q - Quit")
    print("="*50)


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleGripperClient()
    
    print_menu()
    
    try:
        while rclpy.ok():
            cmd = input("\nCommand: ").strip().lower()
            
            if cmd == '1':
                node.open_gripper()
                print("✓ Opening gripper")
            
            elif cmd == '2':
                node.close_gripper()
                print("✓ Closing gripper")
            
            elif cmd == '3':
                node.test_16_steps_open_close_open()
                print("✓ Executing 16-step trajectory (4.8 seconds)")
            
            elif cmd == 'q':
                print("Exiting...")
                break
            
            else:
                print(f"✗ Unknown command: '{cmd}'")
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
