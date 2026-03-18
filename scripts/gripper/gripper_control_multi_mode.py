#!/usr/bin/env python3
"""
Multi-mode Gripper Control Script

Supports three controller types:
1. GripperActionController - Action-based control with effort
2. ForwardCommandController - Direct position command via topic
3. JointTrajectoryController - Trajectory-based control with timing

Usage:
    python3 gripper_control_multi_mode.py --mode action --hand right
    python3 gripper_control_multi_mode.py --mode forward --hand left
    python3 gripper_control_multi_mode.py --mode trajectory --hand both
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand, FollowJointTrajectory
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import argparse
import sys


class MultiModeGripperClient(Node):
    def __init__(self, mode='action', hand='right'):
        super().__init__('multi_mode_gripper_client')
        self.mode = mode
        self.hand = hand
        
        # Gripper position limits (meters)
        self.GRIPPER_OPEN = 0.044
        self.GRIPPER_CLOSE = 0.0
        self.GRIPPER_HALF = 0.022
        
        # Initialize based on mode and hand
        if hand == 'both':
            self.init_both_grippers()
        elif hand == 'left':
            self.init_single_gripper('left')
        else:  # default 'right'
            self.init_single_gripper('right')
        
        self.get_logger().info(f'Gripper client initialized: mode={mode}, hand={hand}')
    
    def init_both_grippers(self):
        """Initialize controllers for both hands"""
        if self.mode == 'action':
            self.left_client = ActionClient(self, GripperCommand, '/left_gripper_controller/gripper_cmd')
            self.right_client = ActionClient(self, GripperCommand, '/right_gripper_controller/gripper_cmd')
        elif self.mode == 'forward':
            self.left_pub = self.create_publisher(Float64MultiArray, '/left_gripper_forward_position_controller/commands', 10)
            self.right_pub = self.create_publisher(Float64MultiArray, '/right_gripper_forward_position_controller/commands', 10)
        elif self.mode == 'trajectory':
            self.left_client = ActionClient(self, FollowJointTrajectory, '/left_gripper_trajectory_controller/follow_joint_trajectory')
            self.right_client = ActionClient(self, FollowJointTrajectory, '/right_gripper_trajectory_controller/follow_joint_trajectory')
    
    def init_single_gripper(self, hand_prefix):
        """Initialize controller for single hand"""
        if self.mode == 'action':
            self.client = ActionClient(self, GripperCommand, f'/{hand_prefix}_gripper_controller/gripper_cmd')
        elif self.mode == 'forward':
            self.pub = self.create_publisher(Float64MultiArray, f'/{hand_prefix}_gripper_forward_controller/commands', 10)
        elif self.mode == 'trajectory':
            self.client = ActionClient(self, FollowJointTrajectory, f'/{hand_prefix}_gripper_trajectory_controller/follow_joint_trajectory')
        self.joint_name = f'openarm_{hand_prefix}_finger_joint1'
    
    def send_gripper_action(self, position, max_effort=50.0, hand_side=None):
        """Send gripper command via GripperActionController"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort
        
        if self.hand == 'both':
            if hand_side == 'left' or hand_side is None:
                self.left_client.wait_for_server()
                self.left_client.send_goal_async(goal_msg)
            if hand_side == 'right' or hand_side is None:
                self.right_client.wait_for_server()
                self.right_client.send_goal_async(goal_msg)
        else:
            self.client.wait_for_server()
            self.client.send_goal_async(goal_msg)
    
    def send_gripper_forward(self, position, hand_side=None):
        """Send gripper command via ForwardCommandController"""
        msg = Float64MultiArray()
        msg.data = [position]
        
        if self.hand == 'both':
            if hand_side == 'left' or hand_side is None:
                self.left_pub.publish(msg)
            if hand_side == 'right' or hand_side is None:
                self.right_pub.publish(msg)
        else:
            self.pub.publish(msg)
    
    def send_gripper_trajectory(self, positions, durations, hand_side=None):
        """
        Send gripper trajectory via JointTrajectoryController
        
        Args:
            positions: list of target positions (e.g., [0.0, 0.044])
            durations: list of time durations in seconds (e.g., [0.5, 1.0])
        """
        goal_msg = FollowJointTrajectory.Goal()
        
        # Determine joint name based on hand
        if self.hand == 'both':
            if hand_side == 'left':
                joint_name = 'openarm_left_finger_joint1'
            else:  # right or both
                joint_name = 'openarm_right_finger_joint1'
        else:
            joint_name = self.joint_name
        
        goal_msg.trajectory.joint_names = [joint_name]
        
        for pos, dur in zip(positions, durations):
            point = JointTrajectoryPoint()
            point.positions = [pos]
            point.time_from_start = Duration(sec=int(dur), nanosec=int((dur % 1) * 1e9))
            goal_msg.trajectory.points.append(point)
        
        if self.hand == 'both':
            if hand_side == 'left' or hand_side is None:
                self.left_client.wait_for_server()
                self.left_client.send_goal_async(goal_msg)
            if hand_side == 'right' or hand_side is None:
                # Re-create goal for right hand
                if hand_side is None:
                    goal_msg_right = FollowJointTrajectory.Goal()
                    goal_msg_right.trajectory.joint_names = ['openarm_right_finger_joint1']
                    for pos, dur in zip(positions, durations):
                        point = JointTrajectoryPoint()
                        point.positions = [pos]
                        point.time_from_start = Duration(sec=int(dur), nanosec=int((dur % 1) * 1e9))
                        goal_msg_right.trajectory.points.append(point)
                    self.right_client.wait_for_server()
                    self.right_client.send_goal_async(goal_msg_right)
                else:
                    self.right_client.wait_for_server()
                    self.right_client.send_goal_async(goal_msg)
        else:
            self.client.wait_for_server()
            self.client.send_goal_async(goal_msg)
    
    def send_command(self, position, hand_side=None):
        """Universal send command that routes to appropriate controller"""
        if self.mode == 'action':
            self.send_gripper_action(position, hand_side=hand_side)
        elif self.mode == 'forward':
            self.send_gripper_forward(position, hand_side=hand_side)
        elif self.mode == 'trajectory':
            # For simple position command, create a single-point trajectory
            self.send_gripper_trajectory([position], [0.5], hand_side=hand_side)
    
    def send_sequence(self, sequence_name, hand_side=None):
        """Send predefined movement sequences"""
        if sequence_name == 'open_close_open':
            if self.mode == 'trajectory':
                self.send_gripper_trajectory(
                    [self.GRIPPER_OPEN, self.GRIPPER_CLOSE, self.GRIPPER_OPEN],
                    [0.5, 1.5, 2.5],
                    hand_side=hand_side
                )
            else:
                # For non-trajectory modes, send commands sequentially
                import time
                self.send_command(self.GRIPPER_OPEN, hand_side)
                time.sleep(1.0)
                self.send_command(self.GRIPPER_CLOSE, hand_side)
                time.sleep(1.0)
                self.send_command(self.GRIPPER_OPEN, hand_side)
        
        elif sequence_name == 'close_open_close':
            if self.mode == 'trajectory':
                self.send_gripper_trajectory(
                    [self.GRIPPER_CLOSE, self.GRIPPER_OPEN, self.GRIPPER_CLOSE],
                    [0.5, 1.5, 2.5],
                    hand_side=hand_side
                )
            else:
                import time
                self.send_command(self.GRIPPER_CLOSE, hand_side)
                time.sleep(1.0)
                self.send_command(self.GRIPPER_OPEN, hand_side)
                time.sleep(1.0)
                self.send_command(self.GRIPPER_CLOSE, hand_side)


def print_menu(hand, mode):
    """Print interactive menu"""
    print("\n" + "="*60)
    print(f"Multi-Mode Gripper Control - Hand: {hand.upper()} - Mode: {mode.upper()}")
    print("="*60)
    print("Basic Commands:")
    print("  1 - Open gripper (0.044m)")
    print("  2 - Close gripper (0.0m)")
    print("  3 - Half open (0.022m)")
    print()
    if hand == 'both':
        print("Hand Selection (for next command):")
        print("  l - Left hand only")
        print("  r - Right hand only")
        print("  b - Both hands (default)")
        print()
    print("Sequences:")
    print("  o - Open-Close-Open sequence")
    print("  c - Close-Open-Close sequence")
    print()
    print("  q - Quit")
    print("="*60)


def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description='Multi-mode gripper control')
    parser.add_argument('--mode', choices=['action', 'forward', 'trajectory'], default=None,
                        help='Controller mode: action (GripperActionController), forward (ForwardCommandController), trajectory (JointTrajectoryController)')
    parser.add_argument('--hand', choices=['left', 'right', 'both'], default=None,
                        help='Which gripper to control: left, right, or both')
    parsed_args = parser.parse_args()
    
    # Interactive mode selection if not provided via CLI
    if parsed_args.mode is None:
        print("\n" + "="*60)
        print("Select Controller Mode:")
        print("="*60)
        print("  1 - Action Mode (GripperActionController)")
        print("      → Single goal with effort control")
        print("      → Has action feedback")
        print()
        print("  2 - Forward Mode (ForwardCommandController)")
        print("      → Direct position commands via topic")
        print("      → Low latency, immediate response")
        print()
        print("  3 - Trajectory Mode (JointTrajectoryController)")
        print("      → Smooth trajectories with timing")
        print("      → Best for sequences")
        print("="*60)
        
        while True:
            mode_choice = input("Enter mode (1/2/3): ").strip()
            if mode_choice == '1':
                parsed_args.mode = 'action'
                break
            elif mode_choice == '2':
                parsed_args.mode = 'forward'
                break
            elif mode_choice == '3':
                parsed_args.mode = 'trajectory'
                break
            else:
                print("Invalid choice. Please enter 1, 2, or 3.")
    
    # Interactive hand selection if not provided via CLI
    if parsed_args.hand is None:
        print("\n" + "="*60)
        print("Select Gripper Hand:")
        print("="*60)
        print("  1 - Left hand")
        print("  2 - Right hand")
        print("  3 - Both hands")
        print("="*60)
        
        while True:
            hand_choice = input("Enter hand (1/2/3): ").strip()
            if hand_choice == '1':
                parsed_args.hand = 'left'
                break
            elif hand_choice == '2':
                parsed_args.hand = 'right'
                break
            elif hand_choice == '3':
                parsed_args.hand = 'both'
                break
            else:
                print("Invalid choice. Please enter 1, 2, or 3.")
    
    node = MultiModeGripperClient(mode=parsed_args.mode, hand=parsed_args.hand)
    
    hand_side = None  # For 'both' mode, tracks current hand selection
    
    print_menu(parsed_args.hand, parsed_args.mode)
    
    try:
        while rclpy.ok():
            cmd = input("\nCommand: ").strip().lower()
            
            if cmd == '1':
                node.send_command(node.GRIPPER_OPEN, hand_side)
                print(f"✓ Sent OPEN command (0.044m)" + (f" to {hand_side or 'both'}" if parsed_args.hand == 'both' else ""))
            
            elif cmd == '2':
                node.send_command(node.GRIPPER_CLOSE, hand_side)
                print(f"✓ Sent CLOSE command (0.0m)" + (f" to {hand_side or 'both'}" if parsed_args.hand == 'both' else ""))
            
            elif cmd == '3':
                node.send_command(node.GRIPPER_HALF, hand_side)
                print(f"✓ Sent HALF-OPEN command (0.022m)" + (f" to {hand_side or 'both'}" if parsed_args.hand == 'both' else ""))
            
            elif cmd == 'o':
                node.send_sequence('open_close_open', hand_side)
                print(f"✓ Sent OPEN-CLOSE-OPEN sequence" + (f" to {hand_side or 'both'}" if parsed_args.hand == 'both' else ""))
            
            elif cmd == 'c':
                node.send_sequence('close_open_close', hand_side)
                print(f"✓ Sent CLOSE-OPEN-CLOSE sequence" + (f" to {hand_side or 'both'}" if parsed_args.hand == 'both' else ""))
            
            elif cmd == 'l' and parsed_args.hand == 'both':
                hand_side = 'left'
                print("→ Selected LEFT hand for next command")
            
            elif cmd == 'r' and parsed_args.hand == 'both':
                hand_side = 'right'
                print("→ Selected RIGHT hand for next command")
            
            elif cmd == 'b' and parsed_args.hand == 'both':
                hand_side = None
                print("→ Selected BOTH hands for next command")
            
            elif cmd == 'q':
                print("Exiting...")
                break
            
            elif cmd == 'h' or cmd == 'help':
                print_menu(parsed_args.hand, parsed_args.mode)
            
            else:
                print(f"Unknown command: '{cmd}'. Type 'h' for help.")
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
