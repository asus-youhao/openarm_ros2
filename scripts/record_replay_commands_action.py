#!/usr/bin/env python3
"""
Record and replay position controller commands.

Records commands from position controllers and replays them using ACTION interface.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import json
import time
import argparse
from threading import Lock


class CommandRecorder(Node):
    """Record commands from position controllers."""
    
    def __init__(self, output_file):
        super().__init__('command_recorder')
        
        self.output_file = output_file
        self.recordings = []
        self.start_time = None
        self.lock = Lock()
        
        # Subscribe to both controllers
        self.left_sub = self.create_subscription(
            Float64MultiArray,
            '/left_forward_position_controller/commands',
            self.left_callback,
            10
        )
        
        self.right_sub = self.create_subscription(
            Float64MultiArray,
            '/right_forward_position_controller/commands',
            self.right_callback,
            10
        )
        
        # Status timer
        self.timer = self.create_timer(5.0, self.print_status)
        
        self.get_logger().info('Recording started...')
        self.get_logger().info('  Topics:')
        self.get_logger().info('    - /left_forward_position_controller/commands')
        self.get_logger().info('    - /right_forward_position_controller/commands')
        self.get_logger().info(f'  Output: {output_file}')
        self.get_logger().info('  Press Ctrl+C to stop and save')
    
    def left_callback(self, msg):
        """Record left controller command."""
        self._record_command('left', msg)
    
    def right_callback(self, msg):
        """Record right controller command."""
        self._record_command('right', msg)
    
    def _record_command(self, controller, msg):
        """Record command with timestamp."""
        with self.lock:
            current_time = time.time()
            
            if self.start_time is None:
                self.start_time = current_time
                timestamp = 0.0
            else:
                timestamp = current_time - self.start_time
            
            record = {
                'timestamp': timestamp,
                'controller': controller,
                'positions': list(msg.data)
            }
            
            self.recordings.append(record)
            
            self.get_logger().info(
                f'[{timestamp:.3f}s] {controller}: {[f"{p:.3f}" for p in msg.data]}'
            )
    
    def print_status(self):
        """Print recording status."""
        with self.lock:
            left_count = sum(1 for r in self.recordings if r['controller'] == 'left')
            right_count = sum(1 for r in self.recordings if r['controller'] == 'right')
            duration = time.time() - self.start_time if self.start_time else 0
            
            self.get_logger().info('─' * 60)
            self.get_logger().info(f'Recording Status:')
            self.get_logger().info(f'  Duration: {duration:.1f}s')
            self.get_logger().info(f'  Left commands: {left_count}')
            self.get_logger().info(f'  Right commands: {right_count}')
            self.get_logger().info(f'  Total: {len(self.recordings)}')
            self.get_logger().info('─' * 60)
    
    def save(self):
        """Save recordings to file."""
        with self.lock:
            data = {
                'duration': time.time() - self.start_time if self.start_time else 0,
                'total_commands': len(self.recordings),
                'recordings': self.recordings
            }
            
            with open(self.output_file, 'w') as f:
                json.dump(data, f, indent=2)
            
            self.get_logger().info(f'Saved {len(self.recordings)} commands to {self.output_file}')


class CommandReplayerAction(Node):
    """Replay recorded commands using ACTION interface."""
    
    def __init__(self, input_file, speed=1.0, time_from_start=0.5):
        super().__init__('command_replayer_action')
        
        self.input_file = input_file
        self.speed = speed
        self.time_from_start = time_from_start
        
        # Joint names
        self.left_arm_joints = [
            'openarm_left_joint1', 'openarm_left_joint2', 'openarm_left_joint3',
            'openarm_left_joint4', 'openarm_left_joint5', 'openarm_left_joint6',
            'openarm_left_joint7'
        ]
        
        self.right_arm_joints = [
            'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3',
            'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6',
            'openarm_right_joint7'
        ]
        
        # Action clients for trajectory control
        self.left_arm_client = ActionClient(
            self, FollowJointTrajectory, 
            '/left_joint_trajectory_controller/follow_joint_trajectory'
        )
        self.right_arm_client = ActionClient(
            self, FollowJointTrajectory, 
            '/right_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Wait for action servers
        self.get_logger().info('Waiting for action servers...')
        self.left_arm_client.wait_for_server()
        self.right_arm_client.wait_for_server()
        self.get_logger().info('Action servers ready!')
        
        # Load recordings
        self.recordings = self.load_recordings()
        
        if not self.recordings:
            self.get_logger().error('No recordings found!')
            return
        
        self.current_index = 0
        self.start_time = None
        
        # Timer for replay (10ms resolution)
        self.timer = self.create_timer(0.01, self.replay_callback)
        
        self.get_logger().info('Replay ready (ACTION mode)...')
        self.get_logger().info(f'  File: {input_file}')
        self.get_logger().info(f'  Commands: {len(self.recordings)}')
        self.get_logger().info(f'  Duration: {self.recordings[-1]["timestamp"]:.2f}s')
        self.get_logger().info(f'  Speed: {speed}x')
        self.get_logger().info(f'  Time from start: {time_from_start}s')
        self.get_logger().info('Starting in 2 seconds...')
        
        # Delayed start
        self.start_timer = self.create_timer(2.0, self._delayed_start)
    
    def load_recordings(self):
        """Load recordings from file."""
        try:
            with open(self.input_file, 'r') as f:
                data = json.load(f)
            return data.get('recordings', [])
        except Exception as e:
            self.get_logger().error(f'Failed to load {self.input_file}: {e}')
            return []
    
    def _delayed_start(self):
        """Delayed start callback."""
        self.start_timer.cancel()
        self.start_time = time.time()
        self.get_logger().info('Replay started!')
    
    def send_trajectory_action(self, controller, positions):
        """Send trajectory command via action."""
        goal_msg = FollowJointTrajectory.Goal()
        
        # Set joint names based on controller
        if controller == 'left':
            goal_msg.trajectory.joint_names = self.left_arm_joints
            action_client = self.left_arm_client
        else:  # right
            goal_msg.trajectory.joint_names = self.right_arm_joints
            action_client = self.right_arm_client
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(self.time_from_start),
            nanosec=int((self.time_from_start % 1) * 1e9)
        )
        
        goal_msg.trajectory.points = [point]
        
        # Send goal asynchronously
        action_client.send_goal_async(goal_msg)
    
    def replay_callback(self):
        """Replay commands at appropriate times."""
        if self.start_time is None or self.current_index >= len(self.recordings):
            return
        
        elapsed = (time.time() - self.start_time) * self.speed
        
        # Publish all commands that should have happened by now
        while self.current_index < len(self.recordings):
            record = self.recordings[self.current_index]
            
            if record['timestamp'] > elapsed:
                break
            
            # Send via action
            self.send_trajectory_action(record['controller'], record['positions'])
            
            self.get_logger().info(
                f'[{record["timestamp"]:.3f}s] {record["controller"]}: '
                f'{[f"{p:.3f}" for p in record["positions"]]}'
            )
            
            self.current_index += 1
        
        # Stop when done
        if self.current_index >= len(self.recordings):
            self.get_logger().info('Replay completed!')
            self.timer.cancel()


def main():
    parser = argparse.ArgumentParser(description='Record/replay position controller commands')
    parser.add_argument('mode', choices=['record', 'replay'], help='Mode: record or replay')
    parser.add_argument('--file', default='/tmp/commands_recording.json', 
                       help='Recording file path')
    parser.add_argument('--speed', type=float, default=1.0,
                       help='Replay speed (1.0 = normal, 2.0 = 2x faster)')
    parser.add_argument('--time-from-start', type=float, default=0.5,
                       help='Time from start for action trajectory (seconds)')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    node = None
    try:
        if args.mode == 'record':
            node = CommandRecorder(args.file)
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                node.get_logger().info('\nStopping recording...')
                node.save()
        
        else:  # replay
            node = CommandReplayerAction(args.file, args.speed, args.time_from_start)
            rclpy.spin(node)
    
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
