#!/usr/bin/env python3
"""
Record and replay position controller commands.

Records commands from position controllers and replays them using either TOPIC or ACTION interface.
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
import os
from datetime import datetime
import glob


# Ensure record_data folder is created in Python logic
class CommandRecorder(Node):
    """Record commands from position controllers."""
    
    def __init__(self, output_file=None):
        super().__init__('command_recorder')

        # Generate default file name and folder if none is provided
        if not output_file:  # Ensure output_file is None or empty
            now_str = datetime.now().strftime('%Y%m%d_%H%M%S')
            date_folder = datetime.now().strftime('%Y%m%d')
            base_dir = os.getcwd()
            record_data_dir = os.path.join(base_dir, 'record_data')
            if not os.path.exists(record_data_dir):
                os.makedirs(record_data_dir)
            output_dir = os.path.join(record_data_dir, date_folder)  # Current folder/record_data/YYYYMMDD
            os.makedirs(output_dir, exist_ok=True)  # Create folder if it doesn't exist
            output_file = os.path.join(output_dir, f'commands_recording_{now_str}.json')

        self.output_file = output_file
        self.recordings = []
        self.start_time = None
        self.lock = Lock()

        self.get_logger().info(f'Recording will be saved to: {self.output_file}')
        
        # Subscribe to arm controllers
        self.left_arm_sub = self.create_subscription(
            Float64MultiArray,
            '/left_forward_position_controller/commands',
            self.left_arm_callback,
            10
        )
        
        self.right_arm_sub = self.create_subscription(
            Float64MultiArray,
            '/right_forward_position_controller/commands',
            self.right_arm_callback,
            10
        )
        
        # Subscribe to LEAP Hand controller
        self.right_leaphand_sub = self.create_subscription(
            Float64MultiArray,
            '/right_hand_forward_position_controller/commands',
            self.right_leaphand_callback,
            10
        )
        
        # Status timer
        self.timer = self.create_timer(5.0, self.print_status)
        
        self.get_logger().info('Recording started...')
        self.get_logger().info('  Topics:')
        self.get_logger().info('    - /left_forward_position_controller/commands (left_arm)')
        self.get_logger().info('    - /right_forward_position_controller/commands (right_arm)')
        self.get_logger().info('    - /right_hand_forward_position_controller/commands (right_leaphand)')
        self.get_logger().info(f'  Output: {output_file}')
        self.get_logger().info('  Press Ctrl+C to stop and save')
    
    def left_arm_callback(self, msg):
        """Record left arm controller command."""
        self._record_command('left_arm', msg)
    
    def right_arm_callback(self, msg):
        """Record right arm controller command."""
        self._record_command('right_arm', msg)
    
    def right_leaphand_callback(self, msg):
        """Record right LEAP Hand controller command."""
        self._record_command('right_leaphand', msg)
    
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
            left_arm_count = sum(1 for r in self.recordings if r['controller'] == 'left_arm')
            right_arm_count = sum(1 for r in self.recordings if r['controller'] == 'right_arm')
            right_leaphand_count = sum(1 for r in self.recordings if r['controller'] == 'right_leaphand')
            duration = time.time() - self.start_time if self.start_time else 0
            
            self.get_logger().info('─' * 60)
            self.get_logger().info(f'Recording Status:')
            self.get_logger().info(f'  Duration: {duration:.1f}s')
            self.get_logger().info(f'  Left arm commands: {left_arm_count}')
            self.get_logger().info(f'  Right arm commands: {right_arm_count}')
            self.get_logger().info(f'  Right LEAP Hand commands: {right_leaphand_count}')
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


class CommandReplayerTopic(Node):
    """Replay recorded commands using TOPIC interface."""
    
    def __init__(self, input_file, speed=1.0):
        super().__init__('command_replayer_topic')
        
        self.input_file = input_file
        self.speed = speed
        
        # Publishers for arm controllers
        self.left_arm_pub = self.create_publisher(
            Float64MultiArray,
            '/left_forward_position_controller/commands',
            10
        )
        
        self.right_arm_pub = self.create_publisher(
            Float64MultiArray,
            '/right_forward_position_controller/commands',
            10
        )
        
        # Publisher for LEAP Hand controller
        self.right_leaphand_pub = self.create_publisher(
            Float64MultiArray,
            '/right_hand_forward_position_controller/commands',
            10
        )
        
        # Load recordings
        self.recordings = self.load_recordings()
        
        if not self.recordings:
            self.get_logger().error('No recordings found!')
            return
        
        self.current_index = 0
        self.start_time = None
        
        # Timer for replay (10ms resolution)
        self.timer = self.create_timer(0.01, self.replay_callback)
        
        self.get_logger().info('Replay ready (TOPIC mode)...')
        self.get_logger().info(f'  File: {input_file}')
        self.get_logger().info(f'  Commands: {len(self.recordings)}')
        self.get_logger().info(f'  Duration: {self.recordings[-1]["timestamp"]:.2f}s')
        self.get_logger().info(f'  Speed: {speed}x')
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
            
            # Publish command via topic
            msg = Float64MultiArray()
            msg.data = record['positions']
            
            if record['controller'] == 'left_arm':
                self.left_arm_pub.publish(msg)
            elif record['controller'] == 'right_arm':
                self.right_arm_pub.publish(msg)
            elif record['controller'] == 'right_leaphand':
                self.right_leaphand_pub.publish(msg)
            else:
                # Backward compatibility for old recordings
                if record['controller'] == 'left':
                    self.left_arm_pub.publish(msg)
                elif record['controller'] == 'right':
                    self.right_arm_pub.publish(msg)
            
            self.get_logger().info(
                f'[{record["timestamp"]:.3f}s] {record["controller"]}: '
                f'{[f"{p:.3f}" for p in record["positions"]]}'
            )
            
            self.current_index += 1
        
        # Stop when done
        if self.current_index >= len(self.recordings):
            self.get_logger().info('Replay completed!')
            self.timer.cancel()


class CommandReplayerAction(Node):
    """Replay recorded commands using ACTION interface."""
    
    def __init__(self, input_file, speed=1.0, time_from_start=1.0, batch_size=16):
        super().__init__('command_replayer_action')
        
        self.input_file = input_file
        self.speed = speed
        self.time_from_start = time_from_start
        self.batch_size = batch_size
        
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
        
        self.right_leaphand_joints = [
            'right_index_mcp_side', 'right_index_mcp_forward', 'right_index_pip', 'right_index_dip',
            'right_middle_mcp_side', 'right_middle_mcp_forward', 'right_middle_pip', 'right_middle_dip',
            'right_ring_mcp_side', 'right_ring_mcp_forward', 'right_ring_pip', 'right_ring_dip',
            'right_thumb_mcp_side', 'right_thumb_mcp_forward', 'right_thumb_pip_joint', 'right_thumb_dip_joint'
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
        self.right_leaphand_client = ActionClient(
            self, FollowJointTrajectory,
            '/right_hand_controller/follow_joint_trajectory'
        )
        
        # Wait for action servers
        self.get_logger().info('Waiting for action servers...')
        self.left_arm_client.wait_for_server()
        self.right_arm_client.wait_for_server()
        self.right_leaphand_client.wait_for_server()
        self.get_logger().info('Action servers ready!')
        
        # Load recordings
        self.recordings = self.load_recordings()
        
        if not self.recordings:
            self.get_logger().error('No recordings found!')
            return
        
        self.current_index = 0
        self.pending_goals = 0
        
        self.get_logger().info('Replay ready (ACTION mode - Batch processing)...')
        self.get_logger().info(f'  File: {input_file}')
        self.get_logger().info(f'  Commands: {len(self.recordings)}')
        self.get_logger().info(f'  Duration: {self.recordings[-1]["timestamp"]:.2f}s')
        self.get_logger().info(f'  Speed: {speed}x')
        self.get_logger().info(f'  Time from start: {time_from_start}s')
        self.get_logger().info(f'  Batch size: {batch_size} steps')
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
        self.get_logger().info('Replay started!')
        self.send_next_batch()
    
    def send_next_batch(self):
        """Send next batch of commands."""
        if self.current_index >= len(self.recordings):
            self.get_logger().info('Replay completed!')
            return
        
        # Collect commands for this batch (group by controller)
        left_arm_batch = []
        right_arm_batch = []
        right_leaphand_batch = []
        batch_start_idx = self.current_index
        
        # Collect up to batch_size commands
        for _ in range(self.batch_size):
            if self.current_index >= len(self.recordings):
                break
            
            record = self.recordings[self.current_index]
            controller = record['controller']
            
            # Handle new naming
            if controller == 'left_arm':
                left_arm_batch.append(record)
            elif controller == 'right_arm':
                right_arm_batch.append(record)
            elif controller == 'right_leaphand':
                right_leaphand_batch.append(record)
            # Backward compatibility for old recordings
            # elif controller == 'left':
            #     left_arm_batch.append(record)
            # elif controller == 'right':
            #     right_arm_batch.append(record)
            
            self.current_index += 1
        
        # Send trajectories for all controllers
        self.pending_goals = 0
        
        if left_arm_batch:
            self.send_trajectory_batch('left_arm', left_arm_batch)
            self.pending_goals += 1
        
        if right_arm_batch:
            self.send_trajectory_batch('right_arm', right_arm_batch)
            self.pending_goals += 1
        
        if right_leaphand_batch:
            self.send_trajectory_batch('right_leaphand', right_leaphand_batch)
            self.pending_goals += 1
        
        batch_end_idx = self.current_index - 1
        self.get_logger().info(
            f'Sent batch [{batch_start_idx}-{batch_end_idx}]: '
            f'Left_arm={len(left_arm_batch)}, Right_arm={len(right_arm_batch)}, '
            f'Right_leaphand={len(right_leaphand_batch)} steps'
        )
    
    def send_trajectory_batch(self, controller, batch):
        """Send trajectory with multiple points."""
        goal_msg = FollowJointTrajectory.Goal()
        
        # Set joint names and client
        if controller == 'left_arm':
            goal_msg.trajectory.joint_names = self.left_arm_joints
            action_client = self.left_arm_client
        elif controller == 'right_arm':
            goal_msg.trajectory.joint_names = self.right_arm_joints
            action_client = self.right_arm_client
        elif controller == 'right_leaphand':
            goal_msg.trajectory.joint_names = self.right_leaphand_joints
            action_client = self.right_leaphand_client
        else:
            self.get_logger().error(f'Unknown controller: {controller}')
            return
        
        # Create trajectory points
        points = []
        base_timestamp = batch[0]['timestamp']
        
        for record in batch:
            point = JointTrajectoryPoint()
            point.positions = record['positions']
            
            # Calculate time from start for this point
            time_offset = (record['timestamp'] - base_timestamp) / self.speed
            time_offset += self.time_from_start
            
            point.time_from_start = Duration(
                sec=int(time_offset),
                nanosec=int((time_offset % 1) * 1e9)
            )
            points.append(point)
        
        goal_msg.trajectory.points = points
        
        # Send goal and wait for result
        future = action_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.goal_response_callback(f, controller))
    
    def goal_response_callback(self, future, controller):
        """Handle goal response."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f'{controller} goal rejected!')
            self.pending_goals -= 1
            self.check_batch_complete()
            return
        
        self.get_logger().info(f'{controller} goal accepted, waiting for result...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.result_callback(f, controller))
    
    def result_callback(self, future, controller):
        """Handle action result."""
        result = future.result()
        self.get_logger().info(f'{controller} batch completed!')
        
        self.pending_goals -= 1
        self.check_batch_complete()
    
    def check_batch_complete(self):
        """Check if current batch is complete and send next."""
        if self.pending_goals == 0:
            self.get_logger().info('Batch complete, sending next batch...')
            self.send_next_batch()


def select_record_file():
    base_dir = os.getcwd()
    record_data_dir = os.path.join(base_dir, 'record_data')
    folders = sorted([d for d in os.listdir(record_data_dir) if os.path.isdir(os.path.join(record_data_dir, d))])
    if not folders:
        print('No record_data folders found.')
        return None
    print('Available folders:')
    for idx, folder in enumerate(folders):
        print(f'{idx+1}: {folder}')
    folder_idx = int(input('Select folder number: ')) - 1
    folder_path = os.path.join(record_data_dir, folders[folder_idx])
    files = sorted(glob.glob(os.path.join(folder_path, '*.json')))
    if not files:
        print('No JSON files found in selected folder.')
        return None
    print('Available files:')
    for idx, file in enumerate(files):
        print(f'{idx+1}: {os.path.basename(file)}')
    file_idx = int(input('Select file number: ')) - 1
    return files[file_idx]


def main():
    parser = argparse.ArgumentParser(
        description='Record/replay position controller commands',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Record commands
  python3 scripts/record_replay_commands.py record --file /tmp/my_recording.json
  
  # Replay using topic interface
  python3 scripts/record_replay_commands.py replay -t --file /tmp/my_recording.json
  
  # Replay using action interface (16 steps per batch)
  python3 scripts/record_replay_commands.py replay -a --file /tmp/my_recording.json --time-from-start 0.8
  
  # Replay with custom batch size
  python3 scripts/record_replay_commands.py replay -a --file /tmp/my_recording.json --batch-size 32
  
  # Replay at 2x speed
  python3 scripts/record_replay_commands.py replay -a --speed 2.0
        """
    )
    
    parser.add_argument('mode', choices=['record', 'replay'], 
                       help='Mode: record or replay')
    parser.add_argument('--file', default=None, 
                       help='Recording file path (default: auto-generate in record_data/YYYYMMDD)')
    parser.add_argument('--speed', type=float, default=1.0,
                       help='Replay speed multiplier (default: 1.0, e.g., 2.0 = 2x faster)')
    
    # Replay mode selection (mutually exclusive)
    replay_group = parser.add_mutually_exclusive_group()
    replay_group.add_argument('-t', action='store_true', dest='topic',
                             help='Replay using TOPIC interface (direct position commands)')
    replay_group.add_argument('-a', action='store_true', dest='action',
                             help='Replay using ACTION interface (trajectory with interpolation)')
    
    # Action-specific parameters
    parser.add_argument('--time-from-start', type=float, default=0.5,
                       help='Time from start for action trajectory in seconds (default: 0.5)')
    parser.add_argument('--batch-size', type=int, default=16,
                       help='Number of trajectory points per batch for action mode (default: 16)')
    
    args = parser.parse_args()
    
    # Validate replay mode selection
    if args.mode == 'replay' and not args.topic and not args.action:
        parser.error('Replay mode requires either -t/--topic or -a/--action')
    
    rclpy.init()
    
    node = None
    try:
        if args.mode == 'record':
            node = CommandRecorder(args.file if args.file else None)
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                node.get_logger().info('\nStopping recording...')
                node.save()
        
        else:  # replay
            input_file = args.file
            if input_file is None:
                input_file = select_record_file()
            if input_file is None:
                print('No valid file selected. Exiting.')
                return
            if args.topic:
                node = CommandReplayerTopic(input_file, args.speed)
            elif args.action:
                node = CommandReplayerAction(input_file, args.speed, args.time_from_start, args.batch_size)
            
            rclpy.spin(node)
    
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
