#!/usr/bin/env python3
"""
Record and replay position controller commands.

Records commands from position controllers and replays them with precise timing.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
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


class CommandReplayer(Node):
    """Replay recorded commands."""
    
    def __init__(self, input_file, speed=1.0):
        super().__init__('command_replayer')
        
        self.input_file = input_file
        self.speed = speed
        
        # Publishers for both controllers
        self.left_pub = self.create_publisher(
            Float64MultiArray,
            '/left_forward_position_controller/commands',
            10
        )
        
        self.right_pub = self.create_publisher(
            Float64MultiArray,
            '/right_forward_position_controller/commands',
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
        
        self.get_logger().info('Replay ready...')
        self.get_logger().info(f'  File: {input_file}')
        self.get_logger().info(f'  Commands: {len(self.recordings)}')
        self.get_logger().info(f'  Duration: {self.recordings[-1]["timestamp"]:.2f}s')
        self.get_logger().info(f'  Speed: {speed}x')
        self.get_logger().info('Starting in 2 seconds...')
        
        # Delayed start (manual one-shot implementation)
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
        """Delayed start callback (one-shot)."""
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
            
            # Publish command
            msg = Float64MultiArray()
            msg.data = record['positions']
            
            # Recordings from record_replay_commands.py use 'left_arm'/'right_arm';
            # older recordings used 'left'/'right'. Accept both.
            if record['controller'] in ('left', 'left_arm'):
                self.left_pub.publish(msg)
            elif record['controller'] in ('right', 'right_arm'):
                self.right_pub.publish(msg)
            else:
                self.get_logger().warn(
                    f'Unknown controller "{record["controller"]}", skipping'
                )
                self.current_index += 1
                continue
            
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
            node = CommandReplayer(args.file, args.speed)
            rclpy.spin(node)
    
    finally:
        if node is not None:
            node.destroy_node()
        
        rclpy.shutdown()


if __name__ == '__main__':
    main()
