#!/usr/bin/env python3
"""
Gripper Controller Multi-Mode Analyzer

- 支援三種 mode: action, forward, trajectory
- 訂閱對應的 command topic/action 以及 joint_states
- 比較 command/state/error，儲存 CSV，繪圖

Usage:
  python3 analy_gripper_multi_mode.py --mode action --hand right
  python3 analy_gripper_multi_mode.py --mode forward --hand left
  python3 analy_gripper_multi_mode.py --mode trajectory --hand both
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand, FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import numpy as np
try:
    import matplotlib
    import matplotlib.pyplot as plt
    for backend in ['TkAgg', 'Qt5Agg', 'GTK3Agg', 'WXAgg']:
        try:
            matplotlib.use(backend)
            break
        except Exception:
            continue
    HAS_MPL = True
except Exception:
    HAS_MPL = False
import threading
import time
import sys
from pathlib import Path
from datetime import datetime
import argparse

GRIPPER_JOINTS = {
    'left': 'openarm_left_finger_joint1',
    'right': 'openarm_right_finger_joint1'
}

class GripperMultiModeAnalyzer(Node):
    def action_goal_cb(self, msg, hand):
        # 取得 action goal position for a given hand
        try:
            cmd = float(msg.command.position)
        except Exception:
            cmd = np.nan
        t = time.time()
        with self.lock:
            if self.start_time is None:
                self.start_time = t
            rel = t - self.start_time
            joint = GRIPPER_JOINTS[hand]
            self.latest_cmd[joint] = cmd
            self.cmd_history[joint].append((rel, cmd))

    def __init__(self, mode='action', hand='right'):
        super().__init__('gripper_multi_mode_analyzer')
        self.mode = mode
        self.hand = hand
        # determine tracked joints (support 'both')
        if hand == 'both':
            self.joint_names = [GRIPPER_JOINTS['right'], GRIPPER_JOINTS['left']]
        else:
            self.joint_names = [GRIPPER_JOINTS[hand]]

        # history per joint: lists of (t, value)
        self.cmd_history = {j: [] for j in self.joint_names}
        self.state_history = {j: [] for j in self.joint_names}
        self.latest_cmd = {j: np.nan for j in self.joint_names}
        self.latest_state = {j: np.nan for j in self.joint_names}
        self.lock = threading.Lock()
        self.start_time = None
        # Topic/action selection
        if mode == 'action':
            # 訂閱 action goal topic
            from control_msgs.action import GripperCommand
            from rclpy.action import ActionClient
            # subscribe to goal for one or both hands
            hands = ['right', 'left'] if hand == 'both' else [hand]
            for h in hands:
                topic = f'/{h}_gripper_controller/gripper_cmd/goal'
                # Use default argument to capture h properly in closure
                self.create_subscription(GripperCommand.Goal, topic, 
                    lambda msg, hand=h: self.action_goal_cb(msg, hand), 10)
        elif mode == 'forward':
            hands = ['right', 'left'] if hand == 'both' else [hand]
            for h in hands:
                topic = f'/{h}_gripper_forward_controller/commands'
                # Use default argument to capture h properly in closure
                self.create_subscription(Float64MultiArray, topic, 
                    lambda msg, hand=h: self.forward_cmd_cb(msg, hand), 10)
        elif mode == 'trajectory':
            hands = ['right', 'left'] if hand == 'both' else [hand]
            for h in hands:
                topic = f'/{h}_gripper_trajectory_controller/controller_state'
                # Use default argument to capture h properly in closure
                self.create_subscription(JointTrajectoryControllerState, topic, 
                    lambda msg, hand=h: self.controller_state_cb(msg, hand), 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 20)
        self.get_logger().info(f'Analyzer initialized: mode={mode}, hand={hand}')

    def controller_state_cb(self, msg, hand):
        # controller_state provides reference.positions for multiple joints
        joint = GRIPPER_JOINTS[hand]
        
        # Debug: print message structure on first call
        if not hasattr(self, '_debug_printed'):
            self.get_logger().info(f'controller_state for {hand}: joint_names={msg.joint_names}')
            self.get_logger().info(f'  reference.positions length: {len(msg.reference.positions) if hasattr(msg.reference, "positions") else "N/A"}')
            self._debug_printed = True
        
        try:
            idx = msg.joint_names.index(joint)
            cmd = msg.reference.positions[idx]
            self.get_logger().info(f'Successfully extracted cmd for {joint}: {cmd:.4f}', throttle_duration_sec=2.0)
        except ValueError as e:
            self.get_logger().warn(f'{joint} not found in msg.joint_names: {msg.joint_names}')
            cmd = np.nan
        except IndexError as e:
            self.get_logger().warn(f'Index {idx} out of range for reference.positions (len={len(msg.reference.positions)})')
            cmd = np.nan
        except Exception as e:
            self.get_logger().warn(f'Failed to extract cmd for {joint}: {type(e).__name__}: {e}')
            cmd = np.nan
        
        t = time.time()
        with self.lock:
            if self.start_time is None:
                self.start_time = t
            rel = t - self.start_time
            self.latest_cmd[joint] = cmd
            self.cmd_history[joint].append((rel, cmd))

    def forward_cmd_cb(self, msg, hand):
        # Only one joint per gripper, take first value
        joint = GRIPPER_JOINTS[hand]
        try:
            print(f'forward_cmd_cb for {hand}: received data={msg.data}')
            cmd = float(msg.data[0]) if len(msg.data) > 0 else np.nan
            self.get_logger().info(f'Received forward cmd for {hand}: {cmd:.4f}', throttle_duration_sec=2.0)
        except Exception as e:
            self.get_logger().warn(f'Failed to extract forward cmd for {hand}: {e}')
            cmd = np.nan
        t = time.time()
        with self.lock:
            if self.start_time is None:
                self.start_time = t
            rel = t - self.start_time
            self.latest_cmd[joint] = cmd
            self.cmd_history[joint].append((rel, cmd))

    def joint_state_cb(self, msg):
        t = time.time()
        with self.lock:
            if self.start_time is None:
                self.start_time = t
            rel = t - self.start_time
            # record state for each tracked joint if available
            for joint in self.joint_names:
                try:
                    idx = msg.name.index(joint)
                    state = msg.position[idx]
                except Exception:
                    state = np.nan
                self.latest_state[joint] = state
                self.state_history[joint].append((rel, state))

    def save_and_plot(self, out_prefix=None):
        import csv
        with self.lock:
            today = datetime.now().strftime('%Y%m%d')
            base_dir = Path('analy_gripper')
            csv_dir = base_dir / 'csv' / today
            img_dir = base_dir / 'img' / today
            csv_dir.mkdir(parents=True, exist_ok=True)
            img_dir.mkdir(parents=True, exist_ok=True)

            for joint in self.joint_names:
                times_cmd = np.array([t for t, _ in self.cmd_history[joint]]) if self.cmd_history[joint] else np.array([])
                cmds = np.array([v for _, v in self.cmd_history[joint]]) if self.cmd_history[joint] else np.array([])
                times_state = np.array([t for t, _ in self.state_history[joint]]) if self.state_history[joint] else np.array([])
                states = np.array([v for _, v in self.state_history[joint]]) if self.state_history[joint] else np.array([])

                if out_prefix is None:
                    prefix = f'{joint.replace("openarm_","").replace("_finger_joint1","")}_{self.mode}_{datetime.now().strftime("%H%M%S")}'
                else:
                    prefix = f'{out_prefix}_{joint}'

                if times_cmd.size == 0 and times_state.size == 0:
                    self.get_logger().info(f'No samples for {joint}; skipping')
                    continue

                # Align data: use union of times, interpolate both cmd and state
                if times_cmd.size > 0 and times_state.size > 0:
                    # Both have data: merge time arrays and interpolate
                    times = np.sort(np.unique(np.concatenate([times_cmd, times_state])))
                    cmds_vals = np.interp(times, times_cmd, cmds, left=np.nan, right=np.nan)
                    interp_states = np.interp(times, times_state, states, left=np.nan, right=np.nan)
                elif times_cmd.size > 0:
                    # Only cmd data available
                    times = times_cmd
                    cmds_vals = cmds
                    interp_states = np.full(times.shape, np.nan)
                elif times_state.size > 0:
                    # Only state data available
                    times = times_state
                    cmds_vals = np.full(times.shape, np.nan)
                    interp_states = states
                else:
                    # No data at all
                    times = np.array([])
                    cmds_vals = np.array([])
                    interp_states = np.array([])
                
                errors = cmds_vals - interp_states

                csv_path = csv_dir / f'{prefix}.csv'
                with open(csv_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['time', 'cmd', 'state', 'error'])
                    for t, cmd, state, err in zip(times, cmds_vals, interp_states, errors):
                        writer.writerow([t, cmd, state, err])
                self.get_logger().info(f'CSV saved: {csv_path}')

                # plotting
                if not HAS_MPL:
                    continue
                plt.figure(figsize=(10,4))
                plt.plot(times, cmds_vals, label='Command', linestyle='--', color='r')
                plt.plot(times, interp_states, label='State', linestyle='-', color='b')
                plt.plot(times, errors, label='Error', linestyle=':', color='g')
                plt.xlabel('Time (s)')
                plt.ylabel('Position (m)')
                plt.title(f'{joint} Command vs State ({self.mode})')
                plt.legend()
                plt.grid(True, alpha=0.3)
                img_path = img_dir / f'{prefix}.png'
                try:
                    plt.savefig(img_path, dpi=150, bbox_inches='tight')
                    self.get_logger().info(f'Plot saved: {img_path}')
                except Exception as ex:
                    self.get_logger().error(f'Failed to save plot for {joint}: {ex}')
                try:
                    plt.show()
                except Exception:
                    pass

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['action', 'forward', 'trajectory'], default='forward')
    parser.add_argument('--hand', choices=['left', 'right', 'both'], default='both')
    parser.add_argument('--csv', type=str, default=None, help='If set, only plot from CSV (no ROS)')
    args = parser.parse_args()
    if args.csv:
        import csv
        times, cmds, states, errors = [], [], [], []
        with open(args.csv, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                times.append(float(row['time']))
                cmds.append(float(row['cmd']))
                states.append(float(row['state']))
                errors.append(float(row['error']))
        plt.figure(figsize=(12,6))
        plt.plot(times, cmds, label='Command', linestyle='--', color='r')
        plt.plot(times, states, label='State', linestyle='-', color='b')
        plt.plot(times, errors, label='Error', linestyle=':', color='g')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.title('Gripper Command vs State')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.show()
        return
    rclpy.init()
    node = GripperMultiModeAnalyzer(mode=args.mode, hand=args.hand)
    print('Press Ctrl+C to stop and save results.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nInterrupted. Saving and plotting...')
        try:
            node.save_and_plot()
        except Exception as e:
            print(f'Error during save_and_plot: {e}')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if not rclpy.ok():
                pass  # already shutdown
            else:
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
