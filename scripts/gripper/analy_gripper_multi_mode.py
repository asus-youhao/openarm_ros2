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
import matplotlib.pyplot as plt
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
    def action_goal_cb(self, msg):
        # 取得 action goal position
        try:
            cmd = float(msg.command.position)
        except Exception:
            cmd = np.nan
        t = time.time()
        with self.lock:
            if self.start_time is None:
                self.start_time = t
            rel = t - self.start_time
            self.latest_cmd = cmd
            self.cmd_history.append((rel, cmd))
    def __init__(self, mode='action', hand='right'):
        super().__init__('gripper_multi_mode_analyzer')
        self.mode = mode
        self.hand = hand
        self.joint_name = GRIPPER_JOINTS[hand]
        self.cmd_history = []  # (t, cmd)
        self.state_history = []  # (t, state)
        self.latest_cmd = np.nan
        self.latest_state = np.nan
        self.lock = threading.Lock()
        self.start_time = None
        # Topic/action selection
        if mode == 'action':
            # 訂閱 action goal topic
            from control_msgs.action import GripperCommand
            from rclpy.action import ActionClient
            # 這裡用 topic subscription 監聽 action goal
            self.create_subscription(
                GripperCommand.Goal,
                f'/{hand}_gripper_controller/gripper_cmd/goal',
                self.action_goal_cb, 10)
            def action_goal_cb(self, msg):
                # 取得 action goal position
                try:
                    cmd = float(msg.command.position)
                except Exception:
                    cmd = np.nan
                t = time.time()
                with self.lock:
                    if self.start_time is None:
                        self.start_time = t
                    rel = t - self.start_time
                    self.latest_cmd = cmd
                    self.cmd_history.append((rel, cmd))
        elif mode == 'forward':
            self.create_subscription(Float64MultiArray,
                f'/{hand}_gripper_forward_controller/commands',
                self.forward_cmd_cb, 10)
        elif mode == 'trajectory':
            self.create_subscription(JointTrajectoryControllerState,
                f'/{hand}_gripper_trajectory_controller/controller_state',
                self.controller_state_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 20)
        self.get_logger().info(f'Analyzer initialized: mode={mode}, hand={hand}')

    def controller_state_cb(self, msg):
        try:
            idx = msg.joint_names.index(self.joint_name)
            cmd = msg.reference.positions[idx]
        except Exception:
            cmd = np.nan
        t = time.time()
        with self.lock:
            if self.start_time is None:
                self.start_time = t
            rel = t - self.start_time
            self.latest_cmd = cmd
            self.cmd_history.append((rel, cmd))

    def forward_cmd_cb(self, msg):
        # Only one joint, so take first value
        try:
            cmd = float(msg.data[0])
        except Exception:
            cmd = np.nan
        t = time.time()
        with self.lock:
            if self.start_time is None:
                self.start_time = t
            rel = t - self.start_time
            self.latest_cmd = cmd
            self.cmd_history.append((rel, cmd))

    def joint_state_cb(self, msg):
        try:
            idx = msg.name.index(self.joint_name)
            state = msg.position[idx]
        except Exception:
            state = np.nan
        t = time.time()
        with self.lock:
            if self.start_time is None:
                self.start_time = t
            rel = t - self.start_time
            self.latest_state = state
            self.state_history.append((rel, state))

    def save_and_plot(self, out_prefix=None):
        import csv
        with self.lock:
            times = np.array([t for t, _ in self.cmd_history])
            cmds = np.array([v for _, v in self.cmd_history])
            states = np.array([v for _, v in self.state_history])
            t_states = np.array([t for t, _ in self.state_history])
            interp_states = np.interp(times, t_states, states, left=np.nan, right=np.nan)
            errors = cmds - interp_states
            if out_prefix is None:
                out_prefix = f'gripper_{self.hand}_{self.mode}_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
            out_dir = Path('analy_gripper')
            out_dir.mkdir(exist_ok=True)
            csv_path = out_dir / f'{out_prefix}.csv'
            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['time', 'cmd', 'state', 'error'])
                for t, cmd, state, err in zip(times, cmds, interp_states, errors):
                    writer.writerow([t, cmd, state, err])
            print(f'CSV saved: {csv_path}')
            plt.figure(figsize=(12,6))
            plt.plot(times, cmds, label='Command', linestyle='--', color='r')
            plt.plot(times, interp_states, label='State', linestyle='-', color='b')
            plt.plot(times, errors, label='Error', linestyle=':', color='g')
            plt.xlabel('Time (s)')
            plt.ylabel('Position (m)')
            plt.title(f'Gripper Command vs State ({self.hand}, {self.mode})')
            plt.legend()
            plt.grid(True, alpha=0.3)
            img_path = out_dir / f'{out_prefix}.png'
            plt.savefig(img_path, dpi=150, bbox_inches='tight')
            print(f'Plot saved: {img_path}')
            plt.show()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['action', 'forward', 'trajectory'], default='action')
    parser.add_argument('--hand', choices=['left', 'right'], default='right')
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
        node.save_and_plot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
