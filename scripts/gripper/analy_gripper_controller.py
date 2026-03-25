#!/usr/bin/env python3
"""
Gripper Controller Command vs State Analyzer

- 訂閱 gripper controller command (trajectory or action)
- 訂閱 joint_states
- 比較 position command 與實際 state，計算 error
- 儲存 CSV，繪製 command/state/error 曲線

Usage:
  python3 analy_gripper_controller.py
  python3 analy_gripper_controller.py --csv <your_csv_path>
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np
import matplotlib.pyplot as plt
import threading
import time
import sys
import os
from pathlib import Path
from datetime import datetime

GRIPPER_JOINT = 'openarm_right_finger_joint1'
CONTROLLER_STATE_TOPIC = '/right_gripper_trajectory_controller/controller_state'

class GripperAnalyzer(Node):
    def __init__(self):
        super().__init__('gripper_analyzer')
        self.cmd_history = []  # (t, cmd)
        self.state_history = []  # (t, state)
        self.error_history = []  # (t, error)
        self.latest_cmd = np.nan
        self.latest_state = np.nan
        self.lock = threading.Lock()
        self.start_time = None
        self.create_subscription(JointTrajectoryControllerState, CONTROLLER_STATE_TOPIC, self.controller_state_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 20)
        self.get_logger().info(f'Subscribed to {CONTROLLER_STATE_TOPIC} and /joint_states')

    def controller_state_cb(self, msg):
        # Find gripper joint index
        try:
            idx = msg.joint_names.index(GRIPPER_JOINT)
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
            # Also record error if state available
            if not np.isnan(self.latest_state):
                err = cmd - self.latest_state
                self.error_history.append((rel, err))

    def joint_state_cb(self, msg):
        try:
            idx = msg.name.index(GRIPPER_JOINT)
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
            # Also record error if cmd available
            if not np.isnan(self.latest_cmd):
                err = self.latest_cmd - state
                self.error_history.append((rel, err))

    def save_and_plot(self, out_prefix=None):
        import csv
        with self.lock:
            # Interpolate to common time base
            times = np.array([t for t, _ in self.cmd_history])
            cmds = np.array([v for _, v in self.cmd_history])
            states = np.array([v for _, v in self.state_history])
            t_states = np.array([t for t, _ in self.state_history])
            interp_states = np.interp(times, t_states, states, left=np.nan, right=np.nan)
            errors = cmds - interp_states
            # Save CSV (no pandas)
            if out_prefix is None:
                out_prefix = f'gripper_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
            out_dir = Path('analy_gripper')
            out_dir.mkdir(exist_ok=True)
            csv_path = out_dir / f'{out_prefix}.csv'
            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['time', 'cmd', 'state', 'error'])
                for t, cmd, state, err in zip(times, cmds, interp_states, errors):
                    writer.writerow([t, cmd, state, err])
            print(f'CSV saved: {csv_path}')
            # Plot
            plt.figure(figsize=(12,6))
            plt.plot(times, cmds, label='Command', linestyle='--', color='r')
            plt.plot(times, interp_states, label='State', linestyle='-', color='b')
            plt.plot(times, errors, label='Error', linestyle=':', color='g')
            plt.xlabel('Time (s)')
            plt.ylabel('Position (m)')
            plt.title('Gripper Command vs State')
            plt.legend()
            plt.grid(True, alpha=0.3)
            img_path = out_dir / f'{out_prefix}.png'
            plt.savefig(img_path, dpi=150, bbox_inches='tight')
            print(f'Plot saved: {img_path}')
            plt.show()

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', type=str, default=None, help='If set, only plot from CSV (no ROS)')
    args = parser.parse_args()
    if args.csv:
        # Only plot from CSV (no pandas)
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
    node = GripperAnalyzer()
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
