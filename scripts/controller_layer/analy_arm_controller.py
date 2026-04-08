#!/usr/bin/env python3
"""
Analyzer for OpenArm (left, right, or bimanual) - supports action, topic, and telep modes.

Monitors:
 - Action mode: /right_arm_controller/controller_state (or /left_... or both)
 - Topic mode: /right_forward_position_controller/commands (or /left_... or both)
 - Telep mode: /joint_actions (commands) and /joint_states (states)
 - /joint_states (for actual positions)

Saves CSV and per-joint plots on exit (Ctrl-C).

Usage:
  # Action mode (default)
  python3 scripts/analy_arm_controller_layer_topic.py --mode action
  
  # Topic mode
  python3 scripts/analy_arm_controller_layer_topic.py --mode topic
  
  # Telep mode
  python3 scripts/analy_arm_controller_layer_topic.py --mode telep
  
  # Left arm only
  python3 scripts/analy_arm_controller_layer_topic.py --arm left --mode action
  
  # Both arms (bimanual)
  python3 scripts/analy_arm_controller_layer_topic.py --arm both --mode topic
  
  # Custom options
  python3 scripts/analy_arm_controller_layer_topic.py --arm right --mode action --no-plot --error-scale deg
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np
import argparse
import time
import signal
import sys
import os
import datetime
import threading
import math

# plotting
try:
    import matplotlib
    # Use interactive backend for plt.show() support
    # Try common GUI backends in order of preference
    for backend in ['TkAgg', 'Qt5Agg', 'GTK3Agg', 'WXAgg']:
        try:
            matplotlib.use(backend)
            break
        except Exception:
            continue
    import matplotlib.pyplot as plt
    HAS_MPL = True
except Exception:
    HAS_MPL = False

RIGHT_ARM_JOINTS = [
    'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3',
    'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6',
    'openarm_right_joint7'
]

LEFT_ARM_JOINTS = [
    'openarm_left_joint1', 'openarm_left_joint2', 'openarm_left_joint3',
    'openarm_left_joint4', 'openarm_left_joint5', 'openarm_left_joint6',
    'openarm_left_joint7'
]


class ArmTopicAnalyzer(Node):
    def __init__(self, arm_config, mode='topic', command_type='float_array', joint_names=None, csv_dir='analy_arm', plot=True, error_scale='rad'):
        super().__init__('arm_analyzer')
        self.arm_config = arm_config  # 'right', 'left', or 'both'
        self.mode = mode  # 'action' or 'topic'
        self.command_type = command_type
        self.plot = plot and HAS_MPL
        self.error_scale = error_scale.lower()

        # Build joint names and topics based on arm_config
        if joint_names is not None and len(joint_names) > 0:
            self.joint_names = joint_names
            self.command_topics = []
            self.state_topics = []
        elif arm_config == 'both':
            self.joint_names = RIGHT_ARM_JOINTS + LEFT_ARM_JOINTS
            self.command_topics = [
                '/right_forward_position_controller/commands',
                '/left_forward_position_controller/commands'
            ]
            self.state_topics = [
                '/right_joint_trajectory_controller/controller_state',
                '/left_joint_trajectory_controller/controller_state'
            ]
        elif arm_config == 'left':
            self.joint_names = LEFT_ARM_JOINTS.copy()
            self.command_topics = ['/left_forward_position_controller/commands']
            self.state_topics = ['/left_joint_trajectory_controller/controller_state']
        else:  # default 'right'
            self.joint_names = RIGHT_ARM_JOINTS.copy()
            self.command_topics = ['/right_forward_position_controller/commands']
            self.state_topics = ['/right_joint_trajectory_controller/controller_state']

        self.get_logger().info(f'Arm analyzer: arm={arm_config} mode={mode} command_type={command_type}')
        if mode == 'action':
            self.get_logger().info(f'Controller state topics: {self.state_topics}')
        else:
            self.get_logger().info(f'Command topics: {self.command_topics}')
        self.get_logger().info(f'Tracking joints ({len(self.joint_names)}): {self.joint_names}')

        # Latest command cache (for action mode - to avoid NaN propagation)
        self.latest_cmd = {name: math.nan for name in self.joint_names}
        
        # subscriptions
        self.latest_js = None
        self.latest_joint_actions = None
        self.lock = threading.Lock()
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 20)

        # Subscribe based on mode
        if mode == 'action':
            # Action mode: subscribe to controller_state topics
            for topic in self.state_topics:
                self.create_subscription(JointTrajectoryControllerState, topic,
                                       lambda msg, t=topic: self.controller_state_cb(msg, t), 10)
        elif mode == 'telep':
            # Telep mode: subscribe to /joint_actions for commands
            self.create_subscription(JointState, '/joint_actions', self.joint_actions_cb, 20)
        else:
            # Topic mode: subscribe to command topics
            for topic in self.command_topics:
                if self.command_type == 'float_array':
                    self.create_subscription(Float64MultiArray, topic, 
                                           lambda msg, t=topic: self.cmd_array_cb(msg, t), 10)
                elif self.command_type == 'joint_trajectory':
                    self.create_subscription(JointTrajectory, topic, 
                                           lambda msg, t=topic: self.cmd_traj_cb(msg, t), 10)
                else:
                    raise RuntimeError('Unknown command_type')

        # Track which topic maps to which joints
        if arm_config == 'both':
            self.topic_joint_map = {
                '/right_forward_position_controller/commands': RIGHT_ARM_JOINTS,
                '/left_forward_position_controller/commands': LEFT_ARM_JOINTS,
                '/right_joint_trajectory_controller/controller_state': RIGHT_ARM_JOINTS,
                '/left_joint_trajectory_controller/controller_state': LEFT_ARM_JOINTS
            }
        elif arm_config == 'left':
            self.topic_joint_map = {
                '/left_forward_position_controller/commands': LEFT_ARM_JOINTS,
                '/left_joint_trajectory_controller/controller_state': LEFT_ARM_JOINTS
            }
        else:
            self.topic_joint_map = {
                '/right_forward_position_controller/commands': RIGHT_ARM_JOINTS,
                '/right_joint_trajectory_controller/controller_state': RIGHT_ARM_JOINTS
            }

        # history: per-joint dict of lists (we record all joints at each command event)
        self.history = {name: {'t': [], 'cmd': [], 'state': [], 'err': []} for name in self.joint_names}
        # directories for output
        today = datetime.datetime.now().strftime('%Y%m%d')
        self.base_dir = csv_dir
        self.csv_dir = os.path.join(self.base_dir, 'csv', today)
        self.img_dir = os.path.join(self.base_dir, 'img', today)
        os.makedirs(self.csv_dir, exist_ok=True)
        os.makedirs(self.img_dir, exist_ok=True)
        self.start_time = None
        self.last_summary_time = 0.0  # For rate-limiting summary prints

    def joint_state_cb(self, msg: JointState):
        with self.lock:
            self.latest_js = msg

    def joint_actions_cb(self, msg: JointState):
        """Callback for /joint_actions topic (telep mode)"""
        with self.lock:
            ts = time.time()
            if self.start_time is None:
                self.start_time = ts
            rel = ts - self.start_time
            
            # Build command map from joint_actions
            cmd_map = {n: p for n, p in zip(msg.name, msg.position)}
            
            # Get state from joint_states
            js_map = {}
            if self.latest_js:
                js_map = {n: p for n, p in zip(self.latest_js.name, self.latest_js.position)}
            
            # Record data for all tracked joints
            for name in self.joint_names:
                cmd = cmd_map.get(name, math.nan)
                state = js_map.get(name, math.nan)
                
                # Update latest_cmd
                if not math.isnan(cmd):
                    self.latest_cmd[name] = cmd
                
                # Compute error
                err = math.nan
                if not math.isnan(cmd) and not math.isnan(state):
                    err = cmd - state
                
                self.history[name]['t'].append(rel)
                self.history[name]['cmd'].append(cmd)
                self.history[name]['state'].append(state)
                self.history[name]['err'].append(err)
            
            # Print summary
            self._print_summary(rel)

    def controller_state_cb(self, msg: JointTrajectoryControllerState, topic):
        """Handle controller_state messages (action mode)"""
        with self.lock:
            ts = time.time()
            if self.start_time is None:
                self.start_time = ts
            rel = ts - self.start_time
            
            # Update latest_cmd from reference.positions
            joint_map = {n: i for i, n in enumerate(msg.joint_names)}
            target_joints = self.topic_joint_map.get(topic, self.joint_names)
            
            for name in target_joints:
                idx = joint_map.get(name, None)
                if idx is not None and idx < len(msg.reference.positions):
                    self.latest_cmd[name] = msg.reference.positions[idx]
            
            # Get actual state from joint_states
            js_map = {}
            if self.latest_js:
                js_map = {n: p for n, p in zip(self.latest_js.name, self.latest_js.position)}
            
            # Record data for all tracked joints
            for name in self.joint_names:
                cmd = self.latest_cmd[name]
                state = js_map.get(name, math.nan)
                err = math.nan
                if not math.isnan(cmd) and not math.isnan(state):
                    err = cmd - state
                
                self.history[name]['t'].append(rel)
                self.history[name]['cmd'].append(cmd)
                self.history[name]['state'].append(state)
                self.history[name]['err'].append(err)
            
            # Print summary
            self._print_summary(rel)

    def cmd_array_cb(self, msg: Float64MultiArray, topic):
        positions = list(msg.data)
        # Determine which joints this topic controls
        target_joints = self.topic_joint_map.get(topic, self.joint_names)
        
        # Build command map: only update joints controlled by this topic
        cmd_map = {n: np.nan for n in self.joint_names}
        for i, name in enumerate(target_joints):
            if i < len(positions):
                cmd_map[name] = positions[i]
        
        cmd_positions = [cmd_map[n] for n in self.joint_names]
        self._record(cmd_positions)

    def cmd_traj_cb(self, msg: JointTrajectory, topic):
        if not msg.points:
            return
        point = msg.points[-1]
        # Build command map: only update joints in this trajectory
        cmd_map = {n: np.nan for n in self.joint_names}
        for i, name in enumerate(msg.joint_names):
            if i < len(point.positions) and name in self.joint_names:
                cmd_map[name] = point.positions[i]
        cmd_positions = [cmd_map[n] for n in self.joint_names]
        self._record(cmd_positions)

    def _record(self, cmd_positions):
        with self.lock:
            if self.latest_js is None:
                # can't compute error yet; still record cmd with NaN state
                ts = time.time()
                if self.start_time is None:
                    self.start_time = ts
                rel = ts - self.start_time
                for i, name in enumerate(self.joint_names):
                    cmd = cmd_positions[i]
                    self.history[name]['t'].append(rel)
                    self.history[name]['cmd'].append(cmd)
                    self.history[name]['state'].append(np.nan)
                    self.history[name]['err'].append(np.nan)
                return

            # map joint_states by name
            js_map = {n: p for n, p in zip(self.latest_js.name, self.latest_js.position)}
            ts = self.latest_js.header.stamp.sec + self.latest_js.header.stamp.nanosec * 1e-9
            if self.start_time is None:
                self.start_time = ts
            rel = ts - self.start_time
            for i, name in enumerate(self.joint_names):
                cmd = cmd_positions[i]
                state = js_map.get(name, np.nan)
                err = np.nan
                try:
                    if not np.isnan(state) and not np.isnan(cmd):
                        err = cmd - state
                except Exception:
                    err = np.nan
                self.history[name]['t'].append(rel)
                self.history[name]['cmd'].append(cmd)
                self.history[name]['state'].append(state)
                self.history[name]['err'].append(err)

            # print quick summary
            self._print_summary(rel)

    def _print_summary(self, rel_time):
        # Rate-limit summary prints to every 0.01 seconds to reduce console spam
        if rel_time - self.last_summary_time < 0.01:
            return
        self.last_summary_time = rel_time
        
        # print one-line summary of rms error across tracked joints
        errs = []
        for name in self.joint_names:
            arr = np.array(self.history[name]['err'], dtype=float)
            if arr.size > 0:
                # take last non-nan
                val = arr[~np.isnan(arr)]
                if val.size > 0:
                    errs.append(val[-1])
        if errs:
            arr = np.array(errs)
            mean = np.nanmean(arr)
            rms = np.sqrt(np.nanmean(arr**2))
            self.get_logger().info(f'[t={rel_time:.3f}] mean_err={mean:.4f} rms={rms:.4f}')

    def save_csv_and_plots(self, prefix=None):
        with self.lock:
            timestamp = datetime.datetime.now().strftime('%H%M%S')
            # Create combined CSV where each row is a timestamped sample across all joints
            # Build DataFrame: columns: time, <joint>_cmd, <joint>_state, <joint>_err
            # Determine number of samples from any joint (they are recorded synchronously)
            sample_count = 0
            for name in self.joint_names:
                sample_count = max(sample_count, len(self.history[name]['t']))

            if sample_count == 0:
                self.get_logger().info('No samples recorded; skipping CSV/plot saving')
                return

            # Build dict for DataFrame
            df_dict = {}
            # Use first joint times as the canonical time vector (they should all align)
            times = self.history[self.joint_names[0]]['t']
            df_dict['time'] = times
            for name in self.joint_names:
                data = self.history[name]
                # pad lists to sample_count with NaN if necessary
                def pad(lst):
                    if len(lst) < sample_count:
                        return list(lst) + [np.nan] * (sample_count - len(lst))
                    return list(lst)

                df_dict[f'{name}_cmd'] = pad(data['cmd'])
                df_dict[f'{name}_state'] = pad(data['state'])
                df_dict[f'{name}_err'] = pad(data['err'])

            try:
                # Use the stdlib csv module to avoid heavy optional dependencies (pandas C extensions)
                import csv
                combined_csv_path = os.path.join(self.csv_dir, f"{prefix or self.arm_config}_combined_{timestamp}.csv")
                # Build header and rows from df_dict
                header = ['time']
                for name in self.joint_names:
                    header += [f'{name}_cmd', f'{name}_state', f'{name}_err']

                # Assemble rows (sample_count rows)
                with open(combined_csv_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(header)
                    for i in range(sample_count):
                        row = [df_dict['time'][i]]
                        for name in self.joint_names:
                            row.append(df_dict[f'{name}_cmd'][i])
                            row.append(df_dict[f'{name}_state'][i])
                            row.append(df_dict[f'{name}_err'][i])
                        writer.writerow(row)

                self.get_logger().info(f'Wrote combined CSV: {combined_csv_path}')
            except Exception as ex:
                self.get_logger().error(f'Failed to write combined CSV: {ex}')

            # Plot: single multi-subplot figure (grid)
            if self.plot:
                if not HAS_MPL:
                    self.get_logger().error('Matplotlib not available; cannot plot')
                    return

                # Split left/right joints for bimanual layout
                left_joints = [n for n in self.joint_names if 'left' in n.lower()]
                right_joints = [n for n in self.joint_names if 'right' in n.lower()]
                
                if left_joints and right_joints:
                    # Two-column layout: left joints in left column, right joints in right column
                    max_joints = max(len(left_joints), len(right_joints))
                    fig, axes = plt.subplots(max_joints, 2, figsize=(16, 3.5 * max_joints), squeeze=False)
                    
                    # Plot left arm joints (left column)
                    for idx, name in enumerate(left_joints):
                        data = self.history[name]
                        if not data['t']:
                            continue
                        t = np.array(data['t']) - data['t'][0]
                        cmd = np.array(data['cmd'], dtype=float)
                        state = np.array(data['state'], dtype=float)
                        err = np.array(data['err'], dtype=float)

                        # Optionally convert error scale
                        if self.error_scale == 'deg':
                            err_plot = err * (180.0 / np.pi)
                            cmd_plot = cmd * (180.0 / np.pi) if np.any(~np.isnan(cmd)) else cmd
                            state_plot = state * (180.0 / np.pi) if np.any(~np.isnan(state)) else state
                            ylabel = 'Degrees'
                        else:
                            err_plot = err
                            cmd_plot = cmd
                            state_plot = state
                            ylabel = 'Radians'

                        ax = axes[idx, 0]
                        lines = []
                        # Plot cmd/state on left axis
                        if np.any(~np.isnan(cmd_plot)):
                            l_cmd = ax.plot(t, cmd_plot, label='cmd', linestyle='--', color='r', alpha=0.8)
                            lines += l_cmd
                        if np.any(~np.isnan(state_plot)):
                            l_state = ax.plot(t, state_plot, label='state', linestyle='-', color='b', alpha=0.8)
                            lines += l_state

                        # Plot error on a separate right-hand axis (independent scale)
                        err_lines = []
                        if np.any(~np.isnan(err_plot)):
                            ax_err = ax.twinx()
                            l_err = ax_err.plot(t, err_plot, label='err', linestyle=':', color='g')
                            err_lines += l_err
                            try:
                                ax_err.fill_between(t, np.nan_to_num(err_plot, nan=0.0), 0, color='g', alpha=0.06)
                            except Exception:
                                pass
                            unit = 'deg' if self.error_scale == 'deg' else 'rad'
                            ax_err.set_ylabel(f'Error ({unit})', color='g', fontsize=9)
                            ax_err.tick_params(axis='y', colors='g', labelsize=8)

                        ax.set_title(name, fontweight='bold', fontsize=10)
                        ax.set_xlabel('time (s)', fontsize=9)
                        ax.set_ylabel(ylabel, fontsize=9)
                        ax.tick_params(labelsize=8)
                        ax.grid(True, linestyle=':', alpha=0.6)

                        # Combine legends
                        all_lines = lines + err_lines
                        if all_lines:
                            labels = [ln.get_label() for ln in all_lines]
                            ax.legend(all_lines, labels, fontsize='small', loc='upper right')
                    
                    # Plot right arm joints (right column)
                    for idx, name in enumerate(right_joints):
                        data = self.history[name]
                        if not data['t']:
                            continue
                        t = np.array(data['t']) - data['t'][0]
                        cmd = np.array(data['cmd'], dtype=float)
                        state = np.array(data['state'], dtype=float)
                        err = np.array(data['err'], dtype=float)

                        # Optionally convert error scale
                        if self.error_scale == 'deg':
                            err_plot = err * (180.0 / np.pi)
                            cmd_plot = cmd * (180.0 / np.pi) if np.any(~np.isnan(cmd)) else cmd
                            state_plot = state * (180.0 / np.pi) if np.any(~np.isnan(state)) else state
                            ylabel = 'Degrees'
                        else:
                            err_plot = err
                            cmd_plot = cmd
                            state_plot = state
                            ylabel = 'Radians'

                        ax = axes[idx, 1]
                        lines = []
                        # Plot cmd/state on left axis
                        if np.any(~np.isnan(cmd_plot)):
                            l_cmd = ax.plot(t, cmd_plot, label='cmd', linestyle='--', color='r', alpha=0.8)
                            lines += l_cmd
                        if np.any(~np.isnan(state_plot)):
                            l_state = ax.plot(t, state_plot, label='state', linestyle='-', color='b', alpha=0.8)
                            lines += l_state

                        # Plot error on a separate right-hand axis (independent scale)
                        err_lines = []
                        if np.any(~np.isnan(err_plot)):
                            ax_err = ax.twinx()
                            l_err = ax_err.plot(t, err_plot, label='err', linestyle=':', color='g')
                            err_lines += l_err
                            try:
                                ax_err.fill_between(t, np.nan_to_num(err_plot, nan=0.0), 0, color='g', alpha=0.06)
                            except Exception:
                                pass
                            unit = 'deg' if self.error_scale == 'deg' else 'rad'
                            ax_err.set_ylabel(f'Error ({unit})', color='g', fontsize=9)
                            ax_err.tick_params(axis='y', colors='g', labelsize=8)

                        ax.set_title(name, fontweight='bold', fontsize=10)
                        ax.set_xlabel('time (s)', fontsize=9)
                        ax.set_ylabel(ylabel, fontsize=9)
                        ax.tick_params(labelsize=8)
                        ax.grid(True, linestyle=':', alpha=0.6)

                        # Combine legends
                        all_lines = lines + err_lines
                        if all_lines:
                            labels = [ln.get_label() for ln in all_lines]
                            ax.legend(all_lines, labels, fontsize='small', loc='upper right')
                    
                    # Hide empty subplots
                    for i in range(len(left_joints), max_joints):
                        fig.delaxes(axes[i, 0])
                    for i in range(len(right_joints), max_joints):
                        fig.delaxes(axes[i, 1])
                    
                else:
                    # Single arm mode: standard grid layout
                    n = len(self.joint_names)
                    cols = 2
                    rows = (n + cols - 1) // cols
                    fig, axes = plt.subplots(rows, cols, figsize=(12, 3 * rows), squeeze=False)
                    axes_flat = axes.flatten()

                    for idx, name in enumerate(self.joint_names):
                        data = self.history[name]
                        if not data['t']:
                            continue
                        t = np.array(data['t']) - data['t'][0]
                        cmd = np.array(data['cmd'], dtype=float)
                        state = np.array(data['state'], dtype=float)
                        err = np.array(data['err'], dtype=float)

                        # Optionally convert error scale
                        if self.error_scale == 'deg':
                            err_plot = err * (180.0 / np.pi)
                            cmd_plot = cmd * (180.0 / np.pi) if np.any(~np.isnan(cmd)) else cmd
                            state_plot = state * (180.0 / np.pi) if np.any(~np.isnan(state)) else state
                            ylabel = 'Degrees'
                        else:
                            err_plot = err
                            cmd_plot = cmd
                            state_plot = state
                            ylabel = 'Radians'

                        ax = axes_flat[idx]
                        lines = []
                        # Plot cmd/state on left axis
                        if np.any(~np.isnan(cmd_plot)):
                            l_cmd = ax.plot(t, cmd_plot, label='cmd', linestyle='--', color='r')
                            lines += l_cmd
                        if np.any(~np.isnan(state_plot)):
                            l_state = ax.plot(t, state_plot, label='state', linestyle='-', color='b')
                            lines += l_state

                        # Plot error on a separate right-hand axis (independent scale)
                        err_lines = []
                        if np.any(~np.isnan(err_plot)):
                            ax_err = ax.twinx()
                            l_err = ax_err.plot(t, err_plot, label='err', linestyle=':', color='g')
                            err_lines += l_err
                            try:
                                ax_err.fill_between(t, np.nan_to_num(err_plot, nan=0.0), 0, color='g', alpha=0.06)
                            except Exception:
                                pass
                            unit = 'deg' if self.error_scale == 'deg' else 'rad'
                            ax_err.set_ylabel(f'Error ({unit})', color='g')
                            ax_err.tick_params(axis='y', colors='g')

                        ax.set_title(name)
                        ax.set_xlabel('time (s)')
                        ax.set_ylabel(ylabel)
                        ax.grid(True, linestyle=':', alpha=0.6)

                        # Combine legends from both axes if needed
                        all_lines = lines + err_lines
                        if all_lines:
                            labels = [ln.get_label() for ln in all_lines]
                            ax.legend(all_lines, labels, fontsize='small')

                    # Hide unused subplots
                    for j in range(len(self.joint_names), len(axes_flat)):
                        fig.delaxes(axes_flat[j])

                plt.suptitle(f'{self.arm_config.capitalize()} Arm Commands vs States - {timestamp}', fontsize=14)
                plt.tight_layout(rect=[0, 0.03, 1, 0.95])
                out_png = os.path.join(self.img_dir, f"{prefix or self.arm_config}_combined_{timestamp}.png")
                try:
                    plt.savefig(out_png, dpi=150)
                    self.get_logger().info(f'Saved combined plot: {out_png}')
                except Exception as ex:
                    self.get_logger().error(f'Failed to save combined plot: {ex}')
                
                # Show plot before exit
                self.get_logger().info('Displaying plot window (close window or Ctrl+C to exit)...')
                try:
                    plt.show(block=True)
                except Exception as e:
                    self.get_logger().warn(f'plt.show() failed: {e}')


def main(argv=None):
    rclpy.init(args=argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('--arm', choices=['right', 'left', 'both'], default='right',
                        help='Which arm(s) to analyze: right, left, or both')
    parser.add_argument('--mode', choices=['action', 'topic', 'telep'], default='action',
                        help='Data source mode: action (controller_state), topic (external commands), or telep (joint_actions)')
    parser.add_argument('--command-type', choices=['float_array','joint_trajectory'], default='float_array',
                        help='Command message type for topic mode (ignored in action mode)')
    parser.add_argument('--joint-names', nargs='*', default=None,
                        help='Override joint names (space separated)')
    parser.add_argument('--csv-dir', default='analy_arm')
    parser.add_argument('--plot', dest='plot', action='store_true', default=True,
                        help='Generate plots (default: enabled)')
    parser.add_argument('--no-plot', dest='plot', action='store_false',
                        help='Disable plot generation')
    parser.add_argument('--out-prefix', default=None,
                        help='Output file prefix (default: based on --arm)')
    parser.add_argument('--error-scale', choices=['rad','deg'], default='rad',
                        help='Scale to plot errors in (radians or degrees)')
    args = parser.parse_args()

    # Default prefix based on arm config
    if args.out_prefix is None:
        args.out_prefix = args.arm + '_arm' if args.arm != 'both' else 'bimanual'

    node = ArmTopicAnalyzer(args.arm, mode=args.mode, command_type=args.command_type, 
                           joint_names=args.joint_names, csv_dir=args.csv_dir, 
                           plot=args.plot, error_scale=args.error_scale)

    saved = {'done': False, 'requested': False}  # Track save state and shutdown request

    def sigint_handler(signum, frame):
        # First Ctrl-C: request shutdown and let main thread save/cleanup.
        # Second Ctrl-C: force exit immediately.
        if saved['requested']:
            try:
                sys.exit(0)
            except Exception:
                os._exit(0)

        saved['requested'] = True
        node.get_logger().info('SIGINT received, shutting down...')
        try:
            if not rclpy.is_shutdown():
                rclpy.shutdown()
        except Exception:
            pass

    signal.signal(signal.SIGINT, sigint_handler)

    try:
        node.get_logger().info('Entering spin loop (use Ctrl-C to exit)')
        # Use spin_once in a short loop so SIGINT is handled promptly even
        # if underlying spin would block. This avoids hanging when subscribed
        # topics are not present.
        while rclpy.ok() and not saved['requested']:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if not saved['done']:  # Only save if not already saved in signal handler
            node.get_logger().info('Shutting down analyzer...')
            try:
                node.save_csv_and_plots(prefix=args.out_prefix)
            except Exception as e:
                node.get_logger().error(f'Error saving on exit: {e}')
            finally:
                saved['done'] = True
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if not rclpy.is_shutdown():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
