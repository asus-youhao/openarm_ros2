#!/usr/bin/env python3
"""
O6 Hand Analyzer

Analyze O6 hand (left/right/both) command vs joint_states and compute
error and velocity metrics.

Usage (interactive mode): run the script and follow the prompts.
"""

import argparse
import threading
import time
import math
import os
import sys
from pathlib import Path
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

try:
    import matplotlib
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

LEFT_JOINTS = [
    'L_thumb_cmc_yaw', 'L_thumb_cmc_pitch',
    'L_index_mcp_pitch', 'L_middle_mcp_pitch',
    'L_ring_mcp_pitch', 'L_pinky_mcp_pitch'
]
RIGHT_JOINTS = [
    'R_thumb_cmc_yaw', 'R_thumb_cmc_pitch',
    'R_index_mcp_pitch', 'R_middle_mcp_pitch',
    'R_ring_mcp_pitch', 'R_pinky_mcp_pitch'
]

class O6HandAnalyzer(Node):
    def __init__(self, hand, mode, csv_dir='analy_o6_hand', plot=True):
        super().__init__('o6_hand_analyzer')
        self.hand = hand
        self.mode = mode
        self.plot = plot and HAS_MPL

        # Initialize attributes (before creating subscriptions)
        if hand == 'both':
            self.joint_names = LEFT_JOINTS + RIGHT_JOINTS
            self.command_topics = [
                '/left_o6_hand_forward_position_controller/commands',
                '/right_o6_hand_forward_position_controller/commands'
            ]
            self.state_topics = [
                '/left_o6_hand_controller/controller_state',
                '/right_o6_hand_controller/controller_state'
            ]
        elif hand == 'left':
            self.joint_names = LEFT_JOINTS
            self.command_topics = ['/left_o6_hand_controller/commands']
            self.state_topics = ['/left_o6_hand_controller/controller_state']
        else:
            self.joint_names = RIGHT_JOINTS
            self.command_topics = ['/right_o6_hand_controller/commands']
            self.state_topics = ['/right_o6_hand_controller/controller_state']

        self.history = {name: {'t': [], 'cmd': [], 'state': [], 'err': [], 'vel': []} for name in self.joint_names}
        self.latest_cmd = {name: math.nan for name in self.joint_names}  # record last valid cmd
        today = time.strftime('%Y%m%d')
        self.csv_dir = os.path.join(csv_dir, 'csv', today)
        self.img_dir = os.path.join(csv_dir, 'img', today)
        os.makedirs(self.csv_dir, exist_ok=True)
        os.makedirs(self.img_dir, exist_ok=True)
        self.start_time = None

        self.get_logger().info(f'O6 hand analyzer: hand={hand} mode={mode}')
        self.get_logger().info(f'Joint names: {self.joint_names}')

        self.latest_js = None
        self.lock = threading.Lock()
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 20)

        if mode == 'topic':
            for topic in self.command_topics:
                self.create_subscription(Float64MultiArray, topic,
                                         lambda msg, t=topic: self.cmd_array_cb(msg, t), 10)
        elif mode == 'action':
            from control_msgs.msg import JointTrajectoryControllerState
            for topic in self.state_topics:
                self.create_subscription(JointTrajectoryControllerState, topic,
                                         lambda msg, t=topic: self.controller_state_cb(msg, t), 10)
        else:
            raise RuntimeError('Unknown mode')

        self.topic_joint_map = {}
        if hand == 'both':
            self.topic_joint_map = {
                '/left_o6_hand_controller/commands': LEFT_JOINTS,
                '/right_o6_hand_controller/commands': RIGHT_JOINTS,
                '/left_o6_hand_controller/follow_joint_trajectory': LEFT_JOINTS,
                '/right_o6_hand_controller/follow_joint_trajectory': RIGHT_JOINTS
            }
        elif hand == 'left':
            self.topic_joint_map = {
                '/left_o6_hand_controller/commands': LEFT_JOINTS,
                '/left_o6_hand_controller/follow_joint_trajectory': LEFT_JOINTS
            }
        else:
            self.topic_joint_map = {
                '/right_o6_hand_controller/commands': RIGHT_JOINTS,
                '/right_o6_hand_controller/follow_joint_trajectory': RIGHT_JOINTS
            }

    def controller_state_cb(self, msg, topic):
        # msg: control_msgs/msg/JointTrajectoryControllerState
        # Use reference.positions as cmd, and joint_states as state
        with self.lock:
            ts = time.time()
            if self.start_time is None:
                self.start_time = ts
            rel = ts - self.start_time
            
                # Update cmd from controller_state (use reference.positions)
            joint_map = {n: i for i, n in enumerate(msg.joint_names)}
            
            # Update latest_cmd (only when a new reference is present)
            for name in self.joint_names:
                idx = joint_map.get(name, None)
                if idx is not None and idx < len(msg.joint_names):
                    if idx < len(msg.reference.positions):
                        new_cmd = msg.reference.positions[idx]
                        if not math.isnan(new_cmd):
                            self.latest_cmd[name] = new_cmd
            
            # Get state from joint_states
            js_map = {}
            if self.latest_js:
                js_map = {n: p for n, p in zip(self.latest_js.name, self.latest_js.position)}
            
            # Record data for all joints
            for name in self.joint_names:
                # Use last valid cmd
                cmd = self.latest_cmd[name]
                
                # state obtained from joint_states
                state = js_map.get(name, math.nan)
                
                # compute error
                err = math.nan
                if not math.isnan(cmd) and not math.isnan(state):
                    err = cmd - state
                
                # compute velocity
                vel = math.nan
                hist = self.history[name]
                if hist['state'] and len(hist['state']) > 0:
                    prev_state = hist['state'][-1]
                    prev_t = hist['t'][-1]
                    if not math.isnan(prev_state) and not math.isnan(state):
                        dt = rel - prev_t
                        if dt > 0:
                            vel = (state - prev_state) / dt
                
                hist['t'].append(rel)
                hist['cmd'].append(cmd)
                hist['state'].append(state)
                hist['err'].append(err)
                hist['vel'].append(vel)

    def joint_state_cb(self, msg: JointState):
        with self.lock:
            self.latest_js = msg

    def cmd_array_cb(self, msg: Float64MultiArray, topic):
        positions = list(msg.data)
        target_joints = self.topic_joint_map.get(topic, self.joint_names)
        cmd_map = {n: math.nan for n in self.joint_names}
        for i, name in enumerate(target_joints):
            if i < len(positions):
                cmd_map[name] = positions[i]
        cmd_positions = [cmd_map[n] for n in self.joint_names]
        self._record(cmd_positions)

    def cmd_traj_cb(self, msg: JointTrajectory, topic):
        if not msg.points:
            return
        point = msg.points[-1]
        cmd_map = {n: math.nan for n in self.joint_names}
        for i, name in enumerate(msg.joint_names):
            if i < len(point.positions) and name in self.joint_names:
                cmd_map[name] = point.positions[i]
        cmd_positions = [cmd_map[n] for n in self.joint_names]
        self._record(cmd_positions)

    def _record(self, cmd_positions):
        with self.lock:
            ts = time.time()
            if self.start_time is None:
                self.start_time = ts
            rel = ts - self.start_time
            js_map = {}
            if self.latest_js:
                js_map = {n: p for n, p in zip(self.latest_js.name, self.latest_js.position)}
            for i, name in enumerate(self.joint_names):
                cmd = cmd_positions[i]
                state = js_map.get(name, math.nan)
                err = math.nan
                if not math.isnan(state) and not math.isnan(cmd):
                    err = cmd - state
                vel = math.nan
                hist = self.history[name]
                if hist['state']:
                    prev_state = hist['state'][-1]
                    prev_t = hist['t'][-1]
                    if not math.isnan(prev_state) and not math.isnan(state):
                        vel = (state - prev_state) / (rel - prev_t) if rel - prev_t > 0 else math.nan
                hist['t'].append(rel)
                hist['cmd'].append(cmd)
                hist['state'].append(state)
                hist['err'].append(err)
                hist['vel'].append(vel)

    def save_csv_and_plot(self):
        timestamp = time.strftime('%H%M%S')
        sample_count = max(len(self.history[n]['t']) for n in self.joint_names)
        if sample_count == 0:
            self.get_logger().info('No samples recorded; skipping CSV/plot saving')
            return
        df_dict = {}
        times = self.history[self.joint_names[0]]['t']
        df_dict['time'] = times
        
        # Calculate performance metrics
        metrics = {}
        for name in self.joint_names:
            data = self.history[name]
            metrics[name] = self._calculate_metrics(
                np.array(data['t'], dtype=float),
                np.array(data['cmd'], dtype=float),
                np.array(data['state'], dtype=float),
                np.array(data['err'], dtype=float)
            )
        
        for name in self.joint_names:
            data = self.history[name]
            def pad(lst):
                return list(lst) + [math.nan] * (sample_count - len(lst)) if len(lst) < sample_count else list(lst)
            df_dict[f'{name}_cmd'] = pad(data['cmd'])
            df_dict[f'{name}_state'] = pad(data['state'])
            df_dict[f'{name}_err'] = pad(data['err'])
            df_dict[f'{name}_vel'] = pad(data['vel'])
        combined_csv_path = os.path.join(self.csv_dir, f"{self.hand}_combined_{timestamp}.csv")
        header = ['time']
        for name in self.joint_names:
            header += [f'{name}_cmd', f'{name}_state', f'{name}_err', f'{name}_vel']
        with open(combined_csv_path, 'w') as f:
            f.write(','.join(header) + '\n')
            for i in range(sample_count):
                row = [df_dict['time'][i]]
                for name in self.joint_names:
                    row += [df_dict[f'{name}_cmd'][i], df_dict[f'{name}_state'][i], df_dict[f'{name}_err'][i], df_dict[f'{name}_vel'][i]]
                f.write(','.join(str(x) for x in row) + '\n')
        self.get_logger().info(f'Wrote combined CSV: {combined_csv_path}')
        
        # Save performance metrics CSV
        metrics_csv_path = os.path.join(self.csv_dir, f"{self.hand}_metrics_{timestamp}.csv")
        with open(metrics_csv_path, 'w') as f:
            f.write('joint,rise_time_ms,settling_time_ms,steady_error_rad,overshoot_percent,oscillation_amplitude_rad\n')
            for name in self.joint_names:
                m = metrics[name]
                f.write(f"{name},{m['rise_time']:.2f},{m['settling_time']:.2f},{m['steady_error']:.6f},{m['overshoot']:.2f},{m['oscillation']:.6f}\n")
        self.get_logger().info(f'Wrote metrics CSV: {metrics_csv_path}')
        
        if self.plot:
            self.plot_combined(df_dict, metrics, combined_csv_path)
    
    def _calculate_metrics(self, t, cmd, state, err):
        """Compute control system performance metrics"""
        metrics = {
            'rise_time': math.nan,
            'settling_time': math.nan,
            'steady_error': math.nan,
            'overshoot': math.nan,
            'oscillation': math.nan
        }
        
        # Filter valid data
        valid_mask = ~np.isnan(cmd) & ~np.isnan(state) & ~np.isnan(err)
        if not np.any(valid_mask) or len(t[valid_mask]) < 10:
            return metrics
        
        t_valid = t[valid_mask]
        cmd_valid = cmd[valid_mask]
        state_valid = state[valid_mask]
        err_valid = err[valid_mask]
        
        # Detect command changes (step response)
        cmd_diff = np.abs(np.diff(cmd_valid, prepend=cmd_valid[0]))
        step_threshold = 0.05  # step threshold
        step_indices = np.where(cmd_diff > step_threshold)[0]
        
        if len(step_indices) == 0:
            # No step detected — compute overall steady-state error
            metrics['steady_error'] = np.mean(err_valid)
            metrics['oscillation'] = np.std(err_valid)
            return metrics
        
        # Analyze the first detected step response
        step_idx = step_indices[0]
        if step_idx >= len(cmd_valid) - 5:
            return metrics
        
        initial_val = state_valid[step_idx]
        target_val = cmd_valid[step_idx + 1] if step_idx + 1 < len(cmd_valid) else cmd_valid[step_idx]
        step_size = target_val - initial_val
        
        if abs(step_size) < 0.01:  # step too small
            return metrics
        
        # Analyze the response after the step
        response_t = t_valid[step_idx:]
        response_state = state_valid[step_idx:]
        response_cmd = cmd_valid[step_idx:]
        response_err = err_valid[step_idx:]
        
        if len(response_t) < 5:
            return metrics
        
        # Rise Time: 10% to 90% of step
        val_10 = initial_val + 0.1 * step_size
        val_90 = initial_val + 0.9 * step_size
        
        if step_size > 0:
            idx_10 = np.where(response_state >= val_10)[0]
            idx_90 = np.where(response_state >= val_90)[0]
        else:
            idx_10 = np.where(response_state <= val_10)[0]
            idx_90 = np.where(response_state <= val_90)[0]
        
        if len(idx_10) > 0 and len(idx_90) > 0:
            t_10 = response_t[idx_10[0]]
            t_90 = response_t[idx_90[0]]
            metrics['rise_time'] = (t_90 - t_10) * 1000  # ms
        
        # Settling Time: enter and remain within ±2% band
        settling_threshold = 0.02 * abs(step_size)
        settled_mask = np.abs(response_state - target_val) <= settling_threshold
        
        # Find the last time the response left the settling band
        if np.any(settled_mask):
            # Search backwards to find the last non-settled index
            unsettled_indices = np.where(~settled_mask)[0]
            if len(unsettled_indices) > 0:
                last_unsettled = unsettled_indices[-1]
                if last_unsettled + 1 < len(response_t):
                    settling_idx = last_unsettled + 1
                    metrics['settling_time'] = (response_t[settling_idx] - response_t[0]) * 1000  # ms
            else:
                # Entire response is settled
                metrics['settling_time'] = 0.0
        
        # Steady-State Error: compute using the last 30% of the response
        steady_start_idx = int(len(response_err) * 0.7)
        if steady_start_idx < len(response_err):
            steady_err = response_err[steady_start_idx:]
            metrics['steady_error'] = np.mean(steady_err)
            metrics['oscillation'] = np.std(steady_err)
        
        # Overshoot: maximum percent above the target value
        if step_size > 0:
            overshoot_vals = response_state - target_val
            max_overshoot = np.max(overshoot_vals)
            if max_overshoot > 0:
                metrics['overshoot'] = (max_overshoot / abs(step_size)) * 100
        else:
            overshoot_vals = target_val - response_state
            max_overshoot = np.max(overshoot_vals)
            if max_overshoot > 0:
                metrics['overshoot'] = (max_overshoot / abs(step_size)) * 100
        
        return metrics

    def plot_combined(self, df_dict, metrics, csv_path):
        if not HAS_MPL:
            self.get_logger().info('Matplotlib not available; cannot plot')
            return
        
        # Split left/right joints: left in left column, right in right column
        left_joints = [n for n in self.joint_names if n.startswith('L_')]
        right_joints = [n for n in self.joint_names if n.startswith('R_')]
        
        max_joints = max(len(left_joints), len(right_joints)) if left_joints or right_joints else len(self.joint_names)
        
        if left_joints or right_joints:
            # Two-column layout: left in left column, right in right column
            fig, axes = plt.subplots(max_joints, 2, figsize=(16, 3.5 * max_joints), squeeze=False)
            t = np.array(df_dict['time'], dtype=float)
            
            # Plot left hand
            for idx, name in enumerate(left_joints):
                ax = axes[idx, 0]
                cmd = np.array(df_dict[f'{name}_cmd'], dtype=float)
                state = np.array(df_dict[f'{name}_state'], dtype=float)
                err = np.array(df_dict[f'{name}_err'], dtype=float)
                vel = np.array(df_dict[f'{name}_vel'], dtype=float)
                m = metrics[name]
                
                # Primary axis: cmd & state (rad)
                l1 = ax.plot(t, cmd, label='cmd', linestyle='--', color='r', alpha=0.8)
                l2 = ax.plot(t, state, label='state', linestyle='-', color='b', alpha=0.8)
                ax.set_ylabel('Position (rad)', color='k')
                ax.tick_params(axis='y', labelcolor='k')
                ax.set_xlabel('time (s)')
                ax.set_title(name, fontweight='bold', fontsize=10)
                ax.grid(True, linestyle=':', alpha=0.5)
                
                # Display performance metrics on the plot (shown in legend)
                metrics_text = []
                if not math.isnan(m['rise_time']) and m['rise_time'] > 0:
                    metrics_text.append(f"Rise Time: {m['rise_time']:.1f}ms")
                if not math.isnan(m['settling_time']) and m['settling_time'] > 0:
                    metrics_text.append(f"Settling Time: {m['settling_time']:.1f}ms")
                if not math.isnan(m['steady_error']):
                    metrics_text.append(f"Steady Error: {m['steady_error']:.4f}rad")
                if not math.isnan(m['overshoot']) and m['overshoot'] > 0:
                    metrics_text.append(f"Overshoot: {m['overshoot']:.1f}%")
                if metrics_text:
                    metrics_str = '\n'.join(metrics_text)
                    ax.text(0.02, 0.98, metrics_str, transform=ax.transAxes,
                           fontsize=8, verticalalignment='top',
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
                
                # Right axis 1: error (rad)
                ax_err = ax.twinx()
                l3 = ax_err.plot(t, err, label='error', linestyle=':', color='g', alpha=0.8)
                ax_err.set_ylabel('Error (rad)', color='g')
                ax_err.tick_params(axis='y', labelcolor='g')
                ax_err.spines['right'].set_position(('outward', 0))
                
                # Right axis 2: velocity (rad/s)
                ax_vel = ax.twinx()
                l4 = ax_vel.plot(t, vel, label='vel', linestyle='-.', color='m', alpha=0.8)
                ax_vel.set_ylabel('Velocity (rad/s)', color='m')
                ax_vel.tick_params(axis='y', labelcolor='m')
                ax_vel.spines['right'].set_position(('outward', 60))
                
                # Combine legend, include performance metrics
                lines = l1 + l2 + l3 + l4
                labels = [l.get_label() for l in lines]
                
                # Add performance metric labels to legend
                metrics_labels = []
                if not math.isnan(m['rise_time']) and m['rise_time'] > 0:
                    metrics_labels.append(f"RT: {m['rise_time']:.1f}ms")
                if not math.isnan(m['settling_time']) and m['settling_time'] > 0:
                    metrics_labels.append(f"ST: {m['settling_time']:.1f}ms")
                if not math.isnan(m['steady_error']):
                    metrics_labels.append(f"Err: {m['steady_error']:.4f}rad")
                if not math.isnan(m['overshoot']) and m['overshoot'] > 0:
                    metrics_labels.append(f"OS: {m['overshoot']:.1f}%")
                
                if metrics_labels:
                    labels.extend(metrics_labels)
                    # Create empty lines for metrics (legend-only)
                    for _ in metrics_labels:
                        lines.append(ax.plot([], [], ' ')[0])
                
                ax.legend(lines, labels, loc='upper right', fontsize='x-small')
            
            # Plot right hand
            for idx, name in enumerate(right_joints):
                ax = axes[idx, 1]
                cmd = np.array(df_dict[f'{name}_cmd'], dtype=float)
                state = np.array(df_dict[f'{name}_state'], dtype=float)
                err = np.array(df_dict[f'{name}_err'], dtype=float)
                vel = np.array(df_dict[f'{name}_vel'], dtype=float)
                m = metrics[name]
                
                # Primary axis: cmd & state (rad)
                l1 = ax.plot(t, cmd, label='cmd', linestyle='--', color='r', alpha=0.8)
                l2 = ax.plot(t, state, label='state', linestyle='-', color='b', alpha=0.8)
                ax.set_ylabel('Position (rad)', color='k')
                ax.tick_params(axis='y', labelcolor='k')
                ax.set_xlabel('time (s)')
                ax.set_title(name, fontweight='bold', fontsize=10)
                ax.grid(True, linestyle=':', alpha=0.5)
                
                # Display performance metrics on the plot (shown in legend)
                metrics_text = []
                if not math.isnan(m['rise_time']) and m['rise_time'] > 0:
                    metrics_text.append(f"Rise Time: {m['rise_time']:.1f}ms")
                if not math.isnan(m['settling_time']) and m['settling_time'] > 0:
                    metrics_text.append(f"Settling Time: {m['settling_time']:.1f}ms")
                if not math.isnan(m['steady_error']):
                    metrics_text.append(f"Steady Error: {m['steady_error']:.4f}rad")
                if not math.isnan(m['overshoot']) and m['overshoot'] > 0:
                    metrics_text.append(f"Overshoot: {m['overshoot']:.1f}%")
                if metrics_text:
                    metrics_str = '\n'.join(metrics_text)
                    ax.text(0.02, 0.98, metrics_str, transform=ax.transAxes,
                           fontsize=8, verticalalignment='top',
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
                
                # Right axis 1: error (rad)
                ax_err = ax.twinx()
                l3 = ax_err.plot(t, err, label='error', linestyle=':', color='g', alpha=0.8)
                ax_err.set_ylabel('Error (rad)', color='g')
                ax_err.tick_params(axis='y', labelcolor='g')
                ax_err.spines['right'].set_position(('outward', 0))
                
                # Right axis 2: velocity (rad/s)
                ax_vel = ax.twinx()
                l4 = ax_vel.plot(t, vel, label='vel', linestyle='-.', color='m', alpha=0.8)
                ax_vel.set_ylabel('Velocity (rad/s)', color='m')
                ax_vel.tick_params(axis='y', labelcolor='m')
                ax_vel.spines['right'].set_position(('outward', 60))
                
                # Combine legend, include performance metrics
                lines = l1 + l2 + l3 + l4
                labels = [l.get_label() for l in lines]
                
                # Add performance metric labels to legend
                metrics_labels = []
                if not math.isnan(m['rise_time']) and m['rise_time'] > 0:
                    metrics_labels.append(f"RT: {m['rise_time']:.1f}ms")
                if not math.isnan(m['settling_time']) and m['settling_time'] > 0:
                    metrics_labels.append(f"ST: {m['settling_time']:.1f}ms")
                if not math.isnan(m['steady_error']):
                    metrics_labels.append(f"Err: {m['steady_error']:.4f}rad")
                if not math.isnan(m['overshoot']) and m['overshoot'] > 0:
                    metrics_labels.append(f"OS: {m['overshoot']:.1f}%")
                
                if metrics_labels:
                    labels.extend(metrics_labels)
                    # Create empty lines for metrics (legend-only)
                    for _ in metrics_labels:
                        lines.append(ax.plot([], [], ' ')[0])
                
                ax.legend(lines, labels, loc='upper right', fontsize='x-small')
            
            # Hide empty subplots
            for i in range(len(left_joints), max_joints):
                fig.delaxes(axes[i, 0])
            for i in range(len(right_joints), max_joints):
                fig.delaxes(axes[i, 1])
        else:
            # Single-hand mode (no L_/R_ prefix)
            n = len(self.joint_names)
            cols = 2
            rows = (n + cols - 1) // cols
            fig, axes = plt.subplots(rows, cols, figsize=(16, 3.5 * rows), squeeze=False)
            axes_flat = axes.flatten()
            t = np.array(df_dict['time'], dtype=float)
            
            for idx, name in enumerate(self.joint_names):
                ax = axes_flat[idx]
                cmd = np.array(df_dict[f'{name}_cmd'], dtype=float)
                state = np.array(df_dict[f'{name}_state'], dtype=float)
                err = np.array(df_dict[f'{name}_err'], dtype=float)
                vel = np.array(df_dict[f'{name}_vel'], dtype=float)
                m = metrics[name]
                
                # Primary axis: cmd & state
                l1 = ax.plot(t, cmd, label='cmd', linestyle='--', color='r', alpha=0.8)
                l2 = ax.plot(t, state, label='state', linestyle='-', color='b', alpha=0.8)
                ax.set_ylabel('Position (rad)', color='k')
                ax.set_xlabel('time (s)')
                ax.set_title(name, fontweight='bold', fontsize=10)
                ax.grid(True, linestyle=':', alpha=0.5)
                
                # Right axis 1: error
                ax_err = ax.twinx()
                l3 = ax_err.plot(t, err, label='error', linestyle=':', color='g', alpha=0.8)
                ax_err.set_ylabel('Error (rad)', color='g')
                ax_err.tick_params(axis='y', labelcolor='g')
                
                # Right axis 2: velocity
                ax_vel = ax.twinx()
                l4 = ax_vel.plot(t, vel, label='vel', linestyle='-.', color='m', alpha=0.8)
                ax_vel.set_ylabel('Velocity (rad/s)', color='m')
                ax_vel.tick_params(axis='y', labelcolor='m')
                ax_vel.spines['right'].set_position(('outward', 60))
                
                # Combine legend, include performance metrics
                lines = l1 + l2 + l3 + l4
                labels = [l.get_label() for l in lines]
                
                # Add performance metric labels to legend
                metrics_labels = []
                if not math.isnan(m['rise_time']) and m['rise_time'] > 0:
                    metrics_labels.append(f"RT: {m['rise_time']:.1f}ms")
                if not math.isnan(m['settling_time']) and m['settling_time'] > 0:
                    metrics_labels.append(f"ST: {m['settling_time']:.1f}ms")
                if not math.isnan(m['steady_error']):
                    metrics_labels.append(f"Err: {m['steady_error']:.4f}rad")
                if not math.isnan(m['overshoot']) and m['overshoot'] > 0:
                    metrics_labels.append(f"OS: {m['overshoot']:.1f}%")
                
                labels.extend(metrics_labels)
                # Create empty lines for metrics (legend-only)
                for _ in metrics_labels:
                    lines.append(ax.plot([], [], ' ')[0])
                
                ax.legend(lines, labels, loc='upper right', fontsize='x-small')
            
            for j in range(len(self.joint_names), len(axes_flat)):
                fig.delaxes(axes_flat[j])
        
        plt.suptitle(f'O6 Hand Performance Analysis ({self.hand})', fontsize=14)
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        out_path = os.path.join(self.img_dir, f"{self.hand}_analysis_{Path(csv_path).stem}.png")
        plt.savefig(out_path, dpi=150)
        self.get_logger().info(f'Saved analysis plot: {out_path}')
        # Show plot before exit
        self.get_logger().info('Displaying plot window (close window or Ctrl+C to exit)...')
        try:
            plt.show(block=True)
        except Exception as e:
            self.get_logger().warn(f'plt.show() failed: {e}')


def main():
    # If command-line arguments provided, use them; otherwise fall back to interactive input
    if len(sys.argv) > 1:
        parser = argparse.ArgumentParser(description='O6 Hand Analyzer')
        parser.add_argument('--hand', choices=['left', 'right', 'both'], default='right')
        parser.add_argument('--hand-id', type=int, choices=[0, 1, 2],
                            help='Numeric hand choice: 0=left, 1=right, 2=both (overrides --hand)')
        parser.add_argument('--mode', choices=['topic', 'action'], default='topic')
        parser.add_argument('--mode-id', type=int, choices=[0, 1],
                            help='Numeric mode choice: 0=topic, 1=action (overrides --mode)')
        parser.add_argument('--csv-dir', default='analy_o6_hand')
        parser.add_argument('--no-plot', dest='plot', action='store_false')
        args = parser.parse_args()

        # map numeric choices to string values if provided
        hand = args.hand
        if args.hand_id is not None:
            hand_map = {0: 'left', 1: 'right', 2: 'both'}
            hand = hand_map.get(args.hand_id, hand)

        mode = args.mode
        if args.mode_id is not None:
            mode_map = {0: 'topic', 1: 'action'}
            mode = mode_map.get(args.mode_id, mode)

        csv_dir = args.csv_dir
        plot = args.plot
    else:
        # Interactive input mode
        print('\nO6 Hand Analyzer — interactive mode')

        # choose hand
        while True:
            hand_input = input("Select hand — enter 0:left, 1:right, 2:both (or name left/right/both). Default 'right': ").strip()
            if hand_input == '':
                hand = 'right'
                break
            if hand_input in ('0', '1', '2'):
                hand_map = {'0': 'left', '1': 'right', '2': 'both'}
                hand = hand_map[hand_input]
                break
            if hand_input.lower() in ('left', 'right', 'both'):
                hand = hand_input.lower()
                break
            print('Invalid choice — try again.')

        # choose mode
        while True:
            mode_input = input("Select mode — enter 0:topic, 1:action (or name topic/action). Default 'topic': ").strip()
            if mode_input == '':
                mode = 'topic'
                break
            if mode_input in ('0', '1'):
                mode_map = {'0': 'topic', '1': 'action'}
                mode = mode_map[mode_input]
                break
            if mode_input.lower() in ('topic', 'action'):
                mode = mode_input.lower()
                break
            print('Invalid choice — try again.')

        csv_dir = input("CSV directory (default 'analy_o6_hand'): ").strip() or 'analy_o6_hand'
        plot_input = input("Enable plotting? (y/n) [y]: ").strip().lower()
        plot = not (plot_input in ('n', 'no', '0'))

    rclpy.init()
    node = O6HandAnalyzer(hand, mode, csv_dir=csv_dir, plot=plot)
    saved = {'done': False}
    def sigint_handler(signum, frame):
        if saved['done']:
            sys.exit(0)
        saved['done'] = True
        node.get_logger().info('SIGINT received, stopping...')
        try:
            node.save_csv_and_plot()
        except Exception as e:
            node.get_logger().error(f'Error saving in signal handler: {e}')
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass
            try:
                if not rclpy.is_shutdown():
                    rclpy.shutdown()
            except Exception:
                pass
            sys.exit(0)
    import signal
    signal.signal(signal.SIGINT, sigint_handler)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not saved['done']:
            node.get_logger().info('Shutting down analyzer...')
            try:
                node.save_csv_and_plot()
            except Exception as e:
                node.get_logger().error(f'Error saving on exit: {e}')
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
