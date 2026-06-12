#!/usr/bin/env python3
"""
plot_joint_tracking_error.py

Simpler alternative to plot_joint_actions_realtime.py that focuses on tracking error.
Shows the difference between commanded and actual positions for all joints.

Features:
  - Single plot showing tracking error for all joints
  - Color-coded by joint group (arms/hands)
  - Optional error threshold highlighting
  - Real-time statistics (mean, max, RMS error)

Usage:
    # Basic usage
    ros2 run openarm_ros2 plot_joint_tracking_error.py
    
    # Show error threshold line (e.g., 0.05 rad = ~2.9 degrees)
    ros2 run openarm_ros2 plot_joint_tracking_error.py --threshold 0.05
    
    # Adjust window
    ros2 run openarm_ros2 plot_joint_tracking_error.py --window 15
"""

import argparse
import sys
import threading
import time
from collections import deque
from typing import Dict, List

import numpy as np

try:
    import matplotlib
    matplotlib.use('Qt5Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError:
    print("Error: matplotlib required. Install: pip install matplotlib")
    sys.exit(1)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointErrorPlotter(Node):
    """Real-time plotter for joint tracking errors."""
    
    def __init__(self, window_sec: float = 10.0, threshold: float = None):
        super().__init__('joint_error_plotter')
        
        self.window_sec = window_sec
        self.threshold = threshold
        
        # Data: {joint_name: deque([(t, error), ...])}
        self.error_data: Dict[str, deque] = {}
        self.lock = threading.Lock()
        
        # Latest positions
        self.latest_cmd: Dict[str, float] = {}
        self.latest_state: Dict[str, float] = {}
        
        # Joint names
        self.joint_names: List[str] = []
        
        # Timing
        self.start_time = time.time()
        
        # Subscribers
        self.create_subscription(JointState, '/joint_actions', self._cmd_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._state_cb, 10)
        
        self.get_logger().info(f"Error plotter initialized (window: {window_sec}s)")
        if threshold:
            self.get_logger().info(f"  Threshold: {threshold:.4f} rad (~{np.degrees(threshold):.2f}°)")
    
    def _cmd_cb(self, msg: JointState):
        """Receive commands."""
        with self.lock:
            if not self.joint_names:
                self.joint_names = list(msg.name)
                for name in self.joint_names:
                    self.error_data[name] = deque(maxlen=int(self.window_sec * 100))
                self.get_logger().info(f"Tracking {len(self.joint_names)} joints")
            
            for name, pos in zip(msg.name, msg.position):
                self.latest_cmd[name] = pos
                self._compute_error(name)
    
    def _state_cb(self, msg: JointState):
        """Receive states."""
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self.joint_names:
                    self.latest_state[name] = pos
                    self._compute_error(name)
    
    def _compute_error(self, name: str):
        """Compute and store error if both cmd and state available."""
        if name in self.latest_cmd and name in self.latest_state:
            error = self.latest_cmd[name] - self.latest_state[name]
            t = time.time() - self.start_time
            self.error_data[name].append((t, error))
    
    def get_plot_data(self):
        """Get current plot data."""
        with self.lock:
            if not self.joint_names:
                return None
            
            data = {}
            for name in self.joint_names:
                err_list = list(self.error_data.get(name, []))
                if err_list:
                    t, e = zip(*err_list)
                    data[name] = {'t': np.array(t), 'error': np.array(e)}
                else:
                    data[name] = {'t': np.array([]), 'error': np.array([])}
            
            return data
    
    def get_statistics(self):
        """Compute error statistics."""
        with self.lock:
            all_errors = []
            for name in self.joint_names:
                err_list = list(self.error_data.get(name, []))
                if err_list:
                    _, e = zip(*err_list)
                    all_errors.extend(e)
            
            if all_errors:
                arr = np.array(all_errors)
                return {
                    'mean': np.mean(np.abs(arr)),
                    'max': np.max(np.abs(arr)),
                    'rms': np.sqrt(np.mean(arr**2))
                }
            return {'mean': 0, 'max': 0, 'rms': 0}


def create_error_plot(node: JointErrorPlotter, update_hz: float = 20.0):
    """Create and run the error plot with separate subplots for each joint."""
    
    # Wait for data
    print("Waiting for data...")
    while not node.joint_names and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    
    if not node.joint_names:
        print("Error: No data received.")
        return
    
    print(f"\nTracking {len(node.joint_names)} joints:")
    for name in node.joint_names:
        print(f"  - {name}")
    print()
    
    # Separate joints into left and right groups
    left_joints = [name for name in node.joint_names 
                   if 'left' in name.lower() or name.startswith('L_')]
    right_joints = [name for name in node.joint_names 
                    if 'right' in name.lower() or name.startswith('R_')]
    
    # Create figure with 2 columns: left side for left arm+hand, right side for right arm+hand
    n_rows = max(len(left_joints), len(right_joints))
    n_cols = 2
    
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(16, 2.5 * n_rows))
    fig.suptitle('Joint Tracking Errors (command - actual) - Real-time\n'
                 'Left Side: Left Arm + Left Hand  |  Right Side: Right Arm + Right Hand', 
                 fontsize=13, fontweight='bold')
    
    # Ensure axes is 2D array
    if n_rows == 1:
        axes = axes.reshape(1, -1)
    
    # Map joint names to axis positions
    joint_to_ax = {}
    for i, name in enumerate(left_joints):
        joint_to_ax[name] = axes[i, 0]
    for i, name in enumerate(right_joints):
        joint_to_ax[name] = axes[i, 1]
    
    # Hide unused axes
    for i in range(len(left_joints), n_rows):
        axes[i, 0].set_visible(False)
    for i in range(len(right_joints), n_rows):
        axes[i, 1].set_visible(False)
    
    # Define get_joint_color function BEFORE using it
    def get_joint_color(joint_name: str) -> tuple:
        """Get color for joint based on name."""
        if 'openarm_left' in joint_name:
            return '#1f77b4', 0.3  # Blue for left arm
        elif 'openarm_right' in joint_name:
            return '#ff7f0e', 0.3  # Orange for right arm
        elif joint_name.startswith('L_'):
            return '#2ca02c', 0.3  # Green for left hand
        elif joint_name.startswith('R_'):
            return '#d62728', 0.3  # Red for right hand
        else:
            return '#7f7f7f', 0.3  # Gray for unknown
    
    # Initialize line objects for each joint
    lines = {}
    threshold_lines = {}
    
    for joint_name in node.joint_names:
        ax = joint_to_ax[joint_name]
        color, _ = get_joint_color(joint_name)
        
        # Error line
        line, = ax.plot([], [], color=color, linewidth=2, alpha=0.8)
        lines[joint_name] = line
        
        # Threshold lines
        if node.threshold:
            line_pos = ax.axhline(node.threshold, color='red', linestyle='--', 
                                  linewidth=1.5, alpha=0.4)
            line_neg = ax.axhline(-node.threshold, color='red', linestyle='--', 
                                  linewidth=1.5, alpha=0.4)
            threshold_lines[joint_name] = [line_pos, line_neg]
        
        # Zero line
        ax.axhline(0, color='black', linestyle='-', linewidth=0.8, alpha=0.4)
        
        ax.set_xlabel('Time (s)', fontsize=9)
        ax.set_ylabel('Error (rad)', fontsize=9)
        ax.set_title(joint_name, fontsize=10, fontweight='bold')
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    def update(frame):
        data = node.get_plot_data()
        if data is None:
            all_lines = list(lines.values())
            for tl in threshold_lines.values():
                all_lines.extend(tl)
            return all_lines
        
        current_time = time.time() - node.start_time
        t_min = max(0, current_time - node.window_sec)
        t_max = current_time
        
        # Update each joint's subplot
        for joint_name in node.joint_names:
            joint_data = data[joint_name]
            ax = joint_to_ax[joint_name]
            
            # Update error line
            lines[joint_name].set_data(joint_data['t'], joint_data['error'])
            
            # Update axes limits
            ax.set_xlim(t_min, t_max)
            
            # Auto-scale Y axis based on visible data
            if len(joint_data['error']) > 0:
                mask = joint_data['t'] >= t_min
                visible_err = joint_data['error'][mask]
                
                if len(visible_err) > 0:
                    max_abs_err = np.max(np.abs(visible_err))
                    margin = max(0.01, max_abs_err * 0.2)
                    ax.set_ylim(-max_abs_err - margin, max_abs_err + margin)
        
        # Update title with overall statistics
        stats = node.get_statistics()
        fig.suptitle(
            f'Joint Tracking Errors - Real-time  |  '
            f'Mean: {stats["mean"]*1000:.2f}mrad ({np.degrees(stats["mean"]):.3f}°)  |  '
            f'Max: {stats["max"]*1000:.2f}mrad ({np.degrees(stats["max"]):.3f}°)  |  '
            f'RMS: {stats["rms"]*1000:.2f}mrad ({np.degrees(stats["rms"]):.3f}°)\n'
            f'Left Side: Left Arm + Left Hand  |  Right Side: Right Arm + Right Hand',
            fontsize=11, fontweight='bold'
        )
        
        all_lines = list(lines.values())
        for tl in threshold_lines.values():
            all_lines.extend(tl)
        return all_lines
    
    interval_ms = int(1000.0 / update_hz)
    anim = FuncAnimation(fig, update, interval=interval_ms, blit=True)
    
    print("Plot started! Close window to exit.\n")
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Plot joint tracking errors in real-time')
    parser.add_argument('--window', type=float, default=10.0,
                       help='Time window in seconds (default: 10)')
    parser.add_argument('--rate', type=float, default=20.0,
                       help='Update rate in Hz (default: 20)')
    parser.add_argument('--threshold', type=float, default=None,
                       help='Error threshold to highlight (rad, e.g., 0.05)')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = JointErrorPlotter(window_sec=args.window, threshold=args.threshold)
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        create_error_plot(node, update_hz=args.rate)
    except KeyboardInterrupt:
        print("\nShutdown requested...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
