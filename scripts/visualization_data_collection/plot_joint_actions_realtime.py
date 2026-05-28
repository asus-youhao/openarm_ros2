#!/usr/bin/env python3
"""
plot_joint_actions_realtime.py

Real-time plotting of /joint_actions (commands) vs /joint_states (actual positions).
Useful for verifying IK control, monitoring joint tracking errors, and debugging.

Features:
  - Real-time scrolling plots for all joints (arms + hands)
  - Color-coded: arms (blue/orange), left hand (green), right hand (red)
  - Shows both command and state for comparison
  - Configurable display: all joints, arms only, or specific groups
  - Automatic detection of joint configuration from /joint_actions

Usage:
    # Show all joints (default)
    ros2 run openarm_ros2 plot_joint_actions_realtime.py
    
    # Show only arm joints (14 joints)
    ros2 run openarm_ros2 plot_joint_actions_realtime.py --arms-only
    
    # Show specific joint indices (0-based)
    ros2 run openarm_ros2 plot_joint_actions_realtime.py --joints 0,1,2,7,8,9
    
    # Adjust time window (default: 10 seconds)
    ros2 run openarm_ros2 plot_joint_actions_realtime.py --window 20
    
    # Adjust update rate (default: 20 Hz)
    ros2 run openarm_ros2 plot_joint_actions_realtime.py --rate 30

Controls:
    - Close window to exit
    - Window auto-scales to data
"""

import argparse
import sys
import threading
import time
from collections import deque
from typing import Dict, List, Optional

import numpy as np

try:
    import matplotlib
    matplotlib.use('Qt5Agg')  # or TkAgg
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError:
    print("Error: matplotlib required. Install: pip install matplotlib")
    sys.exit(1)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointActionsPlotter(Node):
    """Real-time plotter for joint_actions vs joint_states."""
    
    def __init__(self, window_sec: float = 10.0, update_hz: float = 20.0,
                 arms_only: bool = False, joint_indices: Optional[List[int]] = None):
        super().__init__('joint_actions_plotter')
        
        self.window_sec = window_sec
        self.update_hz = update_hz
        self.arms_only = arms_only
        self.joint_indices = joint_indices
        
        # Data storage: {joint_name: deque([(t, value), ...])}
        self.cmd_data: Dict[str, deque] = {}
        self.state_data: Dict[str, deque] = {}
        self.lock = threading.Lock()
        
        # Joint names (will be populated from first /joint_actions message)
        self.all_joint_names: List[str] = []
        self.display_joint_names: List[str] = []
        
        # Timing
        self.start_time = time.time()
        
        # Statistics
        self.cmd_count = 0
        self.state_count = 0
        
        # Subscribers
        self.create_subscription(
            JointState, '/joint_actions',
            self._joint_actions_callback, 10)
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_states_callback, 10)
        
        self.get_logger().info("Real-time plotter initialized")
        self.get_logger().info(f"  Window: {window_sec}s, Update: {update_hz}Hz")
        if arms_only:
            self.get_logger().info("  Display: Arms only (14 joints)")
        elif joint_indices:
            self.get_logger().info(f"  Display: Selected joints {joint_indices}")
        else:
            self.get_logger().info("  Display: All joints")
    
    def _joint_actions_callback(self, msg: JointState):
        """Receive joint action commands."""
        t = time.time() - self.start_time
        
        with self.lock:
            # First message: initialize joint names
            if not self.all_joint_names:
                self.all_joint_names = list(msg.name)
                self._init_display_joints()
                self.get_logger().info(f"Detected {len(self.all_joint_names)} joints from /joint_actions")
                
                # Initialize data structures
                for name in self.display_joint_names:
                    self.cmd_data[name] = deque(maxlen=int(self.window_sec * 100))
                    self.state_data[name] = deque(maxlen=int(self.window_sec * 100))
            
            # Store command data
            for name, pos in zip(msg.name, msg.position):
                if name in self.cmd_data:
                    self.cmd_data[name].append((t, pos))
            
            self.cmd_count += 1
    
    def _joint_states_callback(self, msg: JointState):
        """Receive actual joint states."""
        t = time.time() - self.start_time
        
        with self.lock:
            if not self.all_joint_names:
                # Wait for joint_actions to define joint names
                return
            
            # Store state data
            for name, pos in zip(msg.name, msg.position):
                if name in self.state_data:
                    self.state_data[name].append((t, pos))
            
            self.state_count += 1
    
    def _init_display_joints(self):
        """Initialize which joints to display based on configuration."""
        if self.joint_indices is not None:
            # User-specified joint indices
            self.display_joint_names = [
                self.all_joint_names[i] for i in self.joint_indices
                if i < len(self.all_joint_names)
            ]
        elif self.arms_only:
            # Only arm joints (first 14)
            self.display_joint_names = [
                name for name in self.all_joint_names
                if 'openarm_' in name and '_joint' in name
            ][:14]
        else:
            # All joints
            self.display_joint_names = list(self.all_joint_names)
    
    def get_plot_data(self):
        """Get current plot data (called from animation thread)."""
        with self.lock:
            if not self.display_joint_names:
                return None
            
            data = {}
            for name in self.display_joint_names:
                cmd_list = list(self.cmd_data.get(name, []))
                state_list = list(self.state_data.get(name, []))
                
                if cmd_list:
                    cmd_t, cmd_v = zip(*cmd_list)
                else:
                    cmd_t, cmd_v = [], []
                
                if state_list:
                    state_t, state_v = zip(*state_list)
                else:
                    state_t, state_v = [], []
                
                data[name] = {
                    'cmd_t': np.array(cmd_t),
                    'cmd_v': np.array(cmd_v),
                    'state_t': np.array(state_t),
                    'state_v': np.array(state_v),
                }
            
            return data


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


def create_realtime_plot(node: JointActionsPlotter):
    """Create and run the real-time plot."""
    
    # Wait for first message to get joint names
    print("Waiting for /joint_actions message to detect joint configuration...")
    while not node.all_joint_names and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    
    if not node.all_joint_names:
        print("Error: No data received. Is /joint_actions being published?")
        return
    
    n_joints = len(node.display_joint_names)
    print(f"\nDisplaying {n_joints} joints:")
    for i, name in enumerate(node.display_joint_names):
        print(f"  [{i:2d}] {name}")
    
    # Separate joints into left and right groups
    left_joints = [name for name in node.display_joint_names 
                   if 'left' in name.lower() or name.startswith('L_')]
    right_joints = [name for name in node.display_joint_names 
                    if 'right' in name.lower() or name.startswith('R_')]
    
    # Create figure with 2 columns: left side for left arm+hand, right side for right arm+hand
    n_rows = max(len(left_joints), len(right_joints))
    n_cols = 2
    
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(16, 2.5 * n_rows))
    fig.suptitle('Joint Actions (command) vs Joint States (actual) - Real-time\n'
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
    
    # Initialize line objects
    lines_cmd = {}
    lines_state = {}
    
    for joint_name in node.display_joint_names:
        ax = joint_to_ax[joint_name]
        color, alpha = get_joint_color(joint_name)
        
        line_cmd, = ax.plot([], [], color=color, linewidth=2, label='command', alpha=0.9)
        line_state, = ax.plot([], [], color=color, linewidth=1.5, 
                              linestyle='--', label='state', alpha=0.7)
        
        lines_cmd[joint_name] = line_cmd
        lines_state[joint_name] = line_state
        
        ax.set_xlabel('Time (s)', fontsize=9)
        ax.set_ylabel('Position (rad)', fontsize=9)
        ax.set_title(joint_name, fontsize=10, fontweight='bold')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Animation update function
    def update(frame):
        data = node.get_plot_data()
        if data is None:
            return list(lines_cmd.values()) + list(lines_state.values())
        
        current_time = time.time() - node.start_time
        
        for joint_name in node.display_joint_names:
            joint_data = data[joint_name]
            
            # Update command line
            lines_cmd[joint_name].set_data(joint_data['cmd_t'], joint_data['cmd_v'])
            
            # Update state line
            lines_state[joint_name].set_data(joint_data['state_t'], joint_data['state_v'])
            
            # Auto-scale axes
            ax = joint_to_ax[joint_name]
            
            # Time axis: show last window_sec seconds
            t_min = max(0, current_time - node.window_sec)
            t_max = current_time
            ax.set_xlim(t_min, t_max)
            
            # Y axis: auto-scale based on visible data
            all_t = np.concatenate([joint_data['cmd_t'], joint_data['state_t']])
            all_v = np.concatenate([joint_data['cmd_v'], joint_data['state_v']])
            
            if len(all_t) > 0:
                mask = all_t >= t_min
                visible_v = all_v[mask]
                
                if len(visible_v) > 0:
                    v_min, v_max = visible_v.min(), visible_v.max()
                    margin = (v_max - v_min) * 0.1 if v_max > v_min else 0.5
                    ax.set_ylim(v_min - margin, v_max + margin)
        
        # Update title with stats
        fig.suptitle(
            f'Joint Actions vs States - Real-time  |  '
            f'Commands: {node.cmd_count}  States: {node.state_count}  |  '
            f'Time: {current_time:.1f}s',
            fontsize=12, fontweight='bold'
        )
        
        # Return all line objects for blitting
        all_lines = list(lines_cmd.values()) + list(lines_state.values())
        return all_lines
    
    # Create animation
    interval_ms = int(1000.0 / node.update_hz)
    anim = FuncAnimation(fig, update, interval=interval_ms, blit=True, cache_frame_data=False)
    
    print(f"\n{'='*70}")
    print("Real-time plot started!")
    print("Close the plot window to exit.")
    print(f"{'='*70}\n")
    
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Real-time plot of /joint_actions vs /joint_states',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--window', type=float, default=10.0,
                       help='Time window in seconds (default: 10)')
    parser.add_argument('--rate', type=float, default=20.0,
                       help='Plot update rate in Hz (default: 20)')
    parser.add_argument('--arms-only', action='store_true',
                       help='Show only arm joints (14 joints)')
    parser.add_argument('--joints', type=str, default=None,
                       help='Comma-separated joint indices to display (e.g., "0,1,2,7,8")')
    
    args = parser.parse_args()
    
    # Parse joint indices
    joint_indices = None
    if args.joints:
        try:
            joint_indices = [int(x.strip()) for x in args.joints.split(',')]
        except ValueError:
            print(f"Error: Invalid joint indices '{args.joints}'")
            sys.exit(1)
    
    rclpy.init()
    
    node = JointActionsPlotter(
        window_sec=args.window,
        update_hz=args.rate,
        arms_only=args.arms_only,
        joint_indices=joint_indices
    )
    
    # ROS spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        # Run plot in main thread (required for matplotlib)
        create_realtime_plot(node)
    except KeyboardInterrupt:
        print("\nShutdown requested...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
