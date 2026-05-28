#!/usr/bin/env python3
"""
plot_joint_actions_dual_window.py

Dual-window real-time plotting for arms and hands separately.
- Window 1: Bimanual arms (14 joints: 7 left + 7 right)
- Window 2: Bimanual hands (12 joints: 6 left + 6 right for O6)

Features:
  - Multi-threaded ROS2 topic handling
  - Independent matplotlib windows for arms and hands
  - Real-time updates with synchronized data
  - Left-right grouped layout in each window

Usage:
    # Show both windows
    python3 plot_joint_actions_dual_window.py
    
    # Only show arms window
    python3 plot_joint_actions_dual_window.py --arms-only
    
    # Only show hands window
    python3 plot_joint_actions_dual_window.py --hands-only
    
    # Adjust settings
    python3 plot_joint_actions_dual_window.py --window 5 --rate 30
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
    matplotlib.use('Qt5Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError:
    print("Error: matplotlib required. Install: pip install matplotlib PyQt5")
    sys.exit(1)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class DualWindowPlotter(Node):
    """Real-time plotter with separate windows for arms and hands."""
    
    def __init__(self, window_sec: float = 10.0, update_hz: float = 20.0,
                 show_arms: bool = True, show_hands: bool = True):
        super().__init__('dual_window_plotter')
        
        self.window_sec = window_sec
        self.update_hz = update_hz
        self.show_arms = show_arms
        self.show_hands = show_hands
        
        # Data storage
        self.cmd_data: Dict[str, deque] = {}
        self.state_data: Dict[str, deque] = {}
        self.lock = threading.Lock()
        
        # Joint names
        self.all_joint_names: List[str] = []
        self.arm_joints: List[str] = []
        self.hand_joints: List[str] = []
        
        # Timing
        self.start_time = time.time()
        self.cmd_count = 0
        self.state_count = 0
        
        # Subscribers
        self.create_subscription(JointState, '/joint_actions', self._joint_actions_callback, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_states_callback, 10)
        
        self.get_logger().info("Dual-window plotter initialized")
        self.get_logger().info(f"  Window: {window_sec}s, Update: {update_hz}Hz")
        self.get_logger().info(f"  Show arms: {show_arms}, Show hands: {show_hands}")
    
    def _joint_actions_callback(self, msg: JointState):
        """Receive joint action commands."""
        t = time.time() - self.start_time
        
        with self.lock:
            if not self.all_joint_names:
                self.all_joint_names = list(msg.name)
                self._classify_joints()
                self.get_logger().info(f"Detected {len(self.all_joint_names)} joints")
                self.get_logger().info(f"  Arms: {len(self.arm_joints)}, Hands: {len(self.hand_joints)}")
                
                # Initialize data structures
                for name in self.all_joint_names:
                    self.cmd_data[name] = deque(maxlen=int(self.window_sec * 100))
                    self.state_data[name] = deque(maxlen=int(self.window_sec * 100))
            
            for name, pos in zip(msg.name, msg.position):
                if name in self.cmd_data:
                    self.cmd_data[name].append((t, pos))
            
            self.cmd_count += 1
    
    def _joint_states_callback(self, msg: JointState):
        """Receive actual joint states."""
        t = time.time() - self.start_time
        
        with self.lock:
            if not self.all_joint_names:
                return
            
            for name, pos in zip(msg.name, msg.position):
                if name in self.state_data:
                    self.state_data[name].append((t, pos))
            
            self.state_count += 1
    
    def _classify_joints(self):
        """Classify joints into arms and hands."""
        for name in self.all_joint_names:
            if 'openarm_' in name and '_joint' in name:
                self.arm_joints.append(name)
            elif name.startswith('L_') or name.startswith('R_'):
                self.hand_joints.append(name)
    
    def get_plot_data(self, joint_names: List[str]):
        """Get plot data for specified joints."""
        with self.lock:
            if not self.all_joint_names:
                return None
            
            data = {}
            for name in joint_names:
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


def create_window(node: DualWindowPlotter, joint_list: List[str], 
                  window_title: str, window_num: int):
    """Create a single matplotlib window for a group of joints."""
    
    if not joint_list:
        return None, None
    
    # Separate left and right
    left_joints = [name for name in joint_list 
                   if 'left' in name.lower() or name.startswith('L_')]
    right_joints = [name for name in joint_list 
                    if 'right' in name.lower() or name.startswith('R_')]
    
    n_rows = max(len(left_joints), len(right_joints))
    
    # Create figure
    fig, axes = plt.subplots(n_rows, 2, figsize=(14, 2.5 * n_rows), num=window_num)
    fig.suptitle(window_title, fontsize=13, fontweight='bold')
    
    if n_rows == 1:
        axes = axes.reshape(1, -1)
    
    # Map joints to axes
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
    
    # Initialize lines
    lines_cmd = {}
    lines_state = {}
    
    for joint_name in joint_list:
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
        ax.legend(loc='upper right', fontsize=7)
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Update function
    def update(frame):
        data = node.get_plot_data(joint_list)
        if data is None:
            return list(lines_cmd.values()) + list(lines_state.values())
        
        current_time = time.time() - node.start_time
        t_min = max(0, current_time - node.window_sec)
        t_max = current_time
        
        for joint_name in joint_list:
            joint_data = data[joint_name]
            
            # Update lines
            lines_cmd[joint_name].set_data(joint_data['cmd_t'], joint_data['cmd_v'])
            lines_state[joint_name].set_data(joint_data['state_t'], joint_data['state_v'])
            
            # Auto-scale
            ax = joint_to_ax[joint_name]
            ax.set_xlim(t_min, t_max)
            
            all_t = np.concatenate([joint_data['cmd_t'], joint_data['state_t']])
            all_v = np.concatenate([joint_data['cmd_v'], joint_data['state_v']])
            
            if len(all_t) > 0:
                mask = all_t >= t_min
                visible_v = all_v[mask]
                
                if len(visible_v) > 0:
                    v_min, v_max = visible_v.min(), visible_v.max()
                    margin = (v_max - v_min) * 0.1 if v_max > v_min else 0.5
                    ax.set_ylim(v_min - margin, v_max + margin)
        
        # Update title
        fig.suptitle(
            f'{window_title}  |  Commands: {node.cmd_count}  States: {node.state_count}  |  Time: {current_time:.1f}s',
            fontsize=11, fontweight='bold'
        )
        
        return list(lines_cmd.values()) + list(lines_state.values())
    
    return fig, update


def create_dual_window_plot(node: DualWindowPlotter):
    """Create and manage dual windows."""
    
    # Wait for data
    print("Waiting for /joint_actions...")
    while not node.all_joint_names and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    
    if not node.all_joint_names:
        print("Error: No data received.")
        return
    
    print(f"\nDetected {len(node.all_joint_names)} joints:")
    print(f"  Arms: {len(node.arm_joints)}")
    print(f"  Hands: {len(node.hand_joints)}")
    
    # Create windows
    interval_ms = int(1000.0 / node.update_hz)
    animations = []
    
    if node.show_arms and node.arm_joints:
        print(f"\n[Window 1] Creating arms window ({len(node.arm_joints)} joints)...")
        fig_arms, update_arms = create_window(
            node, node.arm_joints, 
            'Bimanual Arms - Command vs State\nLeft: Left Arm  |  Right: Right Arm',
            window_num=1
        )
        if fig_arms:
            anim_arms = FuncAnimation(fig_arms, update_arms, interval=interval_ms, 
                                     blit=True, cache_frame_data=False)
            animations.append(anim_arms)
    
    if node.show_hands and node.hand_joints:
        print(f"[Window 2] Creating hands window ({len(node.hand_joints)} joints)...")
        fig_hands, update_hands = create_window(
            node, node.hand_joints,
            'Bimanual Hands - Command vs State\nLeft: Left Hand  |  Right: Right Hand',
            window_num=2
        )
        if fig_hands:
            anim_hands = FuncAnimation(fig_hands, update_hands, interval=interval_ms,
                                      blit=True, cache_frame_data=False)
            animations.append(anim_hands)
    
    print(f"\n{'='*70}")
    print("Dual-window plot started!")
    print("Close all windows to exit.")
    print(f"{'='*70}\n")
    
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Dual-window real-time plot: arms and hands separately',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--window', type=float, default=10.0,
                       help='Time window in seconds (default: 10)')
    parser.add_argument('--rate', type=float, default=20.0,
                       help='Update rate in Hz (default: 20)')
    parser.add_argument('--arms-only', action='store_true',
                       help='Show only arms window')
    parser.add_argument('--hands-only', action='store_true',
                       help='Show only hands window')
    
    args = parser.parse_args()
    
    # Determine what to show
    show_arms = not args.hands_only
    show_hands = not args.arms_only
    
    rclpy.init()
    
    node = DualWindowPlotter(
        window_sec=args.window,
        update_hz=args.rate,
        show_arms=show_arms,
        show_hands=show_hands
    )
    
    # ROS spin in background thread (multi-threaded ROS2 handling)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        # Matplotlib windows in main thread
        create_dual_window_plot(node)
    except KeyboardInterrupt:
        print("\nShutdown requested...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
