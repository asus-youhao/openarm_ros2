#!/usr/bin/env python3
"""
Tkinter GUI for O6 bimanual hands control (action & topic mode).
Drag sliders for each joint, select mode, and send commands.
"""

import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk
import time
from sensor_msgs.msg import JointState

RIGHT_JOINTS = [
    'R_thumb_cmc_pitch', 'R_thumb_cmc_yaw', 'R_index_mcp_pitch',
    'R_middle_mcp_pitch', 'R_ring_mcp_pitch', 'R_pinky_mcp_pitch'
]
LEFT_JOINTS = [
    'L_thumb_cmc_pitch', 'L_thumb_cmc_yaw', 'L_index_mcp_pitch',
    'L_middle_mcp_pitch', 'L_ring_mcp_pitch', 'L_pinky_mcp_pitch'
]

# SDK motor order: [pitch, yaw, index, middle, ring, pinky, ...]
O6_JOINT_MIN = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
O6_JOINT_MAX = [0.58, 1.36, 1.6, 1.6, 1.6, 1.6, 1.08, 1.43, 1.43, 1.43, 1.43, 1.43]

class JointStateInitNode(Node):
    def __init__(self):
        super().__init__('joint_state_init_node')
        self.latest = None
        self.sub = self.create_subscription(JointState, '/joint_states', self.cb, 10)
    def cb(self, msg):
        self.latest = msg

def get_o6_joint_init():
    rclpy.init(args=None)
    node = JointStateInitNode()
    start = time.time()
    while rclpy.ok() and time.time() - start < 3:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest:
            names = node.latest.name
            pos = node.latest.position
            right_vals = []
            left_vals = []
            for j in RIGHT_JOINTS:
                if j in names:
                    idx = names.index(j)
                    right_vals.append(pos[idx])
                else:
                    right_vals.append(0.0)
            for j in LEFT_JOINTS:
                if j in names:
                    idx = names.index(j)
                    left_vals.append(pos[idx])
                else:
                    left_vals.append(0.0)
            node.destroy_node()
            rclpy.shutdown()
            return right_vals, left_vals
    node.destroy_node()
    rclpy.shutdown()
    return [0.0]*6, [0.0]*6

class BimanualHandNode(Node):
    def __init__(self, mode):
        super().__init__('bimanual_hand_gui')
        self.mode = mode
        if mode == 'action':
            self.right_client = ActionClient(
                self, FollowJointTrajectory,
                '/right_hand_controller/follow_joint_trajectory'
            )
            self.left_client = ActionClient(
                self, FollowJointTrajectory,
                '/left_hand_controller/follow_joint_trajectory'
            )
            self.get_logger().info('Waiting for action servers...')
            ok1 = self.right_client.wait_for_server(timeout_sec=0.5)
            ok2 = self.left_client.wait_for_server(timeout_sec=0.5)
            if not (ok1 and ok2):
                self.get_logger().error('Action server not connected! Disabled.')
                return
            self.get_logger().info('Action servers ready!')
        else:
            self.right_pub = self.create_publisher(
                Float64MultiArray,
                '/right_hand_forward_position_controller/commands', 10
            )
            self.left_pub = self.create_publisher(
                Float64MultiArray,
                '/left_hand_forward_position_controller/commands', 10
            )
            self.get_logger().info('Topic publishers ready!')

    def send(self, right_positions, left_positions):
        if self.mode == 'action':
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(sec=1, nanosec=0)
            right_goal = FollowJointTrajectory.Goal()
            right_goal.trajectory.joint_names = RIGHT_JOINTS
            point.positions = right_positions
            right_goal.trajectory.points = [point]
            left_goal = FollowJointTrajectory.Goal()
            left_goal.trajectory.joint_names = LEFT_JOINTS
            point.positions = left_positions
            left_goal.trajectory.points = [point]
            self.get_logger().info(f'Sending action goals...')
            self.get_logger().info(f'  Right: {right_positions}')
            self.get_logger().info(f'  Left:  {left_positions}')
            self.right_client.send_goal_async(right_goal)
            self.left_client.send_goal_async(left_goal)
        else:
            right_msg = Float64MultiArray()
            right_msg.data = right_positions
            left_msg = Float64MultiArray()
            left_msg.data = left_positions
            self.right_pub.publish(right_msg)
            self.left_pub.publish(left_msg)
            self.get_logger().info(f'Sent topic positions:')
            self.get_logger().info(f'  Right: {right_positions}')
            self.get_logger().info(f'  Left:  {left_positions}')

class BimanualHandGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title('O6 Bimanual Hands GUI')
        self.mode = tk.StringVar(value='action')
        right_init, left_init = get_o6_joint_init()
        self.right_vars = [tk.DoubleVar(value=right_init[i]) for i in range(6)]
        self.left_vars = [tk.DoubleVar(value=left_init[i]) for i in range(6)]
        self._build()
        self.node = None
        self.rclpy_thread = None

    def _build(self):
        self.root.geometry('1000x800')
        frm = ttk.Frame(self.root, padding=20)
        frm.grid(row=0, column=0)
        bigfont = ('Arial', 16)
        valfont = ('Arial', 16, 'bold')
        minmaxfont = ('Arial', 14)
        modefont = ('Arial', 20, 'bold')
        ttk.Label(frm, text='Mode:', font=modefont).grid(row=0, column=0, sticky='w')
        tk.Radiobutton(frm, text='Action', variable=self.mode, value='action', font=modefont, width=10).grid(row=0, column=1)
        tk.Radiobutton(frm, text='Topic', variable=self.mode, value='topic', font=modefont, width=10).grid(row=0, column=2)
        ttk.Label(frm, text='Right Hand', font=bigfont).grid(row=1, column=0, columnspan=5, pady=(10,0))
        self.right_val_labels = []
        for i, joint in enumerate(RIGHT_JOINTS):
            ttk.Label(frm, text=joint, font=bigfont).grid(row=2+i, column=0, sticky='w', padx=(0,10))
            min_label = ttk.Label(frm, text=f'{O6_JOINT_MIN[i]:.2f}', font=minmaxfont)
            min_label.grid(row=2+i, column=1, sticky='e', padx=(0,5))
            scale = ttk.Scale(frm, from_=O6_JOINT_MIN[i], to=O6_JOINT_MAX[i], variable=self.right_vars[i], orient='horizontal', length=300)
            scale.grid(row=2+i, column=2)
            max_label = ttk.Label(frm, text=f'{O6_JOINT_MAX[i]:.2f}', font=minmaxfont)
            max_label.grid(row=2+i, column=3, sticky='w', padx=(5,0))
            val_label = ttk.Label(frm, text=f'{self.right_vars[i].get():.2f}', font=valfont, foreground='blue')
            val_label.grid(row=2+i, column=4, sticky='w', padx=(10,0))
            self.right_val_labels.append(val_label)
        ttk.Label(frm, text='Left Hand', font=bigfont).grid(row=8, column=0, columnspan=5, pady=(20,0))
        self.left_val_labels = []
        for i, joint in enumerate(LEFT_JOINTS):
            ttk.Label(frm, text=joint, font=bigfont).grid(row=9+i, column=0, sticky='w', padx=(0,10))
            min_label = ttk.Label(frm, text=f'{O6_JOINT_MIN[i+6]:.2f}', font=minmaxfont)
            min_label.grid(row=9+i, column=1, sticky='e', padx=(0,5))
            scale = ttk.Scale(frm, from_=O6_JOINT_MIN[i+6], to=O6_JOINT_MAX[i+6], variable=self.left_vars[i], orient='horizontal', length=300)
            scale.grid(row=9+i, column=2)
            max_label = ttk.Label(frm, text=f'{O6_JOINT_MAX[i+6]:.2f}', font=minmaxfont)
            max_label.grid(row=9+i, column=3, sticky='w', padx=(5,0))
            val_label = ttk.Label(frm, text=f'{self.left_vars[i].get():.2f}', font=valfont, foreground='blue')
            val_label.grid(row=9+i, column=4, sticky='w', padx=(10,0))
            self.left_val_labels.append(val_label)
        self.send_btn = ttk.Button(frm, text='Send', command=self._on_send, style='TButton', width=12)
        self.send_btn.grid(row=15, column=0, columnspan=5, pady=20)
        ttk.Button(frm, text='All Zero', command=self._all_zero, style='TButton', width=12).grid(row=16, column=0, columnspan=5, pady=10)
        self.status_label = ttk.Label(frm, text='', font=('Arial', 14), foreground='red')
        self.status_label.grid(row=17, column=0, columnspan=5, pady=10)
        self.send_status_label = ttk.Label(frm, text='', font=('Arial', 16), foreground='green')
        self.send_status_label.grid(row=18, column=0, columnspan=5, pady=10)
        self._update_joint_labels()

    def _update_joint_labels(self):
        for i in range(6):
            self.right_val_labels[i].config(text=f'{self.right_vars[i].get():.2f}')
            self.left_val_labels[i].config(text=f'{self.left_vars[i].get():.2f}')
        self.root.after(200, self._update_joint_labels)

    def _on_send(self):
        mode = self.mode.get()
        right_positions = [v.get() for v in self.right_vars]
        left_positions = [v.get() for v in self.left_vars]
        self.send_status_label.config(text='Sending...', foreground='orange')
        self.root.update()
        
        if self.node is None or self.node.mode != mode:
            # Clean up old node
            if self.node:
                try:
                    if hasattr(self.node, 'right_client'):
                        self.node.right_client.destroy()
                except Exception:
                    pass
                try:
                    if hasattr(self.node, 'left_client'):
                        self.node.left_client.destroy()
                except Exception:
                    pass
                try:
                    self.node.destroy_node()
                except Exception:
                    pass
                self.node = None
            
            if self.rclpy_thread:
                try:
                    rclpy.shutdown()
                except Exception:
                    pass
                self.rclpy_thread.join()
                self.rclpy_thread = None
            
            # Initialize rclpy
            try:
                rclpy.init(args=None)
            except RuntimeError:
                pass  # Already initialized
            
            # Create new node
            self.node = BimanualHandNode(mode)
            
            # Check action server connection for action mode
            if mode == 'action':
                ok1 = self.node.right_client.wait_for_server(timeout_sec=0.5)
                ok2 = self.node.left_client.wait_for_server(timeout_sec=0.5)
                if not (ok1 and ok2):
                    self.status_label.config(text='Action server not connected! Click Send to retry.', foreground='orange')
                    self.send_status_label.config(text='Connection failed - Retry?', foreground='orange')
                    # Clean up failed node
                    try:
                        self.node.right_client.destroy()
                    except Exception:
                        pass
                    try:
                        self.node.left_client.destroy()
                    except Exception:
                        pass
                    try:
                        self.node.destroy_node()
                    except Exception:
                        pass
                    self.node = None
                    return
                self.status_label.config(text='Action server connected!', foreground='green')
                self.send_btn.config(state='normal')
            
            # Start rclpy spin thread
            self.rclpy_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
            self.rclpy_thread.start()
        
        # Send command
        try:
            self.node.send(right_positions, left_positions)
            self.send_status_label.config(text='Send OK', foreground='green')
        except Exception as e:
            self.send_status_label.config(text=f'Send error: {e}', foreground='red')

    def _all_zero(self):
        for v in self.right_vars:
            v.set(0.0)
        for v in self.left_vars:
            v.set(0.0)

    def run(self):
        self.root.mainloop()
        if self.node:
            try:
                self.node.right_client.destroy()
            except Exception:
                pass
            try:
                self.node.left_client.destroy()
            except Exception:
                pass
            self.node.destroy_node()
        if self.rclpy_thread:
            rclpy.shutdown()
            self.rclpy_thread.join()

if __name__ == '__main__':
    BimanualHandGUI().run()
