#!/usr/bin/env python3
"""
GUI Controller for OpenArm Bimanual System with LEAP Hands.

Features:
- Slider controls for all joints (7+7 arms + 16 right hand)
- Real-time joint state display
- Continuous command sending at 20Hz
- Predefined poses
- Joint limit display
- Uses tkinter (Python built-in, no extra dependencies)
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import threading
import tkinter as tk
from tkinter import ttk


class BimanualGUIController:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        
        # Create main window
        self.root = tk.Tk()
        self.root.title('OpenArm Bimanual Controller - Button Send Mode')
        self.root.geometry('1200x900')
        
        # Joint sliders storage
        self.joint_sliders = {}
        self.joint_value_labels = {}
        self.joint_current_labels = {}
        
        # Position recording storage
        self.saved_positions = []  # List of saved position snapshots
        self.current_position_name = tk.StringVar(value="Position 1")
        
        # Joint limits (in radians)
        self.joint_limits = {
            # Left Arm
            'openarm_left_joint1': (-3.491, 1.396),
            'openarm_left_joint2': (-3.316, 0.175),
            'openarm_left_joint3': (-1.571, 1.571),
            'openarm_left_joint4': (0.0, 2.443),
            'openarm_left_joint5': (-1.571, 1.571),
            'openarm_left_joint6': (-0.785, 0.785),
            'openarm_left_joint7': (-1.571, 1.571),
            
            # Right Arm
            'openarm_right_joint1': (-1.396, 3.491),
            'openarm_right_joint2': (-0.175, 3.316),
            'openarm_right_joint3': (-1.571, 1.571),
            'openarm_right_joint4': (0.0, 2.443),
            'openarm_right_joint5': (-1.571, 1.571),
            'openarm_right_joint6': (-0.785, 0.785),
            'openarm_right_joint7': (-1.571, 1.571),
            
            # LEAP Hand - Index Finger
            'right_index_mcp_side': (-1.047, 1.047),
            'right_index_mcp_forward': (-0.314, 2.23),
            'right_index_pip': (-0.506, 1.885),
            'right_index_dip': (-0.366, 2.042),
            
            # LEAP Hand - Middle Finger
            'right_middle_mcp_side': (-1.047, 1.047),
            'right_middle_mcp_forward': (-0.314, 2.23),
            'right_middle_pip': (-0.506, 1.885),
            'right_middle_dip': (-0.366, 2.042),
            
            # LEAP Hand - Ring Finger
            'right_ring_mcp_side': (-0.436, 0.436),
            'right_ring_mcp_forward': (0.0, 1.571),
            'right_ring_pip': (0.0, 1.571),
            'right_ring_dip': (0.0, 1.571),
            
            # LEAP Hand - Thumb (corrected from URDF)
            'right_thumb_mcp_side': (-0.349, 2.094),
            'right_thumb_mcp_forward': (-0.47, 2.443),
            'right_thumb_pip_joint': (-1.20, 1.90),
            'right_thumb_dip_joint': (-1.34, 1.88),
        }
        
        self.create_ui()
        
        # Update joint states timer
        self.update_joint_states_scheduled()
        
    def create_ui(self):
        """Create the user interface."""
        # Control panel
        control_frame = tk.Frame(self.root, bg='lightgray', padx=10, pady=10)
        control_frame.pack(fill='x')
        
        # Title
        title_label = tk.Label(control_frame, text='Control Panel - Press to Send', 
                               font=('Arial', 12, 'bold'), bg='lightgray')
        title_label.pack(side='left', padx=10)
        
        # Move to Target button
        self.move_btn = tk.Button(control_frame, text='Move to Target', 
                                   bg='#4CAF50', fg='white', 
                                   font=('Arial', 14, 'bold'),
                                   width=15, height=2,
                                   command=self.move_to_target)
        self.move_btn.pack(side='left', padx=5)
        
        # Status label
        self.status_label = tk.Label(control_frame, text='Status: READY',
                                     font=('Arial', 12, 'bold'),
                                     fg='blue', bg='lightgray')
        self.status_label.pack(side='left', padx=20)
        
        # Preset buttons
        tk.Button(control_frame, text='Home Position', 
                 command=self.send_home_pose).pack(side='left', padx=5)
        tk.Button(control_frame, text='Grasp Hand', 
                 command=self.send_grasp_pose).pack(side='left', padx=5)
        tk.Button(control_frame, text='Open Hand', 
                 command=self.send_open_hand).pack(side='left', padx=5)
        
        # Test Hand Interpolation button
        tk.Button(control_frame, text='Test Hand Interpolation',
                  bg='#FFEB3B', fg='black', font=('Arial', 10, 'bold'),
                  command=self.test_send_hand_interpolation_btn).pack(side='left', padx=5)
        
        # Position recording section
        recording_frame = tk.Frame(self.root, bg='lightblue', padx=10, pady=10)
        recording_frame.pack(fill='x')
        
        tk.Label(recording_frame, text='Position Recording:', 
                font=('Arial', 11, 'bold'), bg='lightblue').pack(side='left', padx=5)
        
        # Position name entry
        tk.Entry(recording_frame, textvariable=self.current_position_name, 
                width=15).pack(side='left', padx=5)
        
        # Save current position button
        tk.Button(recording_frame, text='Save Position', 
                 bg='#2196F3', fg='white', font=('Arial', 10, 'bold'),
                 command=self.save_current_position).pack(side='left', padx=5)
        
        # Position counter
        self.position_counter_label = tk.Label(recording_frame, 
                                               text='Saved: 0', 
                                               font=('Arial', 10), bg='lightblue')
        self.position_counter_label.pack(side='left', padx=10)
        
        # Export to JSON button
        tk.Button(recording_frame, text='Export to JSON', 
                 bg='#4CAF50', fg='white', font=('Arial', 10, 'bold'),
                 command=self.export_positions_to_json).pack(side='left', padx=5)
        
        # Load from JSON button
        tk.Button(recording_frame, text='Load from JSON', 
                 bg='#FF9800', fg='white', font=('Arial', 10, 'bold'),
                 command=self.load_positions_from_json).pack(side='left', padx=5)
        
        # Clear all button
        tk.Button(recording_frame, text='Clear All', 
                 bg='#f44336', fg='white', font=('Arial', 10, 'bold'),
                 command=self.clear_all_positions).pack(side='left', padx=5)
        
        # Export to replay format button
        tk.Button(recording_frame, text='Export to Replay', 
                 bg='#9C27B0', fg='white', font=('Arial', 10, 'bold'),
                 command=self.export_to_replay_format).pack(side='left', padx=5)
        
        # Create scrollable canvas
        canvas_frame = tk.Frame(self.root)
        canvas_frame.pack(fill='both', expand=True)
        
        canvas = tk.Canvas(canvas_frame)
        scrollbar = tk.Scrollbar(canvas_frame, orient='vertical', command=canvas.yview)
        scrollable_frame = tk.Frame(canvas)
        
        scrollable_frame.bind(
            '<Configure>',
            lambda e: canvas.configure(scrollregion=canvas.bbox('all'))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor='nw')
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')
        
        # Left Arm Section
        self.create_joint_group(scrollable_frame, 'Left Arm (7 joints)', 
                               self.ros_node.left_arm_joints)
        
        # Right Arm Section
        self.create_joint_group(scrollable_frame, 'Right Arm (7 joints)', 
                               self.ros_node.right_arm_joints)
        
        # Right Hand Section
        self.create_joint_group(scrollable_frame, 'Right Hand (16 joints)', 
                               self.ros_node.right_hand_joints)
    
    def create_joint_group(self, parent, title, joint_names):
        """Create a group of joint sliders."""
        group_frame = tk.LabelFrame(parent, text=title, font=('Arial', 11, 'bold'),
                                    padx=10, pady=10)
        group_frame.pack(fill='x', padx=10, pady=5)
        
        for joint_name in joint_names:
            self.create_joint_slider(group_frame, joint_name)
    
    def create_joint_slider(self, parent, joint_name):
        """Create a single joint slider."""
        frame = tk.Frame(parent)
        frame.pack(fill='x', pady=2)
        
        # Joint name
        name_label = tk.Label(frame, text=joint_name, width=30, anchor='w')
        name_label.pack(side='left', padx=5)
        
        # Get limits
        min_val, max_val = self.joint_limits.get(joint_name, (-3.14, 3.14))
        
        # Min limit label (rad and deg)
        min_deg = min_val * 180.0 / 3.14159
        min_label = tk.Label(frame, text=f'{min_val:.2f}r\n{min_deg:.0f}°', 
                            width=7, font=('Arial', 8))
        min_label.pack(side='left')
        
        # Slider
        slider = tk.Scale(frame, from_=min_val, to=max_val, 
                         resolution=0.001, orient='horizontal',
                         length=300, showvalue=0,
                         command=lambda v: self.on_slider_change(joint_name, v))
        slider.set(0.0)
        slider.pack(side='left', padx=5)
        self.joint_sliders[joint_name] = slider
        
        # Max limit label (rad and deg)
        max_deg = max_val * 180.0 / 3.14159
        max_label = tk.Label(frame, text=f'{max_val:.2f}r\n{max_deg:.0f}°', 
                            width=7, font=('Arial', 8))
        max_label.pack(side='left')
        
        # Target value label (rad and deg)
        target_label = tk.Label(frame, text='Target:\n0.000r (0°)', 
                               width=16, font=('Arial', 9, 'bold'))
        target_label.pack(side='left', padx=5)
        self.joint_value_labels[joint_name] = target_label
        
        # Current value label (rad and deg)
        current_label = tk.Label(frame, text='Current:\n--- (---)', 
                                width=16, fg='blue', font=('Arial', 8))
        current_label.pack(side='left', padx=5)
        self.joint_current_labels[joint_name] = current_label
        
        # Zero button
        zero_btn = tk.Button(frame, text='Zero', width=5,
                            command=lambda: slider.set(0.0))
        zero_btn.pack(side='left', padx=5)
    
    def on_slider_change(self, joint_name, value):
        """Handle slider value change."""
        val = float(value)
        deg = val * 180.0 / 3.14159
        self.joint_value_labels[joint_name].config(
            text=f'Target:\n{val:.3f}r ({deg:.1f}°)'
        )
    
    def move_to_target(self):
        """Send target positions directly to controllers."""
        # Get target positions from sliders
        target_left = [self.joint_sliders[j].get() for j in self.ros_node.left_arm_joints]
        target_right = [self.joint_sliders[j].get() for j in self.ros_node.right_arm_joints]
        target_hand = [self.joint_sliders[j].get() for j in self.ros_node.right_hand_joints]
        
        # Send to controllers
        self.ros_node.publish_positions('left_arm', target_left)
        self.ros_node.publish_positions('right_arm', target_right)
        self.ros_node.publish_positions('right_hand', target_hand)
        
        # Update status
        self.status_label.config(text='Status: Command Sent', fg='green')
        self.root.after(1000, lambda: self.status_label.config(text='Status: READY', fg='blue'))
        
        self.ros_node.get_logger().info('Sent target positions to all controllers')
    
    def update_joint_states_scheduled(self):
        """Scheduled function to update joint state displays."""
        joint_states = self.ros_node.get_current_joint_states()
        for joint_name, position in joint_states.items():
            if joint_name in self.joint_current_labels:
                deg = position * 180.0 / 3.14159
                self.joint_current_labels[joint_name].config(
                    text=f'Current:\n{position:.3f}r ({deg:.1f}°)'
                )
        
        # Schedule next update (500ms)
        self.root.after(500, self.update_joint_states_scheduled)
    
    def send_home_pose(self):
        """Set all sliders to home (0.0)."""
        for slider in self.joint_sliders.values():
            slider.set(0.0)
    
    def send_grasp_pose(self):
        """Set hand to grasp pose."""
        grasp_positions = {
            'right_index_mcp_side': 0.00159, 'right_index_mcp_forward': 1.46159,
            'right_index_pip': 0.17559, 'right_index_dip': 0.52159,
            'right_middle_mcp_side': 0.00159, 'right_middle_mcp_forward': 1.46159,
            'right_middle_pip': 0.17559, 'right_middle_dip': 0.52159,
            'right_ring_mcp_side': 0.00159, 'right_ring_mcp_forward': 1.46159,
            'right_ring_pip': 0.17559, 'right_ring_dip': 0.52159,
            'right_thumb_mcp_forward': 1.70159, 'right_thumb_mcp_side': 0.00159,
            'right_thumb_pip_joint': 0.43759, 'right_thumb_dip_joint': 0.17159,
        }
        for joint_name, position in grasp_positions.items():
            if joint_name in self.joint_sliders:
                self.joint_sliders[joint_name].set(position)
    
    def send_open_hand(self):
        """Set hand to open pose."""
        for joint_name in self.ros_node.right_hand_joints:
            if joint_name in self.joint_sliders:
                self.joint_sliders[joint_name].set(0.0)
    
    def save_current_position(self):
        """Save current joint positions."""
        import datetime
        
        position_data = {
            'name': self.current_position_name.get(),
            'timestamp': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'left_arm': [self.joint_sliders[j].get() for j in self.ros_node.left_arm_joints],
            'right_arm': [self.joint_sliders[j].get() for j in self.ros_node.right_arm_joints],
            'right_hand': [self.joint_sliders[j].get() for j in self.ros_node.right_hand_joints],
            'joint_names': {
                'left_arm': self.ros_node.left_arm_joints,
                'right_arm': self.ros_node.right_arm_joints,
                'right_hand': self.ros_node.right_hand_joints
            }
        }
        
        self.saved_positions.append(position_data)
        
        # Update counter
        self.position_counter_label.config(text=f'Saved: {len(self.saved_positions)}')
        
        # Auto-increment position name
        try:
            # Extract number from name like "Position 1"
            parts = self.current_position_name.get().rsplit(' ', 1)
            if len(parts) == 2 and parts[1].isdigit():
                next_num = int(parts[1]) + 1
                self.current_position_name.set(f'{parts[0]} {next_num}')
        except:
            pass
        
        self.ros_node.get_logger().info(
            f'Saved position: {position_data["name"]} '
            f'(Total: {len(self.saved_positions)})'
        )
    
    def export_positions_to_json(self):
        """Export saved positions to JSON file."""
        import json
        from tkinter import filedialog
        
        if not self.saved_positions:
            self.ros_node.get_logger().warn('No positions to export!')
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension='.json',
            filetypes=[('JSON files', '*.json'), ('All files', '*.*')],
            initialdir='./record_data',
            initialfile='saved_positions.json'
        )
        
        if filename:
            with open(filename, 'w') as f:
                json.dump(self.saved_positions, f, indent=2)
            
            self.ros_node.get_logger().info(
                f'Exported {len(self.saved_positions)} positions to {filename}'
            )
    
    def load_positions_from_json(self):
        """Load positions from JSON file."""
        import json
        from tkinter import filedialog
        
        filename = filedialog.askopenfilename(
            filetypes=[('JSON files', '*.json'), ('All files', '*.*')],
            initialdir='./record_data'
        )
        
        if filename:
            try:
                with open(filename, 'r') as f:
                    self.saved_positions = json.load(f)
                
                self.position_counter_label.config(text=f'Saved: {len(self.saved_positions)}')
                
                self.ros_node.get_logger().info(
                    f'Loaded {len(self.saved_positions)} positions from {filename}'
                )
            except Exception as e:
                self.ros_node.get_logger().error(f'Failed to load: {e}')
    
    def clear_all_positions(self):
        """Clear all saved positions."""
        if self.saved_positions:
            self.saved_positions.clear()
            self.position_counter_label.config(text='Saved: 0')
            self.ros_node.get_logger().info('Cleared all saved positions')
    
    def export_to_replay_format(self):
        """Export positions to replay format compatible with record_replay_commands.py."""
        import json
        from tkinter import filedialog, simpledialog
        
        if not self.saved_positions:
            self.ros_node.get_logger().warn('No positions to export!')
            return
        
        # Ask for duration between positions
        duration = simpledialog.askfloat(
            'Duration', 
            'Duration between positions (seconds):',
            initialvalue=2.0,
            minvalue=0.1,
            maxvalue=10.0
        )
        
        if duration is None:
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension='.json',
            filetypes=[('JSON files', '*.json'), ('All files', '*.*')],
            initialdir='./record_data',
            initialfile='replay_sequence.json'
        )
        
        if not filename:
            return
        
        # Convert to replay format
        recordings = []
        current_time = 0.0
        
        for pos_data in self.saved_positions:
            # Add left arm command
            recordings.append({
                'timestamp': current_time,
                'controller': 'left_arm',
                'positions': pos_data['left_arm']
            })
            
            # Add right arm command (same timestamp for simultaneous movement)
            recordings.append({
                'timestamp': current_time,
                'controller': 'right_arm',
                'positions': pos_data['right_arm']
            })
            
            # Add right LEAP Hand command (same timestamp for simultaneous movement)
            recordings.append({
                'timestamp': current_time,
                'controller': 'right_leaphand',
                'positions': pos_data['right_hand']
            })
            
            current_time += duration
        
        replay_data = {
            'duration': current_time,
            'total_commands': len(recordings),
            'recordings': recordings,
            'metadata': {
                'source': 'bimanual_gui_controller',
                'positions_count': len(self.saved_positions),
                'duration_between_positions': duration
            }
        }
        
        with open(filename, 'w') as f:
            json.dump(replay_data, f, indent=2)
        
        self.ros_node.get_logger().info(
            f'Exported {len(self.saved_positions)} positions to replay format: {filename}'
        )
        self.ros_node.get_logger().info(
            f'Total duration: {current_time:.1f}s, Commands: {len(recordings)}'
        )
    
    def test_send_hand_interpolation_btn(self):
        # Run interpolation in a thread to avoid blocking GUI
        threading.Thread(target=self.ros_node.test_send_hand_interpolation, args=(16,), daemon=True).start()
    
    def run(self):
        """Run the GUI."""
        self.root.mainloop()


class ROSNode(Node):
    def __init__(self):
        super().__init__('bimanual_gui_controller')
        
        # Joint names
        self.left_arm_joints = [
            'openarm_left_joint1', 'openarm_left_joint2', 'openarm_left_joint3',
            'openarm_left_joint4', 'openarm_left_joint5', 'openarm_left_joint6',
            'openarm_left_joint7'
        ]
        
        self.right_arm_joints = [
            'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3',
            'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6',
            'openarm_right_joint7'
        ]
        
        self.right_hand_joints = [
            'right_index_mcp_side', 'right_index_mcp_forward', 'right_index_pip', 'right_index_dip',
            'right_middle_mcp_side', 'right_middle_mcp_forward', 'right_middle_pip', 'right_middle_dip',
            'right_ring_mcp_side', 'right_ring_mcp_forward', 'right_ring_pip', 'right_ring_dip',
            'right_thumb_mcp_side', 'right_thumb_mcp_forward', 'right_thumb_pip_joint', 'right_thumb_dip_joint'
        ]
        
        # Action clients for all controllers
        self.left_arm_client = ActionClient(
            self, FollowJointTrajectory, '/left_joint_trajectory_controller/follow_joint_trajectory'
        )
        self.right_arm_client = ActionClient(
            self, FollowJointTrajectory, '/right_joint_trajectory_controller/follow_joint_trajectory'
        )
        self.right_hand_client = ActionClient(
            self, FollowJointTrajectory, '/right_hand_controller/follow_joint_trajectory'
        )
        
        # Joint state storage
        self.current_joint_states = {}
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.get_logger().info('ROS Node initialized')
    
    def joint_state_callback(self, msg):
        """Handle joint state updates."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = msg.position[i]
    
    def get_current_joint_states(self):
        """Get current joint states."""
        return self.current_joint_states.copy()
    
    def publish_positions(self, controller, positions):
        """Send positions via action client (single point trajectory)."""
        # Create goal message
        goal_msg = FollowJointTrajectory.Goal()
        
        # Set joint names based on controller
        if controller == 'left_arm':
            goal_msg.trajectory.joint_names = self.left_arm_joints
            action_client = self.left_arm_client
        elif controller == 'right_arm':
            goal_msg.trajectory.joint_names = self.right_arm_joints
            action_client = self.right_arm_client
        elif controller == 'right_hand':
            goal_msg.trajectory.joint_names = self.right_hand_joints
            action_client = self.right_hand_client
        else:
            return
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=0, nanosec=100000000)  # 100ms
        
        goal_msg.trajectory.points = [point]
        
        # Send goal asynchronously (non-blocking)
        action_client.send_goal_async(goal_msg)
    
    def test_send_hand_interpolation(self, steps=16):
        import numpy as np
        from trajectory_msgs.msg import JointTrajectoryPoint
        from builtin_interfaces.msg import Duration

        open_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.03, 0.0, 0.0, 0.0])
        closed_pose = np.array([0.00159, 1.46159, 0.17559, 0.52159, 0.00159, 1.46159, 0.17559, 0.52159, 0.00159, 1.46159, 0.17559, 0.52159, 1.70159, 0.00159, 0.43759, 0.17159])

        points = []
        for i in range(steps):
            alpha = i / (steps - 1)
            interp_pose = (1 - alpha) * open_pose + alpha * closed_pose
            point = JointTrajectoryPoint()
            point.positions = interp_pose.tolist()
            point.time_from_start = Duration(sec=0, nanosec=int(1e9 * i / 32))  # 16fps
            points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.right_hand_joints
        goal_msg.trajectory.points = points

        self.right_hand_client.send_goal_async(goal_msg)
        self.get_logger().info(f'Sent trajectory with {steps} points to right_hand_controller')
def main(args=None):
    rclpy.init(args=args)
    
    # Create ROS node
    ros_node = ROSNode()
    
    # Spin ROS node in separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()
    
    # Create and run GUI (blocks until window closed)
    gui = BimanualGUIController(ros_node)
    gui.run()
    
    # Cleanup
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
