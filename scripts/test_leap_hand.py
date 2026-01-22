#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class LeapHandPublisher(Node):
    def __init__(self):
        super().__init__('leap_hand_publisher')
        
        # Publisher for ros2_control interface
        self.controller_pub = self.create_publisher(
            Float64MultiArray, 
            '/right_hand_controller/commands', 
            10)
        
        # Publisher for direct leap_hand interface
        self.leap_pub = self.create_publisher(
            JointState,
            '/cmd_leap',
            10)
        
        # LEAP Hand coordinate system:
        # - 3.14 rad is the home pose (flat/open)
        # - URDF uses 0 rad as home pose
        # - Conversion: URDF_position = LEAP_command - 3.14
        self.LEAP_HOME_OFFSET = 3.14
        
        # Leap hand finger joint positions (LEAP coordinate system)
        # 展開姿勢 (Open/Extended) - LEAP convention: 3.14 = home
        self.open_positions_leap = [3.14] * 16
        
        # 抓取姿勢 (Grasp/Closed) - LEAP coordinate system
        self.grasp_positions_leap = [
            3.14, 3.665, 4.25, 4.0,    # Index finger
            3.14, 3.665, 4.25, 4.0,    # Middle finger
            3.14, 3.665, 4.25, 4.0,    # Ring finger
            4.6, 3.7, 3.85, 4.5        # Thumb
        ]
        
        # Convert to URDF coordinate system (0 = home)
        self.open_positions = [0.0] * 16
        self.grasp_positions = [x - self.LEAP_HOME_OFFSET for x in self.grasp_positions_leap]
        
        # Joint names for leap_hand
        self.joint_names = [
            'right_index_mcp_forward', 'right_index_mcp_side', 'right_index_pip', 'right_index_dip',
            'right_middle_mcp_forward', 'right_middle_mcp_side', 'right_middle_pip', 'right_middle_dip',
            'right_ring_mcp_forward', 'right_ring_mcp_side', 'right_ring_pip', 'right_ring_dip',
            'right_thumb_mcp_side', 'right_thumb_mcp_forward', 'right_thumb_pip_joint', 'right_thumb_dip_joint'
        ]
        
        self.get_logger().info('Leap Hand Publisher initialized')
        
    def publish_to_controller(self, positions):
        """Publish to ros2_control interface (URDF coordinate: 0 = home)"""
        msg = Float64MultiArray()
        msg.data = positions
        self.controller_pub.publish(msg)
        self.get_logger().info(f'Published to /right_hand_controller/commands: {len(positions)} positions (URDF coords)')
        
    def publish_to_leap(self, positions):
        """Publish to direct leap_hand interface (LEAP coordinate: 3.14 = home)"""
        msg = JointState()
        msg.name = self.joint_names
        # Convert URDF coordinates to LEAP coordinates
        leap_positions = [x + self.LEAP_HOME_OFFSET for x in positions]
        msg.position = leap_positions
        self.leap_pub.publish(msg)
        self.get_logger().info(f'Published to /cmd_leap: {len(leap_positions)} positions (LEAP coords)')
    
    def publish_grasp(self, use_controller=True, use_leap=False):
        """Publish grasp posture"""
        self.get_logger().info('Publishing GRASP posture...')
        if use_controller:
            self.publish_to_controller(self.grasp_positions)
        if use_leap:
            self.publish_to_leap(self.grasp_positions)
            
    def publish_open(self, use_controller=True, use_leap=False):
        """Publish open posture"""
        self.get_logger().info('Publishing OPEN posture...')
        if use_controller:
            self.publish_to_controller(self.open_positions)
        if use_leap:
            self.publish_to_leap(self.open_positions)

def main(args=None):
    rclpy.init(args=args)
    node = LeapHandPublisher()
    
    import sys
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        use_leap = '--leap' in sys.argv
        use_controller = '--controller' in sys.argv or not use_leap
        
        if command == 'grasp' or command == 'close':
            node.publish_grasp(use_controller=use_controller, use_leap=use_leap)
        elif command == 'open':
            node.publish_open(use_controller=use_controller, use_leap=use_leap)
        else:
            node.get_logger().error(f'Unknown command: {command}')
            node.get_logger().info('Usage: python3 test_leap_hand.py [grasp|open] [--controller] [--leap]')
    else:
        # Default: publish grasp to controller
        node.publish_grasp(use_controller=True, use_leap=False)
    
    # Keep node alive for a moment to ensure message is sent
    rclpy.spin_once(node, timeout_sec=0.5)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
