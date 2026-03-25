#!/usr/bin/env python3
"""
Topic-based test script for O6 bimanual hands.

This script sends test positions to both left and right O6 hands using topic commands.
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class BimanualHandTopicTester(Node):
    def __init__(self):
        super().__init__('bimanual_hand_topic_tester')
        self.right_pub = self.create_publisher(
            Float64MultiArray,
            '/right_hand_forward_position_controller/commands',
            10
        )
        self.left_pub = self.create_publisher(
            Float64MultiArray,
            '/left_hand_forward_position_controller/commands',
            10
        )
        self.get_logger().info('Publishers ready!')

    def send_positions(self, right_positions, left_positions):
        right_msg = Float64MultiArray()
        right_msg.data = right_positions
        left_msg = Float64MultiArray()
        left_msg.data = left_positions
        self.right_pub.publish(right_msg)
        self.left_pub.publish(left_msg)
        self.get_logger().info(f'Sent positions:')
        self.get_logger().info(f'  Right: {right_positions}')
        self.get_logger().info(f'  Left:  {left_positions}')


def main():
    menu = [
        ('open', 'Open both hands'),
        ('close', 'Close both hands'),
        ('grasp', 'Medium grip'),
        ('point', 'Point with index fingers'),
        ('mirror', 'Right closes, left opens'),
    ]
    print('O6 Bimanual Hands Test Menu:')
    for idx, (cmd, desc) in enumerate(menu, 1):
        print(f'  {idx}. {cmd:<8} - {desc}')
    try:
        sel = input('Enter option number (1-5): ').strip()
        if not sel.isdigit() or not (1 <= int(sel) <= len(menu)):
            print('Invalid option, please rerun.')
            sys.exit(1)
        command = menu[int(sel)-1][0]
    except Exception:
        print('Input error, please rerun.')
        sys.exit(1)

    rclpy.init()
    tester = BimanualHandTopicTester()
    try:
        if command == 'open':
            tester.send_positions(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
        elif command == 'close':
            tester.send_positions(
                [0.5, 1.2, 1.5, 1.5, 1.5, 1.5],
                [0.5, 1.2, 1.5, 1.5, 1.5, 1.5]
            )
        elif command == 'grasp':
            tester.send_positions(
                [0.3, 0.7, 0.8, 0.8, 0.8, 0.8],
                [0.3, 0.7, 0.8, 0.8, 0.8, 0.8]
            )
        elif command == 'point':
            tester.send_positions(
                [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]
            )
        elif command == 'mirror':
            tester.send_positions(
                [0.5, 1.2, 1.5, 1.5, 1.5, 1.5],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
        rclpy.spin_once(tester, timeout_sec=0.2)  # 保證訊息送出
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
