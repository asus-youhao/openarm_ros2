#!/usr/bin/env python3
import argparse
from typing import List, Dict

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


RIGHT_HAND_JOINTS = [
    'right_index_mcp_side',
    'right_index_mcp_forward',
    'right_index_pip',
    'right_index_dip',
    'right_middle_mcp_side',
    'right_middle_mcp_forward',
    'right_middle_pip',
    'right_middle_dip',
    'right_ring_mcp_side',
    'right_ring_mcp_forward',
    'right_ring_pip',
    'right_ring_dip',
    'right_thumb_mcp_side',
    'right_thumb_mcp_forward',
    'right_thumb_pip_joint',
    'right_thumb_dip_joint',
]


def build_positions_from_dict(joint_order: List[str], values: Dict[str, float]) -> List[float]:
    return [float(values.get(name, 0.0)) for name in joint_order]


class RightHandTrajectoryClient(Node):
    def __init__(self, server_name: str = '/right_hand_controller/follow_joint_trajectory'):
        super().__init__('right_hand_trajectory_client')
        self.client = ActionClient(self, FollowJointTrajectory, server_name)

    def send_goal_and_wait(self, positions: List[float], duration_sec: float) -> bool:
        if len(positions) != len(RIGHT_HAND_JOINTS):
            self.get_logger().error(f'Expected {len(RIGHT_HAND_JOINTS)} positions, got {len(positions)}')
            return False

        self.get_logger().info(f'Waiting for action server...')
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = RIGHT_HAND_JOINTS

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        sec = int(duration_sec)
        nsec = int((duration_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nsec)
        goal_msg.trajectory.points = [point]

        self.get_logger().info('Sending goal...')
        send_future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result is None:
            self.get_logger().error('No result received')
            return False

        self.get_logger().info(f'Result received: {result.result.error_code}')
        return True


def preset_positions(name: str) -> List[float]:
    name = (name or '').lower()
    if name == 'open':
        values = {
            'right_index_mcp_side': 0.0,
            'right_index_mcp_forward': 0.0,
            'right_index_pip': 0.0,
            'right_index_dip': 0.0,
            'right_middle_mcp_side': 0.0,
            'right_middle_mcp_forward': 0.0,
            'right_middle_pip': 0.0,
            'right_middle_dip': 0.0,
            'right_ring_mcp_side': 0.0,
            'right_ring_mcp_forward': 0.0,
            'right_ring_pip': 0.0,
            'right_ring_dip': 0.0,
            'right_thumb_mcp_side': 1.57,
            'right_thumb_mcp_forward': 0.0,
            'right_thumb_pip_joint': 0.0,
            'right_thumb_dip_joint': 0.0,
        }
        return build_positions_from_dict(RIGHT_HAND_JOINTS, values)
    if name == 'closed':
        values = {
            'right_index_mcp_side': 0.0,
            'right_index_mcp_forward': 1.46,
            'right_index_pip': 0.174,
            'right_index_dip': 0.52,
            'right_middle_mcp_side': 0.0,
            'right_middle_mcp_forward': 1.46,
            'right_middle_pip': 0.174,
            'right_middle_dip': 0.52,
            'right_ring_mcp_side': 0.0,
            'right_ring_mcp_forward': 1.46,
            'right_ring_pip': 0.174,
            'right_ring_dip': 0.52,
            'right_thumb_mcp_side': 1.7,
            'right_thumb_mcp_forward': 0.0,
            'right_thumb_pip_joint': 0.436,
            'right_thumb_dip_joint': 0.17,
        }
        return build_positions_from_dict(RIGHT_HAND_JOINTS, values)
    if name == 'grasp':
        values = {
            # Index
            'right_index_mcp_side': 0.0,
            'right_index_mcp_forward': 0.0,
            'right_index_pip': 0.525,
            'right_index_dip': 1.11,
            # Middle
            'right_middle_mcp_side': 0.0,
            'right_middle_mcp_forward': 0.0,
            'right_middle_pip': 0.525,
            'right_middle_dip': 1.11,
            # Ring
            'right_ring_mcp_side': 0.0,
            'right_ring_mcp_forward': 0.0,
            'right_ring_pip': 0.525,
            'right_ring_dip': 1.11,
            # Thumb
            'right_thumb_mcp_forward': 1.46,
            'right_thumb_mcp_side': 0.0,
            'right_thumb_pip_joint': 0.56,
            'right_thumb_dip_joint': 0.71,
        }
        return build_positions_from_dict(RIGHT_HAND_JOINTS, values)
    return [0.0] * len(RIGHT_HAND_JOINTS)


def parse_args():
    parser = argparse.ArgumentParser(description='Send a trajectory goal to right hand controller')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--preset', choices=['open', 'closed', 'grasp'], help='Use a preset hand pose')
    group.add_argument('--positions', type=float, nargs=16, metavar='P', help='16 joint positions (radians)')
    parser.add_argument('--duration', type=float, default=0.8, help='Time to reach target (seconds)')
    parser.add_argument('--server', type=str, default='/right_hand_controller/follow_joint_trajectory',
                        help='Action server name')
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = RightHandTrajectoryClient(server_name=args.server)

    if args.positions is not None:
        positions = list(args.positions)
    else:
        positions = preset_positions(args.preset or 'open')

    ok = node.send_goal_and_wait(positions, args.duration)
    node.destroy_node()
    rclpy.shutdown()
    if not ok:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
