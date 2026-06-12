#!/usr/bin/env python3
"""
joint_actions_aggregator.py

Aggregates joint commands from multiple sources into a single /joint_actions topic:
  - Left arm IK commands (7 joints)
  - Right arm IK commands (7 joints)
  - Left hand commands (6 or 16 joints, optional)
  - Right hand commands (6 or 16 joints, optional)

This provides a unified joint command stream for data collection and imitation learning.

Usage:
    # Bimanual arms with O6 hands (both) - DEFAULT
    ros2 run openarm_ros2 joint_actions_aggregator.py
    
    # Bimanual arms with O6 left hand only
    ros2 run openarm_ros2 joint_actions_aggregator.py --hand_config o6_left
    
    # Bimanual arms with O6 right hand only
    ros2 run openarm_ros2 joint_actions_aggregator.py --hand_config o6_right
    
    # Arms only (no hands)
    ros2 run openarm_ros2 joint_actions_aggregator.py --hand_config none
"""

import argparse
import threading
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# ── Joint name definitions ────────────────────────────────────────────────────
LEFT_ARM_JOINTS = [
    "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
    "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
    "openarm_left_joint7",
]

RIGHT_ARM_JOINTS = [
    "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
    "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
    "openarm_right_joint7",
]

# O6 Hand (left) - 6 active joints
# Order must match left_hand_forward_position_controller joints in YAML
O6_LEFT_JOINTS = [
    'L_index_mcp_pitch', 'L_middle_mcp_pitch',
    'L_pinky_mcp_pitch', 'L_ring_mcp_pitch',
    'L_thumb_cmc_pitch', 'L_thumb_cmc_yaw'
]

# O6 Hand (right) - 6 active joints
# Order must match right_hand_forward_position_controller joints in YAML
O6_RIGHT_JOINTS = [
    'R_index_mcp_pitch', 'R_middle_mcp_pitch',
    'R_pinky_mcp_pitch', 'R_ring_mcp_pitch',
    'R_thumb_cmc_pitch', 'R_thumb_cmc_yaw'
]


class JointActionsAggregator(Node):
    """
    Aggregates joint commands from arms and hands into /joint_actions.
    
    Subscriptions (depending on hand_config):
      - /left_arm_ik_commands  (JointState, 7 joints)
      - /right_arm_ik_commands (JointState, 7 joints)
      - /left_hand_forward_position_controller/commands  (Float64MultiArray, 6 joints for O6)
      - /right_hand_forward_position_controller/commands (Float64MultiArray, 6 joints for O6)
    
    Publications:
      - /joint_actions (JointState, aggregated all joints)
    """
    
    def __init__(self, hand_config: str = "o6_both", publish_rate: float = 50.0):
        super().__init__('joint_actions_aggregator')
        
        self.hand_config = hand_config
        self.publish_rate = publish_rate
        
        # Latest commands from each source
        self.left_arm_cmd: Optional[List[float]] = None
        self.right_arm_cmd: Optional[List[float]] = None
        self.left_hand_cmd: Optional[List[float]] = None
        self.right_hand_cmd: Optional[List[float]] = None
        self.lock = threading.Lock()
        
        # Build joint names list based on configuration
        self.all_joint_names = self._build_joint_names()
        
        # Create subscribers
        self._create_subscribers()
        
        # Publisher for aggregated commands
        self.joint_actions_pub = self.create_publisher(JointState, '/joint_actions', 10)
        
        # Timer to publish at fixed rate (independent of input rates)
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_aggregated)
        
        self.msg_count = 0
        self.get_logger().info(f"JointActionsAggregator started")
        self.get_logger().info(f"  Hand config: {hand_config}")
        self.get_logger().info(f"  Total joints: {len(self.all_joint_names)}")
        self.get_logger().info(f"  Publish rate: {publish_rate} Hz")
        self.get_logger().info(f"  Joint order: {self.all_joint_names}")
    
    def _build_joint_names(self) -> List[str]:
        """Build the complete list of joint names based on configuration."""
        names = []
        
        # Always include both arms
        names.extend(LEFT_ARM_JOINTS)   # 7 joints
        names.extend(RIGHT_ARM_JOINTS)  # 7 joints
        
        # Add hands based on config
        if self.hand_config == "o6_left":
            names.extend(O6_LEFT_JOINTS)     # 6 joints
        elif self.hand_config == "o6_right":
            names.extend(O6_RIGHT_JOINTS)    # 6 joints
        elif self.hand_config == "o6_both":
            names.extend(O6_LEFT_JOINTS)     # 6 joints
            names.extend(O6_RIGHT_JOINTS)    # 6 joints
        elif self.hand_config == "none":
            pass  # No hand joints
        else:
            self.get_logger().warn(f"Unknown hand_config '{self.hand_config}', defaulting to 'o6_both'")
            names.extend(O6_LEFT_JOINTS)
            names.extend(O6_RIGHT_JOINTS)
        
        return names
    
    def _create_subscribers(self):
        """Create subscribers based on configuration."""
        # Always subscribe to arm IK commands
        self.create_subscription(
            JointState,
            '/left_arm_ik_commands',
            self._left_arm_callback,
            10
        )
        self.create_subscription(
            JointState,
            '/right_arm_ik_commands',
            self._right_arm_callback,
            10
        )
        
        # Subscribe to hand controllers based on config
        if self.hand_config in ["o6_left", "o6_both"]:
            self.create_subscription(
                Float64MultiArray,
                '/left_hand_forward_position_controller/commands',
                self._left_hand_callback,
                10
            )
        
        if self.hand_config in ["o6_right", "o6_both"]:
            self.create_subscription(
                Float64MultiArray,
                '/right_hand_forward_position_controller/commands',
                self._right_hand_callback,
                10
            )
    
    # ── Callbacks ─────────────────────────────────────────────────────────────
    
    def _left_arm_callback(self, msg: JointState):
        """Receive left arm IK commands (7 joints)."""
        with self.lock:
            self.left_arm_cmd = list(msg.position)
    
    def _right_arm_callback(self, msg: JointState):
        """Receive right arm IK commands (7 joints)."""
        with self.lock:
            self.right_arm_cmd = list(msg.position)
    
    def _left_hand_callback(self, msg: Float64MultiArray):
        """Receive left hand commands (6 joints for O6)."""
        with self.lock:
            self.left_hand_cmd = list(msg.data)
    
    def _right_hand_callback(self, msg: Float64MultiArray):
        """Receive right hand commands (6 joints for O6)."""
        with self.lock:
            self.right_hand_cmd = list(msg.data)
    
    # ── Aggregation and publishing ────────────────────────────────────────────
    
    def _publish_aggregated(self):
        """Aggregate all available commands and publish to /joint_actions."""
        with self.lock:
            # Check if we have required data
            if self.left_arm_cmd is None or self.right_arm_cmd is None:
                # Don't publish until we have at least both arms
                if self.msg_count == 0:
                    self.get_logger().info("Waiting for arm commands...", throttle_duration_sec=5.0)
                return
            
            # Build aggregated position list
            positions = []
            
            # Left arm (7 joints)
            positions.extend(self.left_arm_cmd)
            
            # Right arm (7 joints)
            positions.extend(self.right_arm_cmd)
            
            # Add hands based on config
            if self.hand_config == "o6_left":
                if self.left_hand_cmd is not None:
                    positions.extend(self.left_hand_cmd)
                else:
                    positions.extend([0.0] * len(O6_LEFT_JOINTS))
            
            elif self.hand_config == "o6_right":
                if self.right_hand_cmd is not None:
                    positions.extend(self.right_hand_cmd)
                else:
                    positions.extend([0.0] * len(O6_RIGHT_JOINTS))
            
            elif self.hand_config == "o6_both":
                if self.left_hand_cmd is not None:
                    positions.extend(self.left_hand_cmd)
                else:
                    positions.extend([0.0] * len(O6_LEFT_JOINTS))
                
                if self.right_hand_cmd is not None:
                    positions.extend(self.right_hand_cmd)
                else:
                    positions.extend([0.0] * len(O6_RIGHT_JOINTS))
        
        # Publish aggregated message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.all_joint_names
        msg.position = positions
        
        self.joint_actions_pub.publish(msg)
        
        self.msg_count += 1
        if self.msg_count == 1:
            self.get_logger().info(f"Started publishing to /joint_actions ({len(positions)} joints)")
        elif self.msg_count % 500 == 0:
            self.get_logger().info(f"Published {self.msg_count} messages to /joint_actions")


def main():
    parser = argparse.ArgumentParser(description='Aggregate joint commands to /joint_actions')
    parser.add_argument(
        '--hand_config',
        type=str,
        default='o6_both',
        choices=['none', 'o6_left', 'o6_right', 'o6_both'],
        help='Hand configuration: o6_both (default, O6 both hands), '
             'o6_left (O6 left only), o6_right (O6 right only), '
             'none (arms only, no hands)'
    )
    parser.add_argument(
        '--publish_rate',
        type=float,
        default=50.0,
        help='Publish rate in Hz (default: 50.0)'
    )
    
    args, unknown = parser.parse_known_args()
    
    rclpy.init()
    node = JointActionsAggregator(
        hand_config=args.hand_config,
        publish_rate=args.publish_rate
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
