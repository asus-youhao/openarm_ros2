#!/usr/bin/env python3
"""
GR00T State Publisher for OpenArm Bimanual System with LEAP Hand.

This node collects joint states from the hardware interface and publishes them
at a configurable rate (default 50Hz) suitable for GR00T N1.5 VLA model input.

The node subscribes to /joint_states and provides:
1. Filtered/smoothed joint states for VLA feedback
2. Efficient message format for network transmission
3. Health monitoring and latency tracking

Key Features:
- Configurable publishing rate (default 50Hz)
- Low-pass filtering for smoother state feedback
- Efficient Float64MultiArray format for GR00T
- Health monitoring and statistics

Usage:
  python3 groot_state_publisher.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Header
from builtin_interfaces.msg import Time

import threading
import time
from typing import Dict, List, Optional
from dataclasses import dataclass, field
from collections import deque
import statistics


# ============== CONFIGURABLE RATES ==============
# These can be modified based on system requirements
GROOT_FEEDBACK_RATE_HZ = 50.0  # Publishing rate for GR00T feedback
STATE_BUFFER_SIZE = 10         # Number of states to keep for filtering
LOW_PASS_ALPHA = 0.3           # Low-pass filter coefficient (0-1)


@dataclass
class JointStateData:
    """Container for joint state data with timestamp."""
    positions: List[float] = field(default_factory=list)
    velocities: List[float] = field(default_factory=list)
    efforts: List[float] = field(default_factory=list)
    timestamp: float = 0.0


class LowPassFilter:
    """Simple exponential moving average filter."""
    
    def __init__(self, alpha: float = 0.3):
        self.alpha = alpha
        self.filtered_values: Optional[List[float]] = None
        self.initialized = False
    
    def update(self, new_values: List[float]) -> List[float]:
        """Update filter with new values and return filtered result."""
        if not self.initialized:
            self.filtered_values = new_values.copy()
            self.initialized = True
        else:
            for i in range(len(new_values)):
                self.filtered_values[i] = (
                    self.alpha * new_values[i] + 
                    (1 - self.alpha) * self.filtered_values[i]
                )
        return self.filtered_values.copy()
    
    def reset(self):
        """Reset filter state."""
        self.filtered_values = None
        self.initialized = False


class HealthMonitor:
    """Monitor system health and communication quality."""
    
    def __init__(self, window_size: int = 100):
        self.window_size = window_size
        self.latency_history: deque = deque(maxlen=window_size)
        self.last_msg_time: float = 0.0
        self.msg_count: int = 0
        self.timeout_count: int = 0
        self.start_time: float = time.time()
    
    def record_message(self, latency_ms: float):
        """Record a message with its latency."""
        self.latency_history.append(latency_ms)
        self.msg_count += 1
        self.last_msg_time = time.time()
    
    def record_timeout(self):
        """Record a timeout event."""
        self.timeout_count += 1
    
    def get_statistics(self) -> Dict:
        """Get health statistics."""
        stats = {
            'total_messages': self.msg_count,
            'timeouts': self.timeout_count,
            'uptime_s': time.time() - self.start_time,
            'avg_latency_ms': 0.0,
            'max_latency_ms': 0.0,
            'min_latency_ms': 0.0,
            'jitter_ms': 0.0,
        }
        
        if self.latency_history:
            stats['avg_latency_ms'] = statistics.mean(self.latency_history)
            stats['max_latency_ms'] = max(self.latency_history)
            stats['min_latency_ms'] = min(self.latency_history)
            if len(self.latency_history) > 1:
                stats['jitter_ms'] = statistics.stdev(self.latency_history)
        
        return stats


class GR00TStatePublisher(Node):
    """
    Publishes filtered joint states for GR00T VLA model.
    
    Subscribes to /joint_states and publishes:
    - /groot/joint_states: Float64MultiArray with all joint positions
    - /groot/state_health: Health statistics
    """
    
    # Joint names in the expected order for GR00T
    LEFT_ARM_JOINTS = [
        'openarm_left_joint1', 'openarm_left_joint2', 'openarm_left_joint3',
        'openarm_left_joint4', 'openarm_left_joint5', 'openarm_left_joint6',
        'openarm_left_joint7'
    ]
    
    RIGHT_ARM_JOINTS = [
        'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3',
        'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6',
        'openarm_right_joint7'
    ]
    
    RIGHT_HAND_JOINTS = [
        'right_index_mcp_side', 'right_index_mcp_forward', 'right_index_pip', 'right_index_dip',
        'right_middle_mcp_side', 'right_middle_mcp_forward', 'right_middle_pip', 'right_middle_dip',
        'right_ring_mcp_side', 'right_ring_mcp_forward', 'right_ring_pip', 'right_ring_dip',
        'right_thumb_mcp_side', 'right_thumb_mcp_forward', 'right_thumb_pip_joint', 'right_thumb_dip_joint'
    ]
    
    def __init__(self):
        super().__init__('groot_state_publisher')
        
        # Declare parameters
        self.declare_parameter('feedback_rate_hz', GROOT_FEEDBACK_RATE_HZ)
        self.declare_parameter('low_pass_alpha', LOW_PASS_ALPHA)
        self.declare_parameter('enable_filtering', True)
        self.declare_parameter('health_check_interval_s', 5.0)
        
        # Get parameters
        self.feedback_rate = self.get_parameter('feedback_rate_hz').value
        self.low_pass_alpha = self.get_parameter('low_pass_alpha').value
        self.enable_filtering = self.get_parameter('enable_filtering').value
        self.health_check_interval = self.get_parameter('health_check_interval_s').value
        
        # Calculate total joints: 7 (left arm) + 7 (right arm) + 16 (right hand) = 30
        self.total_joints = len(self.LEFT_ARM_JOINTS) + len(self.RIGHT_ARM_JOINTS) + len(self.RIGHT_HAND_JOINTS)
        
        # Current state storage
        self.current_states: Dict[str, float] = {}
        self.state_lock = threading.Lock()
        self.last_update_time: float = 0.0
        
        # Low-pass filter
        self.position_filter = LowPassFilter(self.low_pass_alpha)
        
        # Health monitor
        self.health_monitor = HealthMonitor()
        
        # QoS profile for joint states
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile
        )
        
        # Publisher for GR00T feedback
        self.groot_state_pub = self.create_publisher(
            Float64MultiArray,
            '/groot/joint_states',
            10
        )
        
        # Publisher for health statistics
        self.health_pub = self.create_publisher(
            Float64MultiArray,
            '/groot/state_health',
            10
        )
        
        # Timer for publishing at fixed rate
        self.publish_timer = self.create_timer(
            1.0 / self.feedback_rate,
            self.publish_state_callback
        )
        
        # Timer for health monitoring
        self.health_timer = self.create_timer(
            self.health_check_interval,
            self.health_check_callback
        )
        
        self.get_logger().info(
            f'GR00T State Publisher initialized:'
            f'\n  Feedback rate: {self.feedback_rate} Hz'
            f'\n  Total joints: {self.total_joints}'
            f'\n  Filtering: {"enabled" if self.enable_filtering else "disabled"}'
        )
    
    def joint_state_callback(self, msg: JointState):
        """Process incoming joint state messages."""
        receive_time = time.time()
        
        # Calculate latency (if header has timestamp)
        if msg.header.stamp.sec > 0 or msg.header.stamp.nanosec > 0:
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            latency_ms = (receive_time - msg_time) * 1000.0
            self.health_monitor.record_message(latency_ms)
        else:
            self.health_monitor.record_message(0.0)
        
        # Update state storage
        with self.state_lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self.current_states[name] = msg.position[i]
            self.last_update_time = receive_time
    
    def publish_state_callback(self):
        """Publish filtered state for GR00T at fixed rate."""
        # Get current positions in correct order
        positions = self._get_ordered_positions()
        
        if not positions:
            self.get_logger().warn('No joint states received yet')
            return
        
        # Apply low-pass filter if enabled
        if self.enable_filtering:
            positions = self.position_filter.update(positions)
        
        # Create and publish message
        msg = Float64MultiArray()
        
        # Format: [timestamp_sec, timestamp_nanosec, pos1, pos2, ...]
        now = self.get_clock().now()
        msg.data = [
            float(now.seconds_nanoseconds()[0]),  # timestamp_sec
            float(now.seconds_nanoseconds()[1]),  # timestamp_nanosec
        ]
        msg.data.extend(positions)
        
        self.groot_state_pub.publish(msg)
    
    def _get_ordered_positions(self) -> List[float]:
        """Get joint positions in the expected order for GR00T."""
        with self.state_lock:
            positions = []
            
            # Left arm joints (7)
            for name in self.LEFT_ARM_JOINTS:
                positions.append(self.current_states.get(name, 0.0))
            
            # Right arm joints (7)
            for name in self.RIGHT_ARM_JOINTS:
                positions.append(self.current_states.get(name, 0.0))
            
            # Right hand joints (16)
            for name in self.RIGHT_HAND_JOINTS:
                positions.append(self.current_states.get(name, 0.0))
            
            return positions
    
    def health_check_callback(self):
        """Periodic health check and statistics reporting."""
        stats = self.health_monitor.get_statistics()
        
        # Create health message
        msg = Float64MultiArray()
        msg.data = [
            float(stats['total_messages']),
            float(stats['timeouts']),
            stats['avg_latency_ms'],
            stats['max_latency_ms'],
            stats['min_latency_ms'],
            stats['jitter_ms'],
            stats['uptime_s'],
        ]
        
        self.health_pub.publish(msg)
        
        # Log health status
        self.get_logger().info(
            f'Health: msgs={stats["total_messages"]}, '
            f'latency={stats["avg_latency_ms"]:.2f}ms (max={stats["max_latency_ms"]:.2f}), '
            f'jitter={stats["jitter_ms"]:.2f}ms'
        )
        
        # Check for issues
        if stats['avg_latency_ms'] > 20.0:
            self.get_logger().warn(
                f'High average latency: {stats["avg_latency_ms"]:.2f}ms'
            )
        
        if stats['jitter_ms'] > 10.0:
            self.get_logger().warn(
                f'High jitter detected: {stats["jitter_ms"]:.2f}ms'
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = GR00TStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down GR00T State Publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()