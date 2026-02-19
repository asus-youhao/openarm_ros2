#!/usr/bin/env python3
"""
Synchronized Action Chunk Controller for OpenArm Bimanual System with LEAP Hand.

This controller receives action chunks from VLA models (e.g., GR00T N1.5) and
executes them with proper timestamp-based synchronization between arms and hands.

Key Features:
- Receives 16-step action chunks with timestamps
- Sends synchronized multi-point trajectories to all controllers
- Ensures arm and hand commands are executed atomically
- Respects GR00T model's timestamps for each step

Usage:
  python3 action_chunk_controller.py

The controller subscribes to action chunk topics and sends synchronized
trajectory commands to:
  - /left_joint_trajectory_controller/follow_joint_trajectory
  - /right_joint_trajectory_controller/follow_joint_trajectory
  - /right_hand_controller/follow_joint_trajectory
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import threading
import json
from typing import List, Dict, Optional
from dataclasses import dataclass
import time


@dataclass
class ActionChunk:
    """Represents a 16-step action chunk from the VLA model."""
    timestamps: List[float]  # Timestamp for each step (seconds from chunk start)
    left_arm_positions: List[List[float]]   # 16 steps x 7 joints
    right_arm_positions: List[List[float]]  # 16 steps x 7 joints
    right_hand_positions: List[List[float]] # 16 steps x 16 joints
    chunk_id: int = 0
    received_time: float = 0.0


class ActionChunkController(Node):
    """
    Synchronized controller for action chunk execution.
    
    This controller ensures:
    1. All three controllers (left arm, right arm, hand) receive commands simultaneously
    2. Multi-point trajectories are sent with proper timestamps
    3. Action chunks are queued and executed in order
    """
    
    # Joint names
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
        super().__init__('action_chunk_controller')
        
        # Use reentrant callback group for concurrent action execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Action clients for all controllers
        self.left_arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/left_joint_trajectory_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        self.right_arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/right_joint_trajectory_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        self.right_hand_client = ActionClient(
            self, FollowJointTrajectory,
            '/right_hand_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        # Action chunk queue
        self.chunk_queue: List[ActionChunk] = []
        self.chunk_lock = threading.Lock()
        self.chunk_id_counter = 0
        
        # Current execution state
        self.executing = False
        self.execution_lock = threading.Lock()
        
        # Joint state subscriber for current positions
        self.current_joint_states: Dict[str, float] = {}
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # Subscribe to action chunk topic (for VLA model integration)
        # The VLA model should publish to these topics
        self.chunk_sub = self.create_subscription(
            Float64MultiArray,
            '/action_chunk',
            self.action_chunk_callback,
            10
        )
        
        # Wait for action servers
        self.get_logger().info('Waiting for action servers...')
        self._wait_for_servers()
        self.get_logger().info('All action servers ready!')
        
        # Timer for processing chunk queue
        self.process_timer = self.create_timer(0.1, self.process_chunk_queue)
        
        self.get_logger().info('Action Chunk Controller initialized')
        self.get_logger().info(f'  Left arm joints: {len(self.LEFT_ARM_JOINTS)}')
        self.get_logger().info(f'  Right arm joints: {len(self.RIGHT_ARM_JOINTS)}')
        self.get_logger().info(f'  Right hand joints: {len(self.RIGHT_HAND_JOINTS)}')
    
    def _wait_for_servers(self, timeout: float = 10.0):
        """Wait for all action servers to be available."""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            left_ready = self.left_arm_client.wait_for_server(timeout_sec=0.1)
            right_ready = self.right_arm_client.wait_for_server(timeout_sec=0.1)
            hand_ready = self.right_hand_client.wait_for_server(timeout_sec=0.1)
            
            if left_ready and right_ready and hand_ready:
                return True
        
        missing = []
        if not left_ready:
            missing.append('left_arm')
        if not right_ready:
            missing.append('right_arm')
        if not hand_ready:
            missing.append('right_hand')
        
        self.get_logger().error(f'Servers not available: {missing}')
        return False
    
    def joint_state_callback(self, msg: JointState):
        """Store current joint states."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = msg.position[i]
    
    def action_chunk_callback(self, msg: Float64MultiArray):
        """
        Receive action chunk from VLA model.
        
        Expected format:
        - First value: chunk_id
        - Next 16 values: timestamps for each step
        - Next 16*7 values: left arm positions (16 steps x 7 joints)
        - Next 16*7 values: right arm positions (16 steps x 7 joints)
        - Next 16*16 values: right hand positions (16 steps x 16 joints)
        
        Total: 1 + 16 + 16*7 + 16*7 + 16*16 = 1 + 16 + 112 + 112 + 256 = 497 values
        """
        data = msg.data
        
        if len(data) < 497:
            self.get_logger().error(f'Invalid action chunk size: {len(data)}, expected 497')
            return
        
        try:
            chunk_id = int(data[0])
            timestamps = list(data[1:17])
            
            # Extract left arm positions (16 steps x 7 joints)
            left_start = 17
            left_end = left_start + 16 * 7
            left_arm_flat = data[left_start:left_end]
            left_arm_positions = [
                list(left_arm_flat[i*7:(i+1)*7]) for i in range(16)
            ]
            
            # Extract right arm positions (16 steps x 7 joints)
            right_start = left_end
            right_end = right_start + 16 * 7
            right_arm_flat = data[right_start:right_end]
            right_arm_positions = [
                list(right_arm_flat[i*7:(i+1)*7]) for i in range(16)
            ]
            
            # Extract right hand positions (16 steps x 16 joints)
            hand_start = right_end
            hand_end = hand_start + 16 * 16
            hand_flat = data[hand_start:hand_end]
            right_hand_positions = [
                list(hand_flat[i*16:(i+1)*16]) for i in range(16)
            ]
            
            chunk = ActionChunk(
                timestamps=timestamps,
                left_arm_positions=left_arm_positions,
                right_arm_positions=right_arm_positions,
                right_hand_positions=right_hand_positions,
                chunk_id=chunk_id,
                received_time=time.time()
            )
            
            with self.chunk_lock:
                self.chunk_queue.append(chunk)
            
            self.get_logger().info(
                f'Received action chunk {chunk_id} with {len(timestamps)} steps'
            )
            
        except Exception as e:
            self.get_logger().error(f'Failed to parse action chunk: {e}')
    
    def process_chunk_queue(self):
        """Process action chunks from queue."""
        with self.execution_lock:
            if self.executing:
                return
        
        with self.chunk_lock:
            if not self.chunk_queue:
                return
            chunk = self.chunk_queue.pop(0)
        
        self.execute_action_chunk(chunk)
    
    def execute_action_chunk(self, chunk: ActionChunk):
        """
        Execute an action chunk by sending synchronized multi-point trajectories.
        
        This sends the entire 16-step trajectory to all controllers simultaneously,
        ensuring synchronized execution based on timestamps.
        """
        with self.execution_lock:
            self.executing = True
        
        self.get_logger().info(f'Executing action chunk {chunk.chunk_id}')
        
        # Build trajectory points from action chunk
        left_points = self._build_trajectory_points(chunk.timestamps, chunk.left_arm_positions)
        right_points = self._build_trajectory_points(chunk.timestamps, chunk.right_arm_positions)
        hand_points = self._build_trajectory_points(chunk.timestamps, chunk.right_hand_positions)
        
        # Send trajectories to all controllers (synchronously)
        futures = []
        
        # Send left arm trajectory
        left_future = self._send_trajectory_async(
            self.left_arm_client, self.LEFT_ARM_JOINTS, left_points
        )
        futures.append(('left_arm', left_future))
        
        # Send right arm trajectory
        right_future = self._send_trajectory_async(
            self.right_arm_client, self.RIGHT_ARM_JOINTS, right_points
        )
        futures.append(('right_arm', right_future))
        
        # Send right hand trajectory
        hand_future = self._send_trajectory_async(
            self.right_hand_client, self.RIGHT_HAND_JOINTS, hand_points
        )
        futures.append(('right_hand', hand_future))
        
        self.get_logger().info(
            f'Sent synchronized trajectories for chunk {chunk.chunk_id}: '
            f'{len(left_points)} points, duration={chunk.timestamps[-1]:.3f}s'
        )
        
        # Wait for all goals to be accepted (non-blocking check)
        # The actual execution continues in background
        rclpy.spin_until_future_complete(self, left_future, timeout_sec=0.1)
        rclpy.spin_until_future_complete(self, right_future, timeout_sec=0.1)
        rclpy.spin_until_future_complete(self, hand_future, timeout_sec=0.1)
        
        with self.execution_lock:
            self.executing = False
    
    def _build_trajectory_points(
        self, 
        timestamps: List[float], 
        positions: List[List[float]]
    ) -> List[JointTrajectoryPoint]:
        """
        Build trajectory points from timestamps and positions.
        
        Args:
            timestamps: List of timestamps for each step (seconds)
            positions: List of joint positions for each step
            
        Returns:
            List of JointTrajectoryPoint messages
        """
        points = []
        
        for i, (ts, pos) in enumerate(zip(timestamps, positions)):
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in pos]
            
            # Convert timestamp to Duration
            sec = int(ts)
            nanosec = int((ts - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            
            # Add velocities for smoother interpolation (optional)
            if i > 0:
                dt = timestamps[i] - timestamps[i-1]
                if dt > 0:
                    point.velocities = [
                        (positions[i][j] - positions[i-1][j]) / dt
                        for j in range(len(pos))
                    ]
            
            points.append(point)
        
        return points
    
    def _send_trajectory_async(
        self,
        action_client: ActionClient,
        joint_names: List[str],
        points: List[JointTrajectoryPoint]
    ):
        """Send trajectory goal asynchronously."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points
        
        return action_client.send_goal_async(goal_msg)
    
    def send_action_chunk_from_dict(self, chunk_data: Dict):
        """
        Alternative method to receive action chunk as dictionary.
        
        Useful for testing or integration with different data formats.
        
        Args:
            chunk_data: Dictionary with keys:
                - timestamps: List[float]
                - left_arm: List[List[float]]
                - right_arm: List[List[float]]
                - right_hand: List[List[float]]
        """
        chunk = ActionChunk(
            timestamps=chunk_data.get('timestamps', [i * 0.05 for i in range(16)]),
            left_arm_positions=chunk_data.get('left_arm', [[0.0] * 7] * 16),
            right_arm_positions=chunk_data.get('right_arm', [[0.0] * 7] * 16),
            right_hand_positions=chunk_data.get('right_hand', [[0.0] * 16] * 16),
            chunk_id=self.chunk_id_counter,
            received_time=time.time()
        )
        self.chunk_id_counter += 1
        
        with self.chunk_lock:
            self.chunk_queue.append(chunk)
        
        self.get_logger().info(f'Queued action chunk {chunk.chunk_id} from dict')
    
    def load_action_chunks_from_file(self, filepath: str):
        """Load action chunks from JSON file for replay."""
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            chunks = data.get('chunks', [data])  # Support single chunk or list
            
            for chunk_data in chunks:
                self.send_action_chunk_from_dict(chunk_data)
            
            self.get_logger().info(f'Loaded {len(chunks)} chunks from {filepath}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load chunks: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    # Create controller node
    controller = ActionChunkController()
    
    # Use multi-threaded executor for concurrent action handling
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down Action Chunk Controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()