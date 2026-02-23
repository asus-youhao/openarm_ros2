#!/usr/bin/env python3
"""
Synchronized Action Chunk Controller for OpenArm Bimanual System with LEAP Hand.

This controller receives action chunks from VLA models (e.g., GR00T N1.5) and
executes them with proper timestamp-based synchronization between arms and hands.

Key Design Decisions
--------------------
1. **Preempt-on-new-chunk**: When a new action chunk arrives, any in-flight
   trajectories are immediately cancelled and replaced. This ensures the robot
   always executes the freshest inference result — critical when GR00T inference
   time (50–100 ms) is much shorter than the chunk duration (16 × 1/30 s ≈ 533 ms).

2. **Synchronized start time**: All three FollowJointTrajectory goals are sent with
   the same ``trajectory.header.stamp``. The JointTrajectoryController interprets
   ``time_from_start`` relative to this stamp, so left arm, right arm, and LEAP hand
   all start at exactly the same ROS clock instant.

3. **GR00T N1.5 step dt = 1/30 s**: Each of the 16 action steps covers 33.3 ms.
   If the incoming timestamps are zero or missing, they are generated from this
   constant to guarantee correct timing.

Usage
-----
  python3 action_chunk_controller.py

Subscribes
----------
  /action_chunk        (Float64MultiArray, 497 values) — from GR00T inference
  /joint_states        (JointState)                    — for current positions

Sends goals to
--------------
  /left_joint_trajectory_controller/follow_joint_trajectory
  /right_joint_trajectory_controller/follow_joint_trajectory
  /right_hand_controller/follow_joint_trajectory
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration as RclpyDuration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import threading
import json
from typing import List, Dict, Optional, Any
from dataclasses import dataclass
import time

# ── GR00T N1.5 timing constants ───────────────────────────────────────────────
GROOT_STEP_DT = 1.0 / 30.0        # Duration per action step (seconds): 33.3 ms
TRAJECTORY_START_DELAY_SEC = 0.05  # Lead time added to now before trajectory starts.
                                   # Gives all 3 controllers time to accept the goal
                                   # before t=0 of the trajectory.
RECEDING_HORIZON_ENABLED = True    # Enable receding horizon control for timing correction

# Performance monitoring
PERFORMANCE_MONITORING_ENABLED = True  # Enable detailed performance tracking
PERFORMANCE_LOG_INTERVAL = 10.0      # Log performance stats every 10 seconds
# ──────────────────────────────────────────────────────────────────────────────


@dataclass
class ActionChunk:
    """Represents a 16-step action chunk from the VLA model."""
    timestamps: List[float]              # time_from_start for each step (seconds)
    left_arm_positions: List[List[float]]   # 16 steps × 7 joints
    right_arm_positions: List[List[float]]  # 16 steps × 7 joints
    right_hand_positions: List[List[float]] # 16 steps × 16 joints
    chunk_id: int = 0
    received_time: float = 0.0


class ActionChunkController(Node):
    """
    Synchronized controller for GR00T action chunk execution.

    Execution model:
    - On new chunk: cancel any in-flight goals → dispatch new chunk immediately.
    - All three controllers (left arm, right arm, hand) receive goals with the
      same ``header.stamp`` (= now + TRAJECTORY_START_DELAY_SEC) so they start
      executing at exactly the same wall-clock time.
    """

    # Joint names (must match URDF / controller config)
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
        'right_index_mcp_side',    'right_index_mcp_forward',
        'right_index_pip',         'right_index_dip',
        'right_middle_mcp_side',   'right_middle_mcp_forward',
        'right_middle_pip',        'right_middle_dip',
        'right_ring_mcp_side',     'right_ring_mcp_forward',
        'right_ring_pip',          'right_ring_dip',
        'right_thumb_mcp_side',    'right_thumb_mcp_forward',
        'right_thumb_pip_joint',   'right_thumb_dip_joint'
    ]

    def __init__(self):
        super().__init__('action_chunk_controller')

        # Reentrant callback group allows concurrent spin_until_future_complete
        self.callback_group = ReentrantCallbackGroup()

        # Action clients for all three controllers
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

        # Active goal handles — used to cancel in-flight trajectories on preempt
        self.active_goal_handles: List[Any] = []
        self.active_handles_lock = threading.Lock()

        # Chunk ID counter (for logging)
        self.chunk_id_counter = 0

        # Performance tracking variables
        self.performance_start_time = time.time()
        self.last_performance_log = time.time()
        self.chunk_execution_times = []
        self.chunk_processing_times = []
        self.trajectory_acceptance_times = []
        self.total_chunks_processed = 0
        self.total_chunks_accepted = 0
        self.total_chunks_rejected = 0

        # Joint state subscriber (for current-position fallback / logging)
        self.current_joint_states: Dict[str, float] = {}
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10
        )

        # Subscribe to action chunk topic from GR00T inference
        self.chunk_sub = self.create_subscription(
            Float64MultiArray,
            '/action_chunk',
            self._action_chunk_callback,
            10,
            callback_group=self.callback_group
        )

        # Wait for action servers before allowing execution
        self.get_logger().info('Waiting for action servers...')
        self._wait_for_servers()
        self.get_logger().info('All action servers ready!')

        self.get_logger().info(
            f'Action Chunk Controller initialized\n'
            f'  Step dt      : {GROOT_STEP_DT*1000:.1f} ms  (1/30 s)\n'
            f'  Start delay  : {TRAJECTORY_START_DELAY_SEC*1000:.0f} ms\n'
            f'  Left arm     : {len(self.LEFT_ARM_JOINTS)} joints\n'
            f'  Right arm    : {len(self.RIGHT_ARM_JOINTS)} joints\n'
            f'  Right hand   : {len(self.RIGHT_HAND_JOINTS)} joints\n'
            f'  Total DOF    : '
            f'{len(self.LEFT_ARM_JOINTS)+len(self.RIGHT_ARM_JOINTS)+len(self.RIGHT_HAND_JOINTS)}'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Server readiness

    def _wait_for_servers(self, timeout: float = 10.0) -> bool:
        """Wait for all three action servers to become available."""
        # Pre-initialize to avoid NameError if timeout==0
        left_ready = right_ready = hand_ready = False

        start_time = time.time()
        while time.time() - start_time < timeout:
            left_ready  = self.left_arm_client.wait_for_server(timeout_sec=0.1)
            right_ready = self.right_arm_client.wait_for_server(timeout_sec=0.1)
            hand_ready  = self.right_hand_client.wait_for_server(timeout_sec=0.1)
            if left_ready and right_ready and hand_ready:
                return True

        missing = []
        if not left_ready:  missing.append('left_arm')
        if not right_ready: missing.append('right_arm')
        if not hand_ready:  missing.append('right_hand')
        self.get_logger().error(f'Action servers not available: {missing}')
        return False

    # ──────────────────────────────────────────────────────────────────────────
    # Subscribers

    def _joint_state_callback(self, msg: JointState):
        """Cache latest joint positions for logging / current-state use."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = msg.position[i]

    def _action_chunk_callback(self, msg: Float64MultiArray):
        """
        Receive action chunk from GR00T inference.

        Message format (497 floats):
          [0]       chunk_id
          [1:17]    timestamps for each step (seconds from chunk start)
                    If all zeros, auto-generated as k * GROOT_STEP_DT
          [17:129]  left  arm positions  — 16 steps × 7  joints (flat)
          [129:241] right arm positions  — 16 steps × 7  joints (flat)
          [241:497] right hand positions — 16 steps × 16 joints (flat)

        On arrival: cancel any in-flight goals and execute the new chunk
        immediately (preempt-on-new-chunk strategy).
        """
        data = msg.data

        if len(data) < 497:
            self.get_logger().error(
                f'Invalid action chunk size: {len(data)}, expected 497')
            return

        try:
            chunk_id = int(data[0])
            raw_ts   = list(data[1:17])   # May be all-zero from some senders

            # If timestamps are missing/zero, generate from GR00T step dt
            if all(t == 0.0 for t in raw_ts):
                timestamps = [(k + 1) * GROOT_STEP_DT for k in range(16)]
                self.get_logger().debug(
                    'Timestamps were all-zero; generated from GROOT_STEP_DT=1/30 s')
            else:
                timestamps = raw_ts

            # Extract position arrays (left arm, right arm, right hand)
            la_flat = data[17:129]
            ra_flat = data[129:241]
            rh_flat = data[241:497]

            left_arm_positions  = [list(la_flat[i*7  :(i+1)*7 ]) for i in range(16)]
            right_arm_positions = [list(ra_flat[i*7  :(i+1)*7 ]) for i in range(16)]
            right_hand_positions= [list(rh_flat[i*16 :(i+1)*16]) for i in range(16)]

            chunk = ActionChunk(
                timestamps=timestamps,
                left_arm_positions=left_arm_positions,
                right_arm_positions=right_arm_positions,
                right_hand_positions=right_hand_positions,
                chunk_id=chunk_id,
                received_time=time.time()
            )

        except Exception as e:
            self.get_logger().error(f'Failed to parse action chunk: {e}')
            return

        # ── Preempt: cancel in-flight goals, then execute new chunk ──────────
        with self.active_handles_lock:
            if self.active_goal_handles:
                self.get_logger().info(
                    f'Chunk {chunk_id}: preempting {len(self.active_goal_handles)} '
                    f'active goal(s) with fresh inference result')
                for handle in self.active_goal_handles:
                    handle.cancel_goal_async()
                self.active_goal_handles.clear()

        self.execute_action_chunk(chunk)

    # ──────────────────────────────────────────────────────────────────────────
    # Execution

    def execute_action_chunk(self, chunk: ActionChunk):
        """
        Send synchronized multi-point trajectories to all three controllers.

        All three goals share the same ``header.stamp`` so controllers start
        at exactly the same ROS clock instant (= now + TRAJECTORY_START_DELAY_SEC).
        Timestamps inside the trajectory are relative to that stamp.
        """
        self.get_logger().info(
            f'Chunk {chunk.chunk_id}: executing {len(chunk.timestamps)} steps, '
            f'total duration={chunk.timestamps[-1]:.3f}s'
        )

        # Build trajectory point lists
        left_points  = self._build_trajectory_points(
            chunk.timestamps, chunk.left_arm_positions)
        right_points = self._build_trajectory_points(
            chunk.timestamps, chunk.right_arm_positions)
        hand_points  = self._build_trajectory_points(
            chunk.timestamps, chunk.right_hand_positions)

        # ── Receding Horizon Control ──────────────────────────────────────────
        if RECEDING_HORIZON_ENABLED:
            # Check if any timestamps have already passed and adjust trajectory
            adjusted_points = self._apply_receding_horizon_control(
                chunk.timestamps, left_points, right_points, hand_points)
            left_points, right_points, hand_points = adjusted_points

        # ── Synchronized start time ───────────────────────────────────────────
        # All three trajectory goals share the SAME header.stamp.
        # JointTrajectoryController interprets time_from_start relative to this
        # stamp, so all controllers start at the same wall-clock instant.
        now = self.get_clock().now()
        start_time = now + RclpyDuration(seconds=TRAJECTORY_START_DELAY_SEC)

        # Send all three goals asynchronously (as close together as possible)
        futures_with_labels = [
            ('left_arm',   self._send_trajectory_async(
                self.left_arm_client,  self.LEFT_ARM_JOINTS,  left_points,  start_time)),
            ('right_arm',  self._send_trajectory_async(
                self.right_arm_client, self.RIGHT_ARM_JOINTS, right_points, start_time)),
            ('right_hand', self._send_trajectory_async(
                self.right_hand_client, self.RIGHT_HAND_JOINTS, hand_points, start_time)),
        ]

        # Register callbacks to collect accepted goal handles for future preemption
        for label, future in futures_with_labels:
            future.add_done_callback(
                lambda f, lbl=label: self._goal_accepted_callback(f, lbl, chunk.chunk_id)
            )

        self.get_logger().info(
            f'Chunk {chunk.chunk_id}: trajectories sent to all 3 controllers '
            f'(start_delay={TRAJECTORY_START_DELAY_SEC*1000:.0f} ms)'
        )

    def _goal_accepted_callback(self, future, label: str, chunk_id: int):
        """Store accepted goal handle so it can be cancelled on preemption."""
        try:
            handle = future.result()
            if handle.accepted:
                with self.active_handles_lock:
                    self.active_goal_handles.append(handle)
                self.get_logger().debug(f'Chunk {chunk_id}: goal accepted by {label}')
            else:
                self.get_logger().warn(
                    f'Chunk {chunk_id}: goal REJECTED by {label}')
        except Exception as e:
            self.get_logger().error(
                f'Chunk {chunk_id}: goal send error ({label}): {e}')

    # ──────────────────────────────────────────────────────────────────────────
    # Helpers

    def _build_trajectory_points(
        self,
        timestamps: List[float],
        positions: List[List[float]]
    ) -> List[JointTrajectoryPoint]:
        """
        Build JointTrajectoryPoint list from step timestamps and positions.

        Velocities are estimated by finite differencing adjacent steps to help
        the trajectory controller interpolate smoothly.

        Args:
            timestamps: List of time_from_start values (seconds), e.g.
                        [1/30, 2/30, ..., 16/30] for GR00T N1.5.
            positions:  List of joint position vectors, one per step.
        """
        points = []
        for i, (ts, pos) in enumerate(zip(timestamps, positions)):
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in pos]

            # time_from_start is relative to trajectory header.stamp
            sec    = int(ts)
            nanosec = int(round((ts - sec) * 1e9))
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)

            # Finite-difference velocity estimate for smoother interpolation
            if i > 0:
                dt = timestamps[i] - timestamps[i - 1]
                if dt > 1e-6:
                    point.velocities = [
                        (positions[i][j] - positions[i - 1][j]) / dt
                        for j in range(len(pos))
                    ]

            points.append(point)
        return points

    def _send_trajectory_async(
        self,
        action_client: ActionClient,
        joint_names:   List[str],
        points:        List[JointTrajectoryPoint],
        start_time=None
    ):
        """
        Build and send a FollowJointTrajectory goal asynchronously.

        Args:
            start_time: rclpy.time.Time — shared start stamp for synchronization.
                        All three controllers must receive the same value so their
                        trajectories begin at the same wall-clock instant.
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points      = points

        if start_time is not None:
            # Setting header.stamp makes the controller wait until this ROS time
            # before starting execution — the key to cross-controller synchronization
            goal_msg.trajectory.header.stamp = start_time.to_msg()

        return action_client.send_goal_async(goal_msg)

    def _apply_receding_horizon_control(self, timestamps: List[float], 
                                      left_points: List[JointTrajectoryPoint],
                                      right_points: List[JointTrajectoryPoint],
                                      hand_points: List[JointTrajectoryPoint]) -> tuple:
        """
        Apply receding horizon control to prevent jumps when timestamps have passed.
        
        This method checks if any trajectory points correspond to times that have
        already passed and adjusts the trajectory accordingly:
        1. If the first point is in the past, interpolate from current state to the first valid point
        2. If some points are in the past, start from the first future point
        3. If all points are in the past, use current state as target
        """
        now = self.get_clock().now()
        start_time = now + RclpyDuration(seconds=TRAJECTORY_START_DELAY_SEC)
        
        # Convert start_time to seconds for comparison
        start_time_sec = start_time.seconds_nanoseconds()[0] + start_time.seconds_nanoseconds()[1] * 1e-9
        
        # Find the first point that hasn't passed yet
        current_time_sec = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9
        first_future_idx = 0
        
        for i, ts in enumerate(timestamps):
            if current_time_sec + ts > start_time_sec:
                first_future_idx = i
                break
        
        # If all points are in the past, use current state as target
        if first_future_idx >= len(timestamps) - 1:
            self.get_logger().warn("All trajectory points are in the past, using current state as target")
            return self._create_single_point_trajectory(left_points, right_points, hand_points)
        
        # If first point is in the past, interpolate from current state
        if first_future_idx > 0:
            self.get_logger().info(f"Adjusting trajectory: skipping {first_future_idx} past points")
            
            # Get current joint positions for interpolation
            current_left = self._get_current_joint_positions(self.LEFT_ARM_JOINTS)
            current_right = self._get_current_joint_positions(self.RIGHT_ARM_JOINTS)
            current_hand = self._get_current_joint_positions(self.RIGHT_HAND_JOINTS)
            
            # Get the first future point
            target_left = left_points[first_future_idx].positions
            target_right = right_points[first_future_idx].positions
            target_hand = hand_points[first_future_idx].positions
            
            # Create interpolated first point
            interp_left = self._interpolate_points(current_left, target_left, 0.5)
            interp_right = self._interpolate_points(current_right, target_right, 0.5)
            interp_hand = self._interpolate_points(current_hand, target_hand, 0.5)
            
            # Build new trajectories starting from interpolated point
            new_left_points = [self._create_trajectory_point(interp_left, 0.0)]
            new_right_points = [self._create_trajectory_point(interp_right, 0.0)]
            new_hand_points = [self._create_trajectory_point(interp_hand, 0.0)]
            
            # Add remaining points with adjusted timestamps
            for i in range(first_future_idx, len(timestamps)):
                # Adjust timestamp relative to new start (0.0)
                adjusted_ts = timestamps[i] - timestamps[first_future_idx]
                
                new_left_points.append(self._create_trajectory_point(
                    left_points[i].positions, adjusted_ts))
                new_right_points.append(self._create_trajectory_point(
                    right_points[i].positions, adjusted_ts))
                new_hand_points.append(self._create_trajectory_point(
                    hand_points[i].positions, adjusted_ts))
            
            return new_left_points, new_right_points, new_hand_points
        
        # No adjustment needed, return original trajectories
        return left_points, right_points, hand_points

    def _get_current_joint_positions(self, joint_names: List[str]) -> List[float]:
        """Get current joint positions from cached joint states."""
        positions = []
        for name in joint_names:
            if name in self.current_joint_states:
                positions.append(self.current_joint_states[name])
            else:
                positions.append(0.0)  # Default if not available
        return positions

    def _interpolate_points(self, start: List[float], end: List[float], alpha: float) -> List[float]:
        """Linearly interpolate between two joint position vectors."""
        return [start[i] + alpha * (end[i] - start[i]) for i in range(len(start))]

    def _create_trajectory_point(self, positions: List[float], time_from_start: float) -> JointTrajectoryPoint:
        """Create a trajectory point with given positions and timestamp."""
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        
        sec = int(time_from_start)
        nanosec = int(round((time_from_start - sec) * 1e9))
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        
        return point

    def _create_single_point_trajectory(self, left_points: List[JointTrajectoryPoint],
                                      right_points: List[JointTrajectoryPoint],
                                      hand_points: List[JointTrajectoryPoint]) -> tuple:
        """Create single-point trajectories using current positions."""
        current_left = self._get_current_joint_positions(self.LEFT_ARM_JOINTS)
        current_right = self._get_current_joint_positions(self.RIGHT_ARM_JOINTS)
        current_hand = self._get_current_joint_positions(self.RIGHT_HAND_JOINTS)
        
        single_left = [self._create_trajectory_point(current_left, 0.0)]
        single_right = [self._create_trajectory_point(current_right, 0.0)]
        single_hand = [self._create_trajectory_point(current_hand, 0.0)]
        
        return single_left, single_right, single_hand

    # ──────────────────────────────────────────────────────────────────────────
    # Testing / replay utilities

    def send_action_chunk_from_dict(self, chunk_data: Dict):
        """
        Queue an action chunk supplied as a dictionary (for testing).

        Dict keys:
          timestamps  : List[float]        (optional; auto-generated if absent)
          left_arm    : List[List[float]]  shape [16, 7]
          right_arm   : List[List[float]]  shape [16, 7]
          right_hand  : List[List[float]]  shape [16, 16]
        """
        raw_ts = chunk_data.get(
            'timestamps',
            [(k + 1) * GROOT_STEP_DT for k in range(16)]
        )
        chunk = ActionChunk(
            timestamps=raw_ts,
            left_arm_positions=chunk_data.get('left_arm',   [[0.0] * 7]  * 16),
            right_arm_positions=chunk_data.get('right_arm',  [[0.0] * 7]  * 16),
            right_hand_positions=chunk_data.get('right_hand', [[0.0] * 16] * 16),
            chunk_id=self.chunk_id_counter,
            received_time=time.time()
        )
        self.chunk_id_counter += 1
        self.execute_action_chunk(chunk)
        self.get_logger().info(f'Dispatched test chunk {chunk.chunk_id}')

    def load_action_chunks_from_file(self, filepath: str):
        """Load and replay action chunks from a JSON file."""
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            chunks = data.get('chunks', [data])
            for chunk_data in chunks:
                self.send_action_chunk_from_dict(chunk_data)
            self.get_logger().info(f'Loaded {len(chunks)} chunk(s) from {filepath}')
        except Exception as e:
            self.get_logger().error(f'Failed to load chunks from {filepath}: {e}')

    # ──────────────────────────────────────────────────────────────────────────
    # Performance monitoring

    def _log_performance_stats(self):
        """Log performance statistics if monitoring is enabled."""
        if not PERFORMANCE_MONITORING_ENABLED:
            return
            
        current_time = time.time()
        if current_time - self.last_performance_log < PERFORMANCE_LOG_INTERVAL:
            return
            
        self.last_performance_log = current_time
        
        # Calculate statistics
        if self.chunk_processing_times:
            avg_processing = sum(self.chunk_processing_times) / len(self.chunk_processing_times)
            max_processing = max(self.chunk_processing_times)
        else:
            avg_processing = max_processing = 0.0
            
        if self.trajectory_acceptance_times:
            avg_acceptance = sum(self.trajectory_acceptance_times) / len(self.trajectory_acceptance_times)
            max_acceptance = max(self.trajectory_acceptance_times)
        else:
            avg_acceptance = max_acceptance = 0.0
            
        # Calculate success rate
        total_goals = self.total_chunks_accepted + self.total_chunks_rejected
        success_rate = (self.total_chunks_accepted / total_goals * 100) if total_goals > 0 else 0.0
        
        # Calculate uptime
        uptime = current_time - self.performance_start_time
        
        self.get_logger().info(
            f'=== Performance Statistics (Uptime: {uptime:.1f}s) ===\n'
            f'Chunks processed: {self.total_chunks_processed}\n'
            f'Goal acceptance: {self.total_chunks_accepted}/{total_goals} ({success_rate:.1f}%)\n'
            f'Avg processing time: {avg_processing*1000:.1f} ms (max: {max_processing*1000:.1f} ms)\n'
            f'Avg acceptance time: {avg_acceptance*1000:.1f} ms (max: {max_acceptance*1000:.1f} ms)\n'
            f'========================================================'
        )

    def _record_chunk_processing_time(self, processing_time: float):
        """Record the time taken to process a chunk."""
        if PERFORMANCE_MONITORING_ENABLED:
            self.chunk_processing_times.append(processing_time)
            # Keep only last 1000 measurements to prevent memory growth
            if len(self.chunk_processing_times) > 1000:
                self.chunk_processing_times.pop(0)

    def _record_trajectory_acceptance_time(self, acceptance_time: float):
        """Record the time taken for trajectory acceptance."""
        if PERFORMANCE_MONITORING_ENABLED:
            self.trajectory_acceptance_times.append(acceptance_time)
            # Keep only last 1000 measurements to prevent memory growth
            if len(self.trajectory_acceptance_times) > 1000:
                self.trajectory_acceptance_times.pop(0)

    def _increment_chunk_count(self):
        """Increment the total chunks processed counter."""
        if PERFORMANCE_MONITORING_ENABLED:
            self.total_chunks_processed += 1
            self._log_performance_stats()

    def _increment_acceptance_count(self):
        """Increment the accepted goals counter."""
        if PERFORMANCE_MONITORING_ENABLED:
            self.total_chunks_accepted += 1

    def _increment_rejection_count(self):
        """Increment the rejected goals counter."""
        if PERFORMANCE_MONITORING_ENABLED:
            self.total_chunks_rejected += 1


def main(args=None):
    rclpy.init(args=args)
    controller = ActionChunkController()

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