#!/usr/bin/env python3
"""
joint_actions_aggregator.py

Aggregates joint commands from four limb sources into a single /joint_actions
topic for data collection and imitation learning:
  - Left arm IK commands   (7 joints)
  - Right arm IK commands  (7 joints)
  - Left hand commands     (6 joints, O6)
  - Right hand commands    (6 joints, O6)

The published vector ALWAYS carries all 26 joints in a fixed order, whatever the
--arm_config / --hand_config selection is. Downstream (vla_control's
data_collector) looks joints up by name and drops the whole sample if any
expected name is absent, so a shrinking vector would silently record nothing.

Publishing model
----------------
Event driven, NOT a fixed-rate timer. A sample is published when every *active*
arm has delivered a new command since the last publish, so /joint_actions
inherits the IK node's rate and phase instead of being resampled a second time
(the collector resamples again at its own control_hz).

A limb becomes ACTIVE the first time a command arrives on it ("latch"), and
stays active for the rest of the session. The distinction that drives everything
else:

  - never seen a command  -> the source is not running; that limb's joints are
    filled from /joint_states and it does not gate publishing.
  - seen, then went quiet -> the source failed or paused; publishing STOPS until
    it recovers, rather than republishing a held value.

Holding a stale command is what makes the collector's freshness telemetry lie:
it measures the age of the last /joint_actions message, so republishing frozen
values at a fixed rate reads as perfectly real-time data. Going silent lets it
correctly flag the episode as non-real-time.

Filling from /joint_states is only ever applied to a limb that is inactive by
configuration or that never started, i.e. one that is physically stationary. It
is deliberately NOT used to paper over a failed source: the collector's
observation vector is also /joint_states, so a substituted action would be an
exact copy of the observation, which trains a policy to reproduce its own state.

Usage:
    This script is not part of a ROS package (there is no package.xml under
    scripts/), so there is no `ros2 run` entry point for it. Run it directly, or
    let openarm_o6_bimanual.launch.py start it.

    # Bimanual arms with O6 hands (both) - DEFAULT
    python3 scripts/data_collection/joint_actions_aggregator.py

    # Bimanual arms, only the right O6 hand is teleoperated
    python3 scripts/data_collection/joint_actions_aggregator.py --hand_config o6_right

    # Started together with the robot (launch_joint_actions_aggregator:=true is
    # the default; aggregator_* arguments map onto the flags below)
    ros2 launch openarm_bringup openarm_o6_bimanual.launch.py \
        aggregator_hand_config:=o6_right
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

# Fixed publish order — must stay in sync with vla_control's
# config/robot.py get_joint_order("openarm_linkerhand_o6").
ALL_JOINT_NAMES = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + O6_LEFT_JOINTS + O6_RIGHT_JOINTS

# A source is stale after 3 missed messages at its own measured rate; the floor
# keeps a fast source (the 100 Hz hands) from tripping on ordinary jitter.
STALE_PERIODS = 3.0
STALE_FLOOR_SEC = 0.03


class _LimbSource:
    """Per-limb command state: latest value, arrival time, measured rate."""

    def __init__(self, key: str, joint_names: List[str], subscribed: bool,
                 expected_rate: float):
        self.key = key
        self.joint_names = joint_names
        self.subscribed = subscribed
        self.value: Optional[List[float]] = None
        self.last_t: Optional[float] = None   # seconds, node clock
        self.stamp_sec: Optional[float] = None  # source header stamp if it has one
        self.latched = False                  # has ever delivered a command
        self.advanced = False                 # new data since the last publish
        self.period_ema = 1.0 / expected_rate if expected_rate > 0 else 0.02
        self.rejects = 0

    def accept(self, values: List[float], now: float, stamp_sec: Optional[float]):
        if self.last_t is not None:
            dt = now - self.last_t
            if 0.0 < dt < 1.0:   # ignore the first sample and long gaps
                self.period_ema = 0.85 * self.period_ema + 0.15 * dt
        self.value = values
        self.last_t = now
        self.stamp_sec = stamp_sec if stamp_sec is not None else now
        self.latched = True
        self.advanced = True

    @property
    def stale_timeout(self) -> float:
        return max(STALE_PERIODS * self.period_ema, STALE_FLOOR_SEC)

    def age(self, now: float) -> float:
        return float('inf') if self.last_t is None else now - self.last_t


class JointActionsAggregator(Node):
    """
    Aggregates joint commands from arms and hands into /joint_actions.

    Subscriptions (arm_config / hand_config decide which are created):
      - /left_arm_ik_commands  (JointState, 7 joints)
      - /right_arm_ik_commands (JointState, 7 joints)
      - /left_hand_forward_position_controller/commands  (Float64MultiArray, 6)
      - /right_hand_forward_position_controller/commands (Float64MultiArray, 6)
      - /joint_states (JointState) — fill for limbs that are inactive

    Publications:
      - /joint_actions (JointState, 26 joints, fixed order)
    """

    def __init__(self, hand_config: str = "o6_both", publish_rate: float = 50.0,
                 arm_config: str = "both", stale_timeout: float = 0.0):
        super().__init__('joint_actions_aggregator')

        if arm_config not in ("both", "left", "right"):
            self.get_logger().warn(f"Unknown arm_config '{arm_config}', defaulting to 'both'")
            arm_config = "both"
        if hand_config not in ("none", "o6_left", "o6_right", "o6_both"):
            self.get_logger().warn(f"Unknown hand_config '{hand_config}', defaulting to 'o6_both'")
            hand_config = "o6_both"
        self.arm_config = arm_config
        self.hand_config = hand_config
        # publish_rate no longer drives publishing (that is event driven); it only
        # seeds the arm rate estimate before enough messages have arrived to measure.
        self.publish_rate = publish_rate
        self.stale_override = stale_timeout if stale_timeout > 0 else None

        self.all_joint_names = list(ALL_JOINT_NAMES)
        self.joint_states_map: Dict[str, float] = {}
        self.lock = threading.Lock()
        self.msg_count = 0
        self._pause_t: Optional[float] = None    # when publishing last stopped
        self._pause_keys: List[str] = []         # which limbs stopped it

        self.sources: Dict[str, _LimbSource] = {
            "left_arm": _LimbSource("left_arm", LEFT_ARM_JOINTS,
                                    arm_config in ("both", "left"), publish_rate),
            "right_arm": _LimbSource("right_arm", RIGHT_ARM_JOINTS,
                                     arm_config in ("both", "right"), publish_rate),
            # Hand teleop runs at ~100 Hz; seeding the estimate avoids a wrong
            # stale window during the first few messages.
            "left_hand": _LimbSource("left_hand", O6_LEFT_JOINTS,
                                     hand_config in ("o6_left", "o6_both"), 100.0),
            "right_hand": _LimbSource("right_hand", O6_RIGHT_JOINTS,
                                      hand_config in ("o6_right", "o6_both"), 100.0),
        }
        self.arm_keys = ("left_arm", "right_arm")
        # Concatenation order of the published vector; must match ALL_JOINT_NAMES.
        self.limb_order = ("left_arm", "right_arm", "left_hand", "right_hand")

        self._create_subscribers()
        self.joint_actions_pub = self.create_publisher(JointState, '/joint_actions', 10)

        # Diagnostics only — publishing is driven by incoming arm commands, so a
        # source that goes quiet would otherwise never be reported.
        self.create_timer(1.0, self._report_health)

        self.get_logger().info("JointActionsAggregator started")
        self.get_logger().info(f"  Arm config: {arm_config}")
        self.get_logger().info(f"  Hand config: {hand_config}")
        self.get_logger().info(f"  Total joints: {len(self.all_joint_names)} (fixed)")
        self.get_logger().info(
            "  Subscribed limbs: "
            + ", ".join(k for k, s in self.sources.items() if s.subscribed))
        self.get_logger().info(
            "  Not subscribed (filled from /joint_states): "
            + (", ".join(k for k, s in self.sources.items() if not s.subscribed) or "none"))
        self.get_logger().info("  Publishing on arm command arrival (no fixed-rate timer)")

    # ── Setup ─────────────────────────────────────────────────────────────────

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _create_subscribers(self):
        """Create subscribers for the configured limbs only."""
        if self.sources["left_arm"].subscribed:
            self.create_subscription(
                JointState, '/left_arm_ik_commands', self._left_arm_callback, 10)
        if self.sources["right_arm"].subscribed:
            self.create_subscription(
                JointState, '/right_arm_ik_commands', self._right_arm_callback, 10)

        # /joint_states — fill for limbs that are inactive by configuration or
        # that never started. Never used to substitute for a failed source.
        self.create_subscription(
            JointState, '/joint_states', self._joint_states_callback, 10)

        if self.sources["left_hand"].subscribed:
            self.create_subscription(
                Float64MultiArray,
                '/left_hand_forward_position_controller/commands',
                self._left_hand_callback, 10)
        if self.sources["right_hand"].subscribed:
            self.create_subscription(
                Float64MultiArray,
                '/right_hand_forward_position_controller/commands',
                self._right_hand_callback, 10)

    # ── Message validation ────────────────────────────────────────────────────

    def _values_from_joint_state(self, src: _LimbSource,
                                 msg: JointState) -> Optional[List[float]]:
        """Extract this limb's joints from a JointState, by name when possible.

        Name lookup is preferred so a reordered publisher cannot silently scramble
        the vector. A publisher that omits `name` still works as long as it sends
        exactly the expected number of positions.
        """
        if msg.name:
            by_name = dict(zip(msg.name, msg.position))
            missing = [n for n in src.joint_names if n not in by_name]
            if not missing:
                return [float(by_name[n]) for n in src.joint_names]
            src.rejects += 1
            self.get_logger().warn(
                f"{src.key}: command is missing {missing} — ignoring message "
                f"({src.rejects} rejected so far)", throttle_duration_sec=5.0)
            return None

        if len(msg.position) != len(src.joint_names):
            src.rejects += 1
            self.get_logger().warn(
                f"{src.key}: unnamed command has {len(msg.position)} positions, "
                f"expected {len(src.joint_names)} — ignoring message "
                f"({src.rejects} rejected so far)", throttle_duration_sec=5.0)
            return None
        return [float(p) for p in msg.position]

    def _values_from_array(self, src: _LimbSource,
                           msg: Float64MultiArray) -> Optional[List[float]]:
        """Extract this limb's joints from a Float64MultiArray (no names to check)."""
        if len(msg.data) != len(src.joint_names):
            src.rejects += 1
            self.get_logger().warn(
                f"{src.key}: command has {len(msg.data)} values, expected "
                f"{len(src.joint_names)} — ignoring message "
                f"({src.rejects} rejected so far)", throttle_duration_sec=5.0)
            return None
        return [float(v) for v in msg.data]

    @staticmethod
    def _stamp_sec(msg) -> Optional[float]:
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        if stamp is None:
            return None
        sec = stamp.sec + stamp.nanosec * 1e-9
        return sec if sec > 0.0 else None

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _arm_command(self, key: str, msg: JointState):
        """Accept an arm command and try to publish — arms drive the rate."""
        src = self.sources[key]
        values = self._values_from_joint_state(src, msg)
        if values is None:
            return
        with self.lock:
            src.accept(values, self._now(), self._stamp_sec(msg))
            sample = self._build_sample_locked()
        if sample is not None:
            self._publish(*sample)

    def _left_arm_callback(self, msg: JointState):
        self._arm_command("left_arm", msg)

    def _right_arm_callback(self, msg: JointState):
        self._arm_command("right_arm", msg)

    def _left_hand_callback(self, msg: Float64MultiArray):
        """Receive left hand commands (6 joints for O6)."""
        src = self.sources["left_hand"]
        values = self._values_from_array(src, msg)
        if values is not None:
            with self.lock:
                src.accept(values, self._now(), None)

    def _right_hand_callback(self, msg: Float64MultiArray):
        """Receive right hand commands (6 joints for O6)."""
        src = self.sources["right_hand"]
        values = self._values_from_array(src, msg)
        if values is not None:
            with self.lock:
                src.accept(values, self._now(), None)

    def _joint_states_callback(self, msg: JointState):
        """Cache latest actual joint positions (fill for inactive limbs)."""
        with self.lock:
            for n, p in zip(msg.name, msg.position):
                self.joint_states_map[n] = p

    # ── Aggregation and publishing ────────────────────────────────────────────

    def _blockers_locked(self, now: float) -> List[str]:
        """Active limbs whose latest command is too old to publish with."""
        return [s.key for s in self.sources.values()
                if s.latched and not self._fresh(s, now)]

    def _fresh(self, src: _LimbSource, now: float) -> bool:
        if not src.latched:
            return False
        timeout = self.stale_override if self.stale_override else src.stale_timeout
        return src.age(now) <= timeout

    def _build_sample_locked(self):
        """Build one aggregated sample, or None if it must not be published.

        Caller must hold self.lock. Publishing requires:
          1. at least one arm has ever delivered a command — otherwise the whole
             vector would come from /joint_states, i.e. a copy of the observation;
          2. every active arm has advanced since the last publish, so both arms
             come from the same IK solve step instead of straddling two;
          3. no active limb is stale.
        """
        now = self._now()

        active_arms = [self.sources[k] for k in self.arm_keys if self.sources[k].latched]
        if not active_arms:
            return None
        if not all(s.advanced for s in active_arms):
            # Not a fault: the other arm's command for this step has not landed yet.
            return None

        blockers = self._blockers_locked(now)
        if blockers:
            if self.msg_count and self._pause_t is None:
                self._pause_t, self._pause_keys = now, blockers
            return None

        positions: List[float] = []
        oldest_stamp = None
        for src in (self.sources[k] for k in self.limb_order):
            if src.latched:
                positions.extend(src.value)
                if src.stamp_sec is not None:
                    oldest_stamp = (src.stamp_sec if oldest_stamp is None
                                    else min(oldest_stamp, src.stamp_sec))
            else:
                fill = self._fill_from_state_locked(src)
                if fill is None:
                    return None
                positions.extend(fill)

        for src in self.sources.values():
            src.advanced = False

        paused_for, paused_keys = None, None
        if self._pause_t is not None:
            paused_for, paused_keys = now - self._pause_t, self._pause_keys
            self._pause_t, self._pause_keys = None, []

        return (positions, (oldest_stamp if oldest_stamp is not None else now),
                paused_for, paused_keys)

    def _fill_from_state_locked(self, src: _LimbSource) -> Optional[List[float]]:
        """Latest /joint_states values for an inactive limb, or None if unavailable.

        Returning None withholds the whole sample rather than inventing a number:
        a constant fabricated column is indistinguishable from real data once it
        is in the dataset. If these joints never appear in /joint_states the
        collector cannot build its observation vector either, so it would record
        nothing regardless.
        """
        missing = [n for n in src.joint_names if n not in self.joint_states_map]
        if missing:
            self.get_logger().warn(
                f"{src.key}: inactive and {missing} absent from /joint_states — "
                f"withholding /joint_actions (nothing truthful to fill with)",
                throttle_duration_sec=5.0)
            return None
        return [self.joint_states_map[n] for n in src.joint_names]

    def _publish(self, positions: List[float], stamp_sec: float,
                 paused_for: Optional[float], paused_keys: Optional[List[str]]):
        msg = JointState()
        # Age of the OLDEST contributing command, not the publish instant, so a
        # consumer that reads header.stamp sees the true age of the sample.
        msg.header.stamp.sec = int(stamp_sec)
        msg.header.stamp.nanosec = int((stamp_sec - int(stamp_sec)) * 1e9)
        msg.name = self.all_joint_names
        msg.position = positions

        self.joint_actions_pub.publish(msg)

        self.msg_count += 1
        if self.msg_count == 1:
            self.get_logger().info(
                f"Started publishing to /joint_actions ({len(positions)} joints)")
        elif paused_for is not None:
            # Only event worth an unthrottled line: a gap in the recorded actions.
            # No periodic heartbeat — the stale warning already names what broke.
            self.get_logger().info(
                f"resumed /joint_actions after {paused_for * 1000:.0f}ms "
                f"waiting on {paused_keys}")

    # ── Diagnostics ───────────────────────────────────────────────────────────

    def _report_health(self):
        """Report limbs that never started or that went quiet (1 Hz, throttled)."""
        with self.lock:
            now = self._now()
            waiting = [s.key for s in self.sources.values()
                       if s.subscribed and not s.latched]
            blockers = self._blockers_locked(now)
            ages = {s.key: s.age(now) for s in self.sources.values() if s.latched}
            any_arm = any(self.sources[k].latched for k in self.arm_keys)

        if waiting:
            # Deliberately repeated: an operator who MEANT to run this source sees
            # the same log line as one who did not, so it must keep nagging.
            self.get_logger().warn(
                f"no command since startup on {waiting} — treating as inactive and "
                f"filling from /joint_states", throttle_duration_sec=5.0)
        if not any_arm:
            self.get_logger().info(
                "waiting for the first arm command before publishing",
                throttle_duration_sec=5.0)
        elif blockers:
            detail = ", ".join(f"{k}={ages[k] * 1000:.0f}ms" for k in blockers)
            self.get_logger().warn(
                f"stale: {detail} — /joint_actions paused until it recovers",
                throttle_duration_sec=2.0)


def main():
    parser = argparse.ArgumentParser(description='Aggregate joint commands to /joint_actions')
    parser.add_argument(
        '--arm_config',
        type=str,
        default='both',
        choices=['both', 'left', 'right'],
        help='Arms to subscribe to: both (default) or a single arm. The other '
             'arm still occupies its columns, filled from /joint_states.'
    )
    parser.add_argument(
        '--hand_config',
        type=str,
        default='o6_both',
        choices=['none', 'o6_left', 'o6_right', 'o6_both'],
        help='Hands to subscribe to: o6_both (default), o6_left, o6_right, or '
             'none. Unsubscribed hands still occupy their columns, filled from '
             '/joint_states — the published vector is always 26 joints.'
    )
    parser.add_argument(
        '--publish_rate',
        type=float,
        default=50.0,
        help='Expected arm command rate in Hz, used to seed the staleness '
             'window before it can be measured (default: 50.0). Publishing is '
             'driven by arm commands, not by this rate.'
    )
    parser.add_argument(
        '--stale_timeout',
        type=float,
        default=0.0,
        help='Fixed staleness window in seconds for every source. Default 0 '
             'derives it per source from the measured rate.'
    )

    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = JointActionsAggregator(
        hand_config=args.hand_config,
        publish_rate=args.publish_rate,
        arm_config=args.arm_config,
        stale_timeout=args.stale_timeout
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
