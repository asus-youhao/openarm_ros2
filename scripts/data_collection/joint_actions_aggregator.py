#!/usr/bin/env python3
"""
joint_actions_aggregator.py.

Aggregates OpenArm joint commands from four limb sources into a single
/joint_actions topic for data collection and imitation learning:
  - Left arm   (7 joints)  left_forward_position_controller
  - Right arm  (7 joints)  right_forward_position_controller
  - Left hand  (6 joints)  left_hand_forward_position_controller
  - Right hand (6 joints)  right_hand_forward_position_controller

Every source is the command topic of the controller that actually drives the
limb (`/<controller>/commands`, an UNNAMED std_msgs/Float64MultiArray), not a
side channel of the IK node: what is recorded is what the controller receives.
Which slot is which joint is read from the controller's own `joints`
parameter, never hard-coded, so a reordered controllers yaml cannot mislabel a
joint. Until a controller has answered, its commands are rejected; a
controller that is not loaded (e.g. robot_controller:=joint_trajectory_controller)
never answers, so nothing is recorded for it.

The published vector ALWAYS carries all 26 joints in a fixed order
(ALL_JOINT_NAMES), whatever --arm_config / --hand_config is. Downstream
(vla_control's data_collector) looks joints up by name and drops the whole
sample if any expected name is absent, so a shrinking vector would silently
record nothing.

Publishing model (same as phu_controller_ros2's gallop_joint_actions_aggregator)
--------------------------------------------------------------------------------
Event driven, NOT a fixed-rate timer. A sample is published when every *active*
arm has delivered a new command since the last publish, so /joint_actions
inherits the IK rate and phase instead of being resampled a second time. The arm
commands are paired by arrival time, so both arms of a sample come from the same
IK step (see _build_sample_locked). Hand commands never trigger a publish; their
latest value is taken.

A limb becomes ACTIVE the first time a command arrives on it ("latch"), and
stays active for the rest of the session:

  - never seen a command  -> the source is not running; that limb's joints are
    filled from /joint_states and it does not gate publishing.
  - seen, then went quiet -> the source failed or paused; publishing STOPS until
    it recovers, rather than republishing a held value.
  - no arm ever commanded -> nothing is published (the whole vector would be a
    copy of /joint_states).

/joint_states is deliberately NOT used to paper over a failed source: the
collector's observation is also /joint_states, so a substituted action would be
a copy of the observation and train a policy to reproduce its own state.

Ownership
---------
openarm_o6_bimanual.launch.py starts this node (with_joint_actions:=true, the
default) through openarm_bringup/launch/joint_actions.launch.py. Nothing else
may publish /joint_actions; the IK entry points no longer start one.

Usage:
    # Started by the bringup (default)
    ros2 launch openarm_bringup openarm_o6_bimanual.launch.py \
        robot_controller:=forward_position_controller

    # Standalone, after a bringup started with with_joint_actions:=false
    ros2 launch openarm_bringup joint_actions.launch.py
    python3 scripts/data_collection/joint_actions_aggregator.py --hand_config none

See scripts/data_collection/README.md and docs/JOINT_ACTIONS.md.
"""

import argparse
import signal
import threading
from typing import Dict, List, Optional

from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# ── Joint name definitions ────────────────────────────────────────────────────
LEFT_ARM_JOINTS = [f'openarm_left_joint{i}' for i in range(1, 8)]
RIGHT_ARM_JOINTS = [f'openarm_right_joint{i}' for i in range(1, 8)]

# O6 hand, 6 active joints. This only fixes the COLUMN order of /joint_actions;
# the command slots are mapped by name through the controller's `joints`.
_O6_ACTIVE = [
    'index_mcp_pitch', 'middle_mcp_pitch', 'pinky_mcp_pitch',
    'ring_mcp_pitch', 'thumb_cmc_pitch', 'thumb_cmc_yaw',
]
O6_LEFT_JOINTS = [f'L_{j}' for j in _O6_ACTIVE]
O6_RIGHT_JOINTS = [f'R_{j}' for j in _O6_ACTIVE]

# Fixed publish order — must stay in sync with vla_control's
# config/robot.py get_joint_order("openarm_linkerhand_o6").
ALL_JOINT_NAMES = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + O6_LEFT_JOINTS + O6_RIGHT_JOINTS

DEFAULT_CONTROLLERS = {
    'left_arm': 'left_forward_position_controller',
    'right_arm': 'right_forward_position_controller',
    'left_hand': 'left_hand_forward_position_controller',
    'right_hand': 'right_hand_forward_position_controller',
}

# A source is stale after 3 missed messages at its own measured rate; the floor
# keeps a fast source from tripping on ordinary jitter.
STALE_PERIODS = 3.0
STALE_FLOOR_SEC = 0.03
# Arm commands of one IK step arrive back to back; an arm's pending command that
# arrived more than this fraction of a period before the newest one belongs to the
# previous step. See _build_sample_locked.
PAIR_FRACTION = 0.5
# Seeds the hands' rate estimate until it is measured. pico_o6_hand_trigger.py
# publishes at 30 Hz by default; seeding lower than the real rate only widens
# the first stale window, seeding higher would trip it on the first messages.
HAND_SEED_RATE = 30.0


class _LimbSource:
    """Per-limb command state: latest value, arrival time, measured rate."""

    def __init__(self, key: str, joint_names: List[str], subscribed: bool,
                 expected_rate: float):
        self.key = key
        self.joint_names = joint_names
        self.subscribed = subscribed
        self.value: Optional[List[float]] = None
        self.last_t: Optional[float] = None   # seconds, node clock
        self.latched = False                  # has ever delivered a command
        self.advanced = False                 # new data since the last publish
        self.period_ema = 1.0 / expected_rate if expected_rate > 0 else 0.02
        self.rejects = 0

    def accept(self, values: List[float], now: float):
        if self.last_t is not None:
            dt = now - self.last_t
            if 0.0 < dt < 1.0:   # ignore the first sample and long gaps
                self.period_ema = 0.85 * self.period_ema + 0.15 * dt
        self.value = values
        self.last_t = now
        self.latched = True
        self.advanced = True

    @property
    def stale_timeout(self) -> float:
        return max(STALE_PERIODS * self.period_ema, STALE_FLOOR_SEC)

    def age(self, now: float) -> float:
        return float('inf') if self.last_t is None else now - self.last_t


class _SlotOrder:
    """Slot order of an unnamed FPC command, read from the controller's `joints`."""

    def __init__(self, controller: str, expected: List[str]):
        self.controller = controller
        self.expected = expected
        self.order: Optional[List[str]] = None   # None until the controller answers
        self.client = None
        self.future = None
        self.timer = None


class JointActionsAggregator(Node):
    """Aggregates OpenArm arm and O6 hand commands into /joint_actions (26 joints)."""

    def __init__(self, hand_config: str = 'o6_both', publish_rate: float = 50.0,
                 arm_config: str = 'both', stale_timeout: float = 0.0,
                 controllers: Optional[Dict[str, str]] = None):
        super().__init__('joint_actions_aggregator')

        if arm_config not in ('both', 'left', 'right'):
            raise ValueError(f'unknown arm_config {arm_config!r}')
        if hand_config not in ('none', 'o6_left', 'o6_right', 'o6_both'):
            raise ValueError(f'unknown hand_config {hand_config!r}')
        self.arm_config = arm_config
        self.hand_config = hand_config
        # publish_rate does not drive publishing (that is event driven); it only
        # seeds the arm rate estimate before enough messages arrive to measure.
        self.stale_override = stale_timeout if stale_timeout > 0 else None
        self.controllers = dict(DEFAULT_CONTROLLERS, **(controllers or {}))

        self.joint_states_map: Dict[str, float] = {}
        self.lock = threading.Lock()
        self.msg_count = 0
        self._pause_t: Optional[float] = None   # when publishing last stopped
        self._pause_keys: List[str] = []        # which limbs stopped it

        self.sources: Dict[str, _LimbSource] = {
            'left_arm': _LimbSource('left_arm', LEFT_ARM_JOINTS,
                                    arm_config in ('both', 'left'), publish_rate),
            'right_arm': _LimbSource('right_arm', RIGHT_ARM_JOINTS,
                                     arm_config in ('both', 'right'), publish_rate),
            'left_hand': _LimbSource('left_hand', O6_LEFT_JOINTS,
                                     hand_config in ('o6_left', 'o6_both'),
                                     HAND_SEED_RATE),
            'right_hand': _LimbSource('right_hand', O6_RIGHT_JOINTS,
                                      hand_config in ('o6_right', 'o6_both'),
                                      HAND_SEED_RATE),
        }
        self.arm_keys = ('left_arm', 'right_arm')
        # Concatenation order of the published vector; must match ALL_JOINT_NAMES.
        self.limb_order = ('left_arm', 'right_arm', 'left_hand', 'right_hand')

        self._slot_orders: Dict[str, _SlotOrder] = {}
        for key, src in self.sources.items():
            if src.subscribed:
                self._watch_fpc(key, self.controllers[key], src.joint_names)
        # Fill for limbs that are inactive or never started. Never used to
        # substitute for a failed source.
        self.create_subscription(JointState, '/joint_states', self._joint_states_callback, 10)

        self.joint_actions_pub = self.create_publisher(JointState, '/joint_actions', 10)

        # Diagnostics only — publishing is driven by incoming arm commands, so a
        # source that goes quiet would otherwise never be reported.
        self.create_timer(1.0, self._report_health)

        log = self.get_logger()
        log.info('JointActionsAggregator started')
        log.info(f'  Arm config: {arm_config}, hand config: {hand_config}')
        log.info(f'  Total joints: {len(ALL_JOINT_NAMES)} (fixed)')
        for key, src in self.sources.items():
            log.info(f'  {key}: ' + (f'/{self.controllers[key]}/commands' if src.subscribed
                                     else 'not subscribed (filled from /joint_states)'))
        log.info('  Publishing on arm command arrival (no fixed-rate timer)')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ── Command sources (unnamed FPC commands) ────────────────────────────────

    def _watch_fpc(self, key: str, controller: str, expected: List[str]):
        """
        Subscribe to an unnamed FPC command and ask the controller for its `joints`.

        The answer says which slot is which joint; retried until it comes.
        """
        so = _SlotOrder(controller, expected)
        so.client = self.create_client(GetParameters, f'/{controller}/get_parameters')
        so.timer = self.create_timer(1.0, lambda k=key: self._query_slot_order(k))
        self._slot_orders[key] = so
        self.create_subscription(Float64MultiArray, f'/{controller}/commands',
                                 lambda msg, k=key: self._fpc_command(k, msg), 10)

    def _query_slot_order(self, key: str):
        """Ask the controller for its `joints` until it answers once (timer)."""
        so = self._slot_orders[key]
        if so.order is not None:
            so.timer.cancel()
            return
        if so.future is not None and not so.future.done():
            return
        if not so.client.service_is_ready():
            return   # not loaded (yet); _report_health says so
        so.future = so.client.call_async(GetParameters.Request(names=['joints']))
        so.future.add_done_callback(lambda f, k=key: self._on_slot_order(k, f))

    def _on_slot_order(self, key: str, future):
        so = self._slot_orders[key]
        try:
            values = future.result().values
        except Exception as exc:  # service went away mid-call; the timer retries
            self.get_logger().warn(f'reading /{so.controller} `joints` failed: {exc}')
            return
        order = list(values[0].string_array_value) if values else []
        if sorted(order) != sorted(so.expected):
            self.get_logger().error(
                f'/{so.controller} `joints` = {order}, expected the joints {so.expected} '
                f'in some order — {key} commands stay rejected',
                throttle_duration_sec=10.0)
            return
        so.order = order
        self.get_logger().info(f'{key} slot order (from /{so.controller}): {order}')

    def _reject(self, src: _LimbSource, reason: str):
        src.rejects += 1
        self.get_logger().warn(
            f'{src.key}: {reason} — ignoring message ({src.rejects} rejected so far)',
            throttle_duration_sec=5.0)

    def _fpc_command(self, key: str, msg: Float64MultiArray):
        """Map unnamed slots by the controller's `joints`; arms then try to publish."""
        src = self.sources[key]
        so = self._slot_orders[key]
        order = so.order
        if order is None:
            self._reject(src, f'slot order unknown — /{so.controller} has not reported '
                              f'its `joints` (not loaded?), so this is not recorded')
            return
        if len(msg.data) != len(order):
            self._reject(src, f'command has {len(msg.data)} values, expected {len(order)}')
            return
        by_name = dict(zip(order, msg.data))
        values = [float(by_name[n]) for n in src.joint_names]
        with self.lock:
            src.accept(values, self._now())
            sample = self._build_sample_locked() if key in self.arm_keys else None
        if sample is not None:
            self._publish(*sample)

    def _joint_states_callback(self, msg: JointState):
        with self.lock:
            for n, p in zip(msg.name, msg.position):
                self.joint_states_map[n] = p

    # ── Aggregation and publishing ────────────────────────────────────────────

    def _fresh(self, src: _LimbSource, now: float) -> bool:
        timeout = self.stale_override if self.stale_override else src.stale_timeout
        return src.latched and src.age(now) <= timeout

    def _blockers_locked(self, now: float) -> List[str]:
        """Active limbs whose latest command is too old to publish with."""
        return [s.key for s in self.sources.values()
                if s.latched and not self._fresh(s, now)]

    def _build_sample_locked(self):
        """
        Build one aggregated sample, or None if it must not be published.

        Caller must hold self.lock. Publishing requires:
          1. at least one arm has ever delivered a command;
          2. every active arm has advanced since the last publish, and their
             commands are from the same IK step (paired by arrival time);
          3. no active limb is stale.
        """
        now = self._now()

        active_arms = [self.sources[k] for k in self.arm_keys if self.sources[k].latched]
        if not active_arms or not all(s.advanced for s in active_arms):
            return None

        # "Every arm advanced" alone can lock onto the wrong pairing for good: the IK
        # publishes left then right each step, the very first left command is
        # published alone (right not latched yet), and from then on every sample
        # would pair left step k with right step k-1. A pending command that arrived
        # more than PAIR_FRACTION of a period before the newest one is from the
        # previous step: drop its advanced flag and wait for its next command. Arms
        # on independent threads (no common step) still pair with their closest
        # neighbour, at the arm rate.
        newest = max(s.last_t for s in active_arms)
        window = PAIR_FRACTION * min(s.period_ema for s in active_arms)
        lagging = [s for s in active_arms if newest - s.last_t > window]
        if lagging:
            for s in lagging:
                s.advanced = False
            return None

        blockers = self._blockers_locked(now)
        if blockers:
            if self.msg_count and self._pause_t is None:
                self._pause_t, self._pause_keys = now, blockers
            return None

        positions: List[float] = []
        oldest = None
        for src in (self.sources[k] for k in self.limb_order):
            if src.latched:
                positions.extend(src.value)
                oldest = src.last_t if oldest is None else min(oldest, src.last_t)
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

        return positions, oldest, paused_for, paused_keys

    def _fill_from_state_locked(self, src: _LimbSource) -> Optional[List[float]]:
        """Latest /joint_states values for an inactive limb; None withholds the sample."""
        missing = [n for n in src.joint_names if n not in self.joint_states_map]
        if missing:
            self.get_logger().warn(
                f'{src.key}: inactive and {missing} absent from /joint_states — '
                f'withholding /joint_actions (nothing truthful to fill with)',
                throttle_duration_sec=5.0)
            return None
        return [self.joint_states_map[n] for n in src.joint_names]

    def _publish(self, positions: List[float], stamp_sec: float,
                 paused_for: Optional[float], paused_keys: Optional[List[str]]):
        msg = JointState()
        # Arrival time of the OLDEST contributing command (FPC commands carry no
        # header), not the publish instant.
        msg.header.stamp.sec = int(stamp_sec)
        msg.header.stamp.nanosec = int((stamp_sec - int(stamp_sec)) * 1e9)
        msg.name = list(ALL_JOINT_NAMES)
        msg.position = positions
        self.joint_actions_pub.publish(msg)

        self.msg_count += 1
        if self.msg_count == 1:
            self.get_logger().info(
                f'Started publishing to /joint_actions ({len(positions)} joints)')
        elif paused_for is not None:
            self.get_logger().info(
                f'resumed /joint_actions after {paused_for * 1000:.0f}ms '
                f'waiting on {paused_keys}')

    # ── Diagnostics ───────────────────────────────────────────────────────────

    def _report_health(self):
        with self.lock:
            now = self._now()
            unknown = [k for k, so in self._slot_orders.items() if so.order is None]
            waiting = [s.key for s in self.sources.values()
                       if s.subscribed and not s.latched]
            # An arm that went quiet never reaches the stale check in
            # _build_sample_locked (nothing calls it), so check freshness here.
            blockers = self._blockers_locked(now)
            ages = {s.key: s.age(now) for s in self.sources.values() if s.latched}
            any_arm = any(self.sources[k].latched for k in self.arm_keys)
            if blockers and self.msg_count and self._pause_t is None:
                self._pause_t, self._pause_keys = now, blockers

        log = self.get_logger()
        if unknown:
            log.info(
                'slot order not known yet for '
                + ', '.join(f'{k} (/{self._slot_orders[k].controller})' for k in unknown)
                + ' — their commands are not recorded until the controller is loaded',
                throttle_duration_sec=30.0)
        if waiting:
            # Deliberately repeated: an operator who MEANT to run this source sees
            # the same log line as one who did not, so it must keep nagging.
            log.warn(
                f'no command since startup on {waiting} — treating as inactive and '
                f'filling from /joint_states', throttle_duration_sec=5.0)
        if not any_arm:
            log.info('waiting for the first arm command before publishing',
                     throttle_duration_sec=5.0)
        elif blockers:
            detail = ', '.join(f'{k}={ages[k] * 1000:.0f}ms' for k in blockers)
            log.warn(f'stale: {detail} — /joint_actions paused until it recovers',
                     throttle_duration_sec=2.0)


def main():
    parser = argparse.ArgumentParser(
        description='Aggregate OpenArm arm/hand commands to /joint_actions (26 joints)')
    parser.add_argument('--arm_config', default='both', choices=['both', 'left', 'right'],
                        help='Arms to subscribe to. The other arm still occupies its '
                             'columns, filled from /joint_states. With both, single-arm '
                             'teleop works too: only arms that ever sent a command gate '
                             'publishing.')
    parser.add_argument('--hand_config', default='o6_both',
                        choices=['none', 'o6_left', 'o6_right', 'o6_both'],
                        help='Hands to subscribe to. Unsubscribed hands still occupy '
                             'their columns; the vector is always 26 joints.')
    parser.add_argument('--publish_rate', type=float, default=50.0,
                        help='Expected arm command rate (Hz), only seeds the staleness '
                             'window. Publishing is driven by arm commands.')
    parser.add_argument('--stale_timeout', type=float, default=0.0,
                        help='Fixed staleness window (s) for every source. '
                             '0 = derive per source from the measured rate.')
    for key, name in DEFAULT_CONTROLLERS.items():
        parser.add_argument(f'--{key}_controller', default=name,
                            help=f'Controller whose /<name>/commands and `joints` are '
                                 f'used for {key} (default: {name}).')
    # parse_known_args: ros2 launch appends --ros-args ... to ExecuteProcess cmds.
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = JointActionsAggregator(
        hand_config=args.hand_config,
        publish_rate=args.publish_rate,
        arm_config=args.arm_config,
        stale_timeout=args.stale_timeout,
        controllers={k: getattr(args, f'{k}_controller') for k in DEFAULT_CONTROLLERS},
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C under ros2 launch delivers SIGINT twice (terminal + launch); a second
        # one inside destroy_node() would end in a traceback and a non-zero exit.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
