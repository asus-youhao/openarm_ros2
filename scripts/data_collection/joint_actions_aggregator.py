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
never answers, so its limb is always filled from /joint_states.

The published vector ALWAYS carries all 26 joints in a fixed order
(ALL_JOINT_NAMES), whatever --arm_config / --hand_config is. Downstream
(vla_control's data_collector) looks joints up by name and drops the whole
sample if any expected name is absent, so a shrinking vector would silently
record nothing.

Publishing model (fixed-rate timer, like phu_controller_ros2 #98)
-----------------------------------------------------------------
A sample goes out on every tick of a --publish_rate timer (default 50 Hz) from
startup, whether or not any command has arrived. Per limb, at each tick:

  - a LIVE command (arrived within the staleness window) -> that command
  - no live command: never sent one, the upstream stopped, the controller is
    not loaded, or the limb is not subscribed              -> latest /joint_states
  - ...and a needed joint is not in /joint_states yet      -> this tick is skipped
                                                              (never a made-up 0.0)

So with no command at all, /joint_actions is /joint_states; while a limb is
commanded its columns are the commands; when its upstream stops for longer than
the staleness window (3 measured periods, at least 30 ms) its columns go back to
/joint_states. Unlike phu #98, which keeps the last command after the upstream
stops, a stopped limb is filled from /joint_states. header.stamp is the publish
instant.

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

# A command is live until 3 periods of its own measured rate have passed without
# a new one; the floor keeps a fast source from dropping out on ordinary jitter.
STALE_PERIODS = 3.0
STALE_FLOOR_SEC = 0.03
# Seed the rate estimates until they are measured: the placo IK runs at 50 Hz,
# pico_o6_hand_trigger.py at 30 Hz. Seeding lower than the real rate only widens
# the first window; seeding higher would drop the first commands to /joint_states.
ARM_SEED_RATE = 50.0
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
        self.period_ema = 1.0 / expected_rate
        self.rejects = 0
        self.using: Optional[str] = None      # 'command' / 'joint_states', for the log

    def accept(self, values: List[float], now: float):
        if self.last_t is not None:
            dt = now - self.last_t
            if 0.0 < dt < 1.0:   # ignore the first sample and long gaps
                self.period_ema = 0.85 * self.period_ema + 0.15 * dt
        self.value = values
        self.last_t = now

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
        if publish_rate <= 0:
            raise ValueError(f'publish_rate must be > 0, got {publish_rate}')
        self.arm_config = arm_config
        self.hand_config = hand_config
        self.stale_override = stale_timeout if stale_timeout > 0 else None
        self.controllers = dict(DEFAULT_CONTROLLERS, **(controllers or {}))

        self.joint_states_map: Dict[str, float] = {}
        self.lock = threading.Lock()
        self.msg_count = 0

        self.sources: Dict[str, _LimbSource] = {
            'left_arm': _LimbSource('left_arm', LEFT_ARM_JOINTS,
                                    arm_config in ('both', 'left'), ARM_SEED_RATE),
            'right_arm': _LimbSource('right_arm', RIGHT_ARM_JOINTS,
                                     arm_config in ('both', 'right'), ARM_SEED_RATE),
            'left_hand': _LimbSource('left_hand', O6_LEFT_JOINTS,
                                     hand_config in ('o6_left', 'o6_both'),
                                     HAND_SEED_RATE),
            'right_hand': _LimbSource('right_hand', O6_RIGHT_JOINTS,
                                      hand_config in ('o6_right', 'o6_both'),
                                      HAND_SEED_RATE),
        }
        # Concatenation order of the published vector; must match ALL_JOINT_NAMES.
        self.limb_order = ('left_arm', 'right_arm', 'left_hand', 'right_hand')

        self._slot_orders: Dict[str, _SlotOrder] = {}
        for key, src in self.sources.items():
            if src.subscribed:
                self._watch_fpc(key, self.controllers[key], src.joint_names)
        self.create_subscription(JointState, '/joint_states', self._joint_states_callback, 10)

        self.joint_actions_pub = self.create_publisher(JointState, '/joint_actions', 10)
        self.create_timer(1.0 / publish_rate, self._publish_tick)
        self.create_timer(1.0, self._report_health)   # diagnostics only

        log = self.get_logger()
        log.info('JointActionsAggregator started')
        log.info(f'  Arm config: {arm_config}, hand config: {hand_config}')
        log.info(f'  Total joints: {len(ALL_JOINT_NAMES)} (fixed)')
        for key, src in self.sources.items():
            log.info(f'  {key}: ' + (f'/{self.controllers[key]}/commands' if src.subscribed
                                     else 'not subscribed (filled from /joint_states)'))
        log.info(f'  Publishing at {publish_rate:g} Hz; a limb without a live command is '
                 f'filled from /joint_states')

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
        """Map unnamed slots by the controller's `joints` and keep the latest value."""
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

    def _joint_states_callback(self, msg: JointState):
        with self.lock:
            for n, p in zip(msg.name, msg.position):
                self.joint_states_map[n] = p

    # ── Aggregation and publishing ────────────────────────────────────────────

    def _live(self, src: _LimbSource, now: float) -> bool:
        """Whether the limb has a command recent enough to record."""
        timeout = self.stale_override if self.stale_override else src.stale_timeout
        return src.age(now) <= timeout

    def _build_sample_locked(self, now: float):
        """
        Return (positions, which source each limb used), or None to skip this tick.

        Caller must hold self.lock. A tick is skipped only when a limb without a
        live command has joints that /joint_states does not carry (yet).
        """
        positions: List[float] = []
        using: Dict[str, str] = {}
        for key in self.limb_order:
            src = self.sources[key]
            if self._live(src, now):
                positions.extend(src.value)
                using[key] = 'command'
                continue
            missing = [n for n in src.joint_names if n not in self.joint_states_map]
            if missing:
                self.get_logger().warn(
                    f'{key}: no live command and {missing} absent from /joint_states — '
                    f'skipping this sample (nothing truthful to fill with)',
                    throttle_duration_sec=5.0)
                return None
            positions.extend(self.joint_states_map[n] for n in src.joint_names)
            using[key] = 'joint_states'
        return positions, using

    def _publish_tick(self):
        """Timer: publish one sample."""
        with self.lock:
            now = self._now()
            built = self._build_sample_locked(now)
            ages = {k: s.age(now) for k, s in self.sources.items()}
        if built is None:
            return
        positions, using = built
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ALL_JOINT_NAMES)
        msg.position = positions
        self.joint_actions_pub.publish(msg)

        self.msg_count += 1
        if self.msg_count == 1:
            self.get_logger().info(
                f'Started publishing to /joint_actions ({len(positions)} joints)')
        self._log_switches(using, ages)

    def _log_switches(self, using: Dict[str, str], ages: Dict[str, float]):
        """Log when a subscribed limb switches between its command and /joint_states."""
        for key, now_using in using.items():
            src = self.sources[key]
            before, src.using = src.using, now_using
            if not src.subscribed or before is None or before == now_using:
                continue
            if now_using == 'command':
                self.get_logger().info(f'{key}: commands arriving — recording them')
            else:
                self.get_logger().info(
                    f'{key}: no command for {ages[key] * 1000:.0f}ms — '
                    f'filling from /joint_states')

    # ── Diagnostics ───────────────────────────────────────────────────────────

    def _report_health(self):
        with self.lock:
            unknown = [k for k, so in self._slot_orders.items() if so.order is None]
            never = [s.key for s in self.sources.values()
                     if s.subscribed and s.last_t is None]

        log = self.get_logger()
        if unknown:
            log.info(
                'slot order not known yet for '
                + ', '.join(f'{k} (/{self._slot_orders[k].controller})' for k in unknown)
                + ' — their commands are not recorded until the controller is loaded',
                throttle_duration_sec=30.0)
        if never:
            log.info(f'no command since startup on {never} — filling from /joint_states',
                     throttle_duration_sec=10.0)


def main():
    parser = argparse.ArgumentParser(
        description='Aggregate OpenArm arm/hand commands to /joint_actions (26 joints)')
    parser.add_argument('--arm_config', default='both', choices=['both', 'left', 'right'],
                        help='Arms to subscribe to. The other arm still occupies its '
                             'columns, filled from /joint_states.')
    parser.add_argument('--hand_config', default='o6_both',
                        choices=['none', 'o6_left', 'o6_right', 'o6_both'],
                        help='Hands to subscribe to. Unsubscribed hands still occupy '
                             'their columns; the vector is always 26 joints.')
    parser.add_argument('--publish_rate', type=float, default=50.0,
                        help='Publish rate of /joint_actions (Hz).')
    parser.add_argument('--stale_timeout', type=float, default=0.0,
                        help='How long (s) a command stays live without a new one; past '
                             'it the limb is filled from /joint_states. 0 = 3 periods of '
                             'the measured rate (at least 30 ms).')
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
