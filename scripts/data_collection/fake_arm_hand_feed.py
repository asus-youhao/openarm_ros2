#!/usr/bin/env python3
"""
fake_arm_hand_feed.py — FAKE HARDWARE ONLY.

Stand-in for the IK / hand teleop upstream when trying joint_actions_aggregator.py
on `use_fake_hardware:=true`. Publishes a small sine around the pose read once
from /joint_states, on the same topics the real upstream uses:
  - arms  -> /{left,right}_forward_position_controller/commands       --arm_hz
  - hands -> /{left,right}_hand_forward_position_controller/commands  --hand_hz

Each controller's slot order is read from its `joints` parameter, like the
aggregator does, so the feed never assumes an order either. Every command is
clamped to the joint limits in /robot_description.

It refuses to run unless every controller_manager hardware component it can see
is a mock (mock_components/GenericSystem), so it cannot move a real arm.

Usage (bringup with robot_controller:=forward_position_controller first):
    python3 scripts/data_collection/fake_arm_hand_feed.py
    python3 scripts/data_collection/fake_arm_hand_feed.py --no_hands --amp 0.2
"""

import argparse
import math
import signal
import sys
import time
import xml.etree.ElementTree as ET

from controller_manager_msgs.srv import ListHardwareComponents
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

ARM_CONTROLLERS = ['left_forward_position_controller', 'right_forward_position_controller']
HAND_CONTROLLERS = ['left_hand_forward_position_controller',
                    'right_hand_forward_position_controller']
MOCK_PLUGIN = 'mock_components/GenericSystem'


class FakeFeed(Node):

    def __init__(self, arm_hz, hand_hz, amp, hands):
        super().__init__('fake_arm_hand_feed')
        self.amp = amp
        self.home = None
        self.t0 = None
        self.order = {}
        self.limits = None   # joint -> (lower, upper), from /robot_description
        self.ready = False   # set once every controller's `joints` has been read
        controllers = ARM_CONTROLLERS + (HAND_CONTROLLERS if hands else [])
        self.pubs = {c: self.create_publisher(Float64MultiArray, f'/{c}/commands', 10)
                     for c in controllers}
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.create_subscription(
            String, '/robot_description', self._on_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1.0 / arm_hz, lambda: self._tick(ARM_CONTROLLERS, False))
        if hands:
            self.create_timer(1.0 / hand_hz, lambda: self._tick(HAND_CONTROLLERS, True))

    def _call(self, srv_type, name, request, timeout=5.0):
        cli = self.create_client(srv_type, name)
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                return None
            future = cli.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            return future.result()
        finally:
            self.destroy_client(cli)

    def check_mock_only(self):
        """Return an error string unless every hardware component is a mock."""
        resp = self._call(ListHardwareComponents, '/controller_manager/list_hardware_components',
                          ListHardwareComponents.Request())
        if resp is None:
            return 'no /controller_manager/list_hardware_components'
        # Jazzy reports the plugin in `plugin_name`; Humble only has `class_type`.
        plugins = {c.name: getattr(c, 'plugin_name', '') or c.class_type
                   for c in resp.component}
        real = [f'{name} ({plugin})' for name, plugin in plugins.items()
                if plugin != MOCK_PLUGIN]
        return f'not fake hardware: {real}' if real else None

    def read_orders(self):
        for c in self.pubs:
            resp = self._call(GetParameters, f'/{c}/get_parameters',
                              GetParameters.Request(names=['joints']))
            if resp is None or not resp.values:
                return f'/{c} is not loaded (robot_controller:=forward_position_controller?)'
            self.order[c] = list(resp.values[0].string_array_value)
        end = time.monotonic() + 5.0
        while self.limits is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.limits is None:
            return 'no /robot_description (needed for the joint limits)'
        self.ready = True
        return None

    def _on_description(self, msg):
        limits = {}
        for j in ET.fromstring(msg.data).findall('joint'):
            lim = j.find('limit')
            if lim is not None and lim.get('lower') is not None:
                limits[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
        self.limits = limits

    def _clamp(self, name, value):
        lo, hi = self.limits.get(name, (-math.inf, math.inf))
        return min(max(value, lo), hi)

    def _on_js(self, msg):
        # /joint_states is already spun while the orders are read; wait for all.
        if self.home is None and self.ready:
            pos = dict(zip(msg.name, msg.position))
            if all(n in pos for names in self.order.values() for n in names):
                self.home = pos
                self.t0 = time.monotonic()
                self.get_logger().info('start pose captured, feeding commands')

    def _tick(self, controllers, is_hand):
        if self.home is None:
            return
        t = time.monotonic() - self.t0
        ramp = min(t / 2.0, 1.0)   # fade in over 2 s so the first command == pose
        off = ramp * self.amp * math.sin(2 * math.pi * 0.2 * t)
        if is_hand:
            off = abs(off)         # O6 joints are [0, max]
        for c in controllers:
            self.pubs[c].publish(Float64MultiArray(
                data=[self._clamp(n, self.home[n] + off) for n in self.order[c]]))


def main():
    p = argparse.ArgumentParser(description='FAKE HARDWARE ONLY arm/hand FPC command feed')
    p.add_argument('--arm_hz', type=float, default=50.0)
    p.add_argument('--hand_hz', type=float, default=30.0)
    p.add_argument('--amp', type=float, default=0.1, help='sine amplitude (rad)')
    p.add_argument('--no_hands', action='store_true')
    args, _ = p.parse_known_args()

    rclpy.init()
    node = FakeFeed(args.arm_hz, args.hand_hz, args.amp, not args.no_hands)
    try:
        error = node.check_mock_only() or node.read_orders()
        if error:
            node.get_logger().error(f'refusing to feed: {error}')
            sys.exit(1)
        node.get_logger().info(f'slot orders: {node.order}')
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
