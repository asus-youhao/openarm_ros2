#!/usr/bin/env python3
"""
End-to-end check of /joint_actions through openarm_o6_bimanual.launch.py on FAKE hardware.

Every launch is use_fake_hardware:=true, on its own unused ROS_DOMAIN_ID with
localhost discovery and 'ulimit -l 64' (same rules as
openarm_description/test/fake_smoke.py), so it cannot reach a real robot.

  F  robot_controller:=forward_position_controller
     F1 jsb + 4 FPCs active               F2 aggregator read the 4 FPC `joints`
     F3 26 names, one message per arm tick, one publisher
     F4 each value equals the command sent for that joint
     F5 /joint_states (the fake hardware follows the FPCs) equals /joint_actions
     F6 fake_arm_hand_feed.py drives /joint_actions
  J  robot_controller:=joint_trajectory_controller (FPCs not loaded)
     J1 aggregator up (one publisher)      J2 FPC-topic commands -> nothing published
     J3 the log says why                   J4 fake_arm_hand_feed.py refuses to run
  O  with_joint_actions:=false
     O1 no publisher on /joint_actions     O2 joint_actions.launch.py on its own works
     O3 Ctrl-C (SIGINT to the group): the aggregator exits cleanly

Usage (workspace with openarm_bringup sourced):
    python3 scripts/data_collection/test/check_bringup_fake.py
"""

import os
import sys
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

from common import (ALL_JOINT_NAMES, CONTROLLERS, HERE, isolate, LIMB_JOINTS, pick_domains,
                    read, Report, start, stop, wait_for)
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

FEED = os.path.join(os.path.dirname(HERE), 'fake_arm_hand_feed.py')
LAUNCH = 'openarm_o6_bimanual.launch.py'
ARMS = ('left_arm', 'right_arm')
HANDS = ('left_hand', 'right_hand')


class Probe:
    """rclpy side of one case: services, /joint_actions + /joint_states sinks, commands."""

    def __init__(self, domain_id):
        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=domain_id)
        self.node = rclpy.create_node('joint_actions_bringup_probe', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.lock = threading.Lock()
        self.actions = []      # (monotonic, JointState)
        self.js = {}
        self.description = None
        self.node.create_subscription(JointState, '/joint_actions', self._on_action, 100)
        self.node.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.node.create_subscription(
            String, '/robot_description', self._on_desc,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pubs = {k: self.node.create_publisher(Float64MultiArray, f'/{c}/commands', 10)
                     for k, c in CONTROLLERS.items()}
        self._spin = True
        self.thread = threading.Thread(target=self._spin_loop, daemon=True)
        self.thread.start()

    def _spin_loop(self):
        while self._spin and self.context.ok():
            self.executor.spin_once(timeout_sec=0.02)

    def _on_action(self, msg):
        with self.lock:
            self.actions.append((time.monotonic(), msg))

    def _on_js(self, msg):
        with self.lock:
            self.js.update(zip(msg.name, msg.position))

    def _on_desc(self, msg):
        self.description = msg.data

    def call(self, srv_type, name, request, timeout=5.0):
        cli = self.node.create_client(srv_type, name)
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                return None
            future = cli.call_async(request)
            end = time.monotonic() + timeout
            while not future.done() and time.monotonic() < end:
                time.sleep(0.02)
            return future.result() if future.done() else None
        finally:
            self.node.destroy_client(cli)

    def active_controllers(self):
        resp = self.call(ListControllers, '/controller_manager/list_controllers',
                         ListControllers.Request(), timeout=2.0)
        return {c.name for c in resp.controller if c.state == 'active'} if resp else set()

    def joints_of(self, controller):
        resp = self.call(GetParameters, f'/{controller}/get_parameters',
                         GetParameters.Request(names=['joints']))
        return list(resp.values[0].string_array_value) if resp and resp.values else None

    def since(self, t):
        with self.lock:
            return [m for m in self.actions if m[0] >= t]

    def wait_subscribed(self, count, timeout=15.0):
        """Wait until every command topic has `count` subscribers (FPC + aggregator)."""
        return wait_for(lambda: all(p.get_subscription_count() >= count
                                    for p in self.pubs.values()), timeout)

    def feed(self, duration, values, orders):
        """Constant commands: arms 50 Hz, hands 30 Hz; return arm ticks sent."""
        t0 = time.monotonic()
        next_arm = next_hand = t0
        ticks = 0
        while time.monotonic() < t0 + duration:
            now = time.monotonic()
            if now >= next_arm:
                ticks += 1
                for limb in ARMS:
                    self.pubs[limb].publish(Float64MultiArray(
                        data=[values[n] for n in orders[limb]]))
                next_arm += 0.02
            if now >= next_hand:
                for limb in HANDS:
                    self.pubs[limb].publish(Float64MultiArray(
                        data=[values[n] for n in orders[limb]]))
                next_hand += 1.0 / 30.0
            time.sleep(0.0005)
        return ticks

    def close(self):
        self._spin = False
        self.thread.join(timeout=2.0)
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)


def start_launch(args, log_path):
    cmd = 'ulimit -l 64; exec ros2 launch openarm_bringup {} use_fake_hardware:=true ' \
          'launch_rviz:=false {}'.format(LAUNCH, ' '.join(args))
    return start(['bash', '-c', cmd], log_path)


def targets_from_urdf(description):
    """Pick a distinct in-limit target per joint (30..70 % of its range)."""
    limits = {}
    for j in ET.fromstring(description).findall('joint'):
        lim = j.find('limit')
        if lim is not None and lim.get('lower') is not None:
            limits[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
    return {n: limits[n][0] + (limits[n][1] - limits[n][0]) * (0.30 + 0.015 * i)
            for i, n in enumerate(ALL_JOINT_NAMES)}


def case_fpc(domain, rep, log_dir):
    rep.section(f'F: {LAUNCH} robot_controller:=forward_position_controller (domain {domain})')
    isolate(domain)
    log = os.path.join(log_dir, 'launch_fpc.log')
    proc = start_launch(['robot_controller:=forward_position_controller'], log)
    probe = Probe(domain)
    try:
        want = {'joint_state_broadcaster', *CONTROLLERS.values()}
        ok = wait_for(lambda: want <= probe.active_controllers(), 40.0, step=0.5)
        if not rep.check(ok, 'F1', f'active: {sorted(probe.active_controllers())}'):
            return
        ok = wait_for(lambda: read(log).count('slot order (from') >= 4, 20.0)
        rep.check(ok, 'F2', 'aggregator read `joints` from the 4 FPCs')
        orders = {k: probe.joints_of(c) for k, c in CONTROLLERS.items()}
        wait_for(lambda: probe.description is not None, 10.0)
        values = targets_from_urdf(probe.description)
        probe.wait_subscribed(2)

        t = time.monotonic()
        ticks = probe.feed(3.0, values, orders)
        time.sleep(0.3)
        msgs = probe.since(t)
        names_ok = all(list(m[1].name) == ALL_JOINT_NAMES for m in msgs)
        pubs = probe.node.count_publishers('/joint_actions')
        rep.check(msgs and names_ok and ticks - 3 <= len(msgs) <= ticks and pubs == 1, 'F3',
                  f'{len(msgs)} messages / {ticks} arm ticks, 26 names in order, '
                  f'{pubs} publisher')
        worst = max((abs(p - values[n]) for m in msgs[1:]
                     for n, p in zip(m[1].name, m[1].position)), default=float('inf'))
        rep.check(worst < 1e-9, 'F4', f'every value is the command sent for that joint '
                                      f'(max error {worst:.1e})')
        last = dict(zip(msgs[-1][1].name, msgs[-1][1].position)) if msgs else {}
        with probe.lock:
            js = dict(probe.js)
        diff = max((abs(js[n] - last[n]) for n in ALL_JOINT_NAMES if n in js and n in last),
                   default=float('inf'))
        rep.check(len(last) == 26 and all(n in js for n in ALL_JOINT_NAMES) and diff < 1e-3,
                  'F5', f'/joint_states follows the FPCs and equals /joint_actions for all '
                        f'26 joints (max diff {diff:.1e} rad)')

        t = time.monotonic()
        feed = start([sys.executable, FEED], os.path.join(log_dir, 'feed_fpc.log'))
        try:
            ok = wait_for(lambda: len(probe.since(t)) > 100, 15.0)
        finally:
            stop(feed)
        rep.check(ok, 'F6', f'fake_arm_hand_feed.py drives /joint_actions '
                            f'({len(probe.since(t))} messages)')
    finally:
        probe.close()
        stop(proc)


def case_jtc(domain, rep, log_dir):
    rep.section(f'J: {LAUNCH} robot_controller:=joint_trajectory_controller (domain {domain})')
    isolate(domain)
    log = os.path.join(log_dir, 'launch_jtc.log')
    proc = start_launch(['robot_controller:=joint_trajectory_controller'], log)
    probe = Probe(domain)
    try:
        want = {'joint_state_broadcaster', 'left_joint_trajectory_controller',
                'right_joint_trajectory_controller', 'left_hand_controller',
                'right_hand_controller'}
        if not wait_for(lambda: want <= probe.active_controllers(), 40.0, step=0.5):
            rep.check(False, 'J0', f'active: {sorted(probe.active_controllers())}')
            return
        ok = wait_for(lambda: 'JointActionsAggregator started' in read(log)
                      and probe.node.count_publishers('/joint_actions') == 1, 20.0)
        rep.check(ok, 'J1', 'aggregator started by the bringup, one publisher')
        probe.wait_subscribed(1)
        time.sleep(1.5)
        orders = {k: LIMB_JOINTS[k] for k in CONTROLLERS}
        values = {n: 0.1 for n in ALL_JOINT_NAMES}
        t = time.monotonic()
        probe.feed(3.0, values, orders)
        time.sleep(0.3)
        rep.check(not probe.since(t), 'J2', 'commands on the FPC topics (FPCs not loaded): '
                                            'nothing published')
        text = read(log)
        rep.check('slot order not known yet for left_arm' in text
                  and 'left_arm: slot order unknown' in text, 'J3',
                  'log: slot order not known / commands not recorded')
        feed_log = os.path.join(log_dir, 'feed_jtc.log')
        feed = start([sys.executable, FEED], feed_log)
        code = feed.wait(timeout=30)
        rep.check(code == 1 and 'refusing to feed' in read(feed_log), 'J4',
                  'fake_arm_hand_feed.py refuses to run (FPC not loaded)')
    finally:
        probe.close()
        stop(proc)


def case_off(domain, rep, log_dir):
    rep.section(f'O: {LAUNCH} with_joint_actions:=false (domain {domain})')
    isolate(domain)
    log = os.path.join(log_dir, 'launch_off.log')
    proc = start_launch(['robot_controller:=forward_position_controller',
                         'with_joint_actions:=false'], log)
    probe = Probe(domain)
    standalone = None
    try:
        want = {'joint_state_broadcaster', *CONTROLLERS.values()}
        if not wait_for(lambda: want <= probe.active_controllers(), 40.0, step=0.5):
            rep.check(False, 'O0', f'active: {sorted(probe.active_controllers())}')
            return
        time.sleep(6.0)   # past the bringup's aggregator delay
        orders = {k: probe.joints_of(c) for k, c in CONTROLLERS.items()}
        values = {n: 0.05 for n in ALL_JOINT_NAMES}
        probe.wait_subscribed(1)
        t = time.monotonic()
        probe.feed(1.5, values, orders)
        rep.check(probe.node.count_publishers('/joint_actions') == 0 and not probe.since(t)
                  and 'joint_actions_aggregator' not in read(log), 'O1',
                  'no aggregator, no publisher on /joint_actions')

        standalone_log = os.path.join(log_dir, 'launch_standalone.log')
        standalone = start(['ros2', 'launch', 'openarm_bringup', 'joint_actions.launch.py'],
                           standalone_log)
        ok = wait_for(lambda: read(standalone_log).count('slot order (from') >= 4, 20.0)
        ok = ok and probe.wait_subscribed(2)
        t = time.monotonic()
        ticks = probe.feed(2.0, values, orders)
        time.sleep(0.3)
        n = len(probe.since(t))
        rep.check(ok and ticks - 3 <= n <= ticks
                  and probe.node.count_publishers('/joint_actions') == 1, 'O2',
                  f'joint_actions.launch.py on its own: {n} messages / {ticks} ticks')
        stop(standalone)   # SIGINT to the whole group, as Ctrl-C in a terminal does
        standalone, text = None, read(standalone_log)
        rep.check('Traceback' not in text and 'process has finished cleanly' in text, 'O3',
                  'SIGINT (Ctrl-C): aggregator exits cleanly, no traceback')
    finally:
        if standalone is not None:
            stop(standalone)
        probe.close()
        stop(proc)


def main():
    log_dir = tempfile.mkdtemp(prefix='joint_actions_bringup_')
    domains = pick_domains(3, start=180)
    print(f'domains {domains}, logs in {log_dir}', flush=True)
    rep = Report()
    for case, domain in zip((case_fpc, case_jtc, case_off), domains):
        case(domain, rep, log_dir)
    print(f'\n{"ALL PASS" if not rep.failed else f"{rep.failed} FAILED"}  (logs: {log_dir})')
    return 1 if rep.failed else 0


if __name__ == '__main__':
    sys.exit(main())
