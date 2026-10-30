#!/usr/bin/env python3
"""
Check the publishing rules of joint_actions_aggregator.py without ros2_control.

The aggregator runs as its own process against stand-in controller nodes (a node
with the controller's name and a `joints` parameter, deliberately PERMUTED) and
commands published from here. /joint_states is published from here too, with
values that no command uses, so every column tells where it came from.
No hardware, no controller_manager.

  R1  slot order is read from each controller's `joints`, not assumed
  R2  no command at all -> /joint_states published at 50 Hz from startup
  R3  hand commands only -> that hand is the command, the rest /joint_states
  R4  26 names in the fixed order; every value mapped by name through the
      permuted `joints`; a never-commanded limb is /joint_states; 50 Hz
  R5  header.stamp is the publish instant
  R6  exactly one publisher on /joint_actions
  R7  arms stop -> publishing goes on, arm columns become /joint_states (logged);
      arms back -> commands again (logged)
  R8  a hand stops -> its columns become /joint_states, arms stay commands
  R9  wrong-length command rejected and logged; that arm falls back to /joint_states
  R10 a slower upstream (20 Hz) stays live between its commands; output stays 50 Hz
  R11 a limb without a live command missing from /joint_states -> tick skipped, never 0.0
  R12 single arm: the idle arm is /joint_states
  R13 controller not loaded / wrong `joints` -> commands rejected, /joint_states
      instead; recovers once the controller reports the right `joints`
  R14 --arm_config right --hand_config none: only the right arm is recorded

Usage (ROS sourced):  python3 scripts/data_collection/test/check_rules.py
"""

import os
import sys
import tempfile
import threading
import time

from common import (AGGREGATOR, ALL_JOINT_NAMES, CONTROLLERS, isolate, LIMB_JOINTS,
                    pick_domains, read, Report, start, stop, wait_for)

DOMAIN = pick_domains(1, start=170)[0]
isolate(DOMAIN)
# The feed loop and the executor thread share the GIL; the default 5 ms switch
# interval would add that much jitter to the command timing this check relies on.
sys.setswitchinterval(1e-4)

import rclpy  # noqa: E402  (after isolate(): the domain is read at init)
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

STEP = 1e-6   # per-tick offset; base values are 0.01 apart, so a tick never aliases
BASE = {n: 0.01 * (i + 1) for i, n in enumerate(ALL_JOINT_NAMES)}
JS = {n: -0.01 * (i + 1) for i, n in enumerate(ALL_JOINT_NAMES)}
RATE = 50.0   # the aggregator's default publish_rate

PERMUTED = {
    'left_arm': list(reversed(LIMB_JOINTS['left_arm'])),
    'right_arm': LIMB_JOINTS['right_arm'][3:] + LIMB_JOINTS['right_arm'][:3],
    'left_hand': ['L_thumb_cmc_yaw', 'L_index_mcp_pitch', 'L_ring_mcp_pitch',
                  'L_thumb_cmc_pitch', 'L_middle_mcp_pitch', 'L_pinky_mcp_pitch'],
    'right_hand': list(LIMB_JOINTS['right_hand']),
}
ARMS = ('left_arm', 'right_arm')


class Harness:
    """Stand-in controllers, command / joint_states publishers, /joint_actions sink."""

    def __init__(self):
        self.node = rclpy.create_node('joint_actions_rules_harness')
        self.pubs = {k: self.node.create_publisher(Float64MultiArray, f'/{c}/commands', 10)
                     for k, c in CONTROLLERS.items()}
        self.js_pub = self.node.create_publisher(JointState, '/joint_states', 10)
        self.js_names = list(ALL_JOINT_NAMES)
        self.lock = threading.Lock()
        self.msgs = []        # (arrival monotonic, arrival ROS seconds, JointState)
        self.node.create_subscription(JointState, '/joint_actions', self._on_action, 100)
        self.node.create_timer(0.01, self._publish_js)
        self.ctrl_nodes = {}
        self.orders = None
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def _on_action(self, msg):
        now = self.node.get_clock().now().nanoseconds * 1e-9
        with self.lock:
            self.msgs.append((time.monotonic(), now, msg))

    def _publish_js(self):
        names = list(self.js_names)
        self.js_pub.publish(JointState(name=names, position=[JS[n] for n in names]))

    def set_controllers(self, orders):
        """orders: key -> `joints` list, or None for a controller that is not loaded."""
        if orders == self.orders:
            return
        self.orders = dict(orders)
        for n in self.ctrl_nodes.values():
            self.executor.remove_node(n)
            n.destroy_node()
        self.ctrl_nodes = {}
        for key, order in orders.items():
            if order is not None:
                n = rclpy.create_node(CONTROLLERS[key], enable_rosout=False)
                n.declare_parameter('joints', order)
                self.executor.add_node(n)
                self.ctrl_nodes[key] = n

    def between(self, t0, t1=None):
        with self.lock:
            return [m for m in self.msgs if m[0] >= t0 and (t1 is None or m[0] < t1)]

    def wait_subscribed(self, limbs, timeout=10.0):
        """Wait until the aggregator has discovered our command publishers."""
        return wait_for(lambda: all(self.pubs[lb].get_subscription_count() >= 1
                                    for lb in limbs), timeout)

    def feed(self, duration, arms=(), hands=(), orders=PERMUTED, bad_left_arm=False,
             arm_hz=50.0):
        """Publish commands in each controller's slot order; return arm ticks sent."""
        t0 = time.monotonic()
        next_arm = next_hand = t0
        k = h = 0
        while time.monotonic() < t0 + duration:
            now = time.monotonic()
            if arms and now >= next_arm:
                k += 1
                for limb in arms:
                    data = [BASE[n] + k * STEP for n in orders[limb]]
                    if bad_left_arm and limb == 'left_arm':
                        data = data[:-1]
                    self.pubs[limb].publish(Float64MultiArray(data=data))
                next_arm += 1.0 / arm_hz
            if hands and now >= next_hand:
                h += 1
                for limb in hands:
                    self.pubs[limb].publish(Float64MultiArray(
                        data=[BASE[n] + h * STEP for n in orders[limb]]))
                next_hand += 1.0 / 30.0
            time.sleep(0.0005)
        return k


def limb_source(msg, limb):
    """
    Classify one limb's columns of a /joint_actions sample.

    Returns 'js' for the /joint_states fill, the tick number for one consistent
    command, or None for neither (a mapping error).
    """
    pos = dict(zip(msg.name, msg.position))
    joints = LIMB_JOINTS[limb]
    if all(abs(pos[n] - JS[n]) < 1e-12 for n in joints):
        return 'js'
    ticks = {round((pos[n] - BASE[n]) / STEP) for n in joints}
    if len(ticks) == 1:
        k = ticks.pop()
        if all(abs(pos[n] - BASE[n] - k * STEP) < 1e-9 for n in joints):
            return k
    return None


def sources(msgs):
    return [{lb: limb_source(m[2], lb) for lb in LIMB_JOINTS} for m in msgs]


def is_cmd(value):
    return isinstance(value, int)


def rate_ok(msgs, seconds):
    """Within 10 % of the 50 Hz publish rate."""
    return abs(len(msgs) - RATE * seconds) <= 0.1 * RATE * seconds


def run_aggregator(log_dir, name, args=()):
    log = os.path.join(log_dir, f'aggregator_{name}.log')
    return start([sys.executable, AGGREGATOR, *args], log), log


def wait_slot_orders(log, count, timeout=15.0):
    return wait_for(lambda: read(log).count('slot order (from') >= count, timeout)


def run_a(h, rep, log_dir):
    rep.section('A: both arms, permuted `joints`')
    h.set_controllers(PERMUTED)
    proc, log = run_aggregator(log_dir, 'A')
    try:
        ok = wait_slot_orders(log, 4)
        text = read(log)
        rep.check(ok and all(str(PERMUTED[k]) in text for k in PERMUTED), 'R1',
                  'slot order of all 4 controllers read from their (permuted) `joints`')
        h.wait_subscribed(LIMB_JOINTS)

        t = time.monotonic()
        time.sleep(2.0)
        msgs = h.between(t + 0.5, t + 2.0)
        src = sources(msgs)
        rep.check(rate_ok(msgs, 1.5) and all(v == 'js' for s in src for v in s.values()),
                  'R2', f'no command: {len(msgs) / 1.5:.1f} Hz, every column /joint_states')

        t = time.monotonic()
        h.feed(1.5, hands=('left_hand',))
        src = sources(h.between(t + 0.3, t + 1.5))
        rep.check(src and all(is_cmd(s['left_hand']) and s['left_arm'] == 'js'
                              and s['right_arm'] == 'js' and s['right_hand'] == 'js'
                              for s in src), 'R3',
                  'left hand commands only: left hand = command, the rest /joint_states')

        t = time.monotonic()
        h.feed(3.0, arms=ARMS, hands=('left_hand',))
        msgs = h.between(t + 0.5, t + 3.0)
        src = sources(msgs)
        names_ok = all(list(m[2].name) == ALL_JOINT_NAMES for m in msgs)
        unmapped = sum(any(v is None for v in s.values()) for s in src)
        cmds = all(is_cmd(s['left_arm']) and is_cmd(s['right_arm'])
                   and is_cmd(s['left_hand']) and s['right_hand'] == 'js' for s in src)
        rep.check(msgs and names_ok and not unmapped and cmds and rate_ok(msgs, 2.5), 'R4',
                  f'{len(msgs) / 2.5:.1f} Hz, 26 names in order, every value mapped by '
                  f'name (unmapped {unmapped}); arms + left hand = commands, never-commanded '
                  f'right hand = /joint_states')
        ages = sorted(m[1] - (m[2].header.stamp.sec + m[2].header.stamp.nanosec * 1e-9)
                      for m in msgs)
        p95 = ages[int(0.95 * (len(ages) - 1))] if ages else float('inf')
        rep.check(ages and ages[0] >= 0.0 and p95 < 0.03, 'R5',
                  f'header.stamp = publish instant (receive - stamp: p95 {p95 * 1e3:.1f} ms)')
        rep.check(h.node.count_publishers('/joint_actions') == 1, 'R6',
                  f'{h.node.count_publishers("/joint_actions")} publisher(s) on /joint_actions')

        t = time.monotonic()
        h.feed(1.6, hands=('left_hand',))
        msgs = h.between(t + 0.3, t + 1.6)
        src = sources(msgs)
        stopped = rate_ok(msgs, 1.3) and all(
            s['left_arm'] == 'js' and s['right_arm'] == 'js' and is_cmd(s['left_hand'])
            for s in src)
        t = time.monotonic()
        h.feed(1.0, arms=ARMS, hands=('left_hand',))
        back = all(is_cmd(s['left_arm']) and is_cmd(s['right_arm'])
                   for s in sources(h.between(t + 0.2, t + 1.0)))
        text = read(log)
        rep.check(stopped and back and 'left_arm: no command for' in text
                  and 'left_arm: commands arriving' in text, 'R7',
                  'arms stopped 1.6 s: still 50 Hz, arm columns /joint_states (logged); '
                  'arms back: commands again (logged)')

        t = time.monotonic()
        h.feed(1.6, arms=ARMS)
        src = sources(h.between(t + 0.3, t + 1.6))
        rep.check(src and all(s['left_hand'] == 'js' and is_cmd(s['left_arm'])
                              and is_cmd(s['right_arm']) for s in src), 'R8',
                  'left hand stopped: its columns /joint_states, arms still commands')

        t = time.monotonic()
        h.feed(1.0, arms=ARMS, hands=('left_hand',), bad_left_arm=True)
        src = sources(h.between(t + 0.3, t + 1.0))
        rep.check(src and all(s['left_arm'] == 'js' and is_cmd(s['right_arm']) for s in src)
                  and 'left_arm: command has 6 values, expected 7' in read(log), 'R9',
                  '6-value left arm command rejected and logged; left arm /joint_states, '
                  'right arm still commands')

        t = time.monotonic()
        h.feed(3.0, arms=ARMS, hands=('left_hand',), arm_hz=20.0)
        msgs = h.between(t + 1.0, t + 3.0)
        src = sources(msgs)
        rep.check(rate_ok(msgs, 2.0) and src
                  and all(is_cmd(s['left_arm']) and is_cmd(s['right_arm']) for s in src),
                  'R10', f'20 Hz arm commands: output {len(msgs) / 2.0:.1f} Hz, arms stay '
                         f'commands between their updates')
    finally:
        stop(proc)


def run_b(h, rep, log_dir):
    rep.section('B: single arm, /joint_states incomplete at first')
    h.set_controllers(PERMUTED)
    h.js_names = [n for n in ALL_JOINT_NAMES if n not in LIMB_JOINTS['left_arm']]
    proc, log = run_aggregator(log_dir, 'B')
    try:
        wait_slot_orders(log, 4)
        h.wait_subscribed(LIMB_JOINTS)
        t = time.monotonic()
        h.feed(1.5, arms=('right_arm',))
        text = read(log)
        rep.check(not h.between(t) and 'left_arm: no live command and' in text
                  and 'skipping this sample' in text, 'R11',
                  'idle left arm absent from /joint_states: ticks skipped, nothing made up')
        h.js_names = list(ALL_JOINT_NAMES)
        time.sleep(0.2)
        t = time.monotonic()
        h.feed(2.0, arms=('right_arm',))
        msgs = h.between(t + 0.3, t + 2.0)
        src = sources(msgs)
        rep.check(rate_ok(msgs, 1.7) and src
                  and all(is_cmd(s['right_arm']) and s['left_arm'] == 'js'
                          and s['left_hand'] == 'js' and s['right_hand'] == 'js'
                          for s in src), 'R12',
                  'right arm alone: right arm = command, left arm and hands /joint_states')
    finally:
        h.js_names = list(ALL_JOINT_NAMES)
        stop(proc)


def run_c(h, rep, log_dir):
    rep.section('C: a controller not loaded, another with the wrong `joints`')
    wrong = LIMB_JOINTS['right_arm'][:6] + ['waist_actr_left']
    h.set_controllers(dict(PERMUTED, left_arm=None, right_arm=wrong))
    proc, log = run_aggregator(log_dir, 'C')
    try:
        wait_slot_orders(log, 2)
        h.wait_subscribed(LIMB_JOINTS)
        time.sleep(1.5)
        t = time.monotonic()
        h.feed(2.0, arms=ARMS, hands=('left_hand',))
        src = sources(h.between(t + 0.3, t + 2.0))
        text = read(log)
        rep.check(src and all(s['left_arm'] == 'js' and s['right_arm'] == 'js'
                              and is_cmd(s['left_hand']) for s in src)
                  and '/right_forward_position_controller `joints` = ' in text
                  and 'left_arm: slot order unknown' in text
                  and 'slot order not known yet for left_arm' in text, 'R13a',
                  'unloaded / mismatched controllers: commands rejected (logged), '
                  'both arms /joint_states')
        h.set_controllers(PERMUTED)          # "loaded later", with the right `joints`
        ok = wait_slot_orders(log, 4, timeout=10.0)
        t = time.monotonic()
        h.feed(1.0, arms=ARMS, hands=('left_hand',))
        src = sources(h.between(t + 0.3, t + 1.0))
        rep.check(ok and src and all(is_cmd(s['left_arm']) and is_cmd(s['right_arm'])
                                     for s in src), 'R13b',
                  'recovers once the controllers report the right `joints`')
    finally:
        stop(proc)


def run_d(h, rep, log_dir):
    rep.section('D: --arm_config right --hand_config none')
    h.set_controllers(PERMUTED)
    proc, log = run_aggregator(log_dir, 'D', ['--arm_config', 'right', '--hand_config', 'none'])
    try:
        wait_slot_orders(log, 1)
        h.wait_subscribed(['right_arm'])
        t = time.monotonic()
        h.feed(2.0, arms=ARMS, hands=('left_hand', 'right_hand'))
        msgs = h.between(t + 0.3, t + 2.0)
        src = sources(msgs)
        rep.check(rate_ok(msgs, 1.7) and src
                  and all(is_cmd(s['right_arm']) and s['left_arm'] == 'js'
                          and s['left_hand'] == 'js' and s['right_hand'] == 'js'
                          for s in src), 'R14',
                  'only the right arm recorded; everything else /joint_states, 26 joints')
    finally:
        stop(proc)


def main():
    log_dir = tempfile.mkdtemp(prefix='joint_actions_rules_')
    print(f'domain {DOMAIN}, logs in {log_dir}', flush=True)
    rclpy.init()
    h = Harness()
    rep = Report()
    try:
        for run in (run_a, run_b, run_c, run_d):
            run(h, rep, log_dir)
    finally:
        h.executor.shutdown()
        rclpy.shutdown()
    print(f'\n{"ALL PASS" if not rep.failed else f"{rep.failed} FAILED"}  (logs: {log_dir})')
    return 1 if rep.failed else 0


if __name__ == '__main__':
    sys.exit(main())
