#!/usr/bin/env python3
"""
Check the publishing rules of joint_actions_aggregator.py without ros2_control.

The aggregator runs as its own process against stand-in controller nodes (a node
with the controller's name and a `joints` parameter, deliberately PERMUTED) and
commands published from here. No hardware, no controller_manager.

  R1  slot order is read from each controller's `joints`, not assumed
  R2  no arm ever commanded -> nothing published, hand commands do not trigger (④)
  R3  26 names in the fixed order; values mapped by name through the permuted
      `joints`; both arms from the same tick; one message per arm tick
  R4  a limb that never sent a command is filled from /joint_states (②)
  R5  header.stamp is the arrival time of the oldest contributing command
  R6  exactly one publisher on /joint_actions
  R7  arms go quiet -> publishing stops, 'stale' logged; back -> 'resumed' (③)
  R8  a latched hand goes quiet -> publishing stops too (③)
  R9  wrong-length command rejected and logged
  R10 inactive limb missing from /joint_states -> sample withheld, never 0.0
  R11 single-arm teleop with arm_config both: the idle arm is filled
  R12 controller not loaded / wrong `joints` -> rejected; recovers once fixed
  R13 --arm_config right --hand_config none: only the right arm is recorded
  R14 arms on independent schedules still pair their closest commands, at arm rate

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

PERMUTED = {
    'left_arm': list(reversed(LIMB_JOINTS['left_arm'])),
    'right_arm': LIMB_JOINTS['right_arm'][3:] + LIMB_JOINTS['right_arm'][:3],
    'left_hand': ['L_thumb_cmc_yaw', 'L_index_mcp_pitch', 'L_ring_mcp_pitch',
                  'L_thumb_cmc_pitch', 'L_middle_mcp_pitch', 'L_pinky_mcp_pitch'],
    'right_hand': list(LIMB_JOINTS['right_hand']),
}


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
        if orders == getattr(self, 'orders', None):
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

    def since(self, t):
        with self.lock:
            return [m for m in self.msgs if m[0] >= t]

    def wait_subscribed(self, limbs, timeout=10.0):
        """Wait until the aggregator has discovered our command publishers."""
        return wait_for(lambda: all(self.pubs[lb].get_subscription_count() >= 1
                                    for lb in limbs), timeout)

    def feed(self, duration, arms=(), hands=(), orders=PERMUTED, bad_left_arm=False,
             phase=None):
        """
        Publish commands in each controller's slot order; return arm ticks sent.

        Arms due at the same instant are published back to back in `arms` order, like
        one IK step. `phase` (limb -> seconds) shifts an arm onto its own schedule.
        """
        t0 = time.monotonic()
        phase = phase or {}
        next_arm = {limb: t0 + phase.get(limb, 0.0) for limb in arms}
        tick = {limb: 0 for limb in arms}
        next_hand = t0
        h = 0
        while time.monotonic() < t0 + duration:
            now = time.monotonic()
            for limb in arms:
                if now >= next_arm[limb]:
                    tick[limb] += 1
                    data = [BASE[n] + tick[limb] * STEP for n in orders[limb]]
                    if bad_left_arm and limb == 'left_arm':
                        data = data[:-1]
                    self.pubs[limb].publish(Float64MultiArray(data=data))
                    next_arm[limb] += 0.02
            if hands and now >= next_hand:
                h += 1
                for limb in hands:
                    self.pubs[limb].publish(Float64MultiArray(
                        data=[BASE[n] + h * STEP for n in orders[limb]]))
                next_hand += 1.0 / 30.0
            time.sleep(0.0005)
        return max(tick.values()) if tick else 0


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
        h.feed(1.5, hands=('left_hand',))
        rep.check(not h.since(t) and 'waiting for the first arm command' in read(log), 'R2',
                  'hand commands only (no arm yet): nothing published')

        t = time.monotonic()
        ticks = h.feed(3.0, arms=('left_arm', 'right_arm'), hands=('left_hand',))
        time.sleep(0.2)
        msgs = h.since(t)
        names_ok = all(list(m[2].name) == ALL_JOINT_NAMES for m in msgs)
        src = [{lb: limb_source(m[2], lb) for lb in LIMB_JOINTS} for m in msgs]
        unmapped = sum(any(s[lb] is None for lb in LIMB_JOINTS) for s in src)
        # The very first sample may carry the left arm alone: the right arm has not
        # latched yet when the first left command arrives (single-arm semantics).
        body = src[1:]
        split = sum(isinstance(s['left_arm'], int) and s['left_arm'] != s['right_arm']
                    for s in body)
        not_cmd = sum(not isinstance(s[lb], int)
                      for s in body for lb in ('left_arm', 'right_arm', 'left_hand'))
        rep.check(msgs and names_ok, 'R3a', f'{len(msgs)} messages, 26 names in fixed order')
        # Scheduling jitter in this harness can tip a rare sample; the defect this
        # guards against (pairing locked one step off) splits every sample.
        rep.check(msgs and not unmapped and split <= 0.02 * len(msgs) and not not_cmd, 'R3b',
                  'every value mapped by name through the permuted `joints`; both arms '
                  f'from the same IK step (unmapped {unmapped}, split {split}/{len(msgs)}, '
                  f'not a command {not_cmd})')
        rep.check(ticks - 3 <= len(msgs) <= ticks, 'R3c',
                  f'one message per arm tick ({len(msgs)} messages / {ticks} ticks, '
                  f'{len(msgs) / 3.0:.1f} Hz); hand commands do not add messages')
        rep.check(msgs and all(s['right_hand'] == 'js' for s in src), 'R4',
                  'never-commanded right hand filled from /joint_states')
        ages = [m[1] - (m[2].header.stamp.sec + m[2].header.stamp.nanosec * 1e-9)
                for m in msgs]
        ages.sort()
        p95 = ages[int(0.95 * (len(ages) - 1))] if ages else float('inf')
        rep.check(msgs and 0.0 <= ages[0] and p95 < 0.1, 'R5',
                  f'header.stamp = oldest command arrival, never ahead of arrival '
                  f'(age min {ages[0] * 1e3:.1f} ms, p95 {p95 * 1e3:.1f} ms)'
                  if msgs else 'no messages')
        rep.check(h.node.count_publishers('/joint_actions') == 1, 'R6',
                  f'{h.node.count_publishers("/joint_actions")} publisher(s) on /joint_actions')

        t_stop = time.monotonic()
        h.feed(1.6, hands=('left_hand',))
        quiet = not h.since(t_stop + 0.1)
        t = time.monotonic()
        h.feed(1.0, arms=('left_arm', 'right_arm'), hands=('left_hand',))
        text = read(log)
        rep.check(quiet and h.since(t) and 'stale: left_arm=' in text
                  and 'resumed /joint_actions after' in text and "'left_arm'" in text, 'R7',
                  'arms quiet 1.6 s: nothing published, stale logged; resumed afterwards')

        before = read(log).count('resumed /joint_actions after')
        t_stop = time.monotonic()
        h.feed(1.6, arms=('left_arm', 'right_arm'))
        quiet = not h.since(t_stop + 0.3)
        t = time.monotonic()
        h.feed(1.0, arms=('left_arm', 'right_arm'), hands=('left_hand',))
        text = read(log)
        rep.check(quiet and h.since(t) and 'stale: left_hand=' in text
                  and text.count('resumed /joint_actions after') > before, 'R8',
                  'latched left hand quiet 1.6 s: publishing paused, resumed afterwards')

        t = time.monotonic()
        h.feed(1.0, arms=('left_arm', 'right_arm'), hands=('left_hand',), bad_left_arm=True)
        bad_quiet = not h.since(t + 0.2)
        t = time.monotonic()
        h.feed(0.6, arms=('left_arm', 'right_arm'), hands=('left_hand',))
        rep.check(bad_quiet and h.since(t)
                  and 'left_arm: command has 6 values, expected 7' in read(log), 'R9',
                  '6-value left arm command rejected and logged; recovers with 7 values')

        # Arms on independent threads: there is no common step, so a sample pairs each
        # arm with the closest command of the other; scheduling jitter can tip a few
        # samples to the next-closest one. Required: no stall (arm rate), never more
        # than one step apart, and the closest pairing for the large majority.
        for lag_ms, want in ((6, 0), (13, 1)):
            # Judge the steady state: the first second follows a phase change and a
            # pause, while the period estimate (and so the pairing window) settles.
            t = time.monotonic() + 1.0
            ticks = h.feed(4.0, arms=('left_arm', 'right_arm'), hands=('left_hand',),
                           phase={'right_arm': lag_ms / 1000.0}) - 50
            time.sleep(0.2)
            src = [{lb: limb_source(m[2], lb) for lb in LIMB_JOINTS} for m in h.since(t)]
            diffs = [s['left_arm'] - s['right_arm'] for s in src
                     if isinstance(s['left_arm'], int) and isinstance(s['right_arm'], int)]
            closest = sum(d == want for d in diffs) / len(diffs) if diffs else 0.0
            rep.check(ticks - 4 <= len(src) <= ticks + 1 and diffs
                      and all(abs(d - want) <= 1 for d in diffs) and closest >= 0.9,
                      f'R14{"ab"[want]}',
                      f'right arm {lag_ms} ms behind on its own schedule: {len(src)} msgs / '
                      f'{ticks} ticks, left - right step in {sorted(set(diffs))}, '
                      f'{closest:.0%} closest pairing (expected step {want})')
    finally:
        stop(proc)


def run_b(h, rep, log_dir):
    rep.section('B: single-arm teleop, /joint_states incomplete at first')
    h.set_controllers(PERMUTED)
    h.js_names = [n for n in ALL_JOINT_NAMES if n not in LIMB_JOINTS['left_arm']]
    proc, log = run_aggregator(log_dir, 'B')
    try:
        wait_slot_orders(log, 4)
        h.wait_subscribed(LIMB_JOINTS)
        t = time.monotonic()
        h.feed(1.5, arms=('right_arm',))
        rep.check(not h.since(t) and 'left_arm: inactive and' in read(log)
                  and 'withholding /joint_actions' in read(log), 'R10',
                  'idle left arm absent from /joint_states: sample withheld, nothing published')
        h.js_names = list(ALL_JOINT_NAMES)
        time.sleep(0.2)
        t = time.monotonic()
        ticks = h.feed(2.0, arms=('right_arm',))
        time.sleep(0.2)
        msgs = h.since(t)
        src = [{lb: limb_source(m[2], lb) for lb in LIMB_JOINTS} for m in msgs]
        rep.check(ticks - 3 <= len(msgs) <= ticks
                  and all(isinstance(s['right_arm'], int) and s['left_arm'] == 'js'
                          and s['left_hand'] == 'js' and s['right_hand'] == 'js' for s in src),
                  'R11', f'right arm alone drives publishing ({len(msgs)}/{ticks} ticks); '
                  'left arm and both hands filled from /joint_states')
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
        h.feed(2.0, arms=('left_arm', 'right_arm'), hands=('left_hand',))
        text = read(log)
        rep.check(not h.since(t)
                  and '/right_forward_position_controller `joints` = ' in text
                  and 'left_arm: slot order unknown' in text
                  and 'slot order not known yet for left_arm' in text, 'R12a',
                  'commands for an unloaded / mismatched controller rejected, nothing '
                  'published, both logged')
        h.set_controllers(PERMUTED)          # "loaded later", with the right `joints`
        ok = wait_slot_orders(log, 4, timeout=10.0)
        t = time.monotonic()
        h.feed(1.0, arms=('left_arm', 'right_arm'), hands=('left_hand',))
        rep.check(ok and len(h.since(t)) > 30, 'R12b',
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
        ticks = h.feed(2.0, arms=('left_arm', 'right_arm'), hands=('left_hand', 'right_hand'))
        time.sleep(0.2)
        msgs = h.since(t)
        src = [{lb: limb_source(m[2], lb) for lb in LIMB_JOINTS} for m in msgs]
        rep.check(ticks - 3 <= len(msgs) <= ticks
                  and all(isinstance(s['right_arm'], int) and s['left_arm'] == 'js'
                          and s['left_hand'] == 'js' and s['right_hand'] == 'js' for s in src),
                  'R13', 'only the right arm recorded; everything else from /joint_states, '
                  'still 26 joints')
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
