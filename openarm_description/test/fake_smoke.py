#!/usr/bin/env python3
"""
Launch openarm_bringup with use_fake_hardware:=true and check it works.

Called by run_full_test.sh with the test workspace sourced. Per case:
  S1  the launch comes up: every required controller reaches 'active'
  S2  /joint_states carries exactly the ros2_control joints of the published
      /robot_description, at no less than the expected rate
  S3  a trajectory sent to each JointTrajectoryController is followed: after
      it finishes every commanded joint sits at its target
  S4  every URDF <mimic> joint on /joint_states equals
      multiplier * parent + offset (checked after S3, so parents are not 0)
  H1  (real-hardware parse probe) with use_fake_hardware:=false the URDF gets
      through ros2_control parsing and the launch stops only because the
      hardware plugin is not installed. Runs only when openarm_hardware is
      NOT installed, so it can never start real hardware; otherwise SKIP.

Each case runs on its own ROS_DOMAIN_ID with discovery limited to localhost,
under 'ulimit -l 64': Jazzy's controller_manager defaults to lock_memory:=true,
and on machines with a large memlock limit the mlockall succeeds and later
allocations fail with EAGAIN (docs/README.md, X2). With a small
limit it only logs a warning. Everything is driven in-process with rclpy, so
no ros2 CLI daemon is left behind. Launch output goes to --log-dir.
"""

import argparse
import os
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

CASES = [
    {
        'name': 'bimanual_default',
        'launch': 'openarm.bimanual.launch.py',
        # no ee_type on purpose: the launch default must render (docs/README.md, X1)
        'args': ['use_fake_hardware:=true', 'launch_rviz:=false'],
        'controllers': ['joint_state_broadcaster',
                        'left_joint_trajectory_controller',
                        'right_joint_trajectory_controller'],
        # needs apt ros-<distro>-gripper-controllers (docs/README.md, X3)
        'optional_controllers': {'gripper_controllers': ['left_gripper_controller',
                                                         'right_gripper_controller']},
        'min_rate': 80.0,  # controller_manager update_rate 100 Hz
        'trajectory_controllers': ['left_joint_trajectory_controller',
                                   'right_joint_trajectory_controller'],
        'mimic_joints': 0,
    },
    {
        'name': 'bimanual_o6',
        'launch': 'openarm_o6_bimanual.launch.py',
        'args': ['use_fake_hardware:=true', 'launch_rviz:=false'],
        'controllers': ['joint_state_broadcaster',
                        'left_joint_trajectory_controller',
                        'right_joint_trajectory_controller',
                        'left_hand_controller',
                        'right_hand_controller'],
        'min_rate': 400.0,  # controller_manager update_rate 500 Hz
        'trajectory_controllers': ['left_joint_trajectory_controller',
                                   'right_joint_trajectory_controller',
                                   'left_hand_controller',
                                   'right_hand_controller'],
        'mimic_joints': 10,  # five *_dip joints per hand
    },
    {
        'name': 'real_o6_parse_probe',
        'kind': 'real_parse',
        'launch': 'openarm_o6_bimanual.launch.py',
        'args': ['use_fake_hardware:=false', 'launch_rviz:=false'],
    },
]

HW_PACKAGE = 'openarm_hardware'
HW_PLUGIN = 'openarm_hardware/OpenArm_v10HW'
MIMIC_ERROR = 'Activated mimic joints cannot have command interfaces'


FATAL_PATTERNS = ['terminate called', 'LibraryLoadException', 'Caught exception']
POS_TOL = 1e-3


class CaseFailed(Exception):
    pass


def first_fatal_line(log_path):
    with open(log_path, errors='replace') as f:
        for line in f:
            if any(p in line for p in FATAL_PATTERNS):
                return line.strip()[:300]
    return None


class Probe:
    """rclpy helpers bound to one domain."""

    def __init__(self, domain_id):
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx, domain_id=domain_id)
        self.node = rclpy.create_node('fake_smoke_probe', context=self.ctx)
        self.executor = SingleThreadedExecutor(context=self.ctx)
        self.executor.add_node(self.node)
        self.joint_msgs = []
        self.node.create_subscription(JointState, '/joint_states', self.joint_msgs.append, 50)
        self.robot_description = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, '/robot_description', self._on_rd, latched)
        self.pubs = {}

    def _on_rd(self, msg):
        self.robot_description = msg.data

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.05)

    def call(self, srv_type, name, request, timeout=5.0):
        cli = self.node.create_client(srv_type, name)
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                return None
            fut = cli.call_async(request)
            end = time.monotonic() + timeout
            while not fut.done() and time.monotonic() < end:
                self.executor.spin_once(timeout_sec=0.05)
            return fut.result() if fut.done() else None
        finally:
            self.node.destroy_client(cli)

    def controller_states(self):
        res = self.call(ListControllers, '/controller_manager/list_controllers',
                        ListControllers.Request(), timeout=2.0)
        return {c.name: c.state for c in res.controller} if res else {}

    def controller_joints(self, controller):
        req = GetParameters.Request(names=['joints'])
        res = self.call(GetParameters, f'/{controller}/get_parameters', req)
        if not res or not res.values:
            raise CaseFailed(f'cannot read {controller}.joints')
        return list(res.values[0].string_array_value)

    def latest_positions(self):
        self.joint_msgs.clear()
        self.spin_for(0.3)
        if not self.joint_msgs:
            raise CaseFailed('no /joint_states')
        m = self.joint_msgs[-1]
        return dict(zip(m.name, m.position))

    def send_trajectory(self, controller, joints, positions, seconds):
        topic = f'/{controller}/joint_trajectory'
        if topic not in self.pubs:
            self.pubs[topic] = self.node.create_publisher(JointTrajectory, topic, 10)
            self.spin_for(0.5)  # let discovery match the subscription
        msg = JointTrajectory(joint_names=joints)
        msg.points = [JointTrajectoryPoint(positions=positions)]
        msg.points[0].time_from_start.sec = int(seconds)
        self.pubs[topic].publish(msg)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.ctx)


def parse_description(xml_text):
    root = ET.fromstring(xml_text)
    rc_joints = [j.get('name') for rc in root.findall('ros2_control') for j in rc.findall('joint')]
    limits = {}
    for j in root.findall('joint'):
        lim = j.find('limit')
        if lim is not None and lim.get('lower') is not None:
            limits[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
    return root, rc_joints, limits


def target_for(joint, index, limits):
    """Pick a target away from 0 and inside the URDF limits."""
    want = 0.1 + 0.05 * index
    lo, hi = limits.get(joint, (-3.0, 3.0))
    margin = 0.1 * (hi - lo)
    return min(max(want, lo + margin), hi - margin)


def check_case(case, probe, results):
    # S1: required controllers active
    required = list(case['controllers'])
    for pkg, names in case.get('optional_controllers', {}).items():
        try:
            get_package_share_directory(pkg)
            required += names
        except PackageNotFoundError:
            results.append(('SKIP', 'S1', f'{names}: package {pkg} not installed'))
    deadline = time.monotonic() + 45.0
    states = {}
    while time.monotonic() < deadline:
        states = probe.controller_states()
        if all(states.get(c) == 'active' for c in required):
            break
        if case['_launch'].poll() is not None:
            break
        if first_fatal_line(case['_log']):
            probe.spin_for(1.0)
            states = probe.controller_states()
            break
        probe.spin_for(1.0)
    not_active = {c: states.get(c, 'missing') for c in required if states.get(c) != 'active'}
    if not_active:
        why = first_fatal_line(case['_log']) or 'timeout'
        raise CaseFailed(f'S1 controllers not active {not_active}; {why}')
    results.append(('PASS', 'S1', f'{len(required)} controllers active'))

    # S2: joint set and rate
    end = time.monotonic() + 10.0
    while probe.robot_description is None and time.monotonic() < end:
        probe.spin_for(0.2)
    if probe.robot_description is None:
        raise CaseFailed('S2 no /robot_description')
    root, rc_joints, limits = parse_description(probe.robot_description)
    probe.joint_msgs.clear()
    t0 = time.monotonic()
    probe.spin_for(3.0)
    rate = len(probe.joint_msgs) / (time.monotonic() - t0)
    names = set(probe.joint_msgs[-1].name) if probe.joint_msgs else set()
    if names != set(rc_joints):
        raise CaseFailed(f'S2 /joint_states joints {sorted(names ^ set(rc_joints))} '
                         'differ from ros2_control joints')
    if rate < case['min_rate']:
        raise CaseFailed(f'S2 /joint_states {rate:.1f} Hz < {case["min_rate"]} Hz')
    results.append(('PASS', 'S2', f'{len(names)} joints at {rate:.0f} Hz'))

    # S3: trajectories are followed
    targets = {}
    for ctrl in case['trajectory_controllers']:
        joints = probe.controller_joints(ctrl)
        pos = [target_for(j, i, limits) for i, j in enumerate(joints)]
        probe.send_trajectory(ctrl, joints, pos, seconds=1)
        targets.update(zip(joints, pos))
    probe.spin_for(2.5)
    actual = probe.latest_positions()
    off = {j: round(actual.get(j, float('nan')) - t, 4) for j, t in targets.items()
           if not abs(actual.get(j, float('nan')) - t) < POS_TOL}
    if off:
        raise CaseFailed(f'S3 joints not at target (error): {off}')
    results.append(('PASS', 'S3', f'{len(targets)} joints reached target'))

    # S4: mimic joints follow their parent as the URDF says
    checked, wrong = 0, {}
    for j in root.findall('joint'):
        m = j.find('mimic')
        if m is None or j.get('name') not in actual:
            continue
        want = (float(m.get('multiplier', 1.0)) * actual[m.get('joint')]
                + float(m.get('offset', 0.0)))
        checked += 1
        if not abs(actual[j.get('name')] - want) < POS_TOL:
            wrong[j.get('name')] = (round(actual[j.get('name')], 4), round(want, 4))
    if wrong:
        raise CaseFailed(f'S4 mimic joints (actual, expected): {wrong}')
    if checked != case['mimic_joints']:
        raise CaseFailed(f'S4 {checked} mimic joints on /joint_states, '
                         f'expected {case["mimic_joints"]}')
    if checked:
        results.append(('PASS', 'S4', f'{checked} mimic joints follow their parent'))


def start_launch(case, domain_id, log_path):
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain_id), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    cmd = 'ulimit -l 64; exec ros2 launch openarm_bringup {} {}'.format(
        case['launch'], ' '.join(case['args']))
    with open(log_path, 'w') as log:
        return subprocess.Popen(['bash', '-c', cmd], stdout=log, stderr=subprocess.STDOUT,
                                env=env, start_new_session=True)


def run_parse_probe(case, domain_id, log_path):
    """H1: return (verdict, text)."""
    try:
        get_package_share_directory(HW_PACKAGE)
        return 'SKIP', f'{HW_PACKAGE} is installed here; refusing to start real hardware'
    except PackageNotFoundError:
        pass
    proc = start_launch(case, domain_id, log_path)
    try:
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            with open(log_path, errors='replace') as f:
                text = f.read()
            if MIMIC_ERROR in text:
                return 'FAIL', 'ros2_control rejected the URDF: ' + MIMIC_ERROR
            if 'LibraryLoadException' in text and HW_PLUGIN in text:
                return 'PASS', f'URDF parsed; stopped at loading {HW_PLUGIN} (not installed)'
            if proc.poll() is not None:
                break
            time.sleep(0.5)
        return 'FAIL', first_fatal_line(log_path) or 'neither parse error nor plugin load seen'
    finally:
        stop(proc)


def run_case(case, domain_id, log_dir):
    log_path = os.path.join(log_dir, f'fake_smoke_{case["name"]}.log')
    if case.get('kind') == 'real_parse':
        verdict, text = run_parse_probe(case, domain_id, log_path)
        print(f'[{case["name"]}] {case["launch"]} {" ".join(case["args"])}  (log: {log_path})')
        print(f'  {verdict} H1 {text}')
        return verdict != 'FAIL'
    results = []
    proc = start_launch(case, domain_id, log_path)
    case['_launch'], case['_log'] = proc, log_path
    probe = Probe(domain_id)
    ok = True
    try:
        check_case(case, probe, results)
    except CaseFailed as e:
        results.append(('FAIL', '', str(e)))
        ok = False
    finally:
        probe.close()
        stop(proc)
    print(f'[{case["name"]}] {case["launch"]} {" ".join(case["args"])}  (log: {log_path})')
    for verdict, check_id, text in results:
        print(f'  {verdict} {check_id} {text}' if check_id else f'  {verdict} {text}')
    return ok


def stop(proc):
    """SIGINT the launch's process group; escalate until the whole group is gone."""
    for sig, wait in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            proc.poll()  # reap the group leader so a zombie does not keep the group alive
            try:
                os.killpg(proc.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.2)


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--log-dir', required=True)
    ap.add_argument('--case', action='append', help='run only these cases')
    a = ap.parse_args()
    os.makedirs(a.log_dir, exist_ok=True)
    base_domain = 60 + os.getpid() % 30
    ok = True
    for i, case in enumerate(CASES):
        if a.case and case['name'] not in a.case:
            continue
        ok &= run_case(case, base_domain + i, a.log_dir)
    print('fake_smoke: ' + ('all cases passed' if ok else 'FAILED'))
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
