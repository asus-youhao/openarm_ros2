"""
Shared helpers for the /joint_actions checks (check_rules.py, check_bringup_fake.py).

Every check runs on its own ROS_DOMAIN_ID that no running process uses (read from
/proc/*/environ, the same rule as openarm_description/test/fake_smoke.py), with
discovery limited to localhost, so a robot or fake someone else is using is never
touched. No ros2 CLI is used, so no daemon is left behind.
"""

import glob
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
AGGREGATOR = os.path.join(os.path.dirname(HERE), 'joint_actions_aggregator.py')
sys.path.insert(0, os.path.dirname(HERE))

from joint_actions_aggregator import (  # noqa: E402
    ALL_JOINT_NAMES, LEFT_ARM_JOINTS, O6_LEFT_JOINTS, O6_RIGHT_JOINTS, RIGHT_ARM_JOINTS)

LIMB_JOINTS = {
    'left_arm': LEFT_ARM_JOINTS,
    'right_arm': RIGHT_ARM_JOINTS,
    'left_hand': O6_LEFT_JOINTS,
    'right_hand': O6_RIGHT_JOINTS,
}
CONTROLLERS = {
    'left_arm': 'left_forward_position_controller',
    'right_arm': 'right_forward_position_controller',
    'left_hand': 'left_hand_forward_position_controller',
    'right_hand': 'right_hand_forward_position_controller',
}
__all__ = ['ALL_JOINT_NAMES', 'LIMB_JOINTS', 'CONTROLLERS', 'AGGREGATOR']


def domains_in_use():
    """Return every ROS_DOMAIN_ID set in the environment of a running process."""
    used = set()
    for path in glob.glob('/proc/[0-9]*/environ'):
        try:
            with open(path, 'rb') as f:
                for item in f.read().split(b'\0'):
                    if item.startswith(b'ROS_DOMAIN_ID='):
                        used.add(item.split(b'=', 1)[1].decode(errors='replace'))
        except OSError:
            continue
    return used


def pick_domains(count, start=150):
    used = domains_in_use()
    picked = [d for d in range(start, 232) if str(d) not in used][:count]
    if len(picked) < count:
        raise RuntimeError('no free ROS_DOMAIN_ID')
    return picked


def isolate(domain_id):
    """Point this process (and children started afterwards) at a private domain."""
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_LOCALHOST_ONLY', None)


def start(cmd, log_path, shell=False):
    with open(log_path, 'w') as log:
        return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, shell=shell,
                                env=dict(os.environ), start_new_session=True)


def stop(proc):
    """SIGINT the process group; escalate until the whole group is gone."""
    for sig, wait in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            proc.poll()
            try:
                os.killpg(proc.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.2)


def read(path):
    with open(path, errors='replace') as f:
        return f.read()


def wait_for(predicate, timeout, step=0.1):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(step)
    return predicate()


class Report:
    """Collects PASS / FAIL lines; exit status is non-zero on any FAIL."""

    def __init__(self):
        self.failed = 0

    def check(self, ok, check_id, text):
        print(f'  {"PASS" if ok else "FAIL"} {check_id} {text}', flush=True)
        if not ok:
            self.failed += 1
        return ok

    def section(self, title):
        print(f'\n[{title}]', flush=True)
