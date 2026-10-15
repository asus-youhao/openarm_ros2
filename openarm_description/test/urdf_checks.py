#!/usr/bin/env python3
"""
Render v10.urdf.xacro across an argument matrix and check the output.

Called by run_full_test.sh. Checks per combination (IDs referenced from docs/):
  R1  xacro renders without error
  R2  check_urdf accepts the result
  R3  every joint listed in a <ros2_control> block exists in the URDF
  M1  every mesh the URDF references exists in the installed package
  F1  fake mode: every ros2_control plugin is mock_components/GenericSystem
  F2  fake mode: the sensor-command switch is passed as mock_sensor_commands
      with the value of the fake_sensor_commands xacro arg
  F3  Jazzy's URDF parser rule: a ros2_control joint that is a <mimic> joint in
      the URDF and not marked mimic="false" must not have command interfaces
  G1  the rendered URDF matches test/golden.json. The golden file stores a
      hash of the whole canonicalized URDF plus a one-line-per-joint summary
      of the ros2_control blocks, so an intended change to what the hardware
      sees shows up as a readable diff in the commit. Regenerate with
      --update-golden.

Known issues are reported as KNOWN and do not fail the run:
  K1  ros2_control declares openarm_*finger_joint1 although the URDF has no
      openarm_hand (hand:=true default with ee_type other than openarm_hand)
  K3  F3 in real-hardware mode (O6 dip joints keep command interfaces)
  K5  leap_hand_left uses meshes/leap_hand/visual/mcp_joint_left.obj, which
      does not exist anywhere
"""

import argparse
import difflib
import hashlib
import itertools
import json
import os
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

PKG = 'openarm_description'
XACRO_REL = f'share/{PKG}/urdf/robot/v10.urdf.xacro'
FAKE_PLUGIN = 'mock_components/GenericSystem'

# ee_type o6_left / o6_right are not in the matrix: they cannot render
# (urdf/ee/o6_left_arguments.xacro does not exist, K2 in docs/README.md).
EE_TYPES = ['openarm_hand', 'o6', 'leap_hand_right', 'leap_hand_left', 'none']

KNOWN_MISSING_MESHES = {
    'meshes/leap_hand/visual/mcp_joint_left.obj': 'K5',
}


def matrix():
    for bimanual, ee_type, fake in itertools.product(
            ['true', 'false'], EE_TYPES, ['true', 'false']):
        yield {
            'bimanual': bimanual,
            'ee_type': ee_type,
            'use_fake_hardware': fake,
            'ros2_control': 'true',
        }
    # F2 needs the non-default value as well
    for bimanual, ee_type in itertools.product(['true', 'false'], ['openarm_hand', 'o6']):
        yield {
            'bimanual': bimanual,
            'ee_type': ee_type,
            'use_fake_hardware': 'true',
            'fake_sensor_commands': 'true',
            'ros2_control': 'true',
        }


def label(args):
    return ' '.join(f'{k}:={v}' for k, v in args.items())


def share_dir(install):
    return os.path.join(install, PKG, 'share', PKG)


def render(install, args):
    """Return (urdf_xml or None, error_text)."""
    prefix = os.path.join(install, PKG)
    env = dict(os.environ)
    env['AMENT_PREFIX_PATH'] = prefix + ':' + env.get('AMENT_PREFIX_PATH', '')
    cmd = ['xacro', os.path.join(prefix, XACRO_REL)]
    cmd += [f'{k}:={v}' for k, v in args.items()]
    proc = subprocess.run(cmd, env=env, capture_output=True, text=True)
    if proc.returncode != 0:
        return None, proc.stderr.strip().splitlines()[-1:] or ['xacro failed']
    # make the output independent of where the package is installed
    return proc.stdout.replace(share_dir(install), f'$(find {PKG})'), None


def check_urdf(urdf_xml):
    with tempfile.NamedTemporaryFile('w', suffix='.urdf') as f:
        f.write(urdf_xml)
        f.flush()
        proc = subprocess.run(['check_urdf', f.name], capture_output=True, text=True)
    return proc.returncode == 0, proc.stdout + proc.stderr


def ros2_control_joints(root):
    for rc in root.findall('ros2_control'):
        for j in rc.findall('joint'):
            yield rc.get('name'), j


def hardware_params(hw):
    return {p.get('name'): (p.text or '').strip() for p in hw.findall('param')}


def mimic_violations(root):
    """List ros2_control joints breaking 'no command interface on an active mimic joint'."""
    mimic = {j.get('name') for j in root.findall('joint') if j.find('mimic') is not None}
    return sorted(j.get('name') for _, j in ros2_control_joints(root)
                  if j.get('name') in mimic and j.get('mimic', '').lower() != 'false'
                  and j.findall('command_interface'))


def missing_meshes(root, install):
    """Return [(filename, known_id or None)] for meshes that do not resolve."""
    out = []
    for mesh in root.iter('mesh'):
        name = mesh.get('filename', '')
        rel = re.sub(rf'^(package://{PKG}/|\$\(find {PKG}\)/)', '', name)
        if rel != name and os.path.exists(os.path.join(share_dir(install), rel)):
            continue
        known = next((k for pat, k in KNOWN_MISSING_MESHES.items() if pat in name), None)
        out.append((name, known))
    return sorted(set(out))


def ros2_control_summary(root):
    """One line per hardware param and joint, so golden diffs stay short and readable."""
    lines = []
    for rc in root.findall('ros2_control'):
        hw = rc.find('hardware')
        lines.append(f"ros2_control {rc.get('name')} plugin={hw.findtext('plugin', '').strip()}")
        lines += [f'  param {k}={v}' for k, v in sorted(hardware_params(hw).items())]
        for j in rc.findall('joint'):
            cmd = ','.join(c.get('name') for c in j.findall('command_interface'))
            state = ','.join(c.get('name') for c in j.findall('state_interface'))
            mimic = f" mimic={j.get('mimic')}" if j.get('mimic') is not None else ''
            lines.append(f"  joint {j.get('name')}{mimic} cmd={cmd or '-'} state={state or '-'}")
    return lines


def golden_entry(urdf_xml, root):
    canon = ET.canonicalize(urdf_xml, strip_text=True)
    return {'sha256': hashlib.sha256(canon.encode()).hexdigest(),
            'ros2_control': ros2_control_summary(root)}


def block_key(lines):
    return hashlib.sha256('\n'.join(lines).encode()).hexdigest()[:12]


def load_golden(path):
    with open(path) as f:
        g = json.load(f)
    return {case: {'sha256': c['sha256'], 'ros2_control': g['ros2_control'][c['ros2_control']]}
            for case, c in g['cases'].items()}


def save_golden(path, entries):
    blocks = {block_key(e['ros2_control']): e['ros2_control'] for e in entries.values()}
    cases = {case: {'sha256': e['sha256'], 'ros2_control': block_key(e['ros2_control'])}
             for case, e in entries.items()}
    with open(path, 'w') as f:
        json.dump({'cases': cases, 'ros2_control': blocks}, f, indent=1, sort_keys=True)
        f.write('\n')


class Report:

    def __init__(self):
        self.failures = []
        self.known = []
        self.passed = 0

    def check(self, ok, check_id, case, detail='', known=None):
        if ok:
            self.passed += 1
        elif known:
            self.known.append(f'{known} ({check_id}) [{case}] {detail}'.rstrip())
        else:
            self.failures.append(f'{check_id} [{case}] {detail}'.rstrip())


def compare_golden(rep, case, entry, want):
    if want is None:
        rep.check(False, 'G1', case, 'not in golden.json')
        return
    if entry['sha256'] == want['sha256']:
        rep.check(True, 'G1', case)
        return
    if entry['ros2_control'] == want['ros2_control']:
        rep.check(False, 'G1', case, 'URDF differs from golden (ros2_control summary unchanged)')
        return
    diff = difflib.unified_diff(want['ros2_control'], entry['ros2_control'],
                                'golden', 'rendered', n=1, lineterm='')
    rep.check(False, 'G1', case, 'ros2_control differs from golden:\n      '
              + '\n      '.join(list(diff)[:40]))


def run(install, golden_path, update):
    rep = Report()
    golden = {}
    if not update:
        golden = load_golden(golden_path)
    new_golden = {}
    for args in matrix():
        case = label(args)
        fake = args['use_fake_hardware'] == 'true'
        urdf_xml, err = render(install, args)
        rep.check(urdf_xml is not None, 'R1', case, ' '.join(err or []))
        if urdf_xml is None:
            continue

        ok, out = check_urdf(urdf_xml)
        rep.check(ok, 'R2', case, out.strip().splitlines()[-1] if not ok else '')

        root = ET.fromstring(urdf_xml)
        joints = {j.get('name') for j in root.findall('joint')}
        missing = [j.get('name') for _, j in ros2_control_joints(root)
                   if j.get('name') not in joints]
        k1 = (args['ee_type'] != 'openarm_hand'
              and all(n.endswith('finger_joint1') for n in missing))
        rep.check(not missing, 'R3', case, f'not in URDF: {missing}', known='K1' if k1 else None)

        for name, known in missing_meshes(root, install) or [(None, None)]:
            rep.check(name is None, 'M1', case, f'missing mesh {name}', known=known)

        if fake:
            hws = [rc.find('hardware') for rc in root.findall('ros2_control')]
            plugins = [hw.findtext('plugin', '').strip() for hw in hws]
            rep.check(plugins and all(p == FAKE_PLUGIN for p in plugins), 'F1', case,
                      f'plugins: {plugins}')
            want = args.get('fake_sensor_commands', 'false')
            bad = []
            for hw in hws:
                params = hardware_params(hw)
                if 'fake_sensor_commands' in params:
                    bad.append('fake_sensor_commands still present')
                # xacro renders the bool as True/False; parse_bool accepts either case
                if params.get('mock_sensor_commands', '').lower() != want:
                    bad.append(f"mock_sensor_commands={params.get('mock_sensor_commands')}")
            rep.check(not bad, 'F2', case, f'want mock_sensor_commands={want}: {bad}')

        bad = mimic_violations(root)
        rep.check(not bad, 'F3', case, f'mimic joints with command interfaces: {bad}',
                  known=None if fake else 'K3')

        entry = golden_entry(urdf_xml, root)
        new_golden[case] = entry
        if not update:
            compare_golden(rep, case, entry, golden.get(case))

    if update:
        save_golden(golden_path, new_golden)
        print(f'golden: wrote {len(new_golden)} cases to {golden_path}')
    print(f'urdf_checks: {rep.passed} passed, {len(rep.failures)} failed, '
          f'{len(rep.known)} known')
    for k in rep.known:
        print(f'  KNOWN {k}')
    for f in rep.failures:
        print(f'  FAIL {f}')
    return not rep.failures


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--install', required=True, help='install space holding openarm_description')
    ap.add_argument('--golden', required=True, help='path of golden.json')
    ap.add_argument('--update-golden', action='store_true',
                    help='rewrite golden.json from this install instead of comparing')
    a = ap.parse_args()
    sys.exit(0 if run(a.install, a.golden, a.update_golden) else 1)


if __name__ == '__main__':
    main()
