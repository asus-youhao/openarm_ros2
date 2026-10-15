#!/usr/bin/env python3
"""
Check that package.xml declares what the package uses, for Humble and Jazzy.

Called by run_full_test.sh.
  D1  the runtime dependencies in package.xml (<depend>, <exec_depend>) are
      exactly the packages the package uses: Node(package=...) in launch/,
      the xacro module, and $(find ...) / package:// in urdf/ other than
      openarm_description itself
  D2  every dependency key resolves with rosdep for humble on ubuntu:jammy
      and jazzy on ubuntu:noble (SKIP when rosdep has no local cache)
"""

import argparse
import glob
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

SELF = 'openarm_description'
TARGETS = [('humble', 'ubuntu:jammy'), ('jazzy', 'ubuntu:noble')]


def declared(pkg_dir):
    root = ET.parse(os.path.join(pkg_dir, 'package.xml')).getroot()
    runtime = {e.text.strip() for tag in ('depend', 'exec_depend') for e in root.iter(tag)}
    every = runtime | {e.text.strip() for e in root
                       if e.tag.endswith('depend') and e.tag not in ('depend', 'exec_depend')}
    return runtime, every


def used(pkg_dir):
    found = set()
    for path in glob.glob(os.path.join(pkg_dir, 'launch', '*.py')):
        text = open(path).read()
        found |= set(re.findall(r'package\s*=\s*[\x27"](\w+)[\x27"]', text))
        if re.search(r'^import xacro|^from xacro', text, re.M):
            found.add('xacro')
    for path in glob.glob(os.path.join(pkg_dir, 'urdf', '**', '*.xacro'), recursive=True):
        text = open(path).read()
        found |= set(re.findall(r'\$\(find (\w+)\)', text))
        found |= set(re.findall(r'package://(\w+)/', text))
    found.discard(SELF)
    return found


def resolves(key, distro, os_name):
    proc = subprocess.run(['rosdep', 'resolve', key, '--rosdistro', distro, '--os', os_name],
                          capture_output=True, text=True)
    return proc.returncode == 0 and proc.stdout.strip() != '', proc.stdout + proc.stderr


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--package', required=True, help='path of the openarm_description package')
    a = ap.parse_args()

    ok = True
    runtime, every = declared(a.package)
    want = used(a.package)
    extra, missing = sorted(runtime - want), sorted(want - runtime)
    if extra or missing:
        ok = False
        print(f'  FAIL D1 declared but not used: {extra}; used but not declared: {missing}')
    else:
        print(f'  PASS D1 runtime dependencies match use: {sorted(runtime)}')

    cache = os.path.expanduser('~/.ros/rosdep/sources.cache')
    if not os.path.isdir(cache):
        print('  SKIP D2 no rosdep cache (run rosdep update)')
    else:
        for distro, os_name in TARGETS:
            bad = [k for k in sorted(every) if not resolves(k, distro, os_name)[0]]
            if bad:
                ok = False
                print(f'  FAIL D2 {distro} ({os_name}) cannot resolve: {bad}')
            else:
                print(f'  PASS D2 {distro} ({os_name}): {len(every)} keys resolve')
    print('deps_check: ' + ('passed' if ok else 'FAILED'))
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
