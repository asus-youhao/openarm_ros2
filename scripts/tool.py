#!/usr/bin/env python3
import argparse
import json
import math
from typing import List


DEFAULT_OFFSET = math.pi
DEFAULT_JOINTS_OPEN = [3.14] * 16


def parse_float_list(raw: str) -> List[float]:
    raw = raw.strip()
    if not raw:
        return []
    if raw.startswith('[') and raw.endswith(']'):
        raw = raw[1:-1]
    parts = [p for p in raw.replace(',', ' ').split() if p]
    return [float(p) for p in parts]


def joints_to_positions(joints: List[float], offset: float = DEFAULT_OFFSET) -> List[float]:
    return [j - offset for j in joints]


def positions_to_joints(positions: List[float], offset: float = DEFAULT_OFFSET) -> List[float]:
    return [p + offset for p in positions]


def print_result(title: str, values: List[float], precision: int) -> None:
    rounded = [round(v, precision) for v in values]
    print(f'{title} (list): {rounded}')
    print(f'{title} (json): {json.dumps(rounded)}')


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='LEAP Hand joint <-> position conversion tool'
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--joints', type=str, help='Motor joint angles (e.g. "3.14 3.14 ...")')
    group.add_argument('--positions', type=str, help='ROS/URDF positions (e.g. "0 0 ...")')
    group.add_argument('--example-open', action='store_true', help='Use 16 values of 3.14 as joints')
    parser.add_argument('--offset', type=float, default=DEFAULT_OFFSET, help='Offset (default: pi)')
    parser.add_argument('--precision', type=int, default=3, help='Rounding precision')
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if args.example_open:
        joints = DEFAULT_JOINTS_OPEN
        positions = joints_to_positions(joints, args.offset)
        print_result('positions', positions, args.precision)
        return

    if args.joints is not None:
        joints = parse_float_list(args.joints)
        positions = joints_to_positions(joints, args.offset)
        print_result('positions', positions, args.precision)
        return

    if args.positions is not None:
        positions = parse_float_list(args.positions)
        joints = positions_to_joints(positions, args.offset)
        print_result('joints', joints, args.precision)
        return


if __name__ == '__main__':
    main()
