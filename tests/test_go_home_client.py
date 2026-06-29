#!/usr/bin/env python3
"""
test_go_home_client.py
======================
Client-side test for the /{arm}/go_home service (std_srvs/Trigger) added to
PlacoOnlineProfiler.

The profiler node must already be running, e.g.:

    python3 placo_ik/ik_node/placo_ik_online_profiler_ws_mesh.py --arm right

Then call this client:

    python3 placo_ik/tests/test_go_home_client.py --arm right
    python3 placo_ik/tests/test_go_home_client.py --arm both   # right then left

What it checks:
  1. The service /{arm}/go_home exists and becomes available within --wait-sec.
  2. Calling it returns a Trigger response.
  3. response.success is True and a message is returned.

Exit code is 0 only if every requested arm homes successfully, so this can be
wired into CI / shell scripts.
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class GoHomeTestClient(Node):
    def __init__(self):
        super().__init__("go_home_test_client")

    def call_go_home(self, arm: str, wait_sec: float, call_timeout: float) -> bool:
        """Call /{arm}/go_home and return True iff response.success is True."""
        srv_name = f"/{arm}/go_home"
        client = self.create_client(Trigger, srv_name)

        print(f"[{arm}] waiting for service {srv_name} (<= {wait_sec:.0f}s)...")
        if not client.wait_for_service(timeout_sec=wait_sec):
            print(f"[{arm}] FAIL — service {srv_name} not available")
            return False

        print(f"[{arm}] service up, sending Trigger request...")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=call_timeout)

        if not future.done():
            print(f"[{arm}] FAIL — no response within {call_timeout:.0f}s")
            return False

        resp = future.result()
        if resp is None:
            print(f"[{arm}] FAIL — service call raised / returned None")
            return False

        status = "OK" if resp.success else "FAIL"
        print(f"[{arm}] {status} — success={resp.success} message='{resp.message}'")
        return bool(resp.success)


def main():
    p = argparse.ArgumentParser(description="Client test for /{arm}/go_home")
    p.add_argument("--arm", default="right",
                   choices=["right", "left", "both"],
                   help="which arm's go_home service to call (default: right)")
    p.add_argument("--wait-sec", type=float, default=10.0,
                   help="how long to wait for the service to appear")
    p.add_argument("--call-timeout", type=float, default=30.0,
                   help="how long to wait for the homing response "
                        "(node blocks until homing finishes, default 30s)")
    args = p.parse_args()

    arms = ["right", "left"] if args.arm == "both" else [args.arm]

    rclpy.init()
    node = GoHomeTestClient()
    all_ok = True
    try:
        for arm in arms:
            ok = node.call_go_home(arm, args.wait_sec, args.call_timeout)
            all_ok = all_ok and ok
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("\n=== RESULT: " + ("PASS" if all_ok else "FAIL") + " ===")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
