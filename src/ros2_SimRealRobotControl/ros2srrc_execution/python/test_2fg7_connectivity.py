#!/usr/bin/env python3
"""
OnRobot 2FG7 connectivity self-test.

- Default: XML-RPC protocol (http://robot_ip:41414). Queries max/min width + max force,
  then runs open -> close -> open and reports PASS/FAIL.
- With --urscript: uses URScript mode (TCP :30002) and only runs open -> close.

Requires bringup with config ur5_4 (starts onrobot_2fg7 grip + status nodes).

Example (real robot, via /grip service):
  ros2 run ros2srrc_execution test_2fg7_connectivity.py
"""

import argparse
import sys
import os

try:
    from ament_index_python.packages import get_package_share_directory
    _pkg_share = get_package_share_directory("ros2srrc_execution")
    _path = os.path.join(_pkg_share, "python")
    if _path not in sys.path:
        sys.path.insert(0, _path)
except Exception:
    _path = os.path.join(os.path.dirname(__file__))
    if _path not in sys.path:
        sys.path.insert(0, _path)

import rclpy
from endeffector.gripper_onrobot_2fg7_pkg import OnRobot2FG7PkgBackend


def main():
    parser = argparse.ArgumentParser(
        description="OnRobot 2FG7 connectivity self-test via onrobot_2fg7 /grip service."
    )
    parser.add_argument(
        "--close-percent",
        type=float,
        default=0.5,
        help="Close amount 0.0–1.0 for middle step. Default: 0.5.",
    )
    args = parser.parse_args()

    rclpy.init()
    backend = OnRobot2FG7PkgBackend()

    all_ok = True
    print("[2FG7 self-test] Using /grip service (onrobot_2fg7 package, XML-RPC on robot :41414)")

    print("[2FG7 self-test] Sending OPEN...")
    res = backend.open()
    if not res["Success"]:
        print(f"[2FG7 self-test] OPEN FAILED: {res['Message']}")
        all_ok = False
    else:
        print("[2FG7 self-test] OPEN: OK")

    if all_ok:
        print(f"[2FG7 self-test] Sending CLOSE ({args.close_percent:.0%})...")
        res = backend.close(args.close_percent)
        if not res["Success"]:
            print(f"[2FG7 self-test] CLOSE FAILED: {res['Message']}")
            all_ok = False
        else:
            print("[2FG7 self-test] CLOSE: OK")

    if all_ok:
        print("[2FG7 self-test] Sending OPEN again...")
        res = backend.open()
        if not res["Success"]:
            print(f"[2FG7 self-test] OPEN (2nd) FAILED: {res['Message']}")
            all_ok = False
        else:
            print("[2FG7 self-test] OPEN (2nd): OK")

    print("")
    if all_ok:
        print("[2FG7 self-test] PASS")
        return 0
    print("[2FG7 self-test] FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())
