#!/usr/bin/env python3
"""
OnRobot 2FG7 connectivity self-test.

- Default: XML-RPC protocol (http://robot_ip:41414). Queries max/min width + max force,
  then runs open -> close -> open and reports PASS/FAIL.
- With --urscript: uses URScript mode (TCP :30002) and only runs open -> close.

Example (real robot, XML-RPC):
  ros2 run ros2srrc_execution test_2fg7_connectivity.py
  ros2 run ros2srrc_execution test_2fg7_connectivity.py --ros-args -p OnRobot2FG7_param_reader.robot_ip:=192.168.1.10

Example (URScript fallback):
  ros2 run ros2srrc_execution test_2fg7_connectivity.py --urscript --close-width-mm 40
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
from endeffector.gripper_onrobot_2fg7 import OnRobot2FG7Backend, PROTOCOL_XMLRPC, PROTOCOL_URSCRIPT


def main():
    parser = argparse.ArgumentParser(
        description="OnRobot 2FG7 connectivity self-test (XML-RPC: query + open/close/open; or URScript: open/close)."
    )
    parser.add_argument(
        "--urscript",
        action="store_true",
        help="Use URScript protocol (TCP :30002) instead of XML-RPC (:41414).",
    )
    parser.add_argument(
        "--close-width-mm",
        type=float,
        default=50.0,
        help="Width in mm for close step (URScript or XML-RPC close). Default: 50.",
    )
    parser.add_argument(
        "--no-open",
        action="store_true",
        help="Skip open step (only close, for URScript).",
    )
    parser.add_argument(
        "--no-close",
        action="store_true",
        help="Skip close step (only open).",
    )
    args = parser.parse_args()

    rclpy.init()
    overrides = {}
    if args.urscript:
        overrides["protocol"] = PROTOCOL_URSCRIPT
    backend = OnRobot2FG7Backend(**overrides)
    cfg = backend._get_config()
    protocol = (cfg.get("protocol") or PROTOCOL_XMLRPC).strip().lower()

    all_ok = True

    if protocol == PROTOCOL_XMLRPC:
        # Query limits
        print("[2FG7 self-test] Protocol: XML-RPC (http://robot_ip:41414)")
        print("[2FG7 self-test] Querying max/min external width and max force...")
        ok, msg, limits = backend.query_limits_xmlrpc()
        if not ok:
            print(f"[2FG7 self-test] Query FAILED: {msg}")
            return 1
        print(f"[2FG7 self-test] max_external_width={limits['max_ext']:.1f} mm, min_external_width={limits['min_ext']:.1f} mm, max_force={limits['max_force']}")
        # open -> close -> open
        if not args.no_open:
            print("[2FG7 self-test] Sending OPEN...")
            res = backend.open()
            if not res["Success"]:
                print(f"[2FG7 self-test] OPEN FAILED: {res['Message']}")
                all_ok = False
            else:
                print("[2FG7 self-test] OPEN: OK")
        if all_ok and not args.no_close:
            # close to a width in [min_ext, max_ext]
            width_mm = max(limits["min_ext"], min(limits["max_ext"], args.close_width_mm))
            percent = 1.0 - (width_mm - limits["min_ext"]) / (limits["max_ext"] - limits["min_ext"]) if (limits["max_ext"] - limits["min_ext"]) > 0 else 1.0
            print(f"[2FG7 self-test] Sending CLOSE to {width_mm:.1f} mm...")
            res = backend.close(percent)
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
    else:
        # URScript path
        print("[2FG7 self-test] Protocol: URScript (TCP :30002)")
        if not args.no_open:
            print("[2FG7 self-test] Sending OPEN...")
            res = backend.open()
            if not res["Success"]:
                print(f"[2FG7 self-test] OPEN FAILED: {res['Message']}")
                all_ok = False
            else:
                print("[2FG7 self-test] OPEN: OK")
        if all_ok and not args.no_close:
            jaw_open = float(cfg.get("jaw_width_open_mm", 110.0))
            width_mm = max(0.0, min(jaw_open, args.close_width_mm))
            percent = 1.0 - (width_mm / jaw_open) if jaw_open > 0 else 1.0
            print(f"[2FG7 self-test] Sending CLOSE to {width_mm:.1f} mm...")
            res = backend.close(percent)
            if not res["Success"]:
                print(f"[2FG7 self-test] CLOSE FAILED: {res['Message']}")
                all_ok = False
            else:
                print("[2FG7 self-test] CLOSE: OK")

    print("")
    if all_ok:
        print("[2FG7 self-test] PASS")
        return 0
    print("[2FG7 self-test] FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())
