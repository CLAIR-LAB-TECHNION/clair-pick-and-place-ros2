#!/usr/bin/env python3
"""Quick open/close test for OnRobot RG gripper via UR_OnRobot_ROS2 sidecar driver.

Requires bringup with config ur5_4 (starts tool_communication + onrobot_driver).

  ros2 run ros2srrc_execution test_onrobot_ros2_connectivity.py
  ros2 run ros2srrc_execution test_onrobot_ros2_connectivity.py --close-percent 0.5
"""

import sys

import rclpy

from endeffector.gripper_onrobot_ros2 import OnRobotRos2Backend


def main(args=None):
    rclpy.init(args=args)
    close_percent = 1.0
    for arg in sys.argv:
        if arg.startswith("--close-percent"):
            close_percent = float(arg.split(":=")[-1] if ":=" in arg else sys.argv[sys.argv.index(arg) + 1])

    backend = OnRobotRos2Backend(settle_time_s=1.0)
    print("Opening gripper...")
    res = backend.open()
    print(res)
    if not res.get("Success"):
        return 1
    print(f"Closing gripper to {close_percent:.0%}...")
    res = backend.close(close_percent)
    print(res)
    if not res.get("Success"):
        return 1
    print("Opening gripper again...")
    res = backend.open()
    print(res)
    return 0 if res.get("Success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
