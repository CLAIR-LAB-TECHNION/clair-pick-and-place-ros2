#!/usr/bin/env python3
"""
Minimal node that exposes robot_ip for OnRobot 2FG7.
Launched by bringup when ee_driver is onrobot_2fg7 so ExecuteProgram
can obtain robot_ip without requiring -p OnRobot2FG7_param_reader.robot_ip.
"""

import rclpy
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    node = Node("onrobot_2fg7_bringup_params")
    node.declare_parameter("robot_ip", "")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
