#!/usr/bin/python3
"""
Publish a single cube pose to /object_poses/<name> and exit.
Used by Hanoi (real robot) to provide known poses to pick.

Usage: ros2 run ros2srrc_execution hanoi_publish_pose.py name:=cube_0 x:=0.1 y:=0.5 z:=0.3
"""

import sys
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


def main():
    # Parse args before rclpy.init (avoids ROS remap interpretation)
    args = [a for a in sys.argv[1:] if a.startswith("name:=") or a.startswith("x:=") or a.startswith("y:=") or a.startswith("z:=")]
    name = x = y = z = None
    for a in args:
        if a.startswith("name:="):
            name = a[6:]
        elif a.startswith("x:="):
            x = float(a[3:])
        elif a.startswith("y:="):
            y = float(a[3:])
        elif a.startswith("z:="):
            z = float(a[3:])
    if name is None or x is None or y is None or z is None:
        print("Usage: name:=<obj> x:=<x> y:=<y> z:=<z>")
        sys.exit(1)
    # Remove our args so rclpy.init doesn't see them
    for a in args:
        if a in sys.argv:
            sys.argv.remove(a)
    rclpy.init()
    node = rclpy.create_node("hanoi_publish_pose")
    pub = node.create_publisher(
        Odometry,
        f"/object_poses/{name}",
        QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        ),
    )
    msg = Odometry()
    # Frame must match MoveIt planning frame (SRDF virtual_joint parent_frame = "world" for UR5 sim/real)
    msg.header.frame_id = "world"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.w = 1.0
    pub.publish(msg)
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
