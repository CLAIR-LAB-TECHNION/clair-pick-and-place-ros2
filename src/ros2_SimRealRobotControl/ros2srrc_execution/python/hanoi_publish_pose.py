#!/usr/bin/python3
"""
Publish cube pose to /object_poses/<name>.

Keeps publishing for a few seconds so pick.py (or ros2 topic echo) can connect.
Used by Hanoi (real robot) to provide known poses to pick.

Usage:
  ros2 run ros2srrc_execution hanoi_publish_pose.py name:=cube1 x:=0.15 y:=0.48 z:=0.865
  ros2 run ros2srrc_execution hanoi_publish_pose.py name:=cube1 x:=0.15 y:=0.48 z:=0.865 duration:=60
"""

import sys
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


def _parse_args():
    name = x = y = z = None
    duration = 30.0
    for a in sys.argv[1:]:
        if a.startswith("name:="):
            name = a[6:]
        elif a.startswith("x:="):
            x = float(a[3:])
        elif a.startswith("y:="):
            y = float(a[3:])
        elif a.startswith("z:="):
            z = float(a[3:])
        elif a.startswith("duration:="):
            duration = float(a[10:])
    return name, x, y, z, duration


def main():
    name, x, y, z, duration = _parse_args()
    if name is None or x is None or y is None or z is None:
        print("Usage: name:=<obj> x:=<x> y:=<y> z:=<z> [duration:=<seconds>]")
        print("  duration: how long to keep publishing (default 30). Use Ctrl+C to stop early.")
        sys.exit(1)

    # Remove custom args so rclpy.init does not treat them as remaps
    for a in list(sys.argv[1:]):
        if a.startswith(("name:=", "x:=", "y:=", "z:=", "duration:=")):
            sys.argv.remove(a)

    rclpy.init()
    node = rclpy.create_node("hanoi_publish_pose")
    topic = f"/object_poses/{name}"
    pub = node.create_publisher(
        Odometry,
        topic,
        QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        ),
    )

    msg = Odometry()
    msg.header.frame_id = "world"
    msg.pose.pose.orientation.w = 1.0

    # Allow subscribers to discover the publisher
    #time.sleep(0.5)

    deadline = time.time() + max(duration, 0.5)
    print(f"Publishing {name} pose to {topic} (world): x={x}, y={y}, z={z}")
    print(f"Keeping publisher alive for {duration:.0f}s — run pick.py in another terminal.")
    print("Press Ctrl+C to stop early.")

    try:
        while time.time() < deadline:
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = z
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.1)
            #time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopped.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
