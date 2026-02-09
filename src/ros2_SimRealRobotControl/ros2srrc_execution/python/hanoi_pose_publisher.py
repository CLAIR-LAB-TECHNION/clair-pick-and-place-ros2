#!/usr/bin/python3
"""
hanoi_pose_publisher.py - Publish known object poses for Hanoi (real robot, no perception)

Subscribes to /hanoi/object_pose (geometry_msgs/PointStamped, frame_id = object name).
Republishes to /object_poses/<name> as nav_msgs/Odometry for pick compatibility.

Used when running Hanoi on real robot with --skip_spawn: physical cubes at known positions,
no Gazebo. Hanoi publishes computed poses before each pick; this node republishes them.

Usage:
  ros2 run ros2srrc_execution hanoi_pose_publisher.py

  (Launch in a separate terminal before running Hanoi with --ee_type and --skip_spawn)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


class HanoiPosePublisher(Node):
    """Republish /hanoi/object_pose -> /object_poses/<name> as Odometry."""

    def __init__(self):
        super().__init__("hanoi_pose_publisher")
        # Transient local so late-joining pick gets last message
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(
            PointStamped,
            "/hanoi/object_pose",
            self.callback,
            10,
        )
        self.publishers = {}  # name -> Publisher
        self.get_logger().info("Hanoi pose publisher: subscribed to /hanoi/object_pose")

    def callback(self, msg):
        name = msg.header.frame_id
        if not name:
            self.get_logger().warn("Received pose with empty frame_id, ignoring")
            return
        if name not in self.publishers:
            self.publishers[name] = self.create_publisher(
                Odometry,
                f"/object_poses/{name}",
                QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                ),
            )
        odom = Odometry()
        # Frame must match MoveIt planning frame (SRDF: parent_frame = "world" for UR5)
        odom.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="world")
        odom.pose.pose.position.x = msg.point.x
        odom.pose.pose.position.y = msg.point.y
        odom.pose.pose.position.z = msg.point.z
        odom.pose.pose.orientation.w = 1.0
        self.publishers[name].publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = HanoiPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
