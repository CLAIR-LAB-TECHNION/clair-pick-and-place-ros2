#!/usr/bin/env python3
"""
Publishes default joint states for the Robotiq 2F-85 gripper joints so that
MoveIt's planning_scene_monitor sees a complete robot state on real robot bringup.
The UR driver only publishes arm joints; this node fills in the 6 gripper joints
with a fixed open pose (0.0) so the "missing joint" warning goes away.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# Robotiq 2F-85 non-fixed joints (from URDF). Default position 0.0 = open.
ROBOTIQ_85_JOINTS = [
    "robotiq_85_left_knuckle_joint",
    "robotiq_85_right_knuckle_joint",
    "robotiq_85_left_inner_knuckle_joint",
    "robotiq_85_right_inner_knuckle_joint",
    "robotiq_85_left_finger_tip_joint",
    "robotiq_85_right_finger_tip_joint",
]
DEFAULT_PUBLISH_RATE = 10.0  # Hz


def main(args=None):
    rclpy.init(args=args)
    node = Node("robotiq_85_joint_state_publisher")
    node.declare_parameter("rate", DEFAULT_PUBLISH_RATE)
    rate_hz = node.get_parameter("rate").value

    pub = node.create_publisher(JointState, "/joint_states", 10)
    msg = JointState()
    msg.name = list(ROBOTIQ_85_JOINTS)
    msg.position = [0.0] * len(ROBOTIQ_85_JOINTS)
    msg.velocity = [0.0] * len(ROBOTIQ_85_JOINTS)
    msg.effort = []

    def publish_cb():
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)

    timer = node.create_timer(1.0 / rate_hz, publish_cb)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
