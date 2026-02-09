# gripper_robotiq.py
# Robotiq backend: uses /Robotiq_Gripper service.
# Converts normalized percent (0.0–1.0) to Robotiq position (0–255).

import rclpy
from rclpy.node import Node
from ros2_robotiqgripper.srv import RobotiqGripper

from .gripper_interface import GripperInterface


class RobotiqServiceClient(Node):
    """ROS 2 service client for Robotiq Gripper."""

    def __init__(self):
        super().__init__("RobotiqGripper_client")
        self.cli = self.create_client(RobotiqGripper, "/Robotiq_Gripper")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for RobotiqGripper service to be available..."
            )
        self.req = RobotiqGripper.Request()

    def call_open(self):
        self.req.action = "OPEN"
        self.req.position = -1
        return self.cli.call_async(self.req)

    def call_close(self, position: int):
        self.req.action = "CLOSE"
        self.req.position = position
        return self.cli.call_async(self.req)


class RobotiqGripperBackend(GripperInterface):
    """Robotiq gripper backend using /Robotiq_Gripper service.
    Converts percent (0.0–1.0) to pos_255 = round(255 * percent).
    """

    def __init__(self):
        rclpy.init() if not rclpy.ok() else None
        self._client = RobotiqServiceClient()

    def open(self):
        future = self._client.call_open()
        while rclpy.ok():
            rclpy.spin_once(self._client)
            if future.done():
                try:
                    res = future.result()
                    return {
                        "Success": res.success,
                        "Message": res.message,
                        "ExecTime": -1.0,
                    }
                except Exception as exc:
                    return {
                        "Success": False,
                        "Message": f"Robotiq OPEN failed: {exc}",
                        "ExecTime": -1.0,
                    }
        return {"Success": False, "Message": "rclpy not ok", "ExecTime": -1.0}

    def close(self, percent: float = 1.0):
        p = self._clamp_percent(percent)
        pos_255 = round(255 * p)
        future = self._client.call_close(pos_255)
        while rclpy.ok():
            rclpy.spin_once(self._client)
            if future.done():
                try:
                    res = future.result()
                    return {
                        "Success": res.success,
                        "Message": res.message,
                        "ExecTime": -1.0,
                    }
                except Exception as exc:
                    return {
                        "Success": False,
                        "Message": f"Robotiq CLOSE failed: {exc}",
                        "ExecTime": -1.0,
                    }
        return {"Success": False, "Message": "rclpy not ok", "ExecTime": -1.0}
