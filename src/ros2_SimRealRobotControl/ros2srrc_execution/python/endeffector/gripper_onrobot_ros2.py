# gripper_onrobot_ros2.py
# OnRobot gripper backend via UR_OnRobot_ROS2 / OnRobot_ROS2_Driver (ros2_control).
# Publishes finger width to the sidecar onrobot_driver stack (default namespace /onrobot).

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .gripper_interface import GripperInterface

_RG2_OPEN_M = 0.085
_RG6_OPEN_M = 0.160
_DEFAULT_CLOSED_M = 0.0
_DEFAULT_POSITION_TOPIC = "/onrobot/finger_width_controller/commands"


def _open_width_for_type(onrobot_type: str, open_width_m: float) -> float:
    if open_width_m > 0.0:
        return open_width_m
    return _RG6_OPEN_M if (onrobot_type or "rg2").strip().lower() == "rg6" else _RG2_OPEN_M


class _OnRobotRos2Client(Node):
    def __init__(
        self,
        onrobot_type: str = "rg2",
        position_topic: str = _DEFAULT_POSITION_TOPIC,
        open_width_m: float = 0.0,
        closed_width_m: float = _DEFAULT_CLOSED_M,
        settle_time_s: float = 0.5,
    ):
        super().__init__("OnRobotRos2_gripper_client")
        self._onrobot_type = onrobot_type
        self._open_width_m = open_width_m
        self._closed_width_m = closed_width_m
        self._settle_time_s = settle_time_s
        self._pub = self.create_publisher(Float64MultiArray, position_topic, 10)

    def _default_open_width(self) -> float:
        return _open_width_for_type(self._onrobot_type, self._open_width_m)

    def _width_for_percent(self, percent: float) -> float:
        open_w = self._default_open_width()
        p = max(0.0, min(1.0, float(percent)))
        return open_w + (1.0 - p) * (self._closed_width_m - open_w)

    def send_width(self, width_m: float) -> tuple[bool, str, float]:
        start = time.monotonic()
        msg = Float64MultiArray()
        msg.data = [float(width_m)]
        self._pub.publish(msg)
        if self._settle_time_s > 0.0:
            time.sleep(self._settle_time_s)
        return True, f"width={width_m:.4f} m", time.monotonic() - start


class OnRobotRos2Backend(GripperInterface):
    """OnRobot RG2/RG6 via ros2_control finger_width_controller (UR_OnRobot_ROS2 stack)."""

    def __init__(
        self,
        onrobot_type: str | None = None,
        position_topic: str | None = None,
        open_width_m: float | None = None,
        closed_width_m: float | None = None,
        settle_time_s: float | None = None,
    ):
        rclpy.init() if not rclpy.ok() else None
        self._client = _OnRobotRos2Client(
            onrobot_type=onrobot_type or "rg2",
            position_topic=position_topic or _DEFAULT_POSITION_TOPIC,
            open_width_m=open_width_m if open_width_m is not None else 0.0,
            closed_width_m=closed_width_m if closed_width_m is not None else _DEFAULT_CLOSED_M,
            settle_time_s=settle_time_s if settle_time_s is not None else 0.5,
        )

    def open(self):
        width = self._client._default_open_width()
        ok, msg, exec_time = self._client.send_width(width)
        return {"Success": ok, "Message": msg, "ExecTime": exec_time}

    def close(self, percent: float = 1.0):
        p = self._clamp_percent(percent)
        width = self._client._width_for_percent(p)
        ok, msg, exec_time = self._client.send_width(width)
        return {"Success": ok, "Message": msg, "ExecTime": exec_time}
