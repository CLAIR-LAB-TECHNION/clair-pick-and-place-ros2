# gripper_onrobot_2fg7_pkg.py
# OnRobot 2FG7 via davedovrat/onrobot_2fg7 ROS 2 service (/grip → XML-RPC on robot :41414).
# Requires bringup to start onrobot_2fg7 grip + status nodes and OnRobot 2FG7 URCap on the robot.

import time

import rclpy
from rclpy.node import Node
from onrobot_2fg7_interfaces.srv import Grip

from .gripper_interface import GripperInterface

_DEFAULT_OPEN_MM = 110.0
_DEFAULT_MIN_MM = 5.0
_DEFAULT_FORCE = 80
_DEFAULT_SQUEEZE_MM = 2.0
_DEFAULT_RELEASE_FORCE = 80
_DEFAULT_SPEED = 50
_DEFAULT_GRIPPER_ID = 0
_DEFAULT_SETTLE_S = 0.75
_GRIP_SERVICE = "/grip"


class _GripServiceClient(Node):
    def __init__(
        self,
        open_width_mm: float,
        min_width_mm: float,
        force: int,
        speed: int,
        gripper_id: int,
        settle_time_s: float,
    ):
        super().__init__("onrobot_2fg7_grip_client")
        self._open_mm = float(open_width_mm)
        self._min_mm = float(min_width_mm)
        self._force = int(force)
        self._release_force = int(_DEFAULT_RELEASE_FORCE)
        self._speed = int(speed)
        self._gripper_id = int(gripper_id)
        self._settle_time_s = float(settle_time_s)
        self._client = self.create_client(Grip, _GRIP_SERVICE)

    def call_grip(self, gap_mm: float, force: int | None = None) -> tuple[bool, str, float]:
        if not self._client.wait_for_service(timeout_sec=5.0):
            return (
                False,
                f"Service {_GRIP_SERVICE} not available. "
                "Start bringup with config ur5_4 (onrobot_2fg7 driver).",
                0.0,
            )
        req = Grip.Request()
        req.gap = float(gap_mm)
        req.id = self._gripper_id
        req.force = int(force if force is not None else self._force)
        req.speed = self._speed
        start = time.monotonic()
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if not future.done() or future.result() is None:
            return False, "Grip service call failed or timed out", time.monotonic() - start
        if self._settle_time_s > 0.0:
            time.sleep(self._settle_time_s)
        return True, f"gap={gap_mm:.1f} mm", time.monotonic() - start


class OnRobot2FG7PkgBackend(GripperInterface):
    """OnRobot 2FG7 through onrobot_2fg7 package /grip service (XML-RPC port 41414 on robot)."""

    def __init__(
        self,
        open_width_mm: float | None = None,
        min_width_mm: float | None = None,
        force: int | None = None,
        speed: int | None = None,
        gripper_id: int | None = None,
        settle_time_s: float | None = None,
    ):
        rclpy.init() if not rclpy.ok() else None
        self._client = _GripServiceClient(
            open_width_mm=open_width_mm if open_width_mm is not None else _DEFAULT_OPEN_MM,
            min_width_mm=min_width_mm if min_width_mm is not None else _DEFAULT_MIN_MM,
            force=force if force is not None else _DEFAULT_FORCE,
            speed=speed if speed is not None else _DEFAULT_SPEED,
            gripper_id=gripper_id if gripper_id is not None else _DEFAULT_GRIPPER_ID,
            settle_time_s=settle_time_s if settle_time_s is not None else _DEFAULT_SETTLE_S,
        )

    def _gap_for_percent(self, percent: float) -> float:
        p = self._clamp_percent(percent)
        return self._client._open_mm * (1.0 - p) + self._client._min_mm * p

    def open(self):
        ok, msg, exec_time = self._client.call_grip(
            self._client._open_mm, force=self._client._release_force
        )
        return {"Success": ok, "Message": msg, "ExecTime": exec_time}

    def close(self, percent: float = 1.0):
        p = self._clamp_percent(percent)
        if p >= 0.999:
            gap = 0.0
        else:
            gap = self._gap_for_percent(p)
            # Extra squeeze so fingers reach the object (gap = finger separation in mm).
            gap = max(self._client._min_mm, gap - _DEFAULT_SQUEEZE_MM)
        ok, msg, exec_time = self._client.call_grip(gap)
        return {"Success": ok, "Message": msg, "ExecTime": exec_time}
