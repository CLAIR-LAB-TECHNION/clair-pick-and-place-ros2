# gripper_interface.py
# Generic gripper abstraction with normalized close percentage (0.0–1.0).
# All gripper backends implement this interface; task logic is hardware-agnostic.

from abc import ABC, abstractmethod


class GripperInterface(ABC):
    """Abstract interface for gripper control.
    percent is normalized in [0.0, 1.0]: 0.0 = fully open, 1.0 = fully closed.
    """

    @abstractmethod
    def open(self):
        """Open the gripper fully. Returns dict with Success, Message, ExecTime."""
        pass

    @abstractmethod
    def close(self, percent: float = 1.0):
        """Close gripper to given percentage (0.0–1.0).
        percent is clamped inside the method.
        Returns dict with Success, Message, ExecTime."""
        pass

    @staticmethod
    def _clamp_percent(percent: float) -> float:
        """Clamp percent to [0.0, 1.0]."""
        return max(0.0, min(1.0, float(percent)))
