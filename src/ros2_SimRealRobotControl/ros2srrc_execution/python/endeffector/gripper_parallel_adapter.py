# gripper_parallel_adapter.py
# Wraps endeffector_gz.parallelGR so it implements GripperInterface (open / close(percent))
# for use with the gripper factory when EndEffector is ParallelGripper (Gazebo sim).

from .gripper_interface import GripperInterface


class ParallelGripperAdapter(GripperInterface):
    """
    Adapter from parallelGR (Gazebo parallel gripper) to GripperInterface.
    percent in [0, 1] is converted to 0–100 for MoveG (CLOSE).
    """

    def __init__(self, parallel_gr):
        """
        parallel_gr: instance of endeffector_gz.parallelGripper.parallelGR
        """
        self._pg = parallel_gr

    def open(self):
        res = self._pg.OPEN()
        return {
            "Success": res.get("Success", False),
            "Message": res.get("Message", ""),
            "ExecTime": res.get("ExecTime", -1.0),
        }

    def close(self, percent: float = 1.0):
        p = self._clamp_percent(percent)
        # parallelGR.CLOSE expects 0–100 (MoveG value)
        val = p * 100.0
        res = self._pg.CLOSE(val)
        return {
            "Success": res.get("Success", False),
            "Message": res.get("Message", ""),
            "ExecTime": res.get("ExecTime", -1.0),
        }
