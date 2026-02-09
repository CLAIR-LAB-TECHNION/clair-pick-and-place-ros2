# gripper_factory.py
# Single selection point for gripper backend based on EndEffector config.
# EndEffector: robotiq_2f85 | onrobot_2fg7 | ParallelGripper (sim)

import os
import sys

from .gripper_interface import GripperInterface


def create_gripper(
    end_effector: str,
    robot: str = "ur5",
    ee_link: str = "EE_robotiq_2f85",
    object_list=None,
) -> GripperInterface:
    """Create gripper backend from config.
    end_effector: 'robotiq_2f85' | 'onrobot_2fg7' | 'ParallelGripper'
    """
    end_effector = (end_effector or "").strip()

    if end_effector in ("robotiq_2f85", "RobotiqHandE/UR"):
        from .gripper_robotiq import RobotiqGripperBackend
        return RobotiqGripperBackend()

    if end_effector == "onrobot_2fg7":
        from .gripper_onrobot_2fg7 import OnRobot2FG7Backend
        return OnRobot2FG7Backend()

    if end_effector == "ParallelGripper":
        from ament_index_python.packages import get_package_share_directory
        from .gripper_parallel_adapter import ParallelGripperAdapter
        path = get_package_share_directory("ros2srrc_execution")
        path_eegz = os.path.join(path, "python", "endeffector_gz")
        if path_eegz not in sys.path:
            sys.path.insert(0, path_eegz)
        from parallelGripper import parallelGR
        pg = parallelGR(object_list or [], robot, ee_link)
        return ParallelGripperAdapter(pg)

    raise ValueError(
        f"Unknown EndEffector '{end_effector}'. "
        "Use: robotiq_2f85 | onrobot_2fg7 | ParallelGripper"
    )
