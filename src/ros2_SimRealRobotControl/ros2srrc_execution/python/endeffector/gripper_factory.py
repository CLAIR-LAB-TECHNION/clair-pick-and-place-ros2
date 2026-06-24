# gripper_factory.py
# Single selection point for gripper backend based on EndEffector config.
# EndEffector: onrobot_2fg7 (real 2FG7) | onrobot_ros2 (RG2/RG6 serial) | ParallelGripper (sim)

import os
import sys

from .gripper_interface import GripperInterface


def create_gripper(
    end_effector: str,
    robot: str = "ur5",
    ee_link: str = "EE_robotiq_2f85",
    object_list=None,
) -> GripperInterface:
    """Create gripper backend from config."""
    end_effector = (end_effector or "").strip()

    if end_effector == "onrobot_2fg7":
        from .gripper_onrobot_2fg7_pkg import OnRobot2FG7PkgBackend
        return OnRobot2FG7PkgBackend()

    if end_effector == "onrobot_ros2":
        from .gripper_onrobot_ros2 import OnRobotRos2Backend
        return OnRobotRos2Backend()

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
        "Use: onrobot_2fg7 | onrobot_ros2 | ParallelGripper"
    )
