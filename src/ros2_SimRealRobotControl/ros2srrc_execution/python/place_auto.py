#!/usr/bin/python3

"""
place_auto.py - Automated Place Action

Automatically places an object at specified coordinates. All parameters are determined automatically:
- Place location is specified by x, y coordinates (z defaults to 0.50 if not provided)
- Place parameters are automatically calculated based on location
- Gripper orientation is automatically determined

Usage:
    ros2 run ros2srrc_execution place_auto.py x:=0.15 y:=0.48
    ros2 run ros2srrc_execution place_auto.py x:=0.15 y:=0.48 z:=0.50
    
Required arguments:
    x:=<value>                  X coordinate of place location (meters)
    y:=<value>                  Y coordinate of place location (meters)
    
Optional arguments:
    z:=<value>                  Z coordinate of place location (meters, default: 0.50)
    object:=<name>              Object name to detach (e.g., cube1) - required if object is attached
    robot:=ur5                  Robot name (default: ur5)
    ee_type:=ParallelGripper    End-effector type (default: ParallelGripper)
    ee_link:=EE_robotiq_2f85    End-effector link name (default: EE_robotiq_2f85)
    approach_height:=0.15       Approach height in meters (default: 0.15)
    place_z_offset:=0.03         Place Z offset (default: 0.03, place slightly above target)
"""

import sys
import os
import time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from ros2srrc_data.msg import Robpose

# Import the Place class from place.py
sys.path.append(os.path.join(get_package_share_directory("ros2srrc_execution"), 'python'))
from place import Place, PlaceConfig


def calculate_place_orientation(object_x, object_y, object_z, robot_base_x=0.0, robot_base_y=0.0, robot_base_z=0.0):
    """
    Calculate a good gripper orientation for placing an object.

    FOR NOW, THIS IS A FIXED ORIENTATION THAT WORKS FOR UR5 + PARALLEL GRIPPER.
    
    Returns:
        quaternion (qx, qy, qz, qw) suitable for placing
    """
    # Use proven fixed orientation that works for UR5 + parallel gripper
    # This is the same orientation used in the working YAML examples (ur5_pick_and_place.yaml)
    # It represents a good top-down placing pose that is reachable for most positions
    return -0.5, 0.5, 0.5, 0.5


def AssignArgument(ARGUMENT):
    """Parse command-line arguments in ROS2 style (arg:=value)."""
    ARGUMENTS = sys.argv
    for y in ARGUMENTS:
        if (ARGUMENT + ":=") in y:
            ARG = y.replace((ARGUMENT + ":="), "")
            return ARG
    return None


def main(args=None):
    """
    Automated Place action - requires x, y coordinates (z defaults to 0.50).
    """
    
    rclpy.init(args=args)
    
    print("==================================================")
    print("ROS 2 Sim-to-Real Robot Control: Automated Place")
    print("==================================================")
    print("")
    
    # ===== PARSE REQUIRED ARGUMENTS ===== #
    x = AssignArgument("x")
    y = AssignArgument("y")
    
    # Check required arguments
    missing_args = []
    if x is None: missing_args.append("x")
    if y is None: missing_args.append("y")
    
    if missing_args:
        print("ERROR: Missing required arguments: " + ", ".join(missing_args))
        print("")
        print("Usage: ros2 run ros2srrc_execution place_auto.py x:=<val> y:=<val> [z:=<val>]")
        print("")
        print("Example:")
        print("  ros2 run ros2srrc_execution place_auto.py x:=0.15 y:=0.48")
        print("  ros2 run ros2srrc_execution place_auto.py x:=0.15 y:=0.48 z:=0.50")
        print("")
        print("Required arguments:")
        print("  x:=<value>                  X coordinate of place location (meters)")
        print("  y:=<value>                  Y coordinate of place location (meters)")
        print("")
        print("Optional arguments:")
        print("  z:=<value>                  Z coordinate of place location (meters, default: 0.50)")
        print("  object:=<name>               Object name to detach (e.g., cube1) - recommended if object is attached)")
        print("  robot:=ur5                  Robot name (default: ur5)")
        print("  ee_type:=ParallelGripper    End-effector type (default: ParallelGripper)")
        print("  ee_link:=EE_robotiq_2f85    End-effector link (default: EE_robotiq_2f85)")
        print("  approach_height:=0.15       Approach height in meters (default: 0.15)")
        print("  place_z_offset:=0.03         Place Z offset (default: 0.03, place slightly above target)")
        print("  cube_size:=<value>          Cube size in meters (if provided, calculates offset = size/2 + 0.001m)")
        print("")
        print("Closing program... BYE!")
        rclpy.shutdown()
        exit(1)
    
    # ===== PARSE OPTIONAL ARGUMENTS ===== #
    z = AssignArgument("z")
    if z is None:
        z = "0.50"  # Default z value
    
    object_name = AssignArgument("object")  # Object name for detachment
    robot = AssignArgument("robot") or "ur5"
    ee_type = AssignArgument("ee_type") or "ParallelGripper"
    ee_link = AssignArgument("ee_link") or "EE_robotiq_2f85"
    
    approach_height = float(AssignArgument("approach_height") or "0.15")
    
    # Get cube size if provided (for dynamic offset calculation)
    cube_size = AssignArgument("cube_size")
    
    # Calculate place_z_offset dynamically based on cube size
    # If cube_size is provided, use: cube_height/2 + gripper_clearance
    # The gripper clearance is needed because the gripper fingers are below the cube
    # when holding it, so we need extra space to avoid collision
    # Otherwise use default: 0.03m
    if cube_size:
        cube_height = float(cube_size)
        # Gripper clearance: 0.04m (4cm) to ensure gripper doesn't collide with surface below
        # This accounts for the gripper fingers being below the cube when holding it
        gripper_clearance = 0.04 
        place_z_offset = (cube_height / 2.0) + gripper_clearance
        # print(f"[Place Auto]: Using dynamic place_z_offset based on cube size: {cube_height:.3f}m -> offset: {place_z_offset:.3f}m (includes {gripper_clearance*1000:.0f}mm gripper clearance)")  # DEBUG
    else:
        # Default place_z_offset: 0.03 (place slightly above target)
        # This helps avoid planning issues and ensures the object is placed on the surface
        place_z_offset = float(AssignArgument("place_z_offset") or "0.03")
        print(f"[Place Auto]: Using default place_z_offset: {place_z_offset:.3f}m")
    
    # Create place location pose from coordinates
    place_pose = Robpose()
    place_pose.x = float(x)
    place_pose.y = float(y)
    place_pose.z = float(z)
    
    print(f"Place Location: x={place_pose.x:.3f}, y={place_pose.y:.3f}, z={place_pose.z:.3f}")
    print(f"Robot: {robot}, EE Type: {ee_type}, EE Link: {ee_link}")
    print("")
    
    # ===== CALCULATE PLACE ORIENTATION ===== #
    print("============================================================")
    print("Calculating place pose...")
    print("")
    
    # Calculate optimal gripper orientation for placing
    # Use the SAME orientation as pick to ensure cubes are placed flat/parallel
    # This matches the pick orientation: -0.5, 0.5, 0.5, 0.5 (top-down, parallel to table)
    qx, qy, qz, qw = calculate_place_orientation(
        place_pose.x, place_pose.y, place_pose.z
    )
    
    # Set place pose orientation - ensure it matches pick orientation exactly
    # This ensures cubes are placed flat and parallel to the table
    place_pose.qx = qx
    place_pose.qy = qy
    place_pose.qz = qz
    place_pose.qw = qw
    
    print(f"[Place Auto]: Place location set successfully!")
    print(f"  Position: x={place_pose.x:.3f}, y={place_pose.y:.3f}, z={place_pose.z:.3f}")
    print(f"  Calculated place orientation: qx={qx:.3f}, qy={qy:.3f}, qz={qz:.3f}, qw={qw:.3f}")
    print("")
    
    # ===== LOAD ROBOT AND GRIPPER CLIENTS ===== #
    print("============================================================")
    print("Loading Robot+EndEffector Python Clients...")
    print("")
    
    # Import robot client
    PATH = os.path.join(get_package_share_directory("ros2srrc_execution"), 'python')
    PATH_ROB = PATH + "/robot"
    PATH_EEGz = PATH + "/endeffector_gz"
    PATH_EE = PATH + "/endeffector"
    
    sys.path.append(PATH_ROB)
    from robot import RBT
    
    print("ROBOT: ")
    RobotClient = RBT()
    print("Loaded.")
    print("")
    
    # Import and initialize gripper client
    print("END-EFFECTOR:")
    EEClient = None
    
    if ee_type == "None":
        print("Not required.")
    elif ee_type == "ParallelGripper":
        sys.path.append(PATH_EEGz)
        from parallelGripper import parallelGR  # type: ignore
        # For place, we need to enable OLCheck so OPEN can try to detach objects
        # OPEN will automatically try common object names if not provided
        if object_name:
            EEClient = parallelGR([object_name], robot, ee_link)
            print(f"Loaded -> ParallelGripper (will try to detach: {object_name}).")
        else:
            # Use placeholder to enable OLCheck - OPEN will try common names automatically
            EEClient = parallelGR(["place_object"], robot, ee_link)
            print("Loaded -> ParallelGripper (will automatically detect and detach attached object).")
    elif ee_type == "VacuumGripper":
        sys.path.append(PATH_EEGz)
        from vacuumGripper import vacuumGR  # type: ignore
        # For place, we need to enable OLCheck so DEACTIVATE can try to detach objects
        if object_name:
            EEClient = vacuumGR([object_name], robot, ee_link)
            print(f"Loaded -> VacuumGripper (will try to detach: {object_name}).")
        else:
            EEClient = vacuumGR(["place_object"], robot, ee_link)
            print("Loaded -> VacuumGripper (will automatically detect and detach attached object).")
    elif ee_type == "RobotiqHandE/UR":
        sys.path.append(PATH_EE)
        from robotiq_ur import RobotiqGRIPPER  # type: ignore
        EEClient = RobotiqGRIPPER()
        print("Loaded -> RobotiqHandE/UR.")
    else:
        print(f"WARNING: Unknown end-effector type '{ee_type}', continuing without gripper.")
    
    print("")
    
    # ===== WAIT FOR SYSTEM TO BE READY ===== #
    print("============================================================")
    print("Waiting for system to be fully ready...")
    print("")
    
    # Wait for robot pose topic to be publishing (if available)
    class PoseChecker(Node):
        def __init__(self):
            super().__init__('place_auto_pose_checker')
            self.pose_received = False
            self.pose_sub = self.create_subscription(
                Robpose, 
                '/Robpose', 
                self.pose_callback, 
                10
            )
        
        def pose_callback(self, msg):
            self.pose_received = True
    
    pose_checker = PoseChecker()
    
    # Wait up to 3 seconds for pose topic (non-blocking if topic doesn't exist)
    print("[Place Auto]: Checking robot pose topic availability...")
    timeout = time.time() + 3.0
    while time.time() < timeout and not pose_checker.pose_received:
        rclpy.spin_once(pose_checker, timeout_sec=0.1)
        if pose_checker.pose_received:
            break
    
    if pose_checker.pose_received:
        print("[Place Auto]: Robot pose topic is publishing.")
    else:
        print("[Place Auto]: Robot pose topic not detected (may not be required).")
    
    # Additional wait to ensure MoveIt!2 planning scene is ready
    print("[Place Auto]: Waiting for MoveIt!2 planning scene to initialize...")
    time.sleep(1.0)  # Give MoveIt!2 time to fully initialize planning scene
    print("[Place Auto]: System ready!")
    print("")
    
    # Clean up pose checker
    pose_checker.destroy_node()
    
    # ===== CREATE CONFIG AND EXECUTE PLACE ===== #
    print("============================================================")
    print("Executing Automated Place action...")
    print("")
    
    # Calculate post_open_push_down based on whether we used dynamic offset
    # If we used dynamic offset with gripper clearance, we need to push down by that amount
    # Otherwise use default 5mm
    if cube_size:
        # Push down by gripper clearance to settle cube on surface
        gripper_clearance = 0.02  # Must match the gripper_clearance used above
        post_open_push_down = gripper_clearance
    else:
        post_open_push_down = 0.005  # 5mm default
    
    config = {
        "approach_height": approach_height,
        "place_z_offset": place_z_offset,
        "post_open_push_down_m": post_open_push_down  # Push down after opening to settle cube on surface
    }
    
    PlaceAction = Place(RobotClient, EEClient, config)
    result = PlaceAction.execute(place_pose)
    
    # ===== RESULT ===== #
    print("============================================================")
    if result["Success"]:
        print("Automated Place action completed SUCCESSFULLY!")
    else:
        print("Automated Place action FAILED!")
    print(f"Message: {result['Message']}")
    print(f"Execution time: {result['ExecTime']}s")
    print("============================================================")
    
    rclpy.shutdown()
    exit(0 if result["Success"] else 1)


if __name__ == '__main__':
    main()
