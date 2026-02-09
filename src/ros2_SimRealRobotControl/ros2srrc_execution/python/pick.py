#!/usr/bin/python3

"""
pick.py - Automated Pick Action

Automatically picks an object by name. All parameters are determined automatically:
- Object pose is retrieved from /object_poses/<name> topic
- Grasp parameters are automatically calculated based on object type
- Gripper orientation is automatically determined
- Fallback mechanism: Tries PTP (fixed kinematics) first, then different solutions if needed

Usage:
    ros2 run ros2srrc_execution pick.py object:=cube1
    
Optional arguments:
    robot:=ur5                  Robot name (default: ur5)
    ee_type:=ParallelGripper    End-effector type (default: ParallelGripper)
    ee_link:=EE_robotiq_2f85    End-effector link name (default: EE_robotiq_2f85)
    approach_height:=0.22       Approach height in meters (default: 0.22, works for all cube sizes up to ~80mm)
    grasp_z_offset:=0.04        Grasp Z offset (default: 0.04, works for all cube sizes up to ~80mm)
    min_lift_height:=<value>    Minimum lift height above object in meters (default: None, uses approach_height)
    cube_size:=<value>          Cube size/width in meters (if provided, calculates gripper close % automatically)
    gripper_value:=50.0         Gripper close percentage (default: 50.0, ignored if cube_size provided)
    gripper_open:=0.085         Gripper open distance in meters (default: 0.085 = 85mm for Robotiq 2F-85)
    gripper_closed:=0.00        Gripper closed distance in meters (default: 0.00)
    gripper_margin:=0.002       Squeeze margin in meters (default: 0.002 = 2mm)
    fallback_enabled:=true      Enable fallback mechanism (default: true)
    max_attempts:=12            Maximum fallback attempts (default: 12)
    grasp_yaw_offset_deg:=<deg> Extra yaw (deg) applied to grasp orientation (default: 0 = same as YAML)
    pose_only:=true             Wait for pose from /object_poses/<name> and exit (no robot/execute); validates pose path.
    
Fallback Behavior:
    - Tries PTP (fixed/joint space kinematics) first for each candidate
    - If PTP fails, tries LIN (linear Cartesian) as fallback
    - Tries different orientations (yaw angles) if first attempt fails
    - Tries different approach heights and grasp offsets
    - Up to 12 different solution attempts before giving up
"""

import sys
import os
import time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from ros2srrc_data.msg import Robpose
from nav_msgs.msg import Odometry

# Import the Pick class and yaw helper from pick_manual.py
sys.path.append(os.path.join(get_package_share_directory("ros2srrc_execution"), 'python'))
from pick_manual import Pick, PickConfig, _quat_rotate_yaw_deg


class ObjectPoseGetter(Node):
    """Get object pose from /object_poses/<name> topic"""
    
    def __init__(self, object_name):
        super().__init__('object_pose_getter')
        self.object_name = object_name
        self.pose_received = False
        self.object_pose = None
        
        # Subscribe to object pose topic
        topic_name = f"/object_poses/{object_name}"
        self.subscription = self.create_subscription(
            Odometry,
            topic_name,
            self.pose_callback,
            10
        )
        self.get_logger().info(f'Subscribed to {topic_name}')
    
    def pose_callback(self, msg):
        """Callback when object pose is received. Pose must be in MoveIt planning frame (world for UR5)."""
        if not self.pose_received:
            frame = msg.header.frame_id if msg.header.frame_id else "unknown"
            if frame != "world":
                self.get_logger().warn(
                    f'Pose frame_id is "{frame}" but MoveIt expects "world". '
                    "Robot may move to wrong location! Fix the publisher (e.g. hanoi_publish_pose / hanoi_pose_publisher) to use frame_id=\"world\"."
                )
            self.object_pose = Robpose()
            self.object_pose.x = msg.pose.pose.position.x
            self.object_pose.y = msg.pose.pose.position.y
            self.object_pose.z = msg.pose.pose.position.z
            self.object_pose.qx = msg.pose.pose.orientation.x
            self.object_pose.qy = msg.pose.pose.orientation.y
            self.object_pose.qz = msg.pose.pose.orientation.z
            self.object_pose.qw = msg.pose.pose.orientation.w
            self.pose_received = True
            self.get_logger().info(f'Received pose for {self.object_name} (frame={frame}): x={self.object_pose.x:.3f}, y={self.object_pose.y:.3f}, z={self.object_pose.z:.3f}')
    
    def get_pose(self, timeout=5.0):
        """Wait for pose with timeout"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose_received:
                return self.object_pose
        return None


def calculate_grasp_orientation(object_x, object_y, object_z, robot_base_x=0.0, robot_base_y=0.0, robot_base_z=0.0):
    """
    Calculate a good gripper orientation for grasping an object.

    FOR NOW, THIS IS A FIXED ORIENTATION THAT WORKS FOR UR5 + PARALLEL GRIPPER.
    
    Returns:
        quaternion (qx, qy, qz, qw) suitable for grasping
    """
    # Use proven fixed orientation that works for UR5 + parallel gripper
    # This is the same orientation used in the working YAML examples (ur5_pick_and_place.yaml)
    # It represents a good top-down grasping pose that is reachable for most positions
    return -0.5, 0.5, 0.5, 0.5


def calculate_gripper_close_percentage(cube_width, g_open=0.085, g_closed=0.00, margin=0.002, p_min=0.0, p_max=100.0):
    """
    Calculate the gripper close percentage based on cube width using geometry.
    
    For a parallel gripper, what matters is the cube width (the dimension between the fingers).
    
    Formula:
        w_eff = w - margin  (effective width with squeeze margin)
        p = 100 * ((g_open - w_eff) / (g_open - g_closed))
        p = max(p_min, min(p, p_max))  (clamp to valid range)
    
    Args:
        cube_width: Cube width in meters (dimension between fingers)
        g_open: Inner distance between fingers when fully open (meters, default: 0.085 for Robotiq 2F-85 = 85mm)
        g_closed: Inner distance when "100% closed" (meters, default: 0.00)
        margin: Squeeze margin in meters to ensure grip (default: 0.002 = 2mm)
        p_min: Minimum close percentage (default: 0.0)
        p_max: Maximum close percentage (default: 100.0)
    
    Returns:
        Close percentage (0-100) to use for gripper
    """
    # Apply squeeze margin
    w_eff = cube_width - margin
    
    # Calculate percentage
    if g_open == g_closed:
        # Avoid division by zero - if open and closed are the same, return max
        return p_max
    
    p = 100.0 * ((g_open - w_eff) / (g_open - g_closed))
    
    # Clamp to valid range
    p = max(p_min, min(p, p_max))
    
    return p


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
    Automated Pick action - only requires object name.
    """
    
    rclpy.init(args=args)
    
    print("==================================================")
    print("ROS 2 Sim-to-Real Robot Control: Automated Pick")
    print("==================================================")
    print("")
    
    # ===== PARSE REQUIRED ARGUMENT ===== #
    object_name = AssignArgument("object")
    
    if object_name is None:
        print("ERROR: Missing required argument: object")
        print("")
        print("Usage: ros2 run ros2srrc_execution pick.py object:=<object_name>")
        print("")
        print("Example:")
        print("  ros2 run ros2srrc_execution pick.py object:=cube1")
        print("")
        print("Optional arguments:")
        print("  robot:=ur5                  Robot name (default: ur5)")
        print("  ee_type:=ParallelGripper    End-effector type (default: ParallelGripper)")
        print("  ee_link:=EE_robotiq_2f85    End-effector link (default: EE_robotiq_2f85)")
        print("  approach_height:=0.22       Approach height in meters (default: 0.22, works for all cube sizes up to ~80mm)")
        print("  grasp_z_offset:=0.04        Grasp Z offset (default: 0.04, works for all cube sizes up to ~80mm)")
        print("  grasp_yaw_offset_deg:=<deg>  Extra yaw (deg) for grasp (default: 0)")
        print("  min_lift_height:=<value>    Minimum lift height above object in meters (default: None, uses approach_height)")
        print("  cube_size:=<value>          Cube size/width in meters (if provided, calculates gripper close % automatically)")
        print("  gripper_value:=50.0         Gripper close percentage (default: 50.0, ignored if cube_size provided)")
        print("  gripper_open:=0.085         Gripper open distance in meters (default: 0.085 = 85mm for Robotiq 2F-85)")
        print("  gripper_closed:=0.00        Gripper closed distance in meters (default: 0.00)")
        print("  gripper_margin:=0.002       Squeeze margin in meters (default: 0.002 = 2mm)")
        print("  pose_only:=true             Wait for pose and exit (no robot/execute); for pose-path validation.")
        print("")
        print("Closing program... BYE!")
        rclpy.shutdown()
        exit(1)
    
    # ===== PARSE OPTIONAL ARGUMENTS ===== #
    robot = AssignArgument("robot") or "ur5"
    ee_type = AssignArgument("ee_type") or "ParallelGripper"
    ee_link = AssignArgument("ee_link") or "EE_robotiq_2f85"
    
    # Defaults chosen to work for all cube sizes (50mm–80mm); higher values avoid INVALID_MOTION_PLAN on descend for large cubes
    approach_height = float(AssignArgument("approach_height") or "0.22")
    grasp_z_offset = float(AssignArgument("grasp_z_offset") or "0.04")
    
    # Parse min_lift_height (optional, None means use approach_height)
    min_lift_height_arg = AssignArgument("min_lift_height")
    min_lift_height = float(min_lift_height_arg) if min_lift_height_arg is not None else None
    
    # Parse cube_size and gripper parameters
    cube_size_arg = AssignArgument("cube_size")
    gripper_open = float(AssignArgument("gripper_open") or "0.085")
    gripper_closed = float(AssignArgument("gripper_closed") or "0.00")
    gripper_margin = float(AssignArgument("gripper_margin") or "0.002")
    default_gripper_value = float(AssignArgument("gripper_value") or "50.0")
    
    # Parse fallback parameters
    def _parse_bool(val, default_true=True):
        s = (AssignArgument(val) or ("true" if default_true else "false")).lower()
        return s in ("1", "true", "yes")
    
    fallback_enabled = _parse_bool("fallback_enabled", True)
    max_attempts = int(AssignArgument("max_attempts") or "12")
    pose_only = _parse_bool("pose_only", False)  # ROS-only test: receive pose and exit (no robot/plan/execute)
    
    # Grasp yaw offset: base orientation (-0.5,0.5,0.5,0.5) matches YAML and works for ParallelGripper (0 deg).
    # Override with grasp_yaw_offset_deg:=<deg> if needed (e.g. for HandE try 90, -90, 180).
    grasp_yaw_arg = AssignArgument("grasp_yaw_offset_deg")
    grasp_yaw_offset_deg = float(grasp_yaw_arg) if grasp_yaw_arg is not None else 0.0
    
    # Calculate gripper close percentage if cube_size is provided
    if cube_size_arg:
        cube_width = float(cube_size_arg)
        gripper_value = calculate_gripper_close_percentage(
            cube_width=cube_width,
            g_open=gripper_open,
            g_closed=gripper_closed,
            margin=gripper_margin
        )

    else:
        gripper_value = default_gripper_value
        print(f"[Pick]: Using default gripper close percentage: {gripper_value:.2f}%")
        print(f"  (Tip: Provide cube_size:=<value> to calculate automatically)")
        print("")
    
    print(f"Object: {object_name}")
    print(f"Robot: {robot}, EE Type: {ee_type}, EE Link: {ee_link}")
    print("")
    
    # ===== GET OBJECT POSE AUTOMATICALLY ===== #
    print("============================================================")
    print("Getting object pose automatically...")
    print("")
    
    pose_getter = ObjectPoseGetter(object_name)
    
    # Wait for pose
    print(f"[Pick]: Waiting for object pose from /object_poses/{object_name}...")
    object_pose = pose_getter.get_pose(timeout=5.0)
    
    if object_pose is None:
        print(f"ERROR: Could not get pose for object '{object_name}'")
        print(f"Make sure:")
        print(f"  1. Object '{object_name}' is spawned in Gazebo")
        print(f"  2. Topic /object_poses/{object_name} is publishing")
        print(f"  3. Object has a pose publisher plugin")
        print("")
        pose_getter.destroy_node()
        rclpy.shutdown()
        exit(1)
    
    print(f"[Pick]: Object pose retrieved successfully!")
    print(f"  Position: x={object_pose.x:.3f}, y={object_pose.y:.3f}, z={object_pose.z:.3f}")
    print(f"  Original orientation: qx={object_pose.qx:.3f}, qy={object_pose.qy:.3f}, qz={object_pose.qz:.3f}, qw={object_pose.qw:.3f}")
    
    if pose_only:
        print("")
        print("[Pick]: pose_only:=true -> Exiting after pose receipt (no robot/plan/execute).")
        print("  This validates the real-robot pose path: /object_poses/<name> -> pick.")
        pose_getter.destroy_node()
        rclpy.shutdown()
        exit(0)
    
    # IMPORTANT: Always use fixed top-down orientation for grasping, regardless of cube's current orientation
    # This ensures cubes are always grasped parallel to the table, even if they were placed at an angle
    # Calculate optimal gripper orientation for grasping (fixed top-down orientation)
    qx, qy, qz, qw = calculate_grasp_orientation(
        object_pose.x, object_pose.y, object_pose.z
    )
    # Rotate by grasp_yaw_offset_deg (0 for ParallelGripper = same as YAML; 90 for HandE so fingers face cube)
    if grasp_yaw_offset_deg != 0.0:
        qx, qy, qz, qw = _quat_rotate_yaw_deg(qx, qy, qz, qw, grasp_yaw_offset_deg)
    
    # Force fixed orientation - ignore cube's current orientation to prevent diagonal grasps
    object_pose.qx = qx
    object_pose.qy = qy
    object_pose.qz = qz
    object_pose.qw = qw
    
    print(f"  Using FIXED top-down grasp orientation (ignoring cube rotation): qx={qx:.3f}, qy={qy:.3f}, qz={qz:.3f}, qw={qw:.3f}")
    if grasp_yaw_offset_deg != 0.0:
        print(f"  Applied grasp_yaw_offset_deg={grasp_yaw_offset_deg:.1f}")
    print("")
    
    pose_getter.destroy_node()
    
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
    elif ee_type == "VacuumGripper":
        sys.path.append(PATH_EEGz)
        from vacuumGripper import vacuumGR  # type: ignore
        EEClient = vacuumGR([object_name], robot, ee_link)
        print("Loaded -> VacuumGripper.")
    elif ee_type in ("ParallelGripper", "robotiq_2f85", "RobotiqHandE/UR", "onrobot_2fg7"):
        sys.path.append(PATH)
        from endeffector.gripper_factory import create_gripper
        EEClient = create_gripper(ee_type, robot, ee_link, [object_name])
        print(f"Loaded -> {ee_type}.")
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
            super().__init__('pick_pose_checker')
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
    print("[Pick]: Checking robot pose topic availability...")
    timeout = time.time() + 3.0
    while time.time() < timeout and not pose_checker.pose_received:
        rclpy.spin_once(pose_checker, timeout_sec=0.1)
        if pose_checker.pose_received:
            break
    
    if pose_checker.pose_received:
        print("[Pick]: Robot pose topic is publishing.")
    else:
        print("[Pick]: Robot pose topic not detected (may not be required).")
    
    # Brief wait for MoveIt!2 planning scene (reduced from 2.0s for faster startup)
    print("[Pick]: Waiting for MoveIt!2 planning scene to initialize...")
    time.sleep(0.5)
    print("[Pick]: System ready!")
    print("")
    
    # Clean up pose checker
    pose_checker.destroy_node()
    
    # ===== CREATE CONFIG AND EXECUTE PICK ===== #
    print("============================================================")
    print("Executing Automated Pick action...")
    print("")
    
    # Explicitly enable fallbacks with proper configuration
    # This ensures the robot tries PTP (fixed kinematics) first, then falls back to different solutions
    config = {
        "approach_height": approach_height,
        "grasp_z_offset": grasp_z_offset,
        "gripper_value": gripper_value,
        "fallback_enabled": fallback_enabled,  # Use parsed value (default: True)
        "max_attempts": max_attempts,  # Use parsed value (default: 12)
        "yaw_candidates_deg": [0.0, 30.0, -30.0, 60.0, -60.0, 90.0],  # Try different orientations
        "approach_height_candidates": [0.22, 0.20, 0.18],  # First value = default; works for all cube sizes up to ~80mm
        "grasp_z_offset_candidates": [0.04, 0.03, 0.02],  # First value = default
        "prefer_lin_descend": False,  # Try PTP (fixed/joint space) first, then LIN fallback
        "min_lift_height": min_lift_height  # Minimum lift height to clear obstacles above
    }
    
    if min_lift_height is not None:
        print(f"[Pick]: Using minimum lift height: {min_lift_height:.3f}m (to clear obstacles above object)")
        print("")
    
    if fallback_enabled:
        print("[Pick]: Fallback mechanism ENABLED")
        print(f"  - Will try PTP (fixed kinematics) first for each candidate")
        print(f"  - Will try up to {max_attempts} different solutions")
        print(f"  - Yaw candidates: {config['yaw_candidates_deg']}")
        print(f"  - Approach height candidates: {config['approach_height_candidates']}")
        print(f"  - Grasp Z offset candidates: {config['grasp_z_offset_candidates']}")
        print("")
    else:
        print("[Pick]: Fallback mechanism DISABLED - will only try single attempt")
        print("")
    
    PickAction = Pick(RobotClient, EEClient, config)
    result = PickAction.execute(object_pose)
    
    # ===== RESULT ===== #
    print("============================================================")
    if result["Success"]:
        print("Automated Pick action completed SUCCESSFULLY!")
    else:
        print("Automated Pick action FAILED!")
    print(f"Message: {result['Message']}")
    print(f"Execution time: {result['ExecTime']}s")
    print("============================================================")
    
    rclpy.shutdown()
    exit(0 if result["Success"] else 1)


if __name__ == '__main__':
    main()
