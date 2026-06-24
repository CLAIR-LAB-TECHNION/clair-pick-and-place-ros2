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
    grasp_z_offset:=0.01        MoveIt grasp offset relative to object center (default 0.01 for 2FG7)
    grasp_extra_descend_m:=0.04 Extra MoveL down after MoveIt grasp (default 0.04 for 2FG7)
    min_lift_height:=<value>    Minimum lift height above object in meters (default: None, uses approach_height)
    cube_size:=<value>          Cube size/width in meters (if provided, calculates gripper close % automatically)
    gripper_value:=50.0         Gripper close percentage (default: 50.0, ignored if cube_size provided)
    gripper_open:=0.085         Gripper open distance in meters (default: 0.085 = 85mm for Robotiq 2F-85)
    gripper_closed:=0.00        Gripper closed distance in meters (default: 0.00)
    gripper_margin:=0.002       Squeeze margin in meters (default: 0.002 = 2mm)
    fallback_enabled:=true      Enable fallback mechanism (default: true)
    max_attempts:=12            Maximum fallback attempts (default: 12)
    grasp_yaw_offset_deg:=<deg> Extra yaw (deg) applied to grasp orientation (default: 0 = same as YAML)
    board_height:=0.02          Physical board on stand (m); used for 2FG7 Z auto-calibration
    z_is_pick_ref:=true|false   Force z as pick ref (0.84) or cube center (0.865); default auto
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory
from ros2srrc_data.msg import Robpose
from nav_msgs.msg import Odometry
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from shape_msgs.msg import SolidPrimitive

# Import the Pick class and yaw helper from pick_manual.py (gripper close % logic lives in PickConfig.get_gripper_close_percent)
sys.path.append(os.path.join(get_package_share_directory("ros2srrc_execution"), 'python'))
from pick_manual import Pick, PickConfig, _quat_rotate_yaw_deg

# Match hanoi_publish_pose.py (TRANSIENT_LOCAL); also compatible with Gazebo volatile publishers.
OBJECT_POSE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


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
            OBJECT_POSE_QOS,
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

    def get_pose_from_planning_scene(self, timeout_sec=2.0):
        """Get object center pose from MoveIt collision object (e.g. cube placed in RViz)."""
        client = self.create_client(GetPlanningScene, '/get_planning_scene')
        if not client.wait_for_service(timeout_sec=float(timeout_sec)):
            self.get_logger().warn('GetPlanningScene service not available.')
            return None
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        future = client.call_async(request)
        deadline = time.time() + timeout_sec
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            return None
        try:
            response = future.result()
            for obj in response.scene.world.collision_objects:
                if obj.id != self.object_name:
                    continue
                p = obj.pose.position
                q = obj.pose.orientation
                robpose = Robpose()
                robpose.x = float(p.x)
                robpose.y = float(p.y)
                robpose.z = float(p.z)
                robpose.qx = float(q.x)
                robpose.qy = float(q.y)
                robpose.qz = float(q.z)
                robpose.qw = float(q.w if q.w else 1.0)
                frame = obj.header.frame_id if obj.header.frame_id else "world"
                self.get_logger().info(
                    f'Pose from MoveIt planning scene ({frame}): '
                    f'x={robpose.x:.3f}, y={robpose.y:.3f}, z={robpose.z:.3f}'
                )
                return robpose
        except Exception as e:
            self.get_logger().warn(f'Failed to get pose from planning scene: {e}')
        return None

    def list_planning_scene_object_ids(self, timeout_sec=2.0):
        """Return collision object ids currently in MoveIt (for error hints)."""
        client = self.create_client(GetPlanningScene, '/get_planning_scene')
        if not client.wait_for_service(timeout_sec=float(timeout_sec)):
            return []
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        future = client.call_async(request)
        deadline = time.time() + timeout_sec
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            return []
        try:
            response = future.result()
            return [obj.id for obj in response.scene.world.collision_objects]
        except Exception:
            return []

    def get_object_size_from_planning_scene(self, timeout_sec=2.0):
        """
        Get object size (meters) from MoveIt planning scene if the object is a collision object.
        Returns float size (cube side or box first dimension), or None if not found or no geometry.
        """
        client = self.create_client(GetPlanningScene, '/get_planning_scene')
        if not client.wait_for_service(timeout_sec=float(timeout_sec)):
            self.get_logger().warn('GetPlanningScene service not available, cannot auto-detect object size.')
            return None
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        future = client.call_async(request)
        deadline = time.time() + timeout_sec
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            return None
        try:
            response = future.result()
            for obj in response.scene.world.collision_objects:
                if obj.id != self.object_name:
                    continue
                if not obj.primitives:
                    return None
                prim = obj.primitives[0]
                if prim.type == SolidPrimitive.BOX and len(prim.dimensions) >= 3:
                    # Cube: use first dimension; box: use min as "grasp width"
                    return float(min(prim.dimensions[0], prim.dimensions[1], prim.dimensions[2]))
                if prim.type == SolidPrimitive.SPHERE and len(prim.dimensions) >= 1:
                    return float(prim.dimensions[0] * 2.0)  # diameter as "size"
                return None
        except Exception as e:
            self.get_logger().warn(f'Failed to get object size from planning scene: {e}')
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
        print("  grasp_z_offset:=0.01        MoveIt grasp offset (object center + offset; default 0.01)")
        print("  grasp_extra_descend_m:=0.04 Extra MoveL down after MoveIt grasp (default 0.04 for onrobot_2fg7)")
        print("  grasp_yaw_offset_deg:=<deg>  Extra yaw (deg) for grasp (default: 0)")
        print("  min_lift_height:=<value>    Minimum lift height above object in meters (default: None, uses approach_height)")
        print("  cube_size:=<value>          Cube size/width in meters (if provided, calculates gripper close % automatically)")
        print("  gripper_value:=50.0         Gripper close percentage (default: 50.0, ignored if cube_size provided)")
        print("  gripper_open:=0.085         Gripper open distance in meters (default: 0.085 = 85mm for Robotiq 2F-85)")
        print("  gripper_closed:=0.00        Gripper closed distance in meters (default: 0.00)")
        print("  gripper_margin:=0.002       Squeeze margin in meters (default: 0.002 = 2mm)")
        print("  x:=<m> y:=<m> z:=<m>        Object pose (world frame); skips topic/MoveIt lookup")
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
    # MoveIt grasp Z (must be IK-reachable). Real cube is lower — use grasp_extra_descend_m for 2FG7.
    _is_2fg7 = ee_type == "onrobot_2fg7"
    grasp_z_arg = AssignArgument("grasp_z_offset")
    extra_descend_arg = AssignArgument("grasp_extra_descend_m")
    grasp_z_offset = float(grasp_z_arg if grasp_z_arg is not None else ("0.01" if _is_2fg7 else "0.01"))
    grasp_extra_descend_m = float(
        extra_descend_arg if extra_descend_arg is not None else ("0.04" if _is_2fg7 else "0.0")
    )
    
    # Parse min_lift_height (optional, None means use approach_height)
    min_lift_height_arg = AssignArgument("min_lift_height")
    min_lift_height = float(min_lift_height_arg) if min_lift_height_arg is not None else None
    min_approach_z_arg = AssignArgument("min_approach_z")
    min_approach_z = float(min_approach_z_arg) if min_approach_z_arg is not None else None
    transit_center_x_arg = AssignArgument("transit_via_center_x")
    transit_via_center_x = float(transit_center_x_arg) if transit_center_x_arg is not None else None
    
    # Parse cube_size and gripper parameters
    cube_size_arg = AssignArgument("cube_size")
    # Robotiq 2F-85 sim default 85 mm; OnRobot 2FG7 opens to ~110 mm — must match for correct gap.
    _default_gripper_open = "0.110" if ee_type == "onrobot_2fg7" else "0.085"
    gripper_open = float(AssignArgument("gripper_open") or _default_gripper_open)
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

    # Optional explicit pose (world frame, meters) — real robot when RViz cube / topic unavailable
    x_arg = AssignArgument("x")
    y_arg = AssignArgument("y")
    z_arg = AssignArgument("z")
    explicit_pose = None
    if x_arg is not None and y_arg is not None and z_arg is not None:
        explicit_pose = (float(x_arg), float(y_arg), float(z_arg))
    elif any(a is not None for a in (x_arg, y_arg, z_arg)):
        print("ERROR: Provide all of x:= y:= z:= together, or none.")
        rclpy.shutdown()
        exit(1)

    support_cubes_arg = AssignArgument("support_cubes")
    support_cube_names = [
        s.strip() for s in support_cubes_arg.split(",") if s.strip()
    ] if support_cubes_arg else []
    support_sizes_arg = AssignArgument("support_cube_sizes")
    support_cube_sizes = {}
    if support_sizes_arg:
        for part in support_sizes_arg.split(","):
            if ":" in part:
                k, v = part.split(":", 1)
                support_cube_sizes[k.strip()] = float(v.strip())
    
    # Grasp yaw offset: base orientation (-0.5,0.5,0.5,0.5) matches YAML and works for ParallelGripper (0 deg).
    # Override with grasp_yaw_offset_deg:=<deg> if needed (e.g. for HandE try 90, -90, 180).
    grasp_yaw_arg = AssignArgument("grasp_yaw_offset_deg")
    grasp_yaw_offset_deg = float(grasp_yaw_arg) if grasp_yaw_arg is not None else 0.0
    
    DEFAULT_CUBE_SIZE = 0.05   # meters; fallback when cube_size not provided and not in planning scene
    print(f"Object: {object_name}")
    print(f"Robot: {robot}, EE Type: {ee_type}, EE Link: {ee_link}")
    print("")
    
    # ===== GET OBJECT POSE ===== #
    print("============================================================")
    print("Getting object pose automatically...")
    print("")

    object_pose = None
    pose_getter = None

    if explicit_pose is not None:
        object_pose = Robpose()
        object_pose.x, object_pose.y, object_pose.z = explicit_pose
        object_pose.qx = 0.0
        object_pose.qy = 0.0
        object_pose.qz = 0.0
        object_pose.qw = 1.0
        print(f"[Pick]: Using explicit pose x:={object_pose.x}, y:={object_pose.y}, z:={object_pose.z}")
    else:
        pose_getter = ObjectPoseGetter(object_name)

        print(f"[Pick]: Waiting for object pose from /object_poses/{object_name}...")
        object_pose = pose_getter.get_pose(timeout=5.0)

        if object_pose is None:
            print(f"[Pick]: No pose on /object_poses/{object_name} — trying MoveIt planning scene...")
            object_pose = pose_getter.get_pose_from_planning_scene(timeout_sec=3.0)

        if object_pose is None:
            scene_ids = pose_getter.list_planning_scene_object_ids(timeout_sec=2.0)
            print(f"ERROR: Could not get pose for object '{object_name}'")
            if scene_ids:
                print(f"  MoveIt scene has: {scene_ids} (name must match object:={object_name})")
            else:
                print("  MoveIt planning scene is empty (RViz objects are lost after bringup restart).")
            print("Fix options:")
            print("  1. Pass pose directly:  pick.py object:=cube1 x:=0.15 y:=-0.12 z:=0.84 ee_type:=onrobot_2fg7 ...")
            print("  2. Re-add cube in RViz Objects tab (name cube1) and Publish")
            print("  3. SpawnObjectMoveIt.py --moveit-only ... OR hanoi_publish_pose.py (keep running)")
            print("  4. pick_manual.py with x/y/z/q")
            print("")
            pose_getter.destroy_node()
            rclpy.shutdown()
            exit(1)
    
    print(f"[Pick]: Object pose retrieved successfully!")
    print(f"  Position: x={object_pose.x:.3f}, y={object_pose.y:.3f}, z={object_pose.z:.3f}")
    print(f"  Original orientation: qx={object_pose.qx:.3f}, qy={object_pose.qy:.3f}, qz={object_pose.qz:.3f}, qw={object_pose.qw:.3f}")
    
    # Resolve cube size: from arg, or from MoveIt planning scene, or default (same gripper logic as Hanoi)
    if cube_size_arg:
        cube_size_for_config = float(cube_size_arg)
        if _is_2fg7:
            print(f"[Pick]: cube_size:={cube_size_arg} -> 2FG7 full close (gap=0 mm).")
        else:
            print(f"[Pick]: cube_size:={cube_size_arg} -> gripper close % from width (same logic as Hanoi).")
    else:
        scene_size = pose_getter.get_object_size_from_planning_scene(timeout_sec=2.0) if pose_getter else None
        if scene_size is not None:
            cube_size_for_config = scene_size
            print(f"[Pick]: Object size from MoveIt planning scene: {cube_size_for_config:.3f}m -> gripper close % from width (same logic as Hanoi).")
        else:
            cube_size_for_config = DEFAULT_CUBE_SIZE
            print(f"[Pick]: Object not in planning scene (or no geometry) -> using default width {DEFAULT_CUBE_SIZE}m; gripper close % from width (same logic as Hanoi).")
            print(f"  (Tip: Pass cube_size:=<value> or ensure object is in MoveIt scene to auto-detect size)")
    print("")

    # UR5+2FG7: same Z calibration as Hanoi when offsets not passed explicitly
    if _is_2fg7 and grasp_z_arg is None and extra_descend_arg is None:
        from real_ur5_2fg7_motion import (
            REAL_DEFAULT_BOARD_HEIGHT_M,
            compute_real_ee_motion_params,
            infer_pick_ref_and_center,
        )
        board_height_arg = AssignArgument("board_height")
        board_height = float(
            board_height_arg if board_height_arg is not None else REAL_DEFAULT_BOARD_HEIGHT_M
        )
        z_is_pick_ref_raw = AssignArgument("z_is_pick_ref")
        z_is_pick_ref = None
        if z_is_pick_ref_raw is not None:
            z_is_pick_ref = z_is_pick_ref_raw.strip().lower() in ("1", "true", "yes")
        z_pick_ref, z_center = infer_pick_ref_and_center(
            object_pose.z, cube_size_for_config, board_height, z_is_pick_ref
        )
        grasp_z_offset, grasp_extra_descend_m, z_ee = compute_real_ee_motion_params(
            z_pick_ref, z_center
        )
        object_pose.z = z_pick_ref
        z_moveit = z_pick_ref + grasp_z_offset
        print("[Pick]: 2FG7 auto Z calibration (same as Hanoi):")
        print(f"  pick_ref Z={z_pick_ref:.3f}m, center Z={z_center:.3f}m, board={board_height:.3f}m")
        print(
            f"  MoveIt grasp Z={z_moveit:.3f}m, EE target Z={z_ee:.3f}m, "
            f"grasp_z_offset={grasp_z_offset:.3f}m, extra_descend={grasp_extra_descend_m * 1000:.0f}mm"
        )
        print("")
    
    if pose_only:
        print("")
        print("[Pick]: pose_only:=true -> Exiting after pose receipt (no robot/plan/execute).")
        print("  This validates the real-robot pose path: /object_poses/<name> -> pick.")
        if pose_getter is not None:
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
    
    if pose_getter is not None:
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
    elif ee_type in ("ParallelGripper", "onrobot_ros2", "onrobot_2fg7"):
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
    #time.sleep(0.5)
    print("[Pick]: System ready!")
    print("")
    
    # Clean up pose checker
    pose_checker.destroy_node()
    
    # ===== CREATE CONFIG AND EXECUTE PICK ===== #
    print("============================================================")
    print("Executing Automated Pick action...")
    print(f"[Pick]: Effective config — grasp_z_offset={grasp_z_offset:.3f}m, grasp_extra_descend_m={grasp_extra_descend_m:.3f}m")
    print("")
    
    # Config: always pass cube_size so gripper close % is computed from width (same logic as Hanoi)
    config = {
        "approach_height": approach_height,
        "grasp_z_offset": grasp_z_offset,
        "grasp_extra_descend_m": grasp_extra_descend_m,
        "fallback_enabled": fallback_enabled,
        "max_attempts": max_attempts,
        "yaw_candidates_deg": [0.0, 30.0, -30.0, 60.0, -60.0, 90.0],
        "approach_height_candidates": [0.22, 0.20, 0.18],
        "grasp_z_offset_candidates": [0.02],
        "prefer_lin_descend": False,
        "min_lift_height": min_lift_height,
        "gripper_open": gripper_open,
        "gripper_closed": gripper_closed,
        "gripper_margin": gripper_margin,
        "cube_size": cube_size_for_config,
        "gripper_close_full": _is_2fg7,
        "object_name": object_name,
        "support_cube_names": support_cube_names,
        "support_cube_sizes": support_cube_sizes,
    }
    if min_approach_z is not None:
        config["min_approach_z"] = min_approach_z
    if transit_via_center_x is not None:
        config["transit_via_center_x"] = transit_via_center_x
    
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
