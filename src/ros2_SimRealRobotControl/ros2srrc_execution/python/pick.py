#!/usr/bin/python3
import sys
import os
import time
import math
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from ros2srrc_data.msg import Robpose


def _quat_rotate_yaw_deg(qx, qy, qz, qw, deg):
    """
    Apply additional yaw rotation about world Z to a quaternion (qx, qy, qz, qw).
    Returns normalized (qx, qy, qz, qw).
    """
    half = math.radians(deg) * 0.5
    cz, sz = math.cos(half), math.sin(half)
    # q_z * q_obj (world Z then object; quat mult w=ab-cd-ed-fg, x=ac+bd+eg-fh, ...)
    nw = cz * qw - sz * qz
    nx = cz * qx - sz * qy
    ny = cz * qy + sz * qx
    nz = cz * qz + sz * qw
    n = math.sqrt(nw * nw + nx * nx + ny * ny + nz * nz)
    if n < 1e-12:
        return (qx, qy, qz, qw)
    return (nx / n, ny / n, nz / n, nw / n)


class PickConfig:
    """Configuration parameters for the Pick action."""
    
    def __init__(self, config_dict=None):
        """
        Initialize Pick configuration with defaults or from a dictionary.
        
        Args:
            config_dict: Optional dictionary with configuration overrides
        """
        # Default configuration values
        self.approach_height = 0.22      # Z offset above object for approach pose (meters; 0.22 works for all cube sizes up to ~80mm)
        self.grasp_z_offset = 0.04       # Z offset for grasp pose relative to object (meters)
        self.approach_speed = 1.0        # Speed for PTP approach move (0.0-1.0)
        self.descend_speed = 0.5         # Speed for LIN descend move (0.0-1.0)
        self.lift_speed = 0.6            # Speed for LIN lift move (0.0-1.0)
        self.gripper_value = 50.0        # Gripper close percentage (0-100)
        self.min_lift_height = None      # Minimum lift height above object (meters, None = use approach_height)
        
        # Fallback candidates
        self.fallback_enabled = True
        self.max_attempts = 12
        self.yaw_candidates_deg = [0.0, 30.0, -30.0, 60.0, -60.0, 90.0]
        self.approach_height_candidates = [0.22, 0.20, 0.18]
        self.grasp_z_offset_candidates = [0.04, 0.03, 0.02]
        self.prefer_lin_descend = False  # Changed: Try PTP (fixed position) first, then LIN fallback
        
        # Override defaults with provided config
        if config_dict:
            if "approach_height" in config_dict:
                self.approach_height = float(config_dict["approach_height"])
            if "grasp_z_offset" in config_dict:
                self.grasp_z_offset = float(config_dict["grasp_z_offset"])
            if "approach_speed" in config_dict:
                self.approach_speed = float(config_dict["approach_speed"])
            if "descend_speed" in config_dict:
                self.descend_speed = float(config_dict["descend_speed"])
            if "lift_speed" in config_dict:
                self.lift_speed = float(config_dict["lift_speed"])
            if "gripper_value" in config_dict:
                self.gripper_value = float(config_dict["gripper_value"])
            if "fallback_enabled" in config_dict:
                self.fallback_enabled = bool(config_dict["fallback_enabled"])
            if "max_attempts" in config_dict:
                self.max_attempts = int(config_dict["max_attempts"])
            if "yaw_candidates_deg" in config_dict:
                self.yaw_candidates_deg = [float(v) for v in config_dict["yaw_candidates_deg"]]
            if "approach_height_candidates" in config_dict:
                self.approach_height_candidates = [float(v) for v in config_dict["approach_height_candidates"]]
            if "grasp_z_offset_candidates" in config_dict:
                self.grasp_z_offset_candidates = [float(v) for v in config_dict["grasp_z_offset_candidates"]]
            if "prefer_lin_descend" in config_dict:
                self.prefer_lin_descend = bool(config_dict["prefer_lin_descend"])
            if "min_lift_height" in config_dict:
                val = config_dict["min_lift_height"]
                self.min_lift_height = float(val) if val is not None else None


class Pick:
    """
    High-level Pick action that encapsulates the full pick sequence.
    
    The Pick action performs:
    1. Move to approach pose (PTP movement above the object)
    2. Descend to grasp pose (LIN movement down to grasp position)
    3. Close gripper (grasp the object)
    4. Lift object (LIN movement back up to approach height)
    """
    
    def __init__(self, robot_client, gripper_client, config=None):
        """
        Initialize the Pick action.
        
        Args:
            robot_client: RBT instance for robot movements
            gripper_client: parallelGR or vacuumGR instance for gripper control
            config: Optional PickConfig or dict with configuration parameters
        """
        self.robot_client = robot_client
        self.gripper_client = gripper_client
        
        # Handle config initialization
        if config is None:
            self.config = PickConfig()
        elif isinstance(config, dict):
            self.config = PickConfig(config)
        else:
            self.config = config
    
    def _calculate_poses(self, object_pose, approach_height=None, grasp_z_offset=None, quat_override=None):
        """
        Calculate approach, grasp, and lift poses from object pose.
        
        Args:
            object_pose: Robpose message with object position (orientation is IGNORED - always uses fixed top-down)
            approach_height: optional override (meters)
            grasp_z_offset: optional override (meters)
            quat_override: optional (qx, qy, qz, qw) for orientation; if None, uses fixed top-down orientation
            
        Returns:
            tuple: (approach_pose, grasp_pose, lift_pose) as Robpose messages
        """
        ah = approach_height if approach_height is not None else self.config.approach_height
        gz = grasp_z_offset if grasp_z_offset is not None else self.config.grasp_z_offset
        if quat_override is not None:
            qx, qy, qz, qw = quat_override
        else:
            # Always use fixed top-down orientation, ignore object_pose orientation (may be diagonal)
            # This ensures consistent parallel grasps regardless of cube's rotation in Gazebo
            qx, qy, qz, qw = -0.5, 0.5, 0.5, 0.5  # Fixed top-down orientation

        # Approach pose: above the object
        approach_pose = Robpose()
        approach_pose.x = object_pose.x
        approach_pose.y = object_pose.y
        approach_pose.z = object_pose.z + ah
        approach_pose.qx, approach_pose.qy, approach_pose.qz, approach_pose.qw = qx, qy, qz, qw

        # Grasp pose: at grasp height (slightly above object center)
        grasp_pose = Robpose()
        grasp_pose.x = object_pose.x
        grasp_pose.y = object_pose.y
        grasp_pose.z = object_pose.z + gz
        grasp_pose.qx, grasp_pose.qy, grasp_pose.qz, grasp_pose.qw = qx, qy, qz, qw

        # Lift pose: use min_lift_height if specified, otherwise use approach_height
        # This ensures we lift high enough to clear obstacles above the object
        lift_height = self.config.min_lift_height if self.config.min_lift_height is not None else ah
        # Ensure lift height is at least as high as approach height
        lift_height = max(lift_height, ah)
        
        # Debug output for lift height calculation
        if self.config.min_lift_height is not None:
            print(f"[Pick]: Using min_lift_height: {self.config.min_lift_height:.3f}m (requested)")
            print(f"[Pick]: Approach height: {ah:.3f}m")
            print(f"[Pick]: Final lift height: {lift_height:.3f}m (relative to object center at z={object_pose.z:.3f}m)")
            print(f"[Pick]: Absolute lift Z position: {object_pose.z + lift_height:.3f}m")
        
        lift_pose = Robpose()
        lift_pose.x = object_pose.x
        lift_pose.y = object_pose.y
        lift_pose.z = object_pose.z + lift_height
        lift_pose.qx, lift_pose.qy, lift_pose.qz, lift_pose.qw = qx, qy, qz, qw

        return approach_pose, grasp_pose, lift_pose

    def _build_candidates(self, object_pose):
        """Build ordered list of fallback candidates (type_str, approach_height, grasp_z_offset, quat)."""
        ah = self.config.approach_height
        gz = self.config.grasp_z_offset
        # CRITICAL: Always use fixed top-down orientation for grasping, IGNORING object's current orientation
        # This ensures consistent, parallel grasps even if cube is rotated/diagonal in Gazebo
        # Fixed orientation: -0.5, 0.5, 0.5, 0.5 (top-down, parallel to table) - same as pick_auto.py
        # DO NOT use object_pose.qx/qy/qz/qw as it may contain diagonal orientation from Gazebo
        q = (-0.5, 0.5, 0.5, 0.5)  # Fixed top-down orientation
        yaws = self.config.yaw_candidates_deg
        ah_list = sorted(self.config.approach_height_candidates, reverse=True)
        gz_list = self.config.grasp_z_offset_candidates
        out = []

        # A) PRIMARY
        out.append({"type_str": "PRIMARY", "approach_height": ah, "grasp_z_offset": gz, "quat": q})

        # B) Same orient, alternative approach heights (higher first)
        for h in ah_list:
            if abs(h - ah) < 1e-6:
                continue
            out.append({"type_str": f"H={h:.2f}", "approach_height": h, "grasp_z_offset": gz, "quat": q})

        # C) Same heights, yaw candidates (exclude 0)
        for yaw in yaws:
            if abs(yaw) < 1e-6:
                continue
            qyaw = _quat_rotate_yaw_deg(*q, yaw)
            out.append({"type_str": f"YAW={int(yaw)}", "approach_height": ah, "grasp_z_offset": gz, "quat": qyaw})

        # D) Yaw + higher approach
        for yaw in yaws:
            if abs(yaw) < 1e-6:
                continue
            qyaw = _quat_rotate_yaw_deg(*q, yaw)
            for h in ah_list:
                if h <= ah + 1e-6:
                    continue
                out.append({"type_str": f"H={h:.2f}|YAW={int(yaw)}", "approach_height": h, "grasp_z_offset": gz, "quat": qyaw})

        # E) Vary grasp_z_offset (primary orient, primary height)
        for zoff in gz_list:
            if abs(zoff - gz) < 1e-6:
                continue
            out.append({"type_str": f"ZOFF={zoff:.2f}", "approach_height": ah, "grasp_z_offset": zoff, "quat": q})

        return out

    def execute_with_fallback(self, object_pose):
        """
        Execute pick with fallback candidates. Try each candidate (approach + descend);
        on first success, run grasp + lift and return. Stop after max_attempts.
        """
        T_start = time.time()
        RES = {"Success": False, "Message": "", "ExecTime": -1.0}

        print("[Pick]: Starting PICK sequence (fallback enabled)...")
        print(f"[Pick]: Object pose -> x: {object_pose.x:.3f}, y: {object_pose.y:.3f}, z: {object_pose.z:.3f}")
        print(f"[Pick]: Orientation -> qx: {object_pose.qx:.3f}, qy: {object_pose.qy:.3f}, qz: {object_pose.qz:.3f}, qw: {object_pose.qw:.3f}")
        print("")

        candidates = self._build_candidates(object_pose)[: self.config.max_attempts]
        N = len(candidates)

        for i, c in enumerate(candidates):
            attempt = i + 1
            approach_pose, grasp_pose, lift_pose = self._calculate_poses(
                object_pose,
                approach_height=c["approach_height"],
                grasp_z_offset=c["grasp_z_offset"],
                quat_override=c["quat"],
            )

            # Step 1: PTP to approach
            approach_result = self.robot_client.RobMove_EXECUTE("PTP", self.config.approach_speed, approach_pose)
            if not approach_result["Success"]:
                print(f"[Pick][Attempt {attempt}/{N}][type={c['type_str']}] Step 1 (Approach) failed: {approach_result['Message']}")
                continue

            # Step 2: Descend (PTP first - fixed position, then LIN fallback)
            # Try PTP (joint space/fixed position) first as it's more reliable
            descend_result = self.robot_client.RobMove_EXECUTE("PTP", self.config.descend_speed, grasp_pose)
            if not descend_result["Success"]:
                # Fallback to LIN if PTP fails
                print(f"[Pick][Attempt {attempt}/{N}][type={c['type_str']}] PTP descend failed, trying LIN fallback...")
                descend_result = self.robot_client.RobMove_EXECUTE("LIN", self.config.descend_speed, grasp_pose)
            
            if not descend_result["Success"]:
                print(f"[Pick][Attempt {attempt}/{N}][type={c['type_str']}] Step 2 (Descend) failed: {descend_result['Message']}")
                continue

            print(f"[Pick][Attempt {attempt}/{N}][type={c['type_str']}] Reached grasp, executing grasp+lift...")
            print("")

            # Step 3: Close gripper
            print(f"[Pick]: Step 3/4 - Closing GRIPPER (value: {self.config.gripper_value}%)...")
            if self.gripper_client is not None:
                if hasattr(self.gripper_client, "CLOSE"):
                    grasp_result = self.gripper_client.CLOSE(self.config.gripper_value)
                elif hasattr(self.gripper_client, "ACTIVATE"):
                    grasp_result = self.gripper_client.ACTIVATE()
                else:
                    RES["Message"] = "Pick FAILED at Step 3 (Grasp): Unknown gripper type"
                    T_end = time.time()
                    RES["ExecTime"] = round(T_end - T_start, 4)
                    return RES
                if not grasp_result["Success"]:
                    RES["Message"] = f"Pick FAILED at Step 3 (Grasp): {grasp_result['Message']}"
                    T_end = time.time()
                    RES["ExecTime"] = round(T_end - T_start, 4)
                    return RES
            else:
                print("[Pick]: WARNING - No gripper client provided, skipping grasp step.")
            print("[Pick]: Step 3/4 - Gripper closed successfully.")
            print("")

            # Step 4: Lift
            print(f"[Pick]: Step 4/4 - LIFTING object (LIN) to pose: x={lift_pose.x:.3f}, y={lift_pose.y:.3f}, z={lift_pose.z:.3f}")
            lift_result = self.robot_client.RobMove_EXECUTE("LIN", self.config.lift_speed, lift_pose)
            if not lift_result["Success"]:
                print(f"[Pick]: LIFT FAILED - Full result: Success={lift_result.get('Success')}, Message={lift_result.get('Message')}, ExecTime={lift_result.get('ExecTime')}")
                print(f"[Pick]: Target lift pose was: x={lift_pose.x:.3f}, y={lift_pose.y:.3f}, z={lift_pose.z:.3f}")
                RES["Message"] = f"Pick FAILED at Step 4 (Lift): {lift_result['Message']}"
                T_end = time.time()
                RES["ExecTime"] = round(T_end - T_start, 4)
                return RES
            print("[Pick]: Step 4/4 - Object lifted successfully.")
            print("")

            T_end = time.time()
            RES["Success"] = True
            RES["Message"] = "Pick sequence completed successfully."
            RES["ExecTime"] = round(T_end - T_start, 4)
            print(f"[Pick]: PICK SEQUENCE COMPLETE. Total time: {RES['ExecTime']}s")
            print("")
            return RES

        RES["Message"] = f"Pick FAILED: all {N} candidates failed."
        RES["ExecTime"] = round(time.time() - T_start, 4)
        print(f"[Pick]: {RES['Message']}")
        return RES

    def execute(self, object_pose):
        """
        Execute the full pick sequence.
        
        Args:
            object_pose: Robpose message with object position and gripper orientation
                        (x, y, z position of object; qx, qy, qz, qw for gripper orientation)
        
        Returns:
            dict: Result with keys:
                - "Success": bool indicating if pick was successful
                - "Message": str with result description
                - "ExecTime": float with total execution time in seconds
        """
        if self.config.fallback_enabled:
            return self.execute_with_fallback(object_pose)

        T_start = time.time()
        
        # Initialize result
        RES = {
            "Success": False,
            "Message": "",
            "ExecTime": -1.0
        }
        
        print("[Pick]: Starting PICK sequence...")
        print(f"[Pick]: Object pose -> x: {object_pose.x:.3f}, y: {object_pose.y:.3f}, z: {object_pose.z:.3f}")
        print(f"[Pick]: Orientation -> qx: {object_pose.qx:.3f}, qy: {object_pose.qy:.3f}, qz: {object_pose.qz:.3f}, qw: {object_pose.qw:.3f}")
        print("")
        
        # Calculate all poses
        approach_pose, grasp_pose, lift_pose = self._calculate_poses(object_pose)
        
        print(f"[Pick]: Approach height: {self.config.approach_height}m")
        print(f"[Pick]: Grasp Z offset: {self.config.grasp_z_offset}m")
        print("")
        
        # ===== STEP 1: Move to approach pose (PTP) ===== #
        print("[Pick]: Step 1/4 - Moving to APPROACH pose (PTP)...")
        approach_result = self.robot_client.RobMove_EXECUTE(
            "PTP",
            self.config.approach_speed,
            approach_pose
        )
        
        if not approach_result["Success"]:
            RES["Message"] = f"Pick FAILED at Step 1 (Approach): {approach_result['Message']}"
            print(f"[Pick]: {RES['Message']}")
            T_end = time.time()
            RES["ExecTime"] = round(T_end - T_start, 4)
            return RES
        
        print("[Pick]: Step 1/4 - Approach pose reached successfully.")
        print("")
        
        # ===== STEP 2: Descend to grasp pose (PTP first, then LIN fallback) ===== #
        print("[Pick]: Step 2/4 - Descending to GRASP pose (PTP - fixed position first)...")
        descend_result = self.robot_client.RobMove_EXECUTE(
            "PTP",
            self.config.descend_speed,
            grasp_pose
        )
        
        # If PTP fails, try LIN as fallback
        if not descend_result["Success"]:
            print("[Pick]: PTP descent failed, trying LIN as fallback...")
            descend_result = self.robot_client.RobMove_EXECUTE(
                "LIN",
                self.config.descend_speed,
                grasp_pose
            )
        
        if not descend_result["Success"]:
            RES["Message"] = f"Pick FAILED at Step 2 (Descend): {descend_result['Message']}"
            print(f"[Pick]: {RES['Message']}")
            T_end = time.time()
            RES["ExecTime"] = round(T_end - T_start, 4)
            return RES
        
        print("[Pick]: Step 2/4 - Grasp pose reached successfully.")
        print("")
        
        # ===== STEP 3: Close gripper ===== #
        print(f"[Pick]: Step 3/4 - Closing GRIPPER (value: {self.config.gripper_value}%)...")
        
        if self.gripper_client is not None:
            # Check if it's a parallel gripper (has CLOSE method with value)
            if hasattr(self.gripper_client, 'CLOSE'):
                grasp_result = self.gripper_client.CLOSE(self.config.gripper_value)
            # Check if it's a vacuum gripper (has ACTIVATE method)
            elif hasattr(self.gripper_client, 'ACTIVATE'):
                grasp_result = self.gripper_client.ACTIVATE()
            else:
                RES["Message"] = "Pick FAILED at Step 3 (Grasp): Unknown gripper type"
                print(f"[Pick]: {RES['Message']}")
                T_end = time.time()
                RES["ExecTime"] = round(T_end - T_start, 4)
                return RES
            
            if not grasp_result["Success"]:
                RES["Message"] = f"Pick FAILED at Step 3 (Grasp): {grasp_result['Message']}"
                print(f"[Pick]: {RES['Message']}")
                T_end = time.time()
                RES["ExecTime"] = round(T_end - T_start, 4)
                return RES
        else:
            print("[Pick]: WARNING - No gripper client provided, skipping grasp step.")
        
        print("[Pick]: Step 3/4 - Gripper closed successfully.")
        print("")
        
        # ===== STEP 4: Lift object (LIN) ===== #
        print(f"[Pick]: Step 4/4 - LIFTING object (LIN) to pose: x={lift_pose.x:.3f}, y={lift_pose.y:.3f}, z={lift_pose.z:.3f}")
        lift_result = self.robot_client.RobMove_EXECUTE(
            "LIN",
            self.config.lift_speed,
            lift_pose
        )
        
        if not lift_result["Success"]:
            print(f"[Pick]: LIFT FAILED - Full result: Success={lift_result.get('Success')}, Message={lift_result.get('Message')}, ExecTime={lift_result.get('ExecTime')}")
            print(f"[Pick]: Target lift pose was: x={lift_pose.x:.3f}, y={lift_pose.y:.3f}, z={lift_pose.z:.3f}")
            RES["Message"] = f"Pick FAILED at Step 4 (Lift): {lift_result['Message']}"
            print(f"[Pick]: {RES['Message']}")
            T_end = time.time()
            RES["ExecTime"] = round(T_end - T_start, 4)
            return RES
        
        print("[Pick]: Step 4/4 - Object lifted successfully.")
        print("")
        
        # ===== SUCCESS ===== #
        T_end = time.time()
        RES["Success"] = True
        RES["Message"] = "Pick sequence completed successfully."
        RES["ExecTime"] = round(T_end - T_start, 4)
        
        print(f"[Pick]: PICK SEQUENCE COMPLETE. Total time: {RES['ExecTime']}s")
        print("")
        
        return RES


# ========================================================================================= #
# ========================================= MAIN ========================================== #
# ========================================================================================= #

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
    Command-line interface for Pick action.
    
    Usage:
        ros2 run ros2srrc_execution pick.py x:=0.15 y:=0.48 z:=0.50 qx:=-0.5 qy:=0.5 qz:=0.5 qw:=0.5
    
    Optional arguments:
        robot:=ur5                  Robot name (default: ur5)
        ee_type:=ParallelGripper    End-effector type (default: ParallelGripper)
        ee_link:=EE_robotiq_2f85    End-effector link name (default: EE_robotiq_2f85)
        objects:=cube1              Comma-separated object names (default: cube1)
        approach_height:=0.15       Approach height in meters (default: 0.15)
        grasp_z_offset:=0.03        Grasp Z offset in meters (default: 0.03)
        gripper_value:=50.0         Gripper close percentage (default: 50.0)
    """
    
    rclpy.init(args=args)
    
    print("==================================================")
    print("ROS 2 Sim-to-Real Robot Control: Pick Action")
    print("==================================================")
    print("")
    
    # ===== PARSE REQUIRED ARGUMENTS ===== #
    x = AssignArgument("x")
    y = AssignArgument("y")
    z = AssignArgument("z")
    qx = AssignArgument("qx")
    qy = AssignArgument("qy")
    qz = AssignArgument("qz")
    qw = AssignArgument("qw")
    
    # Check required arguments
    missing_args = []
    if x is None: missing_args.append("x")
    if y is None: missing_args.append("y")
    if z is None: missing_args.append("z")
    if qx is None: missing_args.append("qx")
    if qy is None: missing_args.append("qy")
    if qz is None: missing_args.append("qz")
    if qw is None: missing_args.append("qw")
    
    if missing_args:
        print("ERROR: Missing required arguments: " + ", ".join(missing_args))
        print("")
        print("Usage: ros2 run ros2srrc_execution pick.py x:=<val> y:=<val> z:=<val> qx:=<val> qy:=<val> qz:=<val> qw:=<val>")
        print("")
        print("Example:")
        print("  ros2 run ros2srrc_execution pick.py x:=0.15 y:=0.48 z:=0.50 qx:=-0.5 qy:=0.5 qz:=0.5 qw:=0.5")
        print("")
        print("Optional arguments:")
        print("  robot:=ur5                  Robot name (default: ur5)")
        print("  ee_type:=ParallelGripper    End-effector type (default: ParallelGripper)")
        print("  ee_link:=EE_robotiq_2f85    End-effector link (default: EE_robotiq_2f85)")
        print("  objects:=cube1              Comma-separated object names (default: cube1)")
        print("  approach_height:=0.15       Approach height in meters (default: 0.15)")
        print("  grasp_z_offset:=0.03        Grasp Z offset in meters (default: 0.03)")
        print("  gripper_value:=50.0         Gripper close percentage (default: 50.0)")
        print("  fallback_enabled:=true      Enable fallback candidates (default: true)")
        print("  max_attempts:=12            Max fallback candidates to try (default: 12)")
        print("  yaw_candidates_deg:=...     Comma-separated yaw offsets (default: 0,30,-30,60,-60,90)")
        print("  approach_height_candidates:=0.15,0.18,0.20  Fallback heights (default)")
        print("  grasp_z_offset_candidates:=0.02,0.03,0.04   Fallback Z offsets (default)")
        print("  prefer_lin_descend:=true    LIN then PTP for descend (default: true)")
        print("")
        print("Closing program... BYE!")
        rclpy.shutdown()
        exit(1)
    
    # ===== PARSE OPTIONAL ARGUMENTS ===== #
    robot = AssignArgument("robot") or "ur5"
    ee_type = AssignArgument("ee_type") or "ParallelGripper"
    ee_link = AssignArgument("ee_link") or "EE_robotiq_2f85"
    objects_str = AssignArgument("objects") or "cube1"
    objects = [obj.strip() for obj in objects_str.split(",")]
    
    approach_height = float(AssignArgument("approach_height") or "0.15")
    grasp_z_offset = float(AssignArgument("grasp_z_offset") or "0.03")
    gripper_value = float(AssignArgument("gripper_value") or "50.0")

    def _parse_bool(val, default_true=True):
        s = (AssignArgument(val) or ("true" if default_true else "false")).lower()
        return s in ("1", "true", "yes")

    fallback_enabled = _parse_bool("fallback_enabled", True)
    max_attempts = int(AssignArgument("max_attempts") or "12")
    yaw_candidates_deg = [float(v) for v in (AssignArgument("yaw_candidates_deg") or "0,30,-30,60,-60,90").split(",")]
    approach_height_candidates = [float(v) for v in (AssignArgument("approach_height_candidates") or "0.15,0.18,0.20").split(",")]
    grasp_z_offset_candidates = [float(v) for v in (AssignArgument("grasp_z_offset_candidates") or "0.02,0.03,0.04").split(",")]
    prefer_lin_descend = _parse_bool("prefer_lin_descend", True)
    
    # Create object pose
    object_pose = Robpose()
    object_pose.x = float(x)
    object_pose.y = float(y)
    object_pose.z = float(z)
    object_pose.qx = float(qx)
    object_pose.qy = float(qy)
    object_pose.qz = float(qz)
    object_pose.qw = float(qw)
    
    print(f"Object Pose: x={object_pose.x}, y={object_pose.y}, z={object_pose.z}")
    print(f"Orientation: qx={object_pose.qx}, qy={object_pose.qy}, qz={object_pose.qz}, qw={object_pose.qw}")
    print(f"Robot: {robot}, EE Type: {ee_type}, EE Link: {ee_link}")
    print(f"Objects: {objects}")
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
        EEClient = parallelGR(objects, robot, ee_link)
        print("Loaded -> ParallelGripper.")
    elif ee_type == "VacuumGripper":
        sys.path.append(PATH_EEGz)
        from vacuumGripper import vacuumGR  # type: ignore
        EEClient = vacuumGR(objects, robot, ee_link)
        print("Loaded -> VacuumGripper.")
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
    
    # Additional wait to ensure MoveIt!2 planning scene is ready
    # This is especially important on first run
    print("[Pick]: Waiting for MoveIt!2 planning scene to initialize...")
    time.sleep(1.0)  # Give MoveIt!2 time to fully initialize planning scene
    print("[Pick]: System ready!")
    print("")
    
    # Clean up pose checker
    pose_checker.destroy_node()
    
    # ===== CREATE CONFIG AND EXECUTE PICK ===== #
    print("============================================================")
    print("Executing Pick action...")
    print("")
    
    config = {
        "approach_height": approach_height,
        "grasp_z_offset": grasp_z_offset,
        "gripper_value": gripper_value,
        "fallback_enabled": fallback_enabled,
        "max_attempts": max_attempts,
        "yaw_candidates_deg": yaw_candidates_deg,
        "approach_height_candidates": approach_height_candidates,
        "grasp_z_offset_candidates": grasp_z_offset_candidates,
        "prefer_lin_descend": prefer_lin_descend,
    }
    
    PickAction = Pick(RobotClient, EEClient, config)
    result = PickAction.execute(object_pose)
    
    # ===== RESULT ===== #
    print("============================================================")
    if result["Success"]:
        print("Pick action completed SUCCESSFULLY!")
    else:
        print("Pick action FAILED!")
    print(f"Message: {result['Message']}")
    print(f"Execution time: {result['ExecTime']}s")
    print("============================================================")
    
    rclpy.shutdown()
    exit(0 if result["Success"] else 1)


if __name__ == '__main__':
    main()
