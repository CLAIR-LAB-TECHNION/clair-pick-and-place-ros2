#!/usr/bin/python3

# ===== IMPORT REQUIRED COMPONENTS ===== #
import sys
import os
import time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from ros2srrc_data.msg import Robpose


class PlaceConfig:
    """Configuration parameters for the Place action."""
    
    def __init__(self, config_dict=None):
        """
        Initialize Place configuration with defaults or from a dictionary.
        
        Args:
            config_dict: Optional dictionary with configuration overrides
        """
        # Default configuration values
        self.approach_height = 0.2     # Z offset above place location for approach pose (meters)
        self.place_z_offset = 0.03       # Z offset for place pose relative to target (meters, positive = above target)
        self.approach_speed = 1.0        # Speed for PTP approach move (0.0-1.0)
        self.descend_speed = 0.5         # Speed for LIN descend move (0.0-1.0)
        self.retract_speed = 0.6         # Speed for LIN retract move (0.0-1.0)
        self.pre_open_lift_m = 0.0       # Optional upward move before opening (m); 0 = open at place height for precise drop
        
        # Override defaults with provided config
        if config_dict:
            if "approach_height" in config_dict:
                self.approach_height = float(config_dict["approach_height"])
            if "place_z_offset" in config_dict:
                self.place_z_offset = float(config_dict["place_z_offset"])
            if "approach_speed" in config_dict:
                self.approach_speed = float(config_dict["approach_speed"])
            if "descend_speed" in config_dict:
                self.descend_speed = float(config_dict["descend_speed"])
            if "retract_speed" in config_dict:
                self.retract_speed = float(config_dict["retract_speed"])
            if "pre_open_lift_m" in config_dict:
                self.pre_open_lift_m = float(config_dict["pre_open_lift_m"])


class Place:
    """
    High-level Place action that encapsulates the full place sequence.
    
    The Place action performs:
    1. Move to approach pose (PTP movement above the place location)
    2. Descend to place pose (LIN movement down to place position)
    3. Open gripper (release the object)
    4. Retract (LIN movement back up to approach height)
    """
    
    def __init__(self, robot_client, gripper_client, config=None):
        """
        Initialize the Place action.
        
        Args:
            robot_client: RBT instance for robot movements
            gripper_client: parallelGR or vacuumGR instance for gripper control
            config: Optional PlaceConfig or dict with configuration parameters
        """
        self.robot_client = robot_client
        self.gripper_client = gripper_client
        
        # Handle config initialization
        if config is None:
            self.config = PlaceConfig()
        elif isinstance(config, dict):
            self.config = PlaceConfig(config)
        else:
            self.config = config
    
    def _calculate_poses(self, place_pose):
        """
        Calculate approach, place, and retract poses from place location pose.
        
        Args:
            place_pose: Robpose message with place location position and orientation
            
        Returns:
            tuple: (approach_pose, place_pose_calc, retract_pose) as Robpose messages
        """
        # Approach pose: above the place location
        approach_pose = Robpose()
        approach_pose.x = place_pose.x
        approach_pose.y = place_pose.y
        approach_pose.z = place_pose.z + self.config.approach_height
        approach_pose.qx = place_pose.qx
        approach_pose.qy = place_pose.qy
        approach_pose.qz = place_pose.qz
        approach_pose.qw = place_pose.qw
        
        # Place pose: at place height (slightly above place location center)
        place_pose_calc = Robpose()
        place_pose_calc.x = place_pose.x
        place_pose_calc.y = place_pose.y
        place_pose_calc.z = place_pose.z + self.config.place_z_offset
        place_pose_calc.qx = place_pose.qx
        place_pose_calc.qy = place_pose.qy
        place_pose_calc.qz = place_pose.qz
        place_pose_calc.qw = place_pose.qw
        
        # Retract pose: same as approach pose (retract back up)
        retract_pose = Robpose()
        retract_pose.x = place_pose.x
        retract_pose.y = place_pose.y
        retract_pose.z = place_pose.z + self.config.approach_height
        retract_pose.qx = place_pose.qx
        retract_pose.qy = place_pose.qy
        retract_pose.qz = place_pose.qz
        retract_pose.qw = place_pose.qw
        
        return approach_pose, place_pose_calc, retract_pose
    
    def execute(self, place_pose):
        """
        Execute the full place sequence.
        
        Args:
            place_pose: Robpose message with place location position and gripper orientation
                       (x, y, z position of place location; qx, qy, qz, qw for gripper orientation)
        
        Returns:
            dict: Result with keys:
                - "Success": bool indicating if place was successful
                - "Message": str with result description
                - "ExecTime": float with total execution time in seconds
        """
        T_start = time.time()
        
        # Initialize result
        RES = {
            "Success": False,
            "Message": "",
            "ExecTime": -1.0
        }
        
        print("[Place]: Starting PLACE sequence...")
        print(f"[Place]: Place location pose -> x: {place_pose.x:.3f}, y: {place_pose.y:.3f}, z: {place_pose.z:.3f}")
        print(f"[Place]: Orientation -> qx: {place_pose.qx:.3f}, qy: {place_pose.qy:.3f}, qz: {place_pose.qz:.3f}, qw: {place_pose.qw:.3f}")
        print("")
        
        # Calculate all poses
        approach_pose, place_pose_calc, retract_pose = self._calculate_poses(place_pose)
        
        print(f"[Place]: Approach height: {self.config.approach_height}m")
        print(f"[Place]: Place Z offset: {self.config.place_z_offset}m")
        print("")
        
        # ===== STEP 1: Move to approach pose (PTP) ===== #
        print("[Place]: Step 1/4 - Moving to APPROACH pose (PTP)...")
        approach_result = self.robot_client.RobMove_EXECUTE(
            "PTP",
            self.config.approach_speed,
            approach_pose
        )
        
        if not approach_result["Success"]:
            RES["Message"] = f"Place FAILED at Step 1 (Approach): {approach_result['Message']}"
            print(f"[Place]: {RES['Message']}")
            T_end = time.time()
            RES["ExecTime"] = round(T_end - T_start, 4)
            return RES
        
        print("[Place]: Step 1/4 - Approach pose reached successfully.")
        print("")
        
        # ===== STEP 2: Descend to place pose (LIN) ===== #
        print("[Place]: Step 2/4 - Descending to PLACE pose (LIN)...")
        print("[Place]: NOTE: If planning fails, the attached object may be causing collision issues.")
        print("[Place]:      Try increasing approach_height or place_z_offset if this fails.")
        descend_result = self.robot_client.RobMove_EXECUTE(
            "LIN",
            self.config.descend_speed,
            place_pose_calc
        )
        
        # If LIN fails, try PTP as fallback (sometimes PTP works when LIN doesn't)
        if not descend_result["Success"]:
            print("[Place]: LIN descent failed, trying PTP as fallback...")
            descend_result = self.robot_client.RobMove_EXECUTE(
                "PTP",
                self.config.descend_speed,
                place_pose_calc
            )
        
        # If both LIN and PTP fail, try with a slightly higher z to avoid collision
        if not descend_result["Success"]:
            print("[Place]: Both LIN and PTP failed. Trying with slightly higher z to avoid collision...")
            # Try smaller increments first (1cm, then 2cm if needed)
            for offset in [0.015, 0.02]:
                place_pose_higher = Robpose()
                place_pose_higher.x = place_pose_calc.x
                place_pose_higher.y = place_pose_calc.y
                place_pose_higher.z = place_pose_calc.z + offset
                place_pose_higher.qx = place_pose_calc.qx
                place_pose_higher.qy = place_pose_calc.qy
                place_pose_higher.qz = place_pose_calc.qz
                place_pose_higher.qw = place_pose_calc.qw
                
                print(f"[Place]: Retrying at z={place_pose_higher.z:.3f} ({offset*100:.0f}mm higher)...")
                descend_result = self.robot_client.RobMove_EXECUTE(
                    "PTP",
                    self.config.descend_speed,
                    place_pose_higher
                )
                
                if descend_result["Success"]:
                    # We reached a higher position to avoid collision
                    # Update place_pose_calc to the higher position
                    place_pose_calc = place_pose_higher
                    print(f"[Place]: Successfully reached place pose at z={place_pose_higher.z:.3f}m to avoid collision.")
                    break
        
        if not descend_result["Success"]:
            RES["Message"] = f"Place FAILED at Step 2 (Descend): {descend_result['Message']}"
            print(f"[Place]: {RES['Message']}")
            # print(f"[Place]: Tried to reach place pose at: x={place_pose_calc.x:.3f}, y={place_pose_calc.y:.3f}, z={place_pose_calc.z:.3f}")  # DEBUG
            # print(f"[Place]: This pose might be:")  # DEBUG
            # print(f"[Place]:   1. Outside the robot's reachable workspace")  # DEBUG
            # print(f"[Place]:   2. In a kinematic singularity")  # DEBUG
            # print(f"[Place]:   3. Causing a collision with the environment")  # DEBUG
            # print(f"[Place]:   4. Unreachable with the current orientation")  # DEBUG
            # print(f"[Place]: Suggestions:")  # DEBUG
            # print(f"[Place]:   - Try a different location (x, y)")  # DEBUG
            # print(f"[Place]:   - Try a different z height")  # DEBUG
            # print(f"[Place]:   - Try adjusting place_z_offset (current: {self.config.place_z_offset}m)")  # DEBUG
            # print(f"[Place]:   - Check if the location is within the robot's workspace")  # DEBUG
            T_end = time.time()
            RES["ExecTime"] = round(T_end - T_start, 4)
            return RES
        
        print("[Place]: Step 2/4 - Place pose reached successfully.")
        
        # Check if we're at a higher z due to collision avoidance
        original_target_z = place_pose.z + self.config.place_z_offset
        if abs(place_pose_calc.z - original_target_z) > 0.01:
            print(f"[Place]: NOTE: Reached z={place_pose_calc.z:.3f}m (target was {original_target_z:.3f}m) due to collision avoidance.")
            print(f"[Place]:       Object will be placed at this height.")
        print("")
        
        # ===== Optional: small lift before open (reduces gripper-vs-cube collision) ===== #
        if self.config.pre_open_lift_m > 0.001:
            lift_pose = Robpose()
            lift_pose.x = place_pose_calc.x
            lift_pose.y = place_pose_calc.y
            lift_pose.z = place_pose_calc.z + self.config.pre_open_lift_m
            lift_pose.qx = place_pose_calc.qx
            lift_pose.qy = place_pose_calc.qy
            lift_pose.qz = place_pose_calc.qz
            lift_pose.qw = place_pose_calc.qw
            print(f"[Place]: Lifting {self.config.pre_open_lift_m*1000:.0f} mm before open...")
            lift_res = self.robot_client.RobMove_EXECUTE("LIN", self.config.descend_speed, lift_pose)
            if not lift_res["Success"]:
                lift_res = self.robot_client.RobMove_EXECUTE("PTP", self.config.descend_speed, lift_pose)
            if not lift_res["Success"]:
                RES["Message"] = f"Place FAILED at pre-open lift: {lift_res['Message']}"
                print(f"[Place]: {RES['Message']}")
                T_end = time.time()
                RES["ExecTime"] = round(T_end - T_start, 4)
                return RES
            print("")
        
        # ===== STEP 3: Open gripper ===== #
        print("[Place]: Step 3/4 - Opening GRIPPER (releasing object)...")
        
        if self.gripper_client is not None:
            # Check if it's a parallel gripper (has OPEN method)
            if hasattr(self.gripper_client, 'OPEN'):
                # For parallel gripper, we need to open without detaching again
                # Since we may have already detached, modify OPEN to skip detach if already done
                # Actually, OPEN will try to detach again, but that's okay - it will just fail silently
                release_result = self.gripper_client.OPEN()
            # Check if it's a vacuum gripper (has DEACTIVATE method)
            elif hasattr(self.gripper_client, 'DEACTIVATE'):
                release_result = self.gripper_client.DEACTIVATE()
            else:
                RES["Message"] = "Place FAILED at Step 3 (Release): Unknown gripper type"
                print(f"[Place]: {RES['Message']}")
                T_end = time.time()
                RES["ExecTime"] = round(T_end - T_start, 4)
                return RES
            
            if not release_result["Success"]:
                RES["Message"] = f"Place FAILED at Step 3 (Release): {release_result['Message']}"
                print(f"[Place]: {RES['Message']}")
                T_end = time.time()
                RES["ExecTime"] = round(T_end - T_start, 4)
                return RES
        else:
            print("[Place]: WARNING - No gripper client provided, skipping release step.")
        
        print("[Place]: Step 3/4 - Gripper opened successfully.")
        print("")
               
        
        # ===== STEP 4: Retract (LIN) ===== #
        print("[Place]: Step 4/4 - RETRACTING (LIN)...")
        retract_result = self.robot_client.RobMove_EXECUTE(
            "LIN",
            self.config.retract_speed,
            retract_pose
        )
        
        # If LIN retract fails, try PTP as fallback
        if not retract_result["Success"]:
            print("[Place]: LIN retract failed, trying PTP as fallback...")
            retract_result = self.robot_client.RobMove_EXECUTE(
                "PTP",
                self.config.retract_speed,
                retract_pose
            )
        
        if not retract_result["Success"]:
            # Retract failure is not critical - object was already placed successfully
            # The placed object might be blocking retraction, but the place operation succeeded
            print(f"[Place]: WARNING - Retract failed: {retract_result['Message']}")
            print("[Place]: Object was successfully placed, but robot could not retract.")
            print("[Place]: This may be due to the placed object blocking the path.")
            print("[Place]: The place operation is still considered successful.")
            print("")
        else:
            print("[Place]: Step 4/4 - Retracted successfully.")
            print("")
        
        # ===== SUCCESS ===== #
        # Place is successful if we reached step 3 (gripper opened)
        # Retract failure doesn't mean place failed
        T_end = time.time()
        RES["Success"] = True
        if retract_result["Success"]:
            RES["Message"] = "Place sequence completed successfully."
        else:
            RES["Message"] = "Place sequence completed (object placed, but retract had issues)."
        RES["ExecTime"] = round(T_end - T_start, 4)
        
        print(f"[Place]: PLACE SEQUENCE COMPLETE. Total time: {RES['ExecTime']}s")
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
    Command-line interface for Place action.
    
    Usage:
        ros2 run ros2srrc_execution place.py x:=0.15 y:=0.48 z:=0.50 qx:=-0.5 qy:=0.5 qz:=0.5 qw:=0.5
    
    Optional arguments:
        robot:=ur5                  Robot name (default: ur5)
        ee_type:=ParallelGripper    End-effector type (default: ParallelGripper)
        ee_link:=EE_robotiq_2f85    End-effector link name (default: EE_robotiq_2f85)
        objects:=cube1              Comma-separated object names (default: cube1)
        approach_height:=0.15       Approach height in meters (default: 0.15)
        place_z_offset:=0.03        Place Z offset in meters (default: 0.02)
    """
    
    rclpy.init(args=args)
    
    print("==================================================")
    print("ROS 2 Sim-to-Real Robot Control: Place Action")
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
        print("Usage: ros2 run ros2srrc_execution place.py x:=<val> y:=<val> z:=<val> qx:=<val> qy:=<val> qz:=<val> qw:=<val>")
        print("")
        print("Example:")
        print("  ros2 run ros2srrc_execution place.py x:=0.15 y:=0.48 z:=0.50 qx:=-0.5 qy:=0.5 qz:=0.5 qw:=0.5")
        print("")
        print("Optional arguments:")
        print("  robot:=ur5                  Robot name (default: ur5)")
        print("  ee_type:=ParallelGripper    End-effector type (default: ParallelGripper)")
        print("  ee_link:=EE_robotiq_2f85    End-effector link (default: EE_robotiq_2f85)")
        print("  objects:=cube1              Comma-separated object names (default: cube1)")
        print("  approach_height:=0.15       Approach height in meters (default: 0.15)")
        print("  place_z_offset:=0.03        Place Z offset in meters (default: 0.03, place slightly above target)")
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
    place_z_offset = float(AssignArgument("place_z_offset") or "0.03")
    
    # Create place location pose
    place_pose = Robpose()
    place_pose.x = float(x)
    place_pose.y = float(y)
    place_pose.z = float(z)
    place_pose.qx = float(qx)
    place_pose.qy = float(qy)
    place_pose.qz = float(qz)
    place_pose.qw = float(qw)
    
    print(f"Place Location Pose: x={place_pose.x}, y={place_pose.y}, z={place_pose.z}")
    print(f"Orientation: qx={place_pose.qx}, qy={place_pose.qy}, qz={place_pose.qz}, qw={place_pose.qw}")
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
            super().__init__('place_pose_checker')
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

    # Wait up to 2 seconds for pose topic (non-blocking if topic doesn't exist)
    print("[Place]: Checking robot pose topic availability...")
    timeout = time.time() + 2.0
    while time.time() < timeout and not pose_checker.pose_received:
        rclpy.spin_once(pose_checker, timeout_sec=0.1)
        if pose_checker.pose_received:
            break
    
    if pose_checker.pose_received:
        print("[Place]: Robot pose topic is publishing.")
    else:
        print("[Place]: Robot pose topic not detected (may not be required).")
    
    # Additional wait to ensure MoveIt!2 planning scene is ready
    # This is especially important on first run
    print("[Place]: Waiting for MoveIt!2 planning scene to initialize...")
    time.sleep(1.0)  # Give MoveIt!2 time to fully initialize planning scene
    print("[Place]: System ready!")
    print("")
    
    # Clean up pose checker
    pose_checker.destroy_node()
    
    # ===== CREATE CONFIG AND EXECUTE PLACE ===== #
    print("============================================================")
    print("Executing Place action...")
    print("")
    
    config = {
        "approach_height": approach_height,
        "place_z_offset": place_z_offset
    }
    
    PlaceAction = Place(RobotClient, EEClient, config)
    result = PlaceAction.execute(place_pose)
    
    # ===== RESULT ===== #
    print("============================================================")
    if result["Success"]:
        print("Place action completed SUCCESSFULLY!")
    else:
        print("Place action FAILED!")
    print(f"Message: {result['Message']}")
    print(f"Execution time: {result['ExecTime']}s")
    print("============================================================")
    
    rclpy.shutdown()
    exit(0 if result["Success"] else 1)


if __name__ == '__main__':
    main()
