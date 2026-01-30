#!/usr/bin/python3

# parallelGripper.py
# This CLIENT operates any parallelGripper (which could be operated by MoveG), and checks for any potential
# attachments to any of the objects within the robot's workspace in Gazebo:

# System functions and classes:
import sys, os, time
# Required to include ROS2 and its components:
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
# Import LinkAttacher (ROS2 SRV):
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
# Import ROS2 messages:
from std_msgs.msg import String
from ros2srrc_data.msg import Action
from ros2srrc_data.msg import Robpose
from objectpose_msgs.msg import ObjectPose
from nav_msgs.msg import Odometry
# Import MoveIt messages for planning scene management:
from moveit_msgs.msg import CollisionObject, PlanningScene

# Import -> RobotClient for MoveG action execution:
PATH = os.path.join(get_package_share_directory("ros2srrc_execution"), 'python', 'robot')
sys.path.append(PATH)
from robot import RBT

# GLOBAL VARIABLE -> OBJECTS:
OBJECTS = []

# GLOBAL VARIABLE -> EEPose:
EEPose = Robpose()

# GLOBAL VARIABLE -> AttachCheck:
from dataclasses import dataclass
@dataclass
class AttDetCHECK:                     # Define -> Data class.
    ATTACHED: bool
    NAME: String
AttachCheck = AttDetCHECK(False,"")    # Initialise variable.

# =============================================================================== #
# ObjectPose SUBSCRIBER:
# Modified to subscribe to /object_poses/<name> (nav_msgs/Odometry) which is what
# the gazebo_ros_p3d plugin publishes, instead of /<name>/ObjectPose which doesn't exist

class ObjPOSE(Node):

    def __init__(self, ObjectLIST):

        super().__init__("ros2srrc_EEGz_ObjectPose_Subscriber")

        self.subLIST = []
        self.objectNames = ObjectLIST  # Store object names for callback matching

        for x in ObjectLIST:
            # Subscribe to /object_poses/<name> topic (published by gazebo_ros_p3d plugin)
            TopicName = "/object_poses/" + x
            self.subLIST.append(self.create_subscription(Odometry, TopicName, lambda msg, name=x: self.CALLBACK_FN(msg, name), 10))

    def CALLBACK_FN(self, odom_msg, object_name):

        global OBJECTS

        for x in OBJECTS:
            if x.objectname == object_name:
                # Extract position from Odometry message
                x.x = odom_msg.pose.pose.position.x
                x.y = odom_msg.pose.pose.position.y
                x.z = odom_msg.pose.pose.position.z
                x.qx = odom_msg.pose.pose.orientation.x
                x.qy = odom_msg.pose.pose.orientation.y
                x.qz = odom_msg.pose.pose.orientation.z
                x.qw = odom_msg.pose.pose.orientation.w

    def getOBJECTS(self):

        global OBJECTS

        T = time.time() + 0.50
        while time.time() < T:
            rclpy.spin_once(self, timeout_sec=0.50)
        
        return(OBJECTS)
    
# =============================================================================== #
# Robot(EE) Pose SUBSCRIBER:

class eePOSE(Node):

    def __init__(self):

        super().__init__("ros2srrc_EEGz_eePose_Subscriber")
        self.SUB = self.create_subscription(Robpose, "/Robpose", self.CALLBACK_FN, 10)

    def CALLBACK_FN(self, POSE):

        global EEPose

        EEPose.x = POSE.x
        EEPose.y = POSE.y
        EEPose.z = POSE.z
        EEPose.qx = POSE.qx
        EEPose.qy = POSE.qy
        EEPose.qz = POSE.qz
        EEPose.qw = POSE.qw

    def getEEPose(self):

        global EEPose

        T = time.time() + 0.50
        while time.time() < T:
            rclpy.spin_once(self)
        
        return(EEPose)
    
# =============================================================================== #
# LinkAttacher SERVICE CLIENT:

class LinkAttacher_Client(Node):

    def __init__(self, ROBOT, EE):

        super().__init__("ros2srrc_EEGz_LinkAttacher_Client")

        self.AttachClient = self.create_client(AttachLink, "/ATTACHLINK")
        self.DetachClient = self.create_client(DetachLink, "/DETACHLINK")

        print("[CLIENT - parallelGripper.py]: Initialising /ATTACHLINK and /DETACHLINK ROS 2 Service Clients.")

        while not self.AttachClient.wait_for_service(timeout_sec=1.0): 
            print("[CLIENT - parallelGripper.py]: /ATTACHLINK ROS2 Service not still available, waiting...")
        print("[CLIENT - parallelGripper.py]: /ATTACHLINK ROS2 Service ready.")
        while not self.DetachClient.wait_for_service(timeout_sec=1.0): 
            print("[CLIENT - parallelGripper.py]: /DETACHLINK ROS2 Service not still available, waiting...")
        print("[CLIENT - parallelGripper.py]: /DETACHLINK ROS2 Service ready.")

        print("")

        self.AttachRequest = AttachLink.Request()
        self.DetachRequest = DetachLink.Request()

        self.ROBOT = ROBOT
        self.EE = EE

    def ATTACHService(self, NAME):

        self.AttachRequest.model1_name = self.ROBOT
        self.AttachRequest.link1_name = self.EE
        self.AttachRequest.model2_name = NAME
        self.AttachRequest.link2_name = NAME

        self.AttachFuture = self.AttachClient.call_async(self.AttachRequest)

    def DETACHService(self, NAME):

        self.DetachRequest.model1_name = self.ROBOT
        self.DetachRequest.link1_name = self.EE
        self.DetachRequest.model2_name = NAME
        self.DetachRequest.link2_name = NAME

        self.DetachFuture = self.DetachClient.call_async(self.DetachRequest)

class LinkAttacher():

    def __init__(self, ROBOT, EE):
        self.CLIENT = LinkAttacher_Client(ROBOT, EE)

    def ATTACH(self, NAME):
        """Attach object to gripper. Timeout so we don't hang if /ATTACHLINK doesn't respond."""
        global AttachCheck

        self.CLIENT.ATTACHService(NAME)

        ATTACH_TIMEOUT_S = 5.0
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < ATTACH_TIMEOUT_S:
            rclpy.spin_once(self.CLIENT, timeout_sec=0.1)
            if self.CLIENT.AttachFuture.done():
                try:
                    AttachRES = self.CLIENT.AttachFuture.result()
                except Exception as exc:
                    print("[CLIENT - parallelGripper.py]: /ATTACHLINK Service call failed -> " + str(exc))
                    print("")
                    return(False)
                else:
                    if (AttachRES.success):
                        print("[CLIENT - parallelGripper.py]: /ATTACHLINK successful -> " + str(AttachRES.message))
                        print("")
                        AttachCheck.ATTACHED = True
                        AttachCheck.NAME = NAME
                        return(True)
                    else:
                        print("[CLIENT - parallelGripper.py]: /ATTACHLINK unuccessful -> " + str(AttachRES.message))
                        print("")
                        return(False)

        print("[CLIENT - parallelGripper.py]: /ATTACHLINK timed out after %.1fs." % ATTACH_TIMEOUT_S)
        print("")
        return(False)
                    
    def DETACH(self, NAME):
        """Detach object from gripper. Timeout so we don't hang if /DETACHLINK doesn't respond."""
        global AttachCheck

        self.CLIENT.DETACHService(NAME)

        DETACH_TIMEOUT_S = 5.0
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < DETACH_TIMEOUT_S:
            rclpy.spin_once(self.CLIENT, timeout_sec=0.1)
            if self.CLIENT.DetachFuture.done():
                try:
                    DetachRES = self.CLIENT.DetachFuture.result()
                except Exception as exc:
                    print("[CLIENT - parallelGripper.py]: /DETACHLINK Service call failed -> " + str(exc))
                    print("")
                    return(False)
                else:
                    if (DetachRES.success):
                        print("[CLIENT - parallelGripper.py]: /DETACHLINK successful -> " + str(DetachRES.message))
                        print("")
                        AttachCheck.ATTACHED = False
                        AttachCheck.NAME = ""
                        return(True)
                    else:
                        print("[CLIENT - parallelGripper.py]: /DETACHLINK unuccessful -> " + str(DetachRES.message))
                        print("")
                        return(False)

        print("[CLIENT - parallelGripper.py]: /DETACHLINK timed out after %.1fs; continuing to open gripper." % DETACH_TIMEOUT_S)
        print("")
        AttachCheck.ATTACHED = False
        AttachCheck.NAME = ""
        return(False)
                    
# =============================================================================== #
# MoveIt Planning Scene Manager:
# Removes objects from MoveIt's collision scene after attachment to prevent
# collision errors when closing the gripper

class MoveItSceneManager(Node):
    """Manages objects in MoveIt's planning scene"""
    
    def __init__(self):
        super().__init__('moveit_scene_manager')
        
        # MoveIt planning scene publisher
        self.planning_scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10)
        
        # Also publish to collision_object topic
        self.collision_object_pub = self.create_publisher(
            CollisionObject, '/collision_object', 10)
        
        # Service client to get planning scene
        from moveit_msgs.srv import GetPlanningScene
        self.get_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
    
    def remove_from_moveit(self, object_name):
        """Remove object from MoveIt planning scene"""
        # Ensure publishers are ready
        rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(0.1)  # Give time for publishers to connect
        
        # print(f"[CLIENT - parallelGripper.py]: Removing {object_name} from MoveIt planning scene...")  # DEBUG
        
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = object_name
        collision_object.operation = CollisionObject.REMOVE
        
        # Publish to collision_object topic
        self.collision_object_pub.publish(collision_object)
        
        # Also publish via planning scene
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        self.planning_scene_pub.publish(planning_scene)
        
        # Give time for message to be processed
        rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(0.3)  # Give MoveIt time to process the removal (increased for stacked objects)
        
        # print(f"[CLIENT - parallelGripper.py]: MoveIt: Removed {object_name} from collision objects")  # DEBUG
        return True
    
    def add_to_moveit(self, object_name, pose, size=0.05):
        """Add object back to MoveIt planning scene (for placing)"""
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose as GeometryPose
        
        self.get_logger().info(f'Adding {object_name} back to MoveIt planning scene...')
        
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = object_name
        
        # Create box primitive
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size, size, size]
        
        # Set pose
        geom_pose = GeometryPose()
        geom_pose.position.x = pose.x
        geom_pose.position.y = pose.y
        geom_pose.position.z = pose.z
        geom_pose.orientation.w = 1.0
        
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(geom_pose)
        collision_object.operation = CollisionObject.ADD
        
        # Publish to collision_object topic
        self.collision_object_pub.publish(collision_object)
        
        # Also publish via planning scene
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        self.planning_scene_pub.publish(planning_scene)
        
        # Give time for message to be processed
        time.sleep(0.1)
        
        self.get_logger().info(f'MoveIt: Added {object_name} back to collision objects')
        return True
    
    def get_all_collision_objects(self):
        """Get all collision objects with their poses from MoveIt planning scene"""
        from moveit_msgs.srv import GetPlanningScene
        from moveit_msgs.msg import PlanningSceneComponents
        
        if not self.get_scene_client.wait_for_service(timeout_sec=2.0):
            print("[CLIENT - parallelGripper.py]: WARNING: GetPlanningScene service not available")
            return {}
        
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        
        future = self.get_scene_client.call_async(request)
        
        # Wait for the service call to complete
        timeout = time.time() + 2.0
        while not future.done() and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if future.done():
            try:
                response = future.result()
                objects_dict = {}
                for obj in response.scene.world.collision_objects:
                    # Get object pose (center of the collision object)
                    if len(obj.primitive_poses) > 0:
                        pose = obj.primitive_poses[0]
                        objects_dict[obj.id] = {
                            'x': pose.position.x,
                            'y': pose.position.y,
                            'z': pose.position.z
                        }
                        # print(f"[CLIENT - parallelGripper.py]: Got pose for {obj.id}: ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")  # DEBUG
                    else:
                        objects_dict[obj.id] = None
                        # print(f"[CLIENT - parallelGripper.py]: No pose available for {obj.id}")  # DEBUG
                return objects_dict
            except Exception as e:
                print(f"[CLIENT - parallelGripper.py]: WARNING: Failed to get planning scene: {e}")
                return {}
        else:
            print("[CLIENT - parallelGripper.py]: WARNING: GetPlanningScene service call timed out")
            return {}

# =============================================================================== #
# parallelGR class, to OPEN/CLOSE the Parallel Gripper in Gazebo:

class parallelGR():

    def __init__(self, ObjectList, ROBOT, EE):
        
        self.OLCheck = False
        if ObjectList != None:
            self.OLCheck = True
        
        # Store object list for later use (e.g., in OPEN method)
        self.objectNames = ObjectList if ObjectList else []
        
        # Initialise RBT client -> For MoveG execution:
        self.RBTClient = RBT()

        # Initialise OBJECTS variable:
        if self.OLCheck:
            
            for x in ObjectList:
                OBJ = ObjectPose()
                OBJ.objectname = x
                OBJECTS.append(OBJ)
                
            # Initialise ObjPose class:
            self.objPoseClient = ObjPOSE(ObjectList)  
            # Initialise eePose class:
            self.eePoseClient = eePOSE()

            # Initialise LinkAttacher class:
            self.LinkAttacher = LinkAttacher(ROBOT, EE)
            
            # Initialise MoveIt scene manager:
            self.moveit_scene_manager = MoveItSceneManager()

    def CLOSE(self, VAL):
        
        T_start = time.time()
        
        # Initialise -> RES:
        RES = {
            "Message": "",
            "Success": False,
            "ExecTime": -1.0
        }

        print("[CLIENT - parallelGripper.py]: EXECUTION REQUEST -> CLOSE GRIPPER.")
        print("")

        objNAME = ""
        
        # ===== ATTACH OBJECT BEFORE CLOSING (prevents sliding) ===== #
        if self.OLCheck:
            
            Objects = self.objPoseClient.getOBJECTS()
            EEPose = self.eePoseClient.getEEPose()

            # Tolerance: how close the EE must be to object center to consider it graspable
            # 0.08m = 8cm tolerance - accounts for gripper geometry (EE is above cube center)
            TOLERANCE = 0.08
            
            for x in Objects:

                Check = True
                # print("[CLIENT - parallelGripper.py]: Checking if object is within grasp range: " + x.objectname)  # DEBUG
                # print("[CLIENT - parallelGripper.py]: EEPose.x -> " + str(EEPose.x) + " / ObjectPose.x -> " + str(x.x))  # DEBUG
                # print("[CLIENT - parallelGripper.py]: EEPose.y -> " + str(EEPose.y) + " / ObjectPose.y -> " + str(x.y))  # DEBUG
                # print("[CLIENT - parallelGripper.py]: EEPose.z -> " + str(EEPose.z) + " / ObjectPose.z -> " + str(x.z))  # DEBUG
                
                # Check if object pose was actually received (not all zeros)
                if (x.x == 0.0 and x.y == 0.0 and x.z == 0.0):
                    # print("[CLIENT - parallelGripper.py]: WARNING: Object pose is (0,0,0) - topic may not be publishing!")  # DEBUG
                    Check = False
                else:
                    if abs(EEPose.x - x.x) > TOLERANCE: 
                        Check = False
                    if abs(EEPose.y - x.y) > TOLERANCE: 
                        Check = False
                    if abs(EEPose.z - x.z) > TOLERANCE: 
                        Check = False
                
                # print("[CLIENT - parallelGripper.py]: Distance check result: " + str(Check))  # DEBUG
                # print("")  # DEBUG

                if Check == True:
                    objNAME = x.objectname
                    break

            # Attach object BEFORE closing gripper
            if objNAME != "":
                print("[CLIENT - parallelGripper.py]: Attaching object BEFORE closing gripper...")
                AttRES = self.LinkAttacher.ATTACH(objNAME)
                if AttRES:
                    print("[CLIENT - parallelGripper.py]: Object " + objNAME + " attached.")
                    
                    # Remove object from MoveIt's collision scene to prevent collision errors
                    # when closing the gripper (the object is now attached to the gripper)
                    self.moveit_scene_manager.remove_from_moveit(objNAME)
                    
                    # Get all collision objects with poses from MoveIt planning scene
                    all_objects = self.moveit_scene_manager.get_all_collision_objects()
                    # print(f"[CLIENT - parallelGripper.py]: Found {len(all_objects)} objects in MoveIt scene: {list(all_objects.keys())}")  # DEBUG
                    # print(f"[CLIENT - parallelGripper.py]: EE Pose: x={EEPose.x:.3f}, y={EEPose.y:.3f}, z={EEPose.z:.3f}")  # DEBUG
                    
                    # Remove nearby objects (e.g., objects in a stack) to prevent collision errors
                    # when closing the gripper. The objects remain in Gazebo for physics.
                    # We check for objects both above and below, and also nearby horizontally.
                    nearby_removed = []
                    
                    # Check objects from pose topics first (more accurate, real-time)
                    for x in Objects:
                        if x.objectname != objNAME and x.objectname in all_objects:
                            # Check if object is nearby (within 0.25m horizontally and within 0.25m vertically)
                            horizontal_dist = ((EEPose.x - x.x)**2 + (EEPose.y - x.y)**2)**0.5
                            vertical_diff = abs(EEPose.z - x.z)  # Absolute vertical distance
                            
                            # If object is within 0.25m horizontally and within 0.25m vertically, remove it
                            # This handles both stacked objects (above/below) and nearby objects
                            if horizontal_dist < 0.25 and vertical_diff < 0.25:
                                # print(f"[CLIENT - parallelGripper.py]: Removing nearby object {x.objectname} from MoveIt scene (stacked/nearby)...")  # DEBUG
                                self.moveit_scene_manager.remove_from_moveit(x.objectname)
                                nearby_removed.append(x.objectname)
                    
                    # Also check objects from MoveIt scene (for objects without pose topics)
                    for obj_name, obj_pose in all_objects.items():
                        if obj_name != objNAME and obj_name not in nearby_removed:
                            if obj_pose is not None:
                                # Check if object is nearby using MoveIt scene pose
                                horizontal_dist = ((EEPose.x - obj_pose['x'])**2 + (EEPose.y - obj_pose['y'])**2)**0.5
                                vertical_diff = abs(EEPose.z - obj_pose['z'])  # Absolute vertical distance
                                
                                # print(f"[CLIENT - parallelGripper.py]: Checking {obj_name}: h_dist={horizontal_dist:.3f}m, v_dist={vertical_diff:.3f}m, pose=({obj_pose['x']:.3f}, {obj_pose['y']:.3f}, {obj_pose['z']:.3f})")  # DEBUG
                                
                                # If object is within 0.25m horizontally and within 0.25m vertically, remove it
                                # This handles both stacked objects (above/below) and nearby objects
                                if horizontal_dist < 0.25 and vertical_diff < 0.25:
                                    # print(f"[CLIENT - parallelGripper.py]: Removing nearby object {obj_name} from MoveIt scene (stacked/nearby, from scene)...")  # DEBUG
                                    self.moveit_scene_manager.remove_from_moveit(obj_name)
                                    nearby_removed.append(obj_name)
                                else:
                                    # print(f"[CLIENT - parallelGripper.py]: Object {obj_name} is too far (h={horizontal_dist:.3f}m, v={vertical_diff:.3f}m), not removing")  # DEBUG
                                    pass
                            else:
                                # Pose is None (e.g. stale or missing in scene). Do NOT remove - we cannot
                                # know if the object is nearby; removing would wrongly remove cubes on
                                # other pegs/positions (only the picked cube should be removed).
                                pass
                    
                    # Only the picked object (objNAME) and objects actually within 0.25m have been
                    # removed. We do NOT remove all other cubes - that would clear MoveIt of cubes
                    # on other pegs when picking one cube.
                    
                    # Give MoveIt extra time to process all removals
                    if nearby_removed:
                        # print(f"[CLIENT - parallelGripper.py]: Removed {len(nearby_removed)} nearby objects from MoveIt scene: {nearby_removed}")  # DEBUG
                        time.sleep(0.5)  # Extra time when removing multiple objects
                    else:
                        # print(f"[CLIENT - parallelGripper.py]: No objects removed from MoveIt scene")  # DEBUG
                        time.sleep(0.3)
                    
                    print("[CLIENT - parallelGripper.py]: Now closing gripper.")
                    print("")
                else:
                    print("[CLIENT - parallelGripper.py]: WARNING: Could not attach " + objNAME + ", gripper will still close.")
                    print("")

        # Close GRIPPER -> /Move:
        G = Action()
        G.action = "MoveG"
        G.speed = 1.0
        G.moveg = VAL

        gRES = self.RBTClient.Move_EXECUTE(G)

        if (gRES["Success"] == True):
            print("[CLIENT - parallelGripper.py]: MoveG-CLOSE, Result -> " + gRES["Message"])
            print("")
            
        else:
            RES["Message"] = "MoveG-CLOSE, Result -> " + gRES["Message"]
            print("[CLIENT - parallelGripper.py]: " + RES["Message"])
            print("")
            return(RES)

        # Set result based on whether object was attached
        if objNAME != "":
            RES["Message"] = "Gripper closed, object->" + objNAME + " attached."
            RES["Success"] = True
            print("[CLIENT - parallelGripper.py]: " + RES["Message"])
            print("")
        else:
            RES["Message"] = "Gripper closed without grasping any object."
            RES["Success"] = True
            print("[CLIENT - parallelGripper.py]: " + RES["Message"])
            print("")
            
        T_end = time.time()
        T = round((T_end - T_start), 4)
        RES["ExecTime"] = T

        return(RES)

    def OPEN(self):
        
        T_start = time.time()

        # Initialise -> RES:
        RES = {
            "Message": "",
            "Success": False,
            "ExecTime": -1.0
        }
         
        print("[CLIENT - parallelGripper.py]: EXECUTION REQUEST -> OPEN GRIPPER.")
        print("")

        objNAME = ""
        
        # ===== DETACH OBJECT BEFORE OPENING (prevents collision issues) ===== #
        if self.OLCheck:
            global AttachCheck 
            objNAME = AttachCheck.NAME
            
            detached = False
            
            # First, try using AttachCheck if it has the object name
            if objNAME != "" and AttachCheck.ATTACHED:
                print("[CLIENT - parallelGripper.py]: Detaching object BEFORE opening gripper...")
                DetRES = self.LinkAttacher.DETACH(objNAME)
                if DetRES:
                    print("[CLIENT - parallelGripper.py]: Object " + objNAME + " detached.")
                    detached = True
                else:
                    # Timeout or service failure: don't retry same object; proceed to open gripper
                    print("[CLIENT - parallelGripper.py]: WARNING: Could not detach " + objNAME + " (timeout or failed); opening gripper anyway.")
                    detached = True
            
            # If not detached yet, try objects from the object list
            if not detached and hasattr(self, 'objectNames') and len(self.objectNames) > 0:
                for try_name in self.objectNames:
                    if try_name != "place_object":  # Skip placeholder
                        print(f"[CLIENT - parallelGripper.py]: Trying to detach object: {try_name}")
                        DetRES = self.LinkAttacher.DETACH(try_name)
                        if DetRES:
                            print(f"[CLIENT - parallelGripper.py]: Object {try_name} detached.")
                            objNAME = try_name
                            detached = True
                            break
            
            # If still not detached, try common object names
            if not detached:
                # Try cube0-cube9, then other common names
                common_names = (["cube" + str(i) for i in range(10)] + 
                               ["cube1", "cube2", "cube3", "box1", "cylinder1", "sphere1"])
                print("[CLIENT - parallelGripper.py]: AttachCheck empty, trying common object names...")
                for try_name in common_names:
                    print(f"[CLIENT - parallelGripper.py]: Trying to detach: {try_name}")
                    DetRES = self.LinkAttacher.DETACH(try_name)
                    if DetRES:
                        print(f"[CLIENT - parallelGripper.py]: Object {try_name} detached.")
                        objNAME = try_name
                        detached = True
                        break
            
            if detached:
                # Wait briefly for physics to settle (reduced from 0.5s to 0.1s for faster place)
                # If object was already detached externally, this is just a brief pause
                time.sleep(0.1)
                print("[CLIENT - parallelGripper.py]: Now opening gripper...")
                print("")
            else:
                print("[CLIENT - parallelGripper.py]: WARNING: Could not find attached object to detach, gripper will still open.")
                print("")

        # Open GRIPPER -> /Move (with retries; execution errors can be transient)
        G = Action()
        G.action = "MoveG"
        G.speed = 1.0
        G.moveg = 0.0
        OPEN_RETRIES = 3
        OPEN_RETRY_DELAY_S = 0.4
        gRES = None
        for attempt in range(1, OPEN_RETRIES + 1):
            gRES = self.RBTClient.Move_EXECUTE(G)
            if gRES["Success"]:
                if attempt > 1:
                    print("[CLIENT - parallelGripper.py]: MoveG-OPEN succeeded on attempt %d." % attempt)
                print("[CLIENT - parallelGripper.py]: MoveG-OPEN, Result -> " + gRES["Message"])
                print("")
                break
            print("[CLIENT - parallelGripper.py]: MoveG-OPEN attempt %d/%d failed: %s" % (attempt, OPEN_RETRIES, gRES["Message"]))
            if attempt < OPEN_RETRIES:
                time.sleep(OPEN_RETRY_DELAY_S)

        open_ok = gRES is not None and gRES["Success"]
        if not open_ok:
            print("[CLIENT - parallelGripper.py]: WARNING: Gripper open (MoveG) failed after %d attempts; object was detached." % OPEN_RETRIES)
            print("")

        # Add the placed object back to MoveIt's planning scene AFTER gripper opens
        # This is needed so MoveIt can plan around it for future operations
        # We do this AFTER opening to avoid MoveIt detecting collisions during gripper opening motion
        if objNAME != "" and AttachCheck.ATTACHED == False:
            # Wait for object to settle after gripper opens
            time.sleep(0.3)  # Give object time to fall and settle
            
            # Get the object's current pose from Gazebo (via pose topic)
            if self.OLCheck:
                Objects = self.objPoseClient.getOBJECTS()
                for x in Objects:
                    if x.objectname == objNAME:
                        # Check if we got a valid pose (not all zeros)
                        if not (x.x == 0.0 and x.y == 0.0 and x.z == 0.0):
                            # Create pose for MoveIt
                            from ros2srrc_data.msg import Robpose
                            obj_pose = Robpose()
                            obj_pose.x = x.x
                            obj_pose.y = x.y
                            obj_pose.z = x.z
                            
                            # print(f"[CLIENT - parallelGripper.py]: Adding {objNAME} back to MoveIt scene at pose: ({x.x:.3f}, {x.y:.3f}, {x.z:.3f})")  # DEBUG
                            # Default cube size is 0.05m (5cm)
                            self.moveit_scene_manager.add_to_moveit(objNAME, obj_pose, size=0.05)
                            # print(f"[CLIENT - parallelGripper.py]: {objNAME} restored to MoveIt planning scene.")  # DEBUG
                            break
                        else:
                            print(f"[CLIENT - parallelGripper.py]: WARNING: Could not get pose for {objNAME} to add back to MoveIt.")
        
        # Set result: require both detach and successful gripper open
        if objNAME != "" and AttachCheck.ATTACHED == False:
            if open_ok:
                RES["Message"] = "Gripper opened, object->" + objNAME + " detached."
                RES["Success"] = True
                print("[CLIENT - parallelGripper.py]: " + RES["Message"])
                print("")
            else:
                RES["Message"] = "Gripper open (MoveG) failed after %d attempts; object was detached." % OPEN_RETRIES
                RES["Success"] = False
                print("[CLIENT - parallelGripper.py]: " + RES["Message"])
                print("")
        else:
            RES["Message"] = "Gripper opened without dropping any object."
            RES["Success"] = True
            print("[CLIENT - parallelGripper.py]: " + RES["Message"])
            print("")

        T_end = time.time()
        T = round((T_end - T_start), 4)
        RES["ExecTime"] = T

        return(RES)