#!/usr/bin/env python3
# ===== IMPORT REQUIRED COMPONENTS ===== #
# System functions and classes:
import sys, os, yaml, time
# Required to include ROS2 and its components:
import rclpy
from ament_index_python.packages import get_package_share_directory

# PATH -> Python Classes:
PATH = os.path.join(get_package_share_directory("ros2srrc_execution"), 'python')
PATH_EE = PATH + "/endeffector"
PATH_EEGz = PATH + "/endeffector_gz"
PATH_ROB = PATH + "/robot"

# IMPORT -> ROBOT:
sys.path.append(PATH_ROB)
from robot import RBT

# IMPORT -> High-level actions (Pick, Place / Put Down) and HLD layers:
sys.path.append(PATH)
from pick_manual import Pick, PickConfig
from place_manual import Place, PlaceConfig
from constraints_handler import apply_constraints
from configuration_handler import set_configuration
from error_handler import handle_step_result

# IMPORT -> EE-Gz:
sys.path.append(PATH_EEGz)
from vacuumGripper import vacuumGR  # type: ignore
from parallelGripper import parallelGR  # type: ignore

# IMPORT -> EE:
sys.path.append(PATH_EE)
# ABB end-effectors (optional, only needed for ABB robots):
try:
    from zimmer_abb import ZimmerGRIPPER  # type: ignore
    from schunk_abb import SchunkGRIPPER  # type: ignore
    from vgr_abb import vgrABB  # type: ignore
    ABB_EE_AVAILABLE = True
except ImportError:
    ABB_EE_AVAILABLE = False
    # Define dummy classes if ABB end-effectors are not available
    class ZimmerGRIPPER:
        pass
    class SchunkGRIPPER:
        pass
    class vgrABB:
        pass

# UR end-effectors: use gripper factory (onrobot_ros2, ParallelGripper)
# RobotiqGRIPPER retained only for backward compat in step dispatch

# IMPORT ROS2 Custom Messages:
from ros2srrc_data.msg import Action
from ros2srrc_data.msg import Joint
from ros2srrc_data.msg import Joints
from ros2srrc_data.msg import Xyz
from ros2srrc_data.msg import Xyzypr
from ros2srrc_data.msg import Ypr
from ros2srrc_data.msg import Robpose

# ========================================================================================= #
# =================================== CLASSES/FUNCTIONS =================================== #
# ========================================================================================= #

# ========================================================================================= #           
# Get SEQUENCE from {program}.yaml file:
def getSEQUENCE(packageNAME, yamlNAME):

    RESULT = {}

    PATH = os.path.join(get_package_share_directory(packageNAME), 'programs')
    yamlPATH = PATH + "/" + yamlNAME + ".yaml"

    if not os.path.exists(yamlPATH):
        RESULT["Success"] = False
        return(RESULT)
    
    # Get sequence from YAML:
    with open(yamlPATH, 'r') as YAML:
        seqYAML = yaml.safe_load(YAML)

    RESULT["Sequence"] = seqYAML["Sequence"]
    RESULT["Robot"] = seqYAML["Specifications"]["Robot"]
    RESULT["EEType"] = seqYAML["Specifications"].get("EndEffector") or ""
    RESULT["EELink"] = seqYAML["Specifications"].get("EELink") or ""
    RESULT["Objects"] = seqYAML["Specifications"]["Objects"]
    RESULT["Success"] = True
    
    return(RESULT)

# ========================================================================================= #           
# EVALUATE INPUT ARGUMENTS:
def AssignArgument(ARGUMENT):
    ARGUMENTS = sys.argv
    for y in ARGUMENTS:
        if (ARGUMENT + ":=") in y:
            ARG = y.replace((ARGUMENT + ":="),"")
            return(ARG)

# ========================================================================================= #
# ========================================= MAIN ========================================== #
# ========================================================================================= #
def main(args=None):
    
    rclpy.init(args=args)

    # PRINT - INIT:
    print("==================================================")
    print("ROS 2 Sim-to-Real Robot Control: Program Execution")
    print("==================================================")
    print("")

    # GET PACKAGE NAME:
    PACKAGE = AssignArgument("package")
    if PACKAGE != None:
        None
    else:
        print("ERROR: 'package' INPUT ARGUMENT has not been defined. Please try again.")
        print("Closing program... BYE!")
        exit()

    # GET PROGRAM (SEQUENCE) NAME:
    PROGRAM = AssignArgument("program")
    if PROGRAM != None:
        None
    else:
        print("ERROR: 'program' INPUT ARGUMENT has not been defined. Please try again.")
        print("Closing program... BYE!")
        exit()

    # Get SEQUENCE from {program}.yaml file:
    seqRES = getSEQUENCE(PACKAGE,PROGRAM)
    yamlPATH = "/" + PACKAGE + "/programs/" + PROGRAM + ".yaml"

    if seqRES["Success"] == False:
        print("ERROR: " + yamlPATH + " file not found. Please try again.")
        print("Closing program... BYE!")
        exit()

    # Optional default EndEffector when program YAML does not specify one (e.g. ee_driver from config):
    node = rclpy.create_node("ExecuteProgram_tmp")
    node.declare_parameter("default_ee_driver", "")
    default_ee = node.get_parameter("default_ee_driver").get_parameter_value().string_value
    node.destroy_node()
    if (not seqRES["EEType"] or str(seqRES["EEType"]).strip() == "None") and default_ee:
        seqRES["EEType"] = default_ee
        print("Using default EndEffector from parameter default_ee_driver: " + default_ee)
    if not seqRES["EELink"] and seqRES["EEType"] and str(seqRES["EEType"]) != "None":
        seqRES["EELink"] = "EE_robotiq_2f85"  # fallback when EELink missing

    # ASSIGN -> SEQUENCE:
    SEQUENCE = seqRES["Sequence"]

    # PRINT - INFORMATION:
    print("PROGRAM -> " + yamlPATH + " found! The sequence is formed by the following steps:")
    print("")

    for x in SEQUENCE:
        print("   - Step Number " + str(x["Step"]) + ":")
        print("     " + x["Name"])
    
    print("")
    print("============================================================")
    print("Loading Robot+EndEffector Python Clients...")
    print("")

    # LOAD ROBOT/EE MOVEMENT PYTHON CLIENTS:
    print("ROBOT: ")
    RobotClient = RBT()
    print("Loaded.")
    print("")
    
    print("END-EFFECTOR:")
    
    if seqRES["EEType"] == "None":
        EEClient = None
        print("Not required.")
    elif seqRES["EEType"] == "VacuumGripper":
        EEClient = vacuumGR(seqRES["Objects"], seqRES["Robot"], seqRES["EELink"])
        print("Loaded -> VacuumGripper.")
    elif seqRES["EEType"] in ("ParallelGripper", "onrobot_ros2", "onrobot_2fg7"):
        from endeffector.gripper_factory import create_gripper
        EEClient = create_gripper(
            seqRES["EEType"], seqRES["Robot"], seqRES["EELink"], seqRES["Objects"]
        )
        print(f"Loaded -> {seqRES['EEType']}.")
    else:
        EEClient = None
        print(f"WARNING: Unknown EndEffector '{seqRES['EEType']}', continuing without gripper.")

    print("")

    print("============================================================")
    print("============================================================")
    print("Executing sequence...")
    print("")

    # Initialise -> RES VARIABLE:
    RES = None

    # ==== EXECUTE PROGRAM, STEP BY STEP ===== #
    for x in SEQUENCE:
        
        try:

            print("============================================================")
            print("Step N" + str(x["Step"]) + ": " + x["Name"])
            print("")

            if x["Type"] == "MoveJ":

                ACTION = Action()
                ACTION.action = "MoveJ"
                ACTION.speed = x["Speed"]

                INPUT = Joints()
                INPUT.joint1 = x["Input"]["joint1"]
                INPUT.joint2 = x["Input"]["joint2"]
                INPUT.joint3 = x["Input"]["joint3"]
                INPUT.joint4 = x["Input"]["joint4"]
                INPUT.joint5 = x["Input"]["joint5"]
                INPUT.joint6 = x["Input"]["joint6"]
                ACTION.movej = INPUT

                RES = RobotClient.Move_EXECUTE(ACTION)

            elif x["Type"] == "MoveJ7":

                ACTION = Action()
                ACTION.action = "MoveJ"
                ACTION.speed = x["Speed"]

                INPUT = Joints()
                INPUT.joint1 = x["Input"]["joint1"]
                INPUT.joint2 = x["Input"]["joint2"]
                INPUT.joint3 = x["Input"]["joint3"]
                INPUT.joint4 = x["Input"]["joint4"]
                INPUT.joint5 = x["Input"]["joint5"]
                INPUT.joint6 = x["Input"]["joint6"]
                INPUT.joint7 = x["Input"]["joint7"]
                ACTION.movej = INPUT

                RES = RobotClient.Move_EXECUTE(ACTION)

            elif x["Type"] == "MoveR":

                ACTION = Action()
                ACTION.action = "MoveR"
                ACTION.speed = x["Speed"]

                INPUT = Joint()
                INPUT.joint = x["Input"]["joint"]
                INPUT.value = x["Input"]["value"]
                ACTION.mover = INPUT

                RES = RobotClient.Move_EXECUTE(ACTION)

            elif x["Type"] == "MoveL":

                ACTION = Action()
                ACTION.action = "MoveL"
                ACTION.speed = x["Speed"]

                INPUT = Xyz()
                INPUT.x = x["Input"]["x"]
                INPUT.y = x["Input"]["y"]
                INPUT.z = x["Input"]["z"]
                ACTION.movel = INPUT

                RES = RobotClient.Move_EXECUTE(ACTION)

            elif x["Type"] == "MoveROT":

                ACTION = Action()
                ACTION.action = "MoveROT"
                ACTION.speed = x["Speed"]

                INPUT = Ypr()
                INPUT.pitch = x["Input"]["pitch"]
                INPUT.yaw = x["Input"]["yaw"]
                INPUT.roll = x["Input"]["roll"]
                ACTION.moverot = INPUT

                RES = RobotClient.Move_EXECUTE(ACTION)

            elif x["Type"] == "MoveRP":

                ACTION = Action()
                ACTION.action = "MoveRP"
                ACTION.speed = x["Speed"]

                INPUT = Xyzypr()
                INPUT.x = x["Input"]["x"]
                INPUT.y = x["Input"]["y"]
                INPUT.z = x["Input"]["z"]
                INPUT.pitch = x["Input"]["pitch"]
                INPUT.yaw = x["Input"]["yaw"]
                INPUT.roll = x["Input"]["roll"]
                ACTION.moverp = INPUT

                RES = RobotClient.Move_EXECUTE(ACTION)

            elif x["Type"] == "MoveG":

                ACTION = Action()
                ACTION.action = "MoveG"
                ACTION.speed = x["Speed"]
                ACTION.moveg = x["Input"]["value"]

                RES = RobotClient.Move_EXECUTE(ACTION)

            elif x["Type"] == "RobMove":

                InputPose = Robpose()
                InputPose.x = x["Input"]["x"]
                InputPose.y = x["Input"]["y"]
                InputPose.z = x["Input"]["z"]
                InputPose.qx = x["Input"]["qx"]
                InputPose.qy = x["Input"]["qy"]
                InputPose.qz = x["Input"]["qz"]
                InputPose.qw = x["Input"]["qw"]

                RES = RobotClient.RobMove_EXECUTE(x["Movement"], x["Speed"], InputPose)

            elif x["Type"] == "ParallelGripper":
                if x["Action"] == "CLOSE":
                    val = x.get("Value", 100)
                    percent = float(val) / 100.0 if val is not None else 1.0
                    RES = EEClient.close(percent)
                else:
                    RES = EEClient.open()
            
            elif x["Type"] == "VacuumGripper":

                if x["Action"] == "ACTIVATE":
                    RES = EEClient.ACTIVATE()
                else:
                    RES = EEClient.DEACTIVATE()

            elif x["Type"] == "EGP64/ABB":

                if x["Action"] == "CLOSE":
                    RES = EEClient.CLOSE()
                else:
                    RES = EEClient.OPEN()

            elif x["Type"] == "GPP5010NC/ABB":

                if x["Action"] == "CLOSE":
                    RES = EEClient.CLOSE()
                else:
                    RES = EEClient.OPEN()

            elif x["Type"] == "vgr/ABB":

                if x["Action"] == "ACTIVATE":
                    RES = EEClient.ACTIVATE()
                else:
                    RES = EEClient.DEACTIVATE()

            elif x["Type"] in ("onrobot_ros2", "onrobot_2fg7"):
                if x["Action"] == "CLOSE":
                    val = x.get("Value", 100)
                    percent = float(val) / 100.0 if val is not None else 1.0
                    RES = EEClient.close(percent)
                else:
                    RES = EEClient.open()

            elif x["Type"] == "Pick":

                # Create object pose from input
                ObjectPose = Robpose()
                ObjectPose.x = x["Input"]["x"]
                ObjectPose.y = x["Input"]["y"]
                ObjectPose.z = x["Input"]["z"]
                ObjectPose.qx = x["Input"]["qx"]
                ObjectPose.qy = x["Input"]["qy"]
                ObjectPose.qz = x["Input"]["qz"]
                ObjectPose.qw = x["Input"]["qw"]

                # Get optional config parameters
                pick_config = x.get("Config", None)

                # Create Pick action instance and execute
                PickAction = Pick(RobotClient, EEClient, pick_config)
                RES = PickAction.execute(ObjectPose)

            elif x["Type"] == "Place" or x["Type"] == "PutDown":

                # Create place pose from input (x ∈ R^6: position + orientation)
                PlacePose = Robpose()
                PlacePose.x = x["Input"]["x"]
                PlacePose.y = x["Input"]["y"]
                PlacePose.z = x["Input"]["z"]
                PlacePose.qx = x["Input"]["qx"]
                PlacePose.qy = x["Input"]["qy"]
                PlacePose.qz = x["Input"]["qz"]
                PlacePose.qw = x["Input"]["qw"]

                # Get optional config parameters
                place_config = x.get("Config", None)

                # Create Place (Put Down) action instance and execute
                PlaceAction = Place(RobotClient, EEClient, place_config)
                RES = PlaceAction.execute(PlacePose)

            elif x["Type"] == "SetConstraints":
                RES = apply_constraints(x)

            elif x["Type"] == "SetConfiguration":
                RES = set_configuration(x)

            else:
                print("ERROR: ACTION TYPE -> " + x["Type"] + " unknown.")
                print("Closing program... BYE!")
                exit()

            # CHECK if STEP EXECUTION WAS SUCCESSFUL (centralized error handling):
            print("")
            handle_step_result(RES, step_name=x.get("Name", ""))
            print("Execution SUCCESSFUL!")
            print("Message -> " + RES["Message"])
            print("")
                
            # ADD -> DELAY (optional pause after this step, in seconds; omit or set to 0 for no wait):
            delay_s = float(x.get("Delay", 0.0))
            if delay_s > 0.0:
                print("Requested a waitTime of " + str(delay_s) + " seconds.")
                #time.sleep(delay_s)
                print("")
                
        except KeyboardInterrupt:
            
            # CANCEL ANY ONGOING ROBOT MOVEMENTS:
            RobotClient.CANCEL()
            
            print("Sequence execution manually interrupted and cancelled.")
            print("Closing... BYE!")
            exit()

    # ==== FINISH ===== #
    print("")
    print("")
    print("Sequence successfully executed. Closing Program... Bye!")
    print("=======================================================")

    rclpy.shutdown()
    exit()

if __name__ == '__main__':
    main()