# simulation.launch.py:
# Launch file for the ROBOT's GAZEBO SIMULATION in ROS2 Humble:

# Import libraries:
import os, sys, xacro, yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

# ===== REQUIRED TO GET THE ROBOT CONFIGURATION === #

# EVALUATE INPUT ARGUMENTS:
def AssignArgument(ARGUMENT):
    ARGUMENTS = sys.argv
    for y in ARGUMENTS:
        if (ARGUMENT + ":=") in y:
            ARG = y.replace((ARGUMENT + ":="),"")
            return(ARG)

# GET CONFIGURATION from YAML:
# Supports legacy "ee" or explicit srdf_ee_id, moveit_ee_group, ee_driver, ee_link.
def GetCONFIG(CONFIGURATION, PKG_PATH):
    
    RESULT = {"Success": False, "ID": "", "Name": "", "urdf": "", "ee": "",
              "srdf_ee_id": "", "moveit_ee_group": "", "ee_driver": "", "ee_link": ""}
    
    YAML_PATH = PKG_PATH + "/config/configurations.yaml"
    
    if not os.path.exists(YAML_PATH):
        return (RESULT)
    
    with open(YAML_PATH, 'r') as YAML:
        cYAML = yaml.safe_load(YAML)

    for x in cYAML["Configurations"]:
        if x["ID"] == CONFIGURATION:
            RESULT["Success"] = True
            RESULT["ID"] = x["ID"]
            RESULT["Name"] = x["Name"]
            RESULT["urdf"] = x["urdf"]
            RESULT["rob"] = x["rob"]
            legacy_ee = x.get("ee", "none")
            RESULT["srdf_ee_id"] = x.get("srdf_ee_id") or legacy_ee
            RESULT["moveit_ee_group"] = x.get("moveit_ee_group") or legacy_ee
            RESULT["ee_driver"] = x.get("ee_driver") or legacy_ee
            RESULT["ee_link"] = x.get("ee_link") or ("EE_" + (x.get("moveit_ee_group") or legacy_ee) if (x.get("moveit_ee_group") or legacy_ee) != "none" else "")
            RESULT["ee"] = RESULT["moveit_ee_group"] if (RESULT["moveit_ee_group"] and RESULT["moveit_ee_group"] != "none") else legacy_ee
            break

    return(RESULT)

# GET EE-Controllers LIST:
def GetEEctr(EEName):
    
    RESULT = []

    PATH = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'ros2_SimRealRobotControl', 'ros2srrc_endeffectors', EEName, 'config')
    YAML_PATH = PATH + "/controller_moveit2.yaml"
    
    with open(YAML_PATH, 'r') as YAML:
        cYAML = yaml.safe_load(YAML)

    for x in cYAML["controller_names"]:
        RESULT.append(x)

    return(RESULT)

# CHECK if CONTROLLER file exists for EE:
def EEctrlEXISTS(EEName):
    
    PATH = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'ros2_SimRealRobotControl', 'ros2srrc_endeffectors', EEName, 'config')
    YAML_PATH = PATH + "/controller.yaml"
    
    RES = os.path.exists(YAML_PATH)
    return(RES)

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    
    LD = LaunchDescription()
    
    # === INPUT ARGUMENT: ROS 2 PACKAGE === #
    PACKAGE_NAME = AssignArgument("package")
    if PACKAGE_NAME != None:
        None
    else:
        print("")
        print("ERROR: package INPUT ARGUMENT has not been defined. Please try again.")
        print("Closing... BYE!")
        exit()
        
    # CHECK if -> PACKAGE EXISTS, and GET PATH:
    try:
        PKG_PATH = get_package_share_directory(PACKAGE_NAME)
    except PackageNotFoundError:
        print("")
        print("ERROR: The defined ROS 2 Package was not found. Please try again.")
        print("Closing... BYE!")
        exit()
    except ValueError:
        print("")
        print("ERROR: The defined ROS 2 Package name is not valid. Please try again.")
        print("Closing... BYE!")
        exit()
    
    # === INPUT ARGUMENT: CONFIGURATION === #
    CONFIG = AssignArgument("config")
    CONFIGURATION = GetCONFIG(CONFIG, PKG_PATH)

    if CONFIGURATION["Success"] == False:
        print("")
        print("ERROR: config INPUT ARGUMENT has not been correctly defined. Please try again.")
        print("Closing... BYE!")
        exit()   

    # ========== CELL INFORMATION ========== #
    print("")
    print("===== GAZEBO: Robot Simulation (" + PACKAGE_NAME + ") =====")
    print("Robot configuration:")
    print(CONFIGURATION["ID"] + " -> " + CONFIGURATION["Name"])
    print("")
    
    # ***** GAZEBO ***** #   
    # DECLARE Gazebo WORLD file:
    world_gazebo = os.path.join(
        get_package_share_directory('ros2srrc_gazebo'),
        'worlds',
        'pick_and_place_fancy.world')
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': world_gazebo}.items(),
            )

    # ***** ROBOT DESCRIPTION ***** #
    # Robot Description file package:
    robot_description_path = os.path.join(get_package_share_directory(PACKAGE_NAME))
    # ROBOT urdf file path:
    xacro_file = os.path.join(robot_description_path,'urdf',CONFIGURATION["urdf"])
    # Generate ROBOT_DESCRIPTION variable:
    doc = xacro.parse(open(xacro_file))
    
    if CONFIGURATION["ee"] == "none" or (CONFIGURATION.get("moveit_ee_group") or "none") == "none":
        EE = "false"
    else:
        EE = "true"
    
    xacro.process_doc(doc, mappings={
        "EE": EE,
        "EE_name": CONFIGURATION["moveit_ee_group"] if EE == "true" else "none",
    })
    
    # EE -> Controller file needed?
    if EE == "true":
        if EEctrlEXISTS(CONFIGURATION["moveit_ee_group"]) == False:
            EE = "true-NOctr"
    
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ]
    )

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description','-entity', CONFIGURATION["rob"]],
                        output='both')

    # ***** CONTROLLERS ***** #
    # Joint STATE BROADCASTER:
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    # Joint TRAJECTORY Controller:
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # EE CONTROLLERS:
    if EE == "true":
        CONTROLLERS = GetEEctr(CONFIGURATION["moveit_ee_group"])
        CONTROLLER_NODES = []

        for x in CONTROLLERS:
            CONTROLLER_NODES.append(
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[x, "-c", "/controller_manager"],
                )
            )

    # =============================================== #
    # ========== RETURN LAUNCH DESCRIPTION ========== #

    # Add ROS 2 Nodes to LaunchDescription() element:
    LD.add_action(gazebo)
    LD.add_action(node_robot_state_publisher)
    LD.add_action(spawn_entity)

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action = spawn_entity,
            on_exit = [
                joint_state_broadcaster_spawner,
                ]
            )
        )
    )

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action = spawn_entity,
            on_exit = [
                joint_trajectory_controller_spawner,
                ]
            )
        )
    )

    if EE == "true":

        for x in CONTROLLER_NODES:

            LD.add_action(RegisterEventHandler(
                OnProcessExit(
                    target_action = joint_trajectory_controller_spawner,
                    on_exit = [
                        x,
                        ]
                    )
                )
            )

    # ***** RETURN  ***** #
    return(LD)