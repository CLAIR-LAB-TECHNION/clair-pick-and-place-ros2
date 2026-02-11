# moveit2.launch.py:
# Launch file for the Robot's GAZEBO SIMULATION + MoveIt!2 Framework in ROS2 Humble:

# Import libraries:
import os, sys, xacro, yaml, re
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
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
# RESULT["ee"] = moveit_ee_group or ee (for "has EE" / "none" checks).
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
        YAML_PATH = PKG_PATH + "/config/configurations.yaml"
        if os.path.exists(YAML_PATH):
            with open(YAML_PATH, 'r') as YAML:
                cYAML = yaml.safe_load(YAML)
            ids = [x["ID"] for x in cYAML.get("Configurations", [])]
            print("Valid config values for " + PACKAGE_NAME + ": " + ", ".join(ids))
        print("Closing... BYE!")
        exit()   

    # ========== CELL INFORMATION ========== #
    print("")
    print("===== GAZEBO: Robot Simulation + MoveIt!2 Framework (" + PACKAGE_NAME + ") =====")
    print("Robot configuration:")
    print(CONFIGURATION["ID"] + " -> " + CONFIGURATION["Name"])
    print("")
    
    # ***** GAZEBO ***** #
    # Ensure Gazebo can find the link attacher plugin (for ATTACHLINK/DETACHLINK services).
    try:
        _linkattacher_share = get_package_share_directory('ros2_linkattacher')
        _linkattacher_lib = os.path.join(os.path.dirname(_linkattacher_share), 'lib')
        _gpp = os.environ.get('GAZEBO_PLUGIN_PATH', '')
        _gazebo_plugin_path = _linkattacher_lib + ((':' + _gpp) if _gpp else '')
        set_linkattacher_env = SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=_gazebo_plugin_path)
    except PackageNotFoundError:
        set_linkattacher_env = None
    # DECLARE Gazebo WORLD file:
    world_gazebo = os.path.join(
        get_package_share_directory('ros2srrc_gazebo'),
        'worlds',
        'ros2srrc_gazebo.world')
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
    # Strip XML comments to avoid gazebo_ros2_control parser error
    # (comments with colons like URLs break the --param parser)
    comments_before = robot_description_config.count('<!--')
    robot_description_config = re.sub(r'<!--.*?-->', '', robot_description_config, flags=re.DOTALL)
    comments_after = robot_description_config.count('<!--')
    print(f"[DEBUG] URDF comments: {comments_before} -> {comments_after}")
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
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', CONFIGURATION["rob"]],
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

    # *********************** MoveIt!2 *********************** #   

    # *** PLANNING CONTEXT *** #
    # Robot description, SRDF (filename from srdf_ee_id):
    if (EE == "false"):
        robot_description_semantic_config = load_file("ros2srrc_moveit", "config/" + CONFIGURATION["rob"] + ".srdf")
    else:
        robot_description_semantic_config = load_file("ros2srrc_moveit", "config/" + CONFIGURATION["rob"] + "_" + CONFIGURATION["srdf_ee_id"] + ".srdf")
    
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    # Kinematics.yaml file:
    kinematics_yaml = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # joint_limits.yaml file:
    if (EE == "false") or (EE == "true-NOctr"):
        joint_limits_yaml = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/joint_limits.yaml")
    else:
        YAML_ROB = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/joint_limits.yaml")["joint_limits"]
        YAML_EE = load_yaml("ros2srrc_endeffectors", CONFIGURATION["moveit_ee_group"] + "/config/joint_limits.yaml")["joint_limits"]
        joint_limits_yaml = {}
        joint_limits_yaml["joint_limits"] = YAML_ROB | YAML_EE
    
    joint_limits = {'robot_description_planning': joint_limits_yaml}

    # pilz_planning_pipeline_config.yaml file:
    pilz_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "request_adapters": """ """,
            "start_state_max_bounds_error": 0.1,
            "default_planner_config": "PTP",
        }
    }
    pilz_cartesian_limits_yaml = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/pilz_cartesian_limits.yaml")
    pilz_cartesian_limits = {'robot_description_planning': pilz_cartesian_limits_yaml}

    # MoveIt!2 Controllers:
    if (EE == "false") or (EE == "true-NOctr"):
        moveit_simple_controllers_yaml = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/controller_moveit2.yaml")
    else:
        YAML_ROB = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/controller_moveit2.yaml")
        YAML_EE = load_yaml("ros2srrc_endeffectors", CONFIGURATION["moveit_ee_group"] + "/config/controller_moveit2.yaml")
        for x in YAML_ROB["controller_names"]:
            YAML_EE["controller_names"].append(x)
        moveit_simple_controllers_yaml = YAML_ROB | YAML_EE

    # MoveIt!2 Parameters:
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    move_group_capabilities = {
        "capabilities": """pilz_industrial_motion_planner/MoveGroupSequenceAction \
            pilz_industrial_motion_planner/MoveGroupSequenceService"""
    }

    # MoveGroup Node:
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            
            pilz_planning_pipeline_config,

            joint_limits,
            pilz_cartesian_limits,

            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {"use_sim_time": True},
        ],
    )

    # RVIZ (config filename from srdf_ee_id):
    rviz_base = os.path.join(get_package_share_directory("ros2srrc_moveit"), "config")
    if EE == "false":
        rviz_full_config = os.path.join(rviz_base, CONFIGURATION["rob"] + ".rviz")
    else:
        rviz_full_config = os.path.join(rviz_base, CONFIGURATION["rob"] + "_" + CONFIGURATION["srdf_ee_id"] + ".rviz")

    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            
            pilz_planning_pipeline_config,

            joint_limits,
            pilz_cartesian_limits,

            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {"use_sim_time": True},
        ]
    )

    # =================================================================================================== #
    # ============================= ros2srrc_execution -> CUSTOM INTERFACES ============================= #

    # Move and Sequence (EE_PARAM = moveit_ee_group):
    move_ee_param = CONFIGURATION["moveit_ee_group"] if (EE == "true" and CONFIGURATION.get("moveit_ee_group")) else "none"
    MoveInterface = Node(
        name="move",
        package="ros2srrc_execution",
        executable="move",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": CONFIGURATION["rob"]}, {"EE_PARAM": move_ee_param}, {"ENV_PARAM": "gazebo"}],
    )

    # RobMove and RobPose:
    RobMoveInterface = Node(
        name="robmove",
        package="ros2srrc_execution",
        executable="robmove",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": CONFIGURATION["rob"]}],
    )
    RobPoseInterface = Node(
        name="robpose",
        package="ros2srrc_execution",
        executable="robpose",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": CONFIGURATION["rob"]}],
    )
    
    # =============================================== #
    # ========== RETURN LAUNCH DESCRIPTION ========== #

    # Add ROS 2 Nodes to LaunchDescription() element:
    if set_linkattacher_env is not None:
        LD.add_action(set_linkattacher_env)
    LD.add_action(gazebo)
    LD.add_action(node_robot_state_publisher)
    LD.add_action(static_tf)
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

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action = spawn_entity,
            on_exit = [
                
                # MoveIt!2:
                TimerAction(
                    period=2.0,
                    actions=[
                        rviz_node_full,
                        run_move_group_node,
                    ]
                ),
                
                ]
            )
        )
    )

    # Start interface nodes after controllers are ready
    # Wait for joint_trajectory_controller to be ready, then add delay for MoveIt!2 initialization
    # This ensures controllers are publishing joint_states before interfaces try to use MoveIt!2
    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action = joint_trajectory_controller_spawner,
            on_exit = [
                # Interfaces - start after controllers are ready with delay for MoveIt!2 initialization
                # MoveIt!2 starts 2s after spawn, so 4s total delay ensures it's ready
                TimerAction(
                    period=4.0,
                    actions=[
                        MoveInterface,
                        RobMoveInterface,
                        RobPoseInterface,
                    ]
                ),
            ]
        )
    ))

    # ***** RETURN  ***** #
    return(LD)