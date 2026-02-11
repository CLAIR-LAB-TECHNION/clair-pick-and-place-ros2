#!/usr/bin/python3

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: June, 2024.                                                                    #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) ROS 2 Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.

# bringup.launch.py:
# Launch file for the Robot's BRINGUP ROS 2 DRIVER + MoveIt!2 Framework in ROS2 Humble:

# Import libraries:
import os, sys, xacro, yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit

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

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():

    LD = LaunchDescription()

    # === INPUT ARGUMENT: robot_ip === #
    robot_ip = AssignArgument("robot_ip")
    if robot_ip != None:
        None
    else:
        print("")
        print("ERROR: robot_ip INPUT ARGUMENT has not been defined. Please try again.")
        print("Closing... BYE!")
        exit()
    
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

    if CONFIGURATION["ee"] == "none" or (CONFIGURATION.get("moveit_ee_group") or "none") == "none":
        EE = "false"
    else:
        EE = "true"

    # ========== CELL INFORMATION ========== #
    print("")
    print("===== " + CONFIGURATION["rob"] + ": Robot Bringup + MoveIt!2 Framework (" + PACKAGE_NAME + ") =====")
    print("Robot IP Address -> " + robot_ip)
    print("Robot configuration:")
    print(CONFIGURATION["ID"] + " -> " + CONFIGURATION["Name"])
    print("")

    # UR_ROBOT_DRIVER variables: 
    urcl_path = os.path.join(get_package_share_directory('ur_client_library'))
    script_filename = os.path.join(urcl_path,
                              'resources',
                              'external_control.urscript')
    ur_path = os.path.join(get_package_share_directory('ur_robot_driver'))
    input_recipe_filename = os.path.join(ur_path,
                              'resources',
                              'rtde_input_recipe.txt')
    output_recipe_filename = os.path.join(ur_path,
                              'resources',
                              'rtde_output_recipe.txt')

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

        "robot_ip": robot_ip,
        "bringup": "true",

        "script_filename": script_filename,
        "input_recipe_filename": input_recipe_filename,
        "output_recipe_filename": output_recipe_filename,
    })
    
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {"use_sim_time": False}
        ]
    )
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # ***** CONTROLLERS ***** #

    # ros2_control:
    ros2_controllers_path = os.path.join(get_package_share_directory("ros2srrc_robots"), CONFIGURATION["rob"], "config", "controller_ur.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="both"
    )

    # IO and STATUS CONTROLLER:
    io_and_status_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["io_and_status_controller", "--controller-manager", "/controller_manager"],
    )
    # Joint STATE BROADCASTER:
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    # Speed scaling STATE BROADCASTER:
    speed_scaling_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["speed_scaling_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    # Joint TRAJECTORY Controller:
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )
    # Joint (SCALED) TRAJECTORY Controller:
    scaled_joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scaled_joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # *********************** MoveIt!2 *********************** #   

    # *** PLANNING CONTEXT *** #
    # Robot description, SRDF (filename from srdf_ee_id):
    if EE == "false":
        robot_description_semantic_config = load_file("ros2srrc_moveit", "config/" + CONFIGURATION["rob"] + ".srdf")
    else:
        robot_description_semantic_config = load_file("ros2srrc_moveit", "config/" + CONFIGURATION["rob"] + "_" + CONFIGURATION["srdf_ee_id"] + ".srdf")
    
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    # Kinematics.yaml file:
    kinematics_yaml = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # joint_limits.yaml file:
    joint_limits_yaml = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/joint_limits.yaml")
    joint_limits = {'robot_description_planning': joint_limits_yaml}

    # pilz_planning_pipeline_config.yaml file:
    # SetDefaultPlannerId adapter fills in planner_id when RViz sends empty (e.g. Plan & Execute).
    pilz_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "request_adapters": "ros2srrc_planning_request_adapters/SetDefaultPlannerId",
            "start_state_max_bounds_error": 0.1,
            "default_planner_config": "PTP",
            "request_adapters.SetDefaultPlannerId.default_planner_id": "PTP",
        }
    }
    pilz_cartesian_limits_yaml = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/pilz_cartesian_limits.yaml")
    pilz_cartesian_limits = {'robot_description_planning': pilz_cartesian_limits_yaml}

    # MoveIt!2 Controllers: use scaled_joint_trajectory_controller (recommended for UR speed scaling).
    # With execution_duration_monitoring disabled below, Plan & Execute should complete without TIMED_OUT.
    moveit_simple_controllers_yaml = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/controller_moveit2.yaml")
    moveit_simple_controllers_yaml["joint_trajectory_controller"]["default"] = False
    moveit_simple_controllers_yaml["scaled_joint_trajectory_controller"]["default"] = True

    # MoveIt!2 Parameters:
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    # Trajectory execution: disable duration monitoring so scaled controller does not trigger TIMED_OUT.
    # Reduced default velocity/acceleration (0.1) for safer testing; change in RViz or via move/robmove goals as needed.
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 10.0,
        "trajectory_execution.allowed_goal_duration_margin": 5.0,
        "trajectory_execution.allowed_start_tolerance": 0.04,
        "trajectory_execution.execution_duration_monitoring": False,
        "default_velocity_scaling_factor": 0.1,
        "default_acceleration_scaling_factor": 0.1,
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
        ]
    )

    # =================================================================================================== #
    # ============================= ros2srrc_execution -> CUSTOM INTERFACES ============================= #

    # Move (EE_PARAM = moveit_ee_group so move node sees valid MoveIt group / endeffector folder):
    move_ee_param = CONFIGURATION["moveit_ee_group"] if (EE == "true" and CONFIGURATION["moveit_ee_group"]) else "none"
    MoveInterface = Node(
        name="move",
        package="ros2srrc_execution",
        executable="move",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"ROB_PARAM": CONFIGURATION["rob"]}, {"EE_PARAM": move_ee_param}, {"ENV_PARAM": "bringup"}],
    )
    # RobMove and RobPose:
    RobMoveInterface = Node(
        name="robmove",
        package="ros2srrc_execution",
        executable="robmove",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"ROB_PARAM": CONFIGURATION["rob"]}],
    )
    RobPoseInterface = Node(
        name="robpose",
        package="ros2srrc_execution",
        executable="robpose",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"ROB_PARAM": CONFIGURATION["rob"]}],
    )

    # =================================================================================================== #
    # ================================== RobotiQ Gripper Service Server ================================= #
    RobotiqServer = Node(
        name="robotiq_server",
        package="ros2_robotiqgripper",
        executable="server.py",
        output="screen",
        parameters=[{"IPAddress": robot_ip}],
    )

    # =============================================== #
    # ========== RETURN LAUNCH DESCRIPTION ========== #

    # Add ROS 2 Nodes to LaunchDescription() element:
    LD.add_action(node_robot_state_publisher)
    LD.add_action(static_tf)
    
    LD.add_action(ros2_control_node)
    LD.add_action(io_and_status_controller_spawner)
    LD.add_action(joint_state_broadcaster_spawner)
    LD.add_action(speed_scaling_state_broadcaster_spawner)
    # Use scaled_joint_trajectory_controller for UR speed scaling; execution_duration_monitoring disabled to avoid TIMED_OUT.
    # LD.add_action(joint_trajectory_controller_spawner)
    LD.add_action(scaled_joint_trajectory_controller_spawner)

    # Real robot with Robotiq 2F-85 model (ur5_2, ur5_4): UR driver only publishes arm joints.
    # Publish default states for the 6 gripper joints so MoveIt planning_scene_monitor sees a complete state (removes "missing joint" warning).
    if EE == "true" and CONFIGURATION.get("moveit_ee_group") == "robotiq_2f85":
        Robotiq85JointStatePublisher = Node(
            name="robotiq_85_joint_state_publisher",
            package="ros2srrc_execution",
            executable="robotiq_85_joint_state_publisher.py",
            output="log",
            parameters=[{"rate": 10.0}],
        )
        LD.add_action(Robotiq85JointStatePublisher)

    # Robotiq server for Robotiq end-effectors that use /Robotiq_Gripper (2F-85 and HandE).
    # Do not start Robotiq server for OnRobot 2FG7 (different protocol; 2FG7 uses URScript via TCP).
    # OnRobot 2FG7: expose robot_ip via a small params node so ExecuteProgram can get it without -p OnRobot2FG7_param_reader.robot_ip.
    if CONFIGURATION["ee_driver"] == "onrobot_2fg7":
        OnRobot2FG7ParamsNode = Node(
            name="onrobot_2fg7_bringup_params",
            package="ros2srrc_execution",
            executable="onrobot_2fg7_params_node.py",
            output="screen",
            parameters=[{"robot_ip": robot_ip}],
        )
        LD.add_action(OnRobot2FG7ParamsNode)
    # Start Robotiq server for both robotiq_2f85 (ur5_2) and RobotiqHandE (ur5_3); both use same /Robotiq_Gripper service.
    # Server is parameterised with robot_ip (HandE is on-robot; 2F-85 often same network; if gripper has different IP, run server separately).
    if CONFIGURATION["ee_driver"] in ("robotiq_2f85", "RobotiqHandE/UR") or CONFIGURATION["moveit_ee_group"] == "robotiq_hande":
        LD.add_action(RobotiqServer)

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=scaled_joint_trajectory_controller_spawner,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[
                        rviz_node_full,
                        run_move_group_node,
                    ]
                ),
            ]
        )
    ))

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=scaled_joint_trajectory_controller_spawner,
            on_exit=[
                TimerAction(
                    period=5.0,
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