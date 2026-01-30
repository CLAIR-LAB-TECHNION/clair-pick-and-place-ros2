# IFRA-Cranfield: ROS2 Sim-to-Real Robot Control

## Robot Simulation & Control: Standard ROS 2 Packages for different Robot Manipulators

The /packages folder contains standard ROS 2 packages for each robot arm included in ros2_SimRealRobotControl. 
These packages provide simple, raw simulation environments where a robot (and end-effector, if applicable) is placed on top of a basic robot stand.

The main purpose of these packages is to serve as reference examples for building your own robot cells. 
By following this structure, you can quickly create a new ROS 2 package for any robot + end-effector combination by adding:

- A _configuration_ file that defines the different layouts and configurations of your robot cell.
- The robot cell’s URDF files, together with CAD files of the cell or any relevant objects.

This modular approach is possible because all robot- and end-effector-specific data is already centralized in the top-level /robots and /endeffectors folders of this repository. Whenever you design a new robot cell, you simply reference these standardized definitions—without duplicating data or parameters.


</br>

---

ROS 2 Package folder structure:

- _/config_: Contains the configurations.yaml file, which defines various robot/cell configurations. This file allows users to switch between different setups by defining a set of parameters for each configuration (e.g., robot type, end effector, and URDF file). The launch files refer to this YAML file to load the required configurations. Example:

    ```sh
    Configurations:

        - ID: "irb120_1"
        Name: "ABB IRB-120 on top of Robot Stand."
        urdf: "irb120.urdf.xacro"
        rob: "irb120"
        ee: "none"

        - ID: "irb120_2"
        Name: "ABB IRB-120 + Schunk EGP-64 Gripper on top of Robot Stand."
        urdf: "irb120_egp64.urdf.xacro"
        rob: "irb120"
        ee: "egp64"
    ```

    In this example, each configuration defines:

    - ID: Unique identifier for the configuration.
    - Name: Descriptive name of the robot setup.
    - urdf: Path to the robot's URDF file.
    - rob: Name of the robot.
    - ee: End effector (if applicable).

- _/urdf_: Contains the URDF (Unified Robot Description Format) files. These files describe the robot's physical properties (joints, links, dimensions) and include references to the robot and end effector models. The URDF defines the geometry, sensors, and controllers of the robot in the simulation environment. Examples:
    
    - irb120.urdf.xacro: URDF for the base robot.
    - irb120_egp64.urdf.xacro: URDF for the robot with the Schunk EGP-64 gripper.

    In these URDF files, the standard urdf files of both the robot and the end-effector (located within the __/robots__ and __/endeffectors__ folders of this repository) are loaded and linked. This approach allows the usage of single URDF files for raw robots and end-effectors in multiple configurations, enhancing scalability, modularity and reusability.

__NOTE: Controller Parameters, Key Specifications and MoveIt!2 Config Files__

One important feature of the ros2srrc repository is its modular architecture, which minimizes duplication and redundancy in robot setup. All the controller parameters and key specifications specific to the robot or end-effector (such as joint limits, PID gains, velocity/position controllers, etc.) are already predefined in the main robot and end-effector folders within the repository. These predefined configurations ensure that:

- Robot-specific parameters: Properties such as joint limits, inertia, and dynamics for robots like the ABB IRB-120 are already set up.
- End-effector-specific parameters: Specifications such as gripper dimensions, actuation constraints, and grasping capabilities for end effectors like the Schunk EGP-64 Gripper are also included.

In addition, the _.rviz_ and _.srdf_ files required for MoveIt!2 configuration have been standardised for every robot+end-effector combination, and included inside the ros2srrc_moveit package.

Features of this modular setup:

- No redefinition is needed: These parameters do not need to be redefined or replicated in the ROS 2 Gazebo or MoveIt!2 packages. The packages simply reference the existing parameters.
- Centralized control: Any updates or changes to the robot or end-effector specifications can be handled in their respective folders without affecting the overall system configuration, maintaining consistency and ease of use across various ROS 2 packages.

This modular approach reduces complexity and makes the system highly maintainable, as different components (robot and end-effector) are managed independently but integrated seamlessly.

## ROS 2 Packages  

__Universal Robots UR5__

Package name: ros2srrc_ur5

Configurations:

- ur5_1: UR5 on top of Robot Stand.
- ur5_2: UR5 + Robotiq 2f-85 gripper on top of Robot Stand.
- ur5_3: UR5 + Robotiq HandE gripper on top of Robot Stand.

This workspace supports UR5 only (no UR5e).
