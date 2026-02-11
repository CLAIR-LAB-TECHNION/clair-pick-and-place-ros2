# Real robot setup (UR)

Before running bringup and sending motion commands to a real UR5, the following are required.

## 1. External Control URCap

The Universal Robots **External Control URCap** must be installed on the robot. The ROS 2 driver and the script it uses (`external_control.urscript`) are designed to work with this URCap. Without the URCap installed, the PC cannot take control of the robot.

- Install the External Control URCap on the robot (from Universal Robots or your distributor).
- On the teach pendant: add the External Control program to your installation, load it, and **start the External Control program** so the robot is ready to accept external control.

## 2. Runtime: External Control program must be running

The robot must be in **External Control** mode—i.e. the External Control program must be **running** on the teach pendant—**before** you launch bringup and send commands. Control is only possible while that program is active; power-on and PolyScope alone are not enough.

**Before starting bringup:** Ensure the External Control program is running on the teach pendant.

## 3. Networking (PC ↔ robot)

External control is Ethernet-based. Connection failures are often due to network or firewall configuration.

- **Topology:** The PC running ROS 2 and the robot must be on the **same subnet** (or otherwise routable).
- **robot_ip:** The `robot_ip` argument to bringup is the robot’s IP address on that network. Use the IP shown on the robot’s teach pendant or your network configuration.
- **Ports:** The driver uses several ports (e.g. reverse_port, script_sender_port, trajectory_port, script_command_port as in the UR driver/URDF). These ports must be **reachable** from the PC and **not blocked** by firewall on either the PC or the robot. See the [Universal Robots ROS 2 driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) and your robot’s xacro/configuration for the exact port list.
- **Firewall:** If connection fails, check that the PC and robot firewall rules allow traffic on the required ports.

## 4. Gripper hardware and connection (OnRobot 2FG7)

For configurations that use the **OnRobot 2FG7** gripper (e.g. config ur5_4):

- **Connection:** The gripper can be connected via the robot’s tool flange I/O or via Ethernet, depending on your hardware. The ROS 2 backend supports control via **URScript** (script sent to the robot) or **XML-RPC** (direct TCP to the robot). When using XML-RPC, the same **robot_ip** as bringup is used to reach the robot; the gripper is controlled through the robot’s interface.
- **Protocol:** The execution layer allows choosing `xmlrpc` or `urscript` (see README Parameters). Ensure your wiring and robot program match the chosen protocol.
- **Object poses on real robot:** There is no Gazebo on real hardware. Poses for pick/place (e.g. for ExecuteProgram, Pick, Place, Hanoi demo) must come from **perception** (e.g. a node publishing `nav_msgs/Odometry` on `/object_poses/<name>`) or from **fixed positions** defined in your program or config. Use `--skip_spawn` for the Hanoi demo and provide or adapt object poses accordingly.

## 5. Safety and before first run

Real-robot operation requires clear safety practice:

- **E-stop:** Ensure an emergency stop is available and that all operators know how and when to use it.
- **First runs:** Use **reduced mode or speed limits** for initial runs (e.g. reduced velocity/scaling on the robot or in MoveIt) until paths and workspace are validated.
- **Workspace and collisions:** Verify workspace limits and collision geometry (e.g. tables, fixtures) in your planning scene and SRDF; check that no unexpected obstacles are in the motion envelope.
- **Before starting:** The operator must ensure the **cell is clear** of personnel and obstacles and that the **robot is in a safe state** before starting bringup and sending motion commands.

This section is procedural; implement safety according to your lab or site requirements.

---

## Real robot (UR + 2FG7) checklist

Before starting the real robot with UR5 + OnRobot 2FG7 (config **ur5_4**):

1. **Network:** PC and robot on same subnet; you have set (or will set) the robot IP and any network changes before starting.
2. **URCap:** External Control URCap installed; External Control program **running** on the teach pendant.
3. **Safety:** E-stop available; cell clear; first runs with reduced speed/limits if needed.
4. **Launch:**  
   `ros2 launch ros2srrc_launch bringup/bringup_ur.launch.py package:=ros2srrc_ur5 config:=ur5_4 robot_ip:=<ROBOT_IP>`
5. **Execution:** Use program `ur5_pick_and_place_2fg7` (or your program with `EndEffector: "onrobot_2fg7"`). After bringup, ExecuteProgram gets `robot_ip` from the bringup params node; no need to pass it again unless bringup is not running.
6. **Object poses:** Provide poses via perception (`/object_poses/<name>`) or fixed positions in your program; no Gazebo on real hardware.
