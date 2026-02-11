# Real Robot Control: Sockets, Protocols, and Architecture

This document explains **how the real robot is controlled** in this workspace: the communication channels, protocols, ports, and data flow between the control PC and the Universal Robots arm and gripper(s).

---

## 1. Overview

Real robot control is **Ethernet-based**. The control PC (running ROS 2) and the robot must be on the **same subnet** (or routable). Two main subsystems are involved:

| Subsystem | What is controlled | Main protocol / mechanism |
|-----------|--------------------|---------------------------|
| **UR arm** | Joint positions, trajectory execution, state feedback | **RTDE** (Real-Time Data Exchange) + **URScript** over TCP; driven by **External Control URCap** |
| **Gripper** | Open/close, force, speed | **OnRobot 2FG7:** XML-RPC (HTTP) or URScript (TCP). **Robotiq:** proprietary TCP text protocol (SET POS / GET POS) |

The PC **initiates** connections to the robot (and optionally to the gripper, depending on setup). The robot runs the **External Control** program (URCap) so it is ready to accept these connections.

---

## 2. UR Arm Control (Universal Robots)

### 2.1 Role of the External Control URCap

- The **External Control URCap** is a program provided by Universal Robots that must be **installed on the robot** and **running on the teach pendant** before the PC can take control.
- The ROS 2 stack uses a script called **`external_control.urscript`** (from the `ur_client_library` package). This script is **sent to the robot** by the driver and is designed to work **with** the External Control URCap.
- Without the URCap installed and the External Control program **running**, the robot will not accept external connections for motion control.

So the sequence is: **Teach pendant: start External Control program** → robot listens for the PC → **PC launches bringup** → driver connects and sends the script.

### 2.2 Driver and Hardware Interface

- **Package:** `ur_robot_driver` (from Universal Robots ROS 2 driver).
- **Plugin:** `ur_robot_driver/URPositionHardwareInterface` (loaded in the robot’s URDF/xacro when `bringup:=true`).
- This plugin is registered with **ros2_control**; the **controller_manager** loads it together with the **joint_trajectory_controller** (or scaled variant). MoveIt 2 sends joint trajectories to that controller, which then forwards commands to the UR hardware interface. The hardware interface talks to the **real robot over Ethernet** using the ports and protocols below.

### 2.3 Ports (TCP) — All to `robot_ip`

The driver uses **four TCP ports** to communicate with the robot. They are defined in `ur5_ros2control.xacro` and passed into the hardware interface. Defaults:

| Port parameter | Default | Purpose (conceptual) |
|----------------|---------|----------------------|
| **reverse_port** | 50001 | Reverse connection: robot connects back to the PC for some data channels (RTDE or secondary interface, depending on driver version). |
| **script_sender_port** | 50002 | Used to **send the URScript** (e.g. `external_control.urscript`) to the robot. |
| **trajectory_port** | 50003 | Used for **trajectory / motion commands** (e.g. joint targets). |
| **script_command_port** | 50004 | Used for **script commands** (e.g. start/stop, control flow). |

- **Direction:** The PC connects **to** the robot at `robot_ip` on these ports (the robot listens after External Control is started).
- **Firewall:** All these ports must be **reachable** from the PC to the robot and **not blocked** on either side. Many connection failures are due to firewall or wrong subnet.

### 2.4 RTDE (Real-Time Data Exchange)

- **RTDE** is Universal Robots’ protocol for **high-rate state and command exchange** (joint positions, velocities, I/O, etc.).
- The driver uses **recipe files** to define what is sent and received:
  - **Input recipe** (`rtde_input_recipe.txt`): data the **PC sends to the robot** (e.g. target joint positions, digital outputs).
  - **Output recipe** (`rtde_output_recipe.txt`): data the **robot sends to the PC** (e.g. actual joint positions, robot status).
- These files come from the `ur_robot_driver` package and are passed via the launch file into the robot description:
  - `input_recipe_filename` → `ur_robot_driver/resources/rtde_input_recipe.txt`
  - `output_recipe_filename` → `ur_robot_driver/resources/rtde_output_recipe.txt`
- The driver (and the script running on the robot) use RTDE over the same TCP connection(s) as above; the exact mapping (which port carries RTDE vs script vs trajectory) is defined in the Universal Robots driver implementation.

### 2.5 URScript (external_control.urscript)

- **Source:** `ur_client_library` → `resources/external_control.urscript`.
- The driver **sends this script to the robot** over the script_sender_port (and related channels). The script runs **on the robot controller** and:
  - Interfaces with the External Control URCap.
  - Reads/writes RTDE channels (according to the recipes).
  - Executes trajectory points or servoj commands received from the PC.
- So: **PC** → TCP (script_sender_port, trajectory_port, script_command_port, reverse_port) + RTDE ↔ **Robot** (PolyScope + External Control program + external_control.urscript).

### 2.6 Summary: UR arm connection

- **Transport:** TCP over Ethernet to `robot_ip`.
- **Ports:** 50001 (reverse), 50002 (script sender), 50003 (trajectory), 50004 (script command).
- **Protocols:** RTDE (input/output recipes) + URScript (external_control.urscript).
- **Prerequisite:** External Control URCap installed and **External Control program running** on the teach pendant.

---

## 3. Gripper Control

Two gripper backends are used on the real robot in this project: **OnRobot 2FG7** (primary for config ur5_4) and **Robotiq** (e.g. HandE, or 2F-85 if used on real hardware).

### 3.1 OnRobot 2FG7 — Two Protocols

The OnRobot 2FG7 backend (`gripper_onrobot_2fg7.py`) supports two ways to talk to the gripper. In both cases, the **robot’s IP** (`robot_ip`) is used (gripper is reached via the robot’s interface or the same host, depending on wiring).

#### 3.1.1 XML-RPC over HTTP (default)

- **Protocol:** XML-RPC (HTTP POST with XML body).
- **Address:** `http://<robot_ip>:41414`
- **Port:** 41414 (configurable via `xmlrpc_port`).
- **No URScript:** The gripper is controlled by an OnRobot 2FG7 XML-RPC server (on the robot or co-located). The PC sends method calls such as:
  - `twofg_grip_external(id, target_width_mm, target_force, speed)` — close to a width with given force/speed.
  - `twofg_get_max_external_width`, `twofg_get_min_external_width`, `twofg_get_max_force` — query limits.
- **Implementation:** The code builds XML-RPC requests with the Python stdlib (`urllib.request`, `xmlrpc.client`) and POSTs them to `http://robot_ip:41414`. No raw TCP socket is used for XML-RPC; it’s HTTP on top of TCP.

#### 3.1.2 URScript over TCP (optional)

- **Protocol:** Send **URScript** strings to the robot; the robot executes them (e.g. `rq_set_width(110)` to open, `rq_set_width({width})` to close).
- **Port:** 50003 (custom default; configurable via `urscript_port`). Used for script execution to the robot.
- **Implementation:** The code opens a **TCP socket** to `(robot_ip, urscript_port)` (default 50003), sends the script string (with newline), optionally waits `wait_after_cmd`, then closes the socket. So: **TCP socket, one-shot script send**, no persistent connection.
- **Templates:** `open_script_template` and `close_script_template` (e.g. `rq_set_width(110)` for open, `rq_set_width({width})` for close) are filled with width/force/speed and sent when the user calls open/close.

#### 3.1.3 Where `robot_ip` comes from (2FG7)

- Bringup can start an **OnRobot 2FG7 params node** (`onrobot_2fg7_params_node.py`, node name `onrobot_2fg7_bringup_params`) with parameter `robot_ip` set from bringup.
- The 2FG7 backend resolves `robot_ip` in this order: explicit parameter → GetParameters from `onrobot_2fg7_bringup_params` → GetParameters from `ros2srrc_RobMove_Client`. So after bringup, ExecuteProgram usually does **not** need an extra `robot_ip` argument.

### 3.2 Robotiq (HandE / 2F-85) — TCP socket

- **Package:** `ros2_robotiqgripper`; server node exposes a ROS 2 service **`/Robotiq_Gripper`**.
- **Hardware connection:** The server connects to the **gripper’s IP** (not necessarily the robot IP), port **63352**.
- **Protocol:** **Proprietary text over TCP**:
  - **Close:** Send `SET POS <value>\n` (value 1–255); wait; then `GET POS\n` and parse reply for position.
  - **Open:** Same socket, different position value and parsing.
- **Socket:** Standard TCP `socket.socket(socket.AF_INET, socket.SOCK_STREAM)`, timeout 3 s, connect to `(HOST, 63352)`, send/recv, then close (per request).
- **When used:** Bringup starts the Robotiq server only for configurations with HandE (e.g. ur5_3). For real Robotiq 2F-85 (ur5_2), the server must be started separately (e.g. with the gripper’s IP as parameter).

---

## 4. Network Topology and Requirements

- **PC:** Runs ROS 2 (bringup, MoveIt 2, ExecuteProgram, motion nodes, gripper clients).
- **Robot:** UR controller with PolyScope; External Control URCap program **running**; listens on ports 50001–50004 (and 50003 for 2FG7 URScript when used for gripper).
- **Same subnet:** PC and robot IPs must be on the same subnet (or routing must allow the PC to reach `robot_ip` and the required ports).
- **robot_ip:** The IP of the **robot controller** (as shown on the teach pendant or your network config). Passed to bringup as `robot_ip:=<ROBOT_IP>` and into the URDF/xacro and (for 2FG7) the params node.
- **Firewall:** Allow TCP (and any required UDP if applicable) for:
  - 50001, 50002, 50003, 50004 (UR driver).
  - 50003 (if using 2FG7 URScript).
  - 41414 (if using 2FG7 XML-RPC to robot_ip).
  - 63352 (only if Robotiq gripper is on a separate IP and the PC talks to it directly).

---

## 5. End-to-End Control Flow (Real Robot)

1. **Operator:** Starts **External Control** on the teach pendant; robot is ready to accept connections.
2. **Operator:** Launches bringup:  
   `ros2 launch ros2srrc_launch bringup/bringup_ur.launch.py package:=ros2srrc_ur5 config:=ur5_4 robot_ip:=<ROBOT_IP>`
3. **Bringup:**
   - Loads robot description with `ur_robot_driver/URPositionHardwareInterface`, `robot_ip`, script filename, RTDE recipes, and port args (reverse_port, script_sender_port, trajectory_port, script_command_port).
   - Starts **controller_manager** (ros2_control) with that description and `controller_ur.yaml` (joint_trajectory_controller, joint_state_broadcaster, etc.).
   - Starts **MoveIt 2** (move_group, planning, trajectory execution).
   - For config ur5_4: starts **onrobot_2fg7_params_node** with `robot_ip` so the 2FG7 backend can get it later.
4. **Driver:** Connects to the robot at `robot_ip` on the four ports; sends `external_control.urscript`; runs RTDE input/output according to the recipes; receives joint state and sends trajectory commands.
5. **User:** Runs a program (e.g. ExecuteProgram with `ur5_pick_and_place_2fg7`). ExecuteProgram sends:
   - **Motion:** Goals to **/Move** and **/Robmove** action servers → move/robmove nodes → MoveIt 2 → **joint_trajectory_controller** → **URPositionHardwareInterface** → **TCP/RTDE/URScript** → robot arm.
   - **Gripper:** Open/close via **OnRobot2FG7Backend** → XML-RPC to `http://robot_ip:41414` or URScript to `robot_ip:50003`.
6. **Robot:** Moves joints and runs gripper commands; sends back joint states over RTDE; driver publishes `/joint_states`; MoveIt and TF use that for planning and visualization.

---

## 6. Quick Reference Table

| Component | Protocol / mechanism | Port(s) | Direction | Notes |
|-----------|----------------------|--------|-----------|--------|
| UR arm – script | URScript over TCP | 50002 (script_sender) | PC → robot | external_control.urscript |
| UR arm – trajectory | Driver protocol over TCP | 50003 (trajectory) | PC → robot | Joint targets |
| UR arm – script commands | TCP | 50004 (script_command) | PC → robot | Control flow |
| UR arm – reverse | TCP | 50001 (reverse_port) | Robot → PC (reverse) | State / data |
| UR arm – state/commands | RTDE | Over same connection(s) | Bidirectional | Recipes define payloads |
| OnRobot 2FG7 – XML-RPC | HTTP (XML-RPC) | 41414 | PC → robot_ip | twofg_grip_external, twofg_get_* |
| OnRobot 2FG7 – URScript | TCP, one-shot script | 50003 | PC → robot_ip | rq_set_width(...) etc. |
| Robotiq | TCP text (SET POS / GET POS) | 63352 | PC → gripper IP | Robotiq server in this repo |

---

## 7. References in This Workspace

- **UR driver and ports:** `ros2srrc_robots/ur5/urdf/ur5_ros2control.xacro` (args and hardware params).
- **Script and RTDE paths:** `ros2srrc_launch/bringup/bringup_ur.launch.py` (ur_client_library, ur_robot_driver resources).
- **OnRobot 2FG7:** `ros2srrc_execution/python/endeffector/gripper_onrobot_2fg7.py` (XML-RPC and URScript).
- **Robotiq server (socket):** `ros2_RobotiqGripper/python/server.py`.
- **Setup and checklist:** `doc/RealRobotSetup.md`, `doc/RealRobotWalkthrough.md`.
- **Audit (architecture, URCap, ports):** `AUDIT_REPORT_UR_ROS2_EXTERNAL_CONTROL.md`.
