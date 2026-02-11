# Walkthrough: Making the real UR5 + 2FG7 move

This guide walks you through **everything you need to know** to move the real robot: prerequisites, one-time setup, and the exact steps each time you want to run it.

---

## Part 1 — What you need (overview)

| What | Details |
|------|--------|
| **Hardware** | UR5 arm, OnRobot 2FG7 gripper, PC with Ubuntu 22.04 on the same network as the robot. |
| **Software** | ROS 2 Humble, this workspace built (`colcon build`), Universal Robots ROS 2 driver (`sudo apt install ros-humble-ur`). |
| **On the robot** | External Control URCap installed; you will start the **External Control** program on the teach pendant before each session. |
| **Network** | PC and robot on the **same subnet**. You need the **robot’s IP address** (shown on the teach pendant or in your network config). |

If any of this is missing, complete it before continuing.

---

## Part 2 — One-time setup (do this once)

### 2.1 Install the External Control URCap on the robot

- Get the **External Control URCap** from Universal Robots (or your distributor).
- Install it on the robot controller (follow Universal Robots’ instructions).
- The ROS 2 driver **only works when this URCap is installed and its program is running**. Without it, the PC cannot take control.

### 2.2 Network: PC and robot on same subnet

- Connect the PC and the robot so they are on the **same subnet** (e.g. same Ethernet switch / same WiFi network, or routing configured).
- Note the **robot’s IP address** (e.g. `192.168.1.10`). You will pass this as `robot_ip` when launching bringup.
- Ensure **firewalls** (PC and/or robot) do **not block** the ports used by the UR driver (e.g. 50001–50004; see [Universal Robots ROS 2 driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) for the exact list). If the connection fails, check the firewall first.

### 2.3 Build the workspace and install the UR driver on the PC

```bash
cd ~/dev_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Install the UR driver:

```bash
sudo apt update
sudo apt install -y ros-humble-ur
```

### 2.4 Safety (before first motion)

- **E-stop:** Know where the emergency stop is and how to use it.
- **First runs:** Prefer **reduced speed or scaling** on the robot or in MoveIt until you have verified paths and workspace.
- **Workspace:** Ensure the motion envelope is clear of people and obstacles; check that tables/fixtures match your planning scene.

After this one-time setup, you only need **Part 3** and **Part 4** each time you want to move the robot.

---

## Part 3 — Every time you want to move the robot (order matters)

Do these steps **in order**, every time you start a session.

### Step 1: Start External Control on the teach pendant

- On the **robot teach pendant**: open the **External Control** program (from the URCap you installed).
- **Start / run** that program so the robot is in **External Control** mode and waiting for the PC.
- If you skip this, bringup will not be able to take control and the robot will not move.

### Step 2: (Optional) Set robot IP / network

- If you use DHCP, the robot may have a new IP; check the teach pendant or your network tool.
- If you changed network or robot IP, use that value as `<ROBOT_IP>` in the next step.

### Step 3: Launch bringup on the PC

On the **PC** (in a terminal where you will keep bringup running):

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash

ros2 launch ros2srrc_launch bringup/bringup_ur.launch.py \
  package:=ros2srrc_ur5 \
  config:=ur5_4 \
  robot_ip:=<ROBOT_IP>
```

Replace `<ROBOT_IP>` with your robot’s IP (e.g. `192.168.1.10`).

- This starts: UR driver, MoveIt 2, move/robmove/robpose nodes, and the 2FG7 params node (so `robot_ip` is available for the gripper).
- Wait until bringup has fully started (RViz and move_group are up; no errors in the terminal). Leave this terminal running.

### Step 4: Run something that sends motion

In **another terminal** (with the same `source` as above), run one of the options in **Part 4** below. The robot will only move when you run one of these commands.

---

## Part 4 — How to actually make the robot move

After bringup is running (Part 3), you can use any of the following. All use the same `robot_ip` from bringup for the 2FG7 gripper; you do **not** need to pass `robot_ip` again.

### Option A: Run a task program (e.g. pick-and-place)

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash

ros2 run ros2srrc_execution ExecuteProgram.py \
  package:=ros2srrc_execution \
  program:=ur5_pick_and_place_2fg7
```

- This runs the sequence defined in `ur5_pick_and_place_2fg7` (moves + gripper open/close).
- **Object poses:** On the real robot there is no Gazebo. The program may expect object poses on `/object_poses/<name>`. You must either:
  - Publish those poses (e.g. from a perception node), or  
  - Use a program/YAML that uses **fixed positions** you define.

### Option B: Run the Pick or Place nodes

- **Pick:** `ros2 run ros2srrc_execution pick.py ...` (see README or program args for parameters).
- **Place:** `ros2 run ros2srrc_execution place.py ...`
- Again, object poses must come from perception or fixed poses; no Gazebo.

### Option C: Test the 2FG7 gripper only (no motion)

To check that the gripper is reachable (open/close only, no arm motion):

```bash
ros2 run ros2srrc_execution test_2fg7_connectivity.py \
  --ros-args -p OnRobot2FG7_param_reader.robot_ip:=<ROBOT_IP>
```

(If bringup is running, the script may still pick up `robot_ip` from the bringup params node, depending on node name/namespace.)

### Option D: Use MoveIt / RViz to plan and execute

- Bringup starts **RViz** and **move_group**. You can use the MoveIt interface in RViz to plan and execute motions (e.g. drag the interactive marker, then Execute). The robot will move when you execute a plan.

---

## Part 5 — Summary: minimum to make the robot move

1. **One-time:** URCap installed, PC and robot on same subnet, workspace built, `ros-humble-ur` installed, safety checked.
2. **Every time:**  
   - Start **External Control** on the teach pendant.  
   - Run **bringup** with `config:=ur5_4` and `robot_ip:=<ROBOT_IP>`.  
   - Run **ExecuteProgram** (or Pick/Place, or MoveIt in RViz) in a second terminal.

If something fails, see **Part 6**.

### Stopping (bringdown)

Stop any running program (ExecuteProgram, Pick, Place) first, then press **Ctrl+C** in the bringup terminal. The robot stops when the driver disconnects. Optionally stop the **External Control** program on the teach pendant to release control.

---

## Part 6 — Quick troubleshooting

| Problem | What to check |
|--------|----------------|
| Bringup fails to connect | External Control program **running** on pendant? Robot and PC on same subnet? Correct `robot_ip`? Firewall not blocking UR ports? |
| Robot does not move when I run ExecuteProgram | Is bringup still running in the other terminal? Any errors in the bringup or ExecuteProgram terminal? |
| **RViz Execute: I see motion in RViz but the real robot doesn't move** | See **Part 6.1** below. |
| Gripper does not open/close | 2FG7 connected and powered? For XML-RPC: same `robot_ip` as robot; robot’s 2FG7 server (port 41414) reachable? Try `test_2fg7_connectivity.py` with `robot_ip`. |
| “Service not available” / timeouts | Bringup must be running first so move_group and the 2FG7 params node are up. Start bringup, wait for it to finish loading, then run your program. |

### Part 6.1 — RViz Execute: visualization updates but real robot doesn't move

When you **Plan** in RViz, you see the planned path (orange) animate—that is only a preview. The **real** robot in the scene (and the physical robot) only move when **Execute** sends the trajectory to the controller and the controller runs it on the hardware. If the physical robot never moves when you click Execute, work through the following.

1. **External Control must be running**  
   On the teach pendant, the **External Control** program must be **running** (not just installed). If it is not running, the driver cannot send trajectories to the robot and the real robot will not move.

2. **Use "Plan & Execute"**  
   In the Motion Planning panel, use **Plan & Execute** instead of Plan then Execute. That plans from the **current** robot state and immediately sends the trajectory, which avoids the robot drifting between plan and execute and reduces start-state tolerance issues.

3. **Confirm the trajectory controller is active**  
   In a terminal (with `source install/setup.bash`):
   ```bash
   ros2 control list_controllers
   ```
   You should see `scaled_joint_trajectory_controller` in state **active**. If it is **inactive** or **unconfigured**, bringup did not start the controller correctly; check the bringup terminal for errors.

4. **Confirm the action is available**  
   ```bash
   ros2 action list | grep follow_joint_trajectory
   ```
   You should see something like `/scaled_joint_trajectory_controller/follow_joint_trajectory`. If it is missing, MoveIt has nothing to send the trajectory to.

5. **Watch the bringup terminal when you Execute**  
   When you click Execute in RViz, watch the terminal where you ran **bringup**. MoveIt (move_group) and the controller often log errors there (e.g. execution failed, start state violation, or action rejected). Those messages explain why the real robot did not move.

6. **Start state tolerance**  
   If the robot has moved after you planned (e.g. you moved it by hand or it drifted), Execute can fail because the start state no longer matches. Use **Plan & Execute** so the plan is computed from the current state and executed right away.

If all of the above are correct and the bringup terminal shows no errors on Execute, the trajectory is being sent; then the issue is likely on the robot side (e.g. External Control not running, or a different UR/driver configuration). See [RealRobotSetup.md](RealRobotSetup.md) for network and URCap details.

For more detail on URCap, network, safety, and object poses, see [RealRobotSetup.md](RealRobotSetup.md).
