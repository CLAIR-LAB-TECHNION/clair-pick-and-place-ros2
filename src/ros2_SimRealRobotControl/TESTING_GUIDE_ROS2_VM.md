# Testing Guide: Robot Movement Actions in ROS 2 VM

---

## Prerequisites Setup

### 1. VM Requirements
- **Operating System**: Ubuntu 22.04 Desktop
- **RAM**: At least 4GB (8GB preferred for better performance)
- **Disk Space**: 20GB+ free space
- **Graphics**: 3D acceleration enabled (for Gazebo visualization)

### 2. Initial System Setup

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Git
sudo apt install git

# Configure Git (optional)
git config --global user.name "YourName"
git config --global user.email "your.email@example.com"
```

### 3. Install ROS 2 Humble

```bash
# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop -y

# Source ROS 2 in .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. Install Required Dependencies

```bash
# MoveIt!2
sudo apt install ros-humble-moveit -y

# Development tools
sudo apt install ros-dev-tools -y
sudo apt install ros-humble-xacro -y

# ROS 2 Control
sudo apt install ros-humble-ros2-control -y
sudo apt install ros-humble-ros2-controllers -y
sudo apt install ros-humble-gripper-controllers -y

# Gazebo
sudo apt install gazebo -y
sudo apt install ros-humble-gazebo-ros2-control -y
sudo apt install ros-humble-gazebo-ros-pkgs -y

# CycloneDDS (recommended for MoveIt!2)
sudo apt install ros-humble-rmw-cyclonedds-cpp -y
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### 5. Create Workspace

```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```

### 6. Install Required IFRA Packages

```bash
# IFRA packages
git clone https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git
git clone https://github.com/IFRA-Cranfield/IFRA_ObjectPose.git
git clone https://github.com/IFRA-Cranfield/IFRA_LinkPose.git
git clone https://github.com/IFRA-Cranfield/ros2_RobotiqGripper.git

# Your workspace (clone your repository)
git clone https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.git -b humble
# OR if you have it locally, copy it to ~/dev_ws/src/
```

### 7. Install MoveIt!2 Improved Header

```bash
# Copy the improved header to MoveIt!2 installation
# (Follow instructions in include/README.md)
sudo cp ~/dev_ws/src/ros2_SimRealRobotControl/include/move_group_interface_improved.h /opt/ros/humble/include/moveit/move_group_interface/
```

### 8. Build Workspace

```bash
cd ~/dev_ws
colcon build --symlink-install
source install/setup.bash

# Add to .bashrc
echo "source ~/dev_ws/install/setup.bash" >> ~/.bashrc
```

---

## Testing the Movement Actions

### Test 1: Launch Simulation + MoveIt!2

**Open Terminal 1:**

```bash
# Launch Gazebo + MoveIt!2 for UR5
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_ur5 config:=ur5_2
```

**What to Expect:**
- ✅ Gazebo window opens with UR5 robot
- ✅ RViz opens showing robot model
- ✅ Terminal shows "MoveIt!2 ready" messages
- ⏱️ Wait 10-20 seconds for full initialization

**Note:** Valid config IDs for `ros2srrc_ur5` are `ur5_1` (no EE), `ur5_2` (Robotiq 2F-85), `ur5_3` (Robotiq HandE), `ur5_4` (2FG7). Example:
```bash
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_ur5 config:=ur5_2
```

---

### Test 2: Monitor Robot State

**Open Terminal 2:**

```bash
# Check current joint values
ros2 run ros2srrc_execution RobotState.py
```

**What to Expect:**
```
Current Robot Joint States:
Joint 1: 0.0 degrees
Joint 2: -90.0 degrees
Joint 3: 0.0 degrees
Joint 4: -90.0 degrees
Joint 5: 0.0 degrees
Joint 6: 0.0 degrees
```

---

### Test 3: Monitor End-Effector Pose

**Open Terminal 3:**

```bash
# Monitor real-time end-effector pose
ros2 topic echo /Robpose
```

**What to Expect:**
```
---
x: 0.0
y: 0.0
z: 0.5
qx: 0.0
qy: 0.0
qz: 0.0
qw: 1.0
---
```

**Press Ctrl+C to stop monitoring.**

---

### Test 4: Test MoveJ (Joint Space Movement)

**In Terminal 2 or new terminal:**

```bash
# Move to home position
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveJ', movej: {joint1: 0.0, joint2: -90.0, joint3: 0.0, joint4: -90.0, joint5: 0.0, joint6: 0.0}, speed: 1.0}"
```

**What to Expect:**
- ✅ Robot moves smoothly to specified joint angles
- ✅ Terminal shows: `result: "SUCCESS"` or `result: "PLANNING: OK"`
- ⏱️ Robot reaches target position in 2-5 seconds
- 👁️ In RViz: robot model moves to new configuration
- 👁️ In Gazebo: robot physically moves

**Try different positions:**
```bash
# Move to a different configuration
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveJ', movej: {joint1: 90.0, joint2: -90.0, joint3: 90.0, joint4: -90.0, joint5: -90.0, joint6: 0.0}, speed: 1.0}"
```

---

### Test 5: Test MoveL (Linear Cartesian Movement)

```bash
# Move 0.1m down in Z direction
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveL', movel: {x: 0.0, y: 0.0, z: -0.1}, speed: 0.5}"
```

**What to Expect:**
- ✅ Robot moves in a straight line downward
- ✅ Orientation remains constant
- ⏱️ Slower than MoveJ (linear paths are more constrained)
- 📝 Terminal shows planning and execution feedback

**Try different directions:**
```bash
# Move in X direction
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveL', movel: {x: 0.1, y: 0.0, z: 0.0}, speed: 0.5}"

# Move in Y direction
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveL', movel: {x: 0.0, y: 0.1, z: 0.0}, speed: 0.5}"
```

---

### Test 6: Test MoveR (Single Joint Rotation)

**First, check your joint names:**
```bash
ros2 topic echo /joint_states
```

**Then test single joint rotation:**
```bash
# Rotate joint1 by 45 degrees
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveR', mover: {joint: 'shoulder_pan_joint', value: 45.0}, speed: 1.0}"
```

**What to Expect:**
- ✅ Only joint1 rotates
- ✅ Other joints stay in place
- ⏱️ Fast execution
- 💡 Useful for fine adjustments

**Note:** Joint names may vary. Common UR5 joint names:
- `shoulder_pan_joint` (joint1)
- `shoulder_lift_joint` (joint2)
- `elbow_joint` (joint3)
- `wrist_1_joint` (joint4)
- `wrist_2_joint` (joint5)
- `wrist_3_joint` (joint6)

---

### Test 7: Test MoveROT (End-Effector Rotation)

```bash
# Rotate end-effector 90 degrees around Z-axis (roll)
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveROT', moverot: {yaw: 0.0, pitch: 0.0, roll: 90.0}, speed: 1.0}"
```

**What to Expect:**
- ✅ End-effector rotates while position stays roughly the same
- ✅ Orientation changes visibly
- 💡 Useful for reorienting gripper

**Try different rotations:**
```bash
# Yaw rotation (around Z-axis)
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveROT', moverot: {yaw: 45.0, pitch: 0.0, roll: 0.0}, speed: 1.0}"

# Pitch rotation (around Y-axis)
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveROT', moverot: {yaw: 0.0, pitch: 30.0, roll: 0.0}, speed: 1.0}"
```

---

### Test 8: Test RobMove (PTP - Point-to-Point)

```bash
# Move to specific pose (PTP - fast)
ros2 action send_goal -f /Robmove ros2srrc_data/action/Robmove "{type: 'PTP', speed: 1.0, x: 0.3, y: 0.2, z: 0.4, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"
```

**What to Expect:**
- ✅ Robot moves to exact pose
- ⏱️ Fast, optimal path
- ✅ Terminal shows: `success: true`

**Get current pose first:**
```bash
# In Terminal 3, check current pose
ros2 topic echo /Robpose --once
```

**Then use those values to move to a nearby position.**

---

### Test 9: Test RobMove (LIN - Linear)

```bash
# Move to pose in straight line (LIN - precise)
ros2 action send_goal -f /Robmove ros2srrc_data/action/Robmove "{type: 'LIN', speed: 0.5, x: 0.4, y: 0.2, z: 0.3, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"
```

**What to Expect:**
- ✅ Robot follows straight-line path
- ⏱️ Slower but more precise
- 💡 Good for approach/retract motions

**Compare PTP vs LIN:**
```bash
# First move with PTP (fast)
ros2 action send_goal -f /Robmove ros2srrc_data/action/Robmove "{type: 'PTP', speed: 1.0, x: 0.3, y: 0.2, z: 0.4, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"

# Then move back with LIN (precise)
ros2 action send_goal -f /Robmove ros2srrc_data/action/Robmove "{type: 'LIN', speed: 0.5, x: 0.2, y: 0.1, z: 0.3, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"
```

---

### Test 10: Execute Complete Demo Program

```bash
# Run the pre-built demo program
ros2 run ros2srrc_execution ExecuteProgram.py package:="ros2srrc_execution" program:="ur5_demo"
```

**What to Expect:**
- ✅ Robot executes all 13 steps from `ur5_demo.yaml`
- 📝 You'll see output like:
  ```
  Step N1: [UR5-Demo]: MoveJ - HomePosition.
  Execution SUCCESSFUL!
  Step N2: [UR5-Demo]: MoveJ - Orientate.
  Execution SUCCESSFUL!
  ...
  ```
- ⏱️ Robot performs sequence automatically
- ⏱️ Takes 1-2 minutes to complete
- ✅ Robot returns to home position at end

**The demo includes:**
- MoveJ movements (home position, orientation)
- MoveL movements (linear movements in X, Y, Z)
- MoveR movements (single joint rotations)

---

### Test 11: Dry-Run Full Stack in Simulation (Pick/Place + Hanoi)

Validates the full stack in Gazebo with the Robotiq/Parallel end-effector: no crashes, gripper open/close unchanged.

**Terminal 1 – Launch simulation + MoveIt!2:**
```bash
source ~/dev_ws/install/setup.bash
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_ur5 config:=ur5_2
```
Wait until Gazebo and RViz are up and you see MoveIt!2 ready.

**Terminal 2 – Spawn table and cube (required for pick/place):**
```bash
source ~/dev_ws/install/setup.bash
# Spawn table (box) – center at (0, 0.48, 0.25), top surface at z ≈ 0.5 m
ros2 run ros2srrc_execution SpawnObjectMoveIt.py --package ros2srrc_objects --urdf box.urdf.xacro \
  --name table1 --x 0.0 --y 0.48 --z 0.25 --size_x 1.0 --size_y 0.8 --size_z 0.50 --mass 50.0 --color white
# Spawn cube on table – 5 cm cube, center at (0.15, 0.48, 0.525) so it sits on table top
ros2 run ros2srrc_execution SpawnObjectMoveIt.py --package ros2srrc_objects --urdf cube.urdf.xacro \
  --name cube1 --x 0.15 --y 0.48 --z 0.525 --size 0.05 --color red
```

**Terminal 2 – Pick/place (after spawn):**
```bash
source ~/dev_ws/install/setup.bash
# Pick at cube pose (ee_type=ParallelGripper by default)
ros2 run ros2srrc_execution pick_manual.py x:=0.15 y:=0.48 z:=0.50 qx:=-0.5 qy:=0.5 qz:=0.5 qw:=0.5
# Place at another pose
ros2 run ros2srrc_execution place_manual.py x:=0.20 y:=0.48 z:=0.50 qx:=-0.5 qy:=0.5 qz:=0.5 qw:=0.5
```
Confirm no crashes and that the gripper opens/closes as expected.

**Optional – pick (uses /object_poses/cube1 from the spawned cube):**
```bash
ros2 run ros2srrc_execution pick.py object:=cube1
# Then place, e.g.:
ros2 run ros2srrc_execution place.py x:=0.20 y:=0.48 z:=0.50
```

**Terminal 2 – Hanoi demo (full dry-run):**
```bash
source ~/dev_ws/install/setup.bash
ros2 run ros2srrc_execution hanoi_tower_demo.py
```
Defaults: 3 cubes, spawns table and cubes, uses `ParallelGripper` (Gazebo). Let it run; confirm no crashes and gripper behavior is correct.

**What to expect:**
- No crashes; robot moves between pegs; gripper open/close unchanged from previous behavior.

---

### Test 12: Real-Robot Pose Path (ROS-only, no Gazebo)

Proves that the “known poses, no perception” path works: `pick` subscribes to `/object_poses/<name>` and receives a pose published by `hanoi_publish_pose.py` (e.g. for real-robot Hanoi). **Start Terminal 1 first** so the subscriber is active, then publish from Terminal 2.

**Terminal 1 – Start pick in pose-only mode (subscribes, then exits after receiving pose):**
```bash
source ~/dev_ws/install/setup.bash
ros2 run ros2srrc_execution pick.py object:=cube_0 pose_only:=true
```
You should see: `[Pick Auto]: Waiting for object pose from /object_poses/cube_0...` (waits up to 5 seconds).

**Terminal 2 – Publish one pose for that object (same name `cube_0`) and exit:**
```bash
source ~/dev_ws/install/setup.bash
ros2 run ros2srrc_execution hanoi_publish_pose.py name:=cube_0 x:=0.1 y:=0.58 z:=0.30
```
Terminal 1 should then show receipt and exit.

**What to expect in Terminal 1:**
- `Received pose for cube_0: x=0.100, y=0.580, z=0.300`
- `[Pick Auto]: Object pose retrieved successfully!`
- `[Pick Auto]: pose_only:=true -> Exiting after pose receipt (no robot/plan/execute).`
- Process exits with code 0.

This validates that the real-robot pose path works: poses can be published to `/object_poses/<name>` (e.g. by Hanoi or any node) and `pick` receives them and would proceed to plan when run without `pose_only:=true`.

---

### Validate frames (before using real robot)

Before you touch the UR5 with published poses (e.g. Hanoi with `hanoi_publish_pose` or `hanoi_pose_publisher`), ensure the **Odometry** you publish uses the correct **header.frame_id**.

- **This project’s MoveIt setup** (from the SRDF): the robot is attached to the world via a fixed virtual joint: `parent_frame="world"`, so the **planning frame is `world`**.
- **Required:** Any `nav_msgs/Odometry` on `/object_poses/<name>` must use **`header.frame_id = "world"`** so that (x, y, z) are in the same frame MoveIt uses.  
- **`hanoi_publish_pose.py`** and **`hanoi_pose_publisher.py`** already set `frame_id="world"`; do not change them unless your robot’s planning frame is different.
- **If the frame is wrong**, the robot will move to the wrong physical location even if everything else is correct.  
- **Runtime check:** When `pick` receives a pose, it checks the message’s `frame_id`. If it is not `"world"`, it logs a **WARNING** that the robot may move to the wrong location.

If your real-robot setup uses a different planning frame (e.g. `base_link`), you must either publish poses in that frame or ensure a TF from your frame to the planning frame so that coordinates align with MoveIt.

---

## Quick Test Sequence

Run this sequence to test all movements quickly:

**Terminal 1 - Launch Simulation:**
```bash
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_ur5 config:=ur5_2
```

**Terminal 2 - Test Movements (wait for Terminal 1 to finish initializing):**
```bash
# 1. MoveJ - Home position
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveJ', movej: {joint1: 0.0, joint2: -90.0, joint3: 0.0, joint4: -90.0, joint5: 0.0, joint6: 0.0}, speed: 1.0}"

# 2. MoveL - Linear down
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveL', movel: {x: 0.0, y: 0.0, z: -0.1}, speed: 0.5}"

# 3. MoveL - Linear up
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveL', movel: {x: 0.0, y: 0.0, z: 0.1}, speed: 0.5}"

# 4. RobMove PTP - Fast movement
ros2 action send_goal -f /Robmove ros2srrc_data/action/Robmove "{type: 'PTP', speed: 1.0, x: 0.3, y: 0.2, z: 0.4, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"

# 5. RobMove LIN - Precise movement
ros2 action send_goal -f /Robmove ros2srrc_data/action/Robmove "{type: 'LIN', speed: 0.5, x: 0.2, y: 0.1, z: 0.3, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"

# 6. MoveROT - Rotate end-effector
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveROT', moverot: {yaw: 0.0, pitch: 0.0, roll: 45.0}, speed: 1.0}"

# 7. Return to home
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveJ', movej: {joint1: 0.0, joint2: -90.0, joint3: 0.0, joint4: -90.0, joint5: 0.0, joint6: 0.0}, speed: 1.0}"
```

---

## Expected Behavior Summary

| Test | Expected Time | Visual Result | Use Case |
|------|---------------|---------------|----------|
| **MoveJ** | 2-5 seconds | Smooth joint movement | Fast repositioning |
| **MoveL** | 3-8 seconds | Straight-line motion | Precise linear approach |
| **MoveR** | 1-3 seconds | Single joint rotates | Fine adjustments |
| **MoveROT** | 2-5 seconds | End-effector rotates | Orientation correction |
| **RobMove PTP** | 2-5 seconds | Fast optimal path | Quick pose changes |
| **RobMove LIN** | 4-10 seconds | Precise straight line | Accurate placement |

---

## Troubleshooting

### Issue: Gazebo doesn't open / crashes

**Symptoms:**
- Gazebo window doesn't appear
- Gazebo crashes immediately
- Black screen in Gazebo

**Solutions:**
```bash
# Check if 3D acceleration is enabled in VM settings
# Increase VM RAM to 4GB+
# Try software rendering:
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_ur5 config:=ur5_2
```

**VM Settings to Check:**
- Enable 3D acceleration in VM settings
- Allocate at least 4GB RAM
- Enable hardware acceleration if available

---

### Issue: "Action server not found"

**Symptoms:**
```
ERROR: Action '/Move' not found
```

**Solutions:**
```bash
# Make sure moveit2.launch.py is running
# Check action server:
ros2 action list
# Should see: /Move, /Robmove

# If not, restart the launch file
```

---

### Issue: "Planning failed"

**Symptoms:**
```
result: "PLANNING: FAILED"
```

**Solutions:**
- ✅ Check if robot is in valid configuration
- ✅ Try slower speed (0.3-0.5)
- ✅ Check for collisions in RViz
- ✅ Move to a known good position first:
  ```bash
  ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveJ', movej: {joint1: 0.0, joint2: -90.0, joint3: 0.0, joint4: -90.0, joint5: 0.0, joint6: 0.0}, speed: 1.0}"
  ```

---

### Issue: Robot doesn't move

**Symptoms:**
- Action succeeds but robot doesn't move
- No error messages

**Solutions:**
```bash
# Check if controllers are running:
ros2 control list_controllers

# Check joint states:
ros2 topic echo /joint_states

# Verify MoveIt!2 is connected:
ros2 topic echo /move_group/status

# Check if simulation is paused in Gazebo
```

---

### Issue: "Package not found"

**Symptoms:**
```
ERROR: Package 'ros2srrc_execution' not found
```

**Solutions:**
```bash
# Make sure workspace is built and sourced:
cd ~/dev_ws
colcon build
source install/setup.bash

# Check if package exists:
ros2 pkg list | grep ros2srrc
```

---

### Issue: Slow performance in VM

**Symptoms:**
- Robot movements are very slow
- Gazebo is laggy

**Solutions:**
- ✅ Increase VM RAM to 8GB+
- ✅ Allocate more CPU cores to VM
- ✅ Disable unnecessary visual effects
- ✅ Use software rendering if hardware acceleration isn't available:
  ```bash
  export LIBGL_ALWAYS_SOFTWARE=1
  ```

---

## Verification Checklist

Before testing, verify:

- [ ] ROS 2 Humble is installed and sourced
- [ ] Workspace is built successfully (`colcon build` completed)
- [ ] All dependencies are installed
- [ ] MoveIt!2 improved header is installed
- [ ] IFRA packages are cloned and built
- [ ] VM has sufficient resources (RAM, disk space)
- [ ] Gazebo can launch successfully

**Quick verification:**
```bash
# Check ROS 2 installation
ros2 --version

# Check if packages are available
ros2 pkg list | grep ros2srrc

# Check if actions are available (after launching)
ros2 action list
```

---

## Next Steps

After successfully testing all movement types:

1. **Create Custom Programs**: Modify or create YAML files for your specific tasks
2. **Test Pick and Place**: Implement pick-and-place sequences
3. **Add Objects**: Spawn objects in Gazebo for manipulation
4. **Integrate Vision**: Add camera and object detection
5. **Real Robot**: Test on physical UR5 robot (when available)

---

## Additional Resources

- **ROS 2 Humble Documentation**: https://docs.ros.org/en/humble/
- **MoveIt!2 Documentation**: https://moveit.picknik.ai/humble/index.html
- **IFRA-Cranfield Repository**: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl
- **Program Execution Guide**: See `instructions/ProgramExecution.md`
- **Robot Operation Guide**: See `instructions/RobotOperation.md`

---

## Notes

- **Speed Parameter**: Always use values between 0.01 and 1.0 (1.0 = maximum speed)
- **Safety**: Start with slower speeds (0.3-0.5) when testing new movements
- **Coordinates**: All positions are relative to robot base frame
- **Angles**: Joint angles are in degrees, Euler angles in degrees, quaternions are unit quaternions
- **Timeouts**: Some actions may timeout if planning takes too long - try simpler movements first

---

**Last Updated**: Based on workspace analysis  
**Tested On**: ROS 2 Humble, Ubuntu 22.04  
**Status**: Ready for testing
