# OnRobot ROS 2 setup (UR_OnRobot_ROS2)

Real-robot gripper control uses [UR_OnRobot_ROS2](https://github.com/tonydle/UR_OnRobot_ROS2) and [OnRobot_ROS2_Driver](https://github.com/tonydle/OnRobot_ROS2_Driver). Supported grippers: **RG2** and **RG6** (Modbus serial via UR Tool I/O).

## Install driver packages

From the workspace root:

```console
cd ~/clair-pick-and-place-ros2
vcs import src --input repos/onrobot.repos --recursive
sudo apt install -y libnet1-dev ros-humble-ur
rosdep install -y --from-paths src --ignore-src
colcon build --symlink-install
source install/setup.bash
```

The OnRobot driver is cloned with `--recurse-submodules` via its own `required.repos` if needed; if Modbus submodule is missing:

```console
cd src/onrobot_driver && git submodule update --init --recursive
```

## Hardware setup (UR e-Series)

Follow the [UR_OnRobot_ROS2 hardware setup](https://github.com/tonydle/UR_OnRobot_ROS2#hardware-setup):

1. Connect the OnRobot Quick Changer to the UR Tool I/O.
2. Install the [RS485 Daemon URCap](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap) on the teach pendant and restart the robot.
3. Set Tool I/O (Installation → General → Tool I/O):
   - Controlled by: **User**
   - Baud rate: **1M**, parity **Even**, stop bits **One**
   - RX idle chars: **1.5**, TX idle chars: **3.5**
   - Tool output voltage: **24V**
   - Digital outputs 0/1: **Sinking (NPN)**

**Note:** The Robotiq URCap and RS485 URCap cannot run together on some firmware versions.

## Gripper model (RG2 vs RG6)

Set in `packages/ros2srrc_ur5/config/configurations.yaml` under config **ur5_4**:

```yaml
onrobot_type: "rg2"   # or "rg6"
```

## Bringup and test

Config **ur5_4** starts:

- UR arm + MoveIt (existing stack)
- `tool_communication.py` (UR Tool I/O → `/tmp/ttyUR`)
- `onrobot_driver` sidecar (`/onrobot` namespace, `finger_width_controller`)

```console
ros2 launch ros2srrc_launch bringup/bringup_ur.launch.py package:=ros2srrc_ur5 config:=ur5_4 robot_ip:=<ROBOT_IP>
```

Test gripper:

```console
ros2 run ros2srrc_execution test_onrobot_ros2_connectivity.py
ros2 topic echo /onrobot/joint_states
```

Manual command (metres):

```console
ros2 topic pub --once /onrobot/finger_width_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.085]}"
```

## Execution layer

Programs and pick/place use `EndEffector: "onrobot_ros2"`. The backend publishes finger width to `/onrobot/finger_width_controller/commands`.

Example program: `ur5_pick_and_place_onrobot.yaml`

MoveIt still uses Robotiq 2F-85 geometry for planning (same approximation as before); the physical gripper is OnRobot RG2/RG6.
