# OnRobot 2FG7 setup (davedovrat/onrobot_2fg7)

Real-robot gripper control for config **ur5_4** uses [onrobot_2fg7](https://github.com/davedovrat/onrobot_2fg7) and [onrobot_2fg7_interfaces](https://github.com/davedovrat/onrobot_2fg7_interfaces). The gripper is controlled via **XML-RPC on the robot at port 41414** (OnRobot 2FG7 URCap on the teach pendant).

**No RS485 Tool Communication URCap is required** for this path.

## Install packages

From the workspace root:

```console
cd ~/clair-pick-and-place-ros2
vcs import src --input repos/onrobot_2fg7.repos
colcon build --symlink-install
source install/setup.bash
```

## Hardware setup (UR + OnRobot 2FG7)

1. Mount the OnRobot 2FG7 on the UR tool flange.
2. Install the **OnRobot 2FG7 URCap** on the teach pendant (from OnRobot / your lab setup).
3. Ensure the 2FG7 XML-RPC server is reachable on the robot IP, port **41414**:

```console
nc -zv <ROBOT_IP> 41414
```

## Bringup and test

Config **ur5_4** starts:

- UR arm + MoveIt (existing stack)
- `onrobot_2fg7` grip service (`/grip`) and status publisher (`/onrobot_2fg7_status`)

```console
ros2 launch ros2srrc_launch bringup_ur.launch.py package:=ros2srrc_ur5 config:=ur5_4 robot_ip:=<ROBOT_IP>
```

Test gripper:

```console
ros2 run ros2srrc_execution test_2fg7_connectivity.py
ros2 topic echo /onrobot_2fg7_status
ros2 service call /grip onrobot_2fg7_interfaces/srv/Grip "{gap: 60.0}"
```

## Execution layer

Programs use `EndEffector: "onrobot_2fg7"`. Example: `ur5_pick_and_place_onrobot.yaml`

MoveIt still uses Robotiq 2F-85 geometry for planning; the physical gripper is OnRobot 2FG7.

## Validated peg positions (real robot)

Outer table marks for Hanoi (world frame, stand center = origin):

| Mark | Hanoi index | X (m) | Y (m) | MoveIt Z (m) |
|------|-------------|-------|-------|--------------|
| peg0 (left) | 0 | −0.15 | −0.12 | 0.84 |
| peg3 (right) | 2 | +0.15 | −0.12 | 0.84 |

Distances from modeled stand edges (84 cm × 184.4 cm, robot on −Y edge):

- **peg0 & peg3:** 27 cm from −X edge, 27 cm from +X edge
- **peg0 & peg3:** 80.2 cm from −Y edge (robot side), 104.2 cm from +Y edge
- **peg0 ↔ peg3:** 30 cm apart (center to center)

Hanoi demo: `ros2 run ros2srrc_execution hanoi_tower_demo.py --peg_layout real_2fg7 --skip_spawn --ee_type onrobot_2fg7 ...`

## Legacy RG2/RG6 (serial / RS485)

For OnRobot **RG2/RG6** over Tool I/O, use `ee_driver: onrobot_ros2` in a separate config and see the older UR_OnRobot stack (`repos/onrobot.repos`).
