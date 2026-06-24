#!/usr/bin/python3

"""
move_to_home.py - Move robot to home position

This script moves the robot to its home position using MoveJ action.

Usage:
    ros2 run ros2srrc_execution move_to_home.py
    
Optional arguments:
    speed:=1.0              Movement speed (0.0 to 1.0, default: 0.5)
    clear_objects:=cube_0,cube_1   Comma-separated MoveIt objects to remove before planning
                                   (virtual cubes block home if gripper overlaps the stack)
    profile:=hanoi          High home for Tower of Hanoi (straight elbow + optional transit)
    transit_x:=0.0          Safe PTP waypoint X before MoveJ home (meters, with profile:=hanoi)
    transit_y:=-0.12        Safe PTP waypoint Y
    transit_z:=1.0          Safe PTP waypoint Z (clears tall stacks on peg 0)
    transit_only:=true      Only run safe transit PTP; skip MoveJ home (Hanoi staging)
"""

# Default pick/place ready pose (elbow bent — can sweep low over tall stacks)
DEFAULT_HOME_POSES = [
    {"joint1": 90.0, "joint2": -89.9, "joint3": 90.0, "joint4": -90.0, "joint5": -90.0, "joint6": 0.0},
    {"joint1": 0.0, "joint2": -89.9, "joint3": 0.0, "joint4": 0.0, "joint5": 0.0, "joint6": 0.0},
    {"joint1": 0.0, "joint2": -80.0, "joint3": 0.0, "joint4": 0.0, "joint5": 0.0, "joint6": 0.0},
]

# Hanoi: elbow straight (j3=0) keeps EE high over peg 0 when 4–5 cubes are stacked
HANOI_HOME_POSES = [
    {"joint1": 90.0, "joint2": -89.9, "joint3": 0.0, "joint4": -90.0, "joint5": 0.0, "joint6": 0.0},
    {"joint1": 90.0, "joint2": -89.9, "joint3": 45.0, "joint4": -90.0, "joint5": -90.0, "joint6": 0.0},
    {"joint1": 90.0, "joint2": -89.9, "joint3": 90.0, "joint4": -90.0, "joint5": -90.0, "joint6": 0.0},
    {"joint1": 0.0, "joint2": -89.9, "joint3": 0.0, "joint4": 0.0, "joint5": 0.0, "joint6": 0.0},
]

import sys
import os
import time
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from ament_index_python.packages import get_package_share_directory

# Import robot client
PATH = os.path.join(get_package_share_directory("ros2srrc_execution"), 'python', 'robot')
sys.path.append(PATH)
from robot import RBT

# Import action message
from ros2srrc_data.msg import Action


def AssignArgument(ARGUMENT):
    """Parse command-line arguments in ROS2 style (arg:=value)."""
    ARGUMENTS = sys.argv
    for y in ARGUMENTS:
        if (ARGUMENT + ":=") in y:
            ARG = y.replace((ARGUMENT + ":="), "")
            return ARG
    return None


def _remove_from_moveit(object_name):
    """Remove a collision object so MoveIt can plan when EE overlaps the virtual cube."""
    node = Node("move_to_home_scene_clear")
    collision_pub = node.create_publisher(CollisionObject, "/collision_object", 10)
    scene_pub = node.create_publisher(PlanningScene, "/planning_scene", 10)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.1)

    co = CollisionObject()
    co.header.frame_id = "world"
    co.header.stamp = node.get_clock().now().to_msg()
    co.id = object_name
    co.operation = CollisionObject.REMOVE
    collision_pub.publish(co)
    scene = PlanningScene()
    scene.world.collision_objects.append(co)
    scene.is_diff = True
    scene_pub.publish(scene)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.2)
    node.destroy_node()
    print(f"[move_to_home]: Removed '{object_name}' from MoveIt planning scene.")


def _clear_moveit_objects(names):
    for name in names:
        name = name.strip()
        if name:
            _remove_from_moveit(name)


def _transit_to_safe_height(robot_client, speed, x, y, z):
    """PTP/LIN to a high pose above the pegs before joint-space homing."""
    from ros2srrc_data.msg import Robpose

    pose = Robpose()
    pose.x = float(x)
    pose.y = float(y)
    pose.z = float(z)
    pose.qx, pose.qy, pose.qz, pose.qw = -0.5, 0.5, 0.5, 0.5
    print(
        f"[move_to_home]: Safe transit → x={pose.x:.3f}, y={pose.y:.3f}, z={pose.z:.3f} "
        f"(clear stacks before homing)"
    )
    result = robot_client.RobMove_EXECUTE("PTP", speed, pose)
    if not result["Success"]:
        print("[move_to_home]: PTP transit failed, trying LIN...")
        result = robot_client.RobMove_EXECUTE("LIN", speed, pose)
    if result["Success"]:
        print("[move_to_home]: Safe transit reached.")
    else:
        print(f"[move_to_home]: WARNING - Safe transit failed: {result['Message']}")
    print("")
    return result


def _execute_home_poses(robot_client, action, home_poses, speed):
    """Try each home joint pose in order; return last Move_EXECUTE result."""
    result = {"Success": False, "Message": "no attempt", "ExecTime": -1.0}
    for home_idx, joints in enumerate(home_poses):
        action.movej.joint1 = float(joints["joint1"])
        action.movej.joint2 = float(joints["joint2"])
        action.movej.joint3 = float(joints["joint3"])
        action.movej.joint4 = float(joints["joint4"])
        action.movej.joint5 = float(joints["joint5"])
        action.movej.joint6 = float(joints["joint6"])
        label = "home" if home_idx == 0 else ("fallback home %d" % home_idx)
        print("Moving to %s..." % label)
        print("  Joint values: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f] degrees" % (
            joints["joint1"], joints["joint2"], joints["joint3"],
            joints["joint4"], joints["joint5"], joints["joint6"]))
        print("")

        max_attempts = 3 if home_idx == 0 else 2
        retry_delay_s = 1.5
        for attempt in range(1, max_attempts + 1):
            if attempt > 1:
                print("Retrying %s (%d/%d)..." % (label, attempt, max_attempts))
                time.sleep(retry_delay_s)
            result = robot_client.Move_EXECUTE(action)
            if result["Success"]:
                break
            print("Attempt %d failed: %s" % (attempt, result["Message"]))
        if result["Success"]:
            if home_idx > 0:
                print("(Used fallback pose.)")
            break
        print("")
    return result


def main(args=None):
    """
    Move robot to home (park) position.

    Home: same ready pose as ur5_pick_and_place (faces +Y table, arm up over workspace).
    - joint1: 90.0, joint2: -89.9, joint3: 90.0, joint4: -90.0, joint5: -90.0, joint6: 0.0
    Falls back to simpler vertical home if planning fails.
    """
    
    rclpy.init(args=args)
    
    print("==================================================")
    print("ROS 2 Sim-to-Real Robot Control: Move to Home")
    print("==================================================")
    print("")
    
    # Parse optional arguments
    speed = float(AssignArgument("speed") or "0.5")
    profile = (AssignArgument("profile") or "default").strip().lower()
    transit_only = (AssignArgument("transit_only") or "false").strip().lower() in (
        "1", "true", "yes",
    )
    clear_objects_arg = AssignArgument("clear_objects")
    clear_objects = [n.strip() for n in clear_objects_arg.split(",")] if clear_objects_arg else []
    transit_z_arg = AssignArgument("transit_z")
    transit_x_arg = AssignArgument("transit_x")
    transit_y_arg = AssignArgument("transit_y")
    
    if speed <= 0.0 or speed > 1.0:
        print("ERROR: Speed must be between 0.0 and 1.0")
        print("")
        rclpy.shutdown()
        exit(1)
    
    print(f"Speed: {speed}")
    if profile == "hanoi":
        print("Profile: hanoi (high home + safe transit)")
    if transit_only:
        print("Mode: transit_only (skip MoveJ home)")
    if clear_objects:
        print(f"Clearing MoveIt objects before planning: {clear_objects}")
    print("")
    
    if clear_objects:
        _clear_moveit_objects(clear_objects)
        print("")
    
    # Initialize robot client
    print("Initializing robot client...")
    RobotClient = RBT()
    print("Robot client ready!")
    print("")

    if transit_z_arg is not None:
        transit_z = float(transit_z_arg)
        transit_x = float(transit_x_arg if transit_x_arg is not None else "0.0")
        transit_y = float(transit_y_arg if transit_y_arg is not None else "-0.12")
        _transit_to_safe_height(RobotClient, speed, transit_x, transit_y, transit_z)

    if transit_only:
        result = {"Success": True, "Message": "Transit complete.", "ExecTime": 0.0}
    else:
        home_poses = HANOI_HOME_POSES if profile == "hanoi" else DEFAULT_HOME_POSES

        action = Action()
        action.action = "MoveJ"
        action.speed = speed

        result = _execute_home_poses(RobotClient, action, home_poses, speed)

    # Gripper often overlaps virtual cubes after pick — retry after removing common cube names
    _FALLBACK_CUBE_NAMES = [
        "cube_0", "cube_1", "cube_2", "cube_3", "cube_4",
        "cube_5", "cube_6", "cube_7",
        "cube1", "cube2", "cube3", "cube4", "cube5",
    ]
    if not result["Success"] and not transit_only:
        to_clear = [n for n in _FALLBACK_CUBE_NAMES if n not in clear_objects]
        if to_clear:
            print("Home planning failed; retrying after removing cubes from MoveIt scene...")
            print(f"  Clearing: {to_clear}")
            print("")
            _clear_moveit_objects(to_clear)
            if transit_z_arg is not None:
                _transit_to_safe_height(
                    RobotClient, speed,
                    float(transit_x_arg or "0.0"),
                    float(transit_y_arg or "-0.12"),
                    float(transit_z_arg),
                )
            action = Action()
            action.action = "MoveJ"
            action.speed = speed
            home_poses = HANOI_HOME_POSES if profile == "hanoi" else DEFAULT_HOME_POSES
            result = _execute_home_poses(RobotClient, action, home_poses, speed)
            if result["Success"]:
                print("(Re-add cubes to MoveIt before pick — Hanoi re-syncs automatically.)")

    # Print result
    print("==================================================")
    if result["Success"]:
        print("SUCCESS: Robot moved to home position!")
    else:
        print("FAILED: Could not move to home position!")
    print(f"Message: {result['Message']}")
    print(f"Execution time: {result['ExecTime']}s")
    print("==================================================")

    rclpy.shutdown()
    exit(0 if result["Success"] else 1)


if __name__ == '__main__':
    main()
