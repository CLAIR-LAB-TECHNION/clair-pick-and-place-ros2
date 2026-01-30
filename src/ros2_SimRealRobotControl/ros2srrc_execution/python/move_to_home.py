#!/usr/bin/python3

"""
move_to_home.py - Move robot to home position

This script moves the robot to its home position using MoveJ action.

Usage:
    ros2 run ros2srrc_execution move_to_home.py
    
Optional arguments:
    speed:=1.0    Movement speed (0.0 to 1.0, default: 1.0)
"""

import sys
import os
import time
import rclpy
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


def main(args=None):
    """
    Move robot to home (park) position.

    Home: joint2 = -89.9 first (near vertical); fallbacks -80, -70, -60 if planning fails.
    - joint1: 0.0, joint2: -89.9, joint3/4/5/6: 0.0
    """
    
    rclpy.init(args=args)
    
    print("==================================================")
    print("ROS 2 Sim-to-Real Robot Control: Move to Home")
    print("==================================================")
    print("")
    
    # Parse optional arguments
    speed = float(AssignArgument("speed") or "1.0")
    
    if speed <= 0.0 or speed > 1.0:
        print("ERROR: Speed must be between 0.0 and 1.0")
        print("")
        rclpy.shutdown()
        exit(1)
    
    print(f"Speed: {speed}")
    print("")
    
    # Initialize robot client
    print("Initializing robot client...")
    RobotClient = RBT()
    print("Robot client ready!")
    print("")
    
    # Home: try -89 first (near vertical, may clear table); fallbacks -80, -70, -60
    HOME_JOINT2_VALUES = [-89.9, -80.0, -70.0, -60.0]  # degrees, within limits [-90, 180]

    action = Action()
    action.action = "MoveJ"
    action.speed = speed
    action.movej.joint1 = 0.0
    action.movej.joint3 = 0.0
    action.movej.joint4 = 0.0
    action.movej.joint5 = 0.0
    action.movej.joint6 = 0.0

    result = None
    for home_idx, joint2_deg in enumerate(HOME_JOINT2_VALUES):
        action.movej.joint2 = float(joint2_deg)
        label = "home (joint2=%.0f)" % joint2_deg if home_idx == 0 else "fallback home (joint2=%.0f)" % joint2_deg
        print("Moving to %s..." % label)
        print("  Joint values: [0.0, %.1f, 0.0, 0.0, 0.0, 0.0] degrees" % joint2_deg)
        print("")

        max_attempts = 3 if home_idx == 0 else 2
        retry_delay_s = 1.5
        for attempt in range(1, max_attempts + 1):
            if attempt > 1:
                print("Retrying %s (%d/%d)..." % (label, attempt, max_attempts))
                time.sleep(retry_delay_s)
            result = RobotClient.Move_EXECUTE(action)
            if result["Success"]:
                break
            print("Attempt %d failed: %s" % (attempt, result["Message"]))
        if result["Success"]:
            if home_idx > 0:
                print("(Used fallback joint2=%.0f.)" % joint2_deg)
            break
        print("")

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
