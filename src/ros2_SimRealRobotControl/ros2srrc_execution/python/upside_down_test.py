#!/usr/bin/python3

"""
upside_down_test.py - Upside Down Stack Test

Spawns three cubes stacked from biggest to smallest (0.08m -> 0.06m -> 0.04m),
then picks them (smallest first) and places them to form a new stack with
smallest at bottom and biggest on top.

Initial stack (one location): [big 0.08] [medium 0.06] [small 0.04]  (bottom to top)
Pick order: small (top), then medium, then big
Final stack (place location): [small 0.04] [medium 0.06] [big 0.08]  (bottom to top)

Usage:
    ros2 run ros2srrc_execution upside_down_test.py

    Or with custom positions:
    ros2 run ros2srrc_execution upside_down_test.py --cubes_x 0.13 --cubes_y 0.54 --place_x 0.15 --place_y 0.35

    Skip spawning (use existing setup):
    ros2 run ros2srrc_execution upside_down_test.py --skip_spawn
"""

import sys
import os
import time
import subprocess
import argparse
from ament_index_python.packages import get_package_share_directory

# Add path for imports
sys.path.append(os.path.join(get_package_share_directory("ros2srrc_execution"), 'python'))


# Cube sizes in meters: big, medium, small (initial stack bottom -> top)
SIZE_BIG = 0.08
SIZE_MEDIUM = 0.06
SIZE_SMALL = 0.04


def run_command(cmd, description, timeout=30):
    """Run a shell command and wait for completion"""
    print(f"\n{'='*60}")
    print(f"STEP: {description}")
    print(f"{'='*60}")
    print(f"Command: {' '.join(cmd)}")
    print("")

    try:
        result = subprocess.run(
            cmd,
            check=True,
            timeout=timeout,
            capture_output=False,
            text=True
        )
        print(f"\n✓ {description} completed successfully")
        return True
    except subprocess.TimeoutExpired:
        print(f"\n✗ {description} timed out after {timeout}s")
        return False
    except subprocess.CalledProcessError as e:
        print(f"\n✗ {description} failed with return code {e.returncode}")
        return False


def spawn_table(x, y, z):
    """Spawn the table"""
    cmd = [
        "python3",
        os.path.join(get_package_share_directory("ros2srrc_execution"),
                    'python', 'SpawnObjectMoveIt.py'),
        "--package", "ros2srrc_objects",
        "--urdf", "box.urdf.xacro",
        "--name", "table1",
        "--x", str(x),
        "--y", str(y),
        "--z", str(z),
        "--size_x", "1.0",
        "--size_y", "0.8",
        "--size_z", "0.84",
        "--mass", "50.0",
        "--color", "white"
    ]
    return run_command(cmd, "Spawning table", timeout=15)


def spawn_cube(name, x, y, z, color, size):
    """Spawn a cube with given size."""
    cmd = [
        "python3",
        os.path.join(get_package_share_directory("ros2srrc_execution"),
                    'python', 'SpawnObjectMoveIt.py'),
        "--package", "ros2srrc_objects",
        "--urdf", "cube.urdf.xacro",
        "--name", name,
        "--x", str(x),
        "--y", str(y),
        "--z", str(z),
        "--size", str(size),
        "--color", color
    ]
    return run_command(cmd, f"Spawning {color} cube {name} (size {size:.2f}m)", timeout=15)


def move_to_home():
    """Move robot to home position."""
    cmd = ["ros2", "run", "ros2srrc_execution", "move_to_home.py"]
    return run_command(cmd, "Moving to home position", timeout=30)


def pick_cube(object_name, cube_size, min_lift_height=None):
    """Pick a cube using pick.py (cube_size for gripper, optional min_lift_height)."""
    cmd = [
        "ros2", "run", "ros2srrc_execution", "pick.py",
        f"object:={object_name}",
        f"cube_size:={cube_size}"
    ]
    if min_lift_height is not None:
        cmd.append(f"min_lift_height:={min_lift_height}")
    return run_command(cmd, f"Picking {object_name} (size {cube_size:.2f}m)", timeout=60)


def place_cube(x, y, z_surface, object_name, cube_size):
    """Place a cube using place.py. z_surface = top of table or top of cube below."""
    cmd = [
        "ros2", "run", "ros2srrc_execution", "place.py",
        f"x:={x}",
        f"y:={y}",
        f"z:={z_surface}",
        f"object:={object_name}",
        f"cube_size:={cube_size}"
    ]
    return run_command(cmd, f"Placing {object_name} at ({x:.3f}, {y:.3f}), surface z={z_surface:.3f}m", timeout=60)


def main():
    parser = argparse.ArgumentParser(
        description='Upside down test: pick 3 stacked cubes (big->med->small) and place as small->med->big'
    )
    parser.add_argument('--table_x', type=float, default=0.0, help='Table X (default: 0.0)')
    parser.add_argument('--table_y', type=float, default=0.48, help='Table Y (default: 0.48)')
    parser.add_argument('--table_z', type=float, default=0.42, help='Table center Z (default: 0.42; top at 0.84 m)')
    parser.add_argument('--cubes_x', type=float, default=0.13, help='Initial stack X (default: 0.13)')
    parser.add_argument('--cubes_y', type=float, default=0.54, help='Initial stack Y (default: 0.54)')
    parser.add_argument('--place_x', type=float, default=0.15, help='Place stack X (default: 0.15)')
    parser.add_argument('--place_y', type=float, default=0.35, help='Place stack Y (default: 0.35)')
    parser.add_argument('--skip_spawn', action='store_true', help='Skip spawning table and cubes')
    args = parser.parse_args()

    table_surface_z = args.table_z + 0.42  # Top of table (0.84 m with default center)

    # Initial stack: bottom = big, middle = medium, top = small (center Z for each)
    z_big = table_surface_z + (SIZE_BIG / 2)
    z_medium = z_big + (SIZE_BIG / 2) + (SIZE_MEDIUM / 2)
    z_small = z_medium + (SIZE_MEDIUM / 2) + (SIZE_SMALL / 2)

    # Min lift when picking from stack (clear cubes above)
    safety_margin = 0.25
    big_top_z = z_big + (SIZE_BIG / 2)
    # For small (top): lift small center above big_top + margin -> min_lift = (big_top_z + safety_margin) - z_small
    min_lift_small = (big_top_z + safety_margin) - z_small
    # For medium: lift medium center above big_top + margin
    min_lift_medium = (big_top_z + safety_margin) - z_medium
    # Big has nothing above; no min_lift or use small default

    print("="*60)
    print("UPSIDE DOWN TEST")
    print("="*60)
    print("Initial stack (one spot): big 0.08 -> medium 0.06 -> small 0.04 (bottom to top)")
    print("Pick: small, then medium, then big.")
    print("Place: small on table, then medium on small, then big on medium.")
    print("")
    print(f"Table surface Z: {table_surface_z:.3f}m")
    print(f"Initial stack Z (centers): big={z_big:.3f}, medium={z_medium:.3f}, small={z_small:.3f}")
    print(f"Place location: ({args.place_x}, {args.place_y})")
    print("")

    if not args.skip_spawn:
        if not spawn_table(args.table_x, args.table_y, args.table_z):
            print("\n✗ FAILED: Could not spawn table")
            return 1
        time.sleep(1.0)

        if not spawn_cube("cube_big", args.cubes_x, args.cubes_y, z_big, "blue", SIZE_BIG):
            print("\n✗ FAILED: Could not spawn big cube")
            return 1
        time.sleep(1.0)

        if not spawn_cube("cube_medium", args.cubes_x, args.cubes_y, z_medium, "green", SIZE_MEDIUM):
            print("\n✗ FAILED: Could not spawn medium cube")
            return 1
        time.sleep(1.0)

        if not spawn_cube("cube_small", args.cubes_x, args.cubes_y, z_small, "red", SIZE_SMALL):
            print("\n✗ FAILED: Could not spawn small cube")
            return 1
        time.sleep(2.0)

    if not move_to_home():
        print("WARNING: Could not move to home, continuing anyway...")
    time.sleep(0.3)

    # 1) Pick small (top), place on table
    print("\n" + "="*60)
    print("PHASE 1: Pick small (top), place on table")
    print("="*60)
    if not pick_cube("cube_small", SIZE_SMALL, min_lift_height=max(0.15, min_lift_small)):
        print("\n✗ FAILED: Could not pick small cube")
        return 1
    if not place_cube(args.place_x, args.place_y, table_surface_z, "cube_small", SIZE_SMALL):
        print("\n✗ FAILED: Could not place small cube")
        return 1
    time.sleep(0.3)

    # 2) Pick medium, place on top of small
    print("\n" + "="*60)
    print("PHASE 2: Pick medium, place on small")
    print("="*60)
    if not pick_cube("cube_medium", SIZE_MEDIUM, min_lift_height=max(0.15, min_lift_medium)):
        print("\n✗ FAILED: Could not pick medium cube")
        return 1
    surface_after_small = table_surface_z + SIZE_SMALL
    if not place_cube(args.place_x, args.place_y, surface_after_small, "cube_medium", SIZE_MEDIUM):
        print("\n✗ FAILED: Could not place medium cube")
        return 1
    time.sleep(0.3)

    # 3) Pick big, place on top of medium
    print("\n" + "="*60)
    print("PHASE 3: Pick big, place on medium")
    print("="*60)
    if not pick_cube("cube_big", SIZE_BIG, min_lift_height=None):
        print("\n✗ FAILED: Could not pick big cube")
        return 1
    surface_after_medium = table_surface_z + SIZE_SMALL + SIZE_MEDIUM
    if not place_cube(args.place_x, args.place_y, surface_after_medium, "cube_big", SIZE_BIG):
        print("\n✗ FAILED: Could not place big cube")
        return 1
    time.sleep(0.3)

    # Return to home
    print("\n" + "="*60)
    print("Returning to HOME")
    print("="*60)
    if not move_to_home():
        print("WARNING: Could not return to home.")
    time.sleep(0.3)

    print("\n" + "="*60)
    print("✓ UPSIDE DOWN TEST COMPLETED SUCCESSFULLY!")
    print("="*60)
    print("Final stack at ({}, {}): small (0.04) -> medium (0.06) -> big (0.08) from bottom to top.")
    print("")
    return 0


if __name__ == '__main__':
    exit(main())
