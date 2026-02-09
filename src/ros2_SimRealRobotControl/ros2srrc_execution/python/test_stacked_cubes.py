#!/usr/bin/python3

"""
test_stacked_cubes.py - Automated Test: Stacked Cubes Pick and Place

This test script:
1. Spawns a table
2. Spawns two cubes (blue bottom, red top) stacked on top of each other
3. Robot picks the red cube (top) and places it on the table
4. Robot picks the blue cube (bottom) and places it on top of the red cube

Usage:
    python3 test_stacked_cubes.py
    
    Or with custom positions:
    python3 test_stacked_cubes.py --table_x 0.0 --table_y 0.48 --cubes_x 0.13 --cubes_y 0.54
"""

import sys
import os
import time
import subprocess
import argparse
from ament_index_python.packages import get_package_share_directory

# Add path for imports
sys.path.append(os.path.join(get_package_share_directory("ros2srrc_execution"), 'python'))


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
        "--size_z", "0.50",
        "--mass", "50.0",
        "--color", "white"
    ]
    return run_command(cmd, "Spawning table", timeout=15)


def spawn_cube(name, x, y, z, color, size=0.05):
    """Spawn a cube"""
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
    return run_command(cmd, f"Spawning {color} cube ({name})", timeout=15)


def pick_cube(object_name):
    """Pick a cube using pick.py"""
    cmd = [
        "ros2", "run", "ros2srrc_execution", "pick.py",
        f"object:={object_name}"
    ]
    return run_command(cmd, f"Picking {object_name}", timeout=60)


def place_cube(x, y, z=0.50, object_name=None):
    """Place a cube using place.py"""
    cmd = [
        "ros2", "run", "ros2srrc_execution", "place.py",
        f"x:={x}",
        f"y:={y}",
        f"z:={z}"
    ]
    if object_name:
        cmd.append(f"object:={object_name}")
    return run_command(cmd, f"Placing cube at ({x:.3f}, {y:.3f}, {z:.3f})", timeout=60)


def main():
    parser = argparse.ArgumentParser(
        description='Test script: Stacked cubes pick and place sequence'
    )
    parser.add_argument('--table_x', type=float, default=0.0,
                        help='Table X position (default: 0.0)')
    parser.add_argument('--table_y', type=float, default=0.48,
                        help='Table Y position (default: 0.48)')
    parser.add_argument('--table_z', type=float, default=0.25,
                        help='Table Z position (default: 0.25)')
    parser.add_argument('--cubes_x', type=float, default=0.13,
                        help='Cubes X position (default: 0.13)')
    parser.add_argument('--cubes_y', type=float, default=0.54,
                        help='Cubes Y position (default: 0.54)')
    parser.add_argument('--cube_size', type=float, default=0.05,
                        help='Cube size in meters (default: 0.05)')
    parser.add_argument('--place_x', type=float, default=0.15,
                        help='Place location X (default: 0.15)')
    parser.add_argument('--place_y', type=float, default=0.35,
                        help='Place location Y (default: 0.35)')
    parser.add_argument('--skip_spawn', action='store_true',
                        help='Skip spawning objects (use if already spawned)')
    
    args = parser.parse_args()
    
    print("="*60)
    print("ROS 2 Sim-to-Real Robot Control: Stacked Cubes Test")
    print("="*60)
    print("")
    print("Test Sequence:")
    print("  1. Spawn table")
    print("  2. Spawn blue cube (bottom)")
    print("  3. Spawn red cube (top, stacked on blue)")
    print("  4. Pick red cube and place on table")
    print("  5. Pick blue cube and place on top of red cube")
    print("")
    print(f"Table position: ({args.table_x}, {args.table_y}, {args.table_z})")
    print(f"Initial cubes position: ({args.cubes_x}, {args.cubes_y})")
    print(f"Place location: ({args.place_x}, {args.place_y})")
    print("")
    
    # Calculate cube positions (stacked)
    # Table center is at table_z, table height is 0.50m, so top surface is at table_z + 0.25
    cube_size = args.cube_size
    table_surface_z = args.table_z + 0.25  # Top of table (center + half height)
    bottom_cube_z = table_surface_z + (cube_size / 2)  # Cube center on table surface
    top_cube_z = bottom_cube_z + cube_size  # Stacked on bottom cube
    
    print(f"Table surface Z: {table_surface_z:.3f}m")
    print(f"Bottom cube (blue) Z: {bottom_cube_z:.3f}m")
    print(f"Top cube (red) Z: {top_cube_z:.3f}m")
    print("")
    
    success = True
    
    # Step 1: Spawn table
    if not args.skip_spawn:
        if not spawn_table(args.table_x, args.table_y, args.table_z):
            print("\n✗ FAILED: Could not spawn table")
            return 1
        time.sleep(1.0)  # Give time for object to settle
    
    # Step 2: Spawn blue cube (bottom)
    if not args.skip_spawn:
        if not spawn_cube("cube_blue", args.cubes_x, args.cubes_y, bottom_cube_z, "blue", cube_size):
            print("\n✗ FAILED: Could not spawn blue cube")
            return 1
        time.sleep(1.0)  # Give time for object to settle
    
    # Step 3: Spawn red cube (top, stacked)
    if not args.skip_spawn:
        if not spawn_cube("cube_red", args.cubes_x, args.cubes_y, top_cube_z, "red", cube_size):
            print("\n✗ FAILED: Could not spawn red cube")
            return 1
        time.sleep(2.0)  # Give time for objects to settle and stabilize
    
    # Step 4: Pick red cube (top) and place on table
    print("\n" + "="*60)
    print("PHASE 1: Pick top cube (red) and place on table")
    print("="*60)
    
    if not pick_cube("cube_red"):
        print("\n✗ FAILED: Could not pick red cube")
        return 1
    
    # time.sleep(1.0)  # Brief pause between operations
    
    # Place red cube on table
    table_surface_z = args.table_z + 0.25  # Top of table
    place_z = table_surface_z + (cube_size / 2)  # Cube center on table surface
    if not place_cube(args.place_x, args.place_y, place_z, "cube_red"):
        print("\n✗ FAILED: Could not place red cube")
        return 1
    
    # time.sleep(2.0)  # Give time for object to settle after placement
    
    # Step 5: Pick blue cube (bottom) and place on top of red cube
    print("\n" + "="*60)
    print("PHASE 2: Pick bottom cube (blue) and place on top of red cube")
    print("="*60)
    
    if not pick_cube("cube_blue"):
        print("\n✗ FAILED: Could not pick blue cube")
        return 1
    
    # time.sleep(1.0)  # Brief pause between operations
    
    # Place blue cube on top of red cube (stacked)
    stacked_z = place_z + cube_size  # On top of red cube (cube center)
    if not place_cube(args.place_x, args.place_y, stacked_z, "cube_blue"):
        print("\n✗ FAILED: Could not place blue cube on top of red cube")
        return 1
    
    # Success!
    print("\n" + "="*60)
    print("✓ TEST COMPLETED SUCCESSFULLY!")
    print("="*60)
    print("")
    print("Final state:")
    print(f"  - Red cube placed on table at ({args.place_x}, {args.place_y}, {place_z:.3f})")
    print(f"  - Blue cube placed on top of red cube at ({args.place_x}, {args.place_y}, {stacked_z:.3f})")
    print("")
    
    return 0


if __name__ == '__main__':
    exit(main())




# source /opt/ros/humble/setup.bash && source install/setup.bash && python3 src/ros2_SimRealRobotControl/ros2srrc_execution/python/test_stacked_cubes.py

