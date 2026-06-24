#!/usr/bin/python3

"""
hanoi_tower_demo.py - Tower of Hanoi Demo with Robot Manipulation

This script implements the classic Tower of Hanoi puzzle using robot pick-and-place operations.
It spawns N cubes of different sizes on the first peg and solves the puzzle by moving
all cubes to the third peg following the rules:
- Only one cube can be moved at a time
- Only the top cube can be moved
- A larger cube cannot be placed on a smaller cube

Usage:
    python3 hanoi_tower_demo.py --num_cubes 3
    python3 hanoi_tower_demo.py --num_cubes 5 --peg_spacing 0.20

Features:
- Configurable number of cubes (1-8 recommended)
- Pegs on robot stand surface, spaced along X near the far (+Y) edge
- Recursive Tower of Hanoi solver
- Visual progress tracking
- Move counting and statistics
"""

import sys
import os
import time
import subprocess
import argparse
from ament_index_python.packages import get_package_share_directory

# Add path for imports
sys.path.append(os.path.join(get_package_share_directory("ros2srrc_execution"), 'python'))

# Robot stand geometry (matches robot_stand link in ros2srrc_ur5 URDF xacro files).
# The stand is spawned with the robot at launch; Hanoi cubes are placed on its top surface.
# Table box: short X × long Y (robot mounted at y ≈ −0.612 on the −Y edge).
ROBOT_STAND_LENGTH_X = 0.84
ROBOT_STAND_WIDTH_Y = 1.844
ROBOT_STAND_HEIGHT_Z = 0.84
ROBOT_STAND_CENTER_Z = ROBOT_STAND_HEIGHT_Z / 2.0  # 0.42 m; stand sits on ground (z=0)
ROBOT_STAND_SURFACE_Z = ROBOT_STAND_CENTER_Z + ROBOT_STAND_HEIGHT_Z / 2.0  # 0.84 m tabletop
ROBOT_STAND_FAR_Y = ROBOT_STAND_WIDTH_Y / 2.0  # +Y edge (toward gripper at joint1=90°)
DEFAULT_PEG_Y_INSET = 0.31  # meters inward from +Y edge
DEFAULT_PEG_X_INSET = DEFAULT_PEG_Y_INSET  # deprecated alias for CLI compat

# Real UR5 + OnRobot 2FG7 (cisrob-09142): validated pick/place XY on robot stand.
# Physical 4-slot table: outer marks are peg0 (left) and peg3 (right); Hanoi uses peg indices 0 and 2.
REAL_PEG0_XY = (-0.15, -0.12)
REAL_PEG3_XY = (0.15, -0.12)   # Hanoi peg index 2 (rightmost)
REAL_PEG_CENTER_X = 0.0
REAL_PEG_SPACING = 0.15        # peg0/peg2 X spacing; 30 cm between outer pegs
REAL_PEG_Y = -0.12
REAL_PEG_Z = ROBOT_STAND_SURFACE_Z  # 0.84 m — modeled stand / table top (world Z)
REAL_BOARD_HEIGHT_M = 0.02  # physical board on stand (2 cm); cubes sit on stand + board

# Real UR5+2FG7 pick calibration (ur5_pick_and_place_onrobot.yaml):
# pick object_pose.z = stand top (0.84) + stacked cube heights below target (not board, not center).
# Fingertips ≈ EE_robotiq_2f85 + offset along world +Z (top-down grasp).
REAL_GRASP_Z_OFFSET_LOW = 0.01   # clearance above EE target before physical MoveL descend
REAL_EE_TO_FINGER_Z = 0.055
REAL_MIN_MOVEIT_GRASP_Z = 0.87   # RobMove descend fails below this (cisrob-09142 peg0)
REAL_STACKED_PLACE_Z_LIFT = 0.02   # raise MoveIt place Z when stacking (less physical descend)
REAL_STACKED_PICK_Z_LIFT = 0.02    # raise MoveIt grasp Z when picking from a stack (solo picks unchanged)
REAL_POSE_PUBLISH_DURATION_S = 0.5  # hanoi_publish_pose keepalive (was 3s; pick.py also gets explicit x/y/z)

from real_ur5_2fg7_motion import compute_real_ee_motion_params


def _stand_edge_insets(x, y):
    """Meters from robot_stand box edges (matches URDF 0.84 × 1.844 m stand, centered at origin)."""
    half_x = ROBOT_STAND_LENGTH_X / 2.0
    half_y = ROBOT_STAND_WIDTH_Y / 2.0
    return {
        "from_minus_x_edge_m": x - (-half_x),
        "from_plus_x_edge_m": half_x - x,
        "from_minus_y_edge_m": y - (-half_y),  # robot mounts near this edge
        "from_plus_y_edge_m": half_y - y,
    }


def _format_peg_marking_guide(peg_positions):
    """Print peg XY and inset from stand edges for marking a physical table."""
    half_x = ROBOT_STAND_LENGTH_X / 2.0
    half_y = ROBOT_STAND_WIDTH_Y / 2.0
    print("  Table model: {:.0f} cm (X) × {:.0f} cm (Y), origin at stand center, robot near −Y edge".format(
        ROBOT_STAND_LENGTH_X * 100, ROBOT_STAND_WIDTH_Y * 100))
    labels = ["peg0 (left)", "peg1 (center)", "peg2 / physical peg3 (right)"]
    for i, (x, y) in enumerate(peg_positions):
        ins = _stand_edge_insets(x, y)
        label = labels[i] if i < len(labels) else f"peg{i}"
        print(f"    {label}: X={x:+.3f}, Y={y:+.3f} m")
        print(f"      {ins['from_minus_x_edge_m']*100:.1f} cm from −X edge, "
              f"{ins['from_plus_x_edge_m']*100:.1f} cm from +X edge")
        print(f"      {ins['from_minus_y_edge_m']*100:.1f} cm from −Y edge (robot side), "
              f"{ins['from_plus_y_edge_m']*100:.1f} cm from +Y edge")
    print(f"    Outer peg spacing (peg0 ↔ peg3): {abs(peg_positions[2][0] - peg_positions[0][0])*100:.0f} cm")


def _publish_cube_pose(name, x, y, z):
    """Publish cube pose to /object_poses/<name> via subprocess (for pick on real robot)."""
    from ament_index_python.packages import get_package_prefix
    pkg_prefix = get_package_prefix("ros2srrc_execution")
    script = os.path.join(pkg_prefix, "lib", "ros2srrc_execution", "hanoi_publish_pose.py")
    cmd = [
        "python3", script,
        f"name:={name}", f"x:={x}", f"y:={y}", f"z:={z}",
        f"duration:={REAL_POSE_PUBLISH_DURATION_S}",
    ]
    try:
        subprocess.run(
            cmd, check=True,
            timeout=max(3.0, REAL_POSE_PUBLISH_DURATION_S + 2.0),
            capture_output=True, env=os.environ.copy(),
        )
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        print(f"[Hanoi]: WARNING - Failed to publish pose for {name}: {e}")


class TowerOfHanoi:
    """Tower of Hanoi puzzle solver with robot manipulation"""
    
    def __init__(self, num_cubes, peg_positions, cube_size_base=0.05, cube_height_base=0.05, 
                 table_z=ROBOT_STAND_CENTER_Z, table_height=ROBOT_STAND_HEIGHT_Z, robot=None, ee_link=None, ee_type=None,
                 moveit_only=False, virtual_moveit_z=False, board_height=0.0):
        """
        Initialize Tower of Hanoi puzzle
        
        Args:
            num_cubes: Number of cubes to use
            peg_positions: List of 3 (x, y) tuples for peg positions
            cube_size_base: Base size for smallest cube (meters)
            cube_height_base: Height of each cube (meters)
            table_z: Z position of robot stand center (meters)
            table_height: Height of robot stand (meters)
            robot: Optional robot name for pick/place (e.g. ur5)
            ee_link: Optional EE link name for ATTACHLINK (e.g. EE_robotiq_2f85 or EE_robotiq_hande)
            ee_type: Optional end-effector type (e.g. ParallelGripper for sim, RobotiqHandE/UR for real)
            board_height: Board thickness on stand for real robot (meters); added to REAL_PEG_Z for cube Z
        """
        self.num_cubes = num_cubes
        self.peg_positions = peg_positions  # [(x1, y1), (x2, y2), (x3, y3)]
        self.cube_size_base = cube_size_base
        self.cube_height_base = cube_height_base
        self.table_surface_z = table_z + (table_height / 2)  # Top of robot stand
        self.robot = robot
        self.ee_link = ee_link
        self.ee_type = ee_type
        self.moveit_only = moveit_only
        self.virtual_moveit_z = virtual_moveit_z
        self.board_height = board_height
        
        # Track state: each peg is a list of cube indices (0 = smallest, num_cubes-1 = largest)
        self.pegs = [[], [], []]  # [peg0, peg1, peg2]
        self.cube_names = []  # Names of spawned cubes
        self.cube_sizes = []  # Sizes of each cube
        self.cube_colors = []  # Colors for each cube
        
        # Statistics
        self.move_count = 0
        self.total_moves = 0  # Expected: 2^num_cubes - 1
        self._pp_robot = None  # in-process pick/place (real robot)
        self._pp_ee = None
        self._gripper_is_open = False
        self.move_sequence = []  # Pre-computed sequence of moves: [(cube_index, from_peg, to_peg), ...]
        self.successful_moves = 0
        self.failed_moves = 0
        
        # Generate cube properties
        self._generate_cube_properties()
        
    def _generate_cube_properties(self):
        """Generate names, sizes, and colors for cubes"""
        colors = ['red', 'orange', 'yellow', 'green', 'blue', 'purple', 'pink', 'cyan']
        
        for i in range(self.num_cubes):
            # Cube 0 is smallest, cube (num_cubes-1) is largest
            cube_index = self.num_cubes - 1 - i  # Reverse order for visual appeal
            size = self.cube_size_base * (1.0 + cube_index * 0.2)  # Increasing size
            name = f"cube_{i}"
            color = colors[i % len(colors)]
            
            self.cube_names.append(name)
            self.cube_sizes.append(size)
            self.cube_colors.append(color)
    
    def _support_surface_z(self):
        """World Z of the surface cubes rest on (bottom face of bottom cube)."""
        if self.virtual_moveit_z:
            return REAL_PEG_Z + self.board_height
        return self.table_surface_z

    def get_cube_z(self, peg_index, stack_position, cube_index=None):
        """
        Calculate Z position for a cube on a peg (geometric center).
        
        Args:
            peg_index: Which peg (0, 1, or 2)
            stack_position: Position in stack (0 = bottom, higher = top)
            cube_index: Optional cube index to get actual cube height (if None, uses cube_height_base)
        
        Returns:
            Z coordinate for cube center
        """
        z = self._support_surface_z()

        if stack_position > 0:
            for pos in range(stack_position):
                if pos < len(self.pegs[peg_index]):
                    cube_idx = self.pegs[peg_index][pos]
                    z += self.cube_sizes[cube_idx]
                else:
                    z += self.cube_height_base

        if cube_index is not None:
            cube_height = self.cube_sizes[cube_index]
        else:
            cube_height = self.cube_height_base
        z += cube_height / 2.0

        return z

    def get_pick_reference_z(self, peg_index, stack_position):
        """
        pick.py object_pose.z for real UR5+2FG7.

        Stand collision model top (REAL_PEG_Z) plus full heights of cubes below the target —
        same convention as ur5_pick_and_place_onrobot.yaml (z=0.84 for bottom cube on stand).
        Board height is NOT added here; it is included in get_cube_z() for spawn/place.
        """
        z = REAL_PEG_Z
        for pos in range(stack_position):
            if pos < len(self.pegs[peg_index]):
                cube_idx = self.pegs[peg_index][pos]
                z += self.cube_sizes[cube_idx]
            else:
                z += self.cube_height_base
        return z

    def compute_real_ee_motion_params(self, z_pick_ref, z_center):
        """Delegate to shared UR5+2FG7 calibration (pick.py / place.py use the same function)."""
        return compute_real_ee_motion_params(z_pick_ref, z_center)
    
    def spawn_initial_state(self):
        """Spawn all cubes on the first peg (peg 0)"""
        print("\n" + "="*60)
        print("SPAWNING INITIAL STATE: All cubes on Peg 0")
        print("="*60)
        
        for i in range(self.num_cubes):
            cube_name = self.cube_names[i]
            cube_size = self.cube_sizes[i]
            cube_color = self.cube_colors[i]
            
            # Position on first peg, stacked from bottom to top
            x, y = self.peg_positions[0]
            z = self.get_cube_z(0, i, cube_index=i)
            
            print(f"\nSpawning {cube_color} cube {i+1}/{self.num_cubes} (size: {cube_size:.3f}m)")
            print(f"  Position: ({x:.3f}, {y:.3f}, {z:.3f})")
            
            if not self._spawn_cube(cube_name, x, y, z, cube_size, cube_color):
                print(f"\n✗ FAILED: Could not spawn cube {cube_name}")
                return False
            
            # Add to peg state
            self.pegs[0].append(i)
            #time.sleep(0.3)  # Brief settle after spawn
        
        print(f"\n✓ Initial state: {self.num_cubes} cubes on Peg 0")
        self._print_state()
        return True
    
    def _spawn_cube(self, name, x, y, z, size, color):
        """Spawn a single cube"""
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
        if self.moveit_only:
            cmd.append("--moveit-only")
        return self._run_command(cmd, f"Spawning {name}", timeout=15)
    
    def _sync_moveit_scene(self):
        """Re-add all cubes to MoveIt at current peg/stack positions (after homing clears scene)."""
        print("\nSyncing MoveIt scene with current cube positions...")
        for peg_idx in range(3):
            for stack_pos, cube_index in enumerate(self.pegs[peg_idx]):
                cube_name = self.cube_names[cube_index]
                cube_size = self.cube_sizes[cube_index]
                cube_color = self.cube_colors[cube_index]
                x, y = self.peg_positions[peg_idx]
                z = self.get_cube_z(peg_idx, stack_pos, cube_index=cube_index)
                if not self._spawn_cube(cube_name, x, y, z, cube_size, cube_color):
                    print(f"WARNING: Could not re-add {cube_name} to MoveIt scene")
                    return False
                #time.sleep(0.15)
        print("MoveIt scene synced.")
        return True

    def _move_to_home(self):
        """Move robot to home position (temporarily clears virtual cubes so EE can plan home)."""
        print("\n" + "="*60)
        print("MOVING TO HOME POSITION")
        print("="*60)
        clear_objects = ",".join(self.cube_names)
        cmd = [
            "ros2", "run", "ros2srrc_execution", "move_to_home.py",
            f"clear_objects:={clear_objects}",
        ]
        result = self._run_command(cmd, "Moving to home position", timeout=30)
        if not self._sync_moveit_scene():
            print("WARNING: MoveIt cube sync failed after homing.")
        return result
    
    def _pick_cube(self, cube_index, from_peg=None, from_stack_pos=None):
        """
        Pick a cube by index with fallback kinematics
        
        Uses pick.py which has built-in fallback mechanisms:
        - Approach: PTP (fixed position/joint space) first
        - Descend: PTP (fixed position) first, with LIN fallback if PTP fails
        - Multiple orientation candidates if first attempt fails
        - Calculates gripper close percentage automatically based on cube size
        - Calculates minimum lift height to clear cubes stacked above
        
        Args:
            cube_index: Index of cube to pick
            from_peg: Optional peg index (if None, will search for cube in all pegs)
            from_stack_pos: Optional stack position (if None, will calculate from current state)
        """
        cube_name = self.cube_names[cube_index]
        cube_size = self.cube_sizes[cube_index]  # Get cube size for gripper calculation
        
        # Find which peg the cube is on if not provided
        if from_peg is None:
            for peg_idx in range(3):
                if cube_index in self.pegs[peg_idx]:
                    from_peg = peg_idx
                    break
        
        # Calculate stack position if not provided
        if from_peg is not None and from_stack_pos is None:
            if cube_index in self.pegs[from_peg]:
                from_stack_pos = self.pegs[from_peg].index(cube_index)
            else:
                from_stack_pos = 0  # Fallback
        
        # Calculate total height of cubes above this one (if any)
        min_lift_height = None
        if from_peg is not None and from_stack_pos is not None:
            # Always have picked cube center Z and safety margin for lift height logic
            picked_cube_z_center = self.get_cube_z(from_peg, from_stack_pos, cube_index=cube_index)
            safety_margin = 0.05  # 20cm safety margin for clearance
            
            # Count how many cubes are above this one
            cubes_above = len(self.pegs[from_peg]) - (from_stack_pos + 1)
            if cubes_above > 0:
                # Get the top cube above (the highest one)
                top_cube_pos = len(self.pegs[from_peg]) - 1
                top_cube_idx = self.pegs[from_peg][top_cube_pos]
                top_cube_z_center = self.get_cube_z(from_peg, top_cube_pos, cube_index=top_cube_idx)
                top_cube_height = self.cube_sizes[top_cube_idx]
                
                # Calculate the top surface of the highest cube above
                top_cube_top_surface = top_cube_z_center + (top_cube_height / 2.0)
                
                # CRITICAL: object_pose.z is the CENTER of the cube being picked
                # We need to lift to at least: top of highest cube above + safety margin
                # Since min_lift_height is relative to object_pose.z (the center), we calculate:
                #   min_lift_height = (top of highest cube) - (center of picked cube) + (safety margin)
                
                # Calculate minimum lift height relative to picked cube's center
                min_lift_height = (top_cube_top_surface - picked_cube_z_center) + safety_margin
                
                # Also ensure we account for the picked cube's own half-height in case of calculation differences
                picked_cube_half_height = cube_size / 2.0
                min_lift_height = max(min_lift_height, picked_cube_half_height + safety_margin)
            
            # Check other pegs: if current lift height would be below their stack tops, lift higher
            other_peg_top_surfaces = []
            for p in range(3):
                if p == from_peg:
                    continue
                if not self.pegs[p]:
                    continue
                top_pos = len(self.pegs[p]) - 1
                top_cube_idx = self.pegs[p][top_pos]
                top_center_z = self.get_cube_z(p, top_pos, cube_index=top_cube_idx)
                top_surface_z = top_center_z + (self.cube_sizes[top_cube_idx] / 2.0)
                other_peg_top_surfaces.append(top_surface_z)
            
            if other_peg_top_surfaces:
                max_other_stack_z = max(other_peg_top_surfaces)
                required_lift_z = max_other_stack_z + safety_margin
                current_lift_height = min_lift_height if min_lift_height is not None else 0.15
                current_lift_z = picked_cube_z_center + current_lift_height
                if current_lift_z < required_lift_z:
                    required_min_lift = required_lift_z - picked_cube_z_center
                    min_lift_height = max(min_lift_height or 0, required_min_lift)
                    # print(f"----------------------------------------------------------------")  # DEBUG
                    # print(f"[Hanoi]: Other pegs require higher lift: required_lift_z={required_lift_z:.3f}m -> min_lift_height={min_lift_height:.3f}m")  # DEBUG
                    # print(f"----------------------------------------------------------------")  # DEBUG
            
            # Cap absolute lift Z so the robot can reach it (avoid NO_IK_SOLUTION).
            # We keep safety_margin at 0.25m so clearance over other pegs (e.g. peg1) is enough;
            # only when the ideal lift would exceed this cap do we lower the lift.
            # Lower MAX_LIFT_ABSOLUTE_Z if IK still fails; raise slightly if you see collision over peg1.
            MAX_LIFT_ABSOLUTE_Z = 1.0  # m
            if min_lift_height is not None:
                absolute_lift_z = picked_cube_z_center + min_lift_height
                if absolute_lift_z > MAX_LIFT_ABSOLUTE_Z:
                    min_lift_height = MAX_LIFT_ABSOLUTE_Z - picked_cube_z_center
        
        #time.sleep(0.02)  # Brief physics settle

        x, y = self.peg_positions[from_peg]
        z_center = self.get_cube_z(from_peg, from_stack_pos, cube_index=cube_index)
        z_pick = self.get_pick_reference_z(from_peg, from_stack_pos)
        grasp_z_offset, extra_descend, z_ee_target = self.compute_real_ee_motion_params(z_pick, z_center)

        # Stacked pick only: raise MoveIt grasp Z (grasp_z_offset). Bottom/solo picks
        # (from_stack_pos == 0) keep compute_real_ee_motion_params unchanged.
        if (
            from_stack_pos is not None
            and from_stack_pos > 0
            and from_peg is not None
            and len(self.pegs[from_peg]) > from_stack_pos
        ):
            grasp_z_offset = round(grasp_z_offset + REAL_STACKED_PICK_Z_LIFT, 4)
            print(
                f"[Hanoi]: Stacked pick on Peg {from_peg} "
                f"({from_stack_pos} cube(s) below target) "
                f"— MoveIt grasp Z +{REAL_STACKED_PICK_Z_LIFT * 1000:.0f} mm "
                f"(solo picks at stack pos 0 unchanged)"
            )

        # Cubes below the target on the same peg block MoveIt descend — clear them in pick.py
        support_names = []
        support_size_dict = {}
        support_cube_args = []
        if from_peg is not None and from_stack_pos is not None and from_stack_pos > 0:
            support_size_pairs = []
            for pos in range(from_stack_pos):
                below_idx = self.pegs[from_peg][pos]
                below_name = self.cube_names[below_idx]
                support_names.append(below_name)
                support_size_dict[below_name] = self.cube_sizes[below_idx]
                support_size_pairs.append(f"{below_name}:{self.cube_sizes[below_idx]}")
            support_cube_args = [
                f"support_cubes:={','.join(support_names)}",
                f"support_cube_sizes:={','.join(support_size_pairs)}",
            ]

        uses_explicit_pose = self.virtual_moveit_z or self.ee_type == "onrobot_2fg7"
        # Topic publisher only needed when pick.py must subscribe (not 2FG7 / explicit x,y,z)
        if (
            self.ee_type is not None
            and from_peg is not None
            and from_stack_pos is not None
            and not uses_explicit_pose
        ):
            _publish_cube_pose(cube_name, x, y, z_pick)

        if uses_explicit_pose:
            z_moveit = z_pick + grasp_z_offset
            pick_desc = (
                f"Picking {cube_name} (pick ref Z={z_pick:.3f}m, center Z={z_center:.3f}m, "
                f"MoveIt grasp Z={z_moveit:.3f}m, EE target Z={z_ee_target:.3f}m, "
                f"extra descend={extra_descend * 1000:.0f}mm)"
            )
            if self._uses_inprocess_pick_place():
                if min_lift_height is not None:
                    pick_desc = f"{pick_desc}, min_lift: {min_lift_height:.3f}m"
                return self._execute_pick_inprocess(
                    cube_name, cube_size, x, y, z_pick, grasp_z_offset, extra_descend,
                    support_names, support_size_dict, min_lift_height, pick_desc,
                    from_peg=from_peg,
                )
        else:
            pick_desc = f"Picking {cube_name} (size: {cube_size:.3f}m)"

        cmd = [
            "ros2", "run", "ros2srrc_execution", "pick.py",
            f"object:={cube_name}",
            f"cube_size:={cube_size}"  # Pass cube size for automatic gripper close % calculation
        ]
        if self.robot is not None:
            cmd.append(f"robot:={self.robot}")
        if self.ee_link is not None:
            cmd.append(f"ee_link:={self.ee_link}")
        if self.ee_type is not None:
            cmd.append(f"ee_type:={self.ee_type}")
        if uses_explicit_pose:
            cmd.append(f"x:={x}")
            cmd.append(f"y:={y}")
            cmd.append(f"z:={z_pick}")
            cmd.append(f"grasp_z_offset:={grasp_z_offset}")
            cmd.append(f"grasp_extra_descend_m:={extra_descend}")
            cmd.extend(support_cube_args)
        
        # Add min_lift_height if calculated
        if min_lift_height is not None:
            cmd.append(f"min_lift_height:={min_lift_height}")
            result = self._run_command(
                cmd, f"{pick_desc}, min_lift: {min_lift_height:.3f}m", timeout=60)
        else:
            result = self._run_command(cmd, pick_desc, timeout=60)
        
        return result
    
    def _place_cube(self, cube_index, peg_index, stack_position):
        """
        Place a cube on a peg at a specific stack position
        
        Uses place.py which has built-in fallback mechanisms:
        - Approach: PTP (fixed position/joint space) first
        - Descend: PTP first, with LIN fallback if PTP fails
        - Retract: LIN first, with PTP fallback if LIN fails
        """
        cube_name = self.cube_names[cube_index]
        cube_size = self.cube_sizes[cube_index]
        x, y = self.peg_positions[peg_index]
        z_center = self.get_cube_z(peg_index, stack_position, cube_index=cube_index)

        if self.virtual_moveit_z or self.ee_type == "onrobot_2fg7":
            z_ref = self.get_pick_reference_z(peg_index, stack_position)
            moveit_z_offset, extra_descend, z_ee_target = self.compute_real_ee_motion_params(
                z_ref, z_center)
            # Stacked place: raise MoveIt stop height (place_z_offset). Trimming only
            # place_extra_descend_m had no effect when extra was already ~10 mm (clamps to 0).
            cubes_on_peg = len(self.pegs[peg_index])
            if stack_position > 0 and cubes_on_peg > 0:
                moveit_z_offset = round(moveit_z_offset + REAL_STACKED_PLACE_Z_LIFT, 4)
                print(
                    f"[Hanoi]: Stacked place on Peg {peg_index} "
                    f"({cubes_on_peg} cube(s) already on peg, stack pos {stack_position}) "
                    f"— MoveIt place Z +{REAL_STACKED_PLACE_Z_LIFT * 1000:.0f} mm "
                    f"(less descend vs empty peg)"
                )
            z_moveit = z_ref + moveit_z_offset
            support_names = []
            support_size_dict = {}
            support_cube_args = []
            if stack_position > 0:
                support_size_pairs = []
                for pos in range(stack_position):
                    below_idx = self.pegs[peg_index][pos]
                    below_name = self.cube_names[below_idx]
                    support_names.append(below_name)
                    support_size_dict[below_name] = self.cube_sizes[below_idx]
                    support_size_pairs.append(f"{below_name}:{self.cube_sizes[below_idx]}")
                support_cube_args = [
                    f"support_cubes:={','.join(support_names)}",
                    f"support_cube_sizes:={','.join(support_size_pairs)}",
                ]
            place_desc = (
                f"Placing {cube_name} on Peg {peg_index} (center Z={z_center:.3f}m, "
                f"place ref Z={z_ref:.3f}m, MoveIt Z={z_moveit:.3f}m, "
                f"EE target Z={z_ee_target:.3f}m, extra descend={extra_descend * 1000:.0f}mm)"
            )
            if self._uses_inprocess_pick_place():
                return self._execute_place_inprocess(
                    cube_name, cube_size, x, y, z_ref, z_center,
                    moveit_z_offset, extra_descend, support_names, support_size_dict, place_desc,
                )
            cmd = [
                "ros2", "run", "ros2srrc_execution", "place.py",
                f"x:={x}",
                f"y:={y}",
                f"z:={z_ref}",
                f"object:={cube_name}",
                f"cube_size:={cube_size}",
                f"place_z_offset:={moveit_z_offset}",
                f"place_extra_descend_m:={extra_descend}",
                f"place_center_z:={z_center}",
            ]
            cmd.extend(support_cube_args)
        else:
            if stack_position == 0:
                z = self.table_surface_z
            else:
                cube_below_index = self.pegs[peg_index][stack_position - 1]
                cube_below_height = self.cube_sizes[cube_below_index]
                z_below_center = self.get_cube_z(
                    peg_index, stack_position - 1, cube_index=cube_below_index)
                z = z_below_center + (cube_below_height / 2.0)
            place_desc = (
                f"Placing {cube_name} on Peg {peg_index} (target surface: {z:.3f}m, "
                f"size: {cube_size:.3f}m)"
            )
            cmd = [
                "ros2", "run", "ros2srrc_execution", "place.py",
                f"x:={x}",
                f"y:={y}",
                f"z:={z}",
                f"object:={cube_name}",
                f"cube_size:={cube_size}",
            ]

        if self.robot is not None:
            cmd.append(f"robot:={self.robot}")
        if self.ee_link is not None:
            cmd.append(f"ee_link:={self.ee_link}")
        if self.ee_type is not None:
            cmd.append(f"ee_type:={self.ee_type}")
        return self._run_command(cmd, place_desc, timeout=60)
    
    def _obstacle_cubes_for_pick(self, from_peg):
        """Cubes on other pegs — removed from MoveIt during approach so arm can cross the table."""
        names = []
        sizes = {}
        for peg_idx in range(3):
            if peg_idx == from_peg:
                continue
            for cube_idx in self.pegs[peg_idx]:
                name = self.cube_names[cube_idx]
                names.append(name)
                sizes[name] = self.cube_sizes[cube_idx]
        return names, sizes

    def _uses_inprocess_pick_place(self):
        """Real UR5+2FG7: reuse robot/gripper clients instead of ros2 run per move."""
        return self.ee_type == "onrobot_2fg7"

    def _ensure_pick_place_clients(self):
        if self._pp_robot is not None:
            return
        import rclpy
        from robot import RBT
        from endeffector.gripper_factory import create_gripper

        if not rclpy.ok():
            rclpy.init()
        print("[Hanoi]: Starting in-process pick/place (reuses robot+gripper clients between moves).")
        self._pp_robot = RBT()
        self._pp_ee = create_gripper(self.ee_type, self.robot, self.ee_link, [])
        self._gripper_is_open = False

    def _execute_pick_inprocess(
        self, cube_name, cube_size, x, y, z_pick, grasp_z_offset, extra_descend,
        support_names, support_size_dict, min_lift_height, pick_desc, from_peg=None,
    ):
        from ros2srrc_data.msg import Robpose
        from pick_manual import Pick

        self._ensure_pick_place_clients()
        object_pose = Robpose()
        object_pose.x, object_pose.y, object_pose.z = x, y, z_pick
        object_pose.qx, object_pose.qy, object_pose.qz, object_pose.qw = -0.5, 0.5, 0.5, 0.5

        obstacle_names, obstacle_sizes = (
            self._obstacle_cubes_for_pick(from_peg) if from_peg is not None else ([], {})
        )

        config = {
            "approach_height": 0.22,
            "grasp_z_offset": grasp_z_offset,
            "grasp_extra_descend_m": extra_descend,
            "fallback_enabled": True,
            "max_attempts": 12,
            "yaw_candidates_deg": [0.0, 30.0, -30.0, 60.0, -60.0, 90.0],
            "approach_height_candidates": [0.22, 0.20, 0.18],
            "grasp_z_offset_candidates": [0.02],
            "prefer_lin_descend": False,
            "gripper_open": 0.110,
            "gripper_closed": 0.0,
            "gripper_margin": 0.002,
            "cube_size": cube_size,
            "gripper_close_full": True,
            "object_name": cube_name,
            "support_cube_names": support_names,
            "support_cube_sizes": support_size_dict,
            "skip_gripper_open_before_pick": self._gripper_is_open,
            "obstacle_cube_names": obstacle_names,
            "obstacle_cube_sizes": obstacle_sizes,
        }
        if min_lift_height is not None:
            config["min_lift_height"] = min_lift_height

        print(f"\n→ {pick_desc}")
        result = Pick(self._pp_robot, self._pp_ee, config).execute(object_pose)
        if result["Success"]:
            self._gripper_is_open = False
        else:
            print(f"\n✗ {pick_desc} failed: {result.get('Message', 'unknown')}")
        return result["Success"]

    def _execute_place_inprocess(
        self, cube_name, cube_size, x, y, z_ref, z_center,
        moveit_z_offset, extra_descend, support_names, support_size_dict, place_desc,
    ):
        from ros2srrc_data.msg import Robpose
        from place_manual import Place

        self._ensure_pick_place_clients()
        place_pose = Robpose()
        place_pose.x, place_pose.y, place_pose.z = x, y, z_ref
        place_pose.qx, place_pose.qy, place_pose.qz, place_pose.qw = -0.5, 0.5, 0.5, 0.5

        config = {
            "approach_height": 0.15,
            "place_z_offset": moveit_z_offset,
            "place_extra_descend_m": extra_descend,
            "object_name": cube_name,
            "cube_size_for_scene": cube_size,
            "place_center_z": z_center,
            "support_cube_names": support_names,
            "support_cube_sizes": support_size_dict,
        }

        print(f"\n→ {place_desc}")
        result = Place(self._pp_robot, self._pp_ee, config).execute(place_pose)
        if result["Success"]:
            self._gripper_is_open = True
        else:
            print(f"\n✗ {place_desc} failed: {result.get('Message', 'unknown')}")
        return result["Success"]

    def _run_command(self, cmd, description, timeout=30):
        """Run a shell command and wait for completion"""
        try:
            result = subprocess.run(
                cmd,
                check=True,
                timeout=timeout,
                capture_output=False,
                text=True
            )
            return True
        except subprocess.TimeoutExpired:
            print(f"\n✗ {description} timed out after {timeout}s")
            return False
        except subprocess.CalledProcessError as e:
            print(f"\n✗ {description} failed with return code {e.returncode}")
            return False
    
    def _print_state(self):
        """Print current state of all pegs"""
        print("\nCurrent State:")
        for peg_idx in range(3):
            cubes_on_peg = [self.cube_names[i] for i in self.pegs[peg_idx]]
            print(f"  Peg {peg_idx}: {cubes_on_peg if cubes_on_peg else '[]'}")
    
    def move_cube(self, from_peg, to_peg, expected_cube_index=None):
        """
        Move the top cube from one peg to another.
        
        When expected_cube_index is set (e.g. from the pre-computed sequence), verifies that
        the top of from_peg is that cube; if not, state is out of sync (e.g. after a failed move)
        and we abort this move to avoid placing the wrong cube (e.g. biggest on smallest).
        
        Args:
            from_peg: Source peg index (0, 1, or 2)
            to_peg: Destination peg index (0, 1, or 2)
            expected_cube_index: If set, require that the top of from_peg is this cube (sequence consistency)
        
        Returns:
            True if successful, False otherwise
        """
        if not self.pegs[from_peg]:
            print(f"ERROR: Cannot move from empty peg {from_peg}")
            return False
        
        # Get top cube from source peg
        cube_index = self.pegs[from_peg][-1]
        cube_name = self.cube_names[cube_index]
        
        # Require sequence consistency: if solver said "move cube_X", the top of from_peg must be cube_X
        if expected_cube_index is not None and cube_index != expected_cube_index:
            expected_name = self.cube_names[expected_cube_index]
            print(f"ERROR: State out of sync. Sequence expected {expected_name} on top of Peg {from_peg}, but Peg {from_peg} has {cube_name} on top (e.g. a previous move failed). Aborting this move to avoid invalid placement.")
            return False
        
        # Check if move is valid (destination peg empty or top cube is larger)
        # Note: smaller list index = larger physical size (cube_0 largest, cube_4 smallest)
        if self.pegs[to_peg] and cube_index < self.pegs[to_peg][-1]:
            print(f"ERROR: Invalid move — would place {cube_name} (larger) on top of {self.cube_names[self.pegs[to_peg][-1]]} (smaller) on Peg {to_peg}. State is out of sync (e.g. a previous move failed). Aborting this move.")
            return False
        
        self.move_count += 1
        
        # Calculate stack positions
        from_stack_pos = len(self.pegs[from_peg]) - 1
        to_stack_pos = len(self.pegs[to_peg])
        
        # Pick cube (pass peg and stack position so it can calculate required lift height)
        if not self._pick_cube(cube_index, from_peg=from_peg, from_stack_pos=from_stack_pos):
            print(f"\n✗ FAILED: Could not pick {cube_name}")
            # Verify actual state - object might still be on the peg
            # Don't update state if pick failed
            return False
        
        # Update state (remove from source peg) - only after successful pick
        self.pegs[from_peg].pop()
        
        # Place cube
        if not self._place_cube(cube_index, to_peg, to_stack_pos):
            print(f"\n✗ FAILED: Could not place {cube_name}")
            # Restore state
            self.pegs[from_peg].append(cube_index)
            return False
        
        # Update state (add to destination peg)
        self.pegs[to_peg].append(cube_index)
        #time.sleep(0.15)  # Brief settle before next move
        
        self._print_state()
        return True
    
    def _compute_move_sequence(self, n, source, destination, auxiliary, pegs_state):
        """
        Recursively compute the sequence of moves without executing them
        
        Args:
            n: Number of cubes to move
            source: Source peg index
            destination: Destination peg index
            auxiliary: Auxiliary peg index
            pegs_state: Current state of pegs (list of lists)
        
        Returns:
            List of moves: [(cube_index, from_peg, to_peg), ...]
        """
        if n == 0:
            return []
        
        moves = []
        
        if n == 1:
            # Base case: move single cube
            if pegs_state[source]:
                cube_index = pegs_state[source][-1]
                moves.append((cube_index, source, destination))
        else:
            # Recursive case:
            # 1. Move n-1 cubes from source to auxiliary
            temp_state = [peg[:] for peg in pegs_state]  # Deep copy for recursive call
            moves_n1 = self._compute_move_sequence(n-1, source, auxiliary, destination, temp_state)
            moves.extend(moves_n1)
            
            # Apply the moves from step 1 to update state correctly
            temp_state = [peg[:] for peg in pegs_state]  # Fresh copy
            for cube_idx, from_peg, to_peg in moves_n1:
                if temp_state[from_peg] and temp_state[from_peg][-1] == cube_idx:
                    temp_state[from_peg].pop()
                    temp_state[to_peg].append(cube_idx)
            
            # 2. Move largest cube from source to destination
            if temp_state[source]:
                cube_index = temp_state[source][-1]
                moves.append((cube_index, source, destination))
                temp_state[source].pop()
                temp_state[destination].append(cube_index)
            
            # 3. Move n-1 cubes from auxiliary to destination
            moves.extend(self._compute_move_sequence(n-1, auxiliary, destination, source, temp_state))
        
        return moves
    
    def _print_move_sequence(self):
        """Print the pre-computed move sequence"""
        print("\n" + "="*60)
        print("PRE-COMPUTED MOVE SEQUENCE")
        print("="*60)
        print(f"Total moves: {len(self.move_sequence)}\n")
        
        for i, (cube_index, from_peg, to_peg) in enumerate(self.move_sequence, 1):
            cube_name = self.cube_names[cube_index]
            print(f"{i:3d}. {cube_name:10s}: Peg {from_peg} -> Peg {to_peg}")
        print("="*60 + "\n")
    
    def solve(self, n=None, source=0, destination=2, auxiliary=1):
        """
        Solve Tower of Hanoi puzzle by executing pre-computed moves
        
        Args:
            n: Number of cubes to move (None = all remaining)
            source: Source peg index
            destination: Destination peg index
            auxiliary: Auxiliary peg index
        
        Returns:
            True if all moves executed successfully, False otherwise
        """
        if n is None:
            n = len(self.pegs[source])
        
        # First, compute the move sequence
        initial_state = [peg[:] for peg in self.pegs]  # Deep copy
        self.move_sequence = self._compute_move_sequence(n, source, destination, auxiliary, initial_state)
        
        # Print the sequence
        self._print_move_sequence()
        
        # Move to home position before starting
        print("\n" + "="*60)
        print("STEP 0: Moving robot to HOME POSITION")
        print("="*60)
        if not self._move_to_home():
            print("WARNING: Could not move to home position, continuing anyway...")
        #time.sleep(0.15)  # Brief settle before moves
        
        # Now execute the moves
        all_successful = True
        for move_num, (cube_index, from_peg, to_peg) in enumerate(self.move_sequence, 1):
            cube_name = self.cube_names[cube_index]
            
            print(f"\n{'='*60}")
            print(f"EXECUTING MOVE {move_num}/{len(self.move_sequence)}: {cube_name} from Peg {from_peg} to Peg {to_peg}")
            print(f"{'='*60}")
            
            # Execute the move (pass expected cube so we never move the wrong cube after a failed move)
            success = self.move_cube(from_peg, to_peg, expected_cube_index=cube_index)
            
            if success:
                self.successful_moves += 1
                print(f"✓ Move {move_num} completed successfully")
            else:
                self.failed_moves += 1
                all_successful = False
                print(f"✗ Move {move_num} FAILED - continuing with next move...")
                # Note: If the failure was due to "State out of sync" or "Invalid move", subsequent
                # moves will likely also fail until the puzzle is reset. Consider re-running with
                # --skip_spawn and --initial_state to resume from the current physical state.
        
        # Return to home position after finishing (whether all succeeded or some failed)
        print("\n" + "="*60)
        print("Returning robot to HOME POSITION...")
        print("="*60)
        if not self._move_to_home():
            print("WARNING: Could not return to home position.")
        #time.sleep(0.15)
        
        return all_successful


def main():
    parser = argparse.ArgumentParser(
        description='Tower of Hanoi demo with robot manipulation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Solve with 3 cubes (default, simulation)
  python3 hanoi_tower_demo.py
  
  # Solve with 5 cubes
  python3 hanoi_tower_demo.py --num_cubes 5
  
  # When sim uses HandE gripper (fix ATTACHLINK "Failed to find link EE_robotiq_2f85")
  python3 hanoi_tower_demo.py --ee_link EE_robotiq_hande
  
  # Real robot with Robotiq gripper
  python3 hanoi_tower_demo.py --ee_type RobotiqHandE/UR --ee_link EE_robotiq_hande --robot ur5
  
  # Real robot: skip cube spawn, physical setup, known poses (no perception needed)
  python3 hanoi_tower_demo.py --ee_type RobotiqHandE/UR --ee_link EE_robotiq_hande --skip_spawn \\
    --stand_z 0.42 --peg_spacing 0.20 --cube_size_base 0.05 --initial_state "0:0,1,2;1:;2:"
  # Poses are computed from peg positions + stack state and published to /object_poses/<name>
  
  # Custom peg positions (pegs run along X near the stand +Y edge)
  python3 hanoi_tower_demo.py --num_cubes 4 --peg_spacing 0.25 --peg_y 0.612 --peg_center_x 0.0
  
  # Deprecated CLI (still accepted): --peg_x maps to --peg_y, --peg_center_y to --peg_center_x
  python3 hanoi_tower_demo.py --num_cubes 4 --peg_spacing 0.25 --peg_x 0.612 --peg_center_y 0.0
  
  # Two 5cm cubes on peg0, real UR5 + 2FG7
  python3 hanoi_tower_demo.py --num_cubes 2 --peg_layout real_2fg7 \\
    --cube_sizes 0.05,0.05 --ee_type onrobot_2fg7 --ee_link EE_robotiq_2f85 --robot ur5

  # Physical cubes already on peg0 (skip MoveIt spawn; poses published before each pick)
  python3 hanoi_tower_demo.py --num_cubes 2 --peg_layout real_2fg7 \\
    --cube_sizes 0.05,0.05 --skip_spawn --initial_state "0:0,1;1:;2:" \\
    --ee_type onrobot_2fg7 --ee_link EE_robotiq_2f85 --robot ur5
  
  # Skip spawning cubes (robot stand is already present from MoveIt/Gazebo launch)
  python3 hanoi_tower_demo.py --num_cubes 3 --skip_spawn
        """
    )
    
    parser.add_argument('--num_cubes', type=int, default=3,
                        help='Number of cubes (1-8 recommended, default: 3)')
    parser.add_argument('--num_disks', type=int, default=None,
                        help='DEPRECATED: Use --num_cubes instead. Number of cubes (1-8 recommended)')
    parser.add_argument('--stand_z', type=float, default=ROBOT_STAND_CENTER_Z,
                        help='Robot stand center Z in meters (default: 0.42)')
    parser.add_argument('--table_z', type=float, default=None,
                        help='DEPRECATED: use --stand_z instead')
    parser.add_argument('--peg_layout', type=str, default=None,
                        choices=['real_2fg7'],
                        help='Preset peg positions: real_2fg7 = peg0 (-0.15,-0.12) and peg3/peg2 (+0.15,-0.12) validated on UR5+2FG7')
    parser.add_argument('--peg_spacing', type=float, default=0.20,
                        help='Spacing between pegs along X in meters (default: 0.20)')
    parser.add_argument('--peg_y', type=float, default=None,
                        help='Y position for all pegs near +Y edge (default: +Y edge minus --peg_y_inset)')
    parser.add_argument('--peg_center_x', type=float, default=0.0,
                        help='Center X position for the three pegs (default: 0.0)')
    parser.add_argument('--peg_y_inset', type=float, default=DEFAULT_PEG_Y_INSET,
                        help=f'Distance from +Y edge when --peg_y is not set (default: {DEFAULT_PEG_Y_INSET})')
    parser.add_argument('--peg_x', type=float, default=None,
                        help='DEPRECATED: use --peg_y. Former fixed peg coordinate near old +X edge.')
    parser.add_argument('--peg_center_y', type=float, default=None,
                        help='DEPRECATED: use --peg_center_x. Former center Y for peg line.')
    parser.add_argument('--peg_x_inset', type=float, default=None,
                        help=f'DEPRECATED: use --peg_y_inset (default: {DEFAULT_PEG_Y_INSET})')
    parser.add_argument('--cube_sizes', type=str, default=None,
                        help='Exact cube sizes in meters for cube_0, cube_1, ... (bottom=0, top=last). '
                             'Example for two 5cm cubes: --cube_sizes 0.05,0.05')
    parser.add_argument('--cube_size_base', type=float, default=0.05,
                        help='Base size for smallest cube in meters (default: 0.05)')
    parser.add_argument('--cube_height', type=float, default=0.05,
                        help='Height of each cube in meters (default: 0.05)')
    parser.add_argument('--board_height', type=float, default=None,
                        help='Height of physical board on stand in meters (default: 0.02 for real_2fg7 / onrobot_2fg7, else 0)')
    parser.add_argument('--disk_size_base', type=float, default=None,
                        help='DEPRECATED: Use --cube_size_base instead')
    parser.add_argument('--disk_height', type=float, default=None,
                        help='DEPRECATED: Use --cube_height instead')
    parser.add_argument('--skip_spawn', action='store_true',
                        help='Skip spawning cubes (robot stand comes from MoveIt/Gazebo launch)')
    parser.add_argument('--moveit_only', action='store_true',
                        help='Add cubes to MoveIt planning scene only (no Gazebo). '
                             'Auto-enabled when --ee_type onrobot_2fg7.')
    parser.add_argument('--gazebo_spawn', action='store_true',
                        help='Also spawn cubes in Gazebo (sim only; ignored if --moveit_only)')
    parser.add_argument('--robot', type=str, default=None,
                        help='Robot name for pick/place (e.g. ur5). If not set, scripts use their default.')
    parser.add_argument('--ee_type', type=str, default=None,
                        help='End-effector type: ParallelGripper (sim), onrobot_2fg7 (real 2FG7), onrobot_ros2 (RG2/RG6 serial). Default: ParallelGripper.')
    parser.add_argument('--ee_link', type=str, default=None,
                        help='End-effector link name (e.g. EE_robotiq_2f85 or EE_robotiq_hande). Must match the link in your robot URDF.')
    parser.add_argument('--initial_state', type=str, default=None,
                        help='Resume from current state. Format: 0:4;1:;2:0,1,2,3 (peg_index:cube_indices per peg, semicolon-separated). Use with --skip_spawn. Example: after move 30 failed with Peg0=[cube_4], Peg1=[], Peg2=[cube_0..cube_3] use --initial_state "0:4;1:;2:0,1,2,3" for 5 cubes (cube index 4=smallest, 0=largest).')
    
    args = parser.parse_args()
    
    # Handle deprecated --num_disks argument
    if args.num_disks is not None:
        print("WARNING: --num_disks is deprecated, use --num_cubes instead")
        num_cubes = args.num_disks
    else:
        num_cubes = args.num_cubes
    
    # Handle deprecated size/height arguments
    cube_size_base = args.cube_size_base if args.disk_size_base is None else args.disk_size_base
    cube_height = args.cube_height if args.disk_height is None else args.disk_height
    
    # Validate number of cubes
    if num_cubes < 1 or num_cubes > 8:
        print("ERROR: Number of cubes must be between 1 and 8")
        return 1
    
    stand_z = args.stand_z if args.table_z is None else args.table_z
    if args.table_z is not None:
        print("WARNING: --table_z is deprecated, use --stand_z instead")

    peg_y_inset = args.peg_y_inset if args.peg_x_inset is None else args.peg_x_inset
    if args.peg_x_inset is not None:
        print("WARNING: --peg_x_inset is deprecated, use --peg_y_inset instead")

    if args.peg_layout == 'real_2fg7':
        peg_center_x = REAL_PEG_CENTER_X
        args.peg_spacing = REAL_PEG_SPACING
        peg_y = REAL_PEG_Y
        print("Using real_2fg7 peg layout (peg0 left, peg3/peg2 right)")
    else:
        if args.peg_y is not None:
            peg_y = args.peg_y
        elif args.peg_x is not None:
            print("WARNING: --peg_x is deprecated, use --peg_y instead")
            peg_y = args.peg_x
        else:
            peg_y = ROBOT_STAND_FAR_Y - peg_y_inset

        peg_center_x = args.peg_center_x
        if args.peg_center_y is not None:
            print("WARNING: --peg_center_y is deprecated, use --peg_center_x instead")
            peg_center_x = args.peg_center_y

    # Three pegs in a line along X, fixed Y near the stand +Y edge (reachable from base at y ≈ −0.612)
    peg_positions = [
        (peg_center_x - args.peg_spacing, peg_y),  # Peg 0
        (peg_center_x, peg_y),                     # Peg 1 (center)
        (peg_center_x + args.peg_spacing, peg_y),  # Peg 2
    ]
    
    # Calculate expected number of moves: 2^n - 1
    expected_moves = (2 ** num_cubes) - 1
    
    print("="*60)
    print("TOWER OF HANOI DEMO - Robot Manipulation")
    print("="*60)
    print(f"\nConfiguration:")
    print(f"  Number of cubes: {num_cubes}")
    print(f"  Expected moves: {expected_moves}")
    print(f"  EE type: {args.ee_type or 'ParallelGripper (default)'}")
    print(f"  EE link: {args.ee_link or '(default)'}")
    print(f"  Robot: {args.robot or '(default)'}")
    print(f"  Playing surface: robot stand (top Z={ROBOT_STAND_SURFACE_Z:.3f}m)")
    print(f"  Stand center Z: {stand_z:.3f}m")
    print(f"  Peg line Y: {peg_y:.3f}m (near +Y edge), center X: {peg_center_x:.3f}m")
    print(f"  Peg spacing (X): {args.peg_spacing}m, cube size base: {cube_size_base}m")
    print(f"  Peg positions:")
    for i, (x, y) in enumerate(peg_positions):
        print(f"    Peg {i}: ({x:.3f}, {y:.3f})")
    if args.peg_layout == 'real_2fg7':
        print(f"  Physical table marks (outer slots):")
        print(f"    peg0 (left):  ({REAL_PEG0_XY[0]:.3f}, {REAL_PEG0_XY[1]:.3f})")
        print(f"    peg3 (right): ({REAL_PEG3_XY[0]:.3f}, {REAL_PEG3_XY[1]:.3f})  [Hanoi peg index 2]")
        print(f"  Distances from modeled stand edges (84 cm × 184.4 cm):")
        _format_peg_marking_guide(peg_positions)
    print("")
    
    # Initialize puzzle
    moveit_only = args.moveit_only or (
        args.ee_type == "onrobot_2fg7" and not args.gazebo_spawn
    )
    virtual_moveit_z = args.peg_layout == "real_2fg7" or args.ee_type == "onrobot_2fg7"
    board_height = (
        REAL_BOARD_HEIGHT_M if args.board_height is None and virtual_moveit_z
        else (0.0 if args.board_height is None else args.board_height)
    )
    if moveit_only:
        print("Cube spawn: MoveIt only (no Gazebo)")
    if virtual_moveit_z:
        support_z = REAL_PEG_Z + board_height
        example_bottom_center = support_z + cube_size_base / 2.0
        example_top_center = support_z + cube_size_base + cube_size_base / 2.0
        example_pick_ref_bottom = REAL_PEG_Z
        example_pick_ref_top = REAL_PEG_Z + cube_size_base
        example_ee_bottom = example_bottom_center - REAL_EE_TO_FINGER_Z
        example_moveit_bottom = max(example_ee_bottom + REAL_GRASP_Z_OFFSET_LOW, REAL_MIN_MOVEIT_GRASP_Z)
        example_ee_top = example_top_center - REAL_EE_TO_FINGER_Z
        example_moveit_top = max(example_ee_top + REAL_GRASP_Z_OFFSET_LOW, REAL_MIN_MOVEIT_GRASP_Z)
        print("Real-robot cube Z: spawn/place at geometric center; pick uses stand-top reference")
        print(f"  Bottom 5cm cube: center={example_bottom_center:.3f}m, "
              f"MoveIt Z={example_moveit_bottom:.3f}m, extra={(example_moveit_bottom - example_ee_bottom) * 1000:.0f}mm")
        print(f"  Top of 2-stack: center={example_top_center:.3f}m, pick ref={example_pick_ref_top:.3f}m, "
              f"MoveIt Z={example_moveit_top:.3f}m, extra={(example_moveit_top - example_ee_top) * 1000:.0f}mm")

    hanoi = TowerOfHanoi(
        num_cubes=num_cubes,
        peg_positions=peg_positions,
        cube_size_base=cube_size_base,
        cube_height_base=cube_height,
        table_z=stand_z,
        table_height=ROBOT_STAND_HEIGHT_Z,
        robot=args.robot,
        ee_link=args.ee_link,
        ee_type=args.ee_type,
        moveit_only=moveit_only,
        virtual_moveit_z=virtual_moveit_z,
        board_height=board_height,
    )
    hanoi.total_moves = expected_moves

    if args.cube_sizes:
        sizes = [float(s.strip()) for s in args.cube_sizes.split(',')]
        if len(sizes) != num_cubes:
            print(f"ERROR: --cube_sizes needs {num_cubes} values, got {len(sizes)}")
            return 1
        hanoi.cube_sizes = sizes
        print(f"Using explicit cube sizes (cube_0 bottom → cube_{num_cubes - 1} top): {sizes}")
        print("")
    
    # Spawn initial state, or set state for resume
    if not args.skip_spawn:
        if not hanoi.spawn_initial_state():
            print("\n✗ FAILED: Could not spawn initial state")
            return 1
        #time.sleep(0.5)  # Brief settle for all spawned objects
    elif args.initial_state:
        # Resume from current state: parse initial_state and set pegs
        # Format: 0:4;1:;2:0,1,2,3  (peg_index:comma-separated cube indices, semicolon between pegs)
        try:
            for peg_part in args.initial_state.split(';'):
                peg_part = peg_part.strip()
                if ':' not in peg_part:
                    continue
                peg_str, indices_str = peg_part.split(':', 1)
                peg_idx = int(peg_str.strip())
                if peg_idx < 0 or peg_idx > 2:
                    raise ValueError(f"Peg index must be 0, 1, or 2, got {peg_idx}")
                indices_str = indices_str.strip()
                if not indices_str:
                    hanoi.pegs[peg_idx] = []
                else:
                    hanoi.pegs[peg_idx] = [int(x.strip()) for x in indices_str.split(',')]
            # Validate: each cube index 0..num_cubes-1 appears exactly once
            seen = []
            for peg in hanoi.pegs:
                for idx in peg:
                    if idx in seen or idx < 0 or idx >= num_cubes:
                        raise ValueError(f"Invalid or duplicate cube index {idx} (num_cubes={num_cubes})")
                    seen.append(idx)
            if len(seen) != num_cubes:
                raise ValueError(f"Expected {num_cubes} cube indices total, got {len(seen)}")
            print("\nResuming from given initial state:")
            hanoi._print_state()
            hanoi.successful_moves = 0
            hanoi.failed_moves = 0
        except Exception as e:
            print(f"\n✗ FAILED: Invalid --initial_state: {e}")
            print('  Example for 5 cubes (Peg0=[cube_4], Peg1=[], Peg2=[cube_0..cube_3]): --initial_state "0:4;1:;2:0,1,2,3"')
            return 1
    else:
        # --skip_spawn without --initial_state: assume standard initial state (all cubes on Peg 0)
        hanoi.pegs[0] = list(range(num_cubes))  # [0, 1, ..., num_cubes-1], smallest at bottom
        hanoi.pegs[1] = []
        hanoi.pegs[2] = []
        print("\nSkip spawn: assuming standard initial state (all cubes on Peg 0).")
        hanoi._print_state()
    
    # Solve the puzzle
    print("\n" + "="*60)
    print("SOLVING TOWER OF HANOI" + (" (resuming from current state)" if args.initial_state else ""))
    print("="*60)
    n_on_source = len(hanoi.pegs[0])
    print(f"Remaining: {n_on_source} cube(s) on Peg 0 to move to Peg 2...")
    print("")
    
    start_time = time.time()
    success = hanoi.solve()
    elapsed_time = time.time() - start_time
    
    # Results
    print("\n" + "="*60)
    if success:
        print("✓ TOWER OF HANOI SOLVED SUCCESSFULLY!")
    else:
        print("⚠ TOWER OF HANOI COMPLETED WITH SOME FAILURES")
    print("="*60)
    print(f"\nStatistics:")
    print(f"  Total moves planned: {len(hanoi.move_sequence)}")
    print(f"  Expected moves: {expected_moves}")
    print(f"  Successful moves: {hanoi.successful_moves}")
    print(f"  Failed moves: {hanoi.failed_moves}")
    print(f"  Time elapsed: {elapsed_time:.2f} seconds")
    if hanoi.successful_moves > 0:
        print(f"  Average time per move: {elapsed_time/hanoi.successful_moves:.2f} seconds")
    print("")
    
    hanoi._print_state()
    print("")
    
    # Return 0 if at least some moves succeeded, 1 if all failed
    return 0 if hanoi.successful_moves > 0 else 1


if __name__ == '__main__':
    exit(main())
