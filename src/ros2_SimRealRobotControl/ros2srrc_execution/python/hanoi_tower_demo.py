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
- Automatic peg positioning
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


def _publish_cube_pose(name, x, y, z):
    """Publish cube pose to /object_poses/<name> via subprocess (for pick on real robot)."""
    from ament_index_python.packages import get_package_prefix
    pkg_prefix = get_package_prefix("ros2srrc_execution")
    script = os.path.join(pkg_prefix, "lib", "ros2srrc_execution", "hanoi_publish_pose.py")
    cmd = ["python3", script, f"name:={name}", f"x:={x}", f"y:={y}", f"z:={z}"]
    try:
        subprocess.run(cmd, check=True, timeout=3, capture_output=True, env=os.environ.copy())
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        print(f"[Hanoi]: WARNING - Failed to publish pose for {name}: {e}")


class TowerOfHanoi:
    """Tower of Hanoi puzzle solver with robot manipulation"""
    
    def __init__(self, num_cubes, peg_positions, cube_size_base=0.04, cube_height_base=0.04, 
                 table_z=0.25, table_height=0.50, robot=None, ee_link=None, ee_type=None):
        """
        Initialize Tower of Hanoi puzzle
        
        Args:
            num_cubes: Number of cubes to use
            peg_positions: List of 3 (x, y) tuples for peg positions
            cube_size_base: Base size for smallest cube (meters)
            cube_height_base: Height of each cube (meters)
            table_z: Z position of table center (meters)
            table_height: Height of table (meters)
            robot: Optional robot name for pick/place (e.g. ur5)
            ee_link: Optional EE link name for ATTACHLINK (e.g. EE_robotiq_2f85 or EE_robotiq_hande)
            ee_type: Optional end-effector type (e.g. ParallelGripper for sim, RobotiqHandE/UR for real)
        """
        self.num_cubes = num_cubes
        self.peg_positions = peg_positions  # [(x1, y1), (x2, y2), (x3, y3)]
        self.cube_size_base = cube_size_base
        self.cube_height_base = cube_height_base
        self.table_surface_z = table_z + (table_height / 2)  # Top of table
        self.robot = robot
        self.ee_link = ee_link
        self.ee_type = ee_type
        
        # Track state: each peg is a list of cube indices (0 = smallest, num_cubes-1 = largest)
        self.pegs = [[], [], []]  # [peg0, peg1, peg2]
        self.cube_names = []  # Names of spawned cubes
        self.cube_sizes = []  # Sizes of each cube
        self.cube_colors = []  # Colors for each cube
        
        # Statistics
        self.move_count = 0
        self.total_moves = 0  # Expected: 2^num_cubes - 1
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
    
    def get_cube_z(self, peg_index, stack_position, cube_index=None):
        """
        Calculate Z position for a cube on a peg
        
        Args:
            peg_index: Which peg (0, 1, or 2)
            stack_position: Position in stack (0 = bottom, higher = top)
            cube_index: Optional cube index to get actual cube height (if None, uses cube_height_base)
        
        Returns:
            Z coordinate for cube center
        """
        # Start from table surface
        z = self.table_surface_z
        
        # Add heights of all cubes below this position
        if stack_position > 0:
            for pos in range(stack_position):
                if pos < len(self.pegs[peg_index]):
                    cube_idx = self.pegs[peg_index][pos]
                    # Use actual cube size (which equals height for cubes)
                    cube_height = self.cube_sizes[cube_idx]
                    z += cube_height
                else:
                    # Fallback if cube not in state yet (during initial spawn)
                    z += self.cube_height_base
        
        # Add half the height of the current cube (to get center)
        if cube_index is not None:
            cube_height = self.cube_sizes[cube_index]
        else:
            cube_height = self.cube_height_base
        z += cube_height / 2.0
        
        return z
    
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
            time.sleep(0.3)  # Brief settle after spawn
        
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
        return self._run_command(cmd, f"Spawning {name}", timeout=15)
    
    def _move_to_home(self):
        """Move robot to home position"""
        print("\n" + "="*60)
        print("MOVING TO HOME POSITION")
        print("="*60)
        cmd = [
            "ros2", "run", "ros2srrc_execution", "move_to_home.py"
        ]
        return self._run_command(cmd, "Moving to home position", timeout=30)
    
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
        
        time.sleep(0.05)  # Brief physics settle
        
        # Real robot with known poses: publish cube pose so pick can get it
        if self.ee_type is not None and from_peg is not None and from_stack_pos is not None:
            x, y = self.peg_positions[from_peg]
            z = self.get_cube_z(from_peg, from_stack_pos, cube_index=cube_index)
            _publish_cube_pose(cube_name, x, y, z)
            time.sleep(0.2)  # Pose propagate to pick (transient_local)
        
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
        
        # Add min_lift_height if calculated
        if min_lift_height is not None:
            cmd.append(f"min_lift_height:={min_lift_height}")
            result = self._run_command(cmd, f"Picking {cube_name} (size: {cube_size:.3f}m, min_lift: {min_lift_height:.3f}m)", timeout=60)
        else:
            result = self._run_command(cmd, f"Picking {cube_name} (size: {cube_size:.3f}m)", timeout=60)
        
        return result
    
    def _place_cube(self, cube_index, peg_index, stack_position):
        """
        Place a cube on a peg at a specific stack position
        
        Uses place.py which has built-in fallback mechanisms:
        - Approach: PTP (fixed position/joint space) first
        - Descend: PTP first, with LIN fallback if PTP fails
        - Retract: LIN first, with PTP fallback if LIN fails
        - Calculates place_z_offset dynamically based on cube size
        """
        cube_name = self.cube_names[cube_index]
        cube_size = self.cube_sizes[cube_index]  # Get cube size (which equals height for cubes)
        x, y = self.peg_positions[peg_index]
        
        # Calculate target surface Z: if stacking on another cube, use top surface of bottom cube
        # Otherwise use table surface
        # Note: place.py expects z to be the target surface, and will add place_z_offset
        if stack_position == 0:
            # Placing on table
            target_surface_z = self.table_surface_z
        else:
            # Placing on top of another cube - get the top surface of the cube below
            cube_below_index = self.pegs[peg_index][stack_position - 1]
            cube_below_height = self.cube_sizes[cube_below_index]
            # Get Z position of cube below (its center)
            z_below_center = self.get_cube_z(peg_index, stack_position - 1, cube_index=cube_below_index)
            # Top surface = center + half height
            target_surface_z = z_below_center + (cube_below_height / 2.0)
        
        # Pass target surface Z to place.py - it will add place_z_offset (cube_size/2 + 0.001)
        # to get the final cube center position
        z = target_surface_z
        
        # Calculate expected place_z_offset for logging (place.py: cube_height/2 + 0.07m; post_open_push_down 0.02m)
        gripper_clearance = 0.02  # used only for expected_cube_center_z log; place.py uses 0.07 for offset
        expected_offset = (cube_size / 2.0) + gripper_clearance
        expected_cube_center_z = z + expected_offset
        # After opening, cube will be pushed down by gripper_clearance to settle on surface
        final_cube_center_z = z + (cube_size / 2.0)
        
        cmd = [
            "ros2", "run", "ros2srrc_execution", "place.py",
            f"x:={x}",
            f"y:={y}",
            f"z:={z}",
            f"object:={cube_name}",
            f"cube_size:={cube_size}"  # Pass cube size for dynamic offset calculation
        ]
        if self.robot is not None:
            cmd.append(f"robot:={self.robot}")
        if self.ee_link is not None:
            cmd.append(f"ee_link:={self.ee_link}")
        if self.ee_type is not None:
            cmd.append(f"ee_type:={self.ee_type}")
        return self._run_command(cmd, f"Placing {cube_name} on Peg {peg_index} (target surface: {target_surface_z:.3f}m, initial cube center: {expected_cube_center_z:.3f}m, final cube center: {final_cube_center_z:.3f}m, size: {cube_size:.3f}m)", timeout=60)
    
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
        time.sleep(0.15)  # Brief settle before next move
        
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
        time.sleep(0.15)  # Brief settle before moves
        
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
        time.sleep(0.15)
        
        return all_successful


def spawn_table(x, y, z, size_x=1.0, size_y=0.8, size_z=0.50):
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
        "--size_x", str(size_x),
        "--size_y", str(size_y),
        "--size_z", str(size_z),
        "--mass", "50.0",
        "--color", "white"
    ]
    
    try:
        subprocess.run(cmd, check=True, timeout=15, capture_output=False, text=True)
        return True
    except:
        return False


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
  
  # Real robot: skip spawn, physical setup, known poses (no perception needed)
  python3 hanoi_tower_demo.py --ee_type RobotiqHandE/UR --ee_link EE_robotiq_hande --skip_spawn \\
    --table_z 0.25 --peg_spacing 0.20 --cube_size_base 0.05 --initial_state "0:0,1,2;1:;2:"
  # Poses are computed from peg positions + stack state and published to /object_poses/<name>
  
  # Custom peg positions
  python3 hanoi_tower_demo.py --num_cubes 4 --peg_spacing 0.25
  
  # Skip spawning (use existing setup)
  python3 hanoi_tower_demo.py --num_cubes 3 --skip_spawn
        """
    )
    
    parser.add_argument('--num_cubes', type=int, default=3,
                        help='Number of cubes (1-8 recommended, default: 3)')
    parser.add_argument('--num_disks', type=int, default=None,
                        help='DEPRECATED: Use --num_cubes instead. Number of cubes (1-8 recommended)')
    parser.add_argument('--table_x', type=float, default=0.0,
                        help='Table X position (default: 0.0)')
    parser.add_argument('--table_y', type=float, default=0.48,
                        help='Table Y position (default: 0.48)')
    parser.add_argument('--table_z', type=float, default=0.25,
                        help='Table Z position (default: 0.25)')
    parser.add_argument('--peg_spacing', type=float, default=0.20,
                        help='Spacing between pegs in meters (default: 0.20)')
    parser.add_argument('--peg_y', type=float, default=None,
                        help='Y position for all pegs (default: table_y + 0.10)')
    parser.add_argument('--cube_size_base', type=float, default=0.05,
                        help='Base size for smallest cube in meters (default: 0.05)')
    parser.add_argument('--cube_height', type=float, default=0.05,
                        help='Height of each cube in meters (default: 0.05)')
    parser.add_argument('--disk_size_base', type=float, default=None,
                        help='DEPRECATED: Use --cube_size_base instead')
    parser.add_argument('--disk_height', type=float, default=None,
                        help='DEPRECATED: Use --cube_height instead')
    parser.add_argument('--skip_spawn', action='store_true',
                        help='Skip spawning table and cubes (use existing setup)')
    parser.add_argument('--robot', type=str, default=None,
                        help='Robot name for pick/place (e.g. ur5). If not set, scripts use their default.')
    parser.add_argument('--ee_type', type=str, default=None,
                        help='End-effector type: ParallelGripper (sim), RobotiqHandE/UR or robotiq_2f85 (real Robotiq), onrobot_2fg7 (real OnRobot). Default: ParallelGripper.')
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
    
    # Calculate peg positions
    if args.peg_y is None:
        peg_y = args.table_y + 0.10
    else:
        peg_y = args.peg_y
    
    # Three pegs spaced evenly
    peg_center_x = args.table_x
    peg_positions = [
        (peg_center_x - args.peg_spacing, peg_y),  # Peg 0 (left)
        (peg_center_x, peg_y),                     # Peg 1 (center)
        (peg_center_x + args.peg_spacing, peg_y),  # Peg 2 (right)
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
    print(f"  Table position: ({args.table_x}, {args.table_y}, {args.table_z})")
    print(f"  Peg spacing: {args.peg_spacing}m, cube size base: {cube_size_base}m")
    print(f"  Peg positions:")
    for i, (x, y) in enumerate(peg_positions):
        print(f"    Peg {i}: ({x:.3f}, {y:.3f})")
    print("")
    
    # Spawn table
    if not args.skip_spawn:
        print("Spawning table...")
        if not spawn_table(args.table_x, args.table_y, args.table_z):
            print("\n✗ FAILED: Could not spawn table")
            return 1
        time.sleep(0.2)
    
    # Initialize puzzle
    hanoi = TowerOfHanoi(
        num_cubes=num_cubes,
        peg_positions=peg_positions,
        cube_size_base=cube_size_base,
        cube_height_base=cube_height,
        table_z=args.table_z,
        table_height=0.50,
        robot=args.robot,
        ee_link=args.ee_link,
        ee_type=args.ee_type
    )
    hanoi.total_moves = expected_moves
    
    # Spawn initial state, or set state for resume
    if not args.skip_spawn:
        if not hanoi.spawn_initial_state():
            print("\n✗ FAILED: Could not spawn initial state")
            return 1
        time.sleep(0.5)  # Brief settle for all spawned objects
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
