#!/bin/bash
# Spawn table + custom Hanoi-like layout:
#   - 1 cube (size like cube_0 = 0.09m) on Peg 0
#   - 4 cubes (sizes like cube_1..4 = 0.08, 0.07, 0.06, 0.05m) stacked on Peg 1
# Run from workspace root with: source install/setup.bash && bash src/ros2_SimRealRobotControl/ros2srrc_execution/scripts/spawn_hanoi_custom_state.sh
# Or: cd ~/dev_ws && source install/setup.bash && bash src/ros2_SimRealRobotControl/ros2srrc_execution/scripts/spawn_hanoi_custom_state.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PYTHON_DIR="$PKG_DIR/python"
SPAWN="$PYTHON_DIR/SpawnObjectMoveIt.py"

# Hanoi-style defaults: table 0, 0.48, 0.25; peg_y=0.58; peg0=(-0.2,0.58), peg1=(0,0.58)
TABLE_X=0.0
TABLE_Y=0.48
TABLE_Z=0.25
PEG_Y=0.58
PEG0_X=-0.2
PEG1_X=0.0
TABLE_SURFACE_Z=0.50   # table_z + table_height/2 = 0.25 + 0.25

echo "Spawning table..."
python3 "$SPAWN" --package ros2srrc_objects --urdf box.urdf.xacro --name table1 \
  --x $TABLE_X --y $TABLE_Y --z $TABLE_Z \
  --size_x 1.0 --size_y 0.8 --size_z 0.50 --mass 50.0 --color white

sleep 0.5

# Peg 0: 1 cube same size as Hanoi cube_0 (0.09 m)
SIZE_CUBE0=0.09
Z_PEG0_CUBE=$(python3 -c "print(0.50 + 0.09/2)")
echo "Spawning cube_0 (size $SIZE_CUBE0 m) on Peg 0 at ($PEG0_X, $PEG_Y, $Z_PEG0_CUBE)..."
python3 "$SPAWN" --package ros2srrc_objects --urdf cube.urdf.xacro --name cube_0 \
  --x $PEG0_X --y $PEG_Y --z $Z_PEG0_CUBE --size $SIZE_CUBE0 --color red

sleep 0.3

# Peg 1: 4 cubes like cube_1..4 (0.08, 0.07, 0.06, 0.05) bottom to top
SIZES=(0.08 0.07 0.06 0.05)
NAMES=(cube_1 cube_2 cube_3 cube_4)
COLORS=(orange yellow green blue)
Z=0.50
for i in 0 1 2 3; do
  s=${SIZES[$i]}
  Z=$(python3 -c "print($Z + $s/2)")
  echo "Spawning ${NAMES[$i]} (size $s m) on Peg 1 at ($PEG1_X, $PEG_Y, $Z)..."
  python3 "$SPAWN" --package ros2srrc_objects --urdf cube.urdf.xacro --name ${NAMES[$i]} \
    --x $PEG1_X --y $PEG_Y --z $Z --size $s --color ${COLORS[$i]}
  Z=$(python3 -c "print($Z + $s/2)")
  sleep 0.3
done

echo "Done: table + 1 cube on Peg 0 (cube_0) + 4 cubes on Peg 1 (cube_1..cube_4)."
