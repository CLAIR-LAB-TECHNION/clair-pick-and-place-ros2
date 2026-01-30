# ros2srrc_objects

**ROS 2 SimRealRobotControl - Object URDF Library**

This package provides parametric URDF/xacro models for common objects that can be spawned in Gazebo for pick-and-place and manipulation tasks.

## Available Objects

| Object | File | Description | Default Size |
|--------|------|-------------|--------------|
| Cube | `cube.urdf.xacro` | Equal-sided box | 5cm sides |
| Box | `box.urdf.xacro` | Rectangular box (different X/Y/Z) | 5x3x2 cm |
| Cylinder | `cylinder.urdf.xacro` | Vertical cylinder | r=2.5cm, h=10cm |
| Sphere | `sphere.urdf.xacro` | Ball | r=2.5cm |

## Features

All objects include:
- **Visual geometry** with configurable colors
- **Collision geometry** matching visual
- **Correct inertial properties** (mass and inertia matrix)
- **Gazebo materials** for proper rendering
- **Gazebo physics properties** (friction, contact parameters)
- **Pose publisher plugin** (publishes to `/object_poses/<name>`)

## Usage

### Spawning Objects with SpawnObject.py

```bash
# Spawn a red cube at position (0.5, 0.0, 0.8)
ros2 run ros2srrc_execution SpawnObject.py \
    --package ros2srrc_objects \
    --urdf cube.urdf.xacro \
    --name my_cube \
    --x 0.5 --y 0.0 --z 0.8

# Spawn a blue cylinder
ros2 run ros2srrc_execution SpawnObject.py \
    --package ros2srrc_objects \
    --urdf cylinder.urdf.xacro \
    --name my_cylinder \
    --x 0.4 --y 0.1 --z 0.85

# Spawn a green sphere
ros2 run ros2srrc_execution SpawnObject.py \
    --package ros2srrc_objects \
    --urdf sphere.urdf.xacro \
    --name my_sphere \
    --x 0.3 --y -0.1 --z 0.82
```

### Object Poses

Each spawned object publishes its pose to a ROS topic:
```bash
# List object pose topics
ros2 topic list | grep object_poses

# Echo a specific object's pose
ros2 topic echo /object_poses/my_cube
```

## Parameters

### cube.urdf.xacro

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `name` | string | "cube" | Object/link name |
| `size` | float | 0.05 | Cube side length (meters) |
| `mass` | float | 0.1 | Mass (kg) |
| `color` | string | "red" | Material color |

### box.urdf.xacro

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `name` | string | "box" | Object/link name |
| `size_x` | float | 0.05 | X dimension (meters) |
| `size_y` | float | 0.03 | Y dimension (meters) |
| `size_z` | float | 0.02 | Z dimension (meters) |
| `mass` | float | 0.1 | Mass (kg) |
| `color` | string | "yellow" | Material color |

### cylinder.urdf.xacro

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `name` | string | "cylinder" | Object/link name |
| `radius` | float | 0.025 | Radius (meters) |
| `length` | float | 0.1 | Height (meters) |
| `mass` | float | 0.1 | Mass (kg) |
| `color` | string | "blue" | Material color |

### sphere.urdf.xacro

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `name` | string | "sphere" | Object/link name |
| `radius` | float | 0.025 | Radius (meters) |
| `mass` | float | 0.1 | Mass (kg) |
| `color` | string | "green" | Material color |

## Available Colors

- `red`
- `green`
- `blue`
- `yellow`
- `orange`
- `purple`
- `white`
- `black`
- `grey`

## Customizing Parameters

To spawn objects with custom parameters, you need to modify the xacro mappings in `SpawnObject.py` or process the xacro file manually:

```python
import xacro

# Process with custom parameters
xacro_file = xacro.process_file(
    'cube.urdf.xacro',
    mappings={
        "name": "big_blue_cube",
        "size": "0.1",      # 10cm cube
        "mass": "0.5",      # 500g
        "color": "blue"
    }
)
urdf_string = xacro_file.toxml()
```

## Building

```bash
cd ~/dev_ws
colcon build --packages-select ros2srrc_objects
source install/setup.bash
```

## Inertia Calculations

The inertial properties are calculated using standard formulas for solid shapes:

**Cube/Box:**
- Ixx = (1/12) * m * (y² + z²)
- Iyy = (1/12) * m * (x² + z²)
- Izz = (1/12) * m * (x² + y²)

**Cylinder (axis along Z):**
- Ixx = Iyy = (1/12) * m * (3r² + h²)
- Izz = (1/2) * m * r²

**Sphere:**
- Ixx = Iyy = Izz = (2/5) * m * r²
