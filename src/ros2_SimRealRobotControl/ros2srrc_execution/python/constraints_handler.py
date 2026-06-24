#!/usr/bin/env python3
"""
Set Constraints (HLD): optional program step to push constraints into the
planning pipeline. Uses MoveIt planning scene (collision objects).
If the step is never called, behavior is unchanged (zero effect).
"""

from __future__ import annotations

import time
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose as GeometryPose


def apply_constraints(step_dict: dict) -> dict:
    """
    Apply constraints from a program step (Type: "SetConstraints").
    Supports at least one constraint: collision_object (box in world frame).
    Stored and passed to the existing planning pipeline via /planning_scene.
    If step has no Input or unsupported type, no-op and return success.

    Args:
        step_dict: Step from YAML, e.g. {"Type": "SetConstraints", "Input": {...}}.

    Returns:
        {"Success": bool, "Message": str} - same shape as other steps.
    """
    inp = step_dict.get("Input") or step_dict.get("input")
    if not inp:
        return {"Success": True, "Message": "SetConstraints: no input (no-op)."}

    constraint_type = (inp.get("type") or inp.get("Type") or "collision_object").strip().lower()
    if constraint_type != "collision_object":
        return {"Success": True, "Message": f"SetConstraints: type '{constraint_type}' not applied (no-op)."}

    # Optional collision object: id, position (x,y,z), size or size_x/size_y/size_z
    obj_id = inp.get("id") or inp.get("name") or "constraint_collision_box"
    x = float(inp.get("x", 0.0))
    y = float(inp.get("y", 0.0))
    z = float(inp.get("z", 0.0))
    size = float(inp.get("size", 0.05))
    if "size_x" in inp or "size_y" in inp or "size_z" in inp:
        lx = float(inp.get("size_x", size))
        ly = float(inp.get("size_y", size))
        lz = float(inp.get("size_z", size))
        dimensions = [lx, ly, lz]
    else:
        dimensions = [size, size, size]

    try:
        # Use existing rclpy context (e.g. from ExecuteProgram); do not shutdown
        node = Node("set_constraints_one_shot")
        pub = node.create_publisher(PlanningScene, "/planning_scene", 10)
        #time.sleep(0.2)
        rclpy.spin_once(node, timeout_sec=0.1)

        co = CollisionObject()
        co.header.frame_id = "world"
        co.header.stamp = node.get_clock().now().to_msg()
        co.id = obj_id
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = dimensions
        pose = GeometryPose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.world.collision_objects.append(co)
        scene.is_diff = True
        pub.publish(scene)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
        #time.sleep(0.15)
        node.destroy_node()
        return {"Success": True, "Message": f"SetConstraints: added collision_object '{obj_id}'."}
    except Exception as e:
        return {"Success": False, "Message": f"SetConstraints: {e!s}"}
