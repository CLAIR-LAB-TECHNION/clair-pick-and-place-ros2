#!/usr/bin/python3

"""
SpawnObjectMoveIt.py - Spawn objects in BOTH Gazebo AND MoveIt planning scene

This script:
1. Spawns the object in Gazebo (for physics simulation)
2. Adds the object to MoveIt's planning scene (for collision avoidance)

Usage:
    python3 SpawnObjectMoveIt.py --package ros2srrc_objects --urdf cube.urdf.xacro \
        --name my_cube --x 0.5 --y 0.0 --z 0.8 --size 0.05 --color red
"""

import argparse
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node


class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner_moveit')
        
        # Gazebo spawn client
        self.gazebo_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        # MoveIt planning scene publisher
        self.planning_scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10)
        
        # Also publish to collision_object topic
        self.collision_object_pub = self.create_publisher(
            CollisionObject, '/collision_object', 10)
    
    def spawn_in_gazebo(self, args, mappings):
        """Spawn object in Gazebo simulation"""
        self.get_logger().info('Connecting to Gazebo /spawn_entity service...')
        
        if not self.gazebo_client.service_is_ready():
            self.gazebo_client.wait_for_service()
        
        request = SpawnEntity.Request()
        request.name = args.name
        
        urdf_file_path = os.path.join(
            get_package_share_directory(args.package), 'urdf', 'objects', args.urdf)
        xacro_file = xacro.process_file(urdf_file_path, mappings=mappings)
        request.xml = xacro_file.toxml()
        
        request.initial_pose.position.x = float(args.x)
        request.initial_pose.position.y = float(args.y)
        request.initial_pose.position.z = float(args.z)
        
        future = self.gazebo_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Gazebo: {future.result().status_message}')
            return future.result().success
        else:
            self.get_logger().error(f'Gazebo spawn failed: {future.exception()}')
            return False
    
    def add_to_moveit(self, args):
        """Add object to MoveIt planning scene as collision object"""
        self.get_logger().info(f'Adding {args.name} to MoveIt planning scene...')
        
        collision_object = CollisionObject()
        # Use world as frame_id - MoveIt's virtual_joint connects base_link to world,
        # so MoveIt will automatically handle the coordinate frame
        collision_object.header.frame_id = "world"
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = args.name
        
        # Determine object shape based on URDF type
        primitive = SolidPrimitive()
        
        if 'cube' in args.urdf:
            primitive.type = SolidPrimitive.BOX
            size = float(args.size) if args.size else 0.05
            primitive.dimensions = [size, size, size]
            
        elif 'box' in args.urdf:
            primitive.type = SolidPrimitive.BOX
            size_x = float(args.size_x) if args.size_x else 0.05
            size_y = float(args.size_y) if args.size_y else 0.03
            size_z = float(args.size_z) if args.size_z else 0.02
            primitive.dimensions = [size_x, size_y, size_z]
            
        elif 'cylinder' in args.urdf:
            primitive.type = SolidPrimitive.CYLINDER
            radius = float(args.radius) if args.radius else 0.025
            length = float(args.length) if args.length else 0.1
            primitive.dimensions = [length, radius]  # height, radius
            
        elif 'sphere' in args.urdf:
            primitive.type = SolidPrimitive.SPHERE
            radius = float(args.radius) if args.radius else 0.025
            primitive.dimensions = [radius]
            
        else:
            self.get_logger().warn(f'Unknown object type: {args.urdf}, defaulting to box')
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [0.05, 0.05, 0.05]
        
        # Set pose - coordinates are already in world frame (same as Gazebo)
        pose = Pose()
        pose.position.x = float(args.x)
        pose.position.y = float(args.y)
        pose.position.z = float(args.z)
        pose.orientation.w = 1.0
        
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        
        # Publish to collision_object topic (legacy method)
        self.collision_object_pub.publish(collision_object)
        self.get_logger().info(f'Published {args.name} to /collision_object topic')
        
        # Also publish via planning scene (preferred method for MoveIt)
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        
        # Publish multiple times to ensure MoveIt receives it
        import time
        for i in range(3):
            self.planning_scene_pub.publish(planning_scene)
            time.sleep(0.1)
        
        self.get_logger().info(f'MoveIt: Added {args.name} as collision object (frame: {collision_object.header.frame_id}, pose: [{args.x}, {args.y}, {args.z}])')
        return True


def main():
    parser = argparse.ArgumentParser(
        description='Spawn object in Gazebo AND add to MoveIt planning scene.')
    
    # Basic arguments
    parser.add_argument('--package', type=str, required=True, 
                        help='Package where URDF/XACRO file is located.')
    parser.add_argument('--urdf', type=str, required=True, 
                        help='URDF of the object to spawn.')
    parser.add_argument('--name', type=str, required=True, 
                        help='Name of the object to spawn.')
    parser.add_argument('--x', type=float, default=0.0, 
                        help='X position [meters].')
    parser.add_argument('--y', type=float, default=0.0, 
                        help='Y position [meters].')
    parser.add_argument('--z', type=float, default=0.0, 
                        help='Z position [meters].')
    
    # Object customization
    parser.add_argument('--color', type=str, default=None, 
                        help='Object color.')
    parser.add_argument('--mass', type=str, default=None, 
                        help='Object mass in kg.')
    parser.add_argument('--size', type=str, default=None, 
                        help='Cube side length in meters.')
    parser.add_argument('--radius', type=str, default=None, 
                        help='Sphere/cylinder radius in meters.')
    parser.add_argument('--length', type=str, default=None, 
                        help='Cylinder length in meters.')
    parser.add_argument('--size_x', type=str, default=None, 
                        help='Box X dimension in meters.')
    parser.add_argument('--size_y', type=str, default=None, 
                        help='Box Y dimension in meters.')
    parser.add_argument('--size_z', type=str, default=None, 
                        help='Box Z dimension in meters.')
    
    # Options
    parser.add_argument('--gazebo-only', action='store_true',
                        help='Only spawn in Gazebo, skip MoveIt.')
    parser.add_argument('--moveit-only', action='store_true',
                        help='Only add to MoveIt, skip Gazebo.')
    
    args, _ = parser.parse_known_args()
    
    # Build xacro mappings
    mappings = {"name": args.name}
    if args.color: mappings["color"] = args.color
    if args.mass: mappings["mass"] = args.mass
    if args.size: mappings["size"] = args.size
    if args.radius: mappings["radius"] = args.radius
    if args.length: mappings["length"] = args.length
    if args.size_x: mappings["size_x"] = args.size_x
    if args.size_y: mappings["size_y"] = args.size_y
    if args.size_z: mappings["size_z"] = args.size_z
    
    # Initialize ROS
    rclpy.init()
    spawner = ObjectSpawner()
    
    # Brief time for publishers to connect
    import time
    time.sleep(0.2)
    
    success = True
    
    # Spawn in Gazebo
    if not args.moveit_only:
        success = spawner.spawn_in_gazebo(args, mappings)
    
    # Add to MoveIt
    if success and not args.gazebo_only:
        spawner.add_to_moveit(args)
        time.sleep(0.2)
    
    spawner.get_logger().info('Done!')
    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
