#!/usr/bin/python3

"""
RemoveObject.py - Remove objects from BOTH Gazebo AND MoveIt planning scene

This script:
1. Removes the object from Gazebo simulation (using /delete_entity service)
2. Removes the object from MoveIt's planning scene (for collision avoidance)

Usage:
    ros2 run ros2srrc_execution RemoveObject.py --name cube1
    
    # Remove from Gazebo only
    ros2 run ros2srrc_execution RemoveObject.py --name cube1 --gazebo-only
    
    # Remove from MoveIt only
    ros2 run ros2srrc_execution RemoveObject.py --name cube1 --moveit-only
"""

import argparse
import time
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity, GetModelState
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene


class ObjectRemover(Node):
    def __init__(self):
        super().__init__('object_remover')
        
        # Gazebo clients
        self.gazebo_delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.gazebo_get_state_client = self.create_client(GetModelState, '/gazebo/get_model_state')
        
        # MoveIt clients and publishers
        self.moveit_get_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.planning_scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10)
        self.collision_object_pub = self.create_publisher(
            CollisionObject, '/collision_object', 10)
    
    def check_exists_in_gazebo(self, object_name):
        """Check if object exists in Gazebo"""
        if not self.gazebo_get_state_client.service_is_ready():
            # If service not available, assume object might exist and let removal attempt handle it
            return None
        
        request = GetModelState.Request()
        request.model_name = object_name
        request.relative_entity_name = "world"
        
        future = self.gazebo_get_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            # If we get a result, the object exists
            return True
        else:
            # Service call failed or object doesn't exist
            return False
    
    def check_exists_in_moveit(self, object_name):
        """Check if object exists in MoveIt planning scene"""
        if not self.moveit_get_scene_client.service_is_ready():
            # If service not available, assume object might exist
            return None
        
        try:
            request = GetPlanningScene.Request()
            # Request world collision objects
            # Use the components field correctly - request all world objects
            components = PlanningSceneComponents()
            components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
            request.components = components
            
            future = self.moveit_get_scene_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                # Check if object name is in the collision objects
                scene = future.result().scene
                for obj in scene.world.collision_objects:
                    if obj.id == object_name:
                        return True
                return False
            else:
                # Service call failed
                return None
        except Exception as e:
            # If check fails, assume object might exist and let removal handle it
            self.get_logger().warn(f'Could not check MoveIt planning scene: {e}')
            return None
    
    def remove_from_gazebo(self, object_name):
        """Remove object from Gazebo simulation"""
        # Check if object exists first
        exists = self.check_exists_in_gazebo(object_name)
        
        if exists is False:
            self.get_logger().info(f'Gazebo: Object {object_name} does not exist, skipping removal.')
            return True  # Not an error if it doesn't exist
        
        if exists is None:
            self.get_logger().warn(f'Gazebo: Could not verify if {object_name} exists, attempting removal anyway...')
        
        self.get_logger().info(f'Removing {object_name} from Gazebo...')
        
        if not self.gazebo_delete_client.service_is_ready():
            self.get_logger().info('Waiting for /delete_entity service...')
            if not self.gazebo_delete_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('/delete_entity service not available!')
                return False
        
        request = DeleteEntity.Request()
        request.name = object_name
        
        future = self.gazebo_delete_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Gazebo: Successfully removed {object_name}')
                return True
            else:
                self.get_logger().warn(f'Gazebo: {future.result().status_message}')
                return False
        else:
            self.get_logger().error(f'Gazebo delete failed: {future.exception()}')
            return False
    
    def remove_from_moveit(self, object_name):
        """Remove object from MoveIt planning scene"""
        # Check if object exists first
        exists = self.check_exists_in_moveit(object_name)
        
        if exists is False:
            self.get_logger().info(f'MoveIt: Object {object_name} does not exist in planning scene, skipping removal.')
            return True  # Not an error if it doesn't exist
        
        if exists is None:
            self.get_logger().warn(f'MoveIt: Could not verify if {object_name} exists, attempting removal anyway...')
        
        self.get_logger().info(f'Removing {object_name} from MoveIt planning scene...')
        
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = object_name
        collision_object.operation = CollisionObject.REMOVE
        
        # Publish to collision_object topic
        self.collision_object_pub.publish(collision_object)
        
        # Also publish via planning scene
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        self.planning_scene_pub.publish(planning_scene)
        
        self.get_logger().info(f'MoveIt: Removed {object_name} from collision objects')
        return True


def main():
    parser = argparse.ArgumentParser(
        description='Remove objects from Gazebo and/or MoveIt planning scene.')
    
    parser.add_argument('--name', type=str, required=True, 
                        help='Name of the object to remove.')
    parser.add_argument('--gazebo-only', action='store_true',
                        help='Remove from Gazebo only (not MoveIt).')
    parser.add_argument('--moveit-only', action='store_true',
                        help='Remove from MoveIt only (not Gazebo).')
    
    args = parser.parse_args()
    
    # Initialize ROS
    rclpy.init()
    remover = ObjectRemover()
    
    # Give time for publishers/clients to connect
    time.sleep(0.3)
    
    success = True
    
    # Remove from Gazebo (unless moveit-only)
    if not args.moveit_only:
        gazebo_success = remover.remove_from_gazebo(args.name)
        if not gazebo_success:
            success = False
    
    # Remove from MoveIt (unless gazebo-only)
    if not args.gazebo_only:
        moveit_success = remover.remove_from_moveit(args.name)
        if not moveit_success:
            success = False
    
    if success:
        remover.get_logger().info(f'Successfully removed {args.name}!')
    else:
        remover.get_logger().warn(f'Some operations may have failed for {args.name}')
    
    remover.destroy_node()
    rclpy.shutdown()
    
    exit(0 if success else 1)


if __name__ == '__main__':
    main()
