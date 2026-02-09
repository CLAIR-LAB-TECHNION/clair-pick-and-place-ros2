#!/usr/bin/python3

"""
RemoveObjectMoveIt.py - Remove collision object from MoveIt planning scene

Usage:
    python3 RemoveObjectMoveIt.py --name table1
"""

import argparse
from moveit_msgs.msg import CollisionObject, PlanningScene
import rclpy
from rclpy.node import Node


class ObjectRemover(Node):
    def __init__(self):
        super().__init__('object_remover_moveit')
        
        # MoveIt planning scene publisher
        self.planning_scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10)
        
        # Also publish to collision_object topic
        self.collision_object_pub = self.create_publisher(
            CollisionObject, '/collision_object', 10)
    
    def remove_from_moveit(self, object_name):
        """Remove object from MoveIt planning scene"""
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
        description='Remove collision object from MoveIt planning scene.')
    
    parser.add_argument('--name', type=str, required=True, 
                        help='Name of the object to remove.')
    
    args = parser.parse_args()
    
    # Initialize ROS
    rclpy.init()
    remover = ObjectRemover()
    
    import time
    time.sleep(0.2)
    
    # Remove from MoveIt
    remover.remove_from_moveit(args.name)
    time.sleep(0.2)
    
    remover.get_logger().info('Done!')
    remover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
