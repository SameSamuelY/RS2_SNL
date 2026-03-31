#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

class AddGroundNode(Node):
    def __init__(self):
        super().__init__('add_ground_node')
        self.publisher = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.get_logger().info('Publishing ground plane...')
        self.add_ground_plane()
        
    def add_ground_plane(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        
        # Create collision object for ground
        ground = CollisionObject()
        ground.header = Header(frame_id='world')  # or 'base_link' / 'base'
        ground.id = 'ground_plane'
        ground.operation = CollisionObject.ADD
        
        # Define a box (width, depth, thickness)
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [2.0, 2.0, 0.05]  # 2m x 2m x 0.05m
        
        # Position it below the robot (adjust z based on your robot's base height)
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = -0.1  # Half thickness below surface
        
        ground.primitives.append(primitive)
        ground.primitive_poses.append(pose)
        
        scene.world.collision_objects.append(ground)
        self.publisher.publish(scene)
        self.get_logger().info('Ground plane added.')
        
def main(args=None):
    rclpy.init(args=args)
    node = AddGroundNode()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()