#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        # Wait for AMCL to be ready (Subscriber Check)
        while self.publisher.get_subscription_count() == 0:
            self.get_logger().info('⏳ Waiting for AMCL (subscriber) on /initialpose...')
            time.sleep(1.0)
            
        # Short stabilization delay
        time.sleep(1.0)
        
        # Publish initial pose
        self.publish_initial_pose()
        self.get_logger().info('📍 Initial pose published to AMCL!')
        
    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Set position (center of map, approximately)
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        
        # Set orientation (facing forward)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        # Set covariance (uncertainty)
        # Diagonal: x, y, z, rot_x, rot_y, rot_z
        msg.pose.covariance[0] = 0.25  # x variance
        msg.pose.covariance[7] = 0.25  # y variance
        msg.pose.covariance[35] = 0.06853891909122467  # yaw variance
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    
    # Publish a few times to ensure it's received
    for _ in range(3):
        node.publish_initial_pose()
        time.sleep(0.5)
    
    node.get_logger().info('✅ Initial pose setup complete. Node shutting down.')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
