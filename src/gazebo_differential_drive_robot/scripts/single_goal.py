#!/usr/bin/env python3
"""
Single Goal Sender
Sends the robot to a specific hardcoded point (x: 3.9, y: 0.67) and then exits.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import time

class SingleGoalSender(Node):
    def __init__(self):
        super().__init__('single_goal_sender')
        
        # Action client for navigation
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Initial Pose Publisher
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        
        self.goal_sent = False

    def publish_initial_pose(self):
        """Publish initial pose at origin (0,0)"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        # Covariance
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853
        
        self.initial_pose_pub.publish(msg)
        self.get_logger().info('Published Initial Pose at (0,0)')

    def send_specific_goal(self):
        """Send goal to x=3.90, y=0.67"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Target Point from user
        goal_msg.pose.pose.position.x = 3.9034337997436523
        goal_msg.pose.pose.position.y = 0.6788737177848816
        goal_msg.pose.pose.position.z = 0.0
        
        # Orient towards right (approx)
        goal_msg.pose.pose.orientation.w = 1.0
        goal_msg.pose.pose.orientation.z = 0.0
        
        self.get_logger().info(f'Sending goal to ({goal_msg.pose.pose.position.x:.2f}, {goal_msg.pose.pose.position.y:.2f})...')
        
        self._action_client.wait_for_server()
        
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted!')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == 4: # SUCCEEDED
             self.get_logger().info('Goal SUCCEEDED!')
        else:
             self.get_logger().info(f'Goal finished with status: {result.status}')
        
        # Shutdown after reachable
        self.get_logger().info('Exiting...')
        rclpy.shutdown()

def main():
    rclpy.init()
    node = SingleGoalSender()
    
    # Wait a bit for connections
    time.sleep(2.0)
    
    # 1. Publish Initial Pose
    node.publish_initial_pose()
    
    # 2. Add delay for localization (AMCL)
    node.get_logger().info('Waiting 5s for AMCL localization...')
    time.sleep(5.0)
    
    # 3. Send Goal
    node.send_specific_goal()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # If rclpy.shutdown() is called, spin might raise ExternalShutdownException in newer ROS2
        pass

if __name__ == '__main__':
    main()
