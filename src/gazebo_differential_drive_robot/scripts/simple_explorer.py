#!/usr/bin/env python3
"""
Simple Working Explorer - sends random goals without complex map reading
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import random
import math
import time


class SimpleExplorer(Node):
    def __init__(self):
        super().__init__('simple_explorer')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_count = 0
        self.start_time = time.time()
        self.duration = 60.0
        
    def send_goal(self):
        if time.time() - self.start_time >= self.duration:
            self.get_logger().info(f'Exploration done! Sent {self.goal_count} goals')
            rclpy.shutdown()
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Random position in turtlebot3_world bounds
        goal_msg.pose.pose.position.x = random.uniform(-1.5, 1.5)
        goal_msg.pose.pose.position.y = random.uniform(-1.5, 1.5)
        goal_msg.pose.pose.position.z = 0.0
        
        yaw = random.uniform(-math.pi, math.pi)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)
        
        self.goal_count += 1
        self.get_logger().info(f'Sending goal #{self.goal_count}: ({goal_msg.pose.pose.position.x:.2f}, {goal_msg.pose.pose.position.y:.2f})')
        
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected! Trying new goal...')
            time.sleep(1.0)
            self.send_goal()
            return
            
        self.get_logger().info('Goal accepted!')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
        
    def result_callback(self, future):
        self.get_logger().info('Goal completed! Sending next goal...')
        time.sleep(0.5)
        self.send_goal()


def main():
    rclpy.init()
    explorer = SimpleExplorer()
    
    explorer.get_logger().info('Waiting for Nav2...')
    explorer._action_client.wait_for_server()
    explorer.get_logger().info('Nav2 ready! Starting exploration...')
    
    time.sleep(2.0)
    explorer.send_goal()
    
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
