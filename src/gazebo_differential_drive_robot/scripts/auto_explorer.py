#!/usr/bin/env python3
"""
Auto Explorer - Automatically sets initial pose and explores very close safe points
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
import random
import math
import time
import numpy as np


class AutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')
        
        # Action client for navigation
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publisher for initial pose
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        
        # Subscribe to odometry to track position
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # Subscribe to map for free space checking
        # Map server publishes with Transient Local durability, so we must match it
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos
        )
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.map_data = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_width = None
        self.map_height = None
        
        self.goal_count = 0
        self.start_time = time.time()
        self.duration = 60.0  # 1 minute
        self.initial_pose_set = False
        
        # MUCH closer range for safe exploration (meters)
        self.min_distance = 0.3  # Very close
        self.max_distance = 2.0  # Stay nearby
        
        
        # Timer to check readiness and start
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.exploration_active = False
        
    def timer_callback(self):
        """Periodically check if we can start or need to retry"""
        if self.exploration_active:
            return  # Already running finding/navigating
            
        if self.map_data is None:
            self.get_logger().warn('Waiting for map data...', throttle_duration_sec=2.0)
            return
            
        if not self.initial_pose_set:
             self.publish_initial_pose()
             return

        # If map is ready and initial pose set, start exploration
        # We also need to wait a bit for AMCL to localize, which we partly handle by delay
        # But let's just try to send a goal now.
        self.send_goal()

    def odom_callback(self, msg):
        """Track robot position"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
    def map_callback(self, msg):
        """Store map data for free space checking"""
        self.get_logger().info('Map received!', once=True)
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        
    def is_free_space(self, x, y):
        """Check if a point is in free space on the map"""
        if self.map_data is None:
            return False
            
        # Convert world coordinates to grid coordinates
        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)
        
        # Check bounds
        if grid_x < 0 or grid_x >= self.map_width or grid_y < 0 or grid_y >= self.map_height:
            return False
            
        # Check if free (0 = free, -1 = unknown, 100 = occupied)
        cell_value = self.map_data[grid_y, grid_x]
        return cell_value == 0
        
    def publish_initial_pose(self):
        """Publish initial pose estimate at origin"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set at origin
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        # Set covariance (uncertainty)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25  # x
        msg.pose.covariance[7] = 0.25  # y
        msg.pose.covariance[35] = 0.06853  # yaw
        
        self.initial_pose_pub.publish(msg)
        self.get_logger().info('Published initial pose at (0, 0)')
        self.initial_pose_set = True
        
    def get_random_valid_goal(self):
        """Generate a random goal anywhere in the map that is in free space"""
        max_attempts = 100  # Try many times to find free space if needed
        
        if self.map_data is None:
            return None, None

        # Map bounds in world coordinates
        map_min_x = self.map_origin_x
        map_max_x = self.map_origin_x + (self.map_width * self.map_resolution)
        map_min_y = self.map_origin_y
        map_max_y = self.map_origin_y + (self.map_height * self.map_resolution)

        for attempt in range(max_attempts):
            # Random position within map bounds
            goal_x = random.uniform(map_min_x, map_max_x)
            goal_y = random.uniform(map_min_y, map_max_y)
            
            # Check if this point is in free space
            if self.is_free_space(goal_x, goal_y):
                # Extra check: ensure distance is reasonable (not 0) and validation passes
                return goal_x, goal_y
                
        self.get_logger().warn('Could not find free space after many attempts')
        return None, None
        
    def send_goal(self):
        """Send a random navigation goal"""
        self.exploration_active = True
        
        # Get random valid goal
        goal_x, goal_y = self.get_random_valid_goal()
        
        if goal_x is None or goal_y is None:
            self.get_logger().warn('Failed to generate valid goal, retrying in 2s...')
            # Retry via a one-shot timer instead of blocking sleep
            self.exploration_active = False # Allow timer to pick it up or create a specific timer
            # Let's simple set exploration_active false, so the main timer loop picks it up again
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.position.z = 0.0
        
        # Random orientation
        yaw = random.uniform(-math.pi, math.pi)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)
        
        dist = math.sqrt((goal_x - self.robot_x)**2 + (goal_y - self.robot_y)**2)
        self.goal_count += 1
        
        elapsed = time.time() - self.start_time
        self.get_logger().info(
            f'Goal #{self.goal_count}: ({goal_x:.2f}, {goal_y:.2f}) '
            f'distance: {dist:.2f}m (elapsed: {elapsed:.1f}s)'
        )
        
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected! Trying new goal...')
            # Retry immediately (or let timer pick up if we set active=False)
            self.send_goal() 
            return
            
        self.get_logger().info('Goal accepted!')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
        
    def result_callback(self, future):
        """Handle goal result"""
        self.get_logger().info('Goal completed! Sending next goal...')
        # Send next goal
        self.send_goal()


def main():
    rclpy.init()
    explorer = AutoExplorer()
    
    # Wait for Nav2 (blocking here is okay-ish, or move to timer)
    # Better to just let it run.
    explorer.get_logger().info('Waiting for Nav2 action server...')
    explorer.get_logger().info('(This might take a moment if Nav2 is starting up)')
    
    # We can check server availability in the timer too, 
    # but waiting once here is common practice.
    if not explorer._action_client.wait_for_server(timeout_sec=10.0):
        explorer.get_logger().warn('Nav2 Action Server not available yet! continuing anyway...')

    explorer.get_logger().info('Auto Explorer Node Started. Waiting for Map and Initial Pose...')
    
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        explorer.destroy_node()


if __name__ == '__main__':
    main()
