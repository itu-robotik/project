#!/usr/bin/env python3
"""
Smart Map Explorer
Explores the map using free space sampling and nearby goal preference.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import random
import time
import math
import numpy as np


class SmartExplorer(Node):
    def __init__(self):
        super().__init__('smart_explorer')
        
        # Nav2 action client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Map data
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.map_width = None
        self.map_height = None
        self.free_cells = []
        
        # Robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        # Timing
        self.exploration_duration = 60.0  # 1 minute
        self.start_time = None
        self.goal_count = 0
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        self.get_logger().info('Smart Explorer initialized. Waiting for map...')
        
    def map_callback(self, msg):
        """Process the occupancy grid map."""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin.position
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        
        # Extract free cells (value 0 = free, -1 = unknown, 100 = occupied)
        self.free_cells = []
        for y in range(self.map_height):
            for x in range(self.map_width):
                if self.map_data[y, x] == 0:  # Free space
                    # Convert grid coordinates to world coordinates
                    world_x = self.map_origin.x + (x + 0.5) * self.map_resolution
                    world_y = self.map_origin.y + (y + 0.5) * self.map_resolution
                    self.free_cells.append((world_x, world_y))
        
        self.get_logger().info(f'Map received: {len(self.free_cells)} free cells found')
        
    def odom_callback(self, msg):
        """Track robot position."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
    def get_nearby_free_point(self):
        """Get a random free point, preferring nearby locations."""
        if not self.free_cells:
            # Fallback to random point if no map
            return (
                random.uniform(-2.0, 2.0),
                random.uniform(-2.0, 2.0)
            )
        
        # Calculate distances to all free cells
        distances = []
        for x, y in self.free_cells:
            dist = math.sqrt((x - self.robot_x)**2 + (y - self.robot_y)**2)
            distances.append((dist, x, y))
        
        # Sort by distance
        distances.sort()
        
        # Sample from nearby cells (bias towards closer ones)
        # Use weighted sampling: closer = higher probability
        max_samples = min(100, len(distances))
        sample_pool = distances[:max_samples]
        
        # Weight closer points more heavily (inverse distance weighting)
        weights = [1.0 / (1.0 + d[0]) for d in sample_pool]
        total_weight = sum(weights)
        weights = [w / total_weight for w in weights]
        
        # Randomly select based on weights
        selected = random.choices(sample_pool, weights=weights, k=1)[0]
        return (selected[1], selected[2])
        
    def start_exploration(self):
        """Start the exploration process."""
        # Wait for Nav2 to be ready
        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server()
        
        # Wait for map
        self.get_logger().info('Waiting for map data...')
        while self.map_data is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
        
        self.get_logger().info('Map ready! Starting smart exploration for 60 seconds...')
        self.start_time = time.time()
        self.send_random_goal()
        
    def send_random_goal(self):
        """Send a smart navigation goal."""
        elapsed = time.time() - self.start_time
        if elapsed >= self.exploration_duration:
            self.get_logger().info(f'Exploration complete! Sent {self.goal_count} goals in 60 seconds.')
            rclpy.shutdown()
            return
        
        # Get a nearby free point
        target_x, target_y = self.get_nearby_free_point()
        
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.position.z = 0.0
        
        # Random orientation
        yaw = random.uniform(-math.pi, math.pi)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)
        
        dist = math.sqrt((target_x - self.robot_x)**2 + (target_y - self.robot_y)**2)
        self.goal_count += 1
        self.get_logger().info(
            f'Goal #{self.goal_count}: ({target_x:.2f}, {target_y:.2f}) '
            f'distance: {dist:.2f}m (elapsed: {elapsed:.1f}s)'
        )
        
        # Send goal
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected, sending new goal...')
            self.send_random_goal()
            return
            
        self.get_logger().info('Goal accepted!')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
        
    def result_callback(self, future):
        """Handle goal completion."""
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached! Sending new goal...')
        else:
            self.get_logger().warn(f'Goal failed with status: {status}. Sending new goal...')
        
        # Small delay before next goal
        time.sleep(0.5)
        self.send_random_goal()
        
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback (optional)."""
        pass


def main():
    rclpy.init()
    
    explorer = SmartExplorer()
    
    # Wait a bit for everything to initialize
    time.sleep(5.0)
    
    # Start exploration
    explorer.start_exploration()
    
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        explorer.destroy_node()


if __name__ == '__main__':
    main()
