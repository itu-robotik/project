#!/usr/bin/env python3
"""
Sequential Goal Sender - Fixed Version
Visits 3 specific points, waiting 10 seconds at each.
Handles failures gracefully and skips to next goal if stuck.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus
import time

class SequentialGoalSender(Node):
    def __init__(self):
        super().__init__('sequential_goal_sender')
        
        # Action client for navigation
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Initial Pose Publisher
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        
        # Robot spawn position is (-2.0, -0.5)
        # These are safer goal points closer to center of map
        self.points = [
            {'x': -0.5, 'y': 0.0, 'name': 'Point 1'},   # First, move forward
            {'x': 0.5, 'y': 0.5, 'name': 'Point 2'},    # Second point
            {'x': 1.0, 'y': -0.5, 'name': 'Point 3'},   # Third point
        ]
        self.current_goal_idx = 0
        self.retry_count = 0
        self.max_retries = 2

    def publish_initial_pose(self, x=0.0, y=0.0):
        """Publish initial pose at robot start position"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        # Covariance
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853
        
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f'Published Initial Pose at ({x}, {y})')

    def send_next_goal(self):
        """Send the next goal in the list"""
        if self.current_goal_idx >= len(self.points):
            self.get_logger().info('✅ All points visited! Exiting...')
            rclpy.shutdown()
            return

        point = self.points[self.current_goal_idx]
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = point['x']
        goal_msg.pose.pose.position.y = point['y']
        goal_msg.pose.pose.position.z = 0.0
        
        # Default orientation
        goal_msg.pose.pose.orientation.w = 1.0
        goal_msg.pose.pose.orientation.z = 0.0
        
        self.get_logger().info(f'🎯 Sending Goal #{self.current_goal_idx + 1} "{point["name"]}": ({point["x"]:.2f}, {point["y"]:.2f})...')
        
        self._action_client.wait_for_server()
        
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('⚠️ Goal rejected!')
            self.retry_count += 1
            if self.retry_count < self.max_retries:
                self.get_logger().info(f'Retrying... ({self.retry_count}/{self.max_retries})')
                time.sleep(2.0)
                self.send_next_goal()
            else:
                self.get_logger().warn('Max retries reached, skipping to next goal...')
                self.retry_count = 0
                self.current_goal_idx += 1
                self.send_next_goal()
            return

        self.get_logger().info('✅ Goal accepted!')
        self.retry_count = 0
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        status = result.status
        
        status_names = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }
        status_name = status_names.get(status, f"UNKNOWN({status})")
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'🎉 Goal #{self.current_goal_idx + 1} SUCCEEDED!')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(f'⚠️ Goal #{self.current_goal_idx + 1} ABORTED (obstacle/unreachable)')
        else:
            self.get_logger().info(f'Goal finished with status: {status_name}')
        
        # Wait at destination
        self.get_logger().info('⏳ Waiting 5 seconds...')
        time.sleep(5.0)
        
        # Move to next goal
        self.current_goal_idx += 1
        self.send_next_goal()

def main():
    rclpy.init()
    node = SequentialGoalSender()
    
    # Wait a bit for connections
    # Wait more for AMCL to come online
    node.get_logger().info('⏳ Waiting 15s for Navigation/AMCL to fully initialize...')
    time.sleep(15.0)
    
    # 1. Publish Initial Pose at robot spawn position (Send multiple times to be sure)
    for i in range(3):
        node.publish_initial_pose(-2.0, -0.5)
        time.sleep(1.0)
    
    node.get_logger().info('📡 Initial Pose published repeatedly.')
    
    # 2. Wait for AMCL localization
    node.get_logger().info('⏳ Waiting 8s for AMCL localization...')
    time.sleep(8.0)
    
    # 3. Send First Goal
    node.send_next_goal()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
