#!/usr/bin/env python3
"""
Sequential Goal Sender (Upgraded with Nav2)
Original: sequential_goal_gpt.py
Re-implemented to use Nav2 Action Client for obstacle avoidance.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from action_msgs.msg import GoalStatus
import time
import math

def euler_from_quaternion(x, y, z, w):
    """Convert quaternion to euler angles"""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

class SequentialGoToPoints(Node):
    def __init__(self):
        super().__init__('sequential_go_to_points_nav2')
        
        # Action client for navigation
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Initial Pose Publisher
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        
        # Robot spawn position is (-2.0, -0.5) if using my_robot_world.launch.py default
        # But we updated launch to spawn at (0,0) in Step 1047.
        # So we assume (0,0).
        
        # Safer goal points closer to center
        self.points = [
            {'x': 3.90, 'y': 0.68, 'name': 'Point 1'},
            {'x': 1.42, 'y': 2.60, 'name': 'Point 2'},
            {'x': 2.63, 'y': -1.45, 'name': 'Point 3'},
        ]
        self.current_goal_idx = 0
        self.retry_count = 0
        self.max_retries = 2
        
        # TF Buffer for Global Pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer for logging pose (1Hz)
        self.create_timer(1.0, self.log_pose)

    def log_pose(self):
        try:
            # Get transform from map to base_link
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            q = t.transform.rotation
            _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
            
            self.get_logger().info(f"📍 Robot Global Pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad")
        except Exception as e:
            # TF might not be ready immediately
            pass

    def publish_initial_pose(self, x=0.0, y=0.0):
        """Publish initial pose at robot start position"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
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
        
        # Orient towards right
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
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'🎉 Goal #{self.current_goal_idx + 1} SUCCEEDED!')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(f'⚠️ Goal #{self.current_goal_idx + 1} ABORTED (obstacle/unreachable)')
        else:
            self.get_logger().info(f'Goal finished with status: {status}')
        
        # Wait 5 seconds
        self.get_logger().info('⏳ Waiting 5 seconds...')
        time.sleep(5.0)
        
        # Move to next
        self.current_goal_idx += 1
        self.send_next_goal()

def main():
    rclpy.init()
    node = SequentialGoToPoints()
    
    # Wait a bit
    time.sleep(2.0)
    
    # 1. Publish Initial Pose
    node.publish_initial_pose(0.0, 0.0)
    
    # 2. Add delay for AMCL
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
