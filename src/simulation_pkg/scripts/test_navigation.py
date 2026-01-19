#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import sys

class NavigationTester(Node):
    def __init__(self):
        super().__init__('navigation_tester')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
    def send_goal(self, x, y, theta=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Simple yaw to quaternion (assuming flat ground)
        import math
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        self.get_logger().info(f'🚀 Sending robot to x={x}, y={y}, theta={theta}...')
        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by Nav2!')
            return

        self.get_logger().info('✅ Goal accepted. Moving...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'🏁 Navigation Result Status: {status}')
        if status == 4: # SUCCEEDED
            self.get_logger().info('🎉 SUCCESS: Robot reached the goal!')
            self.get_logger().info('Ready for Next Test.')
        else:
            self.get_logger().warn('⚠️ FAILURE: Robot did not reach the goal.')
        
        # Shutdown after one goal
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 3:
        print("Usage: python3 test_navigation.py <x> <y> [theta]")
        print("Example: python3 test_navigation.py -1.32 -3.74 1.57")
        return

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    theta = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

    node = NavigationTester()
    node.send_goal(x, y, theta)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
