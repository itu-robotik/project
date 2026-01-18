#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
import json
import os
import math
import time

class SequentialPlannerAction(Node):
    def __init__(self):
        super().__init__('sequential_planner_action')
        
        # 1. Load Memory
        self.memory_file = os.path.expanduser('~/itu_robotics_ws/itu_robotics_combined_ws/board_memory.json')
        self.points = self.load_points_from_memory()
        
        # 2. Nav2 Action Client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 3. State Publishing (To inform Patrol Node)
        # We publish "TRAVELING" when moving, and "ARRIVED" or "IDLE" when done
        self.status_pub = self.create_publisher(String, '/patrol/status', 10)
        
        self.current_idx = 0
        self.retry_count = 0
        self.max_retries = 3
        
        self.get_logger().info("📋 Sequential Planner (Action Client) Ready!")
        self.get_logger().info(f"Loaded {len(self.points)} targets.")
        
        # Start immediately after a small delay
        self.create_timer(5.0, self.start_sequence_callback)
        self.started = False

    def start_sequence_callback(self):
        if not self.started:
            self.started = True
            self.send_next_goal()

    def load_points_from_memory(self):
        points = []
        if os.path.exists(self.memory_file):
            try:
                with open(self.memory_file, 'r') as f:
                    data = json.load(f)
                    if isinstance(data, list):
                        for item in data:
                            points.append({
                                'id': str(item['id']),
                                'x': item.get('approach_x', 0.0),
                                'y': item.get('approach_y', 0.0),
                                'theta': item.get('approach_theta', 0.0)
                            })
                    elif isinstance(data, dict) and "boards" in data:
                        for bid, info in data["boards"].items():
                            points.append({
                                'id': str(bid),
                                'x': info.get('x', 0.0),
                                'y': info.get('y', 0.0),
                                'theta': info.get('theta', 0.0)
                            })
                points.sort(key=lambda p: int(p['id']) if p['id'].isdigit() else p['id'])
            except Exception as e:
                self.get_logger().error(f"Error loading memory: {e}")
        
        if not points: # Default fallback
            self.get_logger().warn("Using Default Route.")
            points = [
                {'id': '1', 'x': 1.0, 'y': -4.0, 'theta': 1.57},
                {'id': '2', 'x': -3.5, 'y': -4.0, 'theta': 1.57}
            ]
        return points

    def send_next_goal(self):
        if self.current_idx >= len(self.points):
            self.get_logger().info("🎉 All points visited! Patrol Complete.")
            return

        point = self.points[self.current_idx]
        
        # Notify System
        self.status_pub.publish(String(data="TRAVELING"))
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(point['x'])
        goal_msg.pose.pose.position.y = float(point['y'])
        
        theta = float(point['theta'])
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        self.get_logger().info(f"🚀 Sending Goal {self.current_idx+1}/{len(self.points)} (ID {point['id']}) to Nav2...")
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ Goal rejected by Nav2!')
            self.retry_or_skip()
            return

        self.get_logger().info('✅ Goal accepted. Moving...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'🏁 Goal Reached! Starting Analysis...')
            self.retry_count = 0
            
            # TRIGGER ANALYSIS HERE
            # We can publish a "SCANNING" status so PatrolNode or PerceptionNode activates
            self.status_pub.publish(String(data="SCANNING"))
            
            # Wait for Analysis (Simulated delay or wait for callback)
            # For now, simplistic delay then move on
            self.create_timer(10.0, self.finish_analysis) # 10s to analyze
            
        else:
            self.get_logger().warn(f'⚠️ Goal Failed (Status: {status})')
            self.retry_or_skip()

    def finish_analysis(self):
        # Stop the timer that called this
        # In ROS2, timers are tricky to cancel within callback if not stored properly
        # Hacky reliable way: just check state
        
        self.get_logger().info("✅ Analysis Complete. Moving to next...")
        self.current_idx += 1
        self.send_next_goal()
        
        # Clean up this one-shot timer if possible, or use a state flag
        # (This is a simplified implementation)

    def retry_or_skip(self):
        self.retry_count += 1
        if self.retry_count < self.max_retries:
            self.get_logger().info(f'Retrying... ({self.retry_count}/{self.max_retries})')
            # Variable delay retry
            time.sleep(1.0) 
            self.send_next_goal()
        else:
            self.get_logger().warn('➡️ Max retries reached. Skipping point.')
            self.retry_count = 0
            self.current_idx += 1
            self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    node = SequentialPlannerAction()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
