#!/usr/bin/env python3
"""
Sequential Planner with Memory Integration
Integrates Emre Sarac's Planner Logic (Priority, Recycling, Memory) into a Nav2 Action Client node.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Path

import json
import os
import math
import time
from typing import List, Dict, Optional

# Import the MemoryManager class (must be in the same folder)
from memory_manager import MemoryManager

class SequentialPlannerAction(Node):
    def __init__(self):
        super().__init__('sequential_planner_action')
        
        # 1. Initialize Memory Manager
        # We assume board_memory.json is in the workspace root or same param as before
        workspace_root = os.path.expanduser('~/itu_robotics_ws/itu_robotics_combined_ws')
        memory_path = os.path.join(workspace_root, 'board_memory.json')
        self.memory = MemoryManager(memory_file=memory_path)
        
        # 2. Load Static Board Positions
        script_dir = os.path.dirname(os.path.realpath(__file__))
        positions_path = os.path.join(script_dir, 'board_positions.json')
        self.board_positions = self.load_board_positions(positions_path)

        # MERGE MEMORY POSITIONS
        # Use coordinates from memory if they exist, overriding static file
        for bid, bdata in self.memory.boards.items():
            if bid in self.board_positions and 'x' in bdata and 'y' in bdata:
                # Check for validity (0,0 is suspicious if originally -5, -4)
                if bdata['x'] != 0 or bdata['y'] != 0:
                     self.board_positions[bid]['x'] = bdata['x']
                     self.board_positions[bid]['y'] = bdata['y']
                     self.board_positions[bid]['theta'] = bdata.get('theta', self.board_positions[bid]['theta'])
                     self.get_logger().info(f"📍 Updated Board {bid} pos from Memory: ({bdata['x']}, {bdata['y']})")
        
        # 3. Nav2 Action Client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 4. Publishers/Subscribers
        self.status_pub = self.create_publisher(String, '/patrol/status', 10)
        
        # Subscribe to /poster_analysis to keep memory updated
        self.poster_sub = self.create_subscription(
            String, 
            '/poster_analysis', 
            self.poster_analysis_callback, 
            10
        )
        
        # Internal State
        self.current_goal_board_id = None
        self.retry_count = 0
        self.max_retries = 3
        self.is_analyzing = False
        
        # Round Robin State
        self.all_board_ids = sorted([int(k) for k in self.board_positions.keys()])
        self.round_robin_index = 0
        
        # Config
        self.recent_visit_threshold_minutes = 10
        
        self.get_logger().info("🧠 Sequential Planner (Smart Mode) Ready!")
        self.get_logger().info(f"Loaded {len(self.board_positions)} static board positions.")
        
        # Start the loop
        self.timer = self.create_timer(2.0, self.planning_loop_callback)
        self.is_moving = False
        
        # 5. TF Listener and Path Subscriber for "Board on Path" Logic
        from tf2_ros import Buffer, TransformListener
        from nav_msgs.msg import Path
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.nav2_path = None
        self.current_robot_position = None
        
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )
        
        self.previous_goal_board_id = None

    def path_callback(self, msg: Path):
        self.nav2_path = msg
        if len(msg.poses) > 0:
            first_pose = msg.poses[0].pose.position
            self.current_robot_position = (first_pose.x, first_pose.y)

    def calculate_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def point_to_line_distance(self, px, py, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            return math.sqrt((px - x1)**2 + (py - y1)**2)
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx*dx + dy*dy)))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)

    def point_to_path_distance(self, px, py, path):
        if path is None or len(path.poses) < 2:
            return float('inf')
        min_dist = float('inf')
        # Check every segment (heavy but accurate)
        # Optimized: only check every Nth point or if path is simplified 
        # But for robustness we check segments.
        for i in range(len(path.poses) - 1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i+1].pose.position
            dist = self.point_to_line_distance(px, py, p1.x, p1.y, p2.x, p2.y)
            if dist < min_dist:
                min_dist = dist
        return min_dist

    def find_boards_on_path(self, from_id, to_id, max_distance=1.0):
        """Check if any boards are on the way to the target"""
        if from_id not in self.board_positions or to_id not in self.board_positions:
            return []
            
        from_pos = self.board_positions[from_id]
        to_pos = self.board_positions[to_id]
        
        boards_on_path = []
        for bid, pos in self.board_positions.items():
            if bid == from_id or bid == to_id:
                continue
            
            # Skip if visited recently
            if self.memory.is_recently_visited(bid, self.recent_visit_threshold_minutes):
                continue

            dist = float('inf')
            if self.nav2_path and len(self.nav2_path.poses) > 0:
                dist = self.point_to_path_distance(pos['x'], pos['y'], self.nav2_path)
            else:
                 # Fallback to straight line
                dist = self.point_to_line_distance(
                    pos['x'], pos['y'], 
                    from_pos['x'], from_pos['y'], 
                    to_pos['x'], to_pos['y']
                )
            
            if dist <= max_distance:
                boards_on_path.append(bid)
        
        return boards_on_path

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def select_closest_by_priority(self, revisit_list, current_pos):
        if not revisit_list: return None
        if len(revisit_list) == 1: return revisit_list[0]
        
        if current_pos is None: 
            current_pos = (0,0) # Fallback

        max_priority = revisit_list[0]['priority']
        same_priority = [x for x in revisit_list if x['priority'] == max_priority]
        
        if len(same_priority) == 1:
            return same_priority[0]
            
        # Find closest
        best_board = None
        min_dist = float('inf')
        
        for item in same_priority:
            bid = item['board_id']
            if bid in self.board_positions:
                bpos = self.board_positions[bid]
                dist = self.calculate_distance(current_pos[0], current_pos[1], bpos['x'], bpos['y'])
                if dist < min_dist:
                    min_dist = dist
                    best_board = item
        
        if best_board:
            self.get_logger().info(f"📏 Selected closest board: {best_board['board_id']} (Dist: {min_dist:.2f}m)")
            return best_board
        return same_priority[0]

    def load_board_positions(self, filename: str) -> dict:
        if not os.path.exists(filename):
            self.get_logger().warn(f"⚠️ {filename} not found! Using fallback.")
            return {}
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                data = json.load(f)
                # Convert keys to int
                return {int(k): v for k, v in data.items()}
        except Exception as e:
            self.get_logger().error(f"Error loading positions: {e}")
            return {}

    def poster_analysis_callback(self, msg):
        """Update memory when a poster is analyzed"""
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f"📥 Received Analysis for Board {data.get('board_id')}")
            self.memory.update_board(data)
            
            # If we were traveling to this board, we might consider it 'arrived' 
            # but usually Nav2 finishes first, then we analyze.
        except Exception as e:
            self.get_logger().error(f"Failed to process analysis: {e}")

    def planning_loop_callback(self):
        """Main loop: Decides where to go next if idle"""
        if self.is_moving or self.is_analyzing:
            return

        # Update Robot Position
        current_pos = self.get_robot_pose()
        if current_pos:
            self.current_robot_position = current_pos

        # DECISION LOGIC (Strict Port)
        
        # 1. Check Revisit List (Expired, Unclear, Duplicate)
        revisit_list = self.memory.get_boards_needing_revisit(self.recent_visit_threshold_minutes)
        target_board_id = None
        
        if revisit_list:
            # Pick highest priority AND closest
            target_item = self.select_closest_by_priority(revisit_list, current_pos)
            if target_item:
                target_board_id = target_item['board_id']
                self.get_logger().info(f"🎯 Priority Target: Board {target_board_id} ({target_item['reason']})")
        else:
            # 2. Round Robin
            available_boards = []
            for bid in self.all_board_ids:
                if not self.memory.is_recently_visited(bid, self.recent_visit_threshold_minutes):
                    available_boards.append(bid)
            
            if not available_boards:
                self.get_logger().info("⏰ All boards visited recently. Resetting filter.")
                available_boards = list(self.all_board_ids)
                
            if not available_boards:
                return

            if self.round_robin_index >= len(available_boards):
                self.round_robin_index = 0
            
            target_board_id = available_boards[self.round_robin_index]
            self.get_logger().info(f"🔄 Round Robin Target: Board {target_board_id}")
            self.round_robin_index = (self.round_robin_index + 1) % len(available_boards)

        # 3. Path Optimization (Check for boards on the way)
        if target_board_id and self.previous_goal_board_id:
            on_path = self.find_boards_on_path(self.previous_goal_board_id, target_board_id)
            if on_path:
                short_term_target = on_path[0]
                self.get_logger().info(f"🛣️  Board {short_term_target} is ON THE WAY to {target_board_id}. Visiting it first!")
                # Override target
                target_board_id = short_term_target
                # Note: We don't increment round robin index here because we want to resume original target later

        # EXECUTE
        if target_board_id:
            # Only send if different from current or if we are idle
            if target_board_id != self.current_goal_board_id or not self.is_moving:
                self.send_goal_to_board(target_board_id)
                self.previous_goal_board_id = self.current_goal_board_id

    def send_goal_to_board(self, board_id: int):
        if board_id not in self.board_positions:
            self.get_logger().error(f"❌ Unknown Board ID {board_id}")
            return

        self.current_goal_board_id = board_id
        pos = self.board_positions[board_id]
        
        # Notify
        self.status_pub.publish(String(data=f"TRAVELING_TO_{board_id}"))
        self.is_moving = True
        
        # Create Goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(pos['x'])
        goal_msg.pose.pose.position.y = float(pos['y'])
        
        theta = float(pos['theta'])
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        self.get_logger().info(f"🚀 Sending Goal: Board {board_id} at ({pos['x']:.2f}, {pos['y']:.2f})")
        
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ Goal rejected!')
            self.is_moving = False
            # Retry logic could go here, or just let the loop pick another one
            return

        self.get_logger().info('✅ Goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        status = result.status
        self.is_moving = False
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'🏁 Arrived at Board {self.current_goal_board_id}. Starting Analysis...')
            self.is_analyzing = True
            self.status_pub.publish(String(data="SCANNING"))
            
            # Start a timer to mock "waiting for analysis"
            # In a real scenario, we might wait for a /patrol/status update or similar
            self.analysis_timer = self.create_timer(10.0, self.on_analysis_complete)
            
        else:
            self.get_logger().warn(f'⚠️ Goal Failed (Status: {status})')

    def on_analysis_complete(self):
        self.get_logger().info("✅ Analysis period ended. Resume patrol.")
        self.is_analyzing = False
        if hasattr(self, 'analysis_timer'):
            self.analysis_timer.cancel()
            self.analysis_timer.destroy()
            
def main(args=None):
    rclpy.init(args=args)
    node = SequentialPlannerAction()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
