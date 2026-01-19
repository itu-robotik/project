#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json
import os
import math
import time

class SimplePlanner(Node):
    def __init__(self):
        super().__init__('sequential_planner_action')
        
        # Paths
        self.workspace_root = os.path.expanduser('~/itu_robotics_ws/itu_robotics_combined_ws')
        self.memory_path = os.path.join(self.workspace_root, 'board_memory.json')
        self.positions_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'board_positions.json')
        
        # Data
        self.board_positions = self.load_board_positions()
        self.board_ids = sorted(list(self.board_positions.keys()))
        self.current_idx = 0
        
        # State
        self.robot_state = "UNKNOWN"
        self.last_command_time = 0.0
        self.waiting_for_result = False
        self.ack_received = False
        
        # ROS
        self.goal_pub = self.create_publisher(PoseStamped, '/planner/goal', 10)
        self.status_sub = self.create_subscription(String, '/patrol/status', self.status_callback, 10)
        self.timer = self.create_timer(1.0, self.planning_loop)
        
        self.get_logger().info("🧠 Simple Planner Ready! Boards: " + str(self.board_ids))

    def load_board_positions(self):
        # Load static first
        positions = {}
        if os.path.exists(self.positions_path):
             with open(self.positions_path, 'r') as f:
                 data = json.load(f)
                 positions = {int(k): v for k,v in data.items()}
        
        # Overlay memory if exists
        if os.path.exists(self.memory_path):
            try:
                with open(self.memory_path, 'r') as f:
                    mem_data = json.load(f)
                    for k, v in mem_data.items():
                        bid = int(k)
                        if bid in positions and v.get('x') != 0:
                            positions[bid]['x'] = v['x']
                            positions[bid]['y'] = v['y']
                            positions[bid]['theta'] = v.get('theta', positions[bid]['theta'])
                            self.get_logger().info(f"Updated Board {bid} from memory.")
            except Exception as e:
                self.get_logger().warn(f"Memory read error: {e}")
                
        return positions

    def status_callback(self, msg):
        previous_state = self.robot_state
        self.robot_state = msg.data
        
        # Log state changes for debugging
        if previous_state != self.robot_state:
            self.get_logger().info(f"🤖 Robot State Changed: {previous_state} -> {self.robot_state}")

        if self.waiting_for_result:
            # PHASE 1: Wait for ACK (Transition to TRAVELING)
            if not self.ack_received:
                if self.robot_state == "TRAVELING":
                    self.ack_received = True
                    self.get_logger().info("✅ Command Acknowledged (Robot is TRAVELING)")
                elif self.robot_state == "FAILED" and previous_state == "IDLE": 
                    # Special Case: Immediate Failure transition (skipped TRAVELING?)
                    # Or if it fails instantly. But we just sent it.
                    # Best to wait for TRAVELING, but if it goes IDLE->FAILED, we should accept that.
                    self.ack_received = True 
                else:
                    # Still IDLE or FAILED from *before*? Ignore.
                    pass
            
            # PHASE 2: Wait for Completion (Once ACK received)
            else:
                if self.robot_state in ["IDLE", "FAILED"]:
                    self.waiting_for_result = False
                    self.ack_received = False
                    self.get_logger().info(f"Task finished with result: {self.robot_state}")

    def planning_loop(self):
        # If robot is busy or we are waiting for result, do nothing
        if self.robot_state not in ["IDLE", "FAILED"] and not self.waiting_for_result:
             # Just wait if it is doing something else (e.g. manual control)
             return
            
        if self.waiting_for_result:
            return

        # Cooldown to prevent spam execution
        if (time.time() - self.last_command_time) < 5.0:
            return

        # Pick target
        bid = self.board_ids[self.current_idx]
        self.send_goal(bid)
        
        # Advance index (Round Robin)
        self.current_idx = (self.current_idx + 1) % len(self.board_ids)
        
        self.waiting_for_result = True
        self.ack_received = False # New command, reset ACK
        self.last_command_time = time.time()

    def send_goal(self, board_id):
        # Reload memory just in case positions updated?
        # self.board_positions = self.load_board_positions() # Uncomment to enable dynamic updates during run
        
        pos = self.board_positions.get(board_id)
        if not pos:
             self.get_logger().error(f"Invalid board {board_id}")
             return

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(pos['x'])
        msg.pose.position.y = float(pos['y'])
        
        # Orientation
        theta = float(pos['theta'])
        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)
        
        self.get_logger().info(f"🚀 Sending Robot to Board {board_id} at ({pos['x']:.2f}, {pos['y']:.2f})")
        self.goal_pub.publish(msg)

def main():
    rclpy.init()
    node = SimplePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
