#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
import math
import time

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        
        # Subscribers / Publishers
        self.vision_sub = self.create_subscription(Float32MultiArray, '/perception/board_status', self.vision_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/patrol/status', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/planner/goal', self.goal_callback, 10)
        self.analysis_sub = self.create_subscription(String, '/perception/poster_analysis', self.analysis_done_callback, 10)
        
        # Clients
        self.client_analysis = self.create_client(Trigger, 'analyze_poster')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # State
        self.state = "IDLE"  # IDLE, TRAVELING (Nav2), SCANNING, NAVIGATE_TO_DOCK (Visual), ANALYZING
        self.board_found = False
        self.target_locked = False
        self.latest_visual_dist = 99.0
        self.min_front_dist = 99.0 # Lidar safety
        
        self.visual_cx_offset = 0.0
        self.visual_poster_width = 0.0
        
        # Last known visual info
        self.last_analysis_pos_x = -999.0 # Start far away
        self.last_analysis_pos_y = -999.0
        
        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("🤖 Patrol Node (Nav2 Integrated) Ready!")

    # ... (rest of methods until scan_callback)
    
    def scan_callback(self, msg):
        # Find minimum distance in front sector (-20 to +20 degrees)
        # LaserScan usually starts from angle_min (often -pi) to angle_max (+pi)
        # We need indices corresponding to front.
        # Assuming typical 360 Lidar where index 0 is front? Or index 0 is -pi?
        # Standard convention: 0 is front (if angle_min approx -pi, then center index is front)
        # Wait, for standard 2D simplified Lidar:
        # Check angle_min. 
        # For simplicity, let's take the CENTER of the array (if ranges size ~ 360, center is 180).
        # OR: usually index 0 is front for some, and index len/2 is front for others.
        # SAFE APPROACH: check msg.angle_min.
        # If min is -3.14, then FRONT is at index len(ranges)/2.
        
        ranges = msg.ranges
        num_readings = len(ranges)
        mid_index = num_readings // 2
        window = num_readings // 10 # +/- 18 degrees approx
        
        # Extract front sector
        front_ranges = ranges[mid_index - window : mid_index + window]
        # Filter valid readings
        valid_ranges = [r for r in front_ranges if msg.range_min < r < msg.range_max]
        
        if valid_ranges:
            self.min_front_dist = min(valid_ranges)
        else:
            self.min_front_dist = 99.0

    def publish_status(self):
        self.get_logger().info(f"📤 Publishing Status: {self.state}") # Debug
        self.status_pub.publish(String(data=self.state))

    def analysis_done_callback(self, msg):
        self.get_logger().info("✅ Analysis Complete. Resetting to IDLE.")
        self.state = "IDLE"
        self.target_locked = False

    def goal_callback(self, msg):
        self.get_logger().info(f"📨 Received Goal: [{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}]")
        self.target_locked = False
        self.send_nav2_goal(msg)

    def send_nav2_goal(self, pose_msg):
        self.state = "TRAVELING"
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg
        
        self.get_logger().info("🚀 Sending goal to Nav2...")
        
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("❌ Nav2 Action Server not active! Goal ignored.")
            self.state = "IDLE"
            return
        
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected by Nav2!")
            self.state = "IDLE"
            return

        self.get_logger().info("✅ Goal accepted by Nav2.")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == 4: # SUCCEEDED
            self.get_logger().info("🏁 Nav2 Reached Goal!")
            
            # Check if we already see the board?
            if self.board_found and self.latest_visual_dist < 4.0:
                 self.get_logger().info("📍 Board already in view! Skipping SCANNING.")
                 self.state = "NAVIGATE_TO_DOCK"
                 self.target_locked = True
            else:
                 self.get_logger().info("🔍 Switching to SCANNING.")
                 self.state = "SCANNING"
                 self.scan_start_time = time.time()
        else:
             self.get_logger().warn(f"⚠️ Nav2 Goal Failed/Canceled status: {status}")
             self.state = "FAILED"
             self._fail_timer = self.create_timer(3.0, self.reset_to_idle)
             
    def reset_to_idle(self):
        self.state = "IDLE"
        if hasattr(self, '_fail_timer'):
            self._fail_timer.cancel()
            self._fail_timer.destroy()

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def vision_callback(self, msg):
        # Update vision state continuously (Passive tracking)
        if len(msg.data) >= 7:
            found = (msg.data[0] > 0.5)
            if found:
                self.board_found = True
                self.latest_visual_dist = msg.data[2]
                self.visual_cx_offset = msg.data[5]
                self.visual_poster_width = msg.data[6]
                
                # Active Logic: TRANSITION SCANNING -> DOCKING
                if self.state == "SCANNING" and self.latest_visual_dist < 4.0:
                    self.get_logger().info("📍 Visual Target Acquired! Locking on...")
                    self.target_locked = True
                    self.state = "NAVIGATE_TO_DOCK"
            else:
                self.board_found = False
    def control_loop(self):
        # 1. TRAVELING is handled by Nav2 Action Server (Background)
        # 2. SCANNING Loop
        if self.state == "SCANNING":
            if self.target_locked:
                self.state = "NAVIGATE_TO_DOCK"
                return
            
            # Simple spin to find board
            twist = Twist()
            twist.angular.z = 0.5
            self.cmd_pub.publish(twist)
            
            # Timeout
            if (time.time() - self.scan_start_time) > 15.0:
                self.get_logger().warn("⏰ Scan timed out. Boards not found.")
                self.stop_robot()
                self.state = "IDLE"

        # 3. DOCKING Loop (Visual Servoing)
        elif self.state == "NAVIGATE_TO_DOCK":
            # Safety Check first!
            if self.min_front_dist < 0.70: # 70cm safety stop
                self.get_logger().warn(f"🛑 Too close to obstacle ({self.min_front_dist:.2f}m)! Stopping.")
                self.stop_robot()
                self.state = "ANALYZING" # Assume we are close enough to read
                time.sleep(1.0)
                self.client_analysis.call_async(Trigger.Request())
                return

            # Visual Servoing Logic
            twist = Twist()
            
            # Angular P-Controller
            err_ang = self.visual_cx_offset
            ang_z = -0.8 * err_ang
            ang_z = max(min(ang_z, 0.4), -0.4)
            if abs(err_ang) < 0.05: ang_z = 0.0
            
            # Linear P-Controller (Based on Width)
            # Reduce target width to stop further away (e.g., 300 instead of 380)
            target_width = 300.0
            current_width = self.visual_poster_width
            
            if current_width > 10:
                err_width = target_width - current_width
                lin_x = 0.0015 * err_width
                
                # If very large/close, back up
                if current_width > 380: 
                    lin_x = -0.1
            else:
                # Fallback distance if width not reliable
                lin_x = 0.2 * (self.latest_visual_dist - 1.5)
                
            lin_x = max(min(lin_x, 0.2), -0.1)
            
            twist.linear.x = float(lin_x)
            twist.angular.z = float(ang_z)
            self.cmd_pub.publish(twist)
            
            # Success Condition: Width ~300 +/- tolerance
            is_aligned_visual = (current_width > 280 and abs(lin_x) < 0.02 and abs(err_ang) < 0.05)
            
            if is_aligned_visual:
                self.get_logger().info("🎯 Visual Docking Successful. Starting Analysis...")
                self.stop_robot()
                self.state = "ANALYZING"
                time.sleep(1.0)
                self.client_analysis.call_async(Trigger.Request()) # Call Gemini

    def destroy_node(self):
        # Send cancel to nav2 if active
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
