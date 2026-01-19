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
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/patrol/status', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/planner/goal', self.goal_callback, 10)
        self.analysis_sub = self.create_subscription(String, '/perception/poster_analysis', self.analysis_done_callback, 10)
        
        # Clients
        self.client_analysis = self.create_client(Trigger, 'analyze_poster')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # State
        self.state = "IDLE"  # IDLE, TRAVELING, SCANNING, DOCKING, ANALYZING, FAILED
        self.board_detected = False
        self.visual_dist = 99.0
        self.visual_offset = 0.0
        
        self.scan_start_time = 0.0
        self.visual_lost_start_time = None
        
        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("🤖 Simple Patrol Node Ready!")

    def publish_status(self):
        self.status_pub.publish(String(data=self.state))

    def analysis_done_callback(self, msg):
        self.get_logger().info("✅ Analysis Complete. Back to IDLE.")
        self.state = "IDLE"

    def goal_callback(self, msg):
        self.get_logger().info(f"📨 Received Goal: [{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}]")
        self.send_nav2_goal(msg)

    def send_nav2_goal(self, pose_msg):
        self.state = "TRAVELING"
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg
        
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("❌ Nav2 unreachable")
            self.state = "FAILED"
            return
        
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected by Nav2")
            self.state = "FAILED"
            return

        self.get_logger().info("✅ Goal accepted")
        self.nav2_goal_handle = goal_handle # Store handle for cancellation
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # If we are already DOCKING (via visual interrupt), ignore Nav2 result (likely CANCELED)
        if self.state == "DOCKING":
            self.get_logger().info("🛑 Nav2 Result Received but ignored (DOCKING active)")
            return

        status = future.result().status
        if status == 4: # SUCCEEDED
            self.get_logger().info("🏁 Reached Waypoint via Nav2")
            
            # DIRECT TRANSITION - NO SCANNING LOOP
            if self.board_detected:
                self.get_logger().info(f"👀 Board visible at {self.visual_dist:.2f}m. Starting DOCKING.")
                self.state = "DOCKING"
            else:
                self.get_logger().warn("⚠️ Board NOT visible at goal. Attempting ANALYSIS anyway (User Request).")
                self.state = "ANALYZING"
                self.client_analysis.call_async(Trigger.Request())
        else:
            self.get_logger().warn(f"⚠️ Nav2 Failed: {status}")
            self.state = "FAILED"

    def vision_callback(self, msg):
        # [found, cx, distance, yaw_err, marker_yaw, poster_cx_offset, poster_width_px]
        if len(msg.data) >= 7:
            found = (msg.data[0] > 0.5)
            
            if found:
                self.board_detected = True
                self.visual_dist = msg.data[2] # ArUco PnP Distance
                self.visual_offset = msg.data[5] # Center Offset (-1 to 1)

                # CRITICAL: Interrupt Nav2 if we see the board clearly!
                if self.state == "TRAVELING" and self.visual_dist < 4.0:
                    self.get_logger().info(f"🚨 VISUAL INTERRUPT: Board seen at {self.visual_dist:.2f}m! Cancelling Nav2.")
                    self.cancel_nav2_goal()
                    self.stop_robot()
                    self.state = "DOCKING"
            else:
                self.board_detected = False

    def cancel_nav2_goal(self):
        if hasattr(self, 'nav2_goal_handle'):
            self.get_logger().info("🛑 Cancelling Nav2 Goal...")
            self.nav2_goal_handle.cancel_goal_async()

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        # SCANNING STATE REMOVED - User requested direct action
        
        if self.state == "DOCKING":
            if not self.board_detected:
                # RECOVERY LOGIC
                if self.visual_lost_start_time is None:
                    self.visual_lost_start_time = time.time()
                
                lost_duration = time.time() - self.visual_lost_start_time
                
                if lost_duration < 2.0:
                    # STOP and WAIT instead of backing up blindly
                    self.get_logger().warn(f"⚠️ Visual Lost! Waiting... ({lost_duration:.1f}s)")
                    self.stop_robot()
                    return
                else:
                    # Timeout - Assume we are close enough or detection failed
                    self.stop_robot()
                    self.get_logger().warn("❌ Visual Recovery failed. Using last known position for ANALYSIS.")
                    self.state = "ANALYZING"
                    self.client_analysis.call_async(Trigger.Request())
                    return
            else:
                # Board Sees! Reset recovery timer
                self.visual_lost_start_time = None

            # Visual Servoing
            # 1. Orientation (Keep offset near 0)
            err_ang = self.visual_offset # -1 (left) to 1 (right)
            # INCREASE GAIN for orientation
            k_ang = 1.5 
            ang_z = -k_ang * err_ang
            
            # 2. Distance (Keep distance at TARGET_DIST)
            TARGET_DIST = 1.2 # Meters
            current_dist = self.visual_dist
            err_dist = current_dist - TARGET_DIST
            k_lin = 0.4
            lin_x = k_lin * err_dist
            
            # STRICT Alignment Priority: If angle is bad, DO NOT move forward
            if abs(err_ang) > 0.2:
                lin_x = 0.0 # Stop linear movement, just turn
            
            # Limits
            lin_x = max(min(lin_x, 0.2), -0.05)
            ang_z = max(min(ang_z, 0.5), -0.5)
            
            # Check success
            if abs(err_dist) < 0.15 and abs(err_ang) < 0.08:
                self.get_logger().info("🎯 Docking Complete. Analyzing...")
                self.stop_robot()
                self.state = "ANALYZING"
                self.client_analysis.call_async(Trigger.Request())
                return
            
            twist = Twist()
            twist.linear.x = float(lin_x)
            twist.angular.z = float(ang_z)
            self.cmd_pub.publish(twist)

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
