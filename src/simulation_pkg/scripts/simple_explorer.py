#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random
import time

class SimpleExplorer(Node):
    def __init__(self):
        super().__init__('simple_explorer')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # State
        self.state = "FORWARD" 
        self.state_change_time = time.time()
        
        # Parameters
        self.min_front_dist = 0.6  # Obstacle avoidance threshold
        self.emergency_dist = 0.35 # Backup threshold
        self.max_speed = 0.25
        self.turn_speed = 0.6
        
        self.turn_target_duration = 0.0
        self.turn_direction = 1 # 1 for left, -1 for right
        
        self.scan_data = {
            'left': float('inf'),
            'front': float('inf'),
            'right': float('inf')
        }

        self.get_logger().info("🧭 Smart Explorer Started (v2)! Steering towards open spaces.")

    def scan_callback(self, msg):
        ranges = msg.ranges
        n = len(ranges)
        if n == 0: return

        # Assumption based on typical Gazebo Lidar:
        # Array spans -PI to +PI.
        # Index 0: Back
        # Index N/4: Right
        # Index N/2: Front
        # Index 3N/4: Left
        
        mid_i = n // 2
        q1_i = n // 4        # Right
        q3_i = (n * 3) // 4  # Left
        
        # Cone width (indices)
        width = n // 10 
        
        def get_min_dist(center_idx):
            start = max(0, center_idx - width)
            end = min(n, center_idx + width)
            chunk = ranges[start:end]
            # Filter invalid readings
            valid = [r for r in chunk if 0.05 < r < 10.0]
            if not valid: return 10.0 # Assume clear if no valid hits
            return min(valid)

        self.scan_data['front'] = get_min_dist(mid_i)
        self.scan_data['right'] = get_min_dist(q1_i) 
        self.scan_data['left'] =  get_min_dist(q3_i)

        self.control_loop()

    def control_loop(self):
        current_time = time.time()
        twist = Twist()
        
        front = self.scan_data['front']
        left = self.scan_data['left']
        right = self.scan_data['right']
        
        # LOGIC
        if self.state == "FORWARD":
            if front < self.emergency_dist:
                # Emergency
                self.state = "BACKUP"
                self.state_change_time = current_time
                self.get_logger().info(f"🚨 Panic! Wall too close ({front:.2f}m). Backing up.")
                
            elif front < self.min_front_dist:
                # Obstacle ahead. Switch to TURN state.
                self.state = "TURN"
                self.state_change_time = current_time
                
                # Heuristic: Turn towards the side with more space
                if left > right:
                    self.turn_direction = 1 # Left
                    self.get_logger().info(f"🚧 Blocked ({front:.2f}m). Turning LEFT (Space L:{left:.1f} > R:{right:.1f})")
                else:
                    self.turn_direction = -1 # Right
                    self.get_logger().info(f"🚧 Blocked ({front:.2f}m). Turning RIGHT (Space R:{right:.1f} > L:{left:.1f})")
                
                # Randomize duration slightly to prevent loops
                self.turn_target_duration = random.uniform(1.5, 3.0) 
            else:
                # Path Clear: Drive
                twist.linear.x = self.max_speed
                
                # Dynamic Steering: Bias slightly towards the more open side to center in halls
                # But don't oscillate too much.
                bias_threshold = 1.0 # Only steer if difference is significant
                if left > (right + bias_threshold): 
                     twist.angular.z = 0.15 # Bias Left
                elif right > (left + bias_threshold):
                     twist.angular.z = -0.15 # Bias Right
                else:
                     twist.angular.z = 0.0

        elif self.state == "TURN":
            if (current_time - self.state_change_time) > self.turn_target_duration:
                self.state = "FORWARD"
                # Check if path is actually clear? No, just switch state and let next loop decide.
            else:
                twist.angular.z = self.turn_speed * self.turn_direction
                twist.linear.x = 0.0
                
        elif self.state == "BACKUP":
            if (current_time - self.state_change_time) > 2.0:
                 self.state = "TURN" 
                 self.state_change_time = current_time
                 self.turn_direction = random.choice([-1, 1])
                 self.turn_target_duration = 2.0
            else:
                twist.linear.x = -0.15
        
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Explorer Stopped.")
        stop_twist = Twist()
        node.cmd_pub.publish(stop_twist)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
