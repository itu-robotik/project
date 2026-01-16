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
        
        self.state = "FORWARD" # FORWARD, TURN_LEFT, TURN_RIGHT, RECOVER
        self.state_change_time = time.time()
        self.min_front_dist = 0.6  # Obstacle distance
        self.turn_duration = 0.0
        
        self.get_logger().info("🧭 Simple Explorer Started! Robot will map automatically.")

    def scan_callback(self, msg):
        # Ranges: [Right ... Front ... Left] (usually)
        # Assuming typical 360 or 180 scan. 
        # Let's verify index ranges for Front.
        # Often front is at 0 or mid, dependent on sensor. 
        # For standard ROS2/Gazebo usually 0 is front? Or index len/2?
        # Let's take a safe slice.
        
        # NOTE: rplidar A1/A2 usually 0 is front. Simulation often 0 is front.
        # Safeguard: Check min distance in a cone around 0 index.
        
        ranges = msg.ranges
        n = len(ranges)
        if n == 0: return

        # Check front cone (-30 to +30 deg approx) - SCAN IS CENTERED AT INDEX N/2
        # ranges goes from -PI to +PI. So 0 is Back, N/2 is Front.
        mid_index = int(n / 2)
        cone_width = int(n / 12) 
        
        # Front readings are in the middle of the array
        front_readings = ranges[mid_index - cone_width : mid_index + cone_width]
        
        # Valid readings (Trust everything > 0.05)
        valid_front = [r for r in front_readings if 0.05 < r < 10.0]
        
        if not valid_front:
             min_front = float('inf')
        else:
             min_front = min(valid_front)

        # Log status periodically
        # if self.state != "FORWARD": # Log only interesting events to avoid spam? No, keepalive is good.
        #    pass

        # Control Logic
        twist = Twist()
        current_time = time.time()
        
        if self.state == "FORWARD":
            if min_front < 0.25: # TOO CLOSE! Back up!
                self.state = "BACKUP"
                self.state_change_time = current_time
                self.get_logger().info(f"🚨 Too close ({min_front:.2f}m)! Backing up...")
                
            elif min_front < self.min_front_dist:
                self.state = random.choice(["TURN_LEFT", "TURN_RIGHT"])
                self.state_change_time = current_time
                self.turn_duration = random.uniform(1.0, 3.0)
                self.get_logger().info(f"🚧 Obstacle detected ({min_front:.2f}m). Turning...")
            else:
                twist.linear.x = 0.25
                
        elif self.state == "BACKUP":
            if (current_time - self.state_change_time) > 1.5: # Back up for 1.5s
                self.state = "TURN_LEFT" # Then turn
                self.state_change_time = current_time
                self.turn_duration = 2.0
            else:
                twist.linear.x = -0.15 # Reverse speed

        elif self.state in ["TURN_LEFT", "TURN_RIGHT"]:
            if (current_time - self.state_change_time) > self.turn_duration:
                self.state = "FORWARD"
                self.get_logger().info("✅ Path clear (hopefully). Moving Forward.")
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.5 if self.state == "TURN_LEFT" else -0.5
                
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Explorer Stopped.")
        # Stop robot
        stop_twist = Twist()
        node.cmd_pub.publish(stop_twist)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
