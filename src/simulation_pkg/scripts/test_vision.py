#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
import math

class VisionTester(Node):
    def __init__(self):
        super().__init__('vision_tester')
        
        self.vision_sub = self.create_subscription(
            Float32MultiArray,
            '/perception/board_status',
            self.vision_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.min_front_dist = 99.9
        self.latest_visual_data = None
        self.get_logger().info("👀 Vision Tester Started. Waiting for data...")

    def scan_callback(self, msg):   
        # Extract front sector (similar to patrol_node)
        num_ranges = len(msg.ranges)
        center_idx = num_ranges // 2
        # Define a narrow front cone (+/- 10 degrees)
        fov_indices = int(10 * (num_ranges / 270.0))  # approximate for 270 deg lidar
        
        front_ranges = msg.ranges[center_idx - fov_indices : center_idx + fov_indices]
        valid_ranges = [r for r in front_ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            self.min_front_dist = min(valid_ranges)
        else:
            self.min_front_dist = 99.9

    def vision_callback(self, msg):
        # Data: [detected (0/1), center_x, center_y, width, height]
        data = msg.data
        detected = bool(data[0])
        
        status_str = "❌ No Board Detected"
        
        if detected:
            cx = data[1]
            cy = data[2]
            w = data[3]
            h = data[4]
            
            # Simple visual offset calculation
            frame_center_x = 640 / 2
            offset_x = cx - frame_center_x
            
            status_str = (
                f"✅ Board SEEN!\n"
                f"   - Position: ({cx:.1f}, {cy:.1f})\n"
                f"   - Size: {w:.1f} x {h:.1f}\n"
                f"   - Center Offset: {offset_x:.1f} pixels\n"
            )
            
            # Advice
            if abs(offset_x) < 20:
                status_str += "   -> ALIGNED (Roughly Centered)"
            elif offset_x > 0:
                status_str += "   -> Robot needs to turn RIGHT"
            else:
                status_str += "   -> Robot needs to turn LEFT"
                
        else:
            status_str += " (Try spinning the robot or checking line of sight)"

        status_str += f"\n   - Lidar Front Dist: {self.min_front_dist:.2f} m"
        
        # Log throttle to avoid flooding
        self.get_logger().info(f"\n{'-'*30}\n{status_str}\n{'-'*30}", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = VisionTester()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
