#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraMonitor(Node):
    def __init__(self):
        super().__init__('camera_monitor')
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.listener_callback,
            10)
        self.br = CvBridge()
        self.get_logger().info('Camera Monitor Started. Waiting for images...')

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            current_frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Display image
            cv2.imshow("Robot Camera", current_frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    camera_monitor = CameraMonitor()
    rclpy.spin(camera_monitor)
    camera_monitor.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
