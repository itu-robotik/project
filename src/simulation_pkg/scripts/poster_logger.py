#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import json
import os
import math
import numpy as np

class PosterLogger(Node):
    def __init__(self):
        super().__init__('poster_logger')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to perception (ArUco/Board detection)
        # Msg format: [found, cx, distance, yaw_err, marker_yaw, cx_offset, width]
        self.vision_sub = self.create_subscription(
            Float32MultiArray, 
            '/perception/board_status', 
            self.vision_callback, 
            10
        )
        
        self.memory_file = os.path.expanduser('~/itu_robotics_ws/itu_robotics_combined_ws/board_memory.json')
        self.posters = []
        self.load_memory()
        
        self.robot_pose = None
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.get_logger().info(f"📋 Poster Logger Started. Saving to: {self.memory_file}")

    def load_memory(self):
        if os.path.exists(self.memory_file):
            try:
                with open(self.memory_file, 'r') as f:
                    self.posters = json.load(f)
                self.get_logger().info(f"Loaded {len(self.posters)} existing posters.")
            except:
                self.posters = []
        else:
            self.posters = []

    def save_memory(self):
        with open(self.memory_file, 'w') as f:
            json.dump(self.posters, f, indent=4)
        self.get_logger().info(f"💾 Saved {len(self.posters)} posters to memory.")

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def vision_callback(self, msg):
        if len(msg.data) < 7: return
        
        found = (msg.data[0] > 0.5)
        dist = msg.data[2]
        yaw_err = msg.data[3]
        
        if found and dist < 6.0: # Log even if further away
            self.register_poster(dist, yaw_err)

    def register_poster(self, dist, yaw_err):
        # We need to transform the relative detection to the Map Frame
        try:
            # Create a "detection" pose in the robot base frame (or camera frame)
            # Assuming camera is forward facing x-axis roughly
            # X = dist * cos(angle), Y = dist * sin(angle)
            
            # Note: perception_node calculates yaw_err relative to robot center? 
            # Let's assume yaw_err = angle to object.
            
            # Simple approach: Get Robot Pose in Map, project point
            if self.robot_pose is None: return
            
            # Wait for transform (Map -> Base Link) might be better than trusting Odom directly if SLAM is drifting,
            # but usually Odom frame is smooth, Map frame jumps.
            # Ideally we want Map coordinates.
            
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
            # Robot Position in Map
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y
            
            # Robot Quaternion to Yaw
            q = trans.transform.rotation
            # ... simple yaw calculation ...
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            robot_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # Poster Position
            # Global Angle = Robot Yaw - Yaw Err (sign depends on implementation, usually + is left)
            # visual_callback in patrol says: global = robot_yaw - yaw_err
            poster_angle = robot_yaw - yaw_err
            
            px = rx + (dist * math.cos(poster_angle))
            py = ry + (dist * math.sin(poster_angle))
            
            # Check for duplicates
            is_new = True
            for p in self.posters:
                dx = p['x'] - px
                dy = p['y'] - py
                if math.sqrt(dx*dx + dy*dy) < 1.0: # 1 meter tolerance
                    is_new = False
                    break
            
            if is_new:
                new_id = len(self.posters) + 1
                entry = {
                    'id': new_id,
                    'poster_x': px,
                    'poster_y': py,
                    'approach_x': rx,
                    'approach_y': ry,
                    'approach_theta': robot_yaw,
                    'detected_at': Time.now().nanoseconds
                }
                self.posters.append(entry)
                self.get_logger().info(f"✨ NEW POSTER DETECTED! ID: {new_id} at [{px:.1f}, {py:.1f}] Approach: [{rx:.1f}, {ry:.1f}]")
                self.save_memory()
                
        except Exception as e:
            # self.get_logger().warn(f"TF Error: {e}")
            pass
            
# Helper for time
from rclpy.time import Time

def main(args=None):
    rclpy.init(args=args)
    node = PosterLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
