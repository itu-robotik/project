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
from rclpy.time import Time

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
        
        # Save to board_positions.json (Source of Truth for Locations)
        # We use the src script path as requested by the user flow
        self.memory_file = os.path.expanduser('~/itu_robotics_ws/itu_robotics_combined_ws/src/simulation_pkg/scripts/board_positions.json')
        self.posters = {}
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
                self.posters = {}
        else:
            self.posters = {}

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
        
        if found and dist < 4.0: # Only register if reasonably close
            self.register_poster(dist, yaw_err)

    def register_poster(self, dist, yaw_err):
        try:
            if self.robot_pose is None: return
            
            # Get Robot Position in Map
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y
            
            # Robot Yaw
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            robot_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # Poster Actual Position (Approximate)
            # Global Angle = Robot Yaw - Yaw Err
            poster_angle = robot_yaw - yaw_err
            px = rx + (dist * math.cos(poster_angle))
            py = ry + (dist * math.sin(poster_angle))
            
            # Check for duplicates
            # Use poster actual position to check for duplicates
            is_new = True
            for pid, p in self.posters.items():
                # Check distance to stored approach point? Or stored poster point if available?
                # The existing file only has x,y (approach).
                # But if we want to detect duplicates, checking approach point is "okay" but riskier.
                # Ideally check poster location.
                
                # Let's check distance to existing approach points.
                # If we are close to an existing approach point (within 2m), assume it's the same board.
                stored_x = p.get('x', 0)
                stored_y = p.get('y', 0)
                
                dx = stored_x - rx
                dy = stored_y - ry
                
                # Also check angle?
                # If we are 2m away from a known stop point, maybe it's the same.
                if math.sqrt(dx*dx + dy*dy) < 1.5: 
                    is_new = False
                    break
            
            if is_new:
                # Generate new ID
                # Find max ID
                max_id = 0
                for k in self.posters.keys():
                    try:
                        iid = int(k)
                        if iid > max_id: max_id = iid
                    except: pass
                
                new_id = str(max_id + 1)
                
                # SAVE APPROACH COORDINATES for Navigation
                # We save where the robot IS right now, because from here we can see the board.
                entry = {
                    "x": rx,
                    "y": ry,
                    "theta": robot_yaw,
                    "description": f"Auto-detected Board {new_id}",
                    "poster_x": px, # Extra info for debugging
                    "poster_y": py
                }
                
                self.posters[new_id] = entry
                self.get_logger().info(f"✨ NEW BOARD DISCOVERED! ID: {new_id} -> Approach: [{rx:.1f}, {ry:.1f}]")
                self.save_memory()
                
        except Exception as e:
            # self.get_logger().warn(f"TF Error: {e}")
            pass

def main(args=None):
    rclpy.init(args=args)
    node = PosterLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
