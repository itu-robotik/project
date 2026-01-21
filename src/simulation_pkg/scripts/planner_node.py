#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import json
import os
import time
import math

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        
        # 1. State Machine & Readiness
        self.startup_state = "WAITING_FOR_LOCALIZATION" # WAITING_FOR_LOCALIZATION, STARTUP_WARMUP, READY
        self.warmup_start_time = None
        self.WARMUP_DURATION = 5.0 # Seconds to wait after ready
        self.amcl_pose_received = False
        
        # 2. TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. Memory Dosyasi Konumu
        self.memory_file = os.path.expanduser('~/itu_robotics_ws/itu_robotics_combined_ws/board_memory.json')
        self.memory = self.load_memory()
        
        # 4. Pano Koordinatlari (Gazebo'daki Konumlar)
        self.board_locations = {
            "1": {"x": -5.0, "y": -4.0, "theta": -1.57},
            "2": {"x": -3.5, "y": -4.0, "theta": -1.57},
            "3": {"x": -0.5, "y": -4.0, "theta": -1.57},
            "4": {"x":  1.0, "y": -4.0, "theta": -1.57},
            "5": {"x":  4.5, "y": -4.0, "theta": -1.57},
            "6": {"x":  6.0, "y": -4.0, "theta": -1.57}
        }
        
        # 5. Yayin ve Abonelikler
        self.goal_pub = self.create_publisher(PoseStamped, '/planner/goal', 10)
        
        self.analysis_sub = self.create_subscription(String, '/perception/poster_analysis', self.analysis_callback, 10)
        self.status_sub = self.create_subscription(String, '/patrol/status', self.patrol_status_callback, 10)
        
        # Readiness Subscriptions
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)
        self.costmap_received = False

        # 6. Durum Degiskenleri
        self.robot_state = "IDLE" # IDLE, MOVING, DOCKING, ANALYZING
        self.current_target_id = None
        self.plan_timer = self.create_timer(1.0, self.planning_loop) # Run loop faster (1Hz) to check readiness
        
        self.get_logger().info("🧠 Planner Node (STABILITY MODE) Başlatıldı!")
        self.get_logger().info(f"📂 Hafıza Dosyası: {self.memory_file}")

    def amcl_callback(self, msg):
        self.amcl_pose_received = True

    def costmap_callback(self, msg):
        self.costmap_received = True

    def check_startup_readiness(self):
        # 1. AMCL Pose Check
        if not self.amcl_pose_received:
            self.get_logger().info("⏳ WAITING_FOR_LOCALIZATION: Waiting for /amcl_pose...", throttle_duration_sec=2.0)
            return False

        # 2. Costmap Check
        if not self.costmap_received:
            self.get_logger().info("⏳ WAITING_FOR_COSTMAP: Waiting for /global_costmap/costmap...", throttle_duration_sec=2.0)
            return False
            
        # 3. TF Check (map -> base_link)
        try:
            self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().info(f"⏳ WAITING_FOR_TF: map -> base_link not ready yet ({e})", throttle_duration_sec=2.0)
            return False

        return True

    def load_memory(self):
        # Default Template
        default_memory = {
            "boards": {
                "1": {"id": 1, "x": -5.0, "y": -4.0, "theta": -1.57, "status": "unknown", "visit_count": 0, "last_visit": 0},
                "2": {"id": 2, "x": -3.5, "y": -4.0, "theta": -1.57, "status": "unknown", "visit_count": 0, "last_visit": 0},
                "3": {"id": 3, "x": -0.5, "y": -4.0, "theta": -1.57, "status": "unknown", "visit_count": 0, "last_visit": 0},
                "4": {"id": 4, "x":  1.0, "y": -4.0, "theta": -1.57, "status": "unknown", "visit_count": 0, "last_visit": 0},
                "5": {"id": 5, "x":  4.5, "y": -4.0, "theta": -1.57, "status": "unknown", "visit_count": 0, "last_visit": 0},
                "6": {"id": 6, "x":  6.0, "y": -4.0, "theta": -1.57, "status": "unknown", "visit_count": 0, "last_visit": 0}
            },
            "history": []
        }

        if os.path.exists(self.memory_file):
            try:
                with open(self.memory_file, 'r') as f:
                    data = json.load(f)
                    
                    # CASE 1: Data is a LIST (From Poster Logger)
                    if isinstance(data, list):
                        self.get_logger().info("📂 Loaded RAW MAP DATA from Logger. Converting to Patrol Memory...")
                        memory = {"boards": {}, "history": []}
                        for item in data:
                            bid = str(item['id'])
                            # Map Logger 'approach' to Planner targets
                            memory["boards"][bid] = {
                                "id": int(bid),
                                "x": item.get('approach_x', 0.0),
                                "y": item.get('approach_y', 0.0),
                                "theta": item.get('approach_theta', 0.0),
                                "status": "unknown",
                                "visit_count": 0, 
                                "last_visit": 0
                            }
                        
                        # Init self.board_locations from this data
                        self.board_locations = {}
                        for bid, b in memory["boards"].items():
                             self.board_locations[bid] = {"x": b["x"], "y": b["y"], "theta": b["theta"]}
                             
                        return memory

                    # CASE 2: Data is a DICT (Existing Patrol Memory)
                    if "boards" in data:
                        # Ensure board_locations are updated from memory if present
                        self.board_locations = {} 
                        # Use data from memory if available, otherwise defaults
                        for bid, info in data["boards"].items():
                            if "x" in info:
                                self.board_locations[bid] = {"x": info["x"], "y": info["y"], "theta": info.get("theta", 0.0)}
                        
                        # If memory lacks coords (legacy), fallback to hardcoded (handled below implicitly if empty)
                        if not self.board_locations: 
                             self.get_logger().warn("⚠️ Memory loaded but no coordinates found. Using defaults.")
                        
                        return data

            except Exception as e:
                self.get_logger().error(f"⚠️ Error loading memory: {e}")
        
        # Fallback
        self.get_logger().warn("⚠️ No valid memory found. Using HARDCODED defaults.")
        self.board_locations = {}
        for bid, b in default_memory["boards"].items():
            self.board_locations[bid] = {"x": b["x"], "y": b["y"], "theta": b["theta"]}
        return default_memory

    def save_memory(self):
        with open(self.memory_file, 'w') as f:
            json.dump(self.memory, f, indent=2)

    def analysis_callback(self, msg):
        self.get_logger().info(f"📨 Analiz Verisi Alındı: {msg.data[:50]}...")
        try:
            data = json.loads(msg.data)
            board_id = str(data.get("board_id"))
            
            if board_id not in self.memory["boards"] and self.current_target_id:
                self.get_logger().warn(f"⚠️ Algılanan ID ({board_id}) bilinmiyor, Hedef ID ({self.current_target_id}) kullanılıyor.")
                board_id = str(self.current_target_id)
            
            if board_id in self.memory["boards"]:
                board = self.memory["boards"][board_id]
                
                board["title"] = data.get("title", "Unknown")
                board["event_date"] = data.get("event_date", None)
                board["status"] = data.get("status", "unknown")
                board["is_expired"] = data.get("is_expired", False)
                board["last_visit"] = time.time()
                board["visit_count"] += 1
                
                entry = {
                    "timestamp": time.time(),
                    "board_id": board_id,
                    "analysis": data
                }
                self.memory["history"].append(entry)
                
                self.save_memory()
                self.get_logger().info(f"💾 Pano {board_id} Hafızası Güncellendi! Status: {board['status']}")
                
                self.robot_state = "IDLE"
                self.current_target_id = None 
                
        except json.JSONDecodeError:
            self.get_logger().error("❌ JSON Decode Hatasi!")

    def patrol_status_callback(self, msg):
        self.robot_state = msg.data

    def planning_loop(self):
        # --- STATE MACHINE START ---
        if self.startup_state == "WAITING_FOR_LOCALIZATION":
            if self.check_startup_readiness():
                self.get_logger().info("✅ Localization & TF Ready! Starting Warmup...")
                self.startup_state = "STARTUP_WARMUP"
                self.warmup_start_time = self.get_clock().now()
            else:
                return # Keep waiting

        elif self.startup_state == "STARTUP_WARMUP":
            elapsed = (self.get_clock().now() - self.warmup_start_time).nanoseconds / 1e9
            if elapsed >= self.WARMUP_DURATION:
                self.get_logger().info("🚀 NAVIGATION_READY: Warmup complete. Planner active.")
                self.startup_state = "READY"
            else:
                self.get_logger().info(f"🌡️ Warming up... {elapsed:.1f}/{self.WARMUP_DURATION}s", throttle_duration_sec=1.0)
                return # Keep warming up

        elif self.startup_state != "READY":
            return
        # --- STATE MACHINE END ---

        # Eger robot mesgulse emir verme
        if self.robot_state != "IDLE":
            return

        target_id = None
        min_dist = float('inf')
        
        current_x = 0.0
        current_y = 0.0
        if self.current_target_id and self.current_target_id in self.board_locations:
             last_loc = self.board_locations[self.current_target_id]
             current_x = last_loc["x"]
             current_y = last_loc["y"]
        
        now = time.time()
        
        unvisited_candidates = []
        revisit_candidates = []
        
        for bid, info in self.memory["boards"].items():
            if info["visit_count"] == 0:
                if (now - info.get("last_attempt", 0)) > 30: 
                     if bid in self.board_locations:
                        loc = self.board_locations[bid]
                        dist = math.sqrt((loc["x"] - current_x)**2 + (loc["y"] - current_y)**2)
                        unvisited_candidates.append((dist, bid))

            elif info["status"] in ["expired", "unclear", "unknown"]:
                if (now - info["last_visit"]) > 60:
                    if bid in self.board_locations:
                        loc = self.board_locations[bid]
                        dist = math.sqrt((loc["x"] - current_x)**2 + (loc["y"] - current_y)**2)
                        revisit_candidates.append((dist, bid))
        
        if unvisited_candidates:
            unvisited_candidates.sort(key=lambda x: x[0])
            target_id = unvisited_candidates[0][1]
            dist_to_target = unvisited_candidates[0][0]
            self.get_logger().info(f"📍 Yeni Hedef (Ziyaret Edilmemiş): Pano {target_id} (Mesafe: {dist_to_target:.2f}m)")
            
        elif revisit_candidates:
            revisit_candidates.sort(key=lambda x: x[0])
            target_id = revisit_candidates[0][1]
            dist_to_target = revisit_candidates[0][0]
            self.get_logger().info(f"📍 Yeni Hedef (Tekrar Kontrol): Pano {target_id} (Mesafe: {dist_to_target:.2f}m)")
        
        if target_id is None:
             oldest_time = float('inf')
             found_candidate = False
             
             for bid, info in self.memory["boards"].items():
                if (now - info.get("last_attempt", 0)) < 30:
                    continue
                
                if info["last_visit"] < oldest_time:
                    oldest_time = info["last_visit"]
                    target_id = bid
                    found_candidate = True
             
             if found_candidate:
                self.get_logger().info(f"🔄 Devriye: Her şey yolunda, en eski Pano {target_id} kontrol ediliyor.")

        if target_id is None:
             all_visited = True
             for bid, info in self.memory["boards"].items():
                 if info["visit_count"] == 0:
                     all_visited = False
                     break
             
             if all_visited:
                 self.get_logger().info("🎉 GÖREV TAMAMLANDI: Tüm panolar kontrol edildi. Devriye bitiyor.", throttle_duration_sec=30)
                 return

        if target_id:
            self.memory["boards"][target_id]["last_attempt"] = time.time()
            self.send_goal(target_id)
            self.current_target_id = target_id

    def send_goal(self, board_id):
        if board_id not in self.board_locations:
            self.get_logger().error(f"❌ Pano {board_id} koordinatları bilinmiyor!")
            return

        target = self.board_locations[board_id]
        
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.position.x = float(target["x"])
        msg.pose.position.y = float(target["y"])
        msg.pose.position.z = 0.0
        
        theta = float(target["theta"])
        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)
        
        self.goal_pub.publish(msg)
        self.current_target_id = board_id
        
        self.get_logger().info(f"🚀 HEDEF GÖNDERİLDİ: Pano {board_id} @ [{target['x']}, {target['y']}]")

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
