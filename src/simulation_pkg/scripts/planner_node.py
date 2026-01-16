#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped
import json
import os
import time
import math

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        
        # 1. Memory Dosyasi Konumu
        self.memory_file = os.path.expanduser('~/itu_robotics_ws/itu_robotics_combined_ws/board_memory.json')
        self.memory = self.load_memory()
        
        # 2. Pano Koordinatlari (Gazebo'daki Konumlar)
        # Posters are at y = -9.6, facing +Y (1.57).
        # Robot should be at y = -8.0, facing -Y (-1.57) to see them.
        self.board_locations = {
            "1": {"x": -10.0, "y": -8.0, "theta": -1.57},
            "2": {"x":  -7.0, "y": -8.0, "theta": -1.57},
            "3": {"x":  -1.0, "y": -8.0, "theta": -1.57},
            "4": {"x":   2.0, "y": -8.0, "theta": -1.57},
            "5": {"x":   9.0, "y": -8.0, "theta": -1.57},
            "6": {"x":  12.0, "y": -8.0, "theta": -1.57}
        }
        
        # 3. Yayin ve Abonelikler
        self.goal_pub = self.create_publisher(PoseStamped, '/planner/goal', 10)
        
        self.analysis_sub = self.create_subscription(String, '/perception/poster_analysis', self.analysis_callback, 10)
        self.status_sub = self.create_subscription(String, '/patrol/status', self.patrol_status_callback, 10)
        
        # 4. Durum Degiskenleri
        self.robot_state = "IDLE" # IDLE, MOVING, DOCKING, ANALYZING
        self.current_target_id = None
        self.plan_timer = self.create_timer(5.0, self.planning_loop)
        
        self.get_logger().info("🧠 Planner Node (MEMORY SYSTEM) Başlatıldı!")
        self.get_logger().info(f"📂 Hafıza Dosyası: {self.memory_file}")

    def load_memory(self):
        # Default Template
        default_memory = {
            "boards": {
                "1": {"id": 1, "x": -10.0, "y": -8.0, "theta": -1.57, "status": "unknown", "visit_count": 0, "last_visit": 0},
                "2": {"id": 2, "x":  -7.0, "y": -8.0, "theta": -1.57, "status": "unknown", "visit_count": 0, "last_visit": 0},
                "3": {"id": 3, "x":  -1.0, "y": -8.0, "theta": -1.57, "status": "unknown", "visit_count": 0, "last_visit": 0},
                "4": {"id": 4, "x":   2.0, "y": -8.0, "theta": -1.57, "status": "unknown", "visit_count": 0, "last_visit": 0},
                "5": {"id": 5, "x":   9.0, "y": -8.0, "theta": -1.57, "status": "unknown", "visit_count": 0, "last_visit": 0},
                "6": {"id": 6, "x":  12.0, "y": -8.0, "theta": -1.57, "status": "unknown", "visit_count": 0, "last_visit": 0}
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
                             # ... logic to use defaults ... 
                        
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
        # self.get_logger().info("💾 Hafıza Kaydedildi.")

    def analysis_callback(self, msg):
        self.get_logger().info(f"📨 Analiz Verisi Alındı: {msg.data[:50]}...")
        try:
            data = json.loads(msg.data)
            board_id = str(data.get("board_id"))
            
            # Eger algilanan ID hafizada yoksa (Orn: ArUco 0 ama Pano 1'e gittik)
            # Mevcut hedefi kabul et
            if board_id not in self.memory["boards"] and self.current_target_id:
                self.get_logger().warn(f"⚠️ Algılanan ID ({board_id}) bilinmiyor, Hedef ID ({self.current_target_id}) kullanılıyor.")
                board_id = str(self.current_target_id)
            
            if board_id in self.memory["boards"]:
                board = self.memory["boards"][board_id]
                
                # Bilgileri Guncelle
                board["title"] = data.get("title", "Unknown")
                board["event_date"] = data.get("event_date", None)
                board["status"] = data.get("status", "unknown")
                board["is_expired"] = data.get("is_expired", False)
                board["last_visit"] = time.time()
                board["visit_count"] += 1
                
                # History'ye ekle
                entry = {
                    "timestamp": time.time(),
                    "board_id": board_id,
                    "analysis": data
                }
                self.memory["history"].append(entry)
                
                self.save_memory()
                self.get_logger().info(f"💾 Pano {board_id} Hafızası Güncellendi! Status: {board['status']}")
                
                # Analiz bitti, robot bosa cikti sayabiliriz (Patrol node IDLE'a donecek)
                self.robot_state = "IDLE"
                self.current_target_id = None 
                
        except json.JSONDecodeError:
            self.get_logger().error("❌ JSON Decode Hatasi!")

    def patrol_status_callback(self, msg):
        # Patrol Node'dan gelen durum bilgisi (bunu patrol node'a ekleyecegiz)
        self.robot_state = msg.data

    def planning_loop(self):
        # Eger robot mesgulse emir verme
        if self.robot_state != "IDLE":
            return

        target_id = None
        min_dist = float('inf')
        
        # Robotun su anki tahmini konumu (Hic gitmediyse 0,0 kabul edelim veya ilk panoya yakin)
        # Daha once bir yere gittiysek ordayizdir
        current_x = 0.0
        current_y = 0.0
        if self.current_target_id and self.current_target_id in self.board_locations:
             last_loc = self.board_locations[self.current_target_id]
             current_x = last_loc["x"]
             current_y = last_loc["y"]
        
        now = time.time()
        
        # Gezilecek adaylari belirle
        unvisited_candidates = []
        revisit_candidates = []
        
        for bid, info in self.memory["boards"].items():
            # Ziyaret edilmemis veya status sorunlu olanlar
            # Ayrica sure kontrolu (60sn)
            
            if info["visit_count"] == 0:
                # Hic gitmemisiz. Ama yakin zamanda denedik mi?
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
        
        # ONCELIK: Ziyaret edilmemisler
        if unvisited_candidates:
            unvisited_candidates.sort(key=lambda x: x[0])
            target_id = unvisited_candidates[0][1]
            dist_to_target = unvisited_candidates[0][0]
            self.get_logger().info(f"📍 Yeni Hedef (Ziyaret Edilmemiş): Pano {target_id} (Mesafe: {dist_to_target:.2f}m)")
            
        # Eger hepsi ziyaret edildiyse, tekrar kontrol edilmesi gerekenlere bak
        elif revisit_candidates:
            revisit_candidates.sort(key=lambda x: x[0])
            target_id = revisit_candidates[0][1]
            dist_to_target = revisit_candidates[0][0]
            self.get_logger().info(f"📍 Yeni Hedef (Tekrar Kontrol): Pano {target_id} (Mesafe: {dist_to_target:.2f}m)")
        
        # Eger aday yoksa, belki hepsi 'ok' durumdadir. 
        # Yine de en eski ziyaret edilene bakalim (devriye niyetiyle)
        if target_id is None:
             oldest_time = float('inf')
             found_candidate = False
             
             for bid, info in self.memory["boards"].items():
                # Devriye sirasinda da yakin zamanda denediklerimizi atlayalim
                if (now - info.get("last_attempt", 0)) < 30:
                    continue
                
                if info["last_visit"] < oldest_time:
                    oldest_time = info["last_visit"]
                    target_id = bid
                    found_candidate = True
             
             if found_candidate:
                self.get_logger().info(f"🔄 Devriye: Her şey yolunda, en eski Pano {target_id} kontrol ediliyor.")

        # Eger hala aday yoksa ve tum panolar kontrol edildiyse
        # Veya tum panolarin statusu 'ok' ise ve sureleri dolmadiysa
        if target_id is None:
             # Kontrol edelim: Hepsi ziyaret edildi mi?
             all_visited = True
             for bid, info in self.memory["boards"].items():
                 if info["visit_count"] == 0:
                     all_visited = False
                     break
             
             if all_visited:
                 self.get_logger().info("🎉 GÖREV TAMAMLANDI: Tüm panolar kontrol edildi. Devriye bitiyor.")
                 # IDLE durumuna gec ve hedef gonderme
                 return

        if target_id:
            # Hedef gondermeden once last_attempt guncelle
            self.memory["boards"][target_id]["last_attempt"] = time.time()
            self.send_goal(target_id)
            # Hedefi set et ki bir sonraki sefer buradan hesaplayalim
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
        
        # Theta -> Quaternion
        theta = float(target["theta"])
        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)
        
        self.goal_pub.publish(msg)
        self.current_target_id = board_id
        # self.robot_state = "MOVING" # BUG FIX: Patrol Node zaten status update gonderecek.
        # Biz burada direkt MOVING yaparsak ve Patrol hemen yanit vermezse senkron kopabilir.
        # Ama asil sorun patrol node analiz bittikten sonra "IDLE" donmuyorsa burasi kilitli kalir.
        
        self.get_logger().info(f"🚀 HEDEF GÖNDERİLDİ: Pano {board_id} @ [{target['x']}, {target['y']}]")

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
