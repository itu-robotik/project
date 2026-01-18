"""
Memory Manager - Board State Persistence
Emre Saraç'ın sorumluluğu: Tüm pano durumlarını kaydetme ve yönetme
"""

import json
import os
from datetime import datetime, timedelta
from typing import Dict, List, Optional


class MemoryManager:
    """Pano durumlarını kalıcı olarak saklayan ve yöneten sınıf"""
    
    def __init__(self, memory_file: str = "board_memory.json"):
        self.memory_file = memory_file
        self.boards: Dict[int, Dict] = {}
        self.load_memory()
    
    def load_memory(self):
        """Kayıtlı hafızayı dosyadan yükle"""
        if os.path.exists(self.memory_file):
            try:
                with open(self.memory_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    if "boards" in data:
                        self.boards = {int(k): v for k, v in data["boards"].items()}
                    else:
                        self.boards = {int(k): v for k, v in data.items()}
                print(f"✅ Hafıza yüklendi: {len(self.boards)} pano kaydı bulundu")
            except Exception as e:
                print(f"⚠️  Hafıza yüklenirken hata: {e}")
                self.boards = {}
        else:
            print("📝 Yeni hafıza dosyası oluşturuluyor...")
            self.boards = {}
    
    def save_memory(self):
        """Hafızayı dosyaya kaydet"""
        try:
            with open(self.memory_file, 'w', encoding='utf-8') as f:
                json.dump(self.boards, f, indent=2, ensure_ascii=False)
            return True
        except Exception as e:
            print(f"❌ Hafıza kaydedilirken hata: {e}")
            return False
    
    def update_board(self, board_data: Dict):
        """
        Pano bilgilerini güncelle veya yeni kayıt oluştur
        
        Args:
            board_data: /poster_analysis topic'inden gelen JSON verisi
        """
        board_id = board_data.get('board_id')
        if board_id is None:
            print("⚠️  board_id bulunamadı, kayıt yapılamadı")
            return False
        
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # Mevcut kayıt varsa güncelle, yoksa yeni oluştur
        if board_id in self.boards:
            # Mevcut kaydı güncelle
            self.boards[board_id].update({
                'last_seen': current_time,
                'title': board_data.get('title', self.boards[board_id].get('title', '')),
                'event_date': board_data.get('event_date', self.boards[board_id].get('event_date', '')),
                'status': board_data.get('status', 'ok'),
                'is_expired': board_data.get('is_expired', False),
                'is_duplicate': board_data.get('is_duplicate', False),
                'summary': board_data.get('summary', self.boards[board_id].get('summary', '')),
                'visit_count': self.boards[board_id].get('visit_count', 0) + 1
            })
        else:
            # Yeni kayıt oluştur
            self.boards[board_id] = {
                'board_id': board_id,
                'last_seen': current_time,
                'first_seen': current_time,
                'title': board_data.get('title', ''),
                'event_date': board_data.get('event_date', ''),
                'status': board_data.get('status', 'ok'),
                'is_expired': board_data.get('is_expired', False),
                'is_duplicate': board_data.get('is_duplicate', False),
                'summary': board_data.get('summary', ''),
                'visit_count': 1,
                'needs_revisit': False,
                'revisit_reason': ''
            }
        
        # Revisit gereksinimini belirle
        self._update_revisit_status(board_id)
        
        # Değişiklikleri kaydet
        self.save_memory()
        
        print(f"💾 Pano {board_id} güncellendi: {self.boards[board_id]['status']}")
        return True
    
    def _update_revisit_status(self, board_id: int):
        """Pano için revisit gereksinimini güncelle"""
        board = self.boards[board_id]
        
        needs_revisit = False
        revisit_reason = ''
        
        if board['status'] == 'expired':
            needs_revisit = True
            revisit_reason = 'expired_date'
        elif board['status'] == 'unclear':
            needs_revisit = True
            revisit_reason = 'unclear_content'
        elif board['is_duplicate']:
            needs_revisit = True
            revisit_reason = 'duplicate_detected'
        
        board['needs_revisit'] = needs_revisit
        board['revisit_reason'] = revisit_reason
    
    def is_recently_visited(self, board_id: int, minutes: int = 10) -> bool:
        """
        Bir pano son X dakika içinde ziyaret edildi mi?
        
        Args:
            board_id: Pano ID'si
            minutes: Kaç dakika önce (varsayılan: 10)
            
        Returns:
            bool: True ise son X dakika içinde ziyaret edilmiş
        """
        if board_id not in self.boards:
            return False
        
        last_seen_str = self.boards[board_id].get('last_seen', '')
        if not last_seen_str:
            return False
        
        try:
            last_seen = datetime.strptime(last_seen_str, "%Y-%m-%d %H:%M:%S")
            time_diff = datetime.now() - last_seen
            return time_diff.total_seconds() < (minutes * 60)
        except:
            return False
    
    def get_boards_needing_revisit(self, exclude_recent_minutes: int = 10) -> List[Dict]:
        """
        Yeniden ziyaret edilmesi gereken panoları öncelik sırasına göre döndür
        
        Args:
            exclude_recent_minutes: Son X dakika içinde ziyaret edilenleri hariç tut (varsayılan: 10)
        """
        revisit_list = []
        
        for board_id, board in self.boards.items():
            if board.get('needs_revisit', False):
                # Son X dakika içinde ziyaret edilmişse atla
                if exclude_recent_minutes > 0 and self.is_recently_visited(board_id, exclude_recent_minutes):
                    continue
                
                # Öncelik hesaplama: expired > unclear > duplicate
                priority = 0
                if board['status'] == 'expired':
                    priority = 3
                elif board['status'] == 'unclear':
                    priority = 2
                elif board['is_duplicate']:
                    priority = 1
                
                revisit_list.append({
                    'board_id': board_id,
                    'priority': priority,
                    'reason': board.get('revisit_reason', ''),
                    'last_seen': board.get('last_seen', ''),
                    'visit_count': board.get('visit_count', 0)
                })
        
        # Önceliğe göre sırala (yüksek öncelik önce)
        revisit_list.sort(key=lambda x: x['priority'], reverse=True)
        return revisit_list
    
    def get_all_boards(self) -> Dict[int, Dict]:
        """Tüm pano kayıtlarını döndür"""
        return self.boards.copy()
    
    def get_board(self, board_id: int) -> Optional[Dict]:
        """Belirli bir pano kaydını döndür"""
        return self.boards.get(board_id)
    
    def mark_revisit_completed(self, board_id: int):
        """Bir pano ziyaret edildikten sonra revisit flag'ini temizle"""
        if board_id in self.boards:
            self.boards[board_id]['needs_revisit'] = False
            self.boards[board_id]['revisit_reason'] = ''
            self.save_memory()
            print(f"✅ Pano {board_id} revisit tamamlandı olarak işaretlendi")
    
    def get_statistics(self) -> Dict:
        """Sistem istatistiklerini döndür"""
        total_boards = len(self.boards)
        expired_count = sum(1 for b in self.boards.values() if b.get('status') == 'expired')
        unclear_count = sum(1 for b in self.boards.values() if b.get('status') == 'unclear')
        duplicate_count = sum(1 for b in self.boards.values() if b.get('is_duplicate', False))
        needs_revisit_count = sum(1 for b in self.boards.values() if b.get('needs_revisit', False))
        
        return {
            'total_boards': total_boards,
            'expired': expired_count,
            'unclear': unclear_count,
            'duplicate': duplicate_count,
            'needs_revisit': needs_revisit_count
        }
