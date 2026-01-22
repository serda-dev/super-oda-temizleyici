#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Otonom Temizleme Yöneticisi - Navigasyon Temelli Kapsama Algoritması
===================================================================
Harita üzerinden gezinme hedefleri (goal) oluşturur ve bunları move_base action
sunucusuna gönderir. Robot hareketi tamamen move_base tarafından kontrol edilir;
doğrudan cmd_vel kontrolü yapılmaz.

Çalışma Mantığı:
1. Kayıtlı veya canlı haritayı (OccupancyGrid) alır.
2. Mevcut harita üzerindeki boş hücrelerden bir hedef listesi (zigzag/boustrophedon) oluşturur.
3. actionlib aracılığı ile hedefleri move_base'e sırayla gönderir.
4. Hedefe ulaşılıp ulaşılamadığını takip eder, ulaşılmazsa sonraki hedefe geçer.
5. Tüm hedefler tamamlandığında raporlama için durumu günceller.
"""

import rospy
import math
import numpy as np
import actionlib
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Float32
from tf.transformations import quaternion_from_euler


class CleaningManager:
    """
    move_base navigasyon hedeflerini yöneten ve tam alan kapsaması sağlayan sınıf.
    
    Hız kontrolü ve engel sakınma navigasyon yığınına (move_base) bırakılmıştır.
    Bu sınıf sadece "nereye gidileceği" kararını verir.
    """
    
    def __init__(self):
        """
        Düğümü ilklendirir, move_base sunucusuna bağlanır ve yayıncı/aboneleri kurar.
        """
        rospy.init_node('cleaning_manager', anonymous=False)
        rospy.loginfo("[TEMIZLIK] Otonom Temizleme Yöneticisi Başlatılıyor...")
        
        # Navigasyon ve ızgara parametrelerini al
        self.grid_resolution = rospy.get_param('~cleaning/grid_resolution', 0.5)  # Hedefler arası mesafe (metre)
        self.goal_timeout = rospy.get_param('~cleaning/goal_timeout', 30.0)      # Hedef başına maksimum süre (saniye)
        self.free_threshold = rospy.get_param('~cleaning/free_threshold', 50)    # Hücrenin "boş" sayılması için gereken üst değer
        
        self.map_received = False
        self.map_data = None
        self.map_info = None
        self.goals = []
        self.current_goal_idx = 0
        self.cleaning_active = False
        
        # move_base action client bağlantısı
        rospy.loginfo("[TEMIZLIK] move_base sunucusuna bağlanılıyor...")
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        connected = self.move_base_client.wait_for_server(timeout=rospy.Duration(30.0))
        
        if connected:
            rospy.loginfo("[TEMIZLIK] move_base bağlantısı başarılı!")
        else:
            rospy.logerr("[TEMIZLIK] move_base sunucusuna bağlanılamadı! Navigasyon çalışmayabilir.")
            return
        
        # Yayıncılar (Durum ve İlerleme)
        self.status_pub = rospy.Publisher('/cleaning_manager/status', String, queue_size=1)
        self.progress_pub = rospy.Publisher('/cleaning_manager/progress', Float32, queue_size=1)
        
        # Aboneler (Harita verisini al)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        rospy.loginfo(f"[TEMIZLIK] Ayarlar -> Çözünürlük: {self.grid_resolution}m, Zaman Aşımı: {self.goal_timeout}s")
    
    def map_callback(self, msg):
        """
        Harita verisini alır ve işleme hazır hale getirir.
        
        Args:
            msg (OccupancyGrid): Gelen harita mesajı.
        """
        if not self.map_received:
            rospy.loginfo("[TEMIZLIK] Harita verisi başarıyla alındı!")
            self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            self.map_info = msg.info
            self.map_received = True
    
    def generate_coverage_goals(self):
        """
        Boustrophedon (zigzag) deseni kullanarak ızgara tabanlı kaplama hedefleri oluşturur.
        
        Hedefler yalnızca harita verisinde "boş" (free) olarak işaretlenmiş hücrelere yerleştirilir.
        Robotun her yere uğramasını sağlayacak bir yol planı üretir.
        
        Returns:
            list: (x, y) koordinat çiftlerinden oluşan hedef listesi.
        """
        if not self.map_received:
            rospy.logwarn("[TEMIZLIK] Hedef oluşturulamıyor: Harita henüz hazır değil!")
            return []
        
        rospy.loginfo("[TEMIZLIK] Kapsama hedefleri (zigzag deseni) hesaplanıyor...")
        
        info = self.map_info
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        
        # Hücre cinsinden adım aralığını hesapla
        step_cells = max(1, int(self.grid_resolution / resolution))
        
        goals = []
        row_count = 0
        
        # Haritayı satır satır tara (zigzag için row-based tarama)
        for row in range(0, info.height, step_cells):
            row_goals = []
            
            for col in range(0, info.width, step_cells):
                # Harita sınırlarını kontrol et ve hücre değerini oku
                cell_value = self.map_data[row, col]
                
                # Sadece güvenli (boş) hücrelere hedef koy
                if 0 <= cell_value < self.free_threshold:
                    # Hücre indisini dünya koordinatlarına (metre) çevir
                    world_x = origin_x + (col + 0.5) * resolution
                    world_y = origin_y + (row + 0.5) * resolution
                    row_goals.append((world_x, world_y))
            
            # Boustrophedon (zigzag) için her iki satırda bir yönü ters çevir
            if row_count % 2 == 1:
                row_goals.reverse()
            
            goals.extend(row_goals)
            row_count += 1
        
        rospy.loginfo(f"[TEMIZLIK] Toplam {len(goals)} adet güvenli temizleme hedefi belirlendi.")
        return goals
    
    def create_move_base_goal(self, x, y, yaw=0.0):
        """
        Verilen koordinatlar için move_base protokolüne uygun bir hedef mesajı oluşturur.
        
        Args:
            x (float): Hedef X koordinatı.
            y (float): Hedef Y koordinatı.
            yaw (float): Hedef yönelim açısı (radyan).
            
        Returns:
            MoveBaseGoal: Gönderilmeye hazır hedef nesnesi.
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        
        # Açıyı quaternion formatına çevir
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation = Quaternion(*q)
        
        return goal
    
    def send_goal(self, x, y):
        """
        Belirtilen konumu move_base'e gönderir ve robotun oraya varmasını bekler.
        
        Args:
            x (float): Hedef X.
            y (float): Hedef Y.
            
        Returns:
            bool: Hedefe başarıyla ulaşıldıysa True, aksi halde False.
        """
        goal = self.create_move_base_goal(x, y)
        
        rospy.loginfo(f"[TEMIZLIK] Hedef {self.current_goal_idx + 1}/{len(self.goals)} gönderiliyor: ({x:.2f}, {y:.2f})")
        
        self.move_base_client.send_goal(goal)
        
        # Belirlenen süre boyunca hedefe varmayı bekle
        finished = self.move_base_client.wait_for_result(rospy.Duration(self.goal_timeout))
        
        if finished:
            state = self.move_base_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"[TEMIZLIK] Hedef {self.current_goal_idx + 1} başarıyla tamamlandı.")
                return True
            else:
                rospy.logwarn(f"[TEMIZLIK] Hedef {self.current_goal_idx + 1} iptal veya başarısız (Durum: {state}). Sıradaki hedefe geçiliyor.")
                return False
        else:
            rospy.logwarn(f"[TEMIZLIK] Hedef {self.current_goal_idx + 1} zaman aşımına uğradı! Navigasyon durduruluyor.")
            self.move_base_client.cancel_goal()
            return False
    
    def run_cleaning(self):
        """
        Temizleme sürecini baştan sona yürüten ana sekans.
        """
        rospy.loginfo("[TEMIZLIK] ===== OTONOM TEMİZLEME SÜRECİ BAŞLATILIYOR =====")
        self.status_pub.publish(String(data="HEDEFLER_OLUŞTURULUYOR"))
        
        # Kaplama hedeflerini üret
        self.goals = self.generate_coverage_goals()
        
        if not self.goals:
            rospy.logerr("[TEMIZLIK] Temizlenecek alan bulunamadı! Harita verisi boş veya hatalı.")
            self.status_pub.publish(String(data="BASARISIZ"))
            return
        
        self.cleaning_active = True
        self.status_pub.publish(String(data="TEMIZLENIYOR"))
        
        successful_goals = 0
        failed_goals = 0
        
        # Tüm hedefleri sırayla işle
        for i, (x, y) in enumerate(self.goals):
            if rospy.is_shutdown():
                break
            
            self.current_goal_idx = i
            
            # İlerleme durumunu yayınla
            progress = (i + 1) / len(self.goals)
            self.progress_pub.publish(Float32(data=progress))
            
            # Hedefe git ve sonucu işle
            success = self.send_goal(x, y)
            
            if success:
                successful_goals += 1
            else:
                failed_goals += 1
        
        # Temizlik bitti
        self.cleaning_active = False
        rospy.loginfo("=" * 50)
        rospy.loginfo(f"[TEMIZLIK] TEMİZLEME GÖREVİ TAMAMLANDI!")
        rospy.loginfo(f"[TEMIZLIK] Başarılı Hedef: {successful_goals}, Atlanan: {failed_goals}")
        rospy.loginfo("=" * 50)
        self.status_pub.publish(String(data="TAMAMLANDI"))
        self.progress_pub.publish(Float32(data=1.0))
    
    def run(self):
        """
        Düğümün ana giriş noktası ve hazırlık döngüsü.
        """
        # Harita gelene kadar bekle
        rospy.loginfo("[TEMIZLIK] Harita verisi için bekleniyor...")
        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and not self.map_received:
            rate.sleep()
        
        if rospy.is_shutdown():
            return
        
        # Sistemin oturması ve move_base'in hazır olması için kısa bir bekleme (2 saniye)
        rospy.sleep(2.0)
        
        # Temizlik sekansını başlat
        self.run_cleaning()
        
        # Düğümü açık tut
        rospy.spin()


if __name__ == '__main__':
    try:
        manager = CleaningManager()
        manager.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[TEMIZLIK] Düğüm kapatılıyor.")
