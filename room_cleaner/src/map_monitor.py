#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Harita İzleme Düğümü - Harita Verilerini Analiz Eder ve İstatistik Üretir
========================================================================
Bu düğüm, SLAM (Gmapping vb.) tarafından yayınlanan harita verilerini (OccupancyGrid)
dinler ve haritadaki bilinmeyen, boş ve dolu alanların oranlarını hesaplayarak
yayınlar. Bu istatistikler keşif ve temizlik süreçlerinin takibi için kullanılır.

Abonelikler (Subscriptions):
    /map                        - SLAM'den gelen doluluk ızgarası (OccupancyGrid)

Yayınlar (Publications):
    /map_monitor/unknown_ratio  - Bilinmeyen (Unknown) hücre oranı (0.0 - 1.0)
    /map_monitor/free_ratio     - Boş (Free) hücre oranı (0.0 - 1.0)
    /map_monitor/occupied_ratio - Dolu (Occupied) hücre oranı (0.0 - 1.0)
    /map_monitor/stats          - Tüm istatistiklerin metin formatındaki özeti
"""

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32, String


class MapMonitor:
    """
    Harita verilerini analiz eden ve doluluk istatistiklerini yayınlayan sınıf.
    
    Bu sınıf haritadaki hücre değerlerini (-1, 0-49, 50-100) kullanarak
    keşfedilen alan miktarını ve robotun hareket edebileceği boş alanları hesaplar.
    """
    
    def __init__(self):
        """
        Düğümü ilklendirir, parametreleri okur ve yayıncı/aboneleri kurar.
        """
        rospy.init_node('map_monitor', anonymous=False)
        rospy.loginfo("[HARITA_IZLEME] Harita İzleme Düğümü Başlatılıyor...")
        
        # Yayın hızı parametresi (Hertz cinsinden)
        self.publish_rate = rospy.get_param('~map_monitor/publish_rate', 1.0)
        
        self.map_data = None
        self.map_info = None
        
        # Başlangıç istatistikleri
        self.unknown_ratio = 1.0
        self.free_ratio = 0.0
        self.occupied_ratio = 0.0
        self.total_cells = 0
        
        # Yayıncılar (Publishers)
        self.unknown_ratio_pub = rospy.Publisher(
            '/map_monitor/unknown_ratio', Float32, queue_size=10)
        self.free_ratio_pub = rospy.Publisher(
            '/map_monitor/free_ratio', Float32, queue_size=10)
        self.occupied_ratio_pub = rospy.Publisher(
            '/map_monitor/occupied_ratio', Float32, queue_size=10)
        self.stats_pub = rospy.Publisher(
            '/map_monitor/stats', String, queue_size=10)
        
        # Aboneler (Subscribers)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        # İstatistikleri periyodik olarak yayınlamak için zamanlayıcı (Timer)
        self.publish_timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self.publish_stats)
        
        rospy.loginfo("[HARITA_IZLEME] Harita İzleyici başarıyla kuruldu.")
    
    def map_callback(self, msg):
        """
        Gelen harita verisini alır ve doluluk oranlarını hesaplar.
        
        Args:
            msg (OccupancyGrid): SLAM düğümünden gelen harita mesajı.
        """
        self.map_data = np.array(msg.data)
        self.map_info = msg.info
        self.total_cells = len(self.map_data)
        
        # Hücre sayılarını hesaplama:
        # -1  : Bilinmeyen (Unknown)
        # 0-49: Boş (Free) - Genellikle 0 kesin boştur
        # 50-100: Dolu (Occupied/Obstacle) - 100 kesin doludur

        unknown_cells = np.sum(self.map_data == -1)
        free_cells = np.sum((self.map_data >= 0) & (self.map_data < 50))
        occupied_cells = np.sum(self.map_data >= 50)
        
        # Oranları hesapla
        if self.total_cells > 0:
            self.unknown_ratio = unknown_cells / self.total_cells
            self.free_ratio = free_cells / self.total_cells
            self.occupied_ratio = occupied_cells / self.total_cells
        else:
            self.unknown_ratio = 1.0
            self.free_ratio = 0.0
            self.occupied_ratio = 0.0
    
    def publish_stats(self, event):
        """
        Hesaplanan istatistikleri ilgili konulara (topics) yayınlar.
        
        Args:
            event: Zamanlayıcıdan gelen olay bilgisi.
        """
        if self.map_data is None:
            return
        
        # Bireysel oranları yayınla
        self.unknown_ratio_pub.publish(Float32(data=self.unknown_ratio))
        self.free_ratio_pub.publish(Float32(data=self.free_ratio))
        self.occupied_ratio_pub.publish(Float32(data=self.occupied_ratio))
        
        # Metin tabanlı özet rapor oluştur ve yayınla
        stats_msg = f"Harita: {self.map_info.width}x{self.map_info.height}, " \
                    f"Bilinmeyen: {self.unknown_ratio:.1%}, " \
                    f"Boş: {self.free_ratio:.1%}, " \
                    f"Dolu: {self.occupied_ratio:.1%}"
        self.stats_pub.publish(String(data=stats_msg))
        
        # Belirli aralıklarla console logu yaz (her 10 saniyede bir)
        rospy.loginfo_throttle(10.0, f"[HARITA_IZLEME] {stats_msg}")
    
    def run(self):
        """
        Düğümün ana döngüsü.
        """
        rospy.loginfo("[HARITA_IZLEME] Harita izleyici çalışıyor...")
        rospy.spin()


if __name__ == '__main__':
    try:
        monitor = MapMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[HARITA_IZLEME] Harita izleyici kapatılıyor.")
