#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rapor Yöneticisi Düğümü - İstatistik Toplama ve Raporlama
=========================================================
Bu düğüm, görev boyunca istatistikleri toplar ve zamanlama bilgileri,
kapsama istatistikleri ve durum bilgilerini içeren bir final raporu oluşturur.

Abonelikler (Subscriptions):
    /task_manager/state           - Durum makinesi aşaması
    /task_manager/report          - Temel görev rapor verileri
    /map_monitor/stats            - Harita doluluk istatistikleri
    /coverage_controller/progress  - Temizlik aşaması ilerlemesi

Çıktılar:
    - Terminale özet rapor yazdırır
    - Belirlenen dizine .txt formatında dosya kaydeder

Yazar: Öğrenci
Ders: ROS1 Noetic - Üniversite Bitirme Projesi
"""


import rospy
import os
from datetime import datetime
from std_msgs.msg import String, Float32


class ReportManager:
    """
    Görev istatistiklerini toplayan ve final raporu üreten sınıf.
    
    Bu sınıf farklı düğümlerden gelen verileri konsolide eder ve proje klasörü
    altındaki 'maps' (veya parametre ile gelen) dizinine tarih-saat damgalı
    rapor dosyaları yazar.
    """
    
    def __init__(self):
        """
        Rapor yöneticisi düğümünü ilklendirir ve gerekli abonelikleri başlatır.
        """
        rospy.init_node('report_manager', anonymous=False)
        rospy.loginfo("[RAPOR] Rapor Yöneticisi Düğümü Başlatılıyor...")
        
        # Parametreleri yükle
        self.output_dir = rospy.get_param('~report/output_directory', 'maps')
        
        # Durum takibi
        self.state_transitions = []  # (zaman_damgası, durum) listesi
        self.current_state = None
        self.mission_start_time = rospy.Time.now()
        
        # Kapsama takibi
        self.last_coverage_progress = 0.0
        
        # Harita istatistikleri
        self.last_map_stats = ""
        
        # Aboneler (Subscribers)
        rospy.Subscriber('/task_manager/state', String, self.state_callback)
        rospy.Subscriber('/task_manager/report', String, self.report_callback)
        rospy.Subscriber('/map_monitor/stats', String, self.map_stats_callback)
        rospy.Subscriber('/coverage_controller/progress', Float32, self.coverage_progress_callback)
        
        rospy.loginfo("[RAPOR] Rapor Yöneticisi başarıyla hazırlandı.")
    
    def state_callback(self, msg):
        """
        Durum geçişlerini takip eder ve listeler.
        
        Args:
            msg (String): Yeni durum metni.
        """
        new_state = msg.data
        if new_state != self.current_state:
            timestamp = rospy.Time.now()
            self.state_transitions.append((timestamp, new_state))
            self.current_state = new_state
            rospy.loginfo(f"[RAPOR] Görev durumu değişti: {new_state}")
    
    def map_stats_callback(self, msg):
        """
        En güncel harita istatistiklerini saklar.
        
        Args:
            msg (String): Harita verisi özeti.
        """
        self.last_map_stats = msg.data
    
    def coverage_progress_callback(self, msg):
        """
        Temizlik (kapsama) ilerlemesini takip eder.
        
        Args:
            msg (Float32): İlerleme oranı (0.0 - 1.0).
        """
        self.last_coverage_progress = msg.data
    
    def report_callback(self, msg):
        """
        Görev yöneticisinden gelen final raporu mesajını işler ve dosyaya kaydeder.
        
        Args:
            msg (String): Temel rapor içeriği.
        """
        report_content = msg.data
        
        # Detaylı durum geçişlerini ekleyerek raporu zenginleştir
        detailed_report = self.generate_detailed_report(report_content)
        
        # Terminale yazdır
        rospy.loginfo("\n" + detailed_report)
        
        # Dosyaya kaydet
        self.save_report(detailed_report)
    
    def generate_detailed_report(self, base_report):
        """
        Temel rapora durum geçişlerini ve istatistikleri ekler.
        
        Args:
            base_report (str): İşlenecek temel rapor metni.
            
        Returns:
            str: Detaylandırılmış rapor metni.
        """
        lines = [base_report]
        lines.append("\n--- DURUM GEÇİŞLERİ ---")
        
        for i, (timestamp, state) in enumerate(self.state_transitions):
            time_str = datetime.fromtimestamp(timestamp.to_sec()).strftime('%H:%M:%S')
            lines.append(f"  {time_str} -> {state}")
        
        lines.append("\n--- FİNAL İSTATİSTİKLERİ ---")
        lines.append(f"  {self.last_map_stats}")
        lines.append(f"  Temizlik İlerlemesi: {self.last_coverage_progress:.1%}")
        lines.append("")
        
        return "\n".join(lines)
    
    def save_report(self, report_content):
        """
        Oluşturulan rapor içeriğini bir metin dosyasına kaydeder.
        
        Args:
            report_content (str): Kaydedilecek metin.
        """
        try:
            # Paket yolunu bul
            import rospkg
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('room_cleaner')
            
            # Çıktı dizinini oluştur (yoksa)
            output_path = os.path.join(pkg_path, self.output_dir)
            if not os.path.exists(output_path):
                os.makedirs(output_path)
            
            # Zaman damgalı dosya adı oluştur
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"gorev_raporu_{timestamp}.txt"
            filepath = os.path.join(output_path, filename)
            
            # Raporu yaz
            with open(filepath, 'w') as f:
                f.write(report_content)
                f.write(f"\n\nRapor oluşturulma tarihi: {datetime.now().isoformat()}\n")
            
            rospy.loginfo(f"[RAPOR] Rapor başarıyla dosyaya kaydedildi: {filepath}")
            
        except Exception as e:
            rospy.logerr(f"[RAPOR] Rapor dosyası kaydedilirken hata oluştu: {e}")
    
    def run(self):
        """
        Düğümün ana döngüsü.
        """
        rospy.loginfo("[RAPOR] Rapor yöneticisi aktif...")
        rospy.spin()


if __name__ == '__main__':
    try:
        manager = ReportManager()
        manager.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[RAPOR] Rapor yöneticisi kapatılıyor.")