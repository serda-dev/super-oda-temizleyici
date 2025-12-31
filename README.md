# TurtleBot3 İle Oda Temizliği

YT DEMO LİNKİ: https://youtu.be/5JacrlKR6MM

Bu ödevin konusu basittir. Turtlebot3 mapi ve sistem özellikleri kullanarak önceden tanımlanmış oda konumları ve temizlik rotalarını kullanarak oda temizliği yapacak bir sistem oluşturmaktır.

Map olarak Turtlebot3'ün kendi house mapi seçilmiştir, fakat mapin taranma işlemi turtlebot3_teleop kullanılarak elle yapılmıştır. (Demoda mevcut.)

İçeriğin yapısı gereği her oda ve ona tanımlı temizleme noktaları/koordinatları elle config/mission.yaml içerisine girilmiştir. Koordinatların tespiti RViz aracılığı ile yapılmıştır.


## Temel Özellikler
- **Task/Mission Yöneticisi**: Ödevde yer aldığı gibi merkezi bir yönlendirme sistemi bulunmaktadır. Robotun navigasyon, doğrulama, temizleme görevi ve koordinatları vs. gibi işlemler bu yönetici program aracılığı ile sağlanır.
- **Checkpoint Yönlendirmesi/Navigasyonu**: Robotun gitmesi gereken spesifik koordinatlarını olduğu arayüzdür.
- **QR Detector**: QR Detector koordinatları elle girilmiş olup bir Message/Subscriber mantığı ile iletişim sağlanmaktadır. Fotoğraf çekip ondan bir yönlendirme sağlanmaz, fakat simülasyon sonrası uygulama için programın kendisi buna uygundur. (( Elle message giriliyor. Ek bir program yazılarak QR kod eklemesi sağlanabilir. Ama ben tam beceremediğim için hiç eklemedim.))

## Prerequisites
- ROS1 Noetic
- `turtlebot3` ve `turtlebot3_simulations` paketleri

## Installation

1. Bu repodaki dosyaları bir klasör halinde catkin workspace'inizin içindeki 'src' klasörünün içine kopyalayın.
   ```bash
   cd ~/catkin_ws/src
   ```

2. catkin_make yaparak paketlerin kurulumunu sağlayın:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

3. **ÖNEMLİ**: Python kodlarını çalıştırılabilir/executable hale getirin:
   ```bash
   chmod +x src/turtlebot3_cleaning/scripts/*.py
   ```

## Kullanımı

QR Codeları aktif hale getirmeyi saymazsak üç farklı terminal kullanmanız yeterlidir.

### 1. Simülasyonu Açın
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

### 2. Navigasyon/MAP
Navigasyon launchunu çalıştırın (AMCL + MoveBase). `path/to/map.yaml` yazan kısmını kendi dosya yolunuz ile değiştirin. Mesela `/catkin_ws/benimmap123.yaml`
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```
*NOT!: RViz açıldığında robotun konumunu simülasyonu yerine koymanız gerekiyor. "2D POSE ESTIMATE" ayarını kullanarak robotun konumunu elle ayarlayabilirsiniz.

### 3. Görevi/Mission Başlatın.
```bash
roslaunch turtlebot3_cleaning mission.launch
```

## Ayarlar/Configuration

`config/mission.yaml` dosyasını kendi isteğinize göre değiştirebilirsiniz:
- **entry_goal**: Robotun QR codu kontrol etmek için nereye gideceğini (koordinat) gösterir.
- **cleaning_goals**: QR code doğrulandıktan sonra temizlenecek koordinatları gösterir.
- **qr_expected**: Doğrulanan QR'un spesifik adını (beklenen) gösterir. (Mesela: "ROOM=KITCHEN"). Eğer yanlış bir isim yazarsanız QR kodu doğrular, ama odayı tespit edemediği için sonraki odaya geçer.

## Node/Düğüm Yapısı

- **mission_manager_node.py**: Ana kontrolcü. Config dosyalarını okur ve robotu yönlendirir.
- **waypoint_navigator_node.py**: `move+base` fonksiyonu için wrapper görevini üstlenir.
- **qr_detector_node.py**: QR kodlarını camera streami üzerinden kontrol eder. 
