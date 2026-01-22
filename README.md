# Room Cleaner - Otonom Haritalama ve Temizlik Paketi

Bu paket, **TurtleBot3** robotları için iki aşamalı (haritalama ve temizlik) otonom bir çözüm sunar. Karmaşık ve hataya açık özel hareket algoritmaları yerine, ROS navigasyon yığınının (`move_base`, `gmapping`, `amcl`) ve `explore_lite` paketinin kararlılığını kullanır.

[DEMO VİDEOSU](https://youtu.be/s3Tn_79ccaM)

## Özellikler

*   **İki Aşamalı Yapı:** Haritalama (Mapping) ve Temizlik (Cleaning) süreçleri birbirinden tamamen ayrılmıştır.
*   **Tam Otonom Keşif:** `explore_lite` kullanarak ortamın bilinmeyen bölgelerini otomatik olarak bulur ve haritalar.
*   **Izgara Tabanlı Temizlik:** Oluşturulan harita üzerinde belirlenen çözünürlükte sistematik bir şekilde gezinir.
*   **Standart Navigasyon:** Tüm hareketler `move_base` üzerinden yönetilir, bu sayede engellerden kaçınma ve yol planlama güvenilirdir.

## Gereksinimler

Bu paket aşağıdaki ROS paketlerine ihtiyaç duyar:

*   ROS Noetic (Ubuntu 20.04)
*   `turtlebot3` paketleri (simülasyon ve navigasyon)
*   `explore_lite` (m-explore paketi)
*   `gmapping` (SLAM için)
*   `map_server`

## Kurulum

1.  Paketi çalışma alanınıza (catkin workspace) klonlayın:
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/serda-dev/super-oda-temizleyici.git room_cleaner
    ```

2.  Bağımlılıkları yükleyin:
    ```bash
    cd ~/catkin_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  Paketi derleyin:
    ```bash
    catkin_make
    source devel/setup.bash
    ```

## Kullanım

### 1. Aşama: Haritalama (Mapping)

Bu aşamada robot, ortamı gezerek bir harita oluşturur. Simülasyon ortamı (Gazebo) otomatik olarak açılacaktır.

1.  Haritalama modunu başlatın:
    ```bash
    export TURTLEBOT3_MODEL=burger
    roslaunch room_cleaner explore_mapping.launch
    ```

2.  Robot otonom olarak ortamı keşfetmeye başlayacaktır. Keşif tamamlandığında veya harita yeterli seviyeye geldiğinde işlemi sonlandırın.

3.  Haritayı kaydedin:
    ```bash
    rosrun map_server map_saver -f ~/catkin_ws/src/room_cleaner/maps/my_map
    ```

### 2. Aşama: Temizlik (Cleaning)

Bu aşamada robot, kaydedilmiş harita üzerinde konumlanır ve tüm erişilebilir alanları gezerek "temizlik" yapar.

1.  Temizlik modunu başlatın (Harita dosya yolunuza dikkat edin):
    ```bash
    export TURTLEBOT3_MODEL=burger
    roslaunch room_cleaner room_cleaner.launch map_file:=$(find room_cleaner)/maps/my_map.yaml
    ```
    *Not: Kendi kaydettiğiniz haritayı kullanmak için `map_file` argümanını düzenleyebilir veya komut satırından verebilirsiniz.*

2.  Robot, `cleaning_manager` düğümü tarafından yönetilen hedeflere sırasıyla gidecektir.

## Ayarlar

Paket ayarları `config/params.yaml` dosyasında bulunur. Önemli parametreler şunlardır:

### Keşif Ayarları (`exploration`)
*   `min_exploration_time`: Robotun keşfi sonlandırmadan önce en az ne kadar süre (saniye) çalışacağı.
*   `exploration_timeout`: Keşif işleminin maksimum süresi.
*   `unknown_ratio_threshold`: Haritanın ne kadarı bilinir olduğunda keşfin tamamlanmış sayılacağı (%30 için 0.30).

### Temizlik Ayarları (`coverage`)
*   `lane_width`: Temizlik şeritleri arasındaki mesafe (metre). Bu değer robotun kapsama alanını belirler.
*   `linear_speed`: Temizlik sırasındaki ilerleme hızı.
*   `angular_speed`: Dönüş hızı.

### Temizlik Yöneticisi (`cleaning_manager`)
*   `cleaning/grid_resolution`: Temizlik hedeflerinin sıklığını belirleyen ızgara boyutu (metre).
*   `cleaning/free_threshold`: Bir hücrenin "boş" (gidilebilir) sayılması için gereken olasılık eşiği (0-100).
