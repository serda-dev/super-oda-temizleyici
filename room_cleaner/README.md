# Oda Temizleyici (Room Cleaner)

**TurtleBot3 için Otonom Haritalama ve Temizlik Sistemi**

Robotun tüm hareketleri için explore_lite ve move_base kullanan iki aşamalı bir sistemdir.
Özel hareket kontrol kodu içermez; tamamen ROS navigasyon yığını (navigation stack) üzerinden çalışır.

## Mimari

**Faz 1: Haritalama (Mapping)**
```
/scan + /odom → slam_gmapping → /map → explore_lite → move_base → /cmd_vel
```

**Faz 2: Temizlik (Cleaning)**
```
map_server → /map → AMCL → explore_lite → move_base → /cmd_vel
```

## Gereksinimler

- ROS1 Noetic
- TurtleBot3 paketleri
- explore_lite (m-explore)
- Navigasyon yığını (Navigation stack)

## Kurulum

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Kullanım

### Faz 1: Haritalama (Mapping)

```bash
export TURTLEBOT3_MODEL=burger
roslaunch room_cleaner explore_mapping.launch
```

Robot otonom olarak çevreyi keşfeder ve bir harita oluşturur.

**Yeterli haritayı elde ettiğinizde, durdurun ve kaydedin:**
```bash
# Durdurmak için Ctrl+C tuşuna basın
rosrun map_server map_saver -f ~/catkin_ws/src/room_cleaner/maps/my_map
```

### Faz 2: Temizlik (Cleaning)

```bash
export TURTLEBOT3_MODEL=burger
roslaunch room_cleaner room_cleaner.launch map_file:=/yol/my_map.yaml
```

Robot, explore_lite kullanarak kaydedilen harita üzerinde gezinir.

## Launch Dosyaları

| Dosya | Amaç |
| `explore_mapping.launch` | SLAM + Keşif (Faz 1) |
| `room_cleaner.launch` | Harita Üzerinde Gezinme (Faz 2) |

## Tasarım Felsefesi

- **Özel hareket kodu yok** - Tüm süreci explore_lite ve move_base yönetir.
- **İki manuel aşama** - Aşama geçişi kullanıcı kontrolündedir.
- **Basit ve sağlam** - Kanıtlanmış TurtleBot3 navigasyon varsayılanlarını kullanır.
