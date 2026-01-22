cmake_minimum_required(VERSION 3.0.2)
project(room_cleaner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

catkin_package()

# Python betiklerini yükle (Install Python scripts)
catkin_install_python(PROGRAMS
  src/cleaning_manager.py
  src/map_monitor.py
  src/report_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Launch dosyalarını yükle (Install launch files)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Yapılandırma dosyalarını yükle (Install config files)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
