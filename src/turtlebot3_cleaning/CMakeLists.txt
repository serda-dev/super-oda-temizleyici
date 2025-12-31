cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_cleaning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  move_base_msgs
  actionlib
  cv_bridge
  image_transport
  sensor_msgs
  nav_msgs
  tf
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES turtlebot3_cleaning
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs move_base_msgs actionlib cv_bridge image_transport sensor_msgs nav_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(task_manager_node src/task_manager_node.cpp)
target_link_libraries(task_manager_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(qr_reader_node src/qr_reader_node.cpp)
target_link_libraries(qr_reader_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
