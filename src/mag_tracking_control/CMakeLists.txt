cmake_minimum_required(VERSION 3.0.2)
project(mag_tracking_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  magnet_msgs
  moveit_msgs
  actionlib
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs magnet_msgs moveit_msgs actionlib tf2 tf2_ros tf2_geometry_msgs
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tracking_controller_node src/tracking_controller_node.cpp src/main.cpp)
target_link_libraries(tracking_controller_node ${catkin_LIBRARIES})
add_dependencies(tracking_controller_node ${catkin_EXPORTED_TARGETS})