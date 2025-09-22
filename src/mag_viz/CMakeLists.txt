cmake_minimum_required(VERSION 3.0.2)
project(mag_viz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mag_sensor_node
)

catkin_package(
  CATKIN_DEPENDS roscpp visualization_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs mag_sensor_node
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mag_sensor_viz_node src/mag_sensor_viz_node.cpp)
target_link_libraries(mag_sensor_viz_node ${catkin_LIBRARIES})

install(TARGETS mag_sensor_viz_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
