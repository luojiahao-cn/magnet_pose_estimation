cmake_minimum_required(VERSION 3.0.2)
project(mag_arm_scan)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  std_srvs
  visualization_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  geometric_shapes
  mag_core_msgs
  mag_core_description
  mag_device_arm
  mag_device_sensor
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    std_srvs
    visualization_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    moveit_core
    moveit_ros_planning_interface
    geometric_shapes
    mag_core_msgs
    mag_core_description
    mag_device_arm
    mag_device_sensor
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 扫描控制节点
add_executable(scan_controller_node src/scan_controller_node.cpp)
target_link_libraries(scan_controller_node ${catkin_LIBRARIES})
add_dependencies(scan_controller_node ${catkin_EXPORTED_TARGETS})

install(TARGETS scan_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY config launch rviz meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

