cmake_minimum_required(VERSION 3.0.2)
project(mag_tracking_control)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  mag_core_msgs
  mag_core_utils
  mag_device_arm
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosparam_shortcuts
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mag_tracking_control_core
  CATKIN_DEPENDS 
    roscpp
    geometry_msgs
    mag_core_msgs
    mag_device_arm
    tf2
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 策略库
add_library(mag_tracking_control_core
  src/strategy_fixed_offset.cpp
  src/strategy_adaptive_distance.cpp
  src/tracking_control_config_loader.cpp
)

# 主节点
add_executable(tracking_control_node
  src/tracking_control_node.cpp
)

target_link_libraries(mag_tracking_control_core
  ${catkin_LIBRARIES}
)

target_link_libraries(tracking_control_node
  mag_tracking_control_core
  ${catkin_LIBRARIES}
)

install(TARGETS mag_tracking_control_core tracking_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.launch"
)
