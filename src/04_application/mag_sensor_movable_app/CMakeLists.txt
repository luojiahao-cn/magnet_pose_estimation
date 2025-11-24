cmake_minimum_required(VERSION 3.0.2)
project(mag_sensor_movable_app)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mag_core_description
  mag_core_msgs
  mag_core_utils
  mag_device_sensor
  mag_device_magnet
  mag_pose_estimator
  mag_viz
  rosparam_shortcuts
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    mag_core_description
    mag_core_msgs
    mag_core_utils
    mag_device_sensor
    mag_device_magnet
    mag_pose_estimator
    mag_viz
    rosparam_shortcuts
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(array_motion_node
  src/array_motion_main.cpp
  src/array_motion_config_loader.cpp
  src/array_motion_node.cpp
)

target_link_libraries(array_motion_node
  ${catkin_LIBRARIES}
)

install(TARGETS array_motion_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

