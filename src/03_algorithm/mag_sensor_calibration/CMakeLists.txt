cmake_minimum_required(VERSION 3.0.2)
project(mag_sensor_calibration)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  mag_core_msgs
  mag_core_utils
  mag_device_arm
  rosparam_shortcuts
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mag_sensor_calibration_core
  CATKIN_DEPENDS mag_core_msgs mag_core_utils mag_device_arm
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(mag_sensor_calibration_core
  src/ellipsoid_fitter.cpp
  src/calibration_data_collector.cpp
  src/calibration_applier.cpp
  src/calibration_config_loader.cpp
)

add_executable(mag_sensor_calibration_node
  src/calibration_node.cpp
)

target_link_libraries(mag_sensor_calibration_core
  ${catkin_LIBRARIES}
)

target_link_libraries(mag_sensor_calibration_node
  mag_sensor_calibration_core
  ${catkin_LIBRARIES}
)

install(TARGETS mag_sensor_calibration_core mag_sensor_calibration_node
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

