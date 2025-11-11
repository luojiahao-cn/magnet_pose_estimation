cmake_minimum_required(VERSION 3.0.2)
project(mag_pose_estimation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  magnet_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

# 本包不再定义/生成自有消息，统一使用 mag_sensor_node 中的消息

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES magnetic_field_calculator mag_pose_estimation_lib
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs std_srvs tf2 tf2_ros magnet_msgs
  DEPENDS Eigen3 Ceres
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)

# ---- Libraries ----
add_library(magnetic_field_calculator
  src/mag_field_calculator.cpp
)

add_library(mag_pose_estimation_lib
  src/mag_pose_estimator_optimization.cpp
  src/mag_pose_estimator_kalman.cpp
)

add_dependencies(magnetic_field_calculator ${catkin_EXPORTED_TARGETS})
add_dependencies(mag_pose_estimation_lib ${catkin_EXPORTED_TARGETS})

target_link_libraries(mag_pose_estimation_lib
  magnetic_field_calculator
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${CERES_LIBRARIES}
)

# ---- Executable ----
add_executable(magnet_pose_estimator_node src/mag_pose_estimator_node.cpp)

add_dependencies(magnet_pose_estimator_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(magnet_pose_estimator_node
  mag_pose_estimation_lib
  ${catkin_LIBRARIES}
)

# ---- Install ----
install(TARGETS magnetic_field_calculator mag_pose_estimation_lib magnet_pose_estimator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
