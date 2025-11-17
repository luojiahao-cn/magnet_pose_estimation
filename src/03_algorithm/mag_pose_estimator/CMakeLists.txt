cmake_minimum_required(VERSION 3.0.2)
project(mag_pose_estimator)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  mag_core_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mag_pose_estimator_core
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs std_msgs mag_core_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(mag_pose_estimator_core
  src/estimator_factory.cpp
  src/ekf_estimator.cpp
  src/optimizer_estimator.cpp
  src/mag_preprocessor.cpp
  src/mag_pose_estimator_node.cpp
)

add_executable(mag_pose_estimator_node
  src/mag_pose_estimator_node_main.cpp
)

target_link_libraries(mag_pose_estimator_core
  ${catkin_LIBRARIES}
)

target_link_libraries(mag_pose_estimator_node
  mag_pose_estimator_core
  ${catkin_LIBRARIES}
)

install(TARGETS mag_pose_estimator_core mag_pose_estimator_node
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
