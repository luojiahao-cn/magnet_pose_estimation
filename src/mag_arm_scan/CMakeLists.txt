cmake_minimum_required(VERSION 3.0.2)
project(mag_arm_scan)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_orchestrator_node src/scan_orchestrator_node.cpp)
target_link_libraries(scan_orchestrator_node ${catkin_LIBRARIES})
add_dependencies(scan_orchestrator_node ${catkin_EXPORTED_TARGETS})
