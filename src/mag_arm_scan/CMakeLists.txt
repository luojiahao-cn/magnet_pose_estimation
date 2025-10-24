cmake_minimum_required(VERSION 3.0.2)
project(mag_arm_scan)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  geometric_shapes
  mag_sensor_node
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    roslib
    std_msgs
    geometry_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs
    moveit_core
    moveit_ros_planning_interface
    geometric_shapes
    mag_sensor_node
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_controller_with_recorder_node src/scan_controller_with_recorder_node.cpp)
target_link_libraries(scan_controller_with_recorder_node ${catkin_LIBRARIES})
add_dependencies(scan_controller_with_recorder_node ${catkin_EXPORTED_TARGETS})

add_executable(tool_manager_node src/tool_manager_node.cpp)
target_link_libraries(tool_manager_node ${catkin_LIBRARIES})
add_dependencies(tool_manager_node ${catkin_EXPORTED_TARGETS})

install(TARGETS scan_controller_with_recorder_node tool_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
