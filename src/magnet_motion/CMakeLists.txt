cmake_minimum_required(VERSION 3.0.2)
project(magnet_motion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp tf2_ros geometry_msgs
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(magnet_motion_node src/magnet_motion_node.cpp)
add_dependencies(magnet_motion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(magnet_motion_node
  ${catkin_LIBRARIES}
)

install(TARGETS magnet_motion_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
