cmake_minimum_required(VERSION 3.0.2)
project(mag_sensor_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  magnet_msgs
  serial
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs magnet_msgs serial tf2 tf2_ros
  INCLUDE_DIRS include
  LIBRARIES sensor_array_description
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(sensor_array_description STATIC
  src/array_description.cpp
)

set_target_properties(sensor_array_description PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_dependencies(sensor_array_description ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(sensor_array_description ${catkin_LIBRARIES})

add_executable(mag_sensor_node src/mag_sensor_node.cpp)
add_dependencies(mag_sensor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mag_sensor_node
  sensor_array_description
  ${catkin_LIBRARIES}
)

# Sim node
add_executable(mag_sensor_sim_node src/mag_sensor_sim_node.cpp)
add_dependencies(mag_sensor_sim_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mag_sensor_sim_node
  sensor_array_description
  ${catkin_LIBRARIES}
)

# Magnet motion node: publishes TF for magnet (configurable parent/child frames)
## magnet_motion_node moved to separate package 'magnet_motion'

install(TARGETS mag_sensor_node mag_sensor_sim_node sensor_array_description
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch config include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
