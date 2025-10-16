cmake_minimum_required(VERSION 3.0.2)
project(mag_sensor_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  serial
  tf2
  tf2_ros
)

add_message_files(FILES
  MagSensorData.msg
  MagnetPose.msg
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs serial message_runtime tf2 tf2_ros
  INCLUDE_DIRS include
  LIBRARIES sensor_config
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(sensor_config STATIC
  src/sensor_config.cpp
)

# 允许被共享库链接
set_target_properties(sensor_config PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_dependencies(sensor_config ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(sensor_config ${catkin_LIBRARIES})

add_executable(mag_sensor_node src/mag_sensor_node.cpp)
add_dependencies(mag_sensor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mag_sensor_node
  sensor_config
  ${catkin_LIBRARIES}
)

# Sim node
add_executable(mag_sensor_sim_node src/mag_sensor_sim_node.cpp)
add_dependencies(mag_sensor_sim_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mag_sensor_sim_node
  sensor_config
  ${catkin_LIBRARIES}
)

# Sensor TF publisher
add_executable(sensor_tf_publisher_node src/sensor_tf_publisher_node.cpp)
add_dependencies(sensor_tf_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_tf_publisher_node
  sensor_config
  ${catkin_LIBRARIES}
)

install(TARGETS mag_sensor_node mag_sensor_sim_node sensor_tf_publisher_node sensor_config
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch config include msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
