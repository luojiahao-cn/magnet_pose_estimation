cmake_minimum_required(VERSION 3.0.2)
project(magnetic_pose_estimation)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MagneticField.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES magnetic_pose_estimation
#  CATKIN_DEPENDS geometry_msgs message_generation roscpp sensor_msgs std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/sensor_config.cpp
  # ...other source files...
)

add_executable(magnetic_field_visualizer src/visualization_node.cpp)
target_link_libraries(magnetic_field_visualizer
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(test_publisher_node src/test_publisher_node.cpp)
target_link_libraries(test_publisher_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

