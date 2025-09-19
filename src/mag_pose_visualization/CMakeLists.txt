cmake_minimum_required(VERSION 3.0.2)
project(mag_pose_visualization)

add_definitions(-DQT_NO_KEYWORDS)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  pluginlib
  visualization_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  mag_sensor_node
)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

catkin_package(
  CATKIN_DEPENDS roscpp rviz pluginlib visualization_msgs geometry_msgs std_srvs tf2 tf2_ros mag_sensor_node
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# RViz Panel library
set(PANEL_SRC
  src/mag_field_panel.cpp
)

add_library(magnetic_field_panel ${PANEL_SRC})
# Link with Qt5 and rviz
target_link_libraries(magnetic_field_panel
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

# Visualization node
add_executable(magnetic_field_visualizer_node src/visualization_node.cpp)

target_link_libraries(magnetic_field_visualizer_node
  ${catkin_LIBRARIES}
)

# Plugin XML
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS magnetic_field_panel magnetic_field_visualizer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Only install the new include path to avoid publishing legacy headers
install(DIRECTORY include/mag_pose_visualization/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/mag_pose_visualization
)

# Install config and launch for runtime usage
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
