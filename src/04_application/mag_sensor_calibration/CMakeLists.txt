cmake_minimum_required(VERSION 3.0.2)
project(mag_sensor_calibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  std_srvs
  visualization_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  eigen_conversions
  moveit_core
  moveit_ros_planning_interface
  mag_core_msgs
  mag_core_description
  mag_device_arm
  mag_device_sensor
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    std_srvs
    visualization_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_eigen
    eigen_conversions
    moveit_core
    moveit_ros_planning_interface
    mag_core_msgs
    mag_core_description
    mag_device_arm
    mag_device_sensor
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 生成服务文件
add_service_files(
  FILES
  StartCalibration.srv
  StopCalibration.srv
  SaveCalibrationData.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

# 校正控制节点
add_executable(calibration_controller_node 
  src/calibration_controller_node.cpp
  src/calibration_controller_main.cpp
)
target_link_libraries(calibration_controller_node ${catkin_LIBRARIES} mag_calibration_lib)
add_dependencies(calibration_controller_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# 数据采集节点
add_executable(data_collector_node 
  src/data_collector_node.cpp
  src/data_collector_main.cpp
)
target_link_libraries(data_collector_node ${catkin_LIBRARIES})
add_dependencies(data_collector_node ${catkin_EXPORTED_TARGETS})

# 校正算法库
add_library(mag_calibration_lib
  src/calibration_algorithm.cpp
)
target_link_libraries(mag_calibration_lib ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES})
target_include_directories(mag_calibration_lib PUBLIC ${yaml-cpp_INCLUDE_DIRS})
add_dependencies(mag_calibration_lib ${catkin_EXPORTED_TARGETS})

# 可视化节点
add_executable(calibration_viz_node 
  src/calibration_viz_node.cpp
  src/calibration_viz_main.cpp
)
target_link_libraries(calibration_viz_node ${catkin_LIBRARIES} mag_calibration_lib)
add_dependencies(calibration_viz_node ${catkin_EXPORTED_TARGETS})

install(TARGETS 
  calibration_controller_node
  data_collector_node
  mag_calibration_lib
  calibration_viz_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

