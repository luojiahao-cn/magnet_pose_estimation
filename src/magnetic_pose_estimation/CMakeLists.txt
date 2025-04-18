cmake_minimum_required(VERSION 3.0.2)
project(magnetic_pose_estimation)

# 设置C++标准为C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  serial
  rviz
)

find_package(Eigen3 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MagneticField.msg
  MagnetPose.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES magnetic_pose_estimation
#  CATKIN_DEPENDS geometry_msgs message_generation roscpp sensor_msgs std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
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

# 添加可执行文件
add_executable(simulation_node 
  src/simulation_node.cpp
)

# 添加依赖项
target_link_libraries(simulation_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# 确保消息生成在编译之前完成
add_dependencies(simulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 添加新的源文件
add_library(magnetic_field_calculator
  src/magnetic_field_calculator.cpp
)

# 链接依赖
target_link_libraries(magnetic_field_calculator
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# 确保 simulation_node 链接新的库
target_link_libraries(simulation_node
  magnetic_field_calculator
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# 添加磁铁位置估计节点
add_executable(magnet_pose_estimator_node 
  src/magnet_pose_estimator_node.cpp
)

# 添加依赖项
target_link_libraries(magnet_pose_estimator_node
  ${PROJECT_NAME}
  magnetic_field_calculator
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# 确保消息生成在编译之前完成
add_dependencies(magnet_pose_estimator_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# 添加新的可执行文件
add_executable(sensor_reader_node src/sensor_reader_node.cpp)
target_link_libraries(sensor_reader_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)



link_directories(${catkin_LIBRARY_DIRS})

set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    ## pull in all required include dirs, define QT_LIBRARIES, etc.
    include(${QT_USE_FILE})
else()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
    set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)


add_library(magnetic_field_panel 
    src/magnetic_field_panel.cpp 
    include/magnetic_pose_estimation/magnetic_field_panel.hpp)


add_dependencies(magnetic_field_panel 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)



target_link_libraries(magnetic_field_panel 
    ${QT_LIBRARIES} 
    ${catkin_LIBRARIES}
)

add_executable(earth_field_processor_node src/earth_field_processor_node.cpp)
target_link_libraries(earth_field_processor_node
  ${catkin_LIBRARIES}
)
add_dependencies(earth_field_processor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
