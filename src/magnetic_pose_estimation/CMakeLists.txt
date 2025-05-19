cmake_minimum_required(VERSION 3.0.2)
project(magnetic_pose_estimation)

# 设置C++标准为C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
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

# 消息定义
add_message_files(
  FILES
  MagneticField.msg
  MagnetPose.msg
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp sensor_msgs std_msgs std_srvs tf2 tf2_ros
  DEPENDS EIGEN3
)

# 头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# 主库，包含核心功能
add_library(${PROJECT_NAME}
  src/sensor_config.cpp
  src/magnet_pose_estimator_optimization.cpp
  # 可在此处添加其它核心源文件
)

# 磁场计算库
add_library(magnetic_field_calculator
  src/magnetic_field_calculator.cpp
)

# 主库依赖磁场计算库
target_link_libraries(${PROJECT_NAME}
  magnetic_field_calculator
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# 可执行文件：磁铁位置估计节点
add_executable(magnet_pose_estimator_node src/magnet_pose_estimator_node.cpp)
target_link_libraries(magnet_pose_estimator_node
  ${PROJECT_NAME}
  magnetic_field_calculator
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
add_dependencies(magnet_pose_estimator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 可执行文件：仿真节点
add_executable(simulation_node src/simulation_node.cpp)
target_link_libraries(simulation_node
  ${PROJECT_NAME}
  magnetic_field_calculator
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
add_dependencies(simulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 可执行文件：磁场可视化
add_executable(magnetic_field_visualizer src/visualization_node.cpp)
target_link_libraries(magnetic_field_visualizer
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# 可执行文件：传感器读取
add_executable(sensor_reader_node src/sensor_reader_node.cpp)
target_link_libraries(sensor_reader_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# 可执行文件：地磁场处理
add_executable(earth_field_processor_node src/earth_field_processor_node.cpp)
target_link_libraries(earth_field_processor_node
  ${catkin_LIBRARIES}
)
add_dependencies(earth_field_processor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# RViz面板（如有Qt依赖）
set(CMAKE_AUTOMOC ON)
if(rviz_QT_VERSION VERSION_LESS "5")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    include(${QT_USE_FILE})
    set(QT_LIBRARIES ${QT_LIBRARIES})
else()
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    set(QT_LIBRARIES Qt5::Widgets)
endif()
add_definitions(-DQT_NO_KEYWORDS)

add_library(magnetic_field_panel 
    src/magnetic_field_panel.cpp 
    include/magnetic_pose_estimation/magnetic_field_panel.hpp)
add_dependencies(magnetic_field_panel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(magnetic_field_panel 
    ${QT_LIBRARIES} 
    ${catkin_LIBRARIES}
)

# 链接catkin库目录
link_directories(${catkin_LIBRARY_DIRS})

# 结束
