cmake_minimum_required(VERSION 3.0.2)
project(mag_single_tracking_app)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mag_core_description
  mag_core_msgs
  mag_core_utils
  mag_device_sensor
  mag_device_magnet
  mag_device_arm
  mag_pose_estimator
  mag_viz
  rosparam_shortcuts
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    mag_core_description
    mag_core_msgs
    mag_core_utils
    mag_device_sensor
    mag_device_magnet
    mag_device_arm
    mag_pose_estimator
    mag_viz
    rosparam_shortcuts
)

# 本包不包含自定义节点，使用 mag_device_sensor 的功能发布传感器阵列 TF
# 如需添加自定义节点，可在此处添加编译目标

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

