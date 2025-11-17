cmake_minimum_required(VERSION 3.0.2)
project(mag_sensor_fixture_app)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mag_core_description
  mag_core_msgs
  mag_device_sensor
  mag_device_magnet
  mag_pose_estimator
  mag_viz
)

catkin_package()

install(DIRECTORY config launch simulation hardware rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
