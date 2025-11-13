cmake_minimum_required(VERSION 3.0.2)
project(magnet_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
  actionlib_msgs
)

add_message_files(
  FILES
  MagSensorData.msg
  MagnetPose.msg
)

add_action_files(
  FILES
  TrackMagnet.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_runtime actionlib_msgs
)