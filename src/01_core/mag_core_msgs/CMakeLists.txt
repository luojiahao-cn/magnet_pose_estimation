cmake_minimum_required(VERSION 3.0.2)
project(mag_core_msgs)

find_package(catkin REQUIRED COMPONENTS
	std_msgs
	geometry_msgs
	actionlib_msgs
	message_generation
)

add_message_files(
	FILES
		MagSensorData.msg
		MagnetPose.msg
		MagSensorBatch.msg
)

add_action_files(
	DIRECTORY action
	FILES TrackMagnet.action
)

generate_messages(
	DEPENDENCIES std_msgs geometry_msgs actionlib_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs geometry_msgs actionlib_msgs
)
