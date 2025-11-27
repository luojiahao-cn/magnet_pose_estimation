cmake_minimum_required(VERSION 3.0.2)
project(mag_device_arm)

## Compile as C++17, supports modern features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs
	shape_msgs
	tf2_ros
	tf2_geometry_msgs
	mag_core_utils
	mag_core_msgs
	moveit_ros_planning_interface
	moveit_msgs
	geometric_shapes
	message_generation
	rosparam_shortcuts
)

add_service_files(
	FILES
		SetEndEffectorPose.srv
		ExecuteNamedTarget.srv
)

generate_messages(
	DEPENDENCIES
		geometry_msgs
		std_msgs
)

catkin_package(
	CATKIN_DEPENDS
		roscpp
		geometry_msgs
		shape_msgs
		tf2_ros
		tf2_geometry_msgs
		mag_core_utils
		mag_core_msgs
		moveit_ros_planning_interface
		moveit_msgs
		geometric_shapes
		message_runtime
		rosparam_shortcuts
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(arm_node
	src/arm_node.cpp
)

add_dependencies(arm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(arm_node
	${catkin_LIBRARIES}
)

install(TARGETS arm_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	FILES_MATCHING PATTERN "*.yaml" PATTERN "*.launch"
)

install(DIRECTORY meshes/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
	FILES_MATCHING PATTERN "*.STL" PATTERN "*.stl" PATTERN "*.SLDPRT"
)

catkin_install_python(
	PROGRAMS scripts/arm_ping_pong.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
