cmake_minimum_required(VERSION 3.0.2)
project(mag_viz)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	mag_core_msgs
	visualization_msgs
	nav_msgs
	geometry_msgs
	tf2_geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS
		roscpp
		mag_core_msgs
		visualization_msgs
		nav_msgs
		geometry_msgs
		tf2_geometry_msgs
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(magnet_viz_node
	src/magnet_viz_node.cpp
)

target_link_libraries(magnet_viz_node
	${catkin_LIBRARIES}
)

install(TARGETS magnet_viz_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	FILES_MATCHING PATTERN "*.yaml" PATTERN "*.launch" PATTERN "*.rviz"
)
