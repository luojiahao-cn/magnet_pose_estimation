cmake_minimum_required(VERSION 3.0.2)
project(mag_device_magnet)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs
	tf2
	tf2_ros
	tf2_geometry_msgs
	mag_core_utils
	mag_core_msgs
)

catkin_package(
	CATKIN_DEPENDS
		roscpp
		geometry_msgs
		tf2
		tf2_ros
		tf2_geometry_msgs
		mag_core_utils
		mag_core_msgs
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(magnet_main
	src/magnet_main.cpp
	src/magnet_node.cpp
)

target_link_libraries(magnet_main
	${catkin_LIBRARIES}
)

install(TARGETS magnet_main
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	FILES_MATCHING PATTERN "*.yaml" PATTERN "*.launch"
)
