cmake_minimum_required(VERSION 3.0.2)
project(mag_core_utils)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rosparam_shortcuts
)

find_package(Eigen3 REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES mag_core_utils
	CATKIN_DEPENDS roscpp rosparam_shortcuts
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)

add_library(mag_core_utils
	src/rosparam_shortcuts_extensions.cpp
	src/xmlrpc_utils.cpp
)

target_link_libraries(mag_core_utils
	${catkin_LIBRARIES}
)

install(TARGETS mag_core_utils
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
