cmake_minimum_required(VERSION 3.0.2)
project(mag_core_utils)

find_package(catkin REQUIRED COMPONENTS
	roscpp
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES mag_core_utils
	CATKIN_DEPENDS roscpp
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(mag_core_utils
	src/param_reader.cpp
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
