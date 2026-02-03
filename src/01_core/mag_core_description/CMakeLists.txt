cmake_minimum_required(VERSION 3.0.2)
project(mag_core_description)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs
	tf2
	tf2_ros
	mag_core_utils
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES mag_core_description
	CATKIN_DEPENDS roscpp geometry_msgs tf2 tf2_ros mag_core_utils
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(mag_core_description
	src/sensor_array_description.cpp
)

target_link_libraries(mag_core_description
	${catkin_LIBRARIES}
)

install(TARGETS mag_core_description
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

add_executable(mag_description_node src/mag_description_node.cpp)
target_link_libraries(mag_description_node
	mag_core_description
	${catkin_LIBRARIES}
)

install(TARGETS mag_description_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
