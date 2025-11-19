cmake_minimum_required(VERSION 3.0.2)
project(mag_device_sensor)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs
	tf2
	tf2_ros
	tf2_geometry_msgs
	mag_core_description
	mag_core_utils
	mag_core_msgs
	rosparam_shortcuts
	serial
)

find_package(Eigen3 REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES mag_device_sensor
	CATKIN_DEPENDS roscpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs mag_core_description mag_core_utils mag_core_msgs rosparam_shortcuts serial
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)

add_library(mag_device_sensor
	src/sensor_node.cpp
	src/sensor_sim_node.cpp
	src/sensor_config_loader.cpp
)

target_link_libraries(mag_device_sensor
	${catkin_LIBRARIES}
)
add_dependencies(mag_device_sensor ${catkin_EXPORTED_TARGETS})

add_executable(sensor_driver_node
	src/sensor_driver_main.cpp
)

target_link_libraries(sensor_driver_node
	mag_device_sensor
	${catkin_LIBRARIES}
)
add_dependencies(sensor_driver_node ${catkin_EXPORTED_TARGETS})

add_executable(sensor_sim_node
	src/sensor_sim_main.cpp
)

target_link_libraries(sensor_sim_node
	mag_device_sensor
	${catkin_LIBRARIES}
)
add_dependencies(sensor_sim_node ${catkin_EXPORTED_TARGETS})

add_executable(sensor_array_batcher_node
	src/sensor_array_batcher_main.cpp
	src/sensor_array_batcher_node.cpp
)

target_link_libraries(sensor_array_batcher_node
	${catkin_LIBRARIES}
)
add_dependencies(sensor_array_batcher_node ${catkin_EXPORTED_TARGETS})

install(TARGETS mag_device_sensor sensor_driver_node sensor_sim_node sensor_array_batcher_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
