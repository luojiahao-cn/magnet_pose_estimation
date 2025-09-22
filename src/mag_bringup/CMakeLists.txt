cmake_minimum_required(VERSION 3.0.2)
project(mag_bringup)

find_package(catkin REQUIRED)

catkin_package()

# Install launch files so roslaunch can find them after install
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
