cmake_minimum_required(VERSION 3.9)
project(chameleon_nav)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs rospy)

## Setup for python modules and scripts
# catkin_python_setup()

# Declare a catkin package
catkin_package(CATKIN_DEPENDS geometry_msgs rospy)

# Install
catkin_install_python(PROGRAMS nodes/move_straight DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})