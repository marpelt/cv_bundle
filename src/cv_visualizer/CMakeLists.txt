cmake_minimum_required(VERSION 3.0.2)
project(cv_visualizer)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  message_filters
  cv_msg
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs cv_bridge message_filters cv_msg
)

catkin_install_python(PROGRAMS
  scripts/visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)