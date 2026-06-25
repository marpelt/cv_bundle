cmake_minimum_required(VERSION 3.0.2)
project(cv_msg_logger)

find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_msg
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)