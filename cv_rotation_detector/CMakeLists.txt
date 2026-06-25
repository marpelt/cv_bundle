cmake_minimum_required(VERSION 3.0.2)
project(cv_rotation_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  cv_msg
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS cv_msg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/detector.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)