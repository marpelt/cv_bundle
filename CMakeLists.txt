cmake_minimum_required(VERSION 3.0.2)
project(cv_qr_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  cv_msg
)

find_package(OpenCV REQUIRED)

catkin_package(
)

find_path(ZBAR_INCLUDE_DIR zbar.h)
find_library(ZBAR_LIBRARY zbar)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ZBAR_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/detector.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARY}
)

add_dependencies(${PROJECT_NAME}_node cv_msg_generate_messages_cpp)