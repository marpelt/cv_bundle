cmake_minimum_required(VERSION 3.5)
project(cv_msg)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)

add_message_files(
    FILES
    CV_msg.msg
    BBox.msg
    QR_Detection.msg
    Rotation_Detection.msg
    April_Detection.msg
    Landolt_Detection.msg
    Hazmat_Detection.msg
    Image_Detection.msg
)

generate_messages(
    DEPENDENCIES std_msgs
)

catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)