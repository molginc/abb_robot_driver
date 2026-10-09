cmake_minimum_required(VERSION 3.0.2)

project(abb_rws_cfg_msgs)

find_package(catkin
  REQUIRED COMPONENTS
    message_generation
)

add_message_files(
  FILES
    ExtMotionData.msg
)

add_service_files(
  FILES
    GetExtMotionData.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
    message_runtime
)
