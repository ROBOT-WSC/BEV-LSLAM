cmake_minimum_required(VERSION 3.0.2)
project(imagecloud_msg)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  cloudparam.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)