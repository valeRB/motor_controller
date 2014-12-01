cmake_minimum_required(VERSION 2.8.3)
project(motor_controller)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  robot_msgs
#  genmsg
)

## Generate added messages and services
# sgenerate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
 CATKIN_DEPENDS roscpp robot_msgs
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(motor_controller src/war_motor_controller.cpp)
target_link_libraries(motor_controller ${catkin_LIBRARIES})

