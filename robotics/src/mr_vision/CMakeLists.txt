cmake_minimum_required(VERSION 2.8.3)
project(mr_vision)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  rospy
  sensor_msgs
)

catkin_package(
  LIBRARIES mr_vision
  CATKIN_DEPENDS geometry_msgs std_msgs sensor_msgs
)

catkin_python_setup()
