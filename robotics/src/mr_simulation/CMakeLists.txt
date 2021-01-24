cmake_minimum_required(VERSION 2.8.3)
project(mr_simulation)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

