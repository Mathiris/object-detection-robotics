cmake_minimum_required(VERSION 2.8.3)
project(mr_roomba)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  message_generation
  tf
  tf2
  tf2_ros
  std_msgs
  roslib
  gazebo_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mr_roomba
  CATKIN_DEPENDS nav_msgs roscpp message_runtime tf2 tf2_ros std_msgs geometry_msgs roslib tf gazebo_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(roomba src/roomba.cpp)

add_dependencies(roomba ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(roomba
	  ${catkin_LIBRARIES}
)
