cmake_minimum_required(VERSION 2.8.3)
project(mr_control)

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
)

add_service_files(
  FILES
  Goal.srv
)

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mr_control
  CATKIN_DEPENDS nav_msgs roscpp message_runtime tf2 tf2_ros std_msgs geometry_msgs roslib tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(paths_follower src/paths_follower.cpp src/csv_reader.cpp)

add_executable(fake_loc src/fake_localization.cpp)

add_dependencies(paths_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(fake_loc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(paths_follower
	  ${catkin_LIBRARIES}
)

target_link_libraries(fake_loc
	  ${catkin_LIBRARIES}
)
