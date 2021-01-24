cmake_minimum_required(VERSION 2.8.3)
project(mr_path_recorder)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mr_path_recorder
  CATKIN_DEPENDS roscpp sensor_msgs std_srvs geometry_msgs tf2 tf2_ros nav_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_recorder src/path_recorder.cpp)

add_dependencies(path_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(path_recorder
  ${catkin_LIBRARIES}
)
