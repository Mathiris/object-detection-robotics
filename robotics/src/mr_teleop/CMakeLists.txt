cmake_minimum_required(VERSION 2.8.3)
project(mr_teleop)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  rospy
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mr_teleop
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs
)

catkin_python_setup()
###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node_cpp src/control_node.cpp
                                    src/control.cpp)

add_dependencies(${PROJECT_NAME}_node_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node_cpp
  ${catkin_LIBRARIES}
)

