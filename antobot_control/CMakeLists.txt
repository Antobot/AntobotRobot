cmake_minimum_required(VERSION 2.8.3)
project(antobot_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
  pluginlib
  transmission_interface
  urdf
  control_toolbox
  joint_limits_interface
  anto_bridge_msgs
  serial
  std_msgs
  message_generation
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    pluginlib
    transmission_interface
    urdf
    control_toolbox
    joint_limits_interface
    serial
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

add_library(ant_control src/ant_control.cpp)
target_link_libraries(ant_control
	${ant_control_LIBRARIES}
	${catkin_LIBRARIES})
add_executable(ant_control_node src/ant_control_node.cpp)
add_dependencies(ant_control_node antobot_move_control anto_bridge_msgs)

target_link_libraries(ant_control_node
	ant_control
	${catkin_LIBRARIES})


