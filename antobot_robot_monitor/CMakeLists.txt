cmake_minimum_required(VERSION 3.0.2)
project(antobot_robot_monitor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  antobot_platform_msgs
)

catkin_python_setup()

catkin_package(
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
