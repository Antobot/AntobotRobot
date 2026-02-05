cmake_minimum_required(VERSION 3.8)
project(antobot_costmap_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(costmap_converter_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(costmap_converter REQUIRED)
find_package(antobot_move_msgs REQUIRED)

add_executable(occgrid_converter_bridge src/occgrid_converter_bridge.cpp)
ament_target_dependencies(occgrid_converter_bridge
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  costmap_converter_msgs
  std_msgs
  nav2_costmap_2d
  pluginlib
  costmap_converter
)
install(TARGETS occgrid_converter_bridge
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(collision_checker src/collision_checker.cpp)
ament_target_dependencies(collision_checker
  rclcpp
  geometry_msgs
  visualization_msgs
  costmap_converter_msgs
  antobot_move_msgs
)
install(TARGETS collision_checker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
