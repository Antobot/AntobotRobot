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

add_executable(occgrid_to_polygons src/occgrid_to_polygons.cpp)
ament_target_dependencies(occgrid_to_polygons
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  costmap_converter_msgs
  OpenCV
  std_msgs
  nav2_costmap_2d
  pluginlib
  costmap_converter
)
target_link_libraries(occgrid_to_polygons ${OpenCV_LIBS})

install(TARGETS occgrid_to_polygons
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(collision_sat_checker src/collision_sat_checker.cpp)
ament_target_dependencies(collision_sat_checker
  rclcpp
  geometry_msgs
  costmap_converter_msgs
)
install(TARGETS collision_sat_checker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/compare_polygons.py
  scripts/footprint_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
