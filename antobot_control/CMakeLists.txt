cmake_minimum_required(VERSION 3.8)
project(antobot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(antobot_platform_msgs REQUIRED)


add_library(antobot_control_core src/control_base.cpp)
ament_target_dependencies(antobot_control_core
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  antobot_platform_msgs
)
target_include_directories(antobot_control_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(diff_control_node
  src/diff_control.cpp
  src/diff_model/wheel_diff_model.cpp
  src/diff_model/track_diff_model.cpp
)
target_link_libraries(diff_control_node antobot_control_core)
ament_target_dependencies(diff_control_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  antobot_platform_msgs
)


add_executable(wheel_control_node src/wheel_control.cpp)
target_link_libraries(wheel_control_node antobot_control_core)
ament_target_dependencies(wheel_control_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  antobot_platform_msgs
)


install(
  TARGETS
    antobot_control_core
    diff_control_node
    wheel_control_node
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
