cmake_minimum_required(VERSION 3.8)
project(antobot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(joint_limits REQUIRED)

add_executable(ant_control_node src/ant_control.cpp)
ament_target_dependencies(ant_control_node rclcpp std_msgs controller_manager controller_manager_msgs hardware_interface joint_state_broadcaster joint_limits)
target_include_directories(ant_control_node PUBLIC include include/antobot_control)

install(
  DIRECTORY include/${PROJECT_NAME}-00000
  TARGETS ant_control
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
