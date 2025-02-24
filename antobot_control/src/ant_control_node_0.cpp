/*
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

Description:  This code defines the ROS node that is used with the antobot hardware interface, including subscribers for
              calibration and track width. It also links the node to the hardware interface via the included header files.

Contacts: 	daniel.freer@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

*/

#include <antobot_control/ant_control.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int8.hpp>

int main(int argc, char** argv)
{
  // Initialises the ROS node and gets the node handle
  rclcpp::init(argc, argv, "am_control");
  rclcpp::Node nh;

  // Defines an antobot_hardware_interface class object using the defined ROS node
  antobot_hardware_interface::antobotHardwareInterface antobot1(nh);

  // Creates a subscriber for wheel velocity feedback from anto_bridge
  rclcpp::Subscriber sub_vWheel = nh.subscribe("/antobridge/wheel_vel", 10, &antobot_hardware_interface::antobotHardwareInterface::wheel_vel_Callback, &antobot1);

  // Send message indicating node is launched
  // ROS_INFO("SW1000: antobot_control node launched");

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  rclcpp::executors::MultiThreadedExecutor spinner();
  spinner.spin();
  rclcpp::shutdown();
  return 0;
}
