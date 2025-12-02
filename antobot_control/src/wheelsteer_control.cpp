#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"


class WheelSteerControl : public rclcpp::Node
{
public:
  WheelSteerControl()
  : Node("wheelsteer_control")
  {

    wheel_count_ = 4;

    enabled_ = {true, true, true, true};

    zero_offset_deg_.assign(wheel_count_, 0.0);

    raw_pos_deg_.assign(wheel_count_, 0.0);
    logical_pos_deg_.assign(wheel_count_, 0.0);

    RCLCPP_INFO(
      get_logger(),
      "WheelSteerControl started with %d wheels (IDs 1..4).",
      wheel_count_);


    // Raw feedback from bridge: degrees [0, 360).
    raw_pos_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/antobot/control/wheelsteer/real_pos_raw",
      50,
      std::bind(&WheelSteerControl::onRawPos, this, std::placeholders::_1));

    // High-level steering command (logical angles), degrees [0, 360).
    cmd_pos_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/antobot/control/wheelsteer/cmd_pos",
      50,
      std::bind(&WheelSteerControl::onCmdPos, this, std::placeholders::_1));

    // Set-zero command:
    //        1..4 : set zero for that single wheel
    //        5    : set zero for ALL enabled wheels
    set_zero_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/antobot/control/wheelsteer/set_zero",
      10,
      std::bind(&WheelSteerControl::onSetZero, this, std::placeholders::_1));

    // Raw command to bridge 
    cmd_pos_raw_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/antobot/control/wheelsteer/cmd_pos_raw",
      50);

    real_pos_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/antobot/control/wheelsteer/real_pos",
      50);
  }

private:

  static double wrap360(double deg)
  {
    double x = std::fmod(deg, 360.0);
    if (x < 0.0) {
      x += 360.0;
    }
    return x;
  }

  void onRawPos(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.empty()) {
      return;
    }

    const size_t n = std::min(msg->data.size(), static_cast<size_t>(wheel_count_));

    // Update raw positions
    for (size_t i = 0; i < n; ++i) {
      raw_pos_deg_[i] = wrap360(msg->data[i]);
    }

    // Compute logical positions with software zero
    for (int i = 0; i < wheel_count_; ++i) {
      logical_pos_deg_[i] = wrap360(raw_pos_deg_[i] - zero_offset_deg_[i]);
    }

    // Publish logical angles
    std_msgs::msg::Float64MultiArray out;
    out.data = logical_pos_deg_;
    real_pos_pub_->publish(out);
  }

  // ======================================================
  // pos command
  // ======================================================
  void onCmdPos(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < static_cast<size_t>(wheel_count_)) {
      RCLCPP_WARN(
        get_logger(),
        "cmd_pos size %zu < wheel_count %d, ignoring.",
        msg->data.size(), wheel_count_);
      return;
    }

    // Prepare raw command array to publish
    std_msgs::msg::Float64MultiArray raw_cmd;
    raw_cmd.data.resize(wheel_count_, 0.0);

    for (int i = 0; i < wheel_count_; ++i) {
      if (!enabled_[i]) {
        // Keep disabled wheels at their current raw position
        raw_cmd.data[i] = raw_pos_deg_[i];
        continue;
      }

      double desired_logical_deg = wrap360(msg->data[i]);
      double target_raw_deg = wrap360(desired_logical_deg + zero_offset_deg_[i]);

      raw_cmd.data[i] = target_raw_deg;

      RCLCPP_INFO(
        get_logger(),
        "Wheel %d: logical_cmd=%.2f deg, zero_offset=%.2f deg -> raw_cmd=%.2f deg",
        i + 1, desired_logical_deg, zero_offset_deg_[i], target_raw_deg);
    }

    cmd_pos_raw_pub_->publish(raw_cmd);
  }

  // ======================================================
  // set-zero command
  // ======================================================
  void onSetZero(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data.empty()) {
      RCLCPP_WARN(get_logger(), "set_zero message has empty data, ignoring.");
      return;
    }

    int wheel_id = msg->data[0];

    if (wheel_id == 5) {
      // Set zero for all enabled wheels
      for (int i = 0; i < wheel_count_; ++i) {
        if (!enabled_[i]) {
          continue;
        }
        zero_offset_deg_[i] = wrap360(raw_pos_deg_[i]);
        RCLCPP_INFO(
          get_logger(),
          "Set zero for wheel %d: raw=%.2f deg -> offset=%.2f deg",
          i + 1, raw_pos_deg_[i], zero_offset_deg_[i]);
      }
      return;
    }

    // Single wheel: ID must be 1..4
    if (wheel_id < 1 || wheel_id > wheel_count_) {
      RCLCPP_WARN(
        get_logger(),
        "Invalid wheel_id %d in set_zero (valid: 1..%d, or 5 for all).",
        wheel_id, wheel_count_);
      return;
    }

    int idx = wheel_id - 1;
    if (!enabled_[idx]) {
      RCLCPP_WARN(
        get_logger(),
        "Wheel %d is disabled, cannot set zero.",
        wheel_id);
      return;
    }

    zero_offset_deg_[idx] = wrap360(raw_pos_deg_[idx]);
    RCLCPP_INFO(
      get_logger(),
      "Set zero for wheel %d: raw=%.2f deg -> offset=%.2f deg",
      wheel_id, raw_pos_deg_[idx], zero_offset_deg_[idx]);
  }

private:
  // Configuration (hard-coded)
  int wheel_count_{4};
  std::vector<bool>   enabled_;          // which wheels are active
  std::vector<double> zero_offset_deg_;  // software zero per wheel

  // Runtime state
  std::vector<double> raw_pos_deg_;      // last raw angle from bridge
  std::vector<double> logical_pos_deg_;  // angle after software zero

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr raw_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr   set_zero_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pos_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr real_pos_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WheelSteerControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
