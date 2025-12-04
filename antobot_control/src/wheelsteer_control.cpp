#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp" 

class WheelSteerControl : public rclcpp::Node
{
public:
  // four control mode
  enum ControlMode {
    CRAB = 0,         
    COUNTERPHASE = 1, 
    SPOTTURN = 2,     // defined angles
    LOCK = 3          // defined angles
  };

  WheelSteerControl()
  : Node("wheelsteer_control")
  {
    wheel_count_ = 4;
    enabled_ = {true, true, true, true};
    zero_offset_deg_.assign(wheel_count_, 0.0);
    raw_pos_deg_.assign(wheel_count_, 0.0);
    logical_pos_deg_.assign(wheel_count_, 0.0);
    
    // Lock as initial mode
    current_mode_ = LOCK; 

    RCLCPP_INFO(
      get_logger(),
      "WheelSteerControl started. Wheels: %d. Default Mode: %d",
      wheel_count_, current_mode_);

    // Raw feedback from bridge
    raw_pos_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/antobot/control/wheelsteer/real_pos_raw",
      50,
      std::bind(&WheelSteerControl::onRawPos, this, std::placeholders::_1));

    // steering command
    cmd_pos_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/antobot/control/wheelsteer/cmd_pos",
      50,
      std::bind(&WheelSteerControl::onCmdPos, this, std::placeholders::_1));

    // Set-zero command
    set_zero_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/antobot/control/wheelsteer/set_zero",
      10,
      std::bind(&WheelSteerControl::onSetZero, this, std::placeholders::_1));

    // Mode command
    mode_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/antobot/control/wheelsteer/mode",
      10,
      std::bind(&WheelSteerControl::onModeCmd, this, std::placeholders::_1));

    // Publishers
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

  // ======================================================
  // Mode Callback
  // ======================================================
  void onModeCmd(const std_msgs::msg::Int32::SharedPtr msg)
  {
    int req_mode = msg->data;
    if (req_mode >= 0 && req_mode <= 3) {
      current_mode_ = static_cast<ControlMode>(req_mode);
      RCLCPP_INFO(get_logger(), "Mode switched to: %d", current_mode_);
    } else {
      RCLCPP_WARN(get_logger(), "Invalid mode request: %d", req_mode);
    }
  }

  // ======================================================
  // Feedback Callback
  // ======================================================
  void onRawPos(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.empty()) return;

    const size_t n = std::min(msg->data.size(), static_cast<size_t>(wheel_count_));

    for (size_t i = 0; i < n; ++i) {
      raw_pos_deg_[i] = wrap360(msg->data[i]);
    }

    for (int i = 0; i < wheel_count_; ++i) {
      logical_pos_deg_[i] = wrap360(raw_pos_deg_[i] - zero_offset_deg_[i]);
    }

    std_msgs::msg::Float64MultiArray out;
    out.data = logical_pos_deg_;
    real_pos_pub_->publish(out);
  }

  // ======================================================
  // Command Callback 
  // ======================================================
  void onCmdPos(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {

    if (msg->data.size() < static_cast<size_t>(wheel_count_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
        "cmd_pos size mismatch. Expected %d", wheel_count_);
      return;
    }


    std::vector<double> target_logical_angles(wheel_count_, 0.0);

    // calculate pos cmds with mode called
    switch (current_mode_) {
      case CRAB:
        // TODO: add CRAB logic
        for(int i=0; i<wheel_count_; i++) target_logical_angles[i] = msg->data[i];
        break;

      case COUNTERPHASE:
        // TODO: add COUNTERPHASE logic
        for(int i=0; i<wheel_count_; i++) target_logical_angles[i] = msg->data[i];
        break;

      case SPOTTURN:
        // SPOTTURN
        if (wheel_count_ >= 4) {
          target_logical_angles[0] =  315.0;// LF
          target_logical_angles[1] =   45.0;// RF
          target_logical_angles[2] =   45.0;// LR
          target_logical_angles[3] =  315.0;// RR
        }
        break;

      case LOCK:

        if (wheel_count_ >= 4) {
          target_logical_angles[0] =   45.0; // LF
          target_logical_angles[1] =  315.0; // RF
          target_logical_angles[2] =  315.0; // LR
          target_logical_angles[3] =   45.0; // RR
        }
        break;
        
      default:
        for(int i=0; i<wheel_count_; i++) target_logical_angles[i] = msg->data[i];
        break;
    }

    std_msgs::msg::Float64MultiArray raw_cmd;
    raw_cmd.data.resize(wheel_count_, 0.0);

    for (int i = 0; i < wheel_count_; ++i) {
      if (!enabled_[i]) {
        raw_cmd.data[i] = raw_pos_deg_[i];
        continue;
      }

      double desired_logical_deg = wrap360(target_logical_angles[i]);

      double target_raw_deg = wrap360(desired_logical_deg + zero_offset_deg_[i]);

      raw_cmd.data[i] = target_raw_deg;

      RCLCPP_DEBUG(
        get_logger(),
        "Wheel %d [Mode %d]: logical=%.2f -> raw=%.2f",
        i + 1, current_mode_, desired_logical_deg, target_raw_deg);
    }

    cmd_pos_raw_pub_->publish(raw_cmd);
  }

  // ======================================================
  // Set Zero Callback
  // ======================================================
  void onSetZero(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data.empty()) return;
    int wheel_id = msg->data[0];

    if (wheel_id == 5) {
      for (int i = 0; i < wheel_count_; ++i) {
        if (!enabled_[i]) continue;
        zero_offset_deg_[i] = wrap360(raw_pos_deg_[i]);
        RCLCPP_INFO(get_logger(), "Set zero W%d: offset=%.2f", i + 1, zero_offset_deg_[i]);
      }
      return;
    }

    if (wheel_id < 1 || wheel_id > wheel_count_) return;
    int idx = wheel_id - 1;
    if (!enabled_[idx]) return;

    zero_offset_deg_[idx] = wrap360(raw_pos_deg_[idx]);
    RCLCPP_INFO(get_logger(), "Set zero W%d: offset=%.2f", wheel_id, zero_offset_deg_[idx]);
  }

private:
  int wheel_count_{4};
  std::vector<bool> enabled_;
  std::vector<double> zero_offset_deg_;
  std::vector<double> raw_pos_deg_;
  std::vector<double> logical_pos_deg_;


  ControlMode current_mode_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr raw_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr set_zero_sub_;
  
  // Mode sub
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;

  // Publishers
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