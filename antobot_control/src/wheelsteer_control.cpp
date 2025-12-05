#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <functional>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Common control base
#include "antobot_control/anto_control_base.hpp"   

using std::placeholders::_1;

// ======================================================================
// Steering geometry / configuration parameters
// ======================================================================
namespace
{
  // Wheelbase (front to rear axle distance) [m]
  constexpr double WHEEL_BASE_M   = 0.9;

  // Track width (left to right wheel distance) [m]
  constexpr double TRACK_WIDTH_M  = 1.2;

  // Base angle for SPOTTURN (deg), other wheels derived from this
  constexpr double SPOTTURN_BASE_DEG = 45.0; //RF1, LR3

  // Base angle for LOCK (deg), other wheels derived from this
  constexpr double LOCK_BASE_DEG     = 45.0; //LF2, RR4

  // Small thresholds for command checking
  constexpr double EPS_V = 1e-3;
  constexpr double EPS_W = 1e-3;

  constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;
}

// ======================================================================

class WheelSteerControl
  : public rclcpp::Node
  , public antobot_control::AntoControlBase
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

    // Initialize common control parameters
    antobot_control::ControlParams params;
    params.wheel_count          = static_cast<std::size_t>(wheel_count_);
    params.track_width          = TRACK_WIDTH_M;
    params.control_frequency_hz = 30.0;
    params.enable_odom          = false;   // odom not used here
    setParams(params);

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

    // Twist command (vx, vy, omega) for CRAB / COUNTERPHASE model
    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/antobot/control/wheelsteer/cmd_vel",
      50,
      std::bind(&WheelSteerControl::onTwistCmd, this, std::placeholders::_1));

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
  // Twist Callback (vx, vy, omega)
  // ======================================================
  void onTwistCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Map geometry_msgs/Twist to common RobotCommand via base helper
    const double now = this->get_clock()->now().seconds();
    robot_cmd_vel_callback_2d(
      msg->linear.x,   // vx
      msg->linear.y,   // vy
      msg->angular.z,  // omega
      now);
    
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

    // wheel positions in body frame: x forward, y left
    // index mapping: 0=RF1, 1=LF2, 2=LR3, 3=RR4 
    const double L = WHEEL_BASE_M;
    const double W = TRACK_WIDTH_M;
    const std::array<std::pair<double,double>, 4> wheel_pos = {{
      { +L * 0.5, -W * 0.5 },  // RF1
      { +L * 0.5, +W * 0.5 },  // LF2
      { -L * 0.5, +W * 0.5 },  // LR3
      { -L * 0.5, -W * 0.5 }   // RR4
    }};

    // Use base class to apply velocity smoothing / timeout
    const double now = this->get_clock()->now().seconds();
    velocity_smoother(now);

    // Fetch smoothed twist (vx, omega) and raw vy from base state
    const auto & st  = getState();
    const auto & cmd = st.smoothed_cmd;
    const double vx  = cmd.linear;              // vx after smoothing / limiting
    const double vy  = st.raw_cmd.linear_y;     // vy (currently passed through)
    const double w   = cmd.angular;             // omega after smoothing / limiting

    // calculate pos cmds with mode called
    switch (current_mode_) {
      case CRAB:
      {
        // CRAB: all wheels aligned with translational velocity (vx, vy)
        const double speed = std::hypot(vx, vy);

        RCLCPP_INFO(
            get_logger(),
            "CRAB mode: vx=%.3f, vy=%.3f, speed=%.6f",
            vx, vy, speed);

        if (speed < EPS_V) {
          // If no clear direction, fall back to incoming cmd_pos
          for (int i = 0; i < wheel_count_; ++i) {
            target_logical_angles[i] = msg->data[i];
          }
        } else {
          const double angle_deg = wrap360(std::atan2(vy, vx) * RAD_TO_DEG);
          for (int i = 0; i < wheel_count_; ++i) {
            target_logical_angles[i] = angle_deg;
          }
        }
        break;
      }

      case COUNTERPHASE:
      {
        // COUNTERPHASE: wheels aligned with combined translational
        // and rotational velocity:
        //   v_i = [vx, vy] + omega × r_i
        // Here we use:
        //   vix = vx + omega * y_i
        //   viy = vy + omega * x_i

        if (std::fabs(vx) < EPS_V && std::fabs(vy) < EPS_V && std::fabs(w) < EPS_W) {
          // No meaningful motion command: fall back to incoming cmd_pos
          for (int i = 0; i < wheel_count_; ++i) {
            target_logical_angles[i] = msg->data[i];
          }
        } else {
          for (int i = 0; i < wheel_count_; ++i) {
            const double x_i = wheel_pos[i].first;
            const double y_i = wheel_pos[i].second;

            const double vix = vx + w * y_i;
            const double viy = vy + w * x_i;

            if (std::fabs(vix) < EPS_V && std::fabs(viy) < EPS_V) {
              // keep current logical if very small
              target_logical_angles[i] = logical_pos_deg_[i];
            } else {
              const double angle_deg = wrap360(std::atan2(viy, vix) * RAD_TO_DEG);
              target_logical_angles[i] = angle_deg;
            }
          }
        }
        break;
      }

      case SPOTTURN:
        // SPOTTURN
        if (wheel_count_ >= 4) {
          // Use a single base angle and derive other wheels
          const double a = SPOTTURN_BASE_DEG;
          target_logical_angles[0] = wrap360(a);              // RF1
          target_logical_angles[1] = wrap360(360.0 - a);      // LF2
          target_logical_angles[2] = wrap360(a);              // LR3
          target_logical_angles[3] = wrap360(360.0 - a);      // RR4
        }
        break;

      case LOCK:

        if (wheel_count_ >= 4) {
          // Use a single base angle and derive other wheels
          const double a = LOCK_BASE_DEG;
          target_logical_angles[0] = wrap360(360.0 - a);      // RF1
          target_logical_angles[1] = wrap360(a);              // LF2
          target_logical_angles[2] = wrap360(360.0 - a);      // LR3
          target_logical_angles[3] = wrap360(a);              // RR4
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

  // ======================================================================
  // wheel_speed_compute (not used in this steering-only node)
  // ======================================================================
  void wheel_speed_compute() override
  {
    // This node only computes steering angles, wheel speeds are handled elsewhere.
  }

  // ======================================================================
  // compute_robot_twist_from_wheels (not used in this node)
  // ======================================================================
  void compute_robot_twist_from_wheels(double & linear, double & angular) override
  {
    // This node does not compute odometry. Return zeros.
    linear  = 0.0;
    angular = 0.0;
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
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr           mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      twist_sub_;

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
