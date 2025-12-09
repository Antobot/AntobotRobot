#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <functional>
#include <array>
#include <chrono>  

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "antobot_platform_msgs/msg/float32_array.hpp"   // Wheel velocity command

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

  // Default steering angle for CRAB when no clear direction or on mode entry [deg]
  constexpr double CRAB_INIT_ANGLE_DEG = 0.0;

  // Default steering angle for COUNTERPHASE when no clear direction or on mode entry [deg]
  constexpr double COUNTERPHASE_INIT_ANGLE_DEG = 0.0;

  // Small thresholds for command checking
  constexpr double EPS_V = 1e-3;
  constexpr double EPS_W = 1e-3;

  // Threshold for deciding if a new steering target is significantly different [deg]
  constexpr double STEER_RETARGET_TOL_DEG = 0.0;

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

  // Steering high-level state
  enum SteerState {
    TRANSITIONING = 0,  // steering is moving to target, wheel speeds forced to zero
    ACTIVE        = 1   // steering is aligned, wheel speeds allowed
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

    // Initialize steering state machine
    steer_state_ = TRANSITIONING;
    transition_tol_deg_ = 2.0;
    target_logical_angles_.assign(wheel_count_, 0.0);
    has_target_ = false;

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

    // steering command (manual override, not required in normal operation)
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

    // Wheel velocity command publisher
    pub_wheel_vel_cmd_ = this->create_publisher<antobot_platform_msgs::msg::Float32Array>(
      "/antobridge/wheel_vel_cmd",
      10);

    {
      // Initial steering command for the default mode
      std::vector<double> init_target(wheel_count_, 0.0);
      computeTargetAnglesForCurrentMode(
        init_target,
        /*use_preset_for_travel_modes=*/true);
      publishSteeringTargets(init_target);
      RCLCPP_INFO(
        get_logger(),
        "Initial steering command published for mode %d",
        static_cast<int>(current_mode_));
    }

    // ------------------------------------------------------------------
    // Periodic control loop timer for wheel speed computation
    // ------------------------------------------------------------------
    const double dt = 1.0 / params.control_frequency_hz;
    control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt),
      [this]()
      {
        this->wheel_speed_compute();
      });
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

  // Compute steering target angles for the current mode using the internal state.
  // If use_preset_for_travel_modes is true, CRAB/COUNTERPHASE use default preset angles
  // instead of deriving from vx, vy, omega.
  void computeTargetAnglesForCurrentMode(std::vector<double> & target_logical_angles,
                                         bool use_preset_for_travel_modes)
  {
    target_logical_angles.assign(wheel_count_, 0.0);

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

    const auto & st  = getState();
    // Use raw command (before smoothing) for steering geometry
    const auto & raw = st.raw_cmd;
    const double vx  = raw.linear;    // raw vx
    const double vy  = raw.linear_y;  // raw vy
    const double w   = raw.angular;   // raw omega

    switch (current_mode_) {
      case CRAB:
      {
        // CRAB: all wheels aligned with translational velocity (vx, vy)
        if (use_preset_for_travel_modes) {
          const double angle_deg = wrap360(CRAB_INIT_ANGLE_DEG);
          for (int i = 0; i < wheel_count_; ++i) {
            target_logical_angles[i] = angle_deg;
          }
        } else {
          const double speed = std::hypot(vx, vy);

          RCLCPP_INFO(
              get_logger(),
              "CRAB mode: vx=%.3f, vy=%.3f, speed=%.6f",
              vx, vy, speed);

          if (speed < EPS_V) {
            // If no clear direction, use preset CRAB angle
            const double angle_deg = wrap360(CRAB_INIT_ANGLE_DEG);
            for (int i = 0; i < wheel_count_; ++i) {
              target_logical_angles[i] = angle_deg;
            }
          } else {
            const double angle_deg = wrap360(std::atan2(vy, vx) * RAD_TO_DEG);
            for (int i = 0; i < wheel_count_; ++i) {
              target_logical_angles[i] = angle_deg;
            }
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

        if (use_preset_for_travel_modes) {
          // Use preset COUNTERPHASE angle when entering mode
          const double angle_deg = wrap360(COUNTERPHASE_INIT_ANGLE_DEG);
          for (int i = 0; i < wheel_count_; ++i) {
            target_logical_angles[i] = angle_deg;
          }
        } else {
          if (std::fabs(vx) < EPS_V && std::fabs(vy) < EPS_V && std::fabs(w) < EPS_W) {
            // No meaningful motion command: use preset COUNTERPHASE angle
            const double angle_deg = wrap360(COUNTERPHASE_INIT_ANGLE_DEG);
            for (int i = 0; i < wheel_count_; ++i) {
              target_logical_angles[i] = angle_deg;
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
        }
        break;
      }

      case SPOTTURN:
      {
        // SPOTTURN fixed geometry
        if (wheel_count_ >= 4) {
          const double a = SPOTTURN_BASE_DEG;
          target_logical_angles[0] = wrap360(a);              // RF1
          target_logical_angles[1] = wrap360(360.0 - a);      // LF2
          target_logical_angles[2] = wrap360(a);              // LR3
          target_logical_angles[3] = wrap360(360.0 - a);      // RR4
        }
        break;
      }

      case LOCK:
      {
        // LOCK fixed geometry
        if (wheel_count_ >= 4) {
          const double a = LOCK_BASE_DEG;
          target_logical_angles[0] = wrap360(360.0 - a);      // RF1
          target_logical_angles[1] = wrap360(a);              // LF2
          target_logical_angles[2] = wrap360(360.0 - a);      // LR3
          target_logical_angles[3] = wrap360(a);              // RR4
        }
        break;
      }

      default:
      {
        // Default: keep current logical angles
        for (int i = 0; i < wheel_count_; ++i) {
          target_logical_angles[i] = logical_pos_deg_[i];
        }
        break;
      }
    }
  }

  // Check whether a new target set is significantly different from the current one.
  bool steeringTargetChangedSignificantly(const std::vector<double> & new_target) const
  {
    if (!has_target_ || new_target.size() != target_logical_angles_.size()) {
      return true;
    }

    for (int i = 0; i < wheel_count_; ++i) {
      if (!enabled_[i]) {
        continue;
      }
      double err = std::fabs(wrap360(new_target[i] - target_logical_angles_[i]));
      if (err > 180.0) {
        err = 360.0 - err;
      }
      if (err > STEER_RETARGET_TOL_DEG) {
        return true;
      }
    }
    return false;
  }

  // Publish steering raw targets and update state machine.
  void publishSteeringTargets(const std::vector<double> & target_logical_angles)
  {
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

    // Update target angles for state machine and enter transitioning
    target_logical_angles_ = target_logical_angles;
    has_target_ = true;
    steer_state_ = TRANSITIONING;
  }

  // ======================================================
  // Mode Callback
  // ======================================================
  void onModeCmd(const std_msgs::msg::Int32::SharedPtr msg)
  {
    int req_mode = msg->data;
    if (req_mode >= 0 && req_mode <= 3) {
      ControlMode new_mode = static_cast<ControlMode>(req_mode);
      if (new_mode != current_mode_) {
        current_mode_ = new_mode;
        RCLCPP_INFO(get_logger(), "Mode switched to: %d", current_mode_);

        // Enter transitioning state on mode change and immediately compute
        // steering targets for this mode using preset angles (if applicable).
        steer_state_ = TRANSITIONING;
        has_target_ = false;

        std::vector<double> target_logical_angles(wheel_count_, 0.0);
        // For CRAB/COUNTERPHASE we use preset angles on mode entry
        computeTargetAnglesForCurrentMode(target_logical_angles, /*use_preset_for_travel_modes=*/true);
        publishSteeringTargets(target_logical_angles);
      } else {
        RCLCPP_INFO(get_logger(), "Mode requested is same as current: %d", current_mode_);
      }
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

    // Update smoothed command for wheel speed control
    velocity_smoother(now);

    // For CRAB/COUNTERPHASE, compute steering targets using raw command
    if (current_mode_ == CRAB || current_mode_ == COUNTERPHASE) {
      std::vector<double> target_logical_angles(wheel_count_, 0.0);
      computeTargetAnglesForCurrentMode(target_logical_angles, /*use_preset_for_travel_modes=*/false);

      if (steeringTargetChangedSignificantly(target_logical_angles)) {
        publishSteeringTargets(target_logical_angles);
      }
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

    // State machine: if transitioning, check whether all steering angles reached target
    if (steer_state_ == TRANSITIONING && has_target_) {
      double max_err = 1.0;
      for (int i = 0; i < wheel_count_; ++i) {
        if (!enabled_[i]) {
          continue;
        }
        double err = std::fabs(wrap360(logical_pos_deg_[i] - target_logical_angles_[i]));
        if (err > 180.0) {
          err = 360.0 - err;
        }
        if (err > max_err) {
          max_err = err;
        }
      }

      if (max_err <= transition_tol_deg_) {
        steer_state_ = ACTIVE;
        RCLCPP_INFO(
          get_logger(),
          "Steering transition finished. Max error=%.2f deg. State -> ACTIVE",
          max_err);
      }
    }
  }

  // ======================================================
  // Command Callback  (manual override, optional)
  // ======================================================
  void onCmdPos(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < static_cast<size_t>(wheel_count_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
        "cmd_pos size mismatch. Expected %d", wheel_count_);
      return;
    }

    // Manual override: directly use incoming logical angles as targets
    std::vector<double> target_logical_angles(wheel_count_, 0.0);
    for (int i = 0; i < wheel_count_; ++i) {
      target_logical_angles[i] = wrap360(msg->data[i]);
    }

    publishSteeringTargets(target_logical_angles);

    RCLCPP_INFO(
      get_logger(),
      "Manual cmd_pos override received and applied as steering targets.");
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
  // wheel_speed_compute
  // ======================================================================
  void wheel_speed_compute() override
  {
    // If steering is still transitioning, force all wheel speeds to zero.
    if (steer_state_ == TRANSITIONING) {
      antobot_platform_msgs::msg::Float32Array wheel_cmd;
      wheel_cmd.data.resize(wheel_count_, 0.0f);
      pub_wheel_vel_cmd_->publish(wheel_cmd);
      return;
    }

    // ACTIVE state: compute wheel speeds from smoothed command and current steering angles.
    const auto & st  = getState();
    const auto & cmd = st.smoothed_cmd;
    const double vx  = cmd.linear;    // smoothed vx
    const double vy  = cmd.linear_y;  // smoothed vy
    const double w   = cmd.angular;   // smoothed omega

    antobot_platform_msgs::msg::Float32Array wheel_cmd;
    wheel_cmd.data.resize(wheel_count_, 0.0f);

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

    for (int i = 0; i < wheel_count_; ++i) {
      if (!enabled_[i]) {
        wheel_cmd.data[i] = 0.0f;
        continue;
      }

      const double x_i = wheel_pos[i].first;
      const double y_i = wheel_pos[i].second;

      // Linear velocity of the wheel center in body frame
      const double vix = vx + w * y_i;
      const double viy = vy + w * x_i;

      // Wheel steering direction in body frame
      const double theta_rad = logical_pos_deg_[i] / RAD_TO_DEG;
      const double ex = std::cos(theta_rad);
      const double ey = std::sin(theta_rad);

      // Project wheel center velocity onto wheel steering direction
      double v_long = vix * ex + viy * ey;

      // Small deadband to avoid tiny residual velocities
      if (std::fabs(v_long) < 1e-4) {
        v_long = 0.0;
      }

      wheel_cmd.data[i] = static_cast<float>(v_long);
    }

    pub_wheel_vel_cmd_->publish(wheel_cmd);
  }

  // ======================================================================
  // compute_robot_twist_from_wheels (not used in this node)
  // ======================================================================
  void compute_robot_twist_from_wheels(double & linear, double & angular) override
  {
    // This node does not compute odometry yet. Return zeros.
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

  // Steering state machine members
  SteerState steer_state_;
  std::vector<double> target_logical_angles_;
  bool has_target_;
  double transition_tol_deg_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr raw_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr set_zero_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr           mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      twist_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pos_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr real_pos_pub_;
  rclcpp::Publisher<antobot_platform_msgs::msg::Float32Array>::SharedPtr pub_wheel_vel_cmd_;

  // Periodic control loop timer
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WheelSteerControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
