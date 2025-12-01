#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "antobot_platform_msgs/msg/float32_array.hpp"

#include "antobot_control/anto_control_base.hpp"

using std::placeholders::_1;

namespace antobot_control
{

class AntobotControl
  : public rclcpp::Node,
    public AntoControlBase
{
public:
  AntobotControl()
  : rclcpp::Node("antobot_control")
  {
    this->declare_parameter<double>("wheel_base", 0.6);
    this->declare_parameter<double>("wheel_radius", 0.165);
    this->declare_parameter<std::vector<double>>(
      "max_velocity", std::vector<double>{0.5, 0.5});
    this->declare_parameter<std::vector<double>>(
      "min_velocity", std::vector<double>{-0.5, -0.5});
    this->declare_parameter<std::vector<double>>(
      "max_accel", std::vector<double>{0.2, 0.5});
    this->declare_parameter<std::vector<double>>(
      "max_decel", std::vector<double>{-3.0, -3.0});
    this->declare_parameter<double>("frequency", 30.0);
    this->declare_parameter<double>("velocity_timeout", 0.1);

    ControlParams p;
    p.wheel_count = 4;

    p.track_width  = this->get_parameter("wheel_base").as_double();
    p.wheel_radius = this->get_parameter("wheel_radius").as_double();
    p.control_frequency_hz = this->get_parameter("frequency").as_double();
    velocity_timeout_      = this->get_parameter("velocity_timeout").as_double();

    auto max_velocities = this->get_parameter("max_velocity").as_double_array();
    auto min_velocities = this->get_parameter("min_velocity").as_double_array();
    auto max_accels     = this->get_parameter("max_accel").as_double_array();
    auto max_decels     = this->get_parameter("max_decel").as_double_array();

    validate_velocity_limits(
      max_velocities, min_velocities, max_accels, max_decels, p);

    p.enable_smoothing = true;
    p.enable_timeout   = false;  // timeout handled in this node for identical log behavior
    p.enable_odom      = true;

    setParams(p);

    sim_ = true;

    sub_robot_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/antobot/robot/cmd_vel", 10,
      std::bind(&AntobotControl::robot_cmd_vel_ros_cb, this, _1));

    sub_wheel_vel_ = this->create_subscription<antobot_platform_msgs::msg::Float32Array>(
      "/antobot/bridge/wheel_vel", 10,
      std::bind(&AntobotControl::wheel_vel_ros_cb, this, _1));

    pub_wheel_vel_cmd_ = this->create_publisher<antobot_platform_msgs::msg::Float32Array>(
      "/antobridge/wheel_vel_cmd", 10);

    pub_wheel_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/antobot/robot/odometry", 10);

    const double freq = getState().params.control_frequency_hz;
    std::chrono::duration<double> period_sec(
      1.0 / ((freq > 0.0) ? freq : 30.0));

    timer_ = this->create_wall_timer(
      period_sec, std::bind(&AntobotControl::timer_callback, this));

    last_command_time_ = this->now();
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_robot_cmd_vel_;
  rclcpp::Subscription<antobot_platform_msgs::msg::Float32Array>::SharedPtr sub_wheel_vel_;
  rclcpp::Publisher<antobot_platform_msgs::msg::Float32Array>::SharedPtr pub_wheel_vel_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_wheel_odom_;

  rclcpp::Time last_command_time_;
  double velocity_timeout_{0.1};
  bool sim_{true};

  void validate_velocity_limits(
    const std::vector<double> & max_velocities,
    const std::vector<double> & min_velocities,
    const std::vector<double> & max_accels,
    std::vector<double>       & max_decels,
    ControlParams & p)
  {
    if (min_velocities.size() != max_velocities.size()) {
      throw std::runtime_error(
        "Parameter validation failed: 'min_velocity' and 'max_velocity' must have the same size. "
        "Got " + std::to_string(min_velocities.size()) + " and " +
        std::to_string(max_velocities.size()) + ".");
    }

    if (max_accels.size() != max_decels.size()) {
      throw std::runtime_error(
        "Parameter validation failed: 'max_accel' and 'max_decel' must have the same size. "
        "Got " + std::to_string(max_accels.size()) + " and " +
        std::to_string(max_decels.size()) + ".");
    }

    for (std::size_t i = 0; i < min_velocities.size(); ++i) {
      if (max_velocities[i] < 0.0) {
        throw std::runtime_error(
          "Parameter validation failed: 'max_velocitie' component [" +
          std::to_string(i) + "] (" + std::to_string(max_velocities[i]) +
          ") is smaller than 0.");
      }

      if (min_velocities[i] > 0.0) {
        throw std::runtime_error(
          "Parameter validation failed: 'min_velocity' component [" +
          std::to_string(i) + "] (" + std::to_string(min_velocities[i]) +
          ") is greater than 0.");
      }

      if (max_accels[i] < 0.0) {
        throw std::runtime_error(
          "Parameter validation failed: 'max_accels' component [" +
          std::to_string(i) + "] (" + std::to_string(max_accels[i]) +
          ") is smaller than 0.");
      }

      if (max_decels[i] > 0.0) {
        throw std::runtime_error(
          "Parameter validation failed: 'max_decel' component [" +
          std::to_string(i) + "] (" + std::to_string(max_decels[i]) +
          ") is greater than 0.");
      }
    }

    for (std::size_t i = 0; i < max_decels.size(); ++i) {
      if (max_decels[i] > -3.0) {
        RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Parameter validation failed: 'max_decel' component [" << i << "] (" <<
          max_decels[i] << ") is greater than -3.0.");
        max_decels[i] = -3.0;
      }
    }

    p.max_linear  = max_velocities[0];
    p.min_linear  = min_velocities[0];
    p.max_angular = max_velocities[1];
    p.min_angular = min_velocities[1];

    p.max_linear_accel  = max_accels[0];
    p.max_linear_decel  = max_decels[0];
    p.max_angular_accel = max_accels[1];
    p.max_angular_decel = max_decels[1];
  }

  void robot_cmd_vel_ros_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double now = this->now().seconds();
    robot_cmd_vel_callback(msg->linear.x, msg->angular.z, now);
    last_command_time_ = this->now();
  }

  void wheel_vel_ros_cb(const antobot_platform_msgs::msg::Float32Array::SharedPtr msg)
  {
    const double now = this->now().seconds();

    if (msg->data.size() < 4) {
      return;
    }

    std::vector<double> measured(msg->data.begin(), msg->data.end());
    wheel_cmd_vel_callback(measured, now);
  }

  void timer_callback()
  {
    const double now_sec = this->now().seconds();

    velocity_smoother(now_sec);

    auto & st = getState();

    double time_since_cmd = (this->now() - last_command_time_).seconds();
    if (time_since_cmd > velocity_timeout_) {
      if (st.smoothed_cmd.linear != 0.0 || st.smoothed_cmd.angular != 0.0) {
        RCLCPP_WARN(
          this->get_logger(),
          "No cmd_vel received for %.3f s, stop robot.",
          time_since_cmd);
      }

      st.raw_cmd.linear     = 0.0;
      st.raw_cmd.angular    = 0.0;
      st.clipped_cmd.linear = 0.0;
      st.clipped_cmd.angular = 0.0;
      st.smoothed_cmd.linear = 0.0;
      st.smoothed_cmd.angular = 0.0;
    }

    wheel_speed_compute();

    robot_odom_compute(now_sec);

    antobot_platform_msgs::msg::Float32Array wheel_cmd_msg;
    wheel_cmd_msg.data.reserve(st.wheel_cmd.target_speed.size());
    for (double v : st.wheel_cmd.target_speed) {
      wheel_cmd_msg.data.push_back(static_cast<float>(v));
    }
    pub_wheel_vel_cmd_->publish(wheel_cmd_msg);

    nav_msgs::msg::Odometry odom_msg;
    fill_odom_msg_from_state(odom_msg);
    pub_wheel_odom_->publish(odom_msg);
  }

  void fill_odom_msg_from_state(nav_msgs::msg::Odometry & msg)
  {
    const auto & st = getState();
    const auto & od = st.odom;

    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id  = "base_link";

    msg.pose.pose.position.x = od.x;
    msg.pose.pose.position.y = od.y;
    msg.pose.pose.position.z = 0.0;

    const double half_yaw = od.yaw * 0.5;
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = std::sin(half_yaw);
    msg.pose.pose.orientation.w = std::cos(half_yaw);

    std::array<double, 36> pose_cov = {
      0.001, 0.0,   0.0,   0.0,   0.0,   0.0,
      0.0,   0.001, 0.0,   0.0,   0.0,   0.0,
      0.0,   0.0,   0.001, 0.0,   0.0,   0.0,
      0.0,   0.0,   0.0,   0.001, 0.0,   0.0,
      0.0,   0.0,   0.0,   0.0,   0.001, 0.0,
      0.0,   0.0,   0.0,   0.0,   0.0,   0.03
    };
    std::array<double, 36> twist_cov = pose_cov;

    for (int i = 0; i < 36; ++i) {
      msg.pose.covariance[i]  = pose_cov[i];
      msg.twist.covariance[i] = twist_cov[i];
    }

    msg.twist.twist.linear.x  = od.linear;
    msg.twist.twist.linear.y  = 0.0;
    msg.twist.twist.linear.z  = 0.0;
    msg.twist.twist.angular.x = 0.0;
    msg.twist.twist.angular.y = 0.0;
    msg.twist.twist.angular.z = od.angular;
  }

  void wheel_speed_compute() override
  {
    auto & st = getState();
    const auto & p = st.params;

    const double lin = st.smoothed_cmd.linear;
    const double ang = st.smoothed_cmd.angular;

    const double W = p.track_width;
    const double r = p.wheel_radius;

    const double vL = lin - ang * W * 0.5;
    const double vR = lin + ang * W * 0.5;

    const double wL = (r > 1e-6) ? (vL / r) : 0.0;
    const double wR = (r > 1e-6) ? (vR / r) : 0.0;

    st.wheel_cmd.target_speed.resize(4);
    st.wheel_cmd.target_speed[0] = wL;
    st.wheel_cmd.target_speed[1] = wL;
    st.wheel_cmd.target_speed[2] = wR;
    st.wheel_cmd.target_speed[3] = wR;
  }

  void compute_robot_twist_from_wheels(double & linear, double & angular) override
  {
    auto & st = getState();
    const auto & p = st.params;

    double omega_L = 0.0;
    double omega_R = 0.0;

    if (sim_) {
      if (st.wheel_cmd.target_speed.size() >= 4) {
        omega_L = 0.5 * (st.wheel_cmd.target_speed[0] +
                         st.wheel_cmd.target_speed[1]);
        omega_R = 0.5 * (st.wheel_cmd.target_speed[2] +
                         st.wheel_cmd.target_speed[3]);
      }
    } else {
      const auto & fb = st.wheel_feedback;
      if (!fb.valid || fb.measured_speed.size() < 4) {
        linear  = 0.0;
        angular = 0.0;
        return;
      }
      omega_L = 0.5 * (fb.measured_speed[0] + fb.measured_speed[1]);
      omega_R = 0.5 * (fb.measured_speed[2] + fb.measured_speed[3]);
    }

    const double r = p.wheel_radius;
    const double W = p.track_width;

    linear = r * (omega_L + omega_R) * 0.5;

    if (std::fabs(W) > 1e-6) {
      angular = (omega_L - omega_R) * r / W;
    } else {
      angular = 0.0;
    }
  }
};

}  // namespace antobot_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<antobot_control::AntobotControl>());
  rclcpp::shutdown();
  return 0;
}
