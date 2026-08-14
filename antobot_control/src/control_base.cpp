#include "control_base.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

namespace
{
    double approach(double current, double target, double accel, double decel, double dt)
    {
        const bool accelerating =
            std::fabs(target) >= std::fabs(current) && current * target >= 0.0;
        const double limit = (accelerating ? accel : decel) * dt;
        return current + std::clamp(target - current, -limit, limit);
    }
}

ControlBase::ControlBase(
    const std::string &node_name,
    const ControlConfig &default_config)
    : rclcpp::Node(node_name)
{
    declare_parameter<std::string>("robot_role", default_config.robot_role);
    declare_parameter<double>("frequency", default_config.frequency_hz);
    declare_parameter<double>("velocity_timeout", default_config.velocity_timeout_sec);
    declare_parameter<bool>("enable_smoothing", default_config.enable_smoothing);
    declare_parameter<bool>("enable_timeout", default_config.enable_timeout);
    declare_parameter<bool>("enable_odom", default_config.enable_odom);

    declare_parameter<std::vector<double>>(
        "min_velocity", {default_config.min_linear, default_config.min_angular});
    declare_parameter<std::vector<double>>(
        "max_velocity", {default_config.max_linear, default_config.max_angular});
    declare_parameter<std::vector<double>>(
        "max_accel", {default_config.max_linear_accel, default_config.max_angular_accel});
    declare_parameter<std::vector<double>>(
        "max_decel", {-default_config.max_linear_decel, -default_config.max_angular_decel});

    config_.robot_role = get_parameter("robot_role").as_string();
    config_.frequency_hz = get_parameter("frequency").as_double();
    config_.velocity_timeout_sec = get_parameter("velocity_timeout").as_double();
    config_.enable_smoothing = get_parameter("enable_smoothing").as_bool();
    config_.enable_timeout = get_parameter("enable_timeout").as_bool();
    config_.enable_odom = get_parameter("enable_odom").as_bool();

    const auto min_velocity = get_parameter("min_velocity").as_double_array();
    const auto max_velocity = get_parameter("max_velocity").as_double_array();
    const auto max_accel = get_parameter("max_accel").as_double_array();
    const auto max_decel = get_parameter("max_decel").as_double_array();

    config_.min_linear = min_velocity[0];
    config_.min_angular = min_velocity[1];
    config_.max_linear = max_velocity[0];
    config_.max_angular = max_velocity[1];
    config_.max_linear_accel = std::fabs(max_accel[0]);
    config_.max_angular_accel = std::fabs(max_accel[1]);
    config_.max_linear_decel = std::fabs(max_decel[0]);
    config_.max_angular_decel = std::fabs(max_decel[1]);

    RCLCPP_INFO_STREAM(
        get_logger(),
        "Control parameters loaded:"
            << " robot_role=" << config_.robot_role
            << ", frequency=" << config_.frequency_hz
            << ", velocity_timeout=" << config_.velocity_timeout_sec
            << ", enable_smoothing=" << std::boolalpha << config_.enable_smoothing
            << ", enable_timeout=" << config_.enable_timeout
            << ", enable_odom=" << config_.enable_odom
            << ", min_velocity=[" << config_.min_linear << ", " << config_.min_angular << "]"
            << ", max_velocity=[" << config_.max_linear << ", " << config_.max_angular << "]"
            << ", max_accel=[" << config_.max_linear_accel << ", " << config_.max_angular_accel << "]"
            << ", max_decel=[" << config_.max_linear_decel << ", " << config_.max_angular_decel << "]");

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/antobot/robot/cmd_vel", 10,
        std::bind(&ControlBase::cmd_vel_callback, this, std::placeholders::_1));
    speed_status_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/antobot/track/status", 10,
        std::bind(&ControlBase::speed_status_callback, this, std::placeholders::_1));

    speed_cmd_pub_ = create_publisher<antobot_platform_msgs::msg::Float32Array>(
        "/antobridge/wheel_vel_cmd", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "/antobot/robot/odometry", 10);

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / config_.frequency_hz),
        std::bind(&ControlBase::control_loop, this));
}

const SpeedCmd &ControlBase::command() const
{
    return smoothed_cmd_twist_;
}

const ControlConfig &ControlBase::config() const
{
    return config_;
}

bool ControlBase::motion_enabled() const
{ 
    return true;
}

void ControlBase::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    speed_cmd_.linear_x = msg->linear.x;
    speed_cmd_.linear_y = msg->linear.y;
    speed_cmd_.angular_z = msg->angular.z;
    last_command_sec_ = now().seconds();
    has_command_ = true;
    
    on_robot_command(speed_cmd_);
}

void ControlBase::speed_status_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < speed_feedback_.size())
    {
        RCLCPP_WARN(get_logger(), "Speed feedback requires four elements, got %zu", msg->data.size());
        return;
    }

    speed_feedback_[0] = msg->data[2];
    speed_feedback_[1] = msg->data[2];
    speed_feedback_[2] = msg->data[3];
    speed_feedback_[3] = msg->data[3];

    update_odometry();
}

void ControlBase::smooth_command()
{
    double now_sec = now().seconds();

    double dt = now_sec - last_control_sec_;
    if (last_control_sec_ <= 0.0 || dt <= 0.0)
    {
        dt = 1.0 / config_.frequency_hz;
    }
    last_control_sec_ = now_sec;

    SpeedCmd target = speed_cmd_;
    if (config_.enable_timeout &&
        (!has_command_ || now_sec - last_command_sec_ > config_.velocity_timeout_sec))
    {
        target = {};
    }
    target.linear_x = std::clamp(target.linear_x, -1.0, 1.0) * config_.max_linear;
    target.linear_y = std::clamp(target.linear_y, -1.0, 1.0) * config_.max_linear;
    target.angular_z = std::clamp(target.angular_z, -1.0, 1.0) * config_.max_angular;

    if (!config_.enable_smoothing)
    {
        smoothed_cmd_twist_ = target;
        return;
    }
    
    smoothed_cmd_twist_.linear_x = approach(
        smoothed_cmd_twist_.linear_x, target.linear_x,
        config_.max_linear_accel, config_.max_linear_decel, dt);

    smoothed_cmd_twist_.linear_y = approach(
        smoothed_cmd_twist_.linear_y, target.linear_y,
        config_.max_linear_accel, config_.max_linear_decel, dt);

    smoothed_cmd_twist_.angular_z = approach(
        smoothed_cmd_twist_.angular_z, target.angular_z,
        config_.max_angular_accel, config_.max_angular_decel, dt);
}

void ControlBase::control_loop()
{
    smooth_command();
    
    speed_cmd_rpm_.fill(0.0);
    
    if (motion_enabled())
    {
        twist_to_rpm(smoothed_cmd_twist_, speed_cmd_rpm_);
    }

    publish_speed_cmd();

    publish_odometry();
}

void ControlBase::update_odometry()
{
    double now_sec = now().seconds();

    if (!config_.enable_odom)
    {
        return;
    }
    SpeedCmd twist;
    if (!rpm_to_twist(speed_feedback_, twist))
    {
        return;
    }
    if (odometry_.last_update_sec <= 0.0)
    {
        odometry_.last_update_sec = now_sec;
        return;
    }
    const double dt = now_sec - odometry_.last_update_sec;
    odometry_.last_update_sec = now_sec;
    if (dt <= 0.0)
    {
        return;
    }

    odometry_.linear_x = twist.linear_x;
    odometry_.linear_y = twist.linear_y;
    odometry_.angular_z = twist.angular_z;
    const double c = std::cos(odometry_.yaw);
    const double s = std::sin(odometry_.yaw);
    odometry_.x += (twist.linear_x * c - twist.linear_y * s) * dt;
    odometry_.y += (twist.linear_x * s + twist.linear_y * c) * dt;
    odometry_.yaw = std::atan2(
        std::sin(odometry_.yaw + twist.angular_z * dt),
        std::cos(odometry_.yaw + twist.angular_z * dt));
}

void ControlBase::publish_speed_cmd()
{
    antobot_platform_msgs::msg::Float32Array speed_cmd_msg;

    speed_cmd_msg.data.reserve(speed_cmd_rpm_.size());
    for (const double value : speed_cmd_rpm_)
    {
        speed_cmd_msg.data.push_back(static_cast<float>(value));
    }

    speed_cmd_pub_->publish(speed_cmd_msg);
}

void ControlBase::publish_odometry()
{
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = odometry_.x;
    msg.pose.pose.position.y = odometry_.y;
    msg.pose.pose.orientation.z = std::sin(odometry_.yaw * 0.5);
    msg.pose.pose.orientation.w = std::cos(odometry_.yaw * 0.5);
    msg.twist.twist.linear.x = odometry_.linear_x;
    msg.twist.twist.linear.y = odometry_.linear_y;
    msg.twist.twist.angular.z = odometry_.angular_z;
    odom_pub_->publish(msg);
}
