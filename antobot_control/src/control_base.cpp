#include "control_base.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

ControlBase::ControlBase(const std::string &node_name)
    : rclcpp::Node(node_name)
{
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/antobot/robot/cmd_vel", 10,
        std::bind(&ControlBase::cmd_vel_callback, this, std::placeholders::_1));
    speed_status_sub_ = create_subscription<antobot_platform_msgs::msg::Float32Array>(
            "/antobot/speed/status", 10,
        std::bind(&ControlBase::speed_status_callback, this, std::placeholders::_1));

    wheel_command_pub_ = create_publisher<antobot_platform_msgs::msg::Float32Array>(
            "/antobridge/wheel_vel_cmd", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "/antobot/robot/odometry", 10);
}

void ControlBase::configure_control(const ControlParams &params)
{
    params_ = params;
    actuator_command_.assign(params_.actuator_count, 0.0);
}

void ControlBase::start_control_loop()
{
    const double frequency = params_.control_frequency_hz > 0.0 ? params_.control_frequency_hz : 30.0;
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / frequency),
        std::bind(&ControlBase::control_loop, this));
}

const RobotCommand &ControlBase::command() const { return smoothed_command_; }
const std::vector<double> &ControlBase::speed_feedback() const { return speed_feedback_; }
const OdometryState &ControlBase::odometry() const { return odometry_; }
void ControlBase::on_robot_command(const RobotCommand &) {}
void ControlBase::on_speed_feedback(const std::vector<double> &) {}
bool ControlBase::motion_enabled() const { return true; }

double ControlBase::approach(
    double current, double target, double accel, double decel, double dt)
{
    const bool accelerating =
        std::fabs(target) >= std::fabs(current) && current * target >= 0.0;
    const double limit = (accelerating ? accel : decel) * dt;
    return current + std::clamp(target - current, -limit, limit);
}

void ControlBase::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    raw_command_.linear_x = msg->linear.x;
    raw_command_.linear_y = msg->linear.y;
    raw_command_.angular_z = msg->angular.z;
    last_command_sec_ = now().seconds();
    has_command_ = true;
    on_robot_command(raw_command_);
}

void ControlBase::speed_status_callback(
    const antobot_platform_msgs::msg::Float32Array::SharedPtr msg)
{
    speed_feedback_.assign(msg->data.begin(), msg->data.end());
    on_speed_feedback(speed_feedback_);
    update_odometry(now().seconds());
}

void ControlBase::smooth_command(double now_sec)
{
    double dt = now_sec - last_control_sec_;
    if (last_control_sec_ <= 0.0 || dt <= 0.0)
    {
        dt = 1.0 / std::max(params_.control_frequency_hz, 1.0);
    }
    last_control_sec_ = now_sec;

    RobotCommand target = raw_command_;
    if (params_.enable_timeout &&
        (!has_command_ || now_sec - last_command_sec_ > params_.velocity_timeout_sec))
    {
        target = {};
    }
    target.linear_x = std::clamp(target.linear_x, params_.min_linear, params_.max_linear);
    target.linear_y = std::clamp(target.linear_y, params_.min_linear, params_.max_linear);
    target.angular_z = std::clamp(target.angular_z, params_.min_angular, params_.max_angular);

    if (!params_.enable_smoothing)
    {
        smoothed_command_ = target;
        return;
    }
    smoothed_command_.linear_x = approach(
        smoothed_command_.linear_x, target.linear_x,
        params_.max_linear_accel, params_.max_linear_decel, dt);
    smoothed_command_.linear_y = approach(
        smoothed_command_.linear_y, target.linear_y,
        params_.max_linear_accel, params_.max_linear_decel, dt);
    smoothed_command_.angular_z = approach(
        smoothed_command_.angular_z, target.angular_z,
        params_.max_angular_accel, params_.max_angular_decel, dt);
}

void ControlBase::control_loop()
{
    smooth_command(now().seconds());
    actuator_command_.assign(params_.actuator_count, 0.0);
    if (motion_enabled())
    {
        command_to_actuators(smoothed_command_, actuator_command_);
    }
    antobot_platform_msgs::msg::Float32Array output;
    output.data.reserve(actuator_command_.size());
    for (const double value : actuator_command_)
    {
        output.data.push_back(static_cast<float>(value));
    }
    wheel_command_pub_->publish(output);
    publish_odometry();
}

void ControlBase::update_odometry(double now_sec)
{
    if (!params_.enable_odom)
    {
        return;
    }
    RobotCommand twist;
    if (!feedback_to_body_twist(speed_feedback_, twist))
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
