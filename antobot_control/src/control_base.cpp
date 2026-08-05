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
    const auto &parameter_overrides = get_node_options().parameter_overrides();
    const auto is_overridden = [&parameter_overrides](const std::string &name)
    {
        return std::any_of(
            parameter_overrides.begin(), parameter_overrides.end(),
            [&name](const rclcpp::Parameter &parameter)
            {
                return parameter.get_name() == name;
            });
    };

    declare_parameter<std::string>("robot_role", default_config.robot_role);
    declare_parameter<double>("frequency", default_config.frequency_hz);
    declare_parameter<double>("velocity_timeout", default_config.velocity_timeout_sec);
    declare_parameter<double>("min_linear", default_config.min_linear);
    declare_parameter<double>("max_linear", default_config.max_linear);
    declare_parameter<double>("min_angular", default_config.min_angular);
    declare_parameter<double>("max_angular", default_config.max_angular);
    declare_parameter<double>("max_linear_accel", default_config.max_linear_accel);
    declare_parameter<double>("max_linear_decel", default_config.max_linear_decel);
    declare_parameter<double>("max_angular_accel", default_config.max_angular_accel);
    declare_parameter<double>("max_angular_decel", default_config.max_angular_decel);
    declare_parameter<bool>("enable_smoothing", default_config.enable_smoothing);
    declare_parameter<bool>("enable_timeout", default_config.enable_timeout);
    declare_parameter<bool>("enable_odom", default_config.enable_odom);

    // Backward-compatible parameters used by control_config stored in the database.
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
    config_.min_linear = get_parameter("min_linear").as_double();
    config_.max_linear = get_parameter("max_linear").as_double();
    config_.min_angular = get_parameter("min_angular").as_double();
    config_.max_angular = get_parameter("max_angular").as_double();
    config_.max_linear_accel = get_parameter("max_linear_accel").as_double();
    config_.max_linear_decel = get_parameter("max_linear_decel").as_double();
    config_.max_angular_accel = get_parameter("max_angular_accel").as_double();
    config_.max_angular_decel = get_parameter("max_angular_decel").as_double();
    config_.enable_smoothing = get_parameter("enable_smoothing").as_bool();
    config_.enable_timeout = get_parameter("enable_timeout").as_bool();
    config_.enable_odom = get_parameter("enable_odom").as_bool();

    const auto apply_legacy_pair = [this, &is_overridden](
        const std::string &legacy_name,
        const std::string &linear_name,
        const std::string &angular_name,
        double &linear_value,
        double &angular_value,
        bool use_absolute_value)
    {
        if (!is_overridden(legacy_name))
        {
            return;
        }

        const auto values = get_parameter(legacy_name).as_double_array();
        if (values.size() < 2)
        {
            RCLCPP_WARN(
                get_logger(), "Parameter '%s' requires two elements; got %zu",
                legacy_name.c_str(), values.size());
            return;
        }

        if (!is_overridden(linear_name))
        {
            linear_value = use_absolute_value ? std::fabs(values[0]) : values[0];
        }
        if (!is_overridden(angular_name))
        {
            angular_value = use_absolute_value ? std::fabs(values[1]) : values[1];
        }
    };

    apply_legacy_pair(
        "min_velocity", "min_linear", "min_angular",
        config_.min_linear, config_.min_angular, false);
    apply_legacy_pair(
        "max_velocity", "max_linear", "max_angular",
        config_.max_linear, config_.max_angular, false);
    apply_legacy_pair(
        "max_accel", "max_linear_accel", "max_angular_accel",
        config_.max_linear_accel, config_.max_angular_accel, true);
    apply_legacy_pair(
        "max_decel", "max_linear_decel", "max_angular_decel",
        config_.max_linear_decel, config_.max_angular_decel, true);

    if (config_.frequency_hz <= 0.0)
    {
        RCLCPP_WARN(
            get_logger(), "Parameter 'frequency' must be positive; using 30 Hz");
        config_.frequency_hz = 30.0;
    }
    

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/antobot/robot/cmd_vel", 10,
        std::bind(&ControlBase::cmd_vel_callback, this, std::placeholders::_1));
    speed_status_sub_ = create_subscription<antobot_platform_msgs::msg::Float32Array>(
        "/antobot/bridge/wheel_vel", 10,
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
    raw_cmd_.linear_x = msg->linear.x;
    raw_cmd_.linear_y = msg->linear.y;
    raw_cmd_.angular_z = msg->angular.z;
    last_command_sec_ = now().seconds();
    has_command_ = true;
    
    on_robot_command(raw_cmd_);
}

void ControlBase::speed_status_callback(
    const antobot_platform_msgs::msg::Float32Array::SharedPtr msg)
{
    if (msg->data.size() < speed_feedback_.size())
    {
        RCLCPP_WARN(get_logger(), "Speed feedback requires four elements, got %zu", msg->data.size());
        return;
    }
    std::copy_n(msg->data.begin(), speed_feedback_.size(), speed_feedback_.begin());
    
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

    SpeedCmd target = raw_cmd_;
    if (config_.enable_timeout &&
        (!has_command_ || now_sec - last_command_sec_ > config_.velocity_timeout_sec))
    {
        target = {};
    }
    target.linear_x = std::clamp(target.linear_x, config_.min_linear, config_.max_linear);
    target.linear_y = std::clamp(target.linear_y, config_.min_linear, config_.max_linear);
    target.angular_z = std::clamp(target.angular_z, config_.min_angular, config_.max_angular);

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
