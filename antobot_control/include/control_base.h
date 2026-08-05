#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "antobot_platform_msgs/msg/float32_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

struct SpeedCmd
{
    double linear_x{0.0};
    double linear_y{0.0};
    double angular_z{0.0};
};

struct OdometryState
{
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double linear_x{0.0};
    double linear_y{0.0};
    double angular_z{0.0};
    double last_update_sec{0.0};
};

struct ControlConfig
{
    std::string robot_role{""};
    double frequency_hz{30.0};
    double velocity_timeout_sec{0.1};
    double min_linear{-0.5};
    double max_linear{0.5};
    double min_angular{-0.5};
    double max_angular{0.5};
    double max_linear_accel{0.2};
    double max_linear_decel{3.0};
    double max_angular_accel{0.5};
    double max_angular_decel{3.0};
    bool enable_smoothing{true};
    bool enable_timeout{true};
    bool enable_odom{true};
};

class ControlBase : public rclcpp::Node
{
public:
    explicit ControlBase(
        const std::string &node_name,
        const ControlConfig &default_config = ControlConfig{});
    virtual ~ControlBase() = default;

protected:
    const SpeedCmd &command() const;
    const ControlConfig &config() const;

    virtual void twist_to_rpm(
        const SpeedCmd &command, std::array<double, 4> &actuator_command) = 0;

    virtual bool rpm_to_twist(
        const std::array<double, 4> &feedback, SpeedCmd &body_twist) const = 0;

    virtual void on_robot_command(const SpeedCmd &) {}
    virtual bool motion_enabled() const;

    virtual void speed_status_callback(const antobot_platform_msgs::msg::Float32Array::SharedPtr msg);

private:
    void control_loop();
    
    void smooth_command();
    
    void update_odometry();

    void publish_speed_cmd();
    void publish_odometry();

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);


private:
    ControlConfig config_;
    SpeedCmd raw_cmd_;
    SpeedCmd smoothed_cmd_twist_;
    OdometryState odometry_;
    std::array<double, 4> speed_cmd_rpm_{};
    std::array<double, 4> speed_feedback_{};
    bool has_command_{false};
    double last_command_sec_{0.0};
    double last_control_sec_{0.0};

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<antobot_platform_msgs::msg::Float32Array>::SharedPtr speed_status_sub_;
    rclcpp::Publisher<antobot_platform_msgs::msg::Float32Array>::SharedPtr speed_cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};
