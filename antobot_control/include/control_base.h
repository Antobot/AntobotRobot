#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "antobot_platform_msgs/msg/float32_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

struct RobotCommand
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

struct ControlParams
{
    std::size_t actuator_count{4};
    double control_frequency_hz{30.0};
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
    explicit ControlBase(const std::string &node_name);
    ~ControlBase() override = default;

protected:
    void configure_control(const ControlParams &params);
    void start_control_loop();

    const RobotCommand &command() const;
    const std::vector<double> &speed_feedback() const;
    const OdometryState &odometry() const;

    virtual void command_to_actuators(const RobotCommand &command, std::vector<double> &actuator_command) = 0;
    virtual bool feedback_to_body_twist(const std::vector<double> &feedback, RobotCommand &body_twist) const = 0;
    virtual void on_robot_command(const RobotCommand &command);
    virtual void on_speed_feedback(const std::vector<double> &feedback);
    virtual bool motion_enabled() const;

private:
    static double approach(
        double current, double target, double accel, double decel, double dt);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void speed_status_callback(
        const antobot_platform_msgs::msg::Float32Array::SharedPtr msg);
    void smooth_command(double now_sec);
    void control_loop();
    void update_odometry(double now_sec);
    void publish_odometry();

    ControlParams params_;
    RobotCommand raw_command_;
    RobotCommand smoothed_command_;
    OdometryState odometry_;
    std::vector<double> actuator_command_;
    std::vector<double> speed_feedback_;
    bool has_command_{false};
    double last_command_sec_{0.0};
    double last_control_sec_{0.0};

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<antobot_platform_msgs::msg::Float32Array>::SharedPtr speed_status_sub_;
    rclcpp::Publisher<antobot_platform_msgs::msg::Float32Array>::SharedPtr wheel_command_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};
