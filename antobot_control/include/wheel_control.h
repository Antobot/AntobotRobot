#pragma once

#include <array>
#include <vector>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

#include "control_base.h"

enum ControlMode: std::size_t
{
    CRAB = 0,
    COUNTERFACE = 1,
    DRIVE = 1,
    SPOTTURN = 2,
    PARK = 3,
    LOCK = 3
};

class WheelControl final : public ControlBase
{
public:
    WheelControl();

private:
    struct Point
    {
        double x;
        double y;
    };
    static constexpr std::array<std::size_t, 4> STEERING_TO_DRIVE{{2, 0, 1, 3}};

    std::array<Point, 4> wheel_positions() const;
    static double limit_steering(double angle_deg);
    void publish_control_mode();
    void on_robot_command(SpeedCmd &cmd) override;
    void mode_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void steering_position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void update_steering_target(const SpeedCmd &command);
    bool motion_enabled() const override;
    
    void twist_to_rpm(
        const SpeedCmd &command, std::array<double, 4> &output) override;
    bool rpm_to_twist(
        const std::array<double, 4> &feedback, SpeedCmd &twist) const override;

private:
    ControlMode mode_{ControlMode::LOCK};
    double wheel_base_{1.156};
    double track_width_{1.1};
    double wheel_radius_{0.203};
    double steering_tolerance_deg_{5.0};
    std::array<double, 4> actual_steering_deg_{};
    std::array<double, 4> target_steering_deg_{};
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr steering_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_pub_;

    rclcpp::TimerBase::SharedPtr mode_pub_timer_;
};
