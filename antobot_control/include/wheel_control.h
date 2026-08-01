#pragma once

#include <array>
#include <vector>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

#include "control_base.h"

class WheelControl final : public ControlBase
{
public:
    enum class Mode
    {
        CRAB = 0,
        COUNTERPHASE = 1,
        SPOTTURN = 2,
        LOCK = 3
    };
    WheelControl();

private:
    struct Point
    {
        double x;
        double y;
    };
    static constexpr std::array<std::size_t, 4> STEERING_TO_DRIVE{{2, 0, 1, 3}};

    std::array<Point, 4> wheel_positions() const;
    static double rad_to_deg(double radians);
    static double deg_to_rad(double degrees);
    static double limit_steering(double angle_deg);
    void on_robot_command(const RobotCommand &command) override;
    void mode_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void steering_position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void update_steering_target(const RobotCommand &command);
    bool motion_enabled() const override;
    void command_to_actuators(
        const RobotCommand &command, std::vector<double> &output) override;
    bool feedback_to_body_twist(
        const std::vector<double> &feedback, RobotCommand &twist) const override;

    Mode mode_{Mode::LOCK};
    double wheel_base_{1.156};
    double track_width_{1.1};
    double wheel_radius_{0.203};
    double steering_tolerance_deg_{5.0};
    std::array<double, 4> actual_steering_deg_{};
    std::array<double, 4> target_steering_deg_{};
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr steering_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_command_pub_;
};
