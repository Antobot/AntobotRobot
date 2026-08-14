#include "wheel_control.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>

namespace
{
    ControlConfig wheel_default_config()
    {
        ControlConfig config;
        config.min_linear = -1.3;
        config.max_linear = 1.3;
        config.min_angular = -0.2;
        config.max_angular = 0.2;
        config.max_linear_accel = 0.1;
        config.max_linear_decel = 3.0;
        config.max_angular_accel = 0.1;
        config.max_angular_decel = 3.0;
        return config;
    }

    double rad_to_deg(double value)
    {
        return value * 180.0 / M_PI;
    }

    double deg_to_rad(double value)
    { 
        return value * M_PI / 180.0;
    }
}

constexpr std::array<std::size_t, 4> WheelControl::STEERING_TO_DRIVE;

WheelControl::WheelControl()
    : ControlBase("wheel_control", wheel_default_config())
{
    declare_parameter<double>("wheel_base", 1.156);
    declare_parameter<double>("track_width", 1.1);
    declare_parameter<double>("wheel_radius", 0.203);
    declare_parameter<double>("steering_tolerance_deg", 5.0);
    
    wheel_base_ = get_parameter("wheel_base").as_double();
    track_width_ = get_parameter("track_width").as_double();
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    steering_tolerance_deg_ = get_parameter("steering_tolerance_deg").as_double();

    steering_position_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/antobot/control/wheelsteer/real_pos_raw", 20,
        std::bind(&WheelControl::steering_position_callback, this, std::placeholders::_1));
    mode_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/antobot/control/wheelsteer/mode", 10,
        std::bind(&WheelControl::mode_callback, this, std::placeholders::_1));
    steering_command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/antobot/control/wheelsteer/cmd_pos_raw", 20);

    mode_ = Mode::LOCK;
    update_steering_target({});
}

std::array<WheelControl::Point, 4> WheelControl::wheel_positions() const
{
    return {{{wheel_base_ / 2.0, track_width_ / 2.0},
             {-wheel_base_ / 2.0, track_width_ / 2.0},
             {wheel_base_ / 2.0, -track_width_ / 2.0},
             {-wheel_base_ / 2.0, -track_width_ / 2.0}}};
}

double WheelControl::limit_steering(double angle)
{
    while (angle > 180.0)
    {
        angle -= 360.0;
    }
    while (angle < -180.0)
    {
        angle += 360.0;
    }
    if (angle > 90.0)
    {
        angle -= 180.0;
    }
    if (angle < -90.0)
    {
        angle += 180.0;
    }
    return std::clamp(angle, -90.0, 90.0);
}

void WheelControl::on_robot_command(const SpeedCmd &value)
{
    update_steering_target(value);
}

void WheelControl::mode_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    mode_ = static_cast<Mode>(msg->data);
    update_steering_target(command());
}

void WheelControl::steering_position_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 4)
    {
        return;
    }
    for (std::size_t i = 0; i < 4; ++i)
    {
        actual_steering_deg_[STEERING_TO_DRIVE[i]] = msg->data[i];
    }
}

void WheelControl::update_steering_target(const SpeedCmd &value)
{
    const auto positions = wheel_positions();
    if (mode_ == Mode::CRAB)
    {
        const double angle = std::hypot(value.linear_x, value.linear_y) > 1e-4 ? rad_to_deg(std::atan2(value.linear_y, value.linear_x)) : 0.0;
        target_steering_deg_.fill(limit_steering(angle));
    }
    else if (mode_ == Mode::COUNTERPHASE)
    {
        for (std::size_t i = 0; i < 4; ++i)
        {
            const double vx = value.linear_x - value.angular_z * positions[i].y;
            const double vy = value.linear_y + value.angular_z * positions[i].x;
            if (std::hypot(vx, vy) > 1e-4)
            {
                target_steering_deg_[i] = limit_steering(rad_to_deg(std::atan2(vy, vx)));
            }
        }
    }
    else if (mode_ == Mode::SPOTTURN)
    {
        const double angle = rad_to_deg(std::atan2(wheel_base_, track_width_));
        target_steering_deg_ = {-angle, angle, angle, -angle};
    }
    else
    {
        target_steering_deg_ = {45.0, -45.0, -45.0, 45.0};
    }

    std_msgs::msg::Float64MultiArray output;
    output.data.resize(4);
    for (std::size_t i = 0; i < 4; ++i)
    {
        output.data[i] = target_steering_deg_[STEERING_TO_DRIVE[i]];
    }
    steering_command_pub_->publish(output);
}

bool WheelControl::motion_enabled() const
{
    if (mode_ == Mode::LOCK)
    {
        return false;
    }
    for (std::size_t i = 0; i < 4; ++i)
    {
        if (std::fabs(actual_steering_deg_[i] - target_steering_deg_[i]) > steering_tolerance_deg_)
        {
            return false;
        }
    }
    return true;
}

void WheelControl::twist_to_rpm(
    const SpeedCmd &value, std::array<double, 4> &output)
{
    const auto positions = wheel_positions();
    for (std::size_t i = 0; i < 4; ++i)
    {
        const double vx = value.linear_x - value.angular_z * positions[i].y;
        const double vy = value.linear_y + value.angular_z * positions[i].x;
        const double angle = deg_to_rad(actual_steering_deg_[i]);
        const double longitudinal = vx * std::cos(angle) + vy * std::sin(angle);
        output[i] = wheel_radius_ > 0.0 ? longitudinal / wheel_radius_ : 0.0;
    }
}

bool WheelControl::rpm_to_twist(
    const std::array<double, 4> &feedback, SpeedCmd &twist) const
{
    if (wheel_radius_ <= 0.0)
    {
        return false;
    }
    const auto positions = wheel_positions();
    double ata[3][3] = {};
    double atb[3] = {};
    for (std::size_t i = 0; i < 4; ++i)
    {
        const double angle = deg_to_rad(actual_steering_deg_[i]);
        const double c = std::cos(angle);
        const double s = std::sin(angle);
        const double row[3] = {c, s, -positions[i].y * c + positions[i].x * s};
        const double longitudinal = feedback[i] * wheel_radius_;
        for (int r = 0; r < 3; ++r)
        {
            atb[r] += row[r] * longitudinal;
            for (int col = 0; col < 3; ++col)
            {
                ata[r][col] += row[r] * row[col];
            }
        }
    }
    double matrix[3][4] = {{ata[0][0], ata[0][1], ata[0][2], atb[0]},
                           {ata[1][0], ata[1][1], ata[1][2], atb[1]},
                           {ata[2][0], ata[2][1], ata[2][2], atb[2]}};
    for (int pivot = 0; pivot < 3; ++pivot)
    {
        int best = pivot;
        for (int row = pivot + 1; row < 3; ++row)
        {
            if (std::fabs(matrix[row][pivot]) > std::fabs(matrix[best][pivot]))
            {
                best = row;
            }
        }
        if (std::fabs(matrix[best][pivot]) < 1e-9)
        {
            return false;
        }
        for (int col = 0; col < 4; ++col)
        {
            std::swap(matrix[pivot][col], matrix[best][col]);
        }
        const double divisor = matrix[pivot][pivot];
        for (int col = pivot; col < 4; ++col)
        {
            matrix[pivot][col] /= divisor;
        }
        for (int row = 0; row < 3; ++row)
        {
            if (row == pivot)
            {
                continue;
            }
            const double factor = matrix[row][pivot];
            for (int col = pivot; col < 4; ++col)
            {
                matrix[row][col] -= factor * matrix[pivot][col];
            }
        }
    }
    twist.linear_x = matrix[0][3];
    twist.linear_y = matrix[1][3];
    twist.angular_z = matrix[2][3];
    return true;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelControl>());
    rclcpp::shutdown();
    return 0;
}
