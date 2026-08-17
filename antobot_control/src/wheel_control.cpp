#include "wheel_control.h"

#include <algorithm>
#include <chrono>
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

    // Sub
    steering_position_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/antobot/control/wheelsteer/real_pos_raw", 20,
        std::bind(&WheelControl::steering_position_callback, this, std::placeholders::_1));
    mode_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/antobot/control/wheelsteer/mode", 10,
        std::bind(&WheelControl::mode_callback, this, std::placeholders::_1));
    
    // Pub
    steering_command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/antobot/control/wheelsteer/cmd_pos_raw", 20);
    mode_pub_ = create_publisher<std_msgs::msg::Int32>(
        "/nats/control_mode", 10);

    mode_pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&WheelControl::publish_control_mode, this));

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

void WheelControl::publish_control_mode()
{
    std_msgs::msg::Int32 msg;
    msg.data = mode_;
    mode_pub_->publish(msg);
}

void WheelControl::on_robot_command(SpeedCmd &cmd)
{

    switch (mode_)
    {
    case ControlMode::CRAB:
        cmd.linear_x = cmd.linear_x;
        cmd.linear_y = -cmd.linear_y ;
        break;
    case ControlMode::DRIVE:
        cmd.linear_x = cmd.linear_x;
        cmd.angular_z = -cmd.linear_y;
    case ControlMode::SPOTTURN:
        break;
        cmd.angular_z = -cmd.linear_y;
        break;
    case ControlMode::LOCK:
    default:
        break;
    }

    update_steering_target(cmd);
}

void WheelControl::mode_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    ControlMode new_mode = static_cast<ControlMode>(msg->data);
    if(mode_ == new_mode)
        return ;

    mode_ = new_mode;
    update_steering_target(command());

    // log
    static constexpr std::array<std::string_view, 4> mode_names{
        "CRAB",
        "DRIVE",
        "SPOTTURN",
        "PARK"
    };
    RCLCPP_INFO(get_logger(), 
        "[Control] set mode to %d(%s)", (int)mode_, mode_names[mode_].data()
    );
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
    if (mode_ == ControlMode::CRAB)
    {
        const double angle = std::hypot(value.linear_x, value.linear_y) > 1e-4 ? rad_to_deg(std::atan2(value.linear_y, value.linear_x)) : 0.0;
        target_steering_deg_.fill(limit_steering(angle));
    }
    else if (mode_ == ControlMode::DRIVE)
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
    else if (mode_ == ControlMode::SPOTTURN)
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
    if (mode_ == ControlMode::LOCK)
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
