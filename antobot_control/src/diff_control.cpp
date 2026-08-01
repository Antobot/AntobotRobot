#include "diff_control.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

DiffControl::DiffControl()
    : ControlBase("diff_control")
{
    declare_parameter<std::string>("drive_type", "wheel");
    declare_parameter<double>("track_width", 0.6);
    declare_parameter<double>("wheel_radius", 0.165);
    declare_parameter<std::vector<double>>(
        "wheel_speed_correction", std::vector<double>{1.0, 1.0, 1.0, 1.0});
    declare_parameter<double>("max_motor_rpm", 3000.0);
    declare_parameter<double>("gear_ratio", 40.0);
    declare_parameter<double>("sprocket_diameter", 0.3038);
    declare_parameter<double>("track_center_distance", 0.54);
    declare_parameter<double>("command_deadband", 0.02);
    declare_parameter<double>("control_frequency", 30.0);
    declare_parameter<double>("velocity_timeout", 0.1);
    declare_parameter<double>("min_linear", -0.5);
    declare_parameter<double>("max_linear", 0.5);
    declare_parameter<double>("min_angular", -0.5);
    declare_parameter<double>("max_angular", 0.5);
    declare_parameter<double>("max_linear_accel", 0.2);
    declare_parameter<double>("max_linear_decel", 3.0);
    declare_parameter<double>("max_angular_accel", 0.5);
    declare_parameter<double>("max_angular_decel", 3.0);

    const auto drive_type = get_parameter("drive_type").as_string();
    if (drive_type == "wheel")
    {
        drive_type_ = DriveType::WHEEL;
    }
    else if (drive_type == "track")
    {
        drive_type_ = DriveType::TRACK;
    }
    else
    {
        throw std::runtime_error("drive_type must be 'wheel' or 'track'");
    }

    track_width_ = get_parameter("track_width").as_double();
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_speed_correction_ = get_parameter("wheel_speed_correction").as_double_array();
    max_motor_rpm_ = get_parameter("max_motor_rpm").as_double();
    gear_ratio_ = get_parameter("gear_ratio").as_double();
    sprocket_diameter_ = get_parameter("sprocket_diameter").as_double();
    track_center_distance_ = get_parameter("track_center_distance").as_double();
    command_deadband_ = get_parameter("command_deadband").as_double();
    if (wheel_speed_correction_.size() != 4)
    {
        throw std::runtime_error("wheel_speed_correction must contain four values");
    }

    ControlParams params;
    params.actuator_count = 4;
    params.control_frequency_hz = get_parameter("control_frequency").as_double();
    params.velocity_timeout_sec = get_parameter("velocity_timeout").as_double();
    params.min_linear = get_parameter("min_linear").as_double();
    params.max_linear = get_parameter("max_linear").as_double();
    params.min_angular = get_parameter("min_angular").as_double();
    params.max_angular = get_parameter("max_angular").as_double();
    params.max_linear_accel = get_parameter("max_linear_accel").as_double();
    params.max_linear_decel = get_parameter("max_linear_decel").as_double();
    params.max_angular_accel = get_parameter("max_angular_accel").as_double();
    params.max_angular_decel = get_parameter("max_angular_decel").as_double();
    configure_control(params);
    start_control_loop();
    RCLCPP_INFO(get_logger(), "DiffControl started with drive_type=%s", drive_type.c_str());
}

void DiffControl::command_to_actuators(
    const RobotCommand &command, std::vector<double> &output)
{
    const double distance = drive_type_ == DriveType::WHEEL ? track_width_ : track_center_distance_;
    const double left_linear = command.linear_x - command.angular_z * distance * 0.5;
    const double right_linear = command.linear_x + command.angular_z * distance * 0.5;
    double left = 0.0;
    double right = 0.0;
    if (drive_type_ == DriveType::WHEEL)
    {
        if (wheel_radius_ > 0.0)
        {
            left = left_linear / wheel_radius_;
            right = right_linear / wheel_radius_;
        }
    }
    else
    {
        const double conversion = transmission_rpm_per_mps();
        if (max_motor_rpm_ > 0.0)
        {
            left = left_linear * conversion / max_motor_rpm_;
            right = right_linear * conversion / max_motor_rpm_;
        }
        const double scale = std::max({1.0, std::fabs(left), std::fabs(right)});
        left /= scale;
        right /= scale;
        if (std::fabs(left) < command_deadband_)
        {
            left = 0.0;
        }
        if (std::fabs(right) < command_deadband_)
        {
            right = 0.0;
        }
    }
    output = {left * wheel_speed_correction_[0], left * wheel_speed_correction_[1],
              right * wheel_speed_correction_[2], right * wheel_speed_correction_[3]};
}

bool DiffControl::feedback_to_body_twist(
    const std::vector<double> &feedback, RobotCommand &twist) const
{
    double left = 0.0;
    double right = 0.0;
    double distance = 0.0;
    if (drive_type_ == DriveType::WHEEL)
    {
        if (feedback.size() < 4 || wheel_radius_ <= 0.0)
        {
            return false;
        }
        left = 0.5 * (feedback[0] + feedback[1]) * wheel_radius_;
        right = 0.5 * (feedback[2] + feedback[3]) * wheel_radius_;
        distance = track_width_;
    }
    else
    {
        if (feedback.size() < 2)
        {
            return false;
        }
        const double conversion = transmission_rpm_per_mps();
        if (conversion <= 0.0)
        {
            return false;
        }
        left = feedback[0] / conversion;
        right = feedback[1] / conversion;
        distance = track_center_distance_;
    }
    if (distance <= 0.0)
    {
        return false;
    }
    twist.linear_x = 0.5 * (left + right);
    twist.linear_y = 0.0;
    twist.angular_z = (right - left) / distance;
    return true;
}

double DiffControl::transmission_rpm_per_mps() const
{
    constexpr double pi = 3.14159265358979323846;
    return sprocket_diameter_ > 0.0 ? 60.0 * gear_ratio_ / (pi * sprocket_diameter_) : 0.0;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffControl>());
    rclcpp::shutdown();
    return 0;
}
