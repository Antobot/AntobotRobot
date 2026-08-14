#include "diff_model/track_diff_model.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

TrackDiffModel::TrackDiffModel(rclcpp::Node &node)
{
    node.declare_parameter<double>("max_motor_rpm", 3000.0);
    node.declare_parameter<double>("gear_ratio", 40.0);
    node.declare_parameter<double>("sprocket_diameter", 0.3038);
    node.declare_parameter<double>("track_center_distance", 0.54);
    node.declare_parameter<double>("command_deadband", 0.02);
    node.declare_parameter<std::vector<double>>(
        "wheel_speed_correction", std::vector<double>{1.0, 1.0, 1.0, 1.0});

    max_motor_rpm_ = node.get_parameter("max_motor_rpm").as_double();
    gear_ratio_ = node.get_parameter("gear_ratio").as_double();
    sprocket_diameter_ = node.get_parameter("sprocket_diameter").as_double();
    track_center_distance_ = node.get_parameter("track_center_distance").as_double();
    command_deadband_ = node.get_parameter("command_deadband").as_double();
    const auto correction = node.get_parameter("wheel_speed_correction").as_double_array();
    if (correction.size() != correction_.size())
    {
        throw std::runtime_error("wheel_speed_correction must contain four values");
    }
    std::copy(correction.begin(), correction.end(), correction_.begin());
}

void TrackDiffModel::twist_to_rpm(
    const SpeedCmd &command, std::array<double, 4> &output) const
{
    const double left_linear =
        command.linear_x - command.angular_z * track_center_distance_ * 0.5;
    const double right_linear =
        command.linear_x + command.angular_z * track_center_distance_ * 0.5;
    const double conversion = transmission_rpm_per_mps();
    double left = 0.0;
    double right = 0.0;
    if (max_motor_rpm_ > 0.0)
    {
        left = left_linear * conversion / max_motor_rpm_;
        right = right_linear * conversion / max_motor_rpm_;
    }
    const double scale = std::max({1.0, std::fabs(left), std::fabs(right)});
    left /= scale;
    right /= scale;
    if (std::fabs(left) < command_deadband_) {left = 0.0;}
    if (std::fabs(right) < command_deadband_) {right = 0.0;}
    output = {left * correction_[0], left * correction_[1],
              right * correction_[2], right * correction_[3]};
}

bool TrackDiffModel::rpm_to_twist(
    const std::array<double, 4> &feedback, SpeedCmd &twist) const
{
    const double conversion = transmission_rpm_per_mps();
    if (conversion <= 0.0 || track_center_distance_ <= 0.0)
    {
        return false;
    }
    const double left = 0.5 * (feedback[0] + feedback[1]) / conversion;
    const double right = 0.5 * (feedback[2] + feedback[3]) / conversion;
    twist.linear_x = 0.5 * (left + right);
    twist.linear_y = 0.0;
    twist.angular_z = (right - left) / track_center_distance_;
    return true;
}

double TrackDiffModel::transmission_rpm_per_mps() const
{
    constexpr double pi = 3.14159265358979323846;
    return sprocket_diameter_ > 0.0 ?
        60.0 * gear_ratio_ / (pi * sprocket_diameter_) : 0.0;
}
