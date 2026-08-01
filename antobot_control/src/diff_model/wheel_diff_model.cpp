#include "diff_model/wheel_diff_model.h"

#include <algorithm>
#include <stdexcept>
#include <vector>

WheelDiffModel::WheelDiffModel(rclcpp::Node &node)
{
    node.declare_parameter<double>("track_width", 0.6);
    node.declare_parameter<double>("wheel_radius", 0.165);
    node.declare_parameter<std::vector<double>>(
        "wheel_speed_correction", std::vector<double>{1.0, 1.0, 1.0, 1.0});

    track_width_ = node.get_parameter("track_width").as_double();
    wheel_radius_ = node.get_parameter("wheel_radius").as_double();
    const auto correction = node.get_parameter("wheel_speed_correction").as_double_array();
    if (correction.size() != correction_.size())
    {
        throw std::runtime_error("wheel_speed_correction must contain four values");
    }
    std::copy(correction.begin(), correction.end(), correction_.begin());
}

void WheelDiffModel::twist_to_rpm(
    const SpeedCmd &command, std::array<double, 4> &output) const
{
    double left = 0.0;
    double right = 0.0;
    if (wheel_radius_ > 0.0)
    {
        left = (command.linear_x - command.angular_z * track_width_ * 0.5) / wheel_radius_;
        right = (command.linear_x + command.angular_z * track_width_ * 0.5) / wheel_radius_;
    }
    output = {left * correction_[0], left * correction_[1],
              right * correction_[2], right * correction_[3]};
}

bool WheelDiffModel::rpm_to_twist(
    const std::array<double, 4> &feedback, SpeedCmd &twist) const
{
    if (wheel_radius_ <= 0.0 || track_width_ <= 0.0)
    {
        return false;
    }
    const double left = 0.5 * (feedback[0] + feedback[1]) * wheel_radius_;
    const double right = 0.5 * (feedback[2] + feedback[3]) * wheel_radius_;
    twist.linear_x = 0.5 * (left + right);
    twist.linear_y = 0.0;
    twist.angular_z = (right - left) / track_width_;
    return true;
}
