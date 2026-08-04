#include "diff_model/track_diff_model_401.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

TrackDiffModel_401::TrackDiffModel_401(rclcpp::Node &node)
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


    buzzer_pub_ = node.create_publisher<std_msgs::msg::Bool>(
        "/antobridge/PDU_C", 10);


}

void TrackDiffModel_401::twist_to_rpm(
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


    // speed should be half when moving backward, to avoid slipping
    if (output[0] < 0) { output[0] *= 0.5; }
    if (output[1] < 0) { output[1] *= 0.5; }
    if (output[2] < 0) { output[2] *= 0.5; }
    if (output[3] < 0) { output[3] *= 0.5; }
}

bool TrackDiffModel_401::rpm_to_twist(
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

double TrackDiffModel_401::transmission_rpm_per_mps() const
{
    constexpr double pi = 3.14159265358979323846;
    return sprocket_diameter_ > 0.0 ?
        60.0 * gear_ratio_ / (pi * sprocket_diameter_) : 0.0;
}

void TrackDiffModel_401::run_buzzer(const antobot_platform_msgs::msg::Float32Array::SharedPtr speed_msg)
{
    static bool buzzer_on = false;

    if(speed_msg->data[0] < 0 && speed_msg->data[2] < 0)
    {
        // add one frequency limit to avoid buzzer on/off too fast
        static auto last_buzzer_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_buzzer_time).count() < 1)
            return;

        last_buzzer_time = now;
        buzzer_on = !buzzer_on;
        

        std_msgs::msg::Bool buzzer_msg;
        buzzer_msg.data = buzzer_on;
        buzzer_pub_->publish(buzzer_msg);
    }
    else if(buzzer_on)
    {
        // if the speed is positive and the buzzer is currently on, turn off the buzzer
        std_msgs::msg::Bool buzzer_msg;
        buzzer_msg.data = false;
        buzzer_pub_->publish(buzzer_msg);
        buzzer_on = false;
    }


}

