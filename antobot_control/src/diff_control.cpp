#include "diff_control.h"

#include <memory>
#include <stdexcept>
#include <string>

#include "diff_model/track_diff_model_401.h"
#include "diff_model/track_diff_model.h"
#include "diff_model/wheel_diff_model.h"

DiffControl::DiffControl()
    : ControlBase("diff_control")
{
    const std::string &robot_role = config().robot_role;

    if (robot_role[1] == '3')
    {
        model_ = std::make_unique<WheelDiffModel>(*this);
    }
    else if (robot_role == "S401")
    {
        model_ = std::make_unique<TrackDiffModel_401>(*this);
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Invalid robot_role: %s", robot_role.c_str());
        throw std::runtime_error("Invalid robot_role: " + robot_role);
    }

    RCLCPP_INFO(get_logger(), "DiffControl started with robot_role=%s", robot_role.c_str());
}

void DiffControl::twist_to_rpm(
    const SpeedCmd &command, std::array<double, 4> &output)
{
    model_->twist_to_rpm(command, output);
}

bool DiffControl::rpm_to_twist(
    const std::array<double, 4> &feedback, SpeedCmd &twist) const
{
    return model_->rpm_to_twist(feedback, twist);
}

void DiffControl::speed_status_callback(const antobot_platform_msgs::msg::Float32Array::SharedPtr msg) 
{
    model_->run_buzzer(msg);

    ControlBase::speed_status_callback(msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffControl>());
    rclcpp::shutdown();
    return 0;
}
