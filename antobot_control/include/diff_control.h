#pragma once

#include <array>
#include <memory>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "control_base.h"
#include "diff_model/diff_model_base.h"


class DiffControl final : public ControlBase
{
public:
    DiffControl();

protected:
    void track_status_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

private:
    void twist_to_rpm(
        const SpeedCmd &command, std::array<double, 4> &output) override;
    bool rpm_to_twist(
        const std::array<double, 4> &feedback, SpeedCmd &twist) const override;

private:
    std::unique_ptr<DiffModelBase> model_;
    
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr track_status_sub_;
};
