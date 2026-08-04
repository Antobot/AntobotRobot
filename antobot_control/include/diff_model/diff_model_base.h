#pragma once

#include <array>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "control_base.h"

class DiffModelBase
{
public:
    virtual ~DiffModelBase() = default;
    
    virtual void twist_to_rpm(
        const SpeedCmd &command, std::array<double, 4> &output) const = 0;
    
    virtual bool rpm_to_twist(
        const std::array<double, 4> &feedback, SpeedCmd &twist) const = 0;

    virtual void run_buzzer(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {};
};
