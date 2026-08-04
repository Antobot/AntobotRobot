#pragma once

#include <array>
#include <memory>

#include "control_base.h"
#include "diff_model/diff_model_base.h"

class DiffControl final : public ControlBase
{
public:
    DiffControl();

protected:
    void speed_status_callback(const antobot_platform_msgs::msg::Float32Array::SharedPtr msg) override;

private:
    void twist_to_rpm(
        const SpeedCmd &command, std::array<double, 4> &output) override;
    bool rpm_to_twist(
        const std::array<double, 4> &feedback, SpeedCmd &twist) const override;

    std::unique_ptr<DiffModelBase> model_;
};
