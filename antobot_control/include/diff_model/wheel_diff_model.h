#pragma once

#include <array>

#include "diff_model/diff_model_base.h"

class WheelDiffModel final : public DiffModelBase
{
public:
    explicit WheelDiffModel(rclcpp::Node &node);
    void twist_to_rpm(
        const SpeedCmd &command, std::array<double, 4> &output) const override;
    bool rpm_to_twist(
        const std::array<double, 4> &feedback, SpeedCmd &twist) const override;

private:
    double track_width_{0.6};
    double wheel_radius_{0.165};
    std::array<double, 4> correction_{{1.0, 1.0, 1.0, 1.0}};
};
