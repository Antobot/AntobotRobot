#pragma once

#include <array>

#include "diff_model/diff_model_base.h"

class TrackDiffModel final : public DiffModelBase
{
public:
    explicit TrackDiffModel(rclcpp::Node &node);
    void twist_to_rpm(
        const SpeedCmd &command, std::array<double, 4> &output) const override;
    bool rpm_to_twist(
        const std::array<double, 4> &feedback, SpeedCmd &twist) const override;

private:
    double transmission_rpm_per_mps() const;

    double max_motor_rpm_{3000.0};
    double gear_ratio_{40.0};
    double sprocket_diameter_{0.3038};
    double track_center_distance_{0.54};
    double command_deadband_{0.02};
    std::array<double, 4> correction_{{1.0, 1.0, 1.0, 1.0}};
};
