#pragma once

#include <string>
#include <vector>

#include "control_base.h"

class DiffControl final : public ControlBase
{
public:
    enum class DriveType
    {
        WHEEL,
        TRACK
    };
    DiffControl();

private:
    void command_to_actuators(
        const RobotCommand &command, std::vector<double> &output) override;
    bool feedback_to_body_twist(
        const std::vector<double> &feedback, RobotCommand &twist) const override;
    double transmission_rpm_per_mps() const;

    DriveType drive_type_{DriveType::WHEEL};
    double track_width_{0.6};
    double wheel_radius_{0.165};
    std::vector<double> wheel_speed_correction_{1.0, 1.0, 1.0, 1.0};
    double max_motor_rpm_{3000.0};
    double gear_ratio_{40.0};
    double sprocket_diameter_{0.3038};
    double track_center_distance_{0.54};
    double command_deadband_{0.02};
};
