#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <cstddef>

namespace antobot_control
{

// Twist command (linear + angular velocity)
struct RobotCommand
{
    double linear{0.0};   // m/s
    double angular{0.0};  // rad/s
};

// Wheel-level command (target wheel or track speeds)
struct WheelCommand
{
    std::vector<double> target_speed;  // length == wheel_count
};

// Wheel-level feedback (measured wheel speeds)
struct WheelFeedback
{
    std::vector<double> measured_speed;
    bool   valid{false};
    double last_update_time_sec{0.0};
};

// Odometry state (position + orientation + twist velocity)
struct OdometryState
{
    double x{0.0};
    double y{0.0};
    double yaw{0.0};  // rad

    double linear{0.0};
    double angular{0.0};

    double last_update_time_sec{0.0};
};

// Control status (time stamps and timeout flags)
struct ControlStatus
{
    double last_robot_cmd_time_sec{0.0};
    double last_control_update_sec{0.0};

    bool has_robot_cmd{false};
    bool timed_out{false};
};

// Control parameters
struct ControlParams
{
    // Geometry / wheel configuration
    std::size_t wheel_count{0};
    double track_width{0.0};     // distance between left and right tracks/wheels [m]
    double wheel_radius{0.0};    // wheel radius [m], if needed

    // Control frequency and timeout
    double control_frequency_hz{30.0};
    double velocity_timeout_sec{0.1};

    // Robot-level velocity limits
    double max_linear{0.5};
    double min_linear{-0.5};
    double max_angular{0.5};
    double min_angular{-0.5};

    // Robot-level acceleration / deceleration limits
    double max_linear_accel{0.2};    // m/s^2
    double max_linear_decel{-3.0};    // m/s^2 (positive, sign handled in code)
    double max_angular_accel{0.5};   // rad/s^2
    double max_angular_decel{-3.0};   // rad/s^2

    // Feature switches
    bool enable_smoothing{true};
    bool enable_timeout{true};
    bool enable_odom{true};
};

// Global control state
struct ControlState
{
    ControlParams  params;

    RobotCommand   raw_cmd;        // last received robot command
    RobotCommand   clipped_cmd;    // command after velocity limits
    RobotCommand   smoothed_cmd;   // command after smoothing

    WheelCommand   wheel_cmd;      // command to wheel/track actuators
    WheelFeedback  wheel_feedback; // wheel/track feedback

    OdometryState  odom;           // odom, will be filled by odom compute

    ControlStatus  status;         // timestamps and timeout flags
};


class AntoControlBase
{
public:
    AntoControlBase() = default;
    virtual ~AntoControlBase() = default;

    // Set parameters (geometry, limits, switches)
    void setParams(const ControlParams & params)
    {
        state_.params = params;
        // Ensure wheel_cmd / feedback vectors have correct size
        state_.wheel_cmd.target_speed.resize(state_.params.wheel_count, 0.0);
        state_.wheel_feedback.measured_speed.resize(state_.params.wheel_count, 0.0);
    }

    const ControlState & getState() const { return state_; }
    ControlState & getState() { return state_; }

    // =====================================================================
    // robot_cmd_vel_callback
    // =====================================================================
    // Update robot-level command and command timestamp.
    void robot_cmd_vel_callback(double msg_linear,
                                double msg_angular,
                                double now_time_sec)
    {
        state_.raw_cmd.linear  = msg_linear;
        state_.raw_cmd.angular = msg_angular;

        state_.status.last_robot_cmd_time_sec = now_time_sec;
        state_.status.has_robot_cmd = true;
        state_.status.timed_out = false;
    }

    // ======================================================================
    // wheel_cmd_vel_callback (wheel feedback)
    // ======================================================================
    // Update wheel/track feedback from measured speeds.
    void wheel_cmd_vel_callback(const std::vector<double> & measured_speed,
                                double now_time_sec)
    {
        if (measured_speed.size() != state_.params.wheel_count ||
            state_.params.wheel_count == 0)
        {
            return;
        }

        state_.wheel_feedback.measured_speed = measured_speed;
        state_.wheel_feedback.last_update_time_sec = now_time_sec;
        state_.wheel_feedback.valid = true;
    }

    // ======================================================================
    // velocity_smoother
    // ======================================================================
    // Apply timeout, velocity limits and acceleration/deceleration constraints.
    void velocity_smoother(double now_time_sec)
    {
        const auto & p = state_.params;

        // Compute dt for control cycle
        double dt = now_time_sec - state_.status.last_control_update_sec;
        if (state_.status.last_control_update_sec <= 0.0 || dt <= 0.0) {
            // First run or non-positive dt: approximate dt by control period
            dt = (p.control_frequency_hz > 0.0) ?
                (1.0 / p.control_frequency_hz) : 0.0;
        }
        state_.status.last_control_update_sec = now_time_sec;

        // Timeout check
        if (p.enable_timeout) {
            double dt_cmd = now_time_sec - state_.status.last_robot_cmd_time_sec;
            if (!state_.status.has_robot_cmd || dt_cmd > p.velocity_timeout_sec) {
                // Stop robot on timeout
                state_.smoothed_cmd.linear  = 0.0;
                state_.smoothed_cmd.angular = 0.0;
                state_.clipped_cmd = state_.smoothed_cmd;
                state_.status.timed_out = true;
                return;
            }
        }

        // Velocity clamping
        state_.clipped_cmd.linear = std::clamp(
            state_.raw_cmd.linear, p.min_linear, p.max_linear);
        state_.clipped_cmd.angular = std::clamp(
            state_.raw_cmd.angular, p.min_angular, p.max_angular);

        // If smoothing disabled, use clipped command directly
        if (!p.enable_smoothing || dt <= 0.0) {
            state_.smoothed_cmd = state_.clipped_cmd;
            state_.status.timed_out = false;
            return;
        }

        // Acceleration / deceleration constraints
        const auto prev = state_.smoothed_cmd;     // previous smoothed command
        const auto cmd  = state_.clipped_cmd;      // current target (after clamp)

        // Linear
        double v_curr = prev.linear;
        double v_cmd  = cmd.linear;
        double dv     = v_cmd - v_curr;

        double dv_max = 0.0;
        double dv_min = 0.0;

        if (std::fabs(v_cmd) >= std::fabs(v_curr) && v_cmd * v_curr >= 0.0) {
            // Accelerating in same direction
            dv_max =  p.max_linear_accel * dt;
            dv_min = -p.max_linear_accel * dt;
        } else {
            // Decelerating or changing direction
            dv_max = -p.max_linear_decel * dt;
            dv_min =  p.max_linear_decel * dt;
        }
        double dv_clamped = std::clamp(dv, dv_min, dv_max);
        double new_linear = v_curr + dv_clamped;

        // Angular
        double w_curr = prev.angular;
        double w_cmd  = cmd.angular;
        double dw     = w_cmd - w_curr;

        double dw_max = 0.0;
        double dw_min = 0.0;

        if (std::fabs(w_cmd) >= std::fabs(w_curr) && w_cmd * w_curr >= 0.0) {
            // Accelerating in same direction
            dw_max =  p.max_angular_accel * dt;
            dw_min = -p.max_angular_accel * dt;
        } else {
            // Decelerating or changing direction
            dw_max =  p.max_angular_decel * dt;
            dw_min = -p.max_angular_decel * dt;
        }
        double dw_clamped = std::clamp(dw, dw_min, dw_max);
        double new_angular = w_curr + dw_clamped;

        state_.smoothed_cmd.linear  = new_linear;
        state_.smoothed_cmd.angular = new_angular;
        state_.status.timed_out = false;
    }

    // ======================================================================
    // wheel_speed_compute (to be implemented by subclasses)
    // ======================================================================
    // Convert smoothed_twist into wheel_cmd.target_speed.
    // Different robots will implement their own calculations here.
    virtual void wheel_speed_compute() = 0;


    // ======================================================================
    // compute_robot_twist_from_wheels (to be implemented by subclasses)
    // ======================================================================
    // Given wheel_feedback.measured_speed, compute robotTwist (v, w)
    // Implementations depend on different motors
    virtual void compute_robot_twist_from_wheels(double & linear, double & angular) = 0;



    // ======================================================================
    // robot_odom_compute
    // ======================================================================
    // Integrate odometry using wheel feedback and chassis kinematics.
    // Base class handles integration; subclasses provide v, w via compute_robot_twist_from_wheels().
    virtual void robot_odom_compute(double now_time_sec){
        auto & p    = state_.params;
        auto & fb   = state_.wheel_feedback;
        auto & odom = state_.odom;

        if (!p.enable_odom) {
            return;
        }
        if (!fb.valid || p.wheel_count == 0) {
            return;
        }

        // Compute dt
        double dt = now_time_sec - odom.last_update_time_sec;
        if (odom.last_update_time_sec <= 0.0 || dt <= 0.0) {
            // First call or invalid dt: just set timestamp and return
            odom.last_update_time_sec = now_time_sec;
            return;
        }

        // Compute robot-level twist from wheel speeds
        double v = 0.0;
        double w = 0.0;
        compute_robot_twist_from_wheels(v, w);

        // Store twist in odom
        odom.linear  = v;
        odom.angular = w;
        
        // Skip integration for very small time intervals to avoid numerical issues
        if (dt < 1e-6) {
            odom.last_update_time_sec = now_time_sec;
            return;
        }


        const double x_old = odom.x;
        const double y_old = odom.y;
        const double yaw_old = odom.yaw;

        if (fabs(w) < 1e-6) {

            odom.x = x_old + v * dt * cos(yaw_old);
            odom.y = y_old + v * dt * sin(yaw_old);
            odom.yaw = yaw_old + w * dt;
        } else {

            const double delta_theta = w * dt;
            const double radius = v / w; 

            const double delta_x_local = radius * sin(delta_theta);
            const double delta_y_local = radius * (1.0 - cos(delta_theta));

            odom.x = x_old + delta_x_local * cos(yaw_old) - delta_y_local * sin(yaw_old);
            odom.y = y_old + delta_x_local * sin(yaw_old) + delta_y_local * cos(yaw_old);
            odom.yaw = yaw_old + delta_theta;
        }

        odom.yaw = normalize_angle(odom.yaw);

        // Update timestamp for next iteration
        odom.last_update_time_sec = now_time_sec;
    }

protected:
    ControlState state_;

    static double normalize_angle(double yaw)
    {
        // Normalize angle to [-pi, pi]
        const double two_pi = 2.0 * 3.14159265358979323846;
        while (yaw >  3.14159265358979323846) yaw -= two_pi;
        while (yaw < -3.14159265358979323846) yaw += two_pi;
        return yaw;
    }
};

} // namespace antobot_control