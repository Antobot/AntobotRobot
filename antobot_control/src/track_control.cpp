#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"


#include "antobot_control/anto_control_base.hpp"

using namespace std::chrono_literals;

class TrackControlNode
    : public rclcpp::Node
    , public antobot_control::AntoControlBase
{
public:

    TrackControlNode()
    : Node("track_control")
    {
        // Declare ROS parameters
        this->declare_parameter<double>("max_motor_rpm",        3000.0);
        this->declare_parameter<double>("gear_ratio",           40.0);
        this->declare_parameter<double>("sprocket_diameter_m",  0.210);
        this->declare_parameter<double>("body_width_m",         0.830);
        this->declare_parameter<double>("track_width_m",        0.150);

        this->declare_parameter<double>("spot_turn_v_thresh",   0.02);
        this->declare_parameter<double>("spot_turn_w_bias",     1.0);

        this->declare_parameter<double>("deadband",             0.02);
        this->declare_parameter<double>("ramp_limit_per_sec",   2.0);

        this->declare_parameter<bool>("use_teleop_topic",       true);

        // AntoControlBase-related parameters
        this->declare_parameter<double>("control_frequency_hz", 30.0);
        this->declare_parameter<double>("velocity_timeout_sec", 0.1);

        this->declare_parameter<double>("max_linear",           0.5);
        this->declare_parameter<double>("min_linear",          -0.5);
        this->declare_parameter<double>("max_angular",          0.5);
        this->declare_parameter<double>("min_angular",         -0.5);

        this->declare_parameter<double>("max_linear_accel",     0.2);
        this->declare_parameter<double>("max_linear_decel",    -3.0);
        this->declare_parameter<double>("max_angular_accel",    0.5);
        this->declare_parameter<double>("max_angular_decel",   -3.0);

        this->declare_parameter<bool>("enable_smoothing",       true);
        this->declare_parameter<bool>("enable_timeout",         true);
        this->declare_parameter<bool>("enable_odom",            true);

        // Topic and frame parameters
        this->declare_parameter<std::string>("cmd_vel_topic",        "/antobot/robot/cmd_vel");
        this->declare_parameter<std::string>("teleop_cmd_vel_topic", "/antobot/teleop/cmd_vel");
        this->declare_parameter<std::string>("track_cmd_topic",      "/antobot/track/vel");
        this->declare_parameter<std::string>("track_status_topic",   "/antobot/track/status");
        this->declare_parameter<std::string>("odom_topic",           "/antobot/robot/odom");
        this->declare_parameter<std::string>("odom_frame_id",        "odom");
        this->declare_parameter<std::string>("base_frame_id",        "base_link");

        // Get ROS parameter values
        this->get_parameter("max_motor_rpm",       max_motor_rpm_);
        this->get_parameter("gear_ratio",          gear_ratio_);
        this->get_parameter("sprocket_diameter_m", sprocket_diameter_m_);
        this->get_parameter("body_width_m",        body_width_m_);
        this->get_parameter("track_width_m",       track_width_m_);

        this->get_parameter("spot_turn_v_thresh",  spot_turn_v_thresh_);
        this->get_parameter("spot_turn_w_bias",    spot_turn_w_bias_);

        this->get_parameter("deadband",            deadband_);
        this->get_parameter("ramp_limit_per_sec",  ramp_limit_per_sec_);

        this->get_parameter("use_teleop_topic",    use_teleop_topic_);

        this->get_parameter("control_frequency_hz", control_frequency_hz_);
        this->get_parameter("velocity_timeout_sec", velocity_timeout_sec_);

        this->get_parameter("max_linear",          max_linear_);
        this->get_parameter("min_linear",          min_linear_);
        this->get_parameter("max_angular",         max_angular_);
        this->get_parameter("min_angular",         min_angular_);

        this->get_parameter("max_linear_accel",    max_linear_accel_);
        this->get_parameter("max_linear_decel",    max_linear_decel_);
        this->get_parameter("max_angular_accel",   max_angular_accel_);
        this->get_parameter("max_angular_decel",   max_angular_decel_);

        this->get_parameter("enable_smoothing", enable_smoothing_);
        this->get_parameter("enable_timeout",   enable_timeout_);
        this->get_parameter("enable_odom",      enable_odom_);

        this->get_parameter("cmd_vel_topic",        cmd_vel_topic_);
        this->get_parameter("teleop_cmd_vel_topic", teleop_cmd_vel_topic_);
        this->get_parameter("track_cmd_topic",      track_cmd_topic_);
        this->get_parameter("track_status_topic",   track_status_topic_);
        this->get_parameter("odom_topic",           odom_topic_);
        this->get_parameter("odom_frame_id",        odom_frame_id_);
        this->get_parameter("base_frame_id",        base_frame_id_);

        track_center_dist_m_ = body_width_m_ - track_width_m_;

        antobot_control::ControlParams params;
        params.wheel_count          = 2;  // two tracks: left and right
        params.track_width          = track_center_dist_m_;      // [m]
        params.wheel_radius         = sprocket_diameter_m_ * 0.5; // [m]

        params.control_frequency_hz = control_frequency_hz_;
        params.velocity_timeout_sec = velocity_timeout_sec_;

        params.max_linear   = max_linear_;
        params.min_linear   = min_linear_;
        params.max_angular  = max_angular_;
        params.min_angular  = min_angular_;

        params.max_linear_accel   = max_linear_accel_;
        params.max_linear_decel   = max_linear_decel_;
        params.max_angular_accel  = max_angular_accel_;
        params.max_angular_decel  = max_angular_decel_;

        params.enable_smoothing   = enable_smoothing_;
        params.enable_timeout     = enable_timeout_;
        params.enable_odom        = enable_odom_;

        setParams(params);

        // Setup ROS publishers
        track_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            track_cmd_topic_, 10);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            odom_topic_, 10);

        // Setup ROS subscribers
        // Choose robot cmd topic: teleop or normal robot cmd_vel
        std::string cmd_topic =
            use_teleop_topic_ ? teleop_cmd_vel_topic_ : cmd_vel_topic_;

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_topic, 10,
            std::bind(&TrackControlNode::cmdVelCallback, this, std::placeholders::_1));

        // Subscribe to track status, which contains motor RPM feedback
        track_status_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            track_status_topic_, 10,
            std::bind(&TrackControlNode::trackStatusCallback, this, std::placeholders::_1));
        if (control_frequency_hz_ > 0.0) {
            auto period = std::chrono::duration<double>(1.0 / control_frequency_hz_);
            control_timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                std::bind(&TrackControlNode::controlLoop, this));
        }

        last_ramp_time_ = this->now();
        prev_left_norm_  = 0.0;
        prev_right_norm_ = 0.0;

        const double k = (60.0 * gear_ratio_) / (M_PI * sprocket_diameter_m_);
        double v_max = 0.0;
        if (k > 0.0) {
            v_max = max_motor_rpm_ / k;
        }

        RCLCPP_INFO(this->get_logger(),
                    "TrackControlNode initialized. "
                    "max_motor_rpm=%.1f gear_ratio=%.2f sprocket_diameter=%.3f m",
                    max_motor_rpm_, gear_ratio_, sprocket_diameter_m_);
        RCLCPP_INFO(this->get_logger(),
                    "track_center_dist=%.3f m, spot_turn_v_thresh=%.3f, deadband=%.3f, ramp_limit_per_sec=%.3f",
                    track_center_dist_m_, spot_turn_v_thresh_, deadband_, ramp_limit_per_sec_);
        RCLCPP_INFO(this->get_logger(),
                    "cmd_vel subscribed from '%s' (use_teleop_topic=%s)",
                    cmd_topic.c_str(), use_teleop_topic_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(),
                    "Publishing track cmd to '%s', odom to '%s'",
                    track_cmd_topic_.c_str(), odom_topic_.c_str());
        RCLCPP_INFO(this->get_logger(),
                    "Computed k=%.3f rpm/(m/s), v_max=%.3f m/s (%.2f km/h)",
                    k, v_max, v_max * 3.6);
    }

private:
    //cmd_vel callback: twist commands
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const double now_sec = this->now().seconds();
        robot_cmd_vel_callback(msg->linear.x, msg->angular.z, now_sec);
    }

    // Track status callback: read motor RPM feedback and update odometry
    void trackStatusCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Extract left/right motor RPMs from indices 2 and 3
        double left_rpm  = static_cast<double>(msg->data[2]);
        double right_rpm = static_cast<double>(msg->data[3]);

        std::vector<double> speeds = { left_rpm, right_rpm };
        const double now_sec = this->now().seconds();

        // Push wheel feedback into AntoControlBase
        wheel_cmd_vel_callback(speeds, now_sec);

        // Compute odometry update
        robot_odom_compute(now_sec);

        // Publish odometry
        publishOdom();
    }

        // Control loop: smoothing + wheel command computation + publishing
    void controlLoop()
    {
        double now_sec = this->now().seconds();

        velocity_smoother(now_sec);

        const auto & st = this->getState();
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "controlLoop: smoothed cmd v=%.3f m/s, w=%.3f rad/s (timed_out=%s)",
            st.smoothed_cmd.linear,
            st.smoothed_cmd.angular,
            st.status.timed_out ? "true" : "false");

        wheel_speed_compute();

        const auto & wheel_cmd = this->getState().wheel_cmd;
        std::size_t n = wheel_cmd.target_speed.size();
        if (n < 2) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "controlLoop: wheel_cmd.target_speed size = %zu (<2), nothing to publish", n);
            return;
        }

        double uL = wheel_cmd.target_speed[0];
        double uR = wheel_cmd.target_speed[1];

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "Publishing track cmd to '%s': [uL=%.3f, uR=%.3f]",
            track_cmd_topic_.c_str(), uL, uR);

        std_msgs::msg::Float32MultiArray out;
        out.data.resize(2);
        out.data[0] = static_cast<float>(uL);
        out.data[1] = static_cast<float>(uR);
        track_cmd_pub_->publish(out);
    }


    //Implementation of AntoControlBase virtual: twist -> track commands

    void wheel_speed_compute() override
    {
        auto & state = this->state_;

        double vx = state.smoothed_cmd.linear;   // [m/s]
        double wz = state.smoothed_cmd.angular;  // [rad/s]

        // Spot-turn behavior: if linear is small, force pure rotation and
        // optionally boost angular velocity.
        if (std::fabs(vx) < spot_turn_v_thresh_) {
            vx = 0.0;
            wz = wz * spot_turn_w_bias_;
        }

        // Differential-drive kinematics for tracks:
        //   vL = vx - wz * W/2
        //   vR = vx + wz * W/2
        const double W = track_center_dist_m_;
        double vL = vx - wz * W * 0.5;
        double vR = vx + wz * W * 0.5;

        // Convert linear speed to motor RPM:
        //   motor_rpm = v * (60*G / (pi*D)) = v * k
        const double k = (60.0 * gear_ratio_) / (M_PI * sprocket_diameter_m_);
        double nL = vL * k;
        double nR = vR * k;

        // Normalize RPM to [-1, 1] based on max_motor_rpm_
        double uL_raw = 0.0;
        double uR_raw = 0.0;
        if (max_motor_rpm_ > 0.0) {
            uL_raw = nL / max_motor_rpm_;
            uR_raw = nR / max_motor_rpm_;
        }

        // Clip overall magnitude to at most 1 by scaling both sides together
        double s = std::max(1.0, std::max(std::fabs(uL_raw), std::fabs(uR_raw)));
        double uL = uL_raw / s;
        double uR = uR_raw / s;

        // Apply deadband
        if (std::fabs(uL) < deadband_) uL = 0.0;
        if (std::fabs(uR) < deadband_) uR = 0.0;

        // Additional ramp (slew-rate) limiting on normalized commands
        rclcpp::Time now = this->now();
        double dt = (now - last_ramp_time_).seconds();
        if (dt < 0.0) dt = 0.0;
        last_ramp_time_ = now;

        double max_step = ramp_limit_per_sec_ * dt;
        auto clamp_step = [max_step](double target, double prev) {
            double lo = prev - max_step;
            double hi = prev + max_step;
            if (target < lo) return lo;
            if (target > hi) return hi;
            return target;
        };

        double uL_smooth = clamp_step(uL, prev_left_norm_);
        double uR_smooth = clamp_step(uR, prev_right_norm_);

        prev_left_norm_  = uL_smooth;
        prev_right_norm_ = uR_smooth;

        // Write normalized commands into AntoControlBase wheel_cmd
        if (state.params.wheel_count >= 2) {
            state.wheel_cmd.target_speed[0] = uL_smooth; // left track command
            state.wheel_cmd.target_speed[1] = uR_smooth; // right track command
        }
    }

    //Implementation of AntoControlBase virtual: wheel RPM -> robot twist
 
    void compute_robot_twist_from_wheels(double & linear, double & angular) override
    {
        auto & state = this->state_;

        if (!state.wheel_feedback.valid || state.params.wheel_count < 2) {
            linear  = 0.0;
            angular = 0.0;
            return;
        }

        // Extract RPM values from feedback
        double nL = state.wheel_feedback.measured_speed[0];
        double nR = state.wheel_feedback.measured_speed[1];

        // Convert RPM to linear wheel speed (m/s)
        const double k = (60.0 * gear_ratio_) / (M_PI * sprocket_diameter_m_);
        if (k <= 0.0 || track_center_dist_m_ <= 0.0) {
            linear  = 0.0;
            angular = 0.0;
            return;
        }

        double vL = nL / k;
        double vR = nR / k;

        // Differential-drive inverse kinematics
        linear  = 0.5 * (vL + vR);
        angular = (vR - vL) / track_center_dist_m_;
    }
    //Odometry publisher: convert AntoControlBase odom state to ROS message

    void publishOdom()
    {
        const auto & odom_state = this->state_.odom;

        nav_msgs::msg::Odometry msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = odom_frame_id_;
        msg.child_frame_id  = base_frame_id_;

        // Position
        msg.pose.pose.position.x = odom_state.x;
        msg.pose.pose.position.y = odom_state.y;
        msg.pose.pose.position.z = 0.0;

        // Orientation: yaw -> quaternion (rotation about Z-axis)
        double yaw = odom_state.yaw;
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = sy;
        msg.pose.pose.orientation.w = cy;

        // Twist
        msg.twist.twist.linear.x  = odom_state.linear;
        msg.twist.twist.linear.y  = 0.0;
        msg.twist.twist.linear.z  = 0.0;
        msg.twist.twist.angular.x = 0.0;
        msg.twist.twist.angular.y = 0.0;
        msg.twist.twist.angular.z = odom_state.angular;

        // Covariances (can be tuned as needed)
        for (int i = 0; i < 36; ++i) {
            msg.pose.covariance[i]  = 0.0;
            msg.twist.covariance[i] = 0.0;
        }

        odom_pub_->publish(msg);
    }

private:

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr track_status_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr track_cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    double max_motor_rpm_{3000.0};
    double gear_ratio_{40.0};
    double sprocket_diameter_m_{0.210};
    double body_width_m_{0.830};
    double track_width_m_{0.150};
    double track_center_dist_m_{0.0};

    double spot_turn_v_thresh_{0.02};
    double spot_turn_w_bias_{1.0};

    double deadband_{0.02};
    double ramp_limit_per_sec_{2.0};

    bool use_teleop_topic_{false};

    double control_frequency_hz_{30.0};
    double velocity_timeout_sec_{0.1};

    double max_linear_{0.5};
    double min_linear_{-0.5};
    double max_angular_{0.5};
    double min_angular_{-0.5};

    double max_linear_accel_{0.2};
    double max_linear_decel_{-3.0};
    double max_angular_accel_{0.5};
    double max_angular_decel_{-3.0};

    bool enable_smoothing_{true};
    bool enable_timeout_{true};
    bool enable_odom_{true};

    std::string cmd_vel_topic_;
    std::string teleop_cmd_vel_topic_;
    std::string track_cmd_topic_;
    std::string track_status_topic_;
    std::string odom_topic_;
    std::string odom_frame_id_;
    std::string base_frame_id_;

    rclcpp::Time last_ramp_time_;
    double prev_left_norm_{0.0};
    double prev_right_norm_{0.0};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
