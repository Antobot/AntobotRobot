#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//#include "std_msgs/msg/float32_multi_array.hpp"
#include "antobot_platform_msgs/msg/float32_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
//#include "geometry_msgs/msg/pose_with_covariance.hpp"
//#include "geometry_msgs/msg/twist_with_covariance.hpp"

//#include "antobot_control/ant_control_demo.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;



class AntobotControl : public rclcpp::Node
{
  public:
    AntobotControl() : Node("antobot_control"), count_(0)
    {   

        this->declare_parameter<double>("wheel_base", 0.6);
        this->declare_parameter<double>("wheel_radius", 0.165);
        this->declare_parameter<std::vector<double>>("max_velocity", std::vector<double>{0.5, 0.5});
        this->declare_parameter<std::vector<double>>("min_velocity", std::vector<double>{-0.5, -0.5});
        this->declare_parameter<std::vector<double>>("max_accel", std::vector<double>{1.0, 1.0});
        this->declare_parameter<std::vector<double>>("max_decel", std::vector<double>{-1.0, 1.0});
        this->declare_parameter<double>("frequency", 30.0);
        this->declare_parameter<double>("velocity_timeout", 0.1);

        wheel_base_  = this->get_parameter("wheel_base").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        max_velocities_ = this->get_parameter("max_velocity").as_double_array();
        min_velocities_ = this->get_parameter("min_velocity").as_double_array();
        max_accels_ = this->get_parameter("max_accel").as_double_array();
        max_decels_ = this->get_parameter("max_decel").as_double_array();
        frequency_ = this->get_parameter("frequency").as_double();
        velocity_timeout_ = this->get_parameter("velocity_timeout").as_double();

        sub_robot_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>("/antobot/robot/cmd_vel", 10,
            std::bind(&AntobotControl::robot_cmd_vel_callback, this, _1));
        sub_wheel_vel_ = this->create_subscription<antobot_platform_msgs::msg::Float32Array>("/antobot/bridge/wheel_vel", 10, 
            std::bind(&AntobotControl::wheel_vel_callback, this, _1));

        pub_wheel_vel_cmd_ = this->create_publisher<antobot_platform_msgs::msg::Float32Array>("/antobridge/wheel_vel_cmd", 10);
        pub_wheel_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/antobot/robot/odometry", 10);
        
        std::chrono::duration<double> period_sec(1.0 / frequency_);
        timer_ = this->create_wall_timer(period_sec, std::bind(&AntobotControl::timer_callback, this));

        last_command_time_ = this->now();
        //get_robot_description();

    }

  private:
    
    // Variable definitions
  
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<antobot_platform_msgs::msg::Float32Array>::SharedPtr pub_wheel_vel_cmd_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_wheel_odom_;
    rclcpp::Subscription<antobot_platform_msgs::msg::Float32Array>::SharedPtr sub_wheel_vel_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_robot_cmd_vel_;

    size_t count_;

    float robot_lin_vel_cmd;
    float robot_ang_vel_cmd;
    float wheel_vels[4];
    std::vector<float> wheel_vel_cmd;
    double wheel_base_;                       // The distance between the left and right wheels, in meters
    double wheel_radius_;
    nav_msgs::msg::Odometry wheel_odom_msg;
    float old_angle;
    geometry_msgs::msg::Point old_pos;

    bool sim = true;

    double frequency_, velocity_timeout_;

    // smooth
    rclcpp::Time last_command_time_;
    std::vector<double> max_velocities_, min_velocities_, max_accels_, max_decels_;
    double robot_lin_vel_cmd_last, robot_ang_vel_cmd_last;

    // Functions
    void timer_callback()
    {   
        // Check for velocity timeout
        double time_since_cmd = (this->now() - last_command_time_).seconds();
        if (time_since_cmd > velocity_timeout_)
        {
            if (robot_lin_vel_cmd != 0.0 || robot_ang_vel_cmd != 0.0)
            {
                RCLCPP_WARN(this->get_logger(), "No cmd_vel received for %.3f s, stop robot.", time_since_cmd);
            }
            robot_lin_vel_cmd = 0.0;
            robot_ang_vel_cmd = 0.0;
        }

        auto wheel_vel_cmd_msg = antobot_platform_msgs::msg::Float32Array();
        get_motor_commands(robot_lin_vel_cmd, robot_ang_vel_cmd);
        wheel_vel_cmd_msg.data = wheel_vel_cmd;

        wheel_odom_msg = nav_msgs::msg::Odometry();
        get_wheel_odom();
        
        pub_wheel_vel_cmd_->publish(wheel_vel_cmd_msg);
        pub_wheel_odom_->publish(wheel_odom_msg);
    }

    void wheel_vel_callback(const antobot_platform_msgs::msg::Float32Array &msg)
    {
        //RCLCPP_INFO(this->get_logger(), "in wheel vel callback");
        wheel_vels[0] = msg.data[0];
        wheel_vels[1] = msg.data[1];
        wheel_vels[2] = msg.data[2];
        wheel_vels[3] = msg.data[3];
    }

    void robot_cmd_vel_callback(const geometry_msgs::msg::Twist &msg)
    {
        //RCLCPP_INFO(this->get_logger(), "in robot cmd vel callback");
        robot_lin_vel_cmd = msg.linear.x;
        robot_ang_vel_cmd = -1.0*msg.angular.z;

        velocity_smoother();
    }

    // refer: https://github.com/ros-navigation/navigation2/blob/main/nav2_smoother/src/nav2_smoother.cpp
    void velocity_smoother()
    {
        last_command_time_ = this->now();
        float robot_lin_vel_cmd_current, robot_ang_vel_cmd_current;
        robot_lin_vel_cmd_current = robot_lin_vel_cmd_last;
        robot_ang_vel_cmd_current = robot_ang_vel_cmd_last;


        robot_lin_vel_cmd = std::clamp(robot_lin_vel_cmd, static_cast<float>(min_velocities_[0]), static_cast<float>(max_velocities_[0]));
        robot_ang_vel_cmd = std::clamp(robot_ang_vel_cmd, static_cast<float>(min_velocities_[1]), static_cast<float>(max_velocities_[1]));

        float eta = 1.0;
    
        robot_lin_vel_cmd = applyConstraints(robot_lin_vel_cmd_current, robot_lin_vel_cmd, static_cast<float>(max_accels_[0]), static_cast<float>(max_decels_[0]), eta);
        robot_ang_vel_cmd = applyConstraints(robot_ang_vel_cmd_current, robot_ang_vel_cmd, static_cast<float>(max_accels_[1]), static_cast<float>(max_decels_[1]), eta);
        
        robot_lin_vel_cmd_last = robot_lin_vel_cmd;
        robot_ang_vel_cmd_last = robot_ang_vel_cmd;

    }

    
    float applyConstraints(const float v_curr, const float v_cmd, const float accel, const float decel, const float eta)
    {
        float dv = v_cmd - v_curr;

        float v_component_max;
        float v_component_min;

        // Accelerating if magnitude of v_cmd is above magnitude of v_curr
        // and if v_cmd and v_curr have the same sign (i.e. speed is NOT passing through 0.0)
        // Decelerating otherwise
        if (abs(v_cmd) >= abs(v_curr) && v_curr * v_cmd >= 0.0) {
            v_component_max = accel / frequency_;
            v_component_min = -accel / frequency_;
        } else {
            v_component_max = -decel / frequency_;
            v_component_min = decel / frequency_;
        }

        return v_curr + std::clamp(eta * dv, v_component_min, v_component_max);
    }

    void get_robot_description()
    {
        /* Should read from URDF or other configuration file */

        wheel_base_ = 0.6;
        wheel_radius_ = 0.165;
    }
    
    void get_motor_commands(float lin_vel, float ang_vel)
    {
        float wheel_ang_vel_l = 0;
        float wheel_ang_vel_r = 0;

        wheel_vel_cmd = std::vector<float>();

        wheel_ang_vel_l = (lin_vel + ang_vel * wheel_base_/2)/wheel_radius_;
        wheel_ang_vel_r = (lin_vel - ang_vel * wheel_base_/2)/wheel_radius_;

        // TODO: Unit conversions (?)

        wheel_vel_cmd.push_back((float)wheel_ang_vel_l);
        wheel_vel_cmd.push_back((float)wheel_ang_vel_l);
        wheel_vel_cmd.push_back((float)wheel_ang_vel_r);
        wheel_vel_cmd.push_back((float)wheel_ang_vel_r);

    }

    void get_wheel_odom()
    {
        
        geometry_msgs::msg::TwistWithCovariance twist_cov;
        geometry_msgs::msg::PoseWithCovariance pose_cov;
        
        twist_cov.twist = calc_twist();
        std::array<double, 36> twist_cov_cov;
        twist_cov_cov =         {0.001, 0.0,    0.0,    0.0,    0.0,    0.0, 
                                 0.0,   0.001,  0.0,    0.0,    0.0,    0.0,
                                 0.0,   0.0,    0.001,  0.0,    0.0,    0.0, 
                                 0.0,   0.0,    0.0,    0.001,  0.0,    0.0, 
                                 0.0,   0.0,    0.0,    0.0,    0.001,  0.0, 
                                 0.0,   0.0,    0.0,    0.0,    0.0,    0.03};
        twist_cov.covariance = twist_cov_cov;

        pose_cov.pose = calc_odom(twist_cov.twist.linear.x, twist_cov.twist.angular.z);
        std::array<double, 36> pose_cov_cov;
        pose_cov_cov =          {0.001, 0.0,    0.0,    0.0,    0.0,    0.0, 
                                 0.0,   0.001,  0.0,    0.0,    0.0,    0.0,
                                 0.0,   0.0,    0.001,  0.0,    0.0,    0.0, 
                                 0.0,   0.0,    0.0,    0.001,  0.0,    0.0, 
                                 0.0,   0.0,    0.0,    0.0,    0.001,  0.0, 
                                 0.0,   0.0,    0.0,    0.0,    0.0,    0.03};
        pose_cov.covariance = pose_cov_cov;
        
        wheel_odom_msg.header.stamp = this->now();
        wheel_odom_msg.header.frame_id = "odom";
        wheel_odom_msg.child_frame_id = "base_link";
        wheel_odom_msg.pose = pose_cov;
        wheel_odom_msg.twist = twist_cov;
    }

    geometry_msgs::msg::Twist calc_twist()
    {
        auto twist_odom = geometry_msgs::msg::Twist();
        auto linear_twist = geometry_msgs::msg::Vector3();
        auto angular_twist = geometry_msgs::msg::Vector3();

        float wheel_ang_vel_l;
        float wheel_ang_vel_r;

        if (sim)
        {
            wheel_vels[0] = wheel_vel_cmd[0];
            wheel_vels[1] = wheel_vel_cmd[1];
            wheel_vels[2] = wheel_vel_cmd[2];
            wheel_vels[3] = wheel_vel_cmd[3];
        }

        wheel_ang_vel_l = (wheel_vels[0] + wheel_vels[1]) / 2;    // Take simple average of most recent wheel velocity information
        wheel_ang_vel_r = (wheel_vels[2] + wheel_vels[3]) / 2;    

        // wheel_ang_vel_l + wheel_ang_vel_r = 2 * lin_vel / wheel_radius_                  // (from equations in get_motor_commands)
        linear_twist.x = wheel_radius_ * (wheel_ang_vel_l + wheel_ang_vel_r) / 2;

        // wheel_ang_vel_l - wheel_ang_vel_r = 2 * ang_vel * wheel_base_/2/wheel_radius_     // (from equations in get_motor_commands)
        angular_twist.z = (wheel_ang_vel_l - wheel_ang_vel_r) * wheel_radius_ / wheel_base_;

        twist_odom.linear = linear_twist;
        twist_odom.angular = angular_twist;

        return twist_odom;

    }

    geometry_msgs::msg::Pose calc_odom(float lin_vel, float ang_vel)
    {
        auto pose_odom = geometry_msgs::msg::Pose();
        auto new_pos = geometry_msgs::msg::Point();
        auto new_quat = geometry_msgs::msg::Quaternion();

        float elapsed_time = 0.04;

        // Calculate new angle
        float new_angle = old_angle + elapsed_time * ang_vel;
        old_angle = new_angle;
        new_quat.x = 0;      // Assume 0 because both roll and pitch are 0
        new_quat.y = 0;      // Assume 0 because both roll and pitch are 0
        new_quat.z = sin(new_angle/2);
        new_quat.w = cos(new_angle/2);
        pose_odom.orientation = new_quat;

        // Calculate new position
        new_pos.x = old_pos.x + lin_vel * elapsed_time * cos(new_angle);
        new_pos.y = old_pos.y + lin_vel * elapsed_time * sin(new_angle);
        pose_odom.position = new_pos;
        old_pos = new_pos;

        return pose_odom;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AntobotControl>());
    rclcpp::shutdown();
    return 0;
}
