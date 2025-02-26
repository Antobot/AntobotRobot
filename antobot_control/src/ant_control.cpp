#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

//#include "antobot_control/ant_control_demo.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;



class AntobotControl : public rclcpp::Node
{
  public:
    AntobotControl() : Node("antobot_control"), count_(0)
    {
        sub_robot_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>("/antobot/robot/cmd_vel", 10,
            std::bind(&AntobotControl::robot_cmd_vel_callback, this, _1));
        sub_wheel_vel_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/antobot/bridge/wheel_vel", 10, 
            std::bind(&AntobotControl::wheel_vel_callback, this, _1));

        pub_wheel_vel_cmd_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/antobot/control/wheel_vel_cmd", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&AntobotControl::timer_callback, this));

        get_robot_description();
    }

  private:
    
    // Variable definitions
  
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_wheel_vel_cmd_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_wheel_vel_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_robot_cmd_vel_;

    size_t count_;

    float robot_lin_vel_cmd;
    float robot_ang_vel_cmd;
    float wheel_vels[4];
    std::vector<float> wheel_vel_cmd;
    float wheel_base;                       // The distance between the left and right wheels, in meters
    float wheel_radius;


    // Functions

    void timer_callback()
    {
        auto wheel_vel_cmd_msg = std_msgs::msg::Float32MultiArray();
        get_motor_commands(robot_lin_vel_cmd, robot_ang_vel_cmd);
        //wheel_vel_cmd_msg.layout = 
        wheel_vel_cmd_msg.data = wheel_vel_cmd;

        pub_wheel_vel_cmd_->publish(wheel_vel_cmd_msg);
    }

    void wheel_vel_callback(const std_msgs::msg::Float32MultiArray &msg)
    {
        RCLCPP_INFO(this->get_logger(), "in wheel vel callback");
    }

    void robot_cmd_vel_callback(const geometry_msgs::msg::Twist &msg)
    {
        RCLCPP_INFO(this->get_logger(), "in robot cmd vel callback");
        robot_lin_vel_cmd = msg.linear.x;
        robot_ang_vel_cmd = msg.angular.z;
    }

    void get_robot_description()
    {
        /* Should be read from URDF or other configuration file */

        wheel_base = 0.6;
        wheel_radius = 0.165;
    }
    
    void get_motor_commands(float lin_vel, float ang_vel)
    {
        float wheel_ang_vel_l = 0;
        float wheel_ang_vel_r = 0;

        wheel_vel_cmd = std::vector<float>();

        wheel_ang_vel_l = (lin_vel + ang_vel * wheel_base/2)/wheel_radius;
        wheel_ang_vel_r = (lin_vel - ang_vel * wheel_base/2)/wheel_radius;

        wheel_vel_cmd.push_back((float)wheel_ang_vel_l);
        wheel_vel_cmd.push_back((float)wheel_ang_vel_l);
        wheel_vel_cmd.push_back((float)wheel_ang_vel_r);
        wheel_vel_cmd.push_back((float)wheel_ang_vel_r);

    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AntobotControl>());
    rclcpp::shutdown();
    return 0;
}