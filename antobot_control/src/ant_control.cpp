#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;



class AntobotControl : public rclcpp::Node
{
  public:
    AntobotControl() : Node("antobot_control"), count_(0)
    {
      
        sub_wheel_vel_ = this->create_subscription<std_msgs::msg::String>("/antobot/bridge/wheel_vel", 10, 
            std::bind(&AntobotControl::wheel_vel_callback, this, _1));
        sub_robot_cmd_vel_ = this->create_subscription<std_msgs::msg::String>("/antobot/robot/cmd_vel", 10,
            std::bind(&AntobotControl::robot_cmd_vel_callback, this, _1));


        pub_wheel_vel_cmd_ = this->create_publisher<std_msgs::msg::String>("/antobot/control/wheel_vel_cmd", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&AntobotControl::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        pub_wheel_vel_cmd_->publish(message);
    }

    void wheel_vel_callback(const std_msgs::msg::String &msg) const
    {
        RCLCPP_INFO(this->get_logger(), "in wheel vel callback");
    }

    void robot_cmd_vel_callback(const std_msgs::msg::String &msg) const
    {
        RCLCPP_INFO(this->get_logger(), "in robot cmd vel callback");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_wheel_vel_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_wheel_vel_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_robot_cmd_vel_;

    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AntobotControl>());
    rclcpp::shutdown();
    return 0;
}