#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "antobot_platform_msgs/msg/u_int16_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;



class AntobotSafety : public rclcpp::Node
{
  public:
    AntobotSafety() : Node("antobot_safety"), count_(0)
    {

        sub_safety_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>("/antobot/safety/cmd_vel", 10,
            std::bind(&AntobotSafety::safetyCmdVelCallback, this, _1));
        sub_uss_dist_ = this->create_subscription<antobot_platform_msgs::msg::UInt16Array>("/antobridge/uss_dist", 10,
            std::bind(&AntobotSafety::ussDistCallback, this, _1));
        sub_release_ = this->create_subscription<std_msgs::msg::Bool>("/antobridge/force_stop_release", 10, 
            std::bind(&AntobotSafety::releaseCallback, this, _1));
        sub_bump_front_ = this->create_subscription<std_msgs::msg::Bool>("/antobridge/bump_front", 10, 
            std::bind(&AntobotSafety::bumpFrontCallback, this, _1));
        sub_bump_back_ = this->create_subscription<std_msgs::msg::Bool>("/antobridge/bump_back", 10, 
            std::bind(&AntobotSafety::bumpBackCallback, this, _1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/antobot/robot/cmd_vel", 10);
        uss_dist_filt_pub_ = this->create_publisher<antobot_platform_msgs::msg::UInt16Array>("/antobot/safety/uss_dist", 10);
        force_stop_type_pub_ = this->create_publisher<std_msgs::msg::Int8>("/antobot/safety/force_stop_type", 10);       // 0 - none (or release); 
                                                                                                                        // 1-8: USS
                                                                                                                            // 1 - front left; 2 - front; 3 - front right; 4 - right; 
                                                                                                                            // 5 - back right; 6 - back; 7 - back left; 8 - left;
                                                                                                                        // 9: front bump stop; 10: back bump stop
        safe_operation_pub_ = this->create_publisher<std_msgs::msg::Bool>("/antobot/safety/safe_operation", 10);
        lights_f_pub_ = this->create_publisher<std_msgs::msg::Bool>("/antobridge/lights_f", 10);
        lights_b_pub_ = this->create_publisher<std_msgs::msg::Bool>("/antobridge/lights_b", 10);

        // Initialising uss_dist_filt with fake data
        antobot_platform_msgs::msg::UInt16Array uss_dist_filt_init;
        for (int i=0; i<8; i++)
        {
            uss_dist_filt_init.data.push_back(200);
        }
        uss_dist_filt = uss_dist_filt_init;
        

        this->declare_parameter<double>("frequency", 30.0);
        frequency_ = this->get_parameter("frequency").as_double();

        this->declare_parameter<bool>("uss_enable", false);
        uss_enable = this->get_parameter("uss_enable").as_bool();

        this->declare_parameter<bool>("auto_release", false);
        auto_release = this->get_parameter("auto_release").as_bool();

        this->declare_parameter<bool>("uss_front_enable", false);
        uss_front_enable = this->get_parameter("uss_front_enable").as_bool();

        this->declare_parameter<bool>("uss_back_enable", false);
        uss_back_enable = this->get_parameter("uss_back_enable").as_bool();

        std::chrono::duration<double> period_sec(1.0 / frequency_);
        timer_ = this->create_wall_timer(period_sec, std::bind(&AntobotSafety::update, this));

        // RCLCPP_INFO_STREAM(this->get_logger(), "SF0105: frequency_: " << frequency_);
        // RCLCPP_INFO_STREAM(this->get_logger(), "SF0105: uss_enable: " << uss_enable);

    }

  private:
    
    // Variable definitions
  
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<antobot_platform_msgs::msg::UInt16Array>::SharedPtr uss_dist_filt_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr force_stop_type_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safe_operation_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lights_f_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lights_b_pub_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_safety_cmd_vel_;
    rclcpp::Subscription<antobot_platform_msgs::msg::UInt16Array>::SharedPtr sub_uss_dist_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_release_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_bump_front_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_bump_back_;

    
    size_t count_;

    std::vector<std::vector<int>> uss_dist_windows;
    std::vector<bool> light_cmd_arr = {false, false};

    float time_to_collision = 100;
    float lin_vel_thresh = 0.11;
    float ang_vel_thresh = 1.0;             // The ratio of angular/linear velocity required to be considered turning
    float time_collision_thresh = 1.2;      // (seconds to collision) Threshold could depend on operation type?
    int hard_dist_thresh = 75;          // (cm)
    int hard_dist_thresh_diag = 30;          // (cm) - for diagonals 
    bool not_safe = false;
    int force_stop_type = 0;
    bool force_stop_bump = false;

    int active_cmd_vel;
    float prev_linear_vel;
    float prev_angular_vel;
    float linear_vel;
    float angular_vel;
    geometry_msgs::msg::Twist cmd_vel_msg;
    clock_t t_lastRcvdCmdVel;
    clock_t t_lastStopTriggerWarning;
    clock_t t_lastSafetyStatusSent;

    std::chrono::time_point<std::chrono::steady_clock> time_lastRcvdCmdVel = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> time_lastStopTriggerWarning = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> time_lastSafetyStatusSent = std::chrono::steady_clock::now();

    std::chrono::time_point<std::chrono::steady_clock> time_force_stop = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> time_safety_light = std::chrono::steady_clock::now();

    bool force_stop;
    clock_t t_force_stop;       // Can be used to release the force stop, if desired
    float fs_release_thresh = 8.0;
    bool force_stop_release;
    clock_t t_release;
    bool fs_warn_msg_sent = true;
    bool fs_err_msg_sent = true;

    bool movement_scale = false;
    bool movement_limit = true;

    int safety_light_pattern = 1;
    float safety_light_freq = 2.0;
    clock_t t_safety_light;     // Can be used to make the lights blink, if desired


    antobot_platform_msgs::msg::UInt16Array uss_dist_filt;


    std::string robot_role;
    int safety_level;

    bool safe_operation;

    double frequency_;
    bool uss_enable = false;
    bool auto_release = false;
    bool uss_front_enable = true;
    bool uss_back_enable = true;

    /*
    float robot_lin_vel_cmd;
    float robot_ang_vel_cmd;
    float wheel_vels[4];
    std::vector<float> wheel_vel_cmd;
    float wheel_base;                       // The distance between the left and right wheels, in meters
    float wheel_radius;
    nav_msgs::msg::Odometry wheel_odom_msg;
    float old_angle;
    geometry_msgs::msg::Point old_pos;*/
    

    // Functions
    void update()
    {
        /*  Fixed update rate to check various safety inputs and broadcast the correct outputs
        */
        // Check USS recommendation
        if (safety_level != 1 && safety_level != 2 && safety_level != 5 && safety_level != 8 && uss_enable)   // Only consider USS for specific defined safety levels
        {
            if (ussDistSafetyCheck() && !force_stop && !force_stop_release) //Safety check not pass, not force stopped, no release // UNCOMMENT TO ENABLE USS!!
            {
                if (force_stop_type > 0)
                {
                    float vel_out = 0;
                    if (movement_scale)        // Scale the movement   
                        vel_out = scaleCmdVel();
                    else if (movement_limit)
                        vel_out = limitCmdVel();

                    // If it isn't safe to scale, force stop the robot
                    if (vel_out == 0)
                    {
                        force_stop = true;
                        t_force_stop = clock();         // Sets when the robot force stopped
                        time_force_stop = std::chrono::steady_clock::now();
                        time_safety_light = std::chrono::steady_clock::now();
                        t_safety_light = clock();
                        
                        fs_warn_msg_sent = false;
                        fs_err_msg_sent = false;
                    }
                    else
                    {
                        force_stop_type = 0;
                    }
                }
            
                else
                {
                    // If force stop is triggered while moving straight, force stop the robot ---what situation will enter this condition?July 4th
                    force_stop = true;
                    t_force_stop = clock();         // Sets when the robot force stopped
                    time_force_stop = std::chrono::steady_clock::now();
                    t_safety_light = clock();
                    time_safety_light = std::chrono::steady_clock::now();
                }
            }
        }
        

        // TODO: Check costmap recommendation - integrate with costmap-based obstacle detection?


        // Check time of last received command - if none received in the last ~1s, the robot should stop
        //if ((float)(clock() - t_lastRcvdCmdVel)/CLOCKS_PER_SEC > 0.05)      // This should NOT use ROS time, as if ROS stops, it should still stop the robot
        auto duration = std::chrono::steady_clock::now() - time_lastRcvdCmdVel;
        // auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
        // RCLCPP_INFO_STREAM(this->get_logger(), "SF0105: update_time" << duration_ms.count() << " ms");
        if (duration > std::chrono::milliseconds(50))
        {   
            //RCLCPP_INFO_STREAM(this->get_logger(), "SF0105: Robot stopped2" << (float)(clock() - t_lastRcvdCmdVel)/CLOCKS_PER_SEC);
            cmd_vel_msg.linear.x = 0;
            cmd_vel_msg.angular.z = 0;
            //if ((float)(clock() - t_lastStopTriggerWarning)/CLOCKS_PER_SEC > 10.0)
            
            auto duration = std::chrono::steady_clock::now() - time_lastStopTriggerWarning;
            //auto duration_s = std::chrono::duration_cast<std::chrono::seconds>(duration);
            if (duration > std::chrono::seconds(10))
            {
                RCLCPP_INFO(this->get_logger(), "SF0105: Robot stopped - no cmd_vel command received (10s)");
                //t_lastStopTriggerWarning = clock();
                time_lastStopTriggerWarning = std::chrono::steady_clock::now();
                safe_operation = false;
            }
        }

        // Robot is moving too quickly toward an obstacle
        if (force_stop || force_stop_bump)
        {
            lightsSafetyOut();	// Broadcast safety lights (if needed)

            // Set command velocity to 0
            cmd_vel_msg.linear.x = 0;
            cmd_vel_msg.angular.z = 0;

            if (30.0*(clock() - t_force_stop)/(float)CLOCKS_PER_SEC > 30.0 && !fs_err_msg_sent)
            {
                RCLCPP_ERROR(this->get_logger(), "SF010%d: Force stop for 30 seconds!", force_stop_type);  //  Current error code indicates USS force stop, but could also be bump switch
                fs_err_msg_sent = true;
                safe_operation = false;
            }
            else if (30.0*(clock() - t_force_stop)/(float)CLOCKS_PER_SEC > 10.0 && !fs_warn_msg_sent)
            {
                RCLCPP_WARN(this->get_logger(), "SF010%d: Force stop for 10 seconds!", force_stop_type);   // Current error code indicates USS force stop, but could also be bump switch
                fs_warn_msg_sent = true;
            }
        }

        prev_linear_vel = cmd_vel_msg.linear.x;
        prev_angular_vel = cmd_vel_msg.angular.z;

        cmd_vel_pub_->publish(cmd_vel_msg);

        //if (30.0*(clock() - t_lastSafetyStatusSent)/CLOCKS_PER_SEC > 1.0)   // Send status every 1 second

        duration = std::chrono::steady_clock::now() - time_lastSafetyStatusSent;
        //auto duration_s = std::chrono::duration_cast<std::chrono::seconds>(duration);
        if (duration >= std::chrono::seconds(1))
        {

            std_msgs::msg::Int8 force_stop_type_msg;
            force_stop_type_msg.data = force_stop_type;
            force_stop_type_pub_->publish(force_stop_type_msg);

            std_msgs::msg::Bool safe_operation_msg;
            safe_operation_msg.data = safe_operation;
            safe_operation_pub_->publish(safe_operation_msg);

            //t_lastSafetyStatusSent = clock();

            time_lastSafetyStatusSent = std::chrono::steady_clock::now();
        }

        autoRelease();
        
        if (30.0*(clock() - t_release)/(float)CLOCKS_PER_SEC > 0.5)
            force_stop_release = false;  
    }

    bool ussDistSafetyCheck()
    {
        /*  Checks whether, based on the ultrasonic sensor (USS) data, the robot should 
            slow down or stop */
        //  Inputs: uss_dist_filt <msgs::UInt16_Array> - 8-element array of filtered USS distances
        //          linear_vel <float> - robot commanded linear velocity
        //          angular_vel <float> - robot commanded angular velocity
        //  Returns: not_safe <bool> - true indicates the robot may collide with an object; false means safe movement is possible

        not_safe = false;
        
        if (linear_vel > lin_vel_thresh)    // Robot is moving forward
        {
            try
            {
                not_safe = ussDistSafetyCheck_f();
            }
            catch (int errorCode)
            {
                RCLCPP_ERROR(this->get_logger(), "SF0200: Unable to check safety with USS (forward movement)");
            }
            
        }
        else if (linear_vel < -lin_vel_thresh)  // Robot is moving backward
        {
            try
            {
                not_safe = ussDistSafetyCheck_b();
            }
            catch (int errorCode)
            {
                RCLCPP_ERROR(this->get_logger(), "SF0201: Unable to check safety with USS (backward movement)");
            }
        }
        else
        {
            // Spot turn
            if (angular_vel > ang_vel_thresh)
            {
                // Left turn
                /*if (uss_dist_filt.data[3] < hard_dist_thresh || uss_dist_filt.data[7] < hard_dist_thresh)
                {
                    // not_safe = true;
                    // force_stop_type = 7;
                }*/
            }
            else if (angular_vel < -ang_vel_thresh)
            {
                // Right turn
                /* if (uss_dist_filt.data[3] < hard_dist_thresh || uss_dist_filt.data[7] < hard_dist_thresh)
                {
                    // not_safe = true;
                    // force_stop_type = 8;
                }*/
                
            }
            else    // Robot is not moving
            {
                time_to_collision = 100.0;
            }
        }

        return not_safe;
    }

    bool ussDistSafetyCheck_f()
    {
        /*  Checks whether, based on the forward-facing ultrasonic sensor (USS) data, the robot should 
            slow down or stop */
        //  Inputs: uss_dist_filt <msgs::UInt16_Array> - 8-element array of filtered USS distances
        //          linear_vel <float> - robot commanded linear velocity
        //          angular_vel <float> - robot commanded angular velocity
        //  Returns: not_safe <bool> - true indicates the robot may collide with an object; false means safe movement is possible

        // force_stop_type: 1 - left front; 2 - straight front; 3 - right front
        bool not_safe_f = false;

        time_to_collision = (float)(uss_dist_filt.data[1])/(100.0*linear_vel);      // Check time to reach nearest obstacle to the robot's front
        if ((time_to_collision < time_collision_thresh || uss_dist_filt.data[1] < hard_dist_thresh) && uss_dist_filt.data[1] < 200)
        {
            not_safe_f = true;
            force_stop_type = 2;
            return not_safe_f;
        } 

        if (safety_level == 4 || safety_level == 7 || safety_level == 10)
        {
            if (angular_vel > ang_vel_thresh * linear_vel)           // Robot is turning left while moving forward
            {
                float time_to_collision_fl = (float)(uss_dist_filt.data[0])/(100.0*linear_vel);      // Check time to reach nearest obstacle to the robot's front left
                if ((time_to_collision_fl < time_collision_thresh || uss_dist_filt.data[0] < hard_dist_thresh_diag) && uss_dist_filt.data[0] < 200)
                {
                    force_stop_type = 1;
                    not_safe_f = true;
                    return not_safe_f;
                }
            }
            else if (angular_vel < -ang_vel_thresh * linear_vel)     // Robot is turning right while moving forward
            {
                float time_to_collision_fr = (float)(uss_dist_filt.data[2])/(100.0*linear_vel);      // Check time to reach nearest obstacle to the robot's front right
                if ((time_to_collision_fr < time_collision_thresh || uss_dist_filt.data[2] < hard_dist_thresh_diag) && uss_dist_filt.data[2] <200)
                {
                    force_stop_type = 3;
                    not_safe_f = true;
                    return not_safe_f;
                } 
            }
        }
        
        return not_safe_f;
    }

    bool ussDistSafetyCheck_b()
    {
        /*  Checks whether, based on the backward-facing ultrasonic sensor (USS) data, the robot should 
            slow down or stop */
        //  Inputs: uss_dist_filt <msgs::UInt16_Array> - 8-element array of filtered USS distances
        //          linear_vel <float> - robot commanded linear velocity
        //          angular_vel <float> - robot commanded angular velocity
        //  Returns: not_safe <bool> - true indicates the robot may collide with an object; false means safe movement is possible
        
        // force_stop_type: 7 - left back; 6 - straight back; 5 - right back
        bool not_safe_b = false;

        time_to_collision = (float)(uss_dist_filt.data[5])/(-100.0*linear_vel);      // Check time to reach nearest obstacle to the robot's back
        if ((time_to_collision < time_collision_thresh || uss_dist_filt.data[5] < hard_dist_thresh) && uss_dist_filt.data[5] < 200)
        {
            not_safe_b = true;
            force_stop_type = 6;
            return not_safe_b;
        } 

        if (safety_level == 4 || safety_level == 7 || safety_level == 10)
        {
            if (angular_vel > - ang_vel_thresh * linear_vel)
            {
                // Robot is moving back right
                float time_to_collision_br = (float)(uss_dist_filt.data[4])/(-100.0*linear_vel);      // Check time to reach nearest obstacle to the robot's back right
                if ((time_to_collision_br < time_collision_thresh || uss_dist_filt.data[4] < hard_dist_thresh_diag) && uss_dist_filt.data[4] < 200)
                {
                    force_stop_type = 5;
                    not_safe_b = true;
                    return not_safe_b;
                }      
            }
            else if (angular_vel <  ang_vel_thresh * linear_vel)
            {
                // Robot is moving back left
                float time_to_collision_bl = (float)(uss_dist_filt.data[6])/(-100.0*linear_vel);      // Check time to reach nearest obstacle to the robot's back left
                if ((time_to_collision_bl < time_collision_thresh || uss_dist_filt.data[6] < hard_dist_thresh_diag) && uss_dist_filt.data[6] < 200)
                {
                    force_stop_type = 7;
                    not_safe_b = true;
                    return not_safe_b;
                }   
            }
        }
        
        return not_safe_b;
    }

    void lightsSafetyOut()
    {
        /* Sends light commands to AntoBridge based on the set pattern
        */

        std_msgs::msg::Bool lights_f_cmd;
        std_msgs::msg::Bool lights_b_cmd;

        if (safety_light_pattern == 0)
        {
            // All lights on

            lights_f_cmd.data = true;
            lights_b_cmd.data = true;
        }
        if (safety_light_pattern == 1)
        {
            // Blinking with constant frequency (light_freq)
            lightCmdFreq();
            lights_f_cmd.data = light_cmd_arr[0];
            lights_b_cmd.data = light_cmd_arr[1];
        }
        
        // Publishes the data to ROS
        lights_f_pub_->publish(lights_f_cmd);
        lights_b_pub_->publish(lights_b_cmd);
    }

    void lightCmdFreq()
    {
        /* Sends light commands at a set frequency, defined in the class initialisation
        */

        int t_light_freq_thresh;
        t_light_freq_thresh = int(1.0/safety_light_freq * 1000);

        // If past a time threshold, lights will change state

        auto duration = std::chrono::steady_clock::now() - time_safety_light;
        //auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
        //RCLCPP_INFO_STREAM(this->get_logger(), "SF0105: lightCmdFreq: " << duration_ms.count() << " ms; t_light_freq_thresh: " << t_light_freq_thresh);
        // if (30.0*(clock() - t_safety_light)/(float)CLOCKS_PER_SEC > t_light_freq_thresh)
        if (duration > std::chrono::milliseconds(t_light_freq_thresh))
        {   
            //RCLCPP_INFO_STREAM(this->get_logger(), "SF0105: lightCmdFreq" << duration_ms.count() << " ms");
            light_cmd_arr[0] = !light_cmd_arr[0];
            light_cmd_arr[1] = !light_cmd_arr[1];
            t_safety_light = clock();
            time_safety_light = std::chrono::steady_clock::now();
        }
    }

    void autoRelease()
    {
        /* Automatically releases the robot from its force stopped state if the previously 
        detected object is no longer being detected */
        
        if (force_stop && auto_release)
        {
            // First, check how the robot is moving
            int cmd_vel_type = getCmdVelType();
            
            if (cmd_vel_type > 0)
            {     
                if (uss_dist_filt.data[cmd_vel_type - 1] > hard_dist_thresh)
                {
                    // Checks timer to re-start navigation (if safe)
                    if (30.0*(clock() - t_force_stop)/(float)CLOCKS_PER_SEC > fs_release_thresh)
                    {
                        // Force stop release if the time threshold has passed
                        force_stop = false;
                        force_stop_release = true;
                        force_stop_type = 0;
                        t_release = clock();
                    }
                }
                else
                {
                    t_force_stop = clock();     // Resets the timer if the object is still there
                    time_force_stop = std::chrono::steady_clock::now();
                }
            }
        }  
    }

    float scaleCmdVel()
    {
        float vel_scale = 0;

        vel_scale = calcVelScale();
        cmd_vel_msg.linear.x = vel_scale * cmd_vel_msg.linear.x;

        if (vel_scale > 0)
            RCLCPP_DEBUG(this->get_logger(), "SF010%d: Scaling linear velocity by %f", force_stop_type, vel_scale);
        else
            RCLCPP_INFO(this->get_logger(), "SF010%d: Force stop by USS!", force_stop_type);

        return vel_scale;  
    }

    float limitCmdVel()
    {
        float vel_scale = 0;
        vel_scale = calcVelScale();

        if (cmd_vel_msg.linear.x > vel_scale)
            cmd_vel_msg.linear.x = vel_scale;

        if (vel_scale > 0)
            RCLCPP_DEBUG(this->get_logger(), "SF010%d: Limiting linear velocity to %f", force_stop_type, vel_scale);
        else
            RCLCPP_INFO(this->get_logger(), "SF010%d: Force stop by USS!", force_stop_type);

        return vel_scale;
    }

    float calcVelScale()
    {
        /*  Calculates the magnitude by which to scale the velocity of the robot based on which ultrasonic sensor
            has detected an obstacle, and how far away that obstacle is. */
        //  Returns: vel_scale <float> the scale (between 0 and 1) by which the velocity will be scaled
        
        float vel_scale = 0;
        
        // Calculate scaling
        if (force_stop_type > 0)
        {
            int uss_data = uss_dist_filt.data[force_stop_type - 1];

            if(uss_data > 100 || uss_data == 0)
                vel_scale = 1;
            else if (uss_data > 53)
            {
                vel_scale = log10(float(uss_data-45)/6);
            }
            else if (uss_data <= 53)
            {
                vel_scale = (0.005*(uss_data-25));
            }
            if (vel_scale < 0)
            {
                vel_scale =0;
            }
        }
        return vel_scale;
    }

    void safetyCmdVelCallback(const geometry_msgs::msg::Twist &msg)
    {
        /* Callback function for /antobot_safety/cmd_vel. Assigns the velocity data received from
        this ROS topic to the robot if appropriate, and ensures reasonable acceleration.
        */
 
        // Max acceleration
        float max_acc = 3.0;  // m/s^2
        float max_acc_s = max_acc / 25.0;   // Conversion to expected speed increase at 25 hz (loop rate)
        float max_dec = 12.0;   // m/s^2
        float max_dec_s = max_dec / 25.0;   // Conversion to expected speed decrease at 25 hz (loop rate)
        
        // Assigns received value to velocity variables
        linear_vel = msg.linear.x;
        angular_vel = msg.angular.z;

        // Checks acceleration
        if (abs(linear_vel) > abs(prev_linear_vel) + max_acc_s)
            linear_vel = prev_linear_vel + copysign(max_acc_s, linear_vel); // If acceleration is too high, lowers command speed

        // Checks deceleration
        if (abs(linear_vel) < abs(prev_linear_vel) - max_dec_s)
            linear_vel = prev_linear_vel - copysign(max_dec_s, prev_linear_vel); // If deceleration is too fast, keeps speed up

        // Assigns values to final output
        cmd_vel_msg.linear.x = linear_vel;
        cmd_vel_msg.angular.z = angular_vel;

        safe_operation = true;

        // Provides time that the command was received
        t_lastRcvdCmdVel = clock();
        time_lastRcvdCmdVel = std::chrono::steady_clock::now();
        time_lastStopTriggerWarning = std::chrono::steady_clock::now();
        
    }

    void ussDistCallback(const antobot_platform_msgs::msg::UInt16Array &msg)
    {
        /*  Reads in the data from the ultrasonic sensors and, based on the current movement of the robot, makes a recommendation 
            for whether the robot should slow down or whether its current speed/movement is acceptable. */
        //  Inputs: msg <std_msgs::msg::Int16MultiArray> - currently an 8-element array which provides the distances sensed by each ultrasonic sensor. 
        //                                                The order starting from msg->data[0] is: 0 - front left; 1 - front; 2 - front right; 3 - right;
        //                                                4 - back right; 5 - back; 6 - back left; 7 - left
        //  Outputs: publishes filtered USS data to /antobot_safety/uss_dist ROS topic

        
        antobot_platform_msgs::msg::UInt16Array uss_dist_filt_all;
        uint16_t uss_dist_ar[8] = {200};
        uint16_t uss_dist_filt_i;
        if (uss_back_enable && uss_front_enable) {
            uint16_t tmp[8] = {200, msg.data[1], 200, 200, 200, msg.data[4], 200, 200};
            memcpy(uss_dist_ar, tmp, sizeof(tmp));
        }else if (uss_front_enable) {
            uint16_t tmp[8] = {200, msg.data[1], 200, 200, 200, 200, 200, 200};
            memcpy(uss_dist_ar, tmp, sizeof(tmp));
        }else if (uss_back_enable) {
            uint16_t tmp[8] = {200, 200, 200, 200, 200, msg.data[5], 200, 200};
            memcpy(uss_dist_ar, tmp, sizeof(tmp));
        }else {
            for (int i=0; i<8; i++)
                uss_dist_ar[i] = msg.data[i];
        }


        // Define the filtered USS dist class variable 
        //uss_dist_filt = ussDistFilt(uss_dist_ar);
        for (int i=0; i<8; i++)
        {
            uss_dist_filt_i = uss_dist_ar[i];
            uss_dist_filt_all.data.push_back(uss_dist_filt_i);
        }
        uss_dist_filt=uss_dist_filt_all;
        uss_dist_filt_pub_->publish(uss_dist_filt);
        
    }

    void releaseCallback(const std_msgs::msg::Bool &msg)
    {
        /*   Callback function for /antobridge/force_stop_release. Tells the robot it is okay to move again
                after a force stop command
        */
    
        if (msg.data) 
        {
            force_stop = false;
            force_stop_release = true;
            force_stop_bump = false;
            force_stop_type = 0;
            t_release = clock();

            std_msgs::msg::Bool lights_f_cmd;
            std_msgs::msg::Bool lights_b_cmd;

            lights_f_cmd.data = false;
            lights_b_cmd.data = false;

            lights_f_pub_->publish(lights_f_cmd);
            lights_b_pub_->publish(lights_b_cmd);

        }
        
    }

    void bumpFrontCallback(const std_msgs::msg::Bool &msg)
    {
        
        if (msg.data)
        {
            int cmd_vel_type;
            cmd_vel_type = getCmdVelType();

            if (cmd_vel_type == 1 || cmd_vel_type == 2 || cmd_vel_type == 3)
            {
                if (!force_stop)
                {
                    force_stop = true;
                    force_stop_bump = true;
                    force_stop_release = false;
                    force_stop_type = 9;
                    t_force_stop = clock();
                    time_force_stop = std::chrono::steady_clock::now();
                    time_safety_light = std::chrono::steady_clock::now();
                    t_safety_light = clock();
                    RCLCPP_INFO(this->get_logger(), "SF0110: Force stop by Front Bump Switch!");
                }
            }
        }
        
    }

    void bumpBackCallback(const std_msgs::msg::Bool &msg)
    {
        
        if (msg.data)
        {
            int cmd_vel_type;
            cmd_vel_type = getCmdVelType();

            if (cmd_vel_type == 5 || cmd_vel_type == 6 || cmd_vel_type == 7)
            {
                if (!force_stop)
                {
                    force_stop = true;
                    force_stop_bump = true;
                    force_stop_release = false;
                    force_stop_type = 10;
                    t_force_stop = clock();
                    time_force_stop = std::chrono::steady_clock::now();
                    time_safety_light = std::chrono::steady_clock::now();
                    t_safety_light = clock();
                    RCLCPP_INFO(this->get_logger(), "SF0111: Force stop by Back Bump Switch!");
                }
            }
        }   
    }


    int getCmdVelType()
    {
        int cmd_vel_type = 0;
        
        if (linear_vel > lin_vel_thresh)
        {
            // Moving forwards
            cmd_vel_type = 2;

            if (angular_vel > ang_vel_thresh * linear_vel)           // Robot is turning left while moving forward
            {
                cmd_vel_type = 1;
            }

            else if (angular_vel < -ang_vel_thresh * linear_vel)     // Robot is turning right while moving forward
            {
                cmd_vel_type = 3;
            }
        }
        else if (linear_vel < -lin_vel_thresh)
        {
            // Moving backwards
            cmd_vel_type = 6;

            if (angular_vel > - ang_vel_thresh * linear_vel)        // Robot is moving back and to the right
            {
                cmd_vel_type = 5;
            }

            else if (angular_vel <  ang_vel_thresh * linear_vel)    // Robot is moving back and to the left
            {
                cmd_vel_type = 7;
            }
        }
        else if (angular_vel > 0.2)
        {
            // Spot turn left
            cmd_vel_type = 8;
        }
        else if (angular_vel < -0.2)
        {
            // Spot turn right
            cmd_vel_type = 4;
        }
        return cmd_vel_type;   
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AntobotSafety>());
    rclcpp::shutdown();
    return 0;
}
