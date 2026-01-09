/*
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

Description:    Motor control node for Jetson-based motor control system
                Header file containing class declarations and definitions

Contacts: jianan.xu@nicecart.ai
*/

#ifndef MOTOR_CONTROL_NODE_H
#define MOTOR_CONTROL_NODE_H

#include <optional>
#include "motor_control_misc/msg/error_reports.hpp" 
#include "motor_control_misc/msg/motor_init_status.hpp" 
#include "motor_control_misc/msg/motor_status.hpp"
#include "motor_control_misc/srv/error_report.hpp" 
#include "motor_control_misc/action/check_error.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "can_bridge_msgs/msg/can_bridge.hpp"
#include "can_bridge_msgs/msg/pid_command.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <mutex>
#include <thread>

enum ControlMode;

// Motor control parameters structure
struct MotorParams {
    int can_id;
    double kp;
    double ki;
    double kd;
    double max_speed;  // in dps
    double acceleration;  // dps^2
    double gear_ratio;  // gear ratio
    double home_position;  // home position in radians
    bool enabled;        // whether this motor is enabled
    bool reset_on_startup; // whether to reset to home position on startup
};

// PID parameter indices
enum PidIndex {
    CURRENT_LOOP_KP = 0x01,
    CURRENT_LOOP_KI = 0x02,
    VELOCITY_LOOP_KP = 0x04,
    VELOCITY_LOOP_KI = 0x05,
    POSITION_LOOP_KP = 0x07,
    POSITION_LOOP_KI = 0x08,
    POSITION_LOOP_KD = 0x09
};

// Function control indices
enum FunctionControlIndex {
    CLEAR_MULTI_TURN = 0x01,
    CANID_FILTER_ENABLE = 0x02,
    ERROR_STATUS_ENABLE = 0x03,
    SAVE_MULTI_TURN_ON_POWER_OFF = 0x04,
    SET_CANID = 0x05,
    SET_MAX_POSITIVE_ANGLE = 0x06,
    SET_MAX_NEGATIVE_ANGLE = 0x07
};

enum ErrorType {
    NO_ERROR = 0,
    MOTOR_NOT_RESPONDING,
    MOTOR_MOVING_WHEN_SHOULDNT,
    MOTOR_TOO_SLOW,
    MOTOR_TOO_FAST,
    POSITION_ERROR_TOO_LARGE,
    COMMUNICATION_ERROR,
    OVERHEATING,
    OVERLOAD,
    MOTOR_POSITION_TIMEOUT,  // New error type for position timeout
    SELF_TEST_FAILED         // New error type for self-test failure
};

class MotorControlNode : public rclcpp::Node
{
public:
    // Define MotorError struct in the public section so it can be used in method declarations
    struct MotorError {
        int motor_index;
        ErrorType error_type;
        std::string description;
        float severity;
    };
    
    struct MotorState {
        bool is_moving;                 
        rclcpp::Time move_start_time;   
        double start_position;          
        double target_position;        
        double expected_move_time;      
        double last_position;          
        rclcpp::Time last_position_time;
        bool timeout_checked;          
    };

    MotorControlNode();
    ~MotorControlNode();

private:
    void loadParams();
    void canCallback(const can_bridge_msgs::msg::CanBridge::SharedPtr msg);
    void controlCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void sendPositionCommand(double position_rad, double max_speed_dps, int motor_id);
    void sendReadCommand(uint8_t command, int motor_id);
    void sendReadPidCommand(int motor_id, uint8_t pid_index);
    void sendWritePidCommand(int motor_id, uint8_t pid_index, float value);
    void sendFunctionControlCommand(int motor_id, uint8_t function_index, uint32_t value);
    double parsePositionResponse(const can_bridge_msgs::msg::CanBridge::SharedPtr msg);
    float parsePidResponse(const can_bridge_msgs::msg::CanBridge::SharedPtr msg);
    int findMotorIndexByCanId(int can_id);
    void readTimerCallback();
    
    // Service callbacks
    void handleReadPid(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void handleWritePid(const can_bridge_msgs::msg::PidCommand::SharedPtr msg);
    void handleFunctionControl(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void handleChangeCanId(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void controlModeCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    
    // Timer
    rclcpp::TimerBase::SharedPtr read_timer_;
    
    // Publisher and Subscriber
    rclcpp::Publisher<can_bridge_msgs::msg::CanBridge>::SharedPtr can_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pid_value_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr self_test_pub_; // New publisher for self-test status
    rclcpp::Subscription<can_bridge_msgs::msg::CanBridge>::SharedPtr can_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr control_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr read_pid_sub_;
    rclcpp::Subscription<can_bridge_msgs::msg::PidCommand>::SharedPtr write_pid_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr function_control_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr change_canid_sub_;
    
    
    // Parameters
    std::vector<MotorParams> motor_params_;
    std::vector<double> desired_positions_;  // in radians
    std::vector<double> current_positions_;  // in radians
    std::vector<double> current_velocities_;  // in rad/s
    std::vector<std::string> joint_names_;
    double read_frequency_;
    int motor_count_;
    std::vector<bool> position_changed_; // Flags to indicate if position has changed for each motor
    
    // Variables to prevent duplicate reading
    std::map<int, rclcpp::Time> last_read_time_;
    double min_read_interval_; // Minimum read interval (seconds)

    // Error handling
    // Error report service
    rclcpp::Service<motor_control_misc::srv::ErrorReport>::SharedPtr error_report_service_;
    
    // Error detection methods
    std::vector<MotorError> checkMotorErrors();
    std::optional<MotorError> checkMotorResponding(int motor_index);
    std::optional<MotorError> checkMotorMovingWhenShouldnt(int motor_index);
    std::optional<MotorError> checkMotorSpeed(int motor_index);
    std::optional<MotorError> checkPositionError(int motor_index);
    std::optional<MotorError> checkCommunication(int motor_index);
    std::optional<MotorError> checkMotorPositionTimeout(int motor_index); // New timeout check
    std::map<ErrorType, std::string> error_type_names_; // Error type name mapping
    
    // Error report service callback
    void handleErrorReport(
        const std::shared_ptr<motor_control_misc::srv::ErrorReport::Request> request,
        std::shared_ptr<motor_control_misc::srv::ErrorReport::Response> response);
    
    // Error detection parameters
    double max_position_error_;  // Maximum allowed position error (radians)
    double min_speed_threshold_; // Minimum speed threshold (rad/s)
    double max_speed_threshold_; // Maximum speed threshold (rad/s)
    double communication_timeout_; // Communication timeout (seconds)
    
    // Position timeout tracking
    std::vector<rclcpp::Time> last_command_time_;  // Time when last command was sent
    std::vector<double> expected_move_time_;       // Expected time to reach position (seconds)
    double position_timeout_factor_;               // Timeout factor (multiplier for expected time)
    
    std::mutex motor_data_mutex_;
    
    // Action server
    using CheckErrorAction = motor_control_misc::action::CheckError;
    using GoalHandleCheckError = rclcpp_action::ServerGoalHandle<CheckErrorAction>;
    rclcpp_action::Server<CheckErrorAction>::SharedPtr action_server_;
    
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const CheckErrorAction::Goal> goal);
        
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleCheckError> goal_handle);
        
    void handle_accepted(const std::shared_ptr<GoalHandleCheckError> goal_handle);
    
    void execute_check_error(const std::shared_ptr<GoalHandleCheckError> goal_handle);
    
    // Self-test functionality
    void performSelfTest();
    bool self_test_completed_;
    bool self_test_success_;
    std::vector<bool> motor_responded_;
    rclcpp::TimerBase::SharedPtr self_test_timer_;
    void selfTestTimerCallback();
    int self_test_attempts_;
    const int MAX_SELF_TEST_ATTEMPTS = 3;
    std::vector<bool> motor_initialized_;  // Track motor initialization status
    std::vector<rclcpp::Time> motor_init_time_;  // Time when motor was initialized
    std::map<int, rclcpp::Time> last_response_time_; // Track when responses are received
    
    // Motor states
    std::vector<MotorState> motor_states_;  
    
    // Status publisher
    rclcpp::Publisher<motor_control_misc::msg::MotorStatus>::SharedPtr motor_status_pub_;
    
    // Reset service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    
    // Status and control functions
    void sendStatusReadCommand(int motor_id);
    void parseStatusResponse(const can_bridge_msgs::msg::CanBridge::SharedPtr msg);
    void handleResetService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void sendResetCommand(int motor_id);
    void checkAndLogErrors(int motor_index, uint16_t error_state);
    
    
    // Status variables
    std::vector<float> motor_temperatures_;
    std::vector<float> motor_voltages_;
    std::vector<uint16_t> motor_error_states_;
    std::vector<bool> motor_brake_states_;
};

#endif // MOTOR_CONTROL_NODE_H