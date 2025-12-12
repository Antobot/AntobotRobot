/*
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

Description:    Motor control node for Jetson-based motor control system
                This node reads motor position and controls motor using CAN bus

Contacts: jianan.xu@nicecart.ai
*/

#include "motor_control_node.hpp"
#include "anto_control_base.hpp"

enum ControlMode {
    OPEN_LOOP = 0,
    CLOSED_LOOP = 1
};

MotorControlNode::MotorControlNode() : Node("motor_control_node"), min_read_interval_(0.02)
{
    RCLCPP_INFO(this->get_logger(), "MotorControlNode constructor started");
    
    // 初始化参数
    read_frequency_ = 50.0;
    motor_count_ = 4;
    
    RCLCPP_INFO(this->get_logger(), "Initializing vectors with motor_count: %d", motor_count_);
    
    // 初始化向量
    motor_params_.resize(motor_count_);
    desired_positions_.resize(motor_count_, 0.0);
    current_positions_.resize(motor_count_, 0.0);
    current_velocities_.resize(motor_count_, 0.0);
    joint_names_.resize(motor_count_);
    position_changed_.resize(motor_count_, false);
    
    RCLCPP_INFO(this->get_logger(), "Vectors initialized");
    
    // 初始化位置超时跟踪
    auto now = this->now();
    last_command_time_.resize(motor_count_, now);
    expected_move_time_.resize(motor_count_, 0.0);
    position_timeout_factor_ = 1.5;
    
    RCLCPP_INFO(this->get_logger(), "Timeout tracking initialized");
    
    // 初始化电机状态
    motor_states_.resize(motor_count_);
    for (int i = 0; i < motor_count_; i++) {
        motor_states_[i].is_moving = false;
        motor_states_[i].move_start_time = now;
        motor_states_[i].start_position = 0.0;
        motor_states_[i].target_position = 0.0;
        motor_states_[i].expected_move_time = 0.0;
        motor_states_[i].last_position = 0.0;
        motor_states_[i].last_position_time = now;
        motor_states_[i].timeout_checked = false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Motor states initialized");
    
    // 初始化关节名称和电机参数
    for (int i = 0; i < motor_count_; i++) {
        joint_names_[i] = "steer_joint_" + std::to_string(i+1);
        motor_params_[i].enabled = true;
    }
    
    RCLCPP_INFO(this->get_logger(), "Joint names and motor params initialized");
    
    // 加载参数
    loadParams();
    
    RCLCPP_INFO(this->get_logger(), "Parameters loaded");
    
    // 错误类型名称映射
    error_type_names_[MOTOR_NOT_RESPONDING] = "MOTOR_NOT_RESPONDING";
    error_type_names_[MOTOR_MOVING_WHEN_SHOULDNT] = "MOTOR_MOVING_WHEN_SHOULDNT";
    error_type_names_[MOTOR_TOO_SLOW] = "MOTOR_TOO_SLOW";
    error_type_names_[MOTOR_TOO_FAST] = "MOTOR_TOO_FAST";
    error_type_names_[POSITION_ERROR_TOO_LARGE] = "POSITION_ERROR_TOO_LARGE";
    error_type_names_[COMMUNICATION_ERROR] = "COMMUNICATION_ERROR";
    error_type_names_[OVERHEATING] = "OVERHEATING";
    error_type_names_[OVERLOAD] = "OVERLOAD";
    error_type_names_[MOTOR_POSITION_TIMEOUT] = "MOTOR_POSITION_TIMEOUT";
    
    RCLCPP_INFO(this->get_logger(), "Error type names initialized");
    
    // 初始化最后读取时间
    for (int i = 0; i < motor_count_; i++) {
        last_read_time_[motor_params_[i].can_id] = this->now();
    }
    
    RCLCPP_INFO(this->get_logger(), "Last read times initialized");
    
    // 创建发布器
    can_pub_ = this->create_publisher<can_bridge_msgs::msg::CanBridge>("/antobot/bridge/can/write", 10);
    joint_state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/antobot/control/real_pos", 10);
    pid_value_pub_ = this->create_publisher<std_msgs::msg::Float32>("/pid_value", 10);
    
    RCLCPP_INFO(this->get_logger(), "Publishers created");
    
    // 创建订阅器
    can_sub_ = this->create_subscription<can_bridge_msgs::msg::CanBridge>(
        "/antobot/bridge/can/read", 10,
        std::bind(&MotorControlNode::canCallback, this, std::placeholders::_1));
    
    auto qos2 = rclcpp::QoS(rclcpp::SensorDataQoS());
    qos2.reliable();
    
    control_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/antobot/control/cmd_pos", qos2,
        std::bind(&MotorControlNode::controlCommandCallback, this, std::placeholders::_1));
    
    read_pid_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/antobot/read_pid", 10,
        std::bind(&MotorControlNode::handleReadPid, this, std::placeholders::_1));
    
    write_pid_sub_ = this->create_subscription<can_bridge_msgs::msg::PidCommand>(
        "/antobot/write_pid", 10,
        std::bind(&MotorControlNode::handleWritePid, this, std::placeholders::_1));
    
    function_control_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/antobot/function_control", 10,
        std::bind(&MotorControlNode::handleFunctionControl, this, std::placeholders::_1));
    
    change_canid_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/antobot/change_canid", 10,
        std::bind(&MotorControlNode::handleChangeCanId, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Subscribers created");
    
    // 创建定时器
    read_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / read_frequency_),
        std::bind(&MotorControlNode::readTimerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Timer created");
    
    // 初始化电机位置
    for (int i = 0; i < motor_count_; i++) {
        if (motor_params_[i].enabled && motor_params_[i].reset_on_startup) {
            sendPositionCommand(motor_params_[i].home_position, motor_params_[i].max_speed, motor_params_[i].can_id);
            desired_positions_[i] = motor_params_[i].home_position;
            RCLCPP_INFO(this->get_logger(), "Sending initial home position to motor %d: %.3f rad", i+1, motor_params_[i].home_position);
        } else if (motor_params_[i].enabled) {
            desired_positions_[i] = motor_params_[i].home_position;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Motor positions initialized");
    
    // 错误检测参数
    max_position_error_ = 0.1;
    min_speed_threshold_ = 0.01;
    max_speed_threshold_ = 10.0;
    communication_timeout_ = 0.5;
    
    // 创建错误报告服务
    error_report_service_ = this->create_service<motor_control_misc::srv::ErrorReport>(
        "/motor_control/error_report",
        std::bind(&MotorControlNode::handleErrorReport, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Error report service created");
    
    // 创建 Action Server
    action_server_ = rclcpp_action::create_server<CheckErrorAction>(
        this,
        "/antobot/control/change_mode/check_error",
        std::bind(&MotorControlNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MotorControlNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&MotorControlNode::handle_accepted, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Action server created");
    for (int i = 0; i < motor_count_; i++) {
        last_response_time_[motor_params_[i].can_id] = this->now();
    }
    
    motor_initialized_.resize(motor_count_, false);
	motor_init_time_.resize(motor_count_, this->now());

    RCLCPP_INFO(this->get_logger(), "Motor Control node started for %d motors", motor_count_);
    RCLCPP_INFO(this->get_logger(), "Check Error Action Server initialized");
    // 初始化状态向量
    motor_temperatures_.resize(motor_count_, 0.0f);
    motor_voltages_.resize(motor_count_, 0.0f);
    motor_error_states_.resize(motor_count_, 0);
    motor_brake_states_.resize(motor_count_, false);
    
    
    // 创建状态发布器
    motor_status_pub_ = this->create_publisher<motor_control_misc::msg::MotorStatus>(
        "/antobot/control/steer_state", 10);
    
    
    // 创建复位服务
    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/motor_control/reset",
        std::bind(&MotorControlNode::handleResetService, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
}

MotorControlNode::~MotorControlNode()
{
    // Destructor
}

void MotorControlNode::loadParams()
{
    // Declare and get parameters for each motor
    for (int i = 0; i < motor_count_; i++) {
        std::string prefix = "motor_" + std::to_string(i+1) + ".";
        
        // Add CAN ID parameter declaration and retrieval
        this->declare_parameter(prefix + "can_id", 0x141 + i); // Default value 0x141+i
        this->get_parameter(prefix + "can_id", motor_params_[i].can_id);
        
        this->declare_parameter(prefix + "kp", 1.0);
        this->declare_parameter(prefix + "ki", 0.0);
        this->declare_parameter(prefix + "kd", 0.0);
        this->declare_parameter(prefix + "max_speed", 500.0);  // dps
        this->declare_parameter(prefix + "acceleration", 1000.0);  // dps^2
        this->declare_parameter(prefix + "gear_ratio", 6.0);
        this->declare_parameter(prefix + "home_position", 0.0);  // radians
        this->declare_parameter(prefix + "enabled", true);
        this->declare_parameter(prefix + "reset_on_startup", false); 

        this->get_parameter(prefix + "kp", motor_params_[i].kp);
        this->get_parameter(prefix + "ki", motor_params_[i].ki);
        this->get_parameter(prefix + "kd", motor_params_[i].kd);
        this->get_parameter(prefix + "max_speed", motor_params_[i].max_speed);
        this->get_parameter(prefix + "acceleration", motor_params_[i].acceleration);
        this->get_parameter(prefix + "gear_ratio", motor_params_[i].gear_ratio);
        this->get_parameter(prefix + "home_position", motor_params_[i].home_position);
        this->get_parameter(prefix + "enabled", motor_params_[i].enabled);
        this->get_parameter(prefix + "reset_on_startup", motor_params_[i].reset_on_startup); 
        
        // Set desired position to home position
        desired_positions_[i] = motor_params_[i].home_position;
        
        RCLCPP_INFO(this->get_logger(), "Motor %d parameters:", i+1);
        RCLCPP_INFO(this->get_logger(), "  CAN ID: 0x%X", motor_params_[i].can_id);
        RCLCPP_INFO(this->get_logger(), "  Max Speed: %.1f dps", motor_params_[i].max_speed);
        RCLCPP_INFO(this->get_logger(), "  Gear Ratio: %.1f", motor_params_[i].gear_ratio);
        RCLCPP_INFO(this->get_logger(), "  Enabled: %s", motor_params_[i].enabled ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  Reset on Startup: %s", motor_params_[i].reset_on_startup ? "true" : "false");
    }
    
    // Declare and get general parameters
    this->declare_parameter("read_frequency", 50.0);
    this->get_parameter("read_frequency", read_frequency_);
    
    // Declare and get error detection parameters
    this->declare_parameter("max_position_error", 0.1);
    this->declare_parameter("min_speed_threshold", 0.01);
    this->declare_parameter("max_speed_threshold", 10.0);
    this->declare_parameter("communication_timeout", 0.5);
    this->declare_parameter("position_timeout_factor", 1.5); // New parameter for timeout factor
    
    this->get_parameter("max_position_error", max_position_error_);
    this->get_parameter("min_speed_threshold", min_speed_threshold_);
    this->get_parameter("max_speed_threshold", max_speed_threshold_);
    this->get_parameter("communication_timeout", communication_timeout_);
    this->get_parameter("position_timeout_factor", position_timeout_factor_);
    
    RCLCPP_INFO(this->get_logger(), "Error detection parameters:");
    RCLCPP_INFO(this->get_logger(), "  Max position error: %.3f rad", max_position_error_);
    RCLCPP_INFO(this->get_logger(), "  Min speed threshold: %.3f rad/s", min_speed_threshold_);
    RCLCPP_INFO(this->get_logger(), "  Max speed threshold: %.3f rad/s", max_speed_threshold_);
    RCLCPP_INFO(this->get_logger(), "  Communication timeout: %.3f s", communication_timeout_);
    RCLCPP_INFO(this->get_logger(), "  Position timeout factor: %.2f", position_timeout_factor_);
}

void MotorControlNode::performSelfTest()
{
    RCLCPP_INFO(this->get_logger(), "=== MOTOR SELF-TEST STARTED ===");
 
    // Reset self-test state
    self_test_completed_ = false;
    self_test_success_ = true;
 
    // Reset response tracking
    for (int i = 0; i < motor_count_; i++) {
        motor_responded_[i] = false;
    }
 
    // Send read command to all enabled motors
    for (int i = 0; i < motor_count_; i++) {
        if (motor_params_[i].enabled) {
            RCLCPP_INFO(this->get_logger(), "Testing motor %d (CAN ID: 0x%X)", i+1, motor_params_[i].can_id);
            sendReadCommand(0x92, motor_params_[i].can_id);
        }
    }
 
    // Start timer to check self-test results
    self_test_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MotorControlNode::selfTestTimerCallback, this));
 
    self_test_attempts_ = 0;
}

void MotorControlNode::selfTestTimerCallback()
{
    self_test_attempts_++;
 
    // Check which motors have responded
    int enabled_motors = 0;
    int responding_motors = 0;
 
    for (int i = 0; i < motor_count_; i++) {
        if (motor_params_[i].enabled) {
            enabled_motors++;
            if (motor_responded_[i]) {
                responding_motors++;
                RCLCPP_INFO(this->get_logger(), "Motor %d (CAN ID: 0x%X) - RESPONDING", 
                           i+1, motor_params_[i].can_id);
            } else {
                RCLCPP_WARN(this->get_logger(), "Motor %d (CAN ID: 0x%X) - NOT RESPONDING", 
                           i+1, motor_params_[i].can_id);
            }
        }
    }
 
    if (responding_motors == enabled_motors) {
        // All enabled motors are responding
        self_test_completed_ = true;
        self_test_success_ = true;
        self_test_timer_->cancel();
 
        RCLCPP_INFO(this->get_logger(), "=== MOTOR SELF-TEST COMPLETED SUCCESSFULLY ===");
        RCLCPP_INFO(this->get_logger(), "All %d enabled motors are responding correctly", enabled_motors);
 
    } else if (self_test_attempts_ >= MAX_SELF_TEST_ATTEMPTS) {
        // Max attempts reached, test failed
        self_test_completed_ = true;
        self_test_success_ = false;
        self_test_timer_->cancel();
 
        RCLCPP_ERROR(this->get_logger(), "=== MOTOR SELF-TEST FAILED ===");
        RCLCPP_ERROR(this->get_logger(), "Only %d of %d enabled motors are responding", 
                    responding_motors, enabled_motors);
 
        // Log detailed error information
        for (int i = 0; i < motor_count_; i++) {
            if (motor_params_[i].enabled && !motor_responded_[i]) {
                RCLCPP_ERROR(this->get_logger(), "Motor %d (CAN ID: 0x%X) failed to respond", 
                           i+1, motor_params_[i].can_id);
            }
        }
    } else {
        // Send read commands again for non-responding motors
        for (int i = 0; i < motor_count_; i++) {
            if (motor_params_[i].enabled && !motor_responded_[i]) {
                RCLCPP_INFO(this->get_logger(), "Retrying motor %d (attempt %d/%d)", 
                           i+1, self_test_attempts_ + 1, MAX_SELF_TEST_ATTEMPTS);
                sendReadCommand(0x92, motor_params_[i].can_id);
            }
        }
    }
}

void MotorControlNode::readTimerCallback()
{
    auto now = this->now();
    static bool read_position = true;  // 用于交替读取位置和状态
    
    for (int i = 0; i < motor_count_; i++) {
        if (motor_params_[i].enabled) {
            // Check if minimum read interval has passed to avoid duplicate reading
            auto last_time = last_read_time_[motor_params_[i].can_id];
            if ((now - last_time).seconds() >= min_read_interval_) {
                if (read_position) {
                    sendReadCommand(0x92, motor_params_[i].can_id);
                } else {
                    sendStatusReadCommand(motor_params_[i].can_id);
                }
                last_read_time_[motor_params_[i].can_id] = now;
            }
        }
    }
    
    // 切换下一次读取的类型
    read_position = !read_position;
}


void MotorControlNode::sendStatusReadCommand(int motor_id)
{
    auto can_msg = can_bridge_msgs::msg::CanBridge();
    can_msg.can_id = motor_id;
    can_msg.length = 8;
    
    std::stringstream ss;
    ss << "0x";
    ss << std::hex << std::setw(2) << std::setfill('0') << 0x9A;  // Command byte
    
    // Fill rest with zeros
    for (int i = 0; i < 7; i++) {
        ss << std::hex << std::setw(2) << std::setfill('0') << 0x00;
    }
    
    can_msg.msg = ss.str();
    
    RCLCPP_DEBUG(this->get_logger(), "Sending status read command to motor ID 0x%X: Data=%s", 
                 can_msg.can_id, can_msg.msg.c_str());
    
    // Publish CAN message
    can_pub_->publish(can_msg);
}



void MotorControlNode::canCallback(const can_bridge_msgs::msg::CanBridge::SharedPtr msg)
{
    // Debug output for received CAN message
    RCLCPP_DEBUG(this->get_logger(), "Raw CAN message: ID=0x%X, Length=%d, Data=%s",
                msg->can_id, msg->length, msg->msg.c_str());

    // Add debug output to check if messages are being received at all
    static int message_count = 0;
    message_count++;
    if (message_count % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "Processed %d CAN messages", message_count);
    }
    
    // Find which motor this message is from
    int motor_index = findMotorIndexByCanId(msg->can_id - 0x100);
    
    if (motor_index >= 0 && motor_index < motor_count_) {
        // 更新最后响应时间
        last_response_time_[motor_params_[motor_index].can_id] = this->now();
        
        // 如果电机尚未初始化，标记为已初始化
        if (!motor_initialized_[motor_index]) {
            motor_initialized_[motor_index] = true;
            motor_init_time_[motor_index] = this->now();
            RCLCPP_INFO(this->get_logger(), "Motor %d (CAN ID: 0x%X) initialized successfully", 
                       motor_index + 1, motor_params_[motor_index].can_id);
        }
        
        // Parse response based on command byte
        uint8_t command_byte = std::stoul(msg->msg.substr(2, 2), nullptr, 16);  // Extract first byte after "0x"
        
        if (command_byte == 0x92) {
            // Multi-turn angle response
            double position_rad = parsePositionResponse(msg);
            double old_position = current_positions_[motor_index];
            current_positions_[motor_index] = position_rad;
            
            // 如果电机正在移动，更新最后位置和时间
            if (motor_states_[motor_index].is_moving) {
                motor_states_[motor_index].last_position = old_position;
                motor_states_[motor_index].last_position_time = this->now();
            }
            
            RCLCPP_INFO(this->get_logger(), "Motor %d current position: %.3f rad", motor_index+1, current_positions_[motor_index]);
            
            // Publish joint states immediately
            auto positions_msg = std_msgs::msg::Float64MultiArray();
            positions_msg.data = current_positions_;
            joint_state_pub_->publish(positions_msg);
            
        } else if (command_byte == 0xA4) {
            // Position control response - we can also extract position from this
            double position_rad = parsePositionResponse(msg);
            double old_position = current_positions_[motor_index];
            current_positions_[motor_index] = position_rad;
            
            // 如果电机正在移动，更新最后位置和时间
            if (motor_states_[motor_index].is_moving) {
                motor_states_[motor_index].last_position = old_position;
                motor_states_[motor_index].last_position_time = this->now();
            }
            
            RCLCPP_INFO(this->get_logger(), "Motor %d control response position: %.3f rad", motor_index+1, current_positions_[motor_index]);
            
            // Publish joint states immediately
            auto positions_msg = std_msgs::msg::Float64MultiArray();
            positions_msg.data = current_positions_;
            joint_state_pub_->publish(positions_msg);
            
        } else if (command_byte == 0x30) {
            // PID read response
            float pid_value = parsePidResponse(msg);
            auto pid_msg = std_msgs::msg::Float32();
            pid_msg.data = pid_value;
            pid_value_pub_->publish(pid_msg);
            RCLCPP_INFO(this->get_logger(), "Motor %d PID value: %.6f", motor_index+1, pid_value);
        } else if (command_byte == 0x31) {
            // PID write response
            RCLCPP_INFO(this->get_logger(), "Motor %d PID parameter written successfully", motor_index+1);
        } else if (command_byte == 0x20) {
            // Function control response
            RCLCPP_INFO(this->get_logger(), "Motor %d function control executed successfully", motor_index+1);
        } else if (command_byte == 0x9A) {
            // Status read response
            parseStatusResponse(msg);
        }
    }
}

void MotorControlNode::parseStatusResponse(const can_bridge_msgs::msg::CanBridge::SharedPtr msg)
{
    if (msg->msg.length() < 18) {  // Minimum length for status response
        RCLCPP_WARN(this->get_logger(), "Invalid status response message length: %zu", msg->msg.length());
        return;
    }
    
    // Find which motor this message is from
    int motor_index = findMotorIndexByCanId(msg->can_id - 0x100);
    
    if (motor_index < 0 || motor_index >= motor_count_) {
        return;
    }
    
    // Extract data bytes
    uint8_t temperature = std::stoul(msg->msg.substr(4, 2), nullptr, 16);      // DATA[1]
    uint8_t mos_temperature = std::stoul(msg->msg.substr(6, 2), nullptr, 16);   // DATA[2]
    uint8_t brake_state = std::stoul(msg->msg.substr(8, 2), nullptr, 16);       // DATA[3]
    
    // Extract voltage (little-endian: DATA[4] is LSB, DATA[5] is MSB)
    uint8_t voltage_low = std::stoul(msg->msg.substr(10, 2), nullptr, 16);      // DATA[4]
    uint8_t voltage_high = std::stoul(msg->msg.substr(12, 2), nullptr, 16);     // DATA[5]
    uint16_t voltage_raw = (static_cast<uint16_t>(voltage_high) << 8) | static_cast<uint16_t>(voltage_low);
    float voltage = static_cast<float>(voltage_raw) * 0.1f;  // Convert to volts
    
    // Extract error state (little-endian: DATA[6] is LSB, DATA[7] is MSB)
    uint8_t error_low = std::stoul(msg->msg.substr(14, 2), nullptr, 16);        // DATA[6]
    uint8_t error_high = std::stoul(msg->msg.substr(16, 2), nullptr, 16);       // DATA[7]
    uint16_t error_state = (static_cast<uint16_t>(error_high) << 8) | static_cast<uint16_t>(error_low);
    
    // Update motor status
    motor_temperatures_[motor_index] = static_cast<float>(temperature);
    motor_voltages_[motor_index] = voltage;
    motor_error_states_[motor_index] = error_state;
    motor_brake_states_[motor_index] = (brake_state != 0);
    
    // Publish motor status
    auto status_msg = motor_control_misc::msg::MotorStatus();
    status_msg.motor_id = motor_index + 1;
    status_msg.temperature = motor_temperatures_[motor_index];
    status_msg.voltage = motor_voltages_[motor_index];
    status_msg.error_state = motor_error_states_[motor_index];
    status_msg.brake_released = motor_brake_states_[motor_index];
    
    motor_status_pub_->publish(status_msg);
    
    // Check and log errors
    checkAndLogErrors(motor_index, error_state);
    
    RCLCPP_DEBUG(this->get_logger(), "Motor %d status: temp=%d°C, MOS temp=%d°C, voltage=%.1fV, brake=%s, error=0x%04X",
                motor_index + 1, temperature, mos_temperature, voltage, 
                brake_state ? "released" : "locked", error_state);
}

void MotorControlNode::checkAndLogErrors(int motor_index, uint16_t error_state)
{
    if (error_state == 0) {
        return;  // No errors
    }
    
    std::map<uint16_t, std::string> error_messages = {
        {0x0002, "Motor stalled"},
        {0x0004, "Low voltage"},
        {0x0008, "Over voltage"},
        {0x0010, "Phase current overrun"},
        {0x0040, "Power overrun"},
        {0x0080, "Calibration parameter write error"},
        {0x0100, "Overspeed"},
        {0x0800, "Component overtemperature"},
        {0x1000, "Motor overtemperature"},
        {0x2000, "Encoder calibration error"},
        {0x4000, "Encoder data error"}
    };
    
    std::vector<std::string> active_errors;
    
    for (const auto& error : error_messages) {
        if (error_state & error.first) {
            active_errors.push_back(error.second);
        }
    }
    
    if (!active_errors.empty()) {
        std::string error_msg = "Motor " + std::to_string(motor_index + 1) + " errors: ";
        for (size_t i = 0; i < active_errors.size(); i++) {
            error_msg += active_errors[i];
            if (i < active_errors.size() - 1) {
                error_msg += ", ";
            }
        }
        
        RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
    }
}

void MotorControlNode::controlCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    static bool first_command = true;
    
    // Add debug log
    RCLCPP_INFO(this->get_logger(), "=== CONTROL COMMAND CALLBACK TRIGGERED ===");
    RCLCPP_DEBUG(this->get_logger(), "Received data size: %zu", msg->data.size());
    
    if (msg->data.size() != static_cast<size_t>(motor_count_)) {
        RCLCPP_WARN(this->get_logger(), "Size mismatch: received %zu, expected %d", 
                   msg->data.size(), motor_count_);
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Received positions: [%.3f, %.3f, %.3f, %.3f]", 
                msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
    
    for (int i = 0; i < motor_count_; i++) {
        RCLCPP_INFO(this->get_logger(), "Motor %d: enabled=%s, current=%.3f, new=%.3f, changed=%s", 
                   i+1, 
                   motor_params_[i].enabled ? "true" : "false",
                   desired_positions_[i], 
                   msg->data[i],
                   (msg->data[i] != desired_positions_[i]) ? "true" : "false");
        
        bool should_send = first_command || (msg->data[i] != desired_positions_[i]);
        
        if (motor_params_[i].enabled && should_send) {
            desired_positions_[i] = msg->data[i];
            
            // Calculate expected move time for timeout detection
            double distance_rad = std::abs(msg->data[i] - current_positions_[i]);
            double distance_deg = distance_rad * 180.0 / M_PI;
            // Expected time = distance / speed (with safety factor)
            expected_move_time_[i] = (distance_deg / motor_params_[i].max_speed) * position_timeout_factor_;
            last_command_time_[i] = this->now();
            
            RCLCPP_INFO(this->get_logger(), "SENDING POSITION COMMAND TO MOTOR %d", i+1);
            RCLCPP_INFO(this->get_logger(), "Distance: %.3f rad (%.3f deg), Expected time: %.3f s", 
                       distance_rad, distance_deg, expected_move_time_[i]);
            
            sendPositionCommand(desired_positions_[i], motor_params_[i].max_speed, motor_params_[i].can_id);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "NOT sending command to motor %d (enabled=%s, changed=%s, first=%s)", 
                       i+1,
                       motor_params_[i].enabled ? "true" : "false",
                       (msg->data[i] != desired_positions_[i]) ? "true" : "false",
                       first_command ? "true" : "false");
        }
    }
    if (first_command) {
        first_command = false;
        RCLCPP_INFO(this->get_logger(), "First command processed, future commands will only be sent if position changes");
    }
}

void MotorControlNode::handleReadPid(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Read PID command requires at least 2 values: motor_index and pid_index");
        return;
    }
    
    int motor_index = msg->data[0];
    uint8_t pid_index = static_cast<uint8_t>(msg->data[1]);
    
    if (motor_index < 0 || motor_index >= motor_count_) {
        RCLCPP_WARN(this->get_logger(), "Invalid motor index: %d", motor_index);
        return;
    }
    
    if (!motor_params_[motor_index].enabled) {
        RCLCPP_WARN(this->get_logger(), "Motor %d is not enabled", motor_index);
        return;
    }
    
    sendReadPidCommand(motor_params_[motor_index].can_id, pid_index);
    RCLCPP_INFO(this->get_logger(), "Reading PID parameter 0x%02X from motor %d", pid_index, motor_index+1);
}

void MotorControlNode::handleWritePid(const can_bridge_msgs::msg::PidCommand::SharedPtr msg)
{
    int motor_index = msg->motor_index;
    uint8_t pid_index = msg->pid_index;
    float value = msg->value;
    
    if (motor_index < 0 || motor_index >= motor_count_) {
        RCLCPP_WARN(this->get_logger(), "Invalid motor index: %d", motor_index);
        return;
    }
    
    if (!motor_params_[motor_index].enabled) {
        RCLCPP_WARN(this->get_logger(), "Motor %d is not enabled", motor_index);
        return;
    }
    
    sendWritePidCommand(motor_params_[motor_index].can_id, pid_index, value);
    RCLCPP_INFO(this->get_logger(), "Writing PID parameter 0x%02X to motor %d with value %.6f", 
                pid_index, motor_index+1, value);
}

void MotorControlNode::handleFunctionControl(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 3) {
        RCLCPP_WARN(this->get_logger(), "Function control command requires at least 3 values: motor_index, function_index, and value");
        return;
    }
    
    int motor_index = msg->data[0];
    uint8_t function_index = static_cast<uint8_t>(msg->data[1]);
    uint32_t value = static_cast<uint32_t>(msg->data[2]);
    
    if (motor_index < 0 || motor_index >= motor_count_) {
        RCLCPP_WARN(this->get_logger(), "Invalid motor index: %d", motor_index);
        return;
    }
    
    if (!motor_params_[motor_index].enabled) {
        RCLCPP_WARN(this->get_logger(), "Motor %d is not enabled", motor_index);
        return;
    }
    
    sendFunctionControlCommand(motor_params_[motor_index].can_id, function_index, value);
    RCLCPP_INFO(this->get_logger(), "Sending function control 0x%02X to motor %d with value 0x%08X", 
                function_index, motor_index+1, value);
}

void MotorControlNode::handleChangeCanId(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Change CAN ID command requires at least 2 values: motor_index and new_can_id");
        return;
    }
    
    int motor_index = msg->data[0];
    int new_can_id = msg->data[1];
    
    if (motor_index < 0 || motor_index >= motor_count_) {
        RCLCPP_WARN(this->get_logger(), "Invalid motor index: %d", motor_index);
        return;
    }
    
    if (!motor_params_[motor_index].enabled) {
        RCLCPP_WARN(this->get_logger(), "Motor %d is not enabled", motor_index);
        return;
    }
    
    if (new_can_id < 0x141 || new_can_id > 0x144) {
        RCLCPP_WARN(this->get_logger(), "Invalid CAN ID: 0x%X. Must be between 0x141 and 0x144", new_can_id);
        return;
    }
    
    // Send function control command to change CAN ID
    sendFunctionControlCommand(motor_params_[motor_index].can_id, SET_CANID, static_cast<uint32_t>(new_can_id));
    
    // Update the motor parameters with the new CAN ID
    motor_params_[motor_index].can_id = new_can_id;
    
    RCLCPP_INFO(this->get_logger(), "Changing motor %d CAN ID to 0x%X", motor_index+1, new_can_id);
}

void MotorControlNode::sendPositionCommand(double position_rad, double max_speed_dps, int motor_id)
{
    // Convert position from radians to 0.01 degrees
    int32_t angle_control = static_cast<int32_t>((position_rad * 18000.0) / M_PI);
    
    // Convert max speed to uint16_t
    uint16_t max_speed = static_cast<uint16_t>(max_speed_dps);
    
    // Create CAN message
    auto can_msg = can_bridge_msgs::msg::CanBridge();
    can_msg.can_id = motor_id;
    can_msg.length = 8;
    
    // Build message data
    std::stringstream ss;
    ss << "0x";
    ss << std::hex << std::setw(2) << std::setfill('0') << 0xA4;  // Command byte
    ss << std::hex << std::setw(2) << std::setfill('0') << 0x00;  // NULL
    
    // Max speed (little-endian)
    ss << std::hex << std::setw(2) << std::setfill('0') << (max_speed & 0xFF);
    ss << std::hex << std::setw(2) << std::setfill('0') << ((max_speed >> 8) & 0xFF);
    
    // Angle control (little-endian)
    ss << std::hex << std::setw(2) << std::setfill('0') << (angle_control & 0xFF);
    ss << std::hex << std::setw(2) << std::setfill('0') << ((angle_control >> 8) & 0xFF);
    ss << std::hex << std::setw(2) << std::setfill('0') << ((angle_control >> 16) & 0xFF);
    ss << std::hex << std::setw(2) << std::setfill('0') << ((angle_control >> 24) & 0xFF);
    
    can_msg.msg = ss.str();
    
    // Publish CAN message
    can_pub_->publish(can_msg);
    int motor_index = findMotorIndexByCanId(motor_id);
    if (motor_index >= 0 && motor_index < motor_count_) {
        // 更新电机状态
        motor_states_[motor_index].is_moving = true;
        motor_states_[motor_index].move_start_time = this->now();
        motor_states_[motor_index].start_position = current_positions_[motor_index];
        motor_states_[motor_index].target_position = position_rad;
        
        // 计算预期移动时间
        double distance_rad = std::abs(position_rad - current_positions_[motor_index]);
        double distance_deg = distance_rad * 180.0 / M_PI;
        motor_states_[motor_index].expected_move_time = (distance_deg / max_speed_dps) * position_timeout_factor_;
        
        motor_states_[motor_index].last_position = current_positions_[motor_index];
        motor_states_[motor_index].last_position_time = this->now();
        motor_states_[motor_index].timeout_checked = false;
        
        RCLCPP_INFO(this->get_logger(), "Motor %d: start=%.3f, target=%.3f, expected_time=%.3fs", 
                   motor_index+1, current_positions_[motor_index], position_rad, 
                   motor_states_[motor_index].expected_move_time);
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Sent position command to motor with ID 0x%X: %s", motor_id, can_msg.msg.c_str());
}

void MotorControlNode::sendReadCommand(uint8_t command, int motor_id)
{
    auto can_msg = can_bridge_msgs::msg::CanBridge();
    can_msg.can_id = motor_id;
    can_msg.length = 8;
    
    std::stringstream ss;
    ss << "0x";
    ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(command);
    
    // Fill rest with zeros
    for (int i = 0; i < 7; i++) {
        ss << std::hex << std::setw(2) << std::setfill('0') << 0x00;
    }
    
    can_msg.msg = ss.str();
    
    RCLCPP_DEBUG(this->get_logger(), "CAN message for motor ID 0x%X: Data=%s", 
                 can_msg.can_id, can_msg.msg.c_str());
    
    // Publish CAN message
    can_pub_->publish(can_msg);
}

void MotorControlNode::sendReadPidCommand(int motor_id, uint8_t pid_index)
{
    auto can_msg = can_bridge_msgs::msg::CanBridge();
    can_msg.can_id = motor_id;
    can_msg.length = 8;
    
    std::stringstream ss;
    ss << "0x";
    ss << std::hex << std::setw(2) << std::setfill('0') << 0x30;  // Command byte
    ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(pid_index);  // PID index
    
    // Fill rest with zeros
    for (int i = 0; i < 6; i++) {
        ss << std::hex << std::setw(2) << std::setfill('0') << 0x00;
    }
    
    can_msg.msg = ss.str();
    
    RCLCPP_DEBUG(this->get_logger(), "CAN message for reading PID from motor ID 0x%X: Data=%s", 
                 can_msg.can_id, can_msg.msg.c_str());
    
    // Publish CAN message
    can_pub_->publish(can_msg);
}

void MotorControlNode::sendWritePidCommand(int motor_id, uint8_t pid_index, float value)
{
    auto can_msg = can_bridge_msgs::msg::CanBridge();
    can_msg.can_id = motor_id;
    can_msg.length = 8;
    
    // Convert float to bytes
    uint32_t value_bytes;
    std::memcpy(&value_bytes, &value, sizeof(float));
    
    std::stringstream ss;
    ss << "0x";
    ss << std::hex << std::setw(2) << std::setfill('0') << 0x31;  // Command byte
    ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(pid_index);  // PID index
    ss << std::hex << std::setw(2) << std::setfill('0') << 0x00;  // NULL
    ss << std::hex << std::setw(2) << std::setfill('0') << 0x00;  // NULL
    
    // Value bytes (little-endian)
    ss << std::hex << std::setw(2) << std::setfill('0') << (value_bytes & 0xFF);
    ss << std::hex << std::setw(2) << std::setfill('0') << ((value_bytes >> 8) & 0xFF);
    ss << std::hex << std::setw(2) << std::setfill('0') << ((value_bytes >> 16) & 0xFF);
    ss << std::hex << std::setw(2) << std::setfill('0') << ((value_bytes >> 24) & 0xFF);
    
    can_msg.msg = ss.str();
    
    RCLCPP_DEBUG(this->get_logger(), "CAN message for writing PID to motor ID 0x%X: Data=%s", 
                 can_msg.can_id, can_msg.msg.c_str());
    
    // Publish CAN message
    can_pub_->publish(can_msg);
}

void MotorControlNode::sendFunctionControlCommand(int motor_id, uint8_t function_index, uint32_t value)
{
    auto can_msg = can_bridge_msgs::msg::CanBridge();
    can_msg.can_id = motor_id;
    can_msg.length = 8;
    
    std::stringstream ss;
    ss << "0x";
    ss << std::hex << std::setw(2) << std::setfill('0') << 0x20;  // Command byte
    ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(function_index);  // Function index
    ss << std::hex << std::setw(2) << std::setfill('0') << 0x00;  // NULL
    ss << std::hex << std::setw(2) << std::setfill('0') << 0x00;  // NULL

    // Value bytes (little-endian)
    ss << std::hex << std::setw(2) << std::setfill('0') << (value & 0xFF);
    ss << std::hex << std::setw(2) << std::setfill('0') << ((value >> 8) & 0xFF);
    ss << std::hex << std::setw(2) << std::setfill('0') << ((value >> 16) & 0xFF);
    ss << std::hex << std::setw(2) << std::setfill('0') << ((value >> 24) & 0xFF);

    can_msg.msg = ss.str();

    RCLCPP_DEBUG(this->get_logger(), "CAN message for function control to motor ID 0x%X: Data=%s", 
             can_msg.can_id, can_msg.msg.c_str());

    // Publish CAN message
    can_pub_->publish(can_msg);
}

double MotorControlNode::parsePositionResponse(const can_bridge_msgs::msg::CanBridge::SharedPtr msg)
{
    if (msg->msg.length() < 18) {  // Minimum length for position response (0x + 16 hex chars)
        RCLCPP_WARN(this->get_logger(), "Invalid response message length: %zu", msg->msg.length());
        return 0.0;
    }
 
    // Extract command byte (first byte after "0x")
    std::string cmd_hex = msg->msg.substr(2, 2);
    uint8_t command_byte = std::stoul(cmd_hex, nullptr, 16);
 
    RCLCPP_DEBUG(this->get_logger(), "Parsing position response: cmd=0x%s, full_msg=%s", 
                cmd_hex.c_str(), msg->msg.c_str());
 
    // Process both 0x92 and 0xA4 commands for position data
    if (command_byte != 0x92 && command_byte != 0xA4) {
        RCLCPP_DEBUG(this->get_logger(), "Not a position response command: 0x%02X", command_byte);
        return 0.0;
    }
 
    int32_t position_raw = 0;
 	double position_degrees = 0.0;
    if (command_byte == 0x92) {
        // For 0x92 command: position data is in bytes 4-7 (32-bit)
        std::string position_hex = msg->msg.substr(10, 8);  // Skip "0x92000000"
 
        RCLCPP_DEBUG(this->get_logger(), "0x92 Position hex data: %s", position_hex.c_str());
 
        // Extract individual bytes (little-endian)
        uint8_t byte4 = std::stoul(position_hex.substr(0, 2), nullptr, 16);  // LSB
        uint8_t byte5 = std::stoul(position_hex.substr(2, 2), nullptr, 16);
        uint8_t byte6 = std::stoul(position_hex.substr(4, 2), nullptr, 16);
        uint8_t byte7 = std::stoul(position_hex.substr(6, 2), nullptr, 16);  // MSB
 
        // Combine bytes (MSB to LSB: byte7, byte6, byte5, byte4)
        position_raw = (static_cast<uint32_t>(byte7) << 24) |
                       (static_cast<uint32_t>(byte6) << 16) |
                       (static_cast<uint32_t>(byte5) << 8) |
                       static_cast<uint32_t>(byte4);
 
        RCLCPP_DEBUG(this->get_logger(), "0x92 Position bytes: 0x%02X 0x%02X 0x%02X 0x%02X -> raw=0x%08X", 
                    byte7, byte6, byte5, byte4, position_raw);
        // Convert to degrees (0.01°/LSB)
    position_degrees = static_cast<double>(position_raw) * 0.01;
    }
    else if (command_byte == 0xA4) {
        // For 0xA4 command: position data is in bytes 6-7 (16-bit)
        std::string position_hex = msg->msg.substr(14, 4);  // Skip "0xA4" + 6 bytes
 
        RCLCPP_DEBUG(this->get_logger(), "0xA4 Position hex data: %s", position_hex.c_str());
 
        // Extract individual bytes (little-endian: byte6 is LSB, byte7 is MSB)
        uint8_t byte6 = std::stoul(position_hex.substr(0, 2), nullptr, 16);  // LSB
        uint8_t byte7 = std::stoul(position_hex.substr(2, 2), nullptr, 16);  // MSB
 
        // Combine bytes (MSB to LSB: byte7, byte6)
        position_raw = (static_cast<uint16_t>(byte7) << 8) | static_cast<uint16_t>(byte6);
 
        // Sign extend 16-bit to 32-bit
        if (position_raw & 0x8000) {
            position_raw |= 0xFFFF0000;
        }
 
        RCLCPP_DEBUG(this->get_logger(), "0xA4 Position bytes: 0x%02X 0x%02X -> raw=0x%08X", 
                    byte7, byte6, position_raw);
        // Convert to degrees (1°/LSB)
    position_degrees = static_cast<double>(position_raw);            
    }
 
    
 
    // Convert to radians
    double position_radians = position_degrees * M_PI / 180.0;
 
    RCLCPP_INFO(this->get_logger(), "Position parse (cmd=0x%02X): raw=0x%08X (%d), deg=%.2f, rad=%.6f", 
                command_byte, position_raw, position_raw, position_degrees, position_radians);
 
    return position_radians;
}

float MotorControlNode::parsePidResponse(const can_bridge_msgs::msg::CanBridge::SharedPtr msg)
{
    if (msg->msg.length() < 18) {  // Minimum length for PID response
        RCLCPP_WARN(this->get_logger(), "Invalid response message length for PID read");
        return 0.0;
    }
    
    // Extract PID value from response (bytes 4-7)
    uint32_t value_data = std::stoul(msg->msg.substr(10, 8), nullptr, 16);
    float value;
    std::memcpy(&value, &value_data, sizeof(float));
    
    return value;
}

int MotorControlNode::findMotorIndexByCanId(int can_id)
{
    for (int i = 0; i < motor_count_; i++) {
        if (motor_params_[i].can_id == can_id) {
            return i;
        }
    }
    return -1;
}

void MotorControlNode::handleErrorReport(
    const std::shared_ptr<motor_control_misc::srv::ErrorReport::Request> request,
    std::shared_ptr<motor_control_misc::srv::ErrorReport::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Motor initialization status requested");
    auto start_time = this->now();
    
    // 初始化响应
    response->success = true;
    
    // 检查是否有启用的电机但未初始化
    bool has_enabled_but_not_initialized = false;
    
    // 填充电机初始化状态
    for (int i = 0; i < motor_count_; i++) {
        motor_control_misc::msg::MotorInitStatus init_status;
        init_status.motor_id = i + 1;  // 1-based ID
        
        // 构建状态消息，包含启用状态
        std::string status_message;
        if (motor_params_[i].enabled) {
            status_message = "Enabled - ";
            if (motor_initialized_[i]) {
                auto time_since_init = (this->now() - motor_init_time_[i]).seconds();
                status_message += "Initialized " + std::to_string(time_since_init) + " seconds ago";
            } else {
                status_message += "Not initialized - no response received";
                has_enabled_but_not_initialized = true;
            }
        } else {
            status_message = "Disabled - ";
            if (motor_initialized_[i]) {
                auto time_since_init = (this->now() - motor_init_time_[i]).seconds();
                status_message += "Initialized " + std::to_string(time_since_init) + " seconds ago";
            } else {
                status_message += "Not initialized";
            }
        }
        
        init_status.initialized = motor_initialized_[i];
        init_status.status_message = status_message;
        
        response->motor_init_status.push_back(init_status);
    }
    
    // 如果有启用的电机但未初始化，设置成功标志为0
    if (has_enabled_but_not_initialized) {
        response->success = false;
        RCLCPP_WARN(this->get_logger(), "Some enabled motors are not initialized");
    }
    
    // 填充错误报告（可选，保持兼容性）
    for (int i = 0; i < motor_count_; i++) {
        double position_diff = desired_positions_[i] - current_positions_[i];
        response->error_report.position_diffs.push_back(position_diff);
    }
    
    auto end_time = this->now();
    auto duration = (end_time - start_time).seconds();
    RCLCPP_INFO(this->get_logger(), "Initialization status processing time: %.3f seconds", duration);
    
    // 打印初始化状态摘要
    int initialized_count = 0;
    int enabled_count = 0;
    for (int i = 0; i < motor_count_; i++) {
        if (motor_initialized_[i]) {
            initialized_count++;
        }
        if (motor_params_[i].enabled) {
            enabled_count++;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Initialization status: %d/%d motors initialized, %d/%d motors enabled", 
               initialized_count, motor_count_, enabled_count, motor_count_);
    
    for (const auto& status : response->motor_init_status) {
        RCLCPP_INFO(this->get_logger(), "Motor %d: %s", 
                   status.motor_id,
                   status.status_message.c_str());
    }
}

std::vector<MotorControlNode::MotorError> MotorControlNode::checkMotorErrors()
{
    std::vector<MotorError> all_errors;
    auto now = this->now();
    
    for (int i = 0; i < motor_count_; i++) {
        if (!motor_params_[i].enabled) {
            continue; // 跳过禁用的电机
        }
        
        // 检查电机是否响应
        if (auto error = checkMotorResponding(i)) {
            all_errors.push_back(*error);
            continue; // 如果电机不响应，跳过其他检查
        }
        
        // 检查位置超时（新检查）
        if (auto error = checkMotorPositionTimeout(i)) {
            all_errors.push_back(*error);
        }
        /*
        // 检查电机是否在不应该移动时移动
        if (auto error = checkMotorMovingWhenShouldnt(i)) {
            all_errors.push_back(*error);
        }
        */
        // 检查电机速度（主要检查过快，过慢已在超时检查中处理）
        if (auto error = checkMotorSpeed(i)) {
            all_errors.push_back(*error);
        }
        
        // 检查位置误差
        if (auto error = checkPositionError(i)) {
            all_errors.push_back(*error);
        }
        
        // 检查通信状态
        if (auto error = checkCommunication(i)) {
            all_errors.push_back(*error);
        }
    }
    
    return all_errors;
}

std::optional<MotorControlNode::MotorError> MotorControlNode::checkMotorResponding(int motor_index)
{
    auto now = this->now();
    auto last_response_time = last_response_time_[motor_params_[motor_index].can_id];
 
    // Check if timeout has occurred without receiving response
    double time_since_last_response = (now - last_response_time).seconds();
    if (time_since_last_response > communication_timeout_) {
        // If motor is moving (in timeout check state), don't report not responding error
        if (motor_states_[motor_index].is_moving) {
            RCLCPP_DEBUG(this->get_logger(), "Motor %d is moving, ignoring not responding error", motor_index+1);
            return std::nullopt;
        }
 
        // If motor is not moving but timed out without response, report error
        MotorError error;
        error.motor_index = motor_index;
        error.error_type = MOTOR_NOT_RESPONDING;
        error.description = "Motor not responding for " + std::to_string(time_since_last_response) + " seconds";
        error.severity = 1.0; // High severity
 
        return error;
    }
 
    return std::nullopt;
}

std::optional<MotorControlNode::MotorError> MotorControlNode::checkMotorMovingWhenShouldnt(int motor_index)
{
    // Simplified implementation: if desired position is 0 but actual position is not 0, report error
    if (std::abs(desired_positions_[motor_index]) < 0.001 && 
        std::abs(current_positions_[motor_index]) > 0.001) {
        MotorError error;
        error.motor_index = motor_index;
        error.error_type = MOTOR_MOVING_WHEN_SHOULDNT;
        error.description = "Motor is moving when it should be stationary";
        error.severity = 0.7; // Medium severity
        
        return error;
    }
    
    return std::nullopt;
}

std::optional<MotorControlNode::MotorError> MotorControlNode::checkMotorSpeed(int motor_index)
{
    // 只检查速度是否过快
    if (std::abs(current_velocities_[motor_index]) > max_speed_threshold_) {
        MotorError error;
        error.motor_index = motor_index;
        error.error_type = MOTOR_TOO_FAST;
        error.description = "Motor is moving too fast: " + std::to_string(current_velocities_[motor_index]) +
                           " rad/s (max: " + std::to_string(max_speed_threshold_) + ")";
        error.severity = 0.8; // 高严重性
        
        return error;
    }
    
    return std::nullopt;
}

std::optional<MotorControlNode::MotorError> MotorControlNode::checkPositionError(int motor_index)
{
    double position_error = std::abs(desired_positions_[motor_index] - current_positions_[motor_index]);
    RCLCPP_INFO(this->get_logger(), "Position errorr: %.3f", 
                 position_error);
    /*if (position_error > max_position_error_) {
        MotorError error;
        error.motor_index = motor_index;
        error.error_type = POSITION_ERROR_TOO_LARGE;
        error.description = "Position error too large: " + std::to_string(position_error) +
                           " rad (max: " + std::to_string(max_position_error_) + ")";
        error.severity = 0.9; // High severity
        
        return error;
    }
    */
    return std::nullopt;
}

std::optional<MotorControlNode::MotorError> MotorControlNode::checkCommunication(int motor_index)
{
    // 这里可以添加更复杂的通信质量检查
    // 例如检查CAN总线错误帧、通信延迟等
    
    // Simplified implementation: check communication success rate in recent period
    static std::map<int, int> communication_errors;
    static std::map<int, int> total_messages;
    
    int can_id = motor_params_[motor_index].can_id;
    
    // Initialize counters
    if (communication_errors.find(can_id) == communication_errors.end()) {
        communication_errors[can_id] = 0;
        total_messages[can_id] = 0;
    }
    
    // 这里只是示例，实际实现需要更复杂的逻辑
    // 例如检查CAN总线状态、错误帧计数等
    
    // If communication issues detected, return error
    if (communication_errors[can_id] > 10 && 
        static_cast<float>(communication_errors[can_id]) / total_messages[can_id] > 0.1) {
        MotorError error;
        error.motor_index = motor_index;
        error.error_type = COMMUNICATION_ERROR;
        error.description = "High communication error rate: " + 
                           std::to_string(communication_errors[can_id]) + " errors in " +
                           std::to_string(total_messages[can_id]) + " messages";
        error.severity = 0.6; // Medium severity
        
        return error;
    }
    
    return std::nullopt;
}

std::optional<MotorControlNode::MotorError> MotorControlNode::checkMotorPositionTimeout(int motor_index)
{
    if (!motor_states_[motor_index].is_moving) {
        return std::nullopt;
    }
    
    auto now = this->now();
    double elapsed = (now - motor_states_[motor_index].move_start_time).seconds();
    
    // Only check timeout if we've passed the expected time
    if (elapsed < motor_states_[motor_index].expected_move_time) {
        return std::nullopt;
    }
    
    double current_error = std::abs(motor_states_[motor_index].target_position - current_positions_[motor_index]);
    
    // If we're within tolerance, clear moving state
    if (current_error <= 0.05) {
        motor_states_[motor_index].is_moving = false;
        RCLCPP_INFO(this->get_logger(), "Motor %d reached target position", motor_index+1);
        return std::nullopt;
    }
    
    // Calculate current speed
    double time_since_last = (now - motor_states_[motor_index].last_position_time).seconds();
    double position_change = std::abs(current_positions_[motor_index] - motor_states_[motor_index].last_position);
    double current_speed = (time_since_last > 0.001) ? position_change / time_since_last : 0.0;
    
    // Update last position and time
    motor_states_[motor_index].last_position = current_positions_[motor_index];
    motor_states_[motor_index].last_position_time = now;
    
    // Check if motor has stopped moving
    if (current_speed < 0.001 && elapsed > motor_states_[motor_index].expected_move_time * 1.2) {
        MotorError error;
        error.motor_index = motor_index;
        error.error_type = MOTOR_POSITION_TIMEOUT;
        error.description = "Motor stopped before reaching target: error=" + 
                           std::to_string(current_error) + " rad";
        error.severity = 0.8;
        
        motor_states_[motor_index].is_moving = false;
        return error;
    }
    
    // Check if moving too slowly
    double expected_speed = std::abs(motor_states_[motor_index].target_position - 
                                   motor_states_[motor_index].start_position) / 
                          motor_states_[motor_index].expected_move_time;
    
    if (current_speed < expected_speed * 0.3 && elapsed > motor_states_[motor_index].expected_move_time * 1.5) {
        MotorError error;
        error.motor_index = motor_index;
        error.error_type = MOTOR_TOO_SLOW;
        error.description = "Motor moving too slowly: current=" + 
                           std::to_string(current_speed) + " rad/s, expected=" + 
                           std::to_string(expected_speed) + " rad/s";
        error.severity = 0.6;
        return error;
    }
    
    return std::nullopt;
}

rclcpp_action::GoalResponse MotorControlNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CheckErrorAction::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received check error goal request");
    (void)uuid;
    
    // 验证目标位置数组大小
    if (goal->target_positions.size() != static_cast<size_t>(motor_count_)) {
        RCLCPP_WARN(this->get_logger(), "Invalid target positions size: %zu, expected %d", 
                   goal->target_positions.size(), motor_count_);
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 处理取消请求
rclcpp_action::CancelResponse MotorControlNode::handle_cancel(
    const std::shared_ptr<GoalHandleCheckError> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 处理接受的目标
void MotorControlNode::handle_accepted(const std::shared_ptr<GoalHandleCheckError> goal_handle)
{
    // 使用线程执行检查，避免阻塞执行器
    std::thread{std::bind(&MotorControlNode::execute_check_error, this, std::placeholders::_1), goal_handle}.detach();
}

void MotorControlNode::execute_check_error(const std::shared_ptr<GoalHandleCheckError> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing check error action");
 
    auto result = std::make_shared<CheckErrorAction::Result>();
    auto feedback = std::make_shared<CheckErrorAction::Feedback>();
 
    const auto goal = goal_handle->get_goal();
 
    try {
        // Publish feedback - starting movement
        feedback->status = "Starting movement to target position";
        feedback->progress = 0.0;
        // Initialize position errors
        feedback->position_errors.clear();
        for (int i = 0; i < motor_count_; i++) {
            if (i < static_cast<int>(goal->target_positions.size())) {
                feedback->position_errors.push_back(goal->target_positions[i] - current_positions_[i]);
            } else {
                feedback->position_errors.push_back(0.0);
            }
        }
        goal_handle->publish_feedback(feedback);
 
        // Save original desired positions
        std::vector<double> original_desired_positions = desired_positions_;
 
        // Set target positions
        {
            std::lock_guard<std::mutex> lock(motor_data_mutex_);
            for (int i = 0; i < motor_count_; i++) {
                if (i < static_cast<int>(goal->target_positions.size())) {
                    desired_positions_[i] = goal->target_positions[i];
                }
            }
        }
 
        // Calculate timeout time points for each motor
        std::vector<rclcpp::Time> motor_timeout_times(motor_count_);
        std::vector<bool> motor_completed(motor_count_, false); // Flag if motor has completed
        std::vector<bool> motor_in_timeout_check(motor_count_, false); // Flag if motor is in timeout check
        std::vector<double> motor_check_positions(motor_count_, 0.0); // Record position when checking
        std::vector<rclcpp::Time> motor_check_times(motor_count_); // Record check time
 
        // Store final positions when motors complete
        std::vector<double> final_positions(motor_count_, 0.0);
 
        // Store initial positions for debugging
        std::vector<double> initial_positions = current_positions_;
 
        // Send position commands to motors
        for (int i = 0; i < motor_count_; i++) {
            if (motor_params_[i].enabled && i < static_cast<int>(goal->target_positions.size())) {
                // Calculate expected move time
                double distance_rad = std::abs(goal->target_positions[i] - current_positions_[i]);
                double distance_deg = distance_rad * 180.0 / M_PI;
 
                // Ensure max_speed is not zero to avoid division by zero
                double max_speed = motor_params_[i].max_speed;
                if (max_speed <= 0.0) {
                    max_speed = 1.0; // Default to 1 dps if invalid
                    RCLCPP_WARN(this->get_logger(), "Motor %d has invalid max_speed (%.1f), using default", i+1, motor_params_[i].max_speed);
                }
 
                // Increase minimum expected time to avoid premature timeout
                expected_move_time_[i] = std::max((distance_deg / max_speed) * position_timeout_factor_, 1.0);
 
                // Calculate timeout time point
                motor_timeout_times[i] = this->now() + rclcpp::Duration::from_seconds(expected_move_time_[i]);
 
                RCLCPP_INFO(this->get_logger(), "Motor %d: initial=%.3f, target=%.3f, distance=%.3f deg, max_speed=%.1f dps, expected_time=%.3fs", 
                           i+1, initial_positions[i], goal->target_positions[i], distance_deg, max_speed, expected_move_time_[i]);
 
                // Send position command
                sendPositionCommand(goal->target_positions[i], motor_params_[i].max_speed, motor_params_[i].can_id);
 
                RCLCPP_INFO(this->get_logger(), "Sent position command to motor %d", i+1);
            } else {
                RCLCPP_INFO(this->get_logger(), "Motor %d is disabled or no target position", i+1);
            }
        }
 
        // Publish feedback - moving
        feedback->status = "Moving to target position";
        feedback->progress = 0.3;
        // Update position errors
        feedback->position_errors.clear();
        for (int i = 0; i < motor_count_; i++) {
            if (i < static_cast<int>(goal->target_positions.size())) {
                feedback->position_errors.push_back(goal->target_positions[i] - current_positions_[i]);
            } else {
                feedback->position_errors.push_back(0.0);
            }
        }
        goal_handle->publish_feedback(feedback);
 
        // Wait for motors to reach target position or timeout
        auto start_time = this->now();
        bool all_motors_completed = false;
        std::vector<MotorError> detected_errors;
 
        while (rclcpp::ok() && !all_motors_completed) {
            // Check if all motors have completed
            all_motors_completed = true;
            auto current_time = this->now();
 
            // Check total timeout (10 seconds)
            double total_elapsed_time = (current_time - start_time).seconds();
            if (total_elapsed_time > 10.0) {
                RCLCPP_WARN(this->get_logger(), "Total timeout after %.1f seconds", total_elapsed_time);
                break;
            }
 
            // Check each motor status
            for (int i = 0; i < motor_count_; i++) {
                if (motor_completed[i] || !motor_params_[i].enabled || 
                    i >= static_cast<int>(goal->target_positions.size())) {
                    continue; // Skip completed, disabled or motors without target position
                }
 
                double position_error = std::abs(goal->target_positions[i] - current_positions_[i]);
 
                // Debug: Log current position and error
                RCLCPP_DEBUG(this->get_logger(), "Motor %d: current=%.3f, target=%.3f, error=%.3f", 
                           i+1, current_positions_[i], goal->target_positions[i], position_error);
 
                // Check if target position is reached
                if (position_error <= 0.05) { // 0.05 rad tolerance
                    motor_completed[i] = true;
                    // Update motor state
                    motor_states_[i].is_moving = false;
                    RCLCPP_INFO(this->get_logger(), "Motor %d reached target position: %.3f", i+1, current_positions_[i]);
                    continue;
                }
 
                // Check if in timeout check
                if (motor_in_timeout_check[i]) {
                    // Check if 100ms has passed
                    if ((current_time - motor_check_times[i]).seconds() >= 0.1) {
                        // Calculate position change
                        double position_change = std::abs(current_positions_[i] - motor_check_positions[i]);
 
                        RCLCPP_INFO(this->get_logger(), "Motor %d timeout check: position_change=%.6f", i+1, position_change);
 
                        // Check again if target position is reached
                        if (position_error <= 0.05) {
                            motor_completed[i] = true;
                            // Update motor state
                            motor_states_[i].is_moving = false;
                            RCLCPP_INFO(this->get_logger(), "Motor %d reached target position during timeout check: %.3f", i+1, current_positions_[i]);
                            continue;
                        }
 
                        if (position_change < 0.001) { // Position change below threshold, consider motor stopped
                            // Report timeout error
                            MotorError timeout_error;
                            timeout_error.motor_index = i;
                            timeout_error.error_type = MOTOR_POSITION_TIMEOUT;
                            timeout_error.description = "Motor position timeout: error=" + std::to_string(position_error) + 
                                                      " rad, motor stopped";
                            timeout_error.severity = 0.9;
                            detected_errors.push_back(timeout_error);
 
                            RCLCPP_WARN(this->get_logger(), "Motor %d timeout: motor stopped, position error=%.3f rad", 
                                       i+1, position_error);
                        } else {
                            // Report too slow error
                            MotorError slow_error;
                            slow_error.motor_index = i;
                            slow_error.error_type = MOTOR_TOO_SLOW;
                            slow_error.description = "Motor moving too slow: error=" + std::to_string(position_error) + 
                                                   " rad, still moving";
                            slow_error.severity = 0.7;
                            detected_errors.push_back(slow_error);
 
                            RCLCPP_WARN(this->get_logger(), "Motor %d moving too slow: position error=%.3f rad", 
                                       i+1, position_error);
                        }
 
                        // Update motor state
                        motor_states_[i].is_moving = false;
                        motor_completed[i] = true;
                    } else {
                        all_motors_completed = false; // Still waiting for check result
                    }
                } else {
                    // Check if timeout time point is reached
                    if (current_time >= motor_timeout_times[i]) {
                        // Start timeout check
                        motor_in_timeout_check[i] = true;
                        motor_check_positions[i] = current_positions_[i];
                        motor_check_times[i] = current_time;
 
                        RCLCPP_INFO(this->get_logger(), "Motor %d timeout reached, checking position change. Current position: %.3f", 
                                   i+1, current_positions_[i]);
                    } else {
                        all_motors_completed = false; // Still waiting for timeout time
 
                        // Debug: Log time remaining until timeout
                        double time_to_timeout = (motor_timeout_times[i] - current_time).seconds();
                        RCLCPP_DEBUG(this->get_logger(), "Motor %d: time to timeout: %.3fs", i+1, time_to_timeout);
                    }
                }
            }
 
            // Calculate overall progress
            double max_progress = 0.0;
            int active_motors = 0;
            for (int i = 0; i < motor_count_; i++) {
                if (motor_params_[i].enabled && i < static_cast<int>(goal->target_positions.size())) {
                    active_motors++;
                    double initial_error = std::abs(goal->target_positions[i] - original_desired_positions[i]);
                    double current_error = std::abs(goal->target_positions[i] - current_positions_[i]);
                    double progress = 1.0 - (current_error / std::max(initial_error, 0.001));
                    max_progress = std::max(max_progress, progress);
                }
            }
 
            // Publish progress feedback with position errors
            feedback->status = "Moving to target position";
            feedback->progress = std::min(0.3 + max_progress * 0.6, 0.95); // 30%-95% progress
 
            // Update position errors in feedback
            feedback->position_errors.clear();
            for (int i = 0; i < motor_count_; i++) {
                if (i < static_cast<int>(goal->target_positions.size())) {
                    double error = goal->target_positions[i] - current_positions_[i];
                    feedback->position_errors.push_back(error);
                    RCLCPP_DEBUG(this->get_logger(), "Motor %d position error: %.3f", i+1, error);
                } else {
                    feedback->position_errors.push_back(0.0);
                }
            }
 
            goal_handle->publish_feedback(feedback);
 
            // Brief sleep
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
 
        // Wait a bit more for motors to stabilize
        RCLCPP_INFO(this->get_logger(), "Waiting for motors to stabilize...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
 
        // Final position check after stabilization
        for (int i = 0; i < motor_count_; i++) {
            if (motor_params_[i].enabled && i < static_cast<int>(goal->target_positions.size())) {
                double position_error = std::abs(goal->target_positions[i] - current_positions_[i]);
                RCLCPP_INFO(this->get_logger(), "Motor %d final check: current=%.3f, target=%.3f, error=%.3f", 
                           i+1, current_positions_[i], goal->target_positions[i], position_error);
 
                // Update final position
                final_positions[i] = current_positions_[i];
            }
        }
 
        // Publish feedback - checking errors
        feedback->status = "Checking motor errors";
        feedback->progress = 0.99;
        // Update position errors one last time
        feedback->position_errors.clear();
        for (int i = 0; i < motor_count_; i++) {
            if (i < static_cast<int>(goal->target_positions.size())) {
                double error = goal->target_positions[i] - final_positions[i];
                feedback->position_errors.push_back(error);
            } else {
                feedback->position_errors.push_back(0.0);
            }
        }
        goal_handle->publish_feedback(feedback);
 
        // Check other types of errors (excluding too slow, as already handled)
        std::vector<MotorError> additional_errors = checkMotorErrors();
 
        RCLCPP_INFO(this->get_logger(), "Additional errors detected: %zu", additional_errors.size());
        for (const auto& error : additional_errors) {
            RCLCPP_INFO(this->get_logger(), "Additional error: motor %d, type %s", 
                       error.motor_index + 1, error_type_names_[error.error_type].c_str());
        }
 	
 	
 	
        // Filter out too slow errors, as already handled in above logic
        // Also filter out false positive not responding errors
        for (auto it = additional_errors.begin(); it != additional_errors.end(); ) {
            /*if (it->error_type == MOTOR_TOO_SLOW) {
                it = additional_errors.erase(it);
            } else*/ if (it->error_type == MOTOR_NOT_RESPONDING) {
                // For not responding errors, check if motor has actually completed or is moving
                int motor_index = it->motor_index;
                if (motor_index < motor_count_ && 
                    (motor_completed[motor_index] || motor_states_[motor_index].is_moving)) {
                    // If motor has completed or is moving, remove this error
                    it = additional_errors.erase(it);
                } else {
                    ++it;
                }
            } else {
                ++it;
            }
        }
 
        detected_errors.insert(detected_errors.end(), additional_errors.begin(), additional_errors.end());
 
        RCLCPP_INFO(this->get_logger(), "Total errors after filtering: %zu", detected_errors.size());
 
        // Calculate position differences using final positions after stabilization
        for (int i = 0; i < motor_count_; i++) {
            if (i < static_cast<int>(goal->target_positions.size())) {
                double position = final_positions[i];
                double position_diff = goal->target_positions[i] - position;
                result->error_report.position_diffs.push_back(position_diff);
 
                RCLCPP_INFO(this->get_logger(), "Motor %d: target=%.3f, final=%.3f, diff=%.3f", 
                           i+1, goal->target_positions[i], position, position_diff);
            } else {
                result->error_report.position_diffs.push_back(0.0);
            }
        }
 
        // Now restore original desired positions
        {
            std::lock_guard<std::mutex> lock(motor_data_mutex_);
            desired_positions_ = original_desired_positions;
        }
 
        // Set result
        if (detected_errors.empty()) {
            result->result = 1; // Success
            RCLCPP_INFO(this->get_logger(), "Check error action completed successfully");
        } else {
            result->result = 0; // Not successful
            RCLCPP_WARN(this->get_logger(), "Check error action completed with %zu errors", detected_errors.size());
        }
 
        // Fill error report
        for (const auto& error : detected_errors) {
            result->error_report.motor_ids.push_back(error.motor_index + 1);
            result->error_report.error_types.push_back(error_type_names_[error.error_type]);
            result->error_report.error_descriptions.push_back(error.description);
            result->error_report.severities.push_back(error.severity);
        }
 
        // Publish final feedback
        feedback->status = "Action completed";
        feedback->progress = 1.0;
        // Update position errors one final time
        feedback->position_errors.clear();
        for (int i = 0; i < motor_count_; i++) {
            if (i < static_cast<int>(goal->target_positions.size())) {
                double error = goal->target_positions[i] - final_positions[i];
                feedback->position_errors.push_back(error);
            } else {
                feedback->position_errors.push_back(0.0);
            }
        }
        goal_handle->publish_feedback(feedback);
 
        // Set final result
        goal_handle->succeed(result);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in execute_check_error: %s", e.what());
        goal_handle->abort(result);
    }
}

void MotorControlNode::handleResetService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;  // Unused parameter
    
    RCLCPP_INFO(this->get_logger(), "Received motor reset request");
    
    // Send reset command to all enabled motors
    for (int i = 0; i < motor_count_; i++) {
        if (motor_params_[i].enabled) {
            sendResetCommand(motor_params_[i].can_id);
            RCLCPP_INFO(this->get_logger(), "Sending reset command to motor %d (CAN ID: 0x%X)", 
                       i + 1, motor_params_[i].can_id);
        }
    }
    
    response->success = true;
    response->message = "Reset commands sent to all enabled motors";
}

void MotorControlNode::sendResetCommand(int motor_id)
{
    auto can_msg = can_bridge_msgs::msg::CanBridge();
    can_msg.can_id = motor_id;
    can_msg.length = 8;
    
    std::stringstream ss;
    ss << "0x";
    ss << std::hex << std::setw(2) << std::setfill('0') << 0x76;  // Command byte
    
    // Fill rest with zeros
    for (int i = 0; i < 7; i++) {
        ss << std::hex << std::setw(2) << std::setfill('0') << 0x00;
    }
    
    can_msg.msg = ss.str();
    
    RCLCPP_DEBUG(this->get_logger(), "Sending reset command to motor ID 0x%X: Data=%s", 
                 can_msg.can_id, can_msg.msg.c_str());
    
    // Publish CAN message
    can_pub_->publish(can_msg);
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

