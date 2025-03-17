#!/usr/bin/env python3

# Copyright (c) 2025, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Functional Description: The script defines a teleop_joystick class that listens for inputs from a connected joystick (or game controller). 
# It maps the joystick axes and buttons to control movements or commands (e.g., linear and angular velocities for robot motion). 
# It can be imported into a teleop.py script to enable teleoperation using joystick inputs

# Contact: soyoung.kim@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time



class teleop_joystick:
    def __init__(self,node):  

        self.node = node  
        # All teleop classes require a cmd_vel
        self.cmd_vel = Twist()
        self.max_lin_speed = 1.3
        self.max_ang_speed = 1.3

        # The default last update to prevent the data being used
        self.lastUpdate=0

        # Map variables to each button that should trigger a function
        self.trigger_active=False   

        self.trigger_shutdown=False
        self.trigger_releaseForceStop=False # This is needed


        # Joystick axis and button configuration
        # axis and button values are their position within the vectors in /joy message (e.g. 'A' button is 1st element in buttons array, 'B' button is 2nd element)

        self.debug=True # Set to true to log joystick inputs

        # axis numbers
        self.axis_analog_left = 1  # left joystick up/down axis number (linear velocity)
        self.axis_analog_right = 3  # right joystick left/right axis number (angular velocity)
        self.axis_dpad_back_forward = 7  # dpad up/down axis number
        self.axis_dpad_left_right = 6  # dpad left/right axis number        
        self.axis_lt = 2 # left trigger axis number
        self.axis_rt = 5 # right trigger axis number (modifier)

        # button numbers
        self.button_a = 0
        self.button_b = 1
        self.button_x = 2  # brake
        self.button_y = 3
        self.button_lb = 4  # init manual control with rb
        self.button_rb = 5  # init manual control with lb
        self.button_back = 6 # back button
        self.button_start = 7 # start button
        self.button_logitech = 8 # logitech button
        self.button_ls = 9  # left joystick button
        self.button_rs = 10  # right joystick button
        
        # button description strings
        self.button_names = [""] * 11
        self.button_names[self.button_a] = "Button A"
        self.button_names[self.button_b] = "Button B"
        self.button_names[self.button_x] = "Button X"
        self.button_names[self.button_y] = "Button Y"
        self.button_names[self.button_lb] = "Button LB"
        self.button_names[self.button_rb] = "Button RB"
        self.button_names[self.button_back] = "Button BACK"
        self.button_names[self.button_start] = "Button START"
        self.button_names[self.button_logitech] = "Button LOGITECH"        
        self.button_names[self.button_ls] = "Button LS"
        self.button_names[self.button_rs] = "Button RS"
        
        # initialise axes and buttons arrays
        self.axes = [0] * 8   # 8 elements in axis array
        self.buttons = [0] * 11   # 11 elements in buttons array
        self.buttons_previous = self.buttons  # previous buttons pressed
        
        self.node.get_logger().info("Joystick Teleop Initialised ROS2")
        
        # initialise linear and angular velocities
        self.v = 0
        self.w = 0
        
        # Setup ROS subscriber
        self.joy_sub = self.node.create_subscription(Joy,"/joy",self.joy_callback,10)  # subscriber for Joy message

        
    def joy_callback(self, msg):
        # # # Callback function for the joystick
        # Inputs: msg [Joy message] - Used for reading the Joystick axes and buttons
        
        if (self.debug == 1):
            self.node.get_logger().info("joy")
            self.node.get_logger().info(" axis linear: %s", msg.axes[self.axis_analog_left])
            self.node.get_logger().info(" axis angular: %s", msg.axes[self.axis_analog_right])
        
            self.node.get_logger().info(" axis dpad back/forward: %s", msg.axes[self.axis_dpad_back_forward])
            self.node.get_logger().info(" axis dpad left/right: %s", msg.axes[self.axis_dpad_left_right])
            
            self.node.get_logger().info(" LT axis: %s", msg.axes[self.axis_lt])
            self.node.get_logger().info(" RT axis: %s", msg.axes[self.axis_rt])
        
            # check the button statuses
            for i in range(0, 11):
                if (self.buttons[i] == 1 and self.buttons_previous[i] == 0):
                    self.node.get_logger().info(" %s pressed", self.button_names[i])
        
        # copy the buttons and axes vars to class variables
        self.buttons = msg.buttons
        self.axes = msg.axes
        

        
        # main state machine for reading the joystick buttons

        
        if (self.buttons[self.button_lb] == 1 and self.buttons[self.button_rb] == 1):
            self.node.get_logger().info("Teleop: Joystick active")
            self.trigger_releaseForceStop=True
            self.trigger_active=True


        if (self.buttons[self.button_back] == 1 and self.buttons[self.button_start] == 1):
            self.trigger_shutdown=True            
            
        
        # if X button is pressed (brake)
        if (self.buttons[self.button_x] == 1): #and self.buttons_previous[self.button_x] == 0):
            self.v = 0
            self.w = 0                           
            self.node.get_logger().info("Brake")
        else:                
            self.v = self.axes[self.axis_analog_left] * self.max_lin_speed
            self.w = self.axes[self.axis_analog_right] * self.max_ang_speed
    
        self.buttons_previous = msg.buttons
        
        self.cmd_vel.linear.x = self.v
        self.cmd_vel.angular.z = self.w
        
        self.lastUpdate = time.time()
