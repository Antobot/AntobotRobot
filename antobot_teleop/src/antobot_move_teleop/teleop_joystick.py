#!/usr/bin/env python3

# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Functional Description:     

# Contact: soyoung.kim@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time
import math

class teleop_joystick:
    def __init__(self):
                
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

        self.debug=False # Set to true to log joystick inputs

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
        
        rospy.loginfo("Joystick Teleop Initialised")
        
        # initialise linear and angular velocities
        self.v = 0
        self.w = 0
        
        # Setup ROS subscriber
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)  # subscriber for Joy message

        
    def joy_callback(self, msg):
        # # # Callback function for the joystick
        # Inputs: msg [Joy message] - Used for reading the Joystick axes and buttons
        
        if (self.debug == 1):
            rospy.loginfo("joy")
            rospy.loginfo(" axis linear: %s", msg.axes[self.axis_analog_left])
            rospy.loginfo(" axis angular: %s", msg.axes[self.axis_analog_right])
        
            rospy.loginfo(" axis dpad back/forward: %s", msg.axes[self.axis_dpad_back_forward])
            rospy.loginfo(" axis dpad left/right: %s", msg.axes[self.axis_dpad_left_right])
            
            rospy.loginfo(" LT axis: %s", msg.axes[self.axis_lt])
            rospy.loginfo(" RT axis: %s", msg.axes[self.axis_rt])
        
            # check the button statuses
            for i in range(0, 11):
                if (self.buttons[i] == 1 and self.buttons_previous[i] == 0):
                    rospy.loginfo(" %s pressed", self.button_names[i])
        
        # copy the buttons and axes vars to class variables
        self.buttons = msg.buttons
        self.axes = msg.axes
        

        
        # main state machine for reading the joystick buttons

        
        if (self.buttons[self.button_lb] == 1 and self.buttons[self.button_rb] == 1):
            rospy.loginfo("Teleop: Joystick active")
            self.trigger_releaseForceStop=True
            self.trigger_active=True


        if (self.buttons[self.button_back] == 1 and self.buttons[self.button_start] == 1):
            self.trigger_shutdown=True            
            
        
        # if X button is pressed (brake)
        if (self.buttons[self.button_x] == 1): #and self.buttons_previous[self.button_x] == 0):
            self.v = 0
            self.w = 0                           
            rospy.loginfo("Brake")
        else:                
            self.v = self.axes[self.axis_analog_left] * self.max_lin_speed
            self.w = self.axes[self.axis_analog_right] * self.max_ang_speed
    
        self.buttons_previous = msg.buttons
        
        self.cmd_vel.linear.x = self.v
        self.cmd_vel.angular.z = self.w
        
        self.lastUpdate = time.time()
