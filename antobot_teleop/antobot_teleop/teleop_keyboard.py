#!/usr/bin/env python3

# Copyright (c) 2025, ANTOBOT LTD.
# All rights reserved.

################################################################################################################################################
# # # Code Description:     Allows for keyboard teleoperation of the robot, using the right side of the keyboard for control commands: [[u, i, o],
# # #                       and the left side to increase/decrease speed: [q/z] (both), [w/x] (linear), e/c (angular)                   [j, k, l],
# # #                       It can be imported into a teleop.py script to enable teleoperation using keyboard inputs                    [m, ,, .]]

# Contact: soyoung.kim@antobot.ai

################################################################################################################################################


import rclpy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import time

class teleop_keyboard:
    def __init__(self,node):

        self.node = node
        # All teleop classes require a cmd_vel
        self.cmd_vel = Twist()

        # The default last update to prevent the data being used
        self.lastUpdate=0

        # Map variables to each button that should trigger a function
        self.trigger_active=False   # Keyboard is always actived when a key is pressed
        self.trigger_shutdown=False
        self.trigger_releaseForceStop=False # A way of triggering this is needed

        self.debug=False # Set to true to log keyboard inputs

        # Set a timeout period, after which the keyboard is assumed to no-longer be in use
        self.timeOut=5 # seconds


        # Defines move bindings (keys associated with robot control)
        self.moveBindings = {
            'i':(1,0),
            'o':(1,-1),
            'j':(0,1),
            'l':(0,-1),
            'u':(1,1),
            ',':(-1,0),
            '.':(-1,1),
            'm':(-1,-1),
            }

        # Defines speed bindings (keys associated with increasing or decreasing max linear/angular velocity)
        self.speedBindings={
            'q':(1.1,1.1),
            'z':(.9,.9),
            'w':(1.1,1),
            'x':(.9,1),
            'e':(1,1.1),
            'c':(1,.9),
            }

        # Sets keyboard input settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Defines initial values for class variables
        self.x = 0
        self.th = 0
        self.status = 0
        self.count= 0
        self.acc = 0.1
        self.target_speed = 0
        self.target_turn = 0
        self.control_speed = 0
        self.control_turn = 0

        # Defines initial values for linear and angular velocity
        self.speed = 0.7
        self.turn = 1

################################################################################################

    def print_msg(self):
        # # # Prints the message explaining the robot controls to terminal
        
        self.msg = """
        Control robot!
        ---------------------------
        Moving around:
           u    i    o
           j    k    l
           m    ,    .

        q/z : increase/decrease max speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%
        space key, k : force stop
        anything else : stop smoothly

        CTRL-C to quit
        """

        print(self.msg)

################################################################################################

    def vels(self, speed, turn):
        # # # Returns a printout of the linear and angular velocities as a string
        # Returns: vels
        return "currently:\tspeed %s\tturn %s " % (speed,turn)

################################################################################################
    
    def update(self):

        #############################################
        # Sample keyboard
        #############################################

        # Begin by sampling keyboard input returns it as a character
        # Returns: key - For example: 'c', 'k', or '\x03' (ctrl+c)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.03)
        if rlist:
            key = sys.stdin.read(1)
            self.lastUpdate = time.time() # Track when this was last updated

        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        #############################################
        # Determine user input
        #############################################

        # Begin by zero-ing the change to the current output (not the output itself)
        self.x = 0
        self.th = 0

        # Check if the last keyboard entry has timed out
        now=time.time()
        if (now-self.lastUpdate)>self.timeOut:

            # Zero the actual output
            self.control_speed = 0
            self.control_turn = 0

            self.trigger_active=False

        else: # If the keyboard has not timed out

            self.trigger_active=True


            # Based on the key input, perform actions and generate a cmd_vel
            # Motion control direction key (1: positive direction, -1 negative direction)
            if key in self.moveBindings.keys():
                self.x = self.moveBindings[key][0]
                self.th = self.moveBindings[key][1]

            # Speed modifier key
            elif key in self.speedBindings.keys():
                self.speed = self.speed * self.speedBindings[key][0]  # Linear speed increased by 10%
                self.turn = self.turn * self.speedBindings[key][1]  # Angular velocity increased by 10%

                self.node.get_logger().info(self.vels(self.speed, self.turn))     # prints current speeds if they are changed
                if (self.status == 14):
                    self.node.get_logger().info(self.msg)             # Re-prints teloperation instructions if 15 speed modifications have occurred
                self.status = (self.status + 1) % 15

            # Stop button or no input
            elif key == 'k' or key==' ':
                self.x = 0
                self.th = 0
                self.control_speed = 0
                self.control_turn = 0
            elif (key == '\x03'):         # ctrl+c
                pass # Pressing ctrl+c Should stop the function itself





        #############################################
        # Calculate cmd_vel
        #############################################

        # Target speed = speed value * direction value
        self.target_speed = self.speed * self.x
        self.target_turn = self.turn * self.th



        # Speed limit to prevent the speed from increasing or decreasing too fast
        if self.target_speed > self.control_speed:
            self.control_speed = min(self.target_speed, self.control_speed + 0.02)
        elif self.target_speed < self.control_speed:
            self.control_speed = max(self.target_speed, self.control_speed - 0.02)
        else:
            self.control_speed = self.target_speed

        if self.target_turn > self.control_turn:
            self.control_turn = min(self.target_turn, self.control_turn + 0.1)
        elif self.target_turn < self.control_turn:
            self.control_turn = max(self.target_turn, self.control_turn - 0.1)
        else:
            self.control_turn = self.target_turn

        # Update cmd_vel
        self.cmd_vel.linear.x = float(self.control_speed)
        self.cmd_vel.angular.z = float(self.control_turn)







