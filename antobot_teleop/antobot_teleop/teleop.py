#!/usr/bin/env python3

# Copyright (c) 2025, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Functional Description:     Creates a master program to allow for switching between our main control methods:
# # #                       1) Keyboard control (inherited from teleop_keyboard) and 2) Joystick control (inherited from teleop_joystick)

# Contact: soyoung.kim@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import sys
import time

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


# Import the different methods of input
from antobot_teleop.teleop_joystick import teleop_joystick
from antobot_teleop.teleop_keyboard import teleop_keyboard



class MasterTeleop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.timer = self.create_timer(1.0/30, self.update)

        # Create classes for each input method
        self.teleop_method=dict()
        self.teleop_method[0] = teleop_joystick(self)
        self.teleop_method[1] = teleop_keyboard(self)
        self.teleop_mode=None # No method is the default

        # Set a timeout, after which a teleop input is no longer considered active
        self.inputTimeout=10 # seconds
        
        # Set default output states
        self.force_stop_release = False
        self.soft_shutdown = False

        # Set states for master teleop
        self.trigger_active=False # Tracks if the teleop type is active

        self.trigger_shutdown=False
        self.trigger_releaseForceStop=False

        # Create an empty cmd_vel twist
        self.teleop_cmd_vel=Twist()
        
        self.force_stop_release_pub = self.create_publisher(Bool,'/antobridge/force_stop_release',  1)  # publisher to release the force stop on Aurix
        self.soft_shutdown_pub = self.create_publisher(Bool,'/antobridge/soft_shutdown_button',1) # publisher to send the soft shutdown signal to antobridge

        # Publisher for physical robot movement
        self.teleop_cmd_vel_pub = self.create_publisher(Twist,'/antobot_ant/cmd_vel', 5)


####################################################################################

    def update(self,event=None):

        # The keyboard mode is currently the only mode that doesnt self update. Request the currently held keys each turn
        # teleop_keyboard can only handle a single key press at once
        self.teleop_method[1].update()

        # Get the current time and compare to when each of the teleop methods were last updated
        now=time.time()

        # Reset the teleop_mode
        self.teleop_mode=None

        # The priority order is established here
        # Priority is given to joystick > terminal(keyboard) > webUI > mqtt (not implemented)
        # If a mode has timed out, deactivate it. DO NOT ACTIVATE MODES HERE - that is manually triggered by each teleop method

        if (now-self.teleop_method[0].lastUpdate)<self.inputTimeout:
            self.teleop_mode=0 # Set the index for the active method
            self.teleop_method[1].trigger_active=False # De-activate the other methods

        elif (now-self.teleop_method[1].lastUpdate)<self.inputTimeout:
            self.teleop_mode=1 # Set the index for this method
            self.teleop_method[0].trigger_active=False # De-activate the other methods
        else: # De-activate all methods
            self.teleop_method[0].trigger_active=False
            self.teleop_method[1].trigger_active=False


        # Update the triggers and cmd_vel
        if self.teleop_mode is not None:

            # Update the master triggers based on the individual method being used
            self.trigger_active=self.teleop_method[self.teleop_mode].trigger_active
            self.trigger_shutdown=self.teleop_method[self.teleop_mode].trigger_shutdown
            self.trigger_releaseForceStop=self.teleop_method[self.teleop_mode].trigger_releaseForceStop

            # Reset the internal triggers onthe selected method
            # Do not reset trigger_active state
            self.teleop_method[self.teleop_mode].trigger_shutdown=False
            self.teleop_method[self.teleop_mode].trigger_releaseForceStop=False

            # Update cmd_vel
            self.teleop_cmd_vel=self.teleop_method[self.teleop_mode].cmd_vel

        else: # No active teleop
            # Set states for master teleop
            self.trigger_active=False # Triggers when any teleop type is active
            self.trigger_shutdown=False
            self.trigger_releaseForceStop=False

            # Create an empty cmd_vel twist
            self.teleop_cmd_vel=Twist()



        ######################################
        # Shutdown calls
        ######################################

        if self.trigger_shutdown:
            self.get_logger().info("Xavier will be powered off now")
            self.soft_shutdown_pub.publish(True)
            self.trigger_shutdown = False # Stop from double sending
            # As the robot will immediately begin powering down, there is no need to reset the request#
            # But it does prevent the system constantly resending the same request



        ######################################
        # Release safety locks
        ######################################

        # Only perform actions if the teleop method is active
        if self.trigger_active:

            if self.trigger_releaseForceStop:

                self.trigger_releaseForceStop=False # Stop from double sending

                self.force_stop_release = True
                self.force_stop_release_pub.publish(self.force_stop_release)
            

            ######################################
            # Drive robot
            ######################################

            self.teleop_cmd_vel_pub.publish(self.teleop_cmd_vel)


def main(args=None):
    rclpy.init()
    try:
        mt = MasterTeleop()
        rclpy.spin(mt)
    except KeyboardInterrupt:
        sys.exit(1)
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        mt.destroy_node()


####################################################################################

# Main
if __name__ == '__main__':
    main()
    

