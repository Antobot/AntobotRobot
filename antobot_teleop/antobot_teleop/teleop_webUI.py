#!/usr/bin/env python3

# Copyright (c) 2025, ANTOBOT LTD.
# All rights reserved.

################################################################################################################
### Functional Description:     Teleoperation class for webUI
################################################################################################################

import rclpy
from geometry_msgs.msg import Twist
import time # Use system time as ROS time can lag, and this is a human interface

class teleop_webUI:
    def __init__(self, teleop_node):

        self.logger = teleop_node.get_logger()
        
        # All teleop classes require a cmd_vel
        self.cmd_vel = Twist()

        # The cmd_vel data is taken directly from the webUI topic
        self.webUI_cmd_vel_sub = teleop_node.create_subscription(Twist, "/webUI/cmd_vel", self.cmd_vel_callback, 10)  # subscriber for Joy message

        # The default last update to prevent the data being used
        self.lastUpdate=0

        # Create a service to allow webUI to send trigger commands. These do not go directly to allow webUI to be cutoff by local user.
        # self.serviceUserInput = teleop_node.create_service(JobManagerUserInput, '/antobot/teleop/webUIinput', self.__serviceCallbackUserInput)

        # Map variables to each button that should trigger a function
        
        self.trigger_active=False                   # Triggers the teleop function to use this mode
                                                    #  0: Test - Doesn't do anything
        self.trigger_releaseForceStop=False         #  8: Release force stop
        self.trigger_shutdown=False
        self.debug=False # Set to true to log webUI inputs

    def cmd_vel_callback(self, data):
        # Simply updates the cmd_vel variable to match incoming data from the webUI
        self.cmd_vel = data

        # Track when this was last updated
        self.lastUpdate = time.time()

        # All new webUI data triggers the mode to be active
        self.trigger_active=True

############################################################################################################


    def __serviceCallbackUserInput(self,request, response):
        """Captures direct user input from service"""

        # This should reactivate the webUI mode
        self.lastUpdate = time.time()

        # Update the class variable
        self.__directUserInput=request.user_input
        self.__directUserInputSourceID=request.source

        # Each request has different and sometimes complex logic, handled by individual methods
        # Current returns only specify if a parameter was changed or not. This might be better if it includes more logic
        if self.__directUserInput == 0: # Test
            self.logger.info('Teleop WebUI: User requested test')
            returnString="Teleop WebUI: User requested test"
            returnBool=True

        elif self.__directUserInput == 8: # Release safety stop
            self.logger.info('Teleop WebUI: User requested to release safety stop')
            self.trigger_releaseForceStop=True
            returnString="Teleop WebUI: User requested to release safety stop"
            returnBool=True

        elif self.__directUserInput == 12: # Shutdown
            self.logger.info('Teleop WebUI: User requested shutdown')
            self.trigger_shutdown=True
            returnString="Teleop WebUI: User requested shutdown"
            returnBool=True

        else: # Unknown
            self.logger.info('Teleop WebUI: User request unknown')
            returnString="Teleop WebUI: User request unknown"
            returnBool=False

        # Unset the user input by reverting back to default
        self.__directUserInput == 0

        # Print and send a response to the client
        response.response_bool=returnBool
        response.response_string=returnString
        
        return response
        
############################################################################################################