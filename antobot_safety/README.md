# antobot_safety

This package contains a variety of code related to the safe operation of our robot. 

This includes basic safety functions for navigation (e.g. stopping if an object is detected to be too close by the ultrasonic sensors), blinking
the lights when appropriate to let users know that a safety function has been triggered. 
It also includes a multiplexer based on code from https://github.com/yujinrobot/yujin_ocs/tree/devel/yocs_cmd_vel_mux. Their original license has been included in this package.

Different safety levels can be configured using the /antobot/safety/safetyLevel service:
- ???: asdfs
- ???: asdf
