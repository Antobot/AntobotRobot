### antobot_robot_monitor

This packages monitors various components of the robot platform which are not related to uRCU system monitoring.

At the moment, this is only robotMonitor, which watches over the movement of the robot, and reports important information such as whether the robot has been stuck (i.e. a command is being sent, but the robot has stopped moving), if the robot is angled too much in pitch or roll.
