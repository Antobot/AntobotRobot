#!/usr/bin/env python3

# Copyright (c) 2024, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:     moveMonitor watches over the movement of the robot, and reports important information such as
#                           whether the robot has been stuck (i.e. a command is being sent, but the robot has stopped moving),
#                           if the robot is angled too much in pitch or roll.

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import time
import os
from shutil import copyfile
from datetime import date
import yaml
from pathlib import Path

from math import radians, cos, sin, asin, sqrt, isnan, pi, atan2
import numpy as np

import rospy
import rospkg

import tf
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32, Bool, UInt8
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist



class robotMonitor():

    def __init__(self):

        
                
        self.As_lat_past = []
        self.As_lon_past = []
        self.As_lat = None
        self.As_lon = None
        self.b_robot_movement = False # default robot is not move

        self.As_b_vel_cmd = False
        self.As_bGNSS = False

        cmdVel = Twist()
        self.cmdVel_linear = cmdVel.linear
        self.cmdVel_angular = cmdVel.angular
        self.cmdVel_time = time.time()
        self.cmdVel_linear_buffer = []
        self.cmdVel_angular_buffer = []
        self.cmdVel_spotTurn_consistency = False
        self.cmdVel_straight_consistency = False

        
        self.stuck_spotTurn = False
        self.stuck_spotTurn_t = time.time()
        self.stuck_spotTurn_f = False
        self.stuck_spotTurn_lvl = 0
        self.robot_yaw5  = 0.0
        self.stuck_straightMove = False
        self.stuck_straightMove_t = time.time()
        self.stuck_straightMove_f = False
        self.stuck_straightMove_lvl = 0
        self.robot_movement_dist5 = 0.0
        self.turn = False
        self.yaw_past = []

        self.pitch_lvl = 0
        self.roll_lvl = 0

        self.imu_freq = 20 # how many imu msgs in 1 second
        self.imu_buffer_Hz = 10        # Hz to check the imu orientation
        self.imu_buffer = []
        self.imu_buffer_len = int(self.imu_freq/self.imu_buffer_Hz)
        self.imu_update_time = time.time()

        self.pitch_thresh_low = 10*pi/180
        self.pitch_thresh_med = 15*pi/180
        self.pitch_thresh_high = 20*pi/180

        self.roll_thresh_low = 10*pi/180
        self.roll_thresh_med = 15*pi/180
        self.roll_thresh_high = 20*pi/180


        self.sim = rospy.get_param("/simulation",False)
        if self.sim:
            self.spotTurn_oscillation_amplitude = rospy.get_param("/antobot_move_path_follow/main_allocate/sim_oscillation_amplitude",0.3)
        else:
            self.spotTurn_oscillation_amplitude = rospy.get_param("/antobot_move_path_follow/main_allocate/oscillation_amplitude",0.3)

        self.sub_GPS_data = rospy.Subscriber("/antobot_gps",NavSatFix,self.GPS_callback)        # Should raw GPS data be used? What if it stops coming?
        self.sub_IMU = rospy.Subscriber('/imu/data_corrected', Imu, self.imu_callback)          # Subscriber for imu data corrected
        self.sub_cmdVel = rospy.Subscriber('/antobot_robot/cmd_vel', Twist, self.cmdVel_callback)    # Subscriber for cmd velocity

        # Publish safety critical data
        self.pub_error_lv1_stk = rospy.Publisher("/as/error_lv1_stk",Bool,queue_size =1)
        self.pub_error_lv1_tilt = rospy.Publisher("/as/error_lv1_tilt",Bool,queue_size =1)
        self.pub_robot_stuck = rospy.Publisher("/as/robot_stuck", UInt8, queue_size = 1)
        
        # Publish robot movement data
        self.pub_robot_dist5 = rospy.Publisher("/robotMonitor/robot_dist_5", Float32, queue_size = 1)

        self.tiltTimer = rospy.Timer(rospy.Duration(1.0/self.imu_buffer_Hz), self.check_imu)



    def cmdVel_consistency_check(self):
        # Just do a simple average check to see if it was consistently spot turning or moving forward
        if len(self.cmdVel_linear_buffer) == 5: # when the list is full
            if all(num > 0.1 for num in self.cmdVel_linear_buffer) or all(num < -0.1 for num in self.cmdVel_linear_buffer):
                self.cmdVel_straight_consistency = True
            else: 
                self.cmdVel_straight_consistency = False

            if all(num > 0.1 for num in self.cmdVel_angular_buffer) or all(num < -0.1 for num in self.cmdVel_angular_buffer):
                self.cmdVel_spotTurn_consistency = True
            else:
                self.cmdVel_spotTurn_consistency = False



    def robot_stuck(self,event=None): 
        # # # Determines whether the robot is currently stuck (i.e. commands are being sent, but the robot isn't moving)
        
        self.cmdVel_consistency_check()

        # Stuck during spot turn (indicated by IMU)
        if abs(self.cmdVel_angular.z) > 0.1 and abs(self.cmdVel_linear.x) <= self.spotTurn_oscillation_amplitude:    # spot turn command
            self.robot_movement_distance()
            if self.cmdVel_spotTurn_consistency and abs(self.robot_yaw5) < 0.05 and self.robot_movement_dist5 < 1:    # if robot hasn't moved 3 degs or more in the last 5 seconds and less than 1m movedment in gps position
                if self.stuck_spotTurn is not True:
                    self.stuck_spotTurn_t = time.time() # will report 5 secons later as it checks for consistency
                    self.stuck_spotTurn_f = True
                self.stuck_spotTurn = True
            else:
                self.stuck_spotTurn = False
        else:
            self.stuck_spotTurn = False

        # Stuck while moving straight(-ish)
        if abs(self.cmdVel_linear.x) > 0.1: # and self.As_bGNSS == True: #when GNSS is good, check robot location every 5 seconds - removed this requirement - robot can still get stuck when GPS is bad, moved inside milage tracker
            self.robot_movement_distance()
            if self.cmdVel_straight_consistency and self.robot_movement_dist5 < 0.1: # If the robot has not moved more than 0.4m in the last 5 seconds
                if self.stuck_straightMove is not True:
                    self.stuck_straightMove_t = time.time() # will report 5 secons later as it checks for consistency
                    self.stuck_straightMove_f = True
                self.stuck_straightMove = True
            else:
                self.stuck_straightMove = False
        else:
            self.stuck_straightMove = False


        if not self.stuck_spotTurn and not self.stuck_straightMove:
            self.pub_robot_stuck.publish(0)
        elif self.stuck_straightMove:
            self.pub_robot_stuck.publish(1)
        elif self.stuck_spotTurn:
            self.pub_robot_stuck.publish(2)


        self.error_lv_report()

        # Report if either form of stuck is occuring
        if ((self.stuck_straightMove_lvl>1) or (self.stuck_spotTurn_lvl>1)):
            self.pub_error_lv1_stk.publish(True)
        else:
            self.pub_error_lv1_stk.publish(False)
            
        # Publish robot_movement_dist5
        self.pub_robot_dist5.publish(self.robot_movement_dist5)




    def error_lv_report(self):
        # # # Reports how "stuck" the robot is, depending on how long it has been stuck (in seconds)
        
        self.stuck_lvl_straightMove()
        self.stuck_lvl_spotTurn()

        return

    def stuck_lvl_straightMove(self):
        # # # Determines how stuck the robot is while attempting to move straight and logs it to /rosout

        # Determining how stuck the robot is
        stuck_straightMove_lvl_i = 0
        if self.stuck_straightMove:
            if not self.stuck_straightMove_f:   # Check for flag saying the robot has just become stuck
                stuck_time = time.time() - self.stuck_straightMove_t
                if stuck_time > 30:
                    stuck_straightMove_lvl_i = 3
                elif stuck_time > 15:
                    stuck_straightMove_lvl_i = 2
                elif stuck_time > 5:
                    stuck_straightMove_lvl_i = 1

        # Logging
        if stuck_straightMove_lvl_i != self.stuck_straightMove_lvl:
            if stuck_straightMove_lvl_i == 3:
                rospy.logerr("MV0100: Robot has been stuck for 30 seconds (straight move)")
            elif stuck_straightMove_lvl_i == 2:
                rospy.logwarn("MV0100: Robot has been stuck for 15 seconds (straight move)")
            elif stuck_straightMove_lvl_i == 1:
                rospy.logwarn("MV0100: Robot has been stuck for 5 seconds (straight move)")
            elif stuck_straightMove_lvl_i == 0:
                rospy.loginfo("MV0100: Robot no longer stuck (straight move)")
            self.stuck_straightMove_lvl = stuck_straightMove_lvl_i

        self.stuck_straightMove_f = False   # Reset the flag

    
    def stuck_lvl_spotTurn(self):
        # # # Determines how stuck the robot is while attempting to spot turn and logs it to /rosout

        # Determining how stuck the robot is
        stuck_spotTurn_lvl_i = 0
        if self.stuck_spotTurn:
            if not self.stuck_spotTurn_f:   # Check for flag saying the robot has just become stuck
                stuck_time = time.time() - self.stuck_spotTurn_t
                if stuck_time > 30:
                    stuck_spotTurn_lvl_i = 3
                elif stuck_time > 15:
                    stuck_spotTurn_lvl_i = 2
                elif stuck_time > 5:
                    stuck_spotTurn_lvl_i = 1
        else:
            self.stuck_spotTurn_lvl = 0
                
        # Logging
        if stuck_spotTurn_lvl_i != self.stuck_spotTurn_lvl:
            if stuck_spotTurn_lvl_i == 3:
                rospy.logerr("MV0101: Robot has been stuck for 30 seconds (spot turn)")
            elif stuck_spotTurn_lvl_i == 2:
                rospy.logwarn("MV0101: Robot has been stuck for 15 seconds (spot turn)")
            elif stuck_spotTurn_lvl_i == 1:
                rospy.logwarn("MV0101: Robot has been stuck for 5 seconds (spot turn)")
            elif stuck_spotTurn_lvl_i == 0:
                rospy.loginfo("MV0101: Robot no longer stuck (spot turn)")
            self.stuck_spotTurn_lvl = stuck_spotTurn_lvl_i
            
        self.stuck_spotTurn_f = False   # Reset the flag

    def robot_movement_distance_reset(self):
        self.robot_movement_dist5=0.0
        self.As_lon_past = []
        self.As_lat_past = []
        # initialise stuck timer
        self.stuck_straightMove_t = time.time()
        self.stuck_spotTurn_t = time.time()

    def robot_movement_distance(self):
        # # # Calculates how long the robot has moved since the tracker was last cleared
        # Inputs: current and past latitudes and longitudes
        # Returns: total robot movement since last check 

        # If there is historic data to work with
        if len(self.As_lat_past)==5:
            # Compare against the oldest reading (5 secs old) to check if the robot is stuck
            self.robot_movement_dist5 = self.haversine(self.As_lat, self.As_lon, self.As_lat_past[0], self.As_lon_past[0])

        else: # Assume no motion at the start
            self.robot_movement_dist5=0.0

        if self.As_lat is not None:
            # Add the new reading to the list
            self.As_lat_past.append(self.As_lat)
            self.As_lon_past.append(self.As_lon)

        # Remove the oldest value in the log
        if len(self.As_lat_past)>5:
            self.As_lat_past.pop(0)
            self.As_lon_past.pop(0)




    def haversine(self,lat1, lon1, lat2, lon2):
        # # # Calculates the distance in meters between two lat/lon pairs
        # Inputs: lat1 - current latitude; lon1 - current longitude; lat2 - previous latitude; lon2 - previous longitude 

        R = 6372.8*1000 #  Earth radius in meters 

        dLat = radians(lat2 - lat1)
        dLon = radians(lon2 - lon1)
        lat1 = radians(lat1)
        lat2 = radians(lat2)

        a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
        c = 2*asin(sqrt(a))

        return R * c

    def check_imu(self, event = None):
        # # # Checks IMU data to determine whether the robot is rolling, pitching, or turning in a yaw direction
        if len(self.imu_buffer) != 0:
            # Compute the mean of each angle within a buffer
            angles_arr = np.array(self.imu_buffer) 
            angles_avg = np.mean(angles_arr, axis=0) # You shouldn't compute mean angles like this - it doesn't account for moving over 0

            # Performs individual actions for each calculated angle
            try:
                self.roll_angle(angles_avg[0])
                self.pitch_angle(angles_avg[1])
                self.yaw_angle(angles_avg[2])
            except IndexError:
                return
        
        # Report if the robot is tilting too much
        if (abs(self.roll_lvl) == 3) or (abs(self.pitch_lvl)==3):
            self.pub_error_lv1_tilt.publish(True)
        else:
            self.pub_error_lv1_tilt.publish(False)
        

    def pitch_angle(self, pitch):
        # # # Calculates whether the robot is pitching (forward or backward) and to what degree, then logs this information to /rosout
        # Input: angles <Vector3> - [roll, pitch, yaw]

        pitch_lvl_i = 0

        if pitch < 0:       # Robot front is facing up a hill (backward pitch)
            if abs(pitch) > self.pitch_thresh_high:
                pitch_lvl_i = -3
            elif abs(pitch) > self.pitch_thresh_med:
                pitch_lvl_i = -2
            elif abs(pitch) > self.pitch_thresh_low:
                pitch_lvl_i = -1
                
        elif pitch > 0:     # Robot front is facing down a hill (forward pitch)
            if abs(pitch) > self.pitch_thresh_high:
                pitch_lvl_i = 3
            elif abs(pitch) > self.pitch_thresh_med:
                pitch_lvl_i = 2
            elif abs(pitch) > self.pitch_thresh_low:
                pitch_lvl_i = 1
                
        # Logging
        if pitch_lvl_i != self.pitch_lvl:
            if pitch_lvl_i == 3:
                rospy.logerr("SN5001: Robot is pitching too far forward!")
            elif pitch_lvl_i == 2:
                rospy.logwarn("SN5001: Robot is pitching forward")
            elif pitch_lvl_i == 1:
                rospy.loginfo("SN5001: Robot is pitching slightly forward")
            elif pitch_lvl_i == 0:
                rospy.loginfo("SN5000: Robot is no longer pitching!")
            elif pitch_lvl_i == -1:
                rospy.loginfo("SN5000: Robot is pitching slightly backward")
            elif pitch_lvl_i == -2:
                rospy.logwarn("SN5000: Robot is pitching backward")
            elif pitch_lvl_i == -3:
                rospy.logerr("SN5000: Robot is pitching too far backward!")
            self.pitch_lvl = pitch_lvl_i

    def roll_angle(self, roll):
        # # # Calculates whether the robot is rolling (left or right) and to what degree, then logs this information to /rosout
        # Input: angles <Vector3> - [roll, pitch, yaw]


        roll_lvl_i = 0
        
        if roll < 0:       # Robot top is rolling left
            if abs(roll) > self.roll_thresh_high:
                roll_lvl_i = -3
            elif abs(roll) > self.roll_thresh_med:
                roll_lvl_i = -2
            elif abs(roll) > self.roll_thresh_low:
                roll_lvl_i = -1
                
        elif roll > 0:     # Robot top is rolling right
            if abs(roll) > self.roll_thresh_high:
                roll_lvl_i = 3
            elif abs(roll) > self.roll_thresh_med:
                roll_lvl_i = 2
            elif abs(roll) > self.roll_thresh_low:
                roll_lvl_i = 1

        # Logging
        if roll_lvl_i != self.roll_lvl:
            if roll_lvl_i == 3:
                rospy.logerr("SN5011: Robot is rolling too far right!")
            elif roll_lvl_i == 2:
                rospy.logwarn("SN5011: Robot is rolling right")
            elif roll_lvl_i == 1:
                rospy.loginfo("SN5011: Robot is rolling slightly right")
            elif roll_lvl_i == 0:
                rospy.loginfo("SN5010: Robot is no longer rolling!")
            elif roll_lvl_i == -1:
                rospy.loginfo("SN5010: Robot is rolling slightly left")
            elif roll_lvl_i == -2:
                rospy.logwarn("SN5010: Robot is rolling left")
            elif roll_lvl_i == -3:
                rospy.logerr("SN5010: Robot is rolling too far left!")
            self.roll_lvl = roll_lvl_i


    def yaw_angle(self, yaw):
        # # # Compares current yaw angle to previous, determining whether the robot is turning or not
        # Input: angles <Vector3> - [roll, pitch, yaw]

        # If there is historic data to work with
        if len(self.yaw_past)==5:

            # Compare against the oldest reading (5 secs old) to check if the robot is stuck
            rad_diff = yaw-self.yaw_past[0]
            self.robot_yaw5 = atan2(sin(rad_diff), cos(rad_diff))
            #self.robot_yaw5 = abs(yaw - self.yaw_past[0])

        else: # Assume no motion at the start
            self.robot_yaw5=0.1 # reset to a value that won't trigger stuck detection
            
        if time.time() - self.imu_update_time > 1.0:
            
            # Add the new reading to the list
            self.yaw_past.append(yaw)
            self.imu_update_time = time.time() # reset imu update time


        # Remove the oldest value in the log
        if len(self.yaw_past)>5:
            self.yaw_past.pop(0)
    
    def GPS_callback(self,gps_msg):
        # gps callback function, if gps status is 3 then it's in fix mode
       
        self.As_lat = gps_msg.latitude
        self.As_lon = gps_msg.longitude
       
        if  gps_msg.status.status == 3:
            self.As_bGNSS = True
        else:
            self.As_bGNSS = False

    def imu_callback(self,data):
        # # # Callback function for IMU data after it is calibrated
        # Input: data <Imu.msg>
        
        roll, pitch, yaw = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]) # Use ROS built in for speed

        angles=[roll, pitch, yaw]


        if len(self.imu_buffer) < self.imu_buffer_len:
            self.imu_buffer.append(angles)
        else:
            self.imu_buffer.pop(0)
            self.imu_buffer.append(angles)

    def cmdVel_callback(self, data):
        # # # Callback function for cmd_vel data of the robot
        # Input: data <Twist.msg>
        self.cmdVel_linear = data.linear
        self.cmdVel_angular = data.angular
        
        if self.robot_movement_dist5 != 0 and abs(self.cmdVel_angular.z) < 0.1 and abs(self.cmdVel_linear.x)<0.1:
            self.robot_movement_distance_reset()

        # Update the buffer every second
        if (time.time()- self.cmdVel_time) > 1:
            if len(self.cmdVel_linear_buffer) > 4:
                self.cmdVel_linear_buffer.pop(0)
                self.cmdVel_angular_buffer.pop(0)
            self.cmdVel_linear_buffer.append(self.cmdVel_linear.x)
            self.cmdVel_angular_buffer.append(self.cmdVel_angular.z)
            self.cmdVel_time = time.time()
            

def main():
    rospy.init_node ('robotMonitor') 
    moveMgr = robotMonitor()
    rospy.Timer(rospy.Duration(1), moveMgr.robot_stuck)  # Runs periodically without blocking
    rospy.spin() 


if __name__ == '__main__':
    main()
