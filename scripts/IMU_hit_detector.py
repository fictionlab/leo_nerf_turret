#!/usr/bin/env python3

import math
import rospy
from sensor_msgs.msg import Imu

THRESHOLD = 15

last_data = None

jerk = {'x' : 0,
        'y' : 0,
        'z' : 0}

j_mag = 0

def jerk_mag(jerk):
    global j_mag
    j_mag = math.sqrt(jerk['x']**2 + jerk['y']**2 + jerk['z']**2)
    return j_mag

def imu_callback(data):
    global last_data
    if last_data == None:
        last_data = data
    else:
        jerk['x'] = data.linear_acceleration.x - last_data.linear_acceleration.x
        jerk['y'] = data.linear_acceleration.y - last_data.linear_acceleration.y
        jerk['z'] = data.linear_acceleration.z - last_data.linear_acceleration.z
        last_data = data
        
        jerk_mag(jerk)
        
        if(j_mag > THRESHOLD):
            print(j_mag)
    pass


rospy.init_node('IMU_listener', anonymous=True)
imu_sub = rospy.Subscriber("imu/data_raw",Imu, imu_callback)
rospy.spin()
    