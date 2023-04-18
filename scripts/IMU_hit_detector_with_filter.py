#!/usr/bin/env python3

import math
import rospy
import time
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

import numpy as np
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt

from collections import deque


# Filter buffer length
BUFFER = 50
# Hit detection threshold value
THRESHOLD = 15
# filter and jerk counter
counter = 0
counter_max = 10

# Define filter parameters
fs = 100 # Sample rate, Hz
cutoff = 30  # Cutoff frequency, Hz
nyq = 0.5 * fs
order = 5

# Create a high-pass Butterworth filter
b, a = butter(order, cutoff/nyq, btype='highpass')

accel = { 'x' : deque([],BUFFER),
          'y' : deque([],BUFFER),
          'z' : deque([],BUFFER)}

accel_filtered = {  'x' : deque([],BUFFER),
                    'y' : deque([],BUFFER),
                    'z' : deque([],BUFFER)}

jerk = deque([],counter_max)

def update_buffer(imu_msg):
    accel['x'].append(imu_msg.linear_acceleration.x)
    
    accel['y'].append(imu_msg.linear_acceleration.y)
    
    accel['z'].append(imu_msg.linear_acceleration.z)
    
def filter_data():
    accel_filtered['x'] = filtfilt(b, a, accel['x'])
    accel_filtered['y'] = filtfilt(b, a, accel['y'])
    accel_filtered['z'] = filtfilt(b, a, accel['z'])
    
def update_jerk():
    
    first = True
    if len(accel['x']) < 3:
        return
    
    temp = [0,0,0]
    temp_value = 0
    
    for i in range (12):
        x = accel['x'][-(i+1)] - accel['x'][-i]
        y = accel['y'][-(i+1)] - accel['y'][-i]
        z = accel['z'][-(i+1)] - accel['z'][-i]
        jerk_value = math.sqrt((x)**2 + (y)**2 + (z)**2)
        jerk.append(jerk_value)
        
        if jerk_value > THRESHOLD:
            temp = [x,y,z]
            temp_value = jerk_value
            
            msg.stamp = rospy.Time.now()
            msg.seq += 1
            hit_confirm.publish(msg)
            
    if max(temp) > 0:
        print("jerk value: ", temp_value)
        print("jerk (x,y,z): ", temp)

def imu_callback(data):
    global counter
    update_buffer(data)
    counter += 1
    
    if counter > counter_max:
        filter_data()                               # THIS IS REALLY BAD - I SHOULD BE USING SOME LIVE FILTERING SYSTEM, AND IT DOESN'T. JUST LOOKS AT ALL THE GATHERED DATA EVERY TIME
        update_jerk()
        counter = 1


#ROS setup
rospy.init_node('IMU_listener', anonymous=True)
# imu_sub = rospy.Subscriber("imu/data_raw",Imu, imu_callback)
imu_sub = rospy.Subscriber("imu/data",Imu, imu_callback)
hit_confirm = rospy.Publisher("chatter",Header,queue_size=10)
#msg parameters
msg = Header()
msg.seq = 0
msg.stamp = rospy.Time.now()
msg.frame_id = "IMU"


rospy.spin()