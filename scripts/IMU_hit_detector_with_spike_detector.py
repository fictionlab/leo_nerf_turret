#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from std_msgs.msg import Float32

from Spike_detector import Detector

THRESHOLD = 10

last_frame = None
jerk = 0

det_x = Detector(-THRESHOLD, THRESHOLD, 10)
det_y = Detector(-THRESHOLD, THRESHOLD, 10)


msg = Float32()
counter = 0 

    
def imu_callback(data):
    global jerk, last_frame, counter,msg
    counter += 1
    
    if last_frame == None:
        last_frame = data
        return
    
    jerk_x = data.linear_acceleration.x - last_frame.linear_acceleration.x
    jerk_y = data.linear_acceleration.y - last_frame.linear_acceleration.y
    
    hit_x = det_x.check(jerk_x)
    hit_y = det_y.check(jerk_y)

    if hit_x and hit_y:
        if (det_x.spike_up and det_x.spike_down) or (det_y.spike_up and det_y.spike_down):
            print("·    ",counter)
            return
        
        if det_x.spike_down and det_y.spike_up:
            print("↗    ",counter)
        elif det_x.spike_down and det_y.spike_down:
            print("↘    ",counter)
        elif det_x.spike_up and det_y.spike_up:
            print("↖    ",counter)
        elif det_x.spike_up and det_y.spike_down:
            print("↙    ",counter)
        
    elif hit_x:
        if (det_x.spike_up and det_x.spike_down):
            print("·    ",counter)
            return
        
        if det_x.spike_down:
            print("↑    ",counter)
        elif det_x.spike_up:
            print("↓    ",counter)

            
    elif hit_y:
        if (det_y.spike_up and det_y.spike_down):
            print("·    ",counter)
            return
        
        if det_y.spike_up:
            print("→    ",counter)
        elif det_y.spike_down:
            print("←    ",counter)

    last_frame = data


#ROS setup
rospy.init_node('IMU_listener', anonymous=True)
imu_sub = rospy.Subscriber("imu/data_raw",Imu, imu_callback)
hit_confirm = rospy.Publisher("chatter",Header,queue_size=10)
plotter = rospy.Publisher("plot_data",Float32,queue_size=10)


#msg parameters
msg = Header()
msg.seq = 0
msg.stamp = rospy.Time.now()
msg.frame_id = "IMU"


rospy.spin()

# ↖ ↑ ↗
# ← · →
# ↙ ↓ ↘