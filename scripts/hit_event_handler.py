#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header

rospy.init_node('HIT_message_listener', anonymous=True)
last_IMU_predicted_hit = rospy.Time.now() - rospy.Duration(100)
last_PIEZO_predicted_hit = rospy.Time.now()

msg = Header()
msg.frame_id

def check_hit_event():
    if abs(last_IMU_predicted_hit - last_PIEZO_predicted_hit) < rospy.Duration(0,500000000):
        print("HIT")
        hit_confirm.publish(msg)


def msg_callback(data):
    global last_PIEZO_predicted_hit, last_IMU_predicted_hit
    if data.frame_id == "IMU":
        last_IMU_predicted_hit = data.stamp
        pass
    elif data.frame_id == "PIEZO":
        last_PIEZO_predicted_hit = data.stamp
        
    check_hit_event()
        
    

#ROS setup

imu_sub = rospy.Subscriber("chatter",Header, msg_callback)

hit_confirm = rospy.Publisher("chatter2",Header,queue_size=10)
#msg parameters
msg = Header()
msg.seq = 0
msg.stamp = rospy.Time.now()
msg.frame_id = "HIT"


rospy.spin()