#!/usr/bin/env python3

import numpy as np
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt

import rospy
from std_msgs.msg import Header


# Define filter parameters
fs = 44100  # Sample rate, Hz
cutoff = 1000  # Cutoff frequency, Hz
nyq = 0.5 * fs
order = 5
CHUNK = 1024

# Create a high-pass Butterworth filter
b, a = butter(order, cutoff/nyq, btype='highpass')

# Set up a live data stream
# For example, you could use the pyaudio library to read from a microphone
import pyaudio

p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paFloat32,
                channels=1,
                rate=int(fs),
                input=True)


max_value = 0

#ROS setup
rospy.init_node('piezo_listener', anonymous=True)
hit_confirm = rospy.Publisher("chatter",Header,queue_size=10)
#msg parameters
msg = Header()
msg.seq = 0
msg.stamp = rospy.Time.now()
msg.frame_id = "PIEZO"



# Process live data
while not rospy.is_shutdown():
    # Read a chunk of data from the stream
    data = np.fromstring(stream.read(CHUNK), dtype=np.float32)
    
    # Apply the filter to the data
    filtered = filtfilt(b, a, data)
    
    # Do something with the filtered data
    # For example, you could send it to another process, write it to a file, etc.
    
    max_value = max(data)
    if max_value > 0.8:
        print(max_value)
        msg.stamp = rospy.Time.now()
        msg.seq += 1
        hit_confirm.publish(msg)