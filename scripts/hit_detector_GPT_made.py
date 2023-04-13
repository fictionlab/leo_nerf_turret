#!/usr/bin/env python3

import math
import time
import numpy as np
import pyaudio
import board
import busio
import adafruit_lsm6ds.lsm6ds33

# Initialize IMU
i2c = busio.I2C(board.SCL, board.SDA)
imu = adafruit_lsm6ds.lsm6ds33.LSM6DS33(i2c)

# Initialize microphone
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
p = pyaudio.PyAudio()
stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

# Initialize hit detector
THRESHOLD = 5000  # adjust this to your needs
WINDOW_SIZE = 10  # number of previous samples to use for smoothing
MAGNITUDE_THRESHOLD = 10  # minimum magnitude of acceleration to be considered a hit
history = [0] * WINDOW_SIZE
hit_detected = False

while True:
    # Read data from IMU
    acceleration = imu.acceleration
    magnitude = math.sqrt(sum(a**2 for a in acceleration))
    
    # Smooth acceleration data
    history.pop(0)
    history.append(magnitude)
    smoothed = sum(history) / WINDOW_SIZE
    
    # Check for hit
    if smoothed > MAGNITUDE_THRESHOLD and not hit_detected:
        # Read data from microphone
        data = stream.read(CHUNK)
        amplitude = np.frombuffer(data, dtype=np.int16).max()
        
        # Check if sound was loud enough
        if amplitude > THRESHOLD:
            # Hit detected!
            print("Hit detected!")
            hit_detected = True
            
    if hit_detected:
        # Wait a moment before resetting hit detector
        time.sleep(0.5)
        hit_detected = False
