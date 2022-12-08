import rospy
import numpy as np

import cv2
from cv_bridge import CvBridge,CvBridgeError

from sensor_msgs.msg import Image,CompressedImage

#HOG human detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
# hog.setSVMDetector(cv2.HOGDescriptor_getDaimlerPeopleDetector())

cv2.startWindowThread()
cap = cv2.VideoCapture(0)
# cap = cv2.VideoCapture("people.mp4")
ret, frame = cap.read()
print(frame.shape[:2])


while(True):
    # reading the frame
    ret, frame = cap.read()
    frame = cv2.resize(frame,(320,240))
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #ret, frame = cv2.threshold(frame,80,255,cv2.THRESH_BINARY)

    boxes, weights = hog.detectMultiScale(frame, winStride=(8,8) )

    boxes = np.array([[x,y,x+w,y+h]for (x,y,w,h) in boxes])

    for (x1,y1,x2,y2) in boxes:
        cv2.rectangle(frame,(x1,y1),(x2,y2),(0,0,255), thickness=2)
    
    # displaying the frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        # breaking the loop if the user types q
        # note that the video window must be highlighted!
        break

cap.release()
cv2.destroyAllWindows()
# the following is necessary on the mac,
# maybe not on other platforms:
cv2.waitKey(1)