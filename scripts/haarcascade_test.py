#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import pathlib

# A list of haar cascades i can choose from
# haarcascade_eye_tree_eyeglasses.xml      haarcascade_licence_plate_rus_16stages.xml
# haarcascade_eye.xml             4/10     haarcascade_lowerbody.xml 2/10
# haarcascade_frontalcatface_extended.xml  haarcascade_profileface.xml
# haarcascade_frontalcatface.xml           haarcascade_righteye_2splits.xml 8/10
# haarcascade_frontalface_alt2.xml    6/10 haarcascade_russian_plate_number.xml
# haarcascade_frontalface_alt_tree.xml 2/10 haarcascade_smile.xml 1/10
# haarcascade_frontalface_alt.xml     5/10 haarcascade_upperbody.xml 2/10
# haarcascade_frontalface_default.xml 6/10 __init__.py
# haarcascade_fullbody.xml  3/10           __pycache__
# haarcascade_lefteye_2splits.xml 9/10

#setup of the image classifier
cascade_path = pathlib.Path(cv2.__file__).parent.absolute() / "data/haarcascade_smile.xml"
# print(cascade_path)
classifier = cv2.CascadeClassifier(str(cascade_path))

# classifier = cv2.CascadeClassifier('haarcascade_fullbody.xml')

cv2.startWindowThread()
# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture("people.mp4")
ret, frame = cap.read()
print(frame.shape[:2])

while(True):
    # reading the frame
    ret, frame = cap.read()
    frame = cv2.resize(frame,(640,480))
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # frame = cv2.blur(frame,(5,5))
    #ret, frame = cv2.threshold(frame,80,255,cv2.THRESH_BINARY)

    boxes= classifier.detectMultiScale(frame, 1.9,1)

    boxes= np.array([[x,y,x+w,y+h]for (x,y,w,h) in boxes])

    # print(weights)

    for (x1,y1,x2,y2) in boxes:
        cv2.rectangle(frame,(x1,y1),(x2,y2),(255,255,255), thickness=2)
    
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
    