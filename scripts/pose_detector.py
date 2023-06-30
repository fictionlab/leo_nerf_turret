import rospy
import cv2
import mediapipe as mp
import time
from geometry_msgs.msg import Twist

# ROS setup
rospy.init_node('nerf_turret_camera_node', anonymous=True)
command_publisher = rospy.Publisher("/nerf_turret/command",Twist,queue_size=1)


# Mediapipe setup
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

# "webcam" input:
cap = cv2.VideoCapture(0)
# Using this command you can convert any video stream into a fake "usb camera". Which is cool and for some reason makes the whole thing work much faster
# than with any other solution I've tested
# ffmpeg -i http://ip_address:port/video -vf format=yuv420p -f v4l2 /dev/video0
# for me: ffmpeg -i http://10.0.0.1:8080/stream?topic=/usb_cam/image_raw -vf format=yuv420p -f v4l2 /dev/video0


# Limiting max possible FPS
fps_limit = 200
startTime = time.time()

def draw_crosshair(image):
  """ Takes cv2 image and returns the same one with CS 1.6 style crosshair in the middle.
  
  Parameters
  ----------
  image: cv2 compatybile image format
  
  Returns
  -------
  image: image with crosshair applied
  """
  
  (h,w,c) = image.shape
  image_with_crosshair = cv2.line(image,(int(w/2)-20, int(h/2)),(int(w/2 - 10), int(h/2)),(0,255,0),thickness=2)
  image_with_crosshair = cv2.line(image,(int(w/2)+20, int(h/2)),(int(w/2 + 10), int(h/2)),(0,255,0),thickness=2)
  image_with_crosshair = cv2.line(image,(int(w/2), int(h/2+20)),(int(w/2), int(h/2 + 10)),(0,255,0),thickness=2)
  return image_with_crosshair

def react_to_target(image, landmark):  
  
  """ Processes landmark position and sends Twist msg to be used by dynamixels

  Parameters
  ----------
  image: cv2 compatybile image format
  landmark: Mediapipe landmark
  
  Returns
  ----------
  image: cv2 image with pink lines showing landmark position 
  """
  (h,w,c) = image.shape
  (x,y,z,vis) = (landmark.x, landmark.y, landmark.z, landmark.visibility)
  if (vis < 0.5):
    return image
  
  # Sending msg to move dynamixels
  command = Twist()
  command.angular.z =-(x*2 - 1)           #Pan
  command.angular.x = y*2 - 1             #Tilt
  command_publisher.publish(command)
  
  #Draws a line form the center of the screen to the landmark position
  x = x*w
  y = y*h
  debug_image = cv2.line(image,(int(w/2),int(h/2)),(int(x),int(y)),(255,0,255),thickness=3)
  # debug_image = cv2.line(image,(int(w/2),int(h/2)),(int(x),int(h/2)),(255,0,255),thickness=3)
  return debug_image

# The main function of the code reads frames from the webcam, processes them using the MediaPipe pose model
# and publishes Twist commands to control the Nerf turret. It limits the maximum FPS and draws a crosshair on the center of the screen.
# Exits when the user presses the 'q' key or 'esc'

with mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
    model_complexity=0) as pose:
  while cap.isOpened():
    if (time.time()- startTime) > 1/fps_limit:
      
      # getting video frame
      success, image = cap.read()
      startTime = time.time()
      image = cv2.resize(image,(640,480))
      if not success:
        print("Ignoring empty camera frame.")
        # If loading a video, use 'break' instead of 'continue'.
        continue
      
      # boosting processing performance
      image.flags.writeable = False
      image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
      # processing the frame
      results = pose.process(image)

      # Draw the pose annotation on the image.
      image.flags.writeable = True
      image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
      mp_drawing.draw_landmarks(
          image,
          results.pose_landmarks,
          mp_pose.POSE_CONNECTIONS,
          landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
      if results.pose_landmarks:
        # If a human has been detected: react
        image = react_to_target(image, results.pose_landmarks.landmark[0])
        
        
      # Adding a crosshair to the image
      image = draw_crosshair(image)
      
      # Showing the view from camera with crosshair and debug lines
      cv2.imshow('MediaPipe Pose', cv2.flip(image, 1))
      
    key = cv2.waitKey(5)
    if key == ord('q') or key == 27:
      break
    
cap.release()
cv2.destroyAllWindows()

