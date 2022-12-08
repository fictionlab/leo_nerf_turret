import rospy
import cv2
import mediapipe as mp
import time
from geometry_msgs.msg import Twist
rospy.init_node('nerf_turret_camera_node', anonymous=True)
command_publisher = rospy.Publisher("/nerf_turret/command",Twist,queue_size=1)

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

fps_limit = 200
startTime = time.time()

def draw_crosshair(image):
  (h,w,c) = image.shape
  image_with_crosshair = cv2.line(image,(int(w/2)-20, int(h/2)),(int(w/2 - 10), int(h/2)),(0,255,0),thickness=2)
  image_with_crosshair = cv2.line(image,(int(w/2)+20, int(h/2)),(int(w/2 + 10), int(h/2)),(0,255,0),thickness=2)
  image_with_crosshair = cv2.line(image,(int(w/2), int(h/2+20)),(int(w/2), int(h/2 + 10)),(0,255,0),thickness=2)
  return image_with_crosshair

def react_to_target(image, landmark):  
  (h,w,c) = image.shape
  (x,y,z,vis) = (landmark.x, landmark.y, landmark.z, landmark.visibility)
  if (vis < 0.5):
    return image
  
  #Command is basically telling dynamixels to move with X% of their max speed
  #in chosen direction
  command = Twist()
  command.angular.z =-(x*2 - 1)           #Pan
  command.angular.x = y*2 - 1             #Tilt
  command_publisher.publish(command)
  
  #Draws a line form the center of the screen to therever it thinks the tip of the nose is
  x = x*w
  y = y*h
  debug_image = cv2.line(image,(int(w/2),int(h/2)),(int(w/2),int(y)),(255,0,255),thickness=3)
  debug_image = cv2.line(image,(int(w/2),int(h/2)),(int(x),int(h/2)),(255,0,255),thickness=3)
  return debug_image



# "webcam" input:
cap = cv2.VideoCapture(0)
with mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
    model_complexity=0) as pose:
  while cap.isOpened():
    success, image = cap.read()
    if (time.time()- startTime) > 1/fps_limit:
      startTime = time.time()
      image = cv2.resize(image,(640,480))
      if not success:
        print("Ignoring empty camera frame.")
        # If loading a video, use 'break' instead of 'continue'.
        continue

      # To improve performance, optionally mark the image as not writeable to
      # pass by reference.
      image.flags.writeable = False
      image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
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
        image = react_to_target(image, results.pose_landmarks.landmark[0])
      # Flip the image horizontally for a selfie-view display.
      image = draw_crosshair(image)
      
      cv2.imshow('MediaPipe Pose', cv2.flip(image, 1))
      # print("---%s seconds since last frame", (time.time() - startTime))
    if cv2.waitKey(5) & 0xFF == ord('q'):
      break
cap.release()
