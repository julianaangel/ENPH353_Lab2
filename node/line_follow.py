#!/usr/bin/env python
from __future__ import print_function
 
import roslib
roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
 
class image_converter:

  def __init__(self):
    self.move = Twist()
    self.move_pub = rospy.Publisher('/cmd_vel',Twist, queue_size = 3)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/rrbot/camera/image_raw",Image,self.callback)
 
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:      print(e)
 
    (rows,cols,channels) = cv_image.shape
    blur = cv2.GaussianBlur(cv_image,(7,7),3)

      #then greyscale since blue colour isn't very strong so doing it in colour isn't helpful
    grey = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

      #converted picture to binary: only want road in white because contours function identifies white objects
      #threshold value of 90 found through trial and error
    ret,thresh1 = cv2.threshold(grey,90,255,cv2.THRESH_BINARY_INV)

      #cropped image to bottom part of screen to locate centre of road closest to screen
    crop = thresh1[(rows-10):rows, 1:cols]

      #applying the contours function allows me to then find the centroid
    contours,hierarchy = cv2.findContours(crop, 1, 2)

      #if it doesn't find a distinct shape, maintain the centroid to what it was previously
    if len(contours) !=0:
      cnt = contours[0]
      M = cv2.moments(cnt)
       #add 1e-5 to avoid divide by zero (found this suggestion on the openCV documentation)
      cx = int(M['m10']/(M['m00']+ 1e-5))
      cy = int(M['m01']/(M['m00']+ 1e-5))	 
  #    if cx<cols/2:
  #	move.linear.x = 0.5
  #	move.angular.z = 0.5
  #   else:
  
    self.move.linear.x = 0.5
    self.move.angular.z = -0.5

    try:
      self.move_pub.publish(self.move)
    except CvBridgeError as e:
      print(e)
 
def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)

#for publishes do cmd_Vel, for substribe put correct camera path. instead of move = twist() just put PID
# import rospy
# from geometry_msgs.msg import Twist

# rospy.init_node('topic_publisher')
# pub = rospy.Publisher('/cmd_vel', Twist, 
#   queue_size=1)
# rate = rospy.Rate(2)
# move = Twist()
# move.linear.x = 0.5
# move.angular.z = 0.5

# while not rospy.is_shutdown():
#    pub.publish(move)
#    rate.sleep()