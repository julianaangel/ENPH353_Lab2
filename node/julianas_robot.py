#! /usr/bin/env python3

##This script is used subscribe to the topic /rrbot/camera1/image_raw, modify the input images and then publish a movement control Twist message to the topic /cmd_vel.

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/rrbot/camera1/image_raw',Image,self.callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
        (rows,cols,channels) = cv_image.shape
        blur = cv2.GaussianBlur(cv_image,(7,7),3)
        grey = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        ret,thresh1 = cv2.threshold(grey,90,255,cv2.THRESH_BINARY_INV)
        crop = thresh1[(rows-5):rows, 1:cols]
        contours,hierarchy = cv2.findContours(crop, 1, 2)
        if len(contours) !=0:
          cnt = contours[0]
          M = cv2.moments(cnt)
          #add 1e-5 to avoid divide by zero (found this suggestion on the openCV documentation)
          cx = int(M['m10']/(M['m00']+ 1e-5))
          if cx<(cols/2-cols/4):
                self.twist.linear.x = 0.05
                self.twist.angular.z = 10
          elif cx>(cols/2+cols/4):
                self.twist.linear.x= 0.05
                self.twist.angular.z = -10
          elif cx<(cols/2-cols/6):
                self.twist.linear.x= 0.1
                self.twist.angular.z = 8
          elif cx>(cols/2+cols/6):
                self.twist.linear.x= 0.1
                self.twist.angular.z = -8
          elif cx<(cols/2-cols/8):
                self.twist.linear.x= 0.2
                self.twist.angular.z = 6
          elif cx>(cols/2+cols/8):
                self.twist.linear.x= 0.2
                self.twist.angular.z = -6
          elif cx<(cols/2-cols/10):
                self.twist.linear.x= 0.3
                self.twist.angular.z = 4
          elif cx>(cols/2+cols/10):
                self.twist.linear.x= 0.3
                self.twist.angular.z = -4
          else:
                self.twist.linear.x=0.4
                self.twist.angular.z=0

            
        # ##Code to publish the Twist message
        try:
            self.cmd_vel_pub.publish(self.twist)
        except CvBridgeError as e:
            print(e)

def main():
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    rospy.Subscriber("rrbot/camera1/image_raw",Image,ic.callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()