#! /usr/bin/env python3
## @package enph353_ros_lab
#  This script is used subscribe to the topic /rrbot/camera1/image_raw, modify the input images and then publish a movement control Twist message to the topic /cmd_vel.

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class robot_mover:

      ## The constructor creates a bridge to convert image to cv object, 
      # an image subscriber to take in camera input as the robot drives,
      # a Twist publisher that enables the robot to respond with movement
      # and a Twist object that I will modify based on how far off the road the robot is
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/rrbot/camera1/image_raw',Image,self.callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.prevx = 400

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
        ## getting the number of rows and columns as variables to not have magic numbers
        (rows,cols,channels) = cv_image.shape
        ## first blur the image
        blur = cv2.GaussianBlur(cv_image,(7,7),3)
        ## then grey the image
        grey = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        ## now invert the image colors (road should be white)
        ret,thresh1 = cv2.threshold(grey,200,255,cv2.THRESH_BINARY_INV)
        ## crop the image so that it's only the last 5 rows
        crop = thresh1[(rows-20):rows, 1:cols]
        ## find the contours of the white shape
        contours,hierarchy = cv2.findContours(crop, 1, 2)
        ## only update the twist command if you see the road
        contour = False
        if len(contours) !=0:
          cnt = contours[0]
          M = cv2.moments(cnt)
          if M['m00'] != 0:
            contour = True
          #add 1e-5 to avoid divide by zero (found this suggestion on the openCV documentation)
            cx = int(M['m10']/(M['m00']))
            self.prevx = cx
            ## turning more and going slower if the centroid is near the edges
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
        if contour == False:
            cx = self.prevx
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
                   
        ##Code to publish the Twist message
        try:
            self.cmd_vel_pub.publish(self.twist)
        except CvBridgeError as e:
            print(e)

def main():
    ic = robot_mover()
    rospy.init_node('robot_mover', anonymous=True)
    rospy.Subscriber("rrbot/camera1/image_raw",Image,ic.callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()