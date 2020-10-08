#!/usr/bin/env python
# Code property of Matteo Scanavino - matteo.svanavino@gmail.it
# Minor changes by Iris David Du Mutel
import rospy
# from std_msgs.msg import Float32MultiArray
from myrobot.msg import vect_msg
import cv2 
# import cv2.cv
import os
import math
import numpy as np
#import pyrealsense2 as rs
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import imutils #collection of OpenCV and Python convenience functions
from collections import deque

def green_ball():
    rospy.init_node('realsense_behaviour', anonymous=True)
    pub = rospy.Publisher('gb_vect', vect_msg, queue_size=10)
    color_sub = message_filters.Subscriber('camera/color/image_raw',Image)
    depth_sub = message_filters.Subscriber('camera/depth/image_raw',Image)
    ts = message_filters.TimeSynchronizer([color_sub, depth_sub], 10)
    ts.registerCallback(callback,pub)
    rospy.spin()

def callback(color_raw, depth_raw,pub):
    vect = [0, 0]
    msg = vect_msg()
    bridge = CvBridge()
    greenLower = (29, 86, 6)
    greenUpper = (64, 255, 255)
    # realsense min and max distance 
    try:
            color_image = bridge.imgmsg_to_cv2(color_raw, "bgr8")
    except CvBridgeError as e:
            print(e)

    frame = imutils.resize(color_image, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if radius<100:
            vect[0]=0
            vect[1]=0.8
        else:
            vect[0]=0
            vect[1]=0
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        print(center)
        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
                (0, 255, 255), 2)
            print('radius=', radius)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
    else:
        print('out of frame')
    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    # Send data
    msg.header.stamp = rospy.Time.now()
    msg.angle = vect[0]
    msg.value = vect[1]
    rospy.loginfo('Realsense vector data sent')
    pub.publish(msg)
    
if __name__ == '__main__':
    try:
        green_ball()
    except rospy.ROSInterruptException:
        pass