#!/usr/bin/env python
# Code property of Matteo Scanavino
# Minor changes by Iris David Du Mutel
import rospy
# from std_msgs.msg import Float32MultiArray
from myrobot.msg import vect_msg
# import cv2 
import os
import math
import random
# import numpy as np
#import pyrealsense2 as rs
import message_filters
from sensor_msgs.msg import Image, CameraInfo, Illuminance
# from cv_bridge import CvBridge, CvBridgeError

def wandering():
    rospy.init_node('wandering', anonymous=True)
    pub = rospy.Publisher('wander_vect', vect_msg, queue_size=10)
    msg = vect_msg()
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        value = 1
        angle = random.randint(1,360)
        # Fill vector message
        msg.angle = angle
        msg.value = value
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rospy.loginfo('Wander vector sent')
        rate.sleep()

if __name__ == '__main__':
    try:
        wandering()
    except rospy.ROSInterruptException:
        pass