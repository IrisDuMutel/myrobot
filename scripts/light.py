#!/usr/bin/env python
# Code property of Matteo Scanavino
# Minor changes by Iris David Du Mutel
import rospy
# from std_msgs.msg import Float32MultiArray
from myrobot.msg import vect_msg
# import cv2 
import os
import math
# import numpy as np
#import pyrealsense2 as rs
import message_filters
from sensor_msgs.msg import Image, CameraInfo, Illuminance
# from cv_bridge import CvBridge, CvBridgeError

def light():
    rospy.init_node('light', anonymous=True)
    # image_sub = message_filters.Subscriber('camera2/rgb/image_raw',Image)
    light_sub = message_filters.Subscriber('light_sensor_plugin/lightSensor',Illuminance)
    ts = message_filters.TimeSynchronizer([light_sub], 10)
    ts.registerCallback(callback)
    rospy.spin()
def callback( light_sub):
    light_val = light_sub.illuminance
    # Measurement of the Photometric Illuminance in Lux.
    rospy.loginfo('The iluminance value is %s\n', light_val)
    msg = vect_msg()
    #TODO define here a behavior according to the lux value
    angle = 0
    # Fill vector message
    msg.angle = angle
    msg.value = light_val
    msg.header.stamp = rospy.Time.now()
    pub = rospy.Publisher('light_vect', vect_msg, queue_size=10)
    pub.publish(msg)
    rospy.loginfo('Light vector sent')
if __name__ == '__main__':
    try:
        light()
    except rospy.ROSInterruptException:
        pass