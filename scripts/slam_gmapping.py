#!/usr/bin/env python

# SLAM gmapping done with depth camera. 
# This script has the task of transforming the depth data from the realsense
# into a laser scan message.
import rospy
from myrobot.msg import vect_msg
import cv2 
import os
import math
import time
import numpy as np
import message_filters
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge, CvBridgeError

def slam_gmapping():
    rospy.init_node('realsense_behaviour', anonymous=True)
    pub = rospy.Publisher('rs_vect', LaserScan, queue_size=10)
    color_sub = message_filters.Subscriber('camera/color/image_raw',Image)
    depth_sub = message_filters.Subscriber('camera/depth/image_raw',Image)
    ts = message_filters.TimeSynchronizer([color_sub, depth_sub], 10)
    ts.registerCallback(callback,pub)
    rospy.spin()

def callback(color_raw, depth_raw,pub):
    test = vect_msg()
    msg = LaserScan()
    bridge = CvBridge()
    # realsense min and max distance 
    minDist = 0.3
    maxDist = 4.5
    # set show_depth to True to show the rgb and depth frames together
    rs_plt = True # Display what is seen in color frames
    show_depth = False
    # Show images
    if rs_plt:
        cv2.namedWindow('RealSense color', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('RealSense depth', cv2.WINDOW_AUTOSIZE)
    try:
        
        try:
            color_image = bridge.imgmsg_to_cv2(color_raw, "bgr8")
            depth_image = bridge.imgmsg_to_cv2(depth_raw, "32FC1")
        except CvBridgeError as e:
            print(e)
        
        depth_array = np.array(depth_image, dtype=np.float32)
        norm_depth=cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        valX = np.arange(30,609,10)
        valY = np.array([200])
        dist = np.zeros(len(valX))
        valid_matrix =np.zeros(len(valX))
        color = np.asanyarray(color_image)
        colorized_depth = np.asanyarray(depth_image)


        for xCount in range(0,len(valX)):
            distance = colorized_depth[valY[0],valX[xCount]]
            if(distance < minDist) or math.isnan(distance)==True:
                distance =  np.inf
                valid_matrix[xCount] = 0
            else:
                valid_matrix[xCount] = 1
            dist[xCount] = distance
            if rs_plt:
                cv2.circle(color,(valX[xCount],valY[0]),5,(0,0,255),2)
                cv2.circle(colorized_depth,(valX[xCount],valY[0]),5,(0,255,0),2)

        # # Find index of valid  values 
        print(dist)

        RGB = np.dstack((norm_depth, np.zeros_like(norm_depth), np.zeros_like(norm_depth)))
        grey_3_channel = cv2.cvtColor(norm_depth, cv2.COLOR_GRAY2BGR)

        if rs_plt:
            cv2.imshow('RealSense color', color)
            if show_depth:
                cv2.imshow('RealSense depth', grey_3_channel)
                
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                # break
                pass

            # Send data
            msg.header.stamp=rospy.Time.now()
            msg.header.frame_id='RealSense'
            msg.angle_min = 0
            msg.angle_max = 6.28319
            msg.time_increment = 0;  # instantaneous simulator scan
            # msg.angle_increment=0.017
            msg.scan_time = 0;  # not sure whether this is correct
            msg.range_min = 0.11
            msg.range_max = 3.5
            msg.ranges = dist
            msg.intensities = np.zeros(len(dist))
            # msg.header.stamp = rospy.Time.now()
            # msg.angle = dist[1][1]
            # msg.value = dist[1][0]
            # rospy.loginfo('Realsense vector data sent')
            pub.publish(msg)

    finally:
        pass
if __name__ == '__main__':
    try:
        slam_gmapping()
    except rospy.ROSInterruptException:
        pass