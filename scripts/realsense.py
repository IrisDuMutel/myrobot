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

def realsense_behaviour():
    rospy.init_node('realsense_behaviour', anonymous=True)
    pub = rospy.Publisher('rs_vect', vect_msg, queue_size=10)
    color_sub = message_filters.Subscriber('camera/color/image_raw',Image)
    depth_sub = message_filters.Subscriber('camera/depth/image_raw',Image)
    ts = message_filters.TimeSynchronizer([color_sub, depth_sub], 10)
    ts.registerCallback(callback,pub)
    rospy.spin()

def callback(color_raw, depth_raw,pub):
    test = vect_msg()
    msg = vect_msg()
    bridge = CvBridge()
    # realsense min and max distance 
    minDist = 0.3
    maxDist = 4.5
    # set show_depth to True to show the rgb and depth frames together
    rs_plt = True # Display what is seen in color frames
    show_depth = True
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
        
        valX = np.array([30, 319, 609])
        valY = np.array([30, 239, 449])
        dist = np.zeros((3,3))
        valid_matrix =np.zeros((3,3))
        color = np.asanyarray(color_image)
        colorized_depth = np.asanyarray(depth_image)
        # print(colorized_depth)
        # print(len(colorized_depth[1]))
        # print((colorized_depth[120,1]))
        # print((colorized_depth[120,319]))

        for xCount in range(0,3):
            for yCount in range(0,3):
                distance = colorized_depth[valY[yCount],valX[xCount]]
                if(distance < minDist) or math.isnan(distance)==True:
                    distance =  np.inf
                    valid_matrix[yCount,xCount] = 0
                else:
                    valid_matrix[yCount,xCount] = 1
                dist[yCount,xCount] = distance
                if rs_plt:
                    cv2.circle(color,(valX[xCount],valY[yCount]),5,(0,0,255),2)
                    cv2.circle(colorized_depth,(valX[xCount],valY[yCount]),5,(0,255,0),2)
        # print("\033c")
        # print(dist)
        # # Find index of valid  values 
        print(dist)
        validIdxL = np.nonzero(valid_matrix[0:3,0])
        validIdxR = np.nonzero(valid_matrix[0:3,2])
        
        # if(((len(validIdxL[0]))>1 and len(validIdxR[0]))>1):
        #         angle = 180 - (np.mean(dist[validIdxL,0]) - np.mean(dist[validIdxR,2]))*180/np.pi
        #         value = 1/np.amin(dist)
        #         # #TODO remove print functions
        #         # print(validIdxL[0])
        #         # print(dist[validIdxL,0])
        #         # saturation 
        #         if value > 1/minDist:       # saturation to max speed
        #             value = 1.5
        #         if value<1/maxDist:     # saturation to min speed
        #             value = 0
        # else:
        #     angle = 0
        #     value = 0
        
        if(((len(validIdxL[0]))>1 and len(validIdxR[0]))>1):
                angle = 180 - (np.mean(dist[validIdxL,0]) - np.mean(dist[validIdxR,2]))*180/np.pi
                value = 1/np.amin(dist)
                # #TODO remove print functions
                # print(validIdxL[0])
                # print(dist[validIdxL,0])
                # saturation 
                if value > 1/minDist:       # saturation to max speed
                    value = 1.5
                if value<1/maxDist:     # saturation to min speed
                    value = 0
        else:
            angle = 0
            value = 0
        print(angle, value)
        vect = [angle,value]
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
            msg.header.stamp = rospy.Time.now()
            msg.angle = dist[1][1]
            msg.value = dist[1][0]
            rospy.loginfo('Realsense vector data sent')
            pub.publish(msg)

    finally:
        pass
if __name__ == '__main__':
    try:
        realsense_behaviour()
    except rospy.ROSInterruptException:
        pass