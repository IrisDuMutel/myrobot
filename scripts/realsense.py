#!/usr/bin/env python
# Code property of Matteo Scanavino
# Minor changes by Iris David Du Mutel
import rospy
# from std_msgs.msg import Float32MultiArray
from myrobot.msg import vect_msg
import cv2 
# import cv2.cv
import os
import numpy as np
#import pyrealsense2 as rs
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

def realsense_behaviour():
    rospy.init_node('realsense_behaviour', anonymous=True)
    color_sub = message_filters.Subscriber('camera/color/image_raw',Image)
    depth_sub = message_filters.Subscriber('camera/depth/image_raw',Image)
    ts = message_filters.TimeSynchronizer([color_sub, depth_sub], 10)
    ts.registerCallback(realsense)
    rospy.spin()
def realsense(color_raw, depth_raw):
    test = vect_msg()
    # Intialize node and topic
    pub = rospy.Publisher('rs_vect', vect_msg, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    msg = vect_msg()
    bridge = CvBridge()
    # set show_depth to True to show the rgb and depth frames together
     # rs_plt = rospy.get_param("/rs_video")
     # show_depth = rospy.get_param("/rs_depth")
    # realsense min and max distance 
    minDist = 0.3
    maxDist = 4.5


    # Configure depth and color streams
     # pipeline = rs.pipeline()
     # config = rs.config()
     # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
     # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
     # profile = pipeline.start(config)

    rs_plt = True # Display what is seen
    show_depth = True
    # Show images
    if rs_plt:
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)

    try:
        while not rospy.is_shutdown():

            # # skip 5 first frames to give Auto-Exposure time to adjust
             # for i in range(5):
             #     pipeline.wait_for_frames()

            # Wait for a coherent pair of frames: depth and color
             # frames = pipeline.wait_for_frames()
             # color_frame = frames.get_color_frame()
             # depth_frame = frames.get_depth_frame()
            

            # Access color component of the frame
             # color = np.asanyarray(color_frame.get_data())
            try:
                color_image = bridge.imgmsg_to_cv2(color_raw, "bgr8")
                depth_image = bridge.imgmsg_to_cv2(depth_raw, "32FC1")            
            except CvBridgeError as e:
                print(e)
            

            depth_array = np.array(depth_image, dtype=np.float32)
            cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
            rospy.loginfo("AAAAAAAAA\n")
            # Access depth component
             # colorizer = rs.colorizer()
             # colorizer_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data()    
            # Depth and rgb image alignment
    #         align = rs.align(rs.stream.color)
    #         frames = align.process(frames)
    #         aligned_depth_frame = frames.get_depth_frame()
    #         colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

            valX = np.array([30, 319, 609])
            valY = np.array([30, 239, 449])
            dist = np.zeros((3,3))
            valid_matrix =np.zeros((3,3))

            color = np.asanyarray(color_image)
            colorized_depth = np.asanyarray(depth_image)
            # print(colorized_depth)
            print((colorized_depth[120,1]))
            print((colorized_depth[120,319]))
            # for xCount in range(0,3):
            #     for yCount in range(0,3):
            #         distance = aligned_depth_frame.get_distance(valX[xCount],valY[yCount])
            #         if(distance < 0.3):
            #             distance =  np.inf
            #             valid_matrix[yCount,xCount] = 0
            #         else:
            #             valid_matrix[yCount,xCount] = 1
            #         dist[yCount,xCount] = distance
            #         if rs_plt:
            #             cv2.circle(color,(valX[xCount],valY[yCount]),5,(0,0,255),2)
            #             cv2.circle(colorized_depth,(valX[xCount],valY[yCount]),5,(0,255,0),2)
            # # print("\033c")
            # # print(dist)

            # # Find index of valid  values 
            # validIdxL = np.nonzero(valid_matrix[0:3,0])
            # validIdxR = np.nonzero(valid_matrix[0:3,2])
            
            # Show the two frames together:
            # if show_depth: 
            #     images = np.hstack((color, colorized_depth))
            # else:
            #     images = color
            # if rs_plt:
            #     cv2.imshow('RealSense', images)
            #     k = cv2.waitKey(1) & 0xFF
            #     if k == ord('q'):
            #         break

    #         # Send data
    #         msg.header.stamp = rospy.Time.now()
    #         msg.angle = vect[0]
    #         msg.value = vect[1]
    #         rospy.loginfo('Realsense vector data sent')
    #         pub.publish(msg)
    #         rate.sleep()
     
    # finally:
    #     # Stop streaming
    #     pipeline.stop()
    finally:
        pass
if __name__ == '__main__':
    try:
        realsense_behaviour()
    except rospy.ROSInterruptException:
        pass