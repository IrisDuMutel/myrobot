#!/usr/bin/env python
# license removed for brevity
import rospy
# from std_msgs.msg import Float32MultiArray
from myrobot.msg import vect_msg
import cv2 
import os
import numpy as np
import pyrealsense2 as rs

def realsense():
    # Intialize node and topic
    pub = rospy.Publisher('rs_vect', vect_msg, queue_size=10)
    rospy.init_node('realsense_behaviour', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg = vect_msg()

    # set show_depth to True to show the rgb and depth frames together
    rs_plt = rospy.get_param("/rs_video")
    show_depth = rospy.get_param("/rs_depth")

    # realsense min and max distance 
    minDist = 0.3
    maxDist = 4.5

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Show images
    if rs_plt:
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)

    try:
        while not rospy.is_shutdown():

            # skip 5 first frames to give Auto-Exposure time to adjust
            for i in range(5):
                pipeline.wait_for_frames()

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            # Access color component of the frame
            color = np.asanyarray(color_frame.get_data())

            # Access depth component
            colorizer = rs.colorizer()
            colorizer_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

            # Depth and rgb image alignment
            align = rs.align(rs.stream.color)
            frames = align.process(frames)
            aligned_depth_frame = frames.get_depth_frame()
            colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

            valX = np.array([30, 319, 609])
            valY = np.array([30, 239, 449])
            dist = np.zeros((3,3))
            valid_matrix =np.zeros((3,3))

            for xCount in range(0,3):
                for yCount in range(0,3):
                    distance = aligned_depth_frame.get_distance(valX[xCount],valY[yCount])
                    if(distance < 0.3):
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

            # Find index of valid  values 
            validIdxL = np.nonzero(valid_matrix[0:3,0])
            validIdxR = np.nonzero(valid_matrix[0:3,2])
            
            if(((len(validIdxL[0]))>1 and len(validIdxR[0]))>1):
                angle = 180 - (np.mean(dist[validIdxL,0]) - np.mean(dist[validIdxR,2]))*180/np.pi
                value = 1/np.amin(dist)
                # saturation 
                if value > 1/minDist:       # saturation to max speed
                    value = 1.5
                if value<1/maxDist:     # saturation to min speed
                    value = 0
            else:
                angle = 0
                value = 0
            
            # print(angle, value)
            vect = [180,value]
    
            # Show the two frames together:
            if show_depth: 
                images = np.hstack((color, colorized_depth))
            else:
                images = color
            if rs_plt:
                cv2.imshow('RealSense', images)
                k = cv2.waitKey(1) & 0xFF
                if k == ord('q'):
                    break

            # Send data
            msg.header.stamp = rospy.Time.now()
            msg.angle = vect[0]
            msg.value = vect[1]
            rospy.loginfo('Realsense vector data sent')
            pub.publish(msg)
            rate.sleep()
     
    finally:
        # Stop streaming
        pipeline.stop()

if __name__ == '__main__':
    try:
        realsense()
    except rospy.ROSInterruptException:
        pass