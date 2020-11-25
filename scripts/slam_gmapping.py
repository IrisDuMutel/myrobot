#!/usr/bin/env python
import rospy 
from nav_msgs.msg import OccupancyGrid, MapMetaData,Odometry
from myrobot.msg import vect_msg

import cv2 
import os
import math
import time
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R

class slam_gmapping():
    
    def __init__(self):
        self.odom_ = Odometry()
        self.depth_ = Image()
        self.dist_ = vect_msg()
        self.map_ = OccupancyGrid()
        self.map_.info.width = 20
        self.map_.info.height = 20
        self.bridge = CvBridge()
        self.map_.data = [0] * (self.map_.info.width * self.map_.info.height) # start an unoccupied map
        self.pub = rospy.Publisher('/slam_map',OccupancyGrid, queue_size=10)
        self.odom_sub   = message_filters.Subscriber('/odom', Odometry)
        self.color_sub = message_filters.Subscriber('rs_vect', vect_msg)
        self.depth_sub = message_filters.Subscriber('camera/depth/image_raw',Image)
        ts = message_filters.ApproximateTimeSynchronizer([self.odom_sub,self.depth_sub,self.color_sub], queue_size=10, slop=0.05)
        ts.registerCallback(self.callback,self.pub)
        self.gmapping()

    def callback(self,odom_sub,depth_sub,color_sub,pub):
        self.odom_= odom_sub
        self.depth_ = depth_sub
        self.dist_ = color_sub


    def get_rotation(self):
        orientation_q = self.odom_.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        r = R.from_quat(orientation_list)
        EuAn = r.as_euler('zyx', degrees=False)
        return EuAn 
    def max_index(self,y,x):
        index = self.map_.info.width*y+x
        return index
    def gmapping(self):
        print(self.depth_)
        while True:
            try:
                # minDist = 0.3
                # maxDist = 4.5
                # try:
                #     color_image = self.bridge.imgmsg_to_cv2(self.color_, "bgr8")
                #     depth_image = self.bridge.imgmsg_to_cv2(self.depth_, "32FC4")
                # except CvBridgeError as e:
                #     print(e)
                # depth_array = np.array(depth_image, dtype=np.float32)
                # norm_depth=cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
                # valX = np.array([30, 319, 609])
                # valY = np.array([30, 239, 449])
                # dist = np.zeros((3,3))
                # valid_matrix =np.zeros((3,3))
                # colorized_depth = np.asanyarray(depth_image)
                # for xCount in range(0,3):
                #     for yCount in range(0,3):
                #         distance = colorized_depth[valY[yCount],valX[xCount]]
                #         if(distance < minDist) or math.isnan(distance)==True:
                #             distance =  np.inf
                #             valid_matrix[yCount,xCount] = 0
                #         else:
                #             valid_matrix[yCount,xCount] = 1
                #         dist[yCount,xCount] = distance
                dist = self.dist_.angle
                theta, _, _ = self.get_rotation()
                if dist> 0 and dist!='inf':

                    x = dist[1][1]*np.cos(theta)
                    y = dist[1][1]*np.sin(theta)
                    lin_index = self.max_index(j,i)
                    self.map_.data[lin_index] = 100
                else:
                    pass
                
                self.map_.header.stamp = rospy.Time.now()
                # self.map_.header.frame_id = 0
                self.map_.info.resolution = 0.5
                self.map_.info.width = 20
                self.map_.info.height = 20
                # np.resize(self.map_.data.resize, self.map_.info.width*self.map_.info.height) 
                self.map_.info.origin.position.x = 0
                self.map_.info.origin.position.y = 0
                self.map_.info.origin.position.z = 0
                self.map_.info.origin.orientation.x = 0
                self.map_.info.origin.orientation.y = 0
                self.map_.info.origin.orientation.z = 0
                self.map_.info.origin.orientation.w = 1.0

                # for i in range(self.map_.info.width):
                #     for j in range(self.map_.info.height):
                #         lin_index = self.max_index(j,i)
                #         print(lin_index)
                #         self.map_.data[lin_index] = 10
                # self.map_.data = [100] * (self.map_.info.width * self.map_.info.height) 
                # print(len(self.map_.data))
                self.pub.publish(self.map_)
            except rospy.ROSInterruptException:
                pass

if __name__=='__main__':
    try:
        rospy.init_node('slam',anonymous=False)
        slam_gmapping()
    except rospy.ROSInterruptException:
        pass