#!/usr/bin/env python
import rospy 
from nav_msgs.msg import OccupancyGrid, MapMetaData,Odometry
import time
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import numpy as np

class slam_gmapping():
    def __init__(self):
        self.odom_ = Odometry()
        self.map_ = OccupancyGrid()
        self.pub = rospy.Publisher('/slam_map',OccupancyGrid, queue_size=10)
        self.odom_sub   = message_filters.Subscriber('/odom', Odometry)
        self.depth_sub = message_filters.Subscriber('camera/depth/image_raw',Image)
        ts = message_filters.ApproximateTimeSynchronizer([self.odom_sub,self.depth_sub], queue_size=10, slop=0.05)
        ts.registerCallback(self.callback,self.pub)
        self.gmapping()

    def callback(self,odom_sub,depth_sub,pub):
        self.odom_= odom_sub

    def max_index(self,y,x):
        index = self.map_.info.width*y+x
        return index
    def gmapping(self):
        while True:
            try:
                self.map_.header.stamp = rospy.Time.now()
                # self.map_.header.frame_id = 0
                self.map_.info.resolution = 0.5
                self.map_.info.width = 10
                self.map_.info.height = 10
                self.map_.info.origin.position.x = self.odom_.pose.pose.position.x
                self.map_.info.origin.position.y = self.odom_.pose.pose.position.y
                self.map_.info.origin.position.z = self.odom_.pose.pose.position.z
                self.map_.info.origin.orientation.x = 0
                self.map_.info.origin.orientation.y = 0
                self.map_.info.origin.orientation.z = 0
                self.map_.info.origin.orientation.w = 1.0
                # for i in range(self.map_.info.width):
                #     for j in range(self.map_.info.height):
                #         lin_index = self.max_index(j,i)
                #         self.map_.data[self.max_index(j,i)] = 0
                self.map_.data = [0] * (self.map_.info.width * self.map_.info.height) 
                self.pub.publish(self.map_)
            except rospy.ROSInterruptException:
                pass

if __name__=='__main__':
    try:
        rospy.init_node('slam',anonymous=False)
        slam_gmapping()
    except rospy.ROSInterruptException:
        pass