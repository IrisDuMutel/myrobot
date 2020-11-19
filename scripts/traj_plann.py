#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import pot_field 
import rospy
# import os
# os.environ['MAVLINK20']='1' #set mavlink2 
# import pymavlink import mavutil
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from myrobot.msg import vect_msg
import message_filters
from scipy.spatial.transform import Rotation as R
# connection = mavutil.mavlink_connection("udp:localhost:14540",input=False,dialect = "standard")
global i
global i_old
i = 0
i_old = 0
def traj_plann():
    rospy.init_node('traj_plann',anonymous=True)
    pub = rospy.Publisher('traj_plann',Odometry,queue_size=10)
    # x_sub   = rospy.Subscriber("odom", Odometry, callback)
    x_sub   = message_filters.Subscriber('/odom', Odometry)
    vel_sub = message_filters.Subscriber('/odom', Odometry)
    vx,th,rx,ry = pot_field.pot_field()
    
    # vel_sub = message_filters.Subscriber('/rs_vect', vect_msg)
    ts = message_filters.ApproximateTimeSynchronizer([vel_sub,x_sub], queue_size=10, slop=0.05)
    ts.registerCallback(callback,pub,vx,th,rx,ry)
    
    rospy.spin()

def get_rotation(Xest):
    orientation_q = Xest.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    r = R.from_quat(orientation_list)
    EuAn = r.as_euler('zyx', degrees=False)
    return EuAn 

def callback(vel_sub,x_sub,pub,vx,th,rx,ry):
    traj_plann_command = Odometry()
    global i
    global i_old


    if i <= len(rx)-1:
        next_x = rx[i+1]
        next_y = ry[i+1]
        th_des = th[i+1]
        actual_x = x_sub.pose.pose.position.x
        actual_y = x_sub.pose.pose.position.y
        actual_th,_,_ = get_rotation(x_sub)
        
        # print("next_x",next_x,"next_y",next_y)
        # print("rx:",rx)
        # print("th:",th)
        actual_x = x_sub.pose.pose.position.x
        actual_y = x_sub.pose.pose.position.y
        # print("x odom:",x_sub.pose.pose.position.x)
        actual_th,_,_ = get_rotation(x_sub)
        if np.hypot(next_x-actual_x,next_y-actual_y)>0.1:
            # Create odometry shaped message from planner
            actual_x = x_sub.pose.pose.position.x
            actual_y = x_sub.pose.pose.position.y
            actual_th,_,_ = get_rotation(x_sub)
            # print("distance to node:",np.hypot(next_x-actual_x,next_y-actual_y) )
            traj_plann_command.header.stamp = rospy.Time.now()
            traj_plann_command.pose.pose.position.x = next_x # x from next waypoint
            traj_plann_command.pose.pose.position.y = next_y # y from next waypoint
            traj_plann_command.pose.pose.orientation.w = th[i+1]*180/math.pi # heading
            traj_plann_command.twist.twist.linear.x = 0.5 # linear velocity
            i_old = i
    
            pub.publish(traj_plann_command)
        elif np.hypot(next_x-actual_x,next_y-actual_y)<0.05 and i_old == i: 
            i_old = i
            i += 1
            print("CHANGING WAYPOINT:", i+1)
    

if __name__=='__main__':
    try:
        
        traj_plann()
    except rospy.ROSInterruptException:
        
        pass