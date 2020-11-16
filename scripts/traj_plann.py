#!/usr/bin/env python

from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import my_pot_field as pot_field
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from myrobot.msg import vect_msg
import message_filters
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R

def traj_plann():
    rospy.init_node('traj_plann',anonymous=True)
    pub = rospy.Publisher('traj_plann',vect_msg,queue_size=10)
    x_sub   = message_filters.Subscriber('/odom', Odometry)
    vel_sub = message_filters.Subscriber('/odom', Odometry)
    # vel_sub = message_filters.Subscriber('/rs_vect', vect_msg)
    ts = message_filters.ApproximateTimeSynchronizer([vel_sub,x_sub], queue_size=10, slop=0.1)
    ts.registerCallback(callback,pub)
    rospy.spin()

def get_rotation(Xest):
    orientation_q = Xest.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    r = R.from_quat(orientation_list)
    EuAn = r.as_euler('zyx', degrees=False)
    return EuAn 

def callback(vel_sub, x_sub, pub):
    vx,th,rx,ry = pot_field.main()

    i = 0
    while i <= len(rx)-1:
        next_x = rx[i+1]
        next_y = ry[i+1]
        th_des = th[i+1]
        actual_x = x_sub.pose.pose.position.x
        actual_y = x_sub.pose.pose.position.y
        actual_th,_,_ = get_rotation(x_sub)
        traj_plann_command = vect_msg()

        if np.hypot(next_x-actual_x,next_y-actual_y)>0.2:
            traj_plann_command.value = 0.5
            traj_plann_command.angle = th[i+1]
            pub.publish(traj_plann_command)
        else: 
            i += 1

 


    





if __name__=='__main__':
    try:
        traj_plann()
    except rospy.ROSInterruptException:
        
        pass