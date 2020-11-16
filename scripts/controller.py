#!/usr/bin/env python
# //
# // Academic License - for use in teaching, academic research, and meeting
# // course requirements at degree granting institutions only.  Not for
# // government, commercial, or other organizational use.
# //
# // File: feedback_control.cpp
# //
# // Code generated for Simulink model 'feedback_control'.
# //
# // Model version                  : 1.85
# // Simulink Coder version         : 9.1 (R2019a) 23-Nov-2018
# // /++ source code generated on : Thu May 21 16:56:39 2020
# //
# // Target selection: ert.tlc
# // Embedded hardware selection: ARM Compatible->ARM Cortex
# // Code generation objectives: Unspecified
# // Validation result: Not run
# //

import rospy
import math
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from myrobot.msg import vect_msg
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from matplotlib import pyplot as plt


def controller():
    rospy.init_node('controller',anonymous=True)
    pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    x_sub   = message_filters.Subscriber('/odom', Odometry)
    vel_sub = message_filters.Subscriber('/traj_plann', vect_msg)
    # vel_sub = message_filters.Subscriber('/rs_vect', vect_msg)
    ts = message_filters.ApproximateTimeSynchronizer([vel_sub,x_sub], queue_size=10, slop=0.1)
    ts.registerCallback(callback,pub)
    # rate=rospy.Rate(30)
    # rate.sleep()
    plt.ion()
    plt.show()
    rospy.spin()


def callback(vel_sub, x_sub, pub):
    U = vect_msg()
    Xest = Odometry()
    cmd = Twist()
    U = vel_sub
    Xest = x_sub
    # Variable assignation:
    x_ref = U.value                    
    x_est = Xest.twist.twist.linear.x
    [yaw, pitch, roll] = get_rotation(Xest)
    psi_ref = U.angle
    psi_est = yaw*180/math.pi
    # Error computation:
    x_error = x_ref-x_est
    psi_error = psi_ref-psi_est
    # print('psi_error:',psi_error)
    # print(psi_est)
    # print('ref_vel:', x_ref)

    # Control
    x_cmd = x_error*.5
    psi_cmd = psi_error*1
    # Normalization
    psi_cmd = psi_cmd/180 #degrees
    x_cmd = x_cmd/1
    # Saturation
    if psi_cmd > 0.5:
        psi_cmd = 0.5
    if psi_cmd < -0.5:
        psi_cmd = -0.5
    print('psi_cmd: ', psi_cmd)
    if x_cmd > 1:
        x_cmd = 1
    if x_cmd < -1:
        x_cmd = -1
    
    # Transform into velocity commands
    cmd.linear.x = x_cmd
    cmd.angular.z = psi_cmd
    # Publishing
    pub.publish(cmd)
    # rospy.loginfo("In the loop")
    show_plot=True
    # if show_plot==True:
    #     stamp = vel_sub.header.stamp
    #     time = stamp.secs + stamp.nsecs * 1e-9
    #     plt.plot(time, psi_cmd, 'g*', label='psi cmd')
    #     # plt.plot(time, psi_ref/180, 'bo', label='psi ref')
    #     plt.plot(time, psi_est/180, 'r+', label='psi est')
    #     # plt.axis("equal")
    #     plt.ylim(-1.5,1.5)
    #     plt.grid()
    #     plt.draw()
    #     # plt.legend()
    #     plt.pause(0.0000001)



def get_rotation(Xest):
    orientation_q = Xest.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    r = R.from_quat(orientation_list)
    EuAn = r.as_euler('zyx', degrees=False)
    return EuAn 
    
if __name__ == "__main__":
    try:
        controller()
                    
    except rospy.ROSInterruptException:
        pass