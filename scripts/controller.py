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



def controller():
    print('ccccc')
    rospy.init_node('controller',anonymous=True)
    pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    vel_sub = message_filters.Subscriber('/ref_vel', vect_msg)
    x_sub   = message_filters.Subscriber('/odom', Odometry)
    ts = message_filters.TimeSynchronizer([vel_sub,x_sub], queue_size = 10)
    ts.registerCallback(callback,pub)
    # rate=rospy.Rate(30)
    # rate.sleep()
    print('dddddd')
    rospy.spin()
   

def callback(vel_sub, x_sub, pub):
    print('aaaaaaaaa')
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
    print(psi_est)
    print(psi_ref)
    
    # Control
    x_cmd = x_error*0.25
    psi_cmd = psi_error*0.3
    # Normalization
    psi_cmd = psi_cmd/180 #degrees
    x_cmd = x_cmd/1
    # Saturation
    if psi_cmd > 0.5:
        psi_cmd = 0.5
    if psi_cmd < -0.5:
        psi_cmd = -0.5
    if x_cmd > 1:
        x_cmd = 1
    if x_cmd < -1:
        x_cmd = -1
    
    # Transform into velocity commands
    cmd.linear.x = x_cmd
    cmd.angular.z = psi_cmd
    # Publishing
    pub.publish(cmd)
    rospy.loginfo("In the loop")



def get_rotation(Xest):
    orientation_q = Xest.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    r = R.from_quat(orientation_list)
    EuAn = r.as_euler('zyx', degrees=False)
    return EuAn 
    
if __name__ == "__main__":
    try:
        print('bbbbb')
        controller()            
    except rospy.ROSInterruptException:
        pass




