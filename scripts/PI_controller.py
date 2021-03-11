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
import numpy
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from myrobot.msg import vect_msg
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
global psi_int
psi_int = 0


def controller():
    rospy.init_node('controller',anonymous=True)
    # odom_sub   = rospy.Subscriber('/odom', Odometry)
    # pub = rospy.Publisher('/PWM_refer',Twist,queue_size=10)
    pub2 = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    odom_sub   = message_filters.Subscriber('/odom', Odometry)
    ref_sub = message_filters.Subscriber('/simulink_references', Twist)
    ts = message_filters.ApproximateTimeSynchronizer([ref_sub,odom_sub], queue_size=10, slop=0.5, allow_headerless=True)
    ts.registerCallback(callback,pub2)
    rospy.spin()
    
def callback(ref_sub,odom_sub,pub2):



    
    # Initialization
    global psi_int
    cmd = Twist()
    Refer = Twist()
    Refer = ref_sub
    Odom = Odometry()
    Odom = odom_sub

    # Variable assignation:
    px_goal = Refer.linear.y
    py_goal = Refer.linear.z
    px_start = 0
    py_start = 0
    px_odom = Odom.pose.pose.position.x
    py_odom = Odom.pose.pose.position.y
    vel_ref = Refer.linear.x                  
    vel_odom = Odom.twist.twist.linear.x
    psi_ref = Refer.angular.z
    
    # real orientation
    [yaw, pitch, roll] = get_rotation(Odom) 
    psi_est = yaw*180/math.pi

    # Distance differences: complete distance vs distance run
    tot_dist = np.hypot(px_goal-px_start,py_goal-py_start)
    rel_dist = np.hypot(px_odom-px_start,py_odom-py_start)

    # Error computation:
    dist_error = tot_dist-rel_dist
    vx_error = vel_ref-vel_odom
    psi_error = psi_ref*180/math.pi-psi_est

    if psi_error>180:
        psi_error = psi_error-360*numpy.sign(psi_error)

    # Control tuning
    Kp_x_Gain = 1
    Ki_x_Gain = 1
    Kp_psi_Gain = 0.7
    Ki_psi_Gain = 0.0
    Integrator_psi_gainval = 0.02

    # Control
    vx_cmd = ((dist_error)*Ki_x_Gain+(vx_error)*Kp_x_Gain)
    psi_int += Integrator_psi_gainval*psi_error
    psi_cmd = (psi_error*Kp_psi_Gain+psi_int*Ki_psi_Gain)

    psivel_cmd = psi_error/30

    # Normalization
    psi_cmd = psi_cmd/180 #degrees
    vx_cmd = vx_cmd/1

    # Saturation
    # if psi_cmd > 0.5:
    #     psi_cmd = 0.5
    # if psi_cmd < -0.5:
    #     psi_cmd = -0.5
    if psivel_cmd > 3:
        psivel_cmd = 3
    if psivel_cmd < -3:
        psivel_cmd = -3

    if vx_cmd > 0.4:
        vx_cmd = 0.4
    if vx_cmd < -0.4:
        vx_cmd = -0.4
    
    # PWM commands

    # if dist_error>0.1:
    #     # PWM_right
    #     cmd.linear.x = (vx_cmd+psi_cmd)*20000
    #     # PWM_left
    #     cmd.linear.y = (vx_cmd-psi_cmd)*20000
    # else:
    #     cmd.linear.x = 0
    #     cmd.linear.y = 0
    
    # pub.publish(cmd)

    # cmd_vel commands
    if dist_error>0.1:
        cmd.linear.x = vel_ref
        cmd.angular.z = psivel_cmd
        cmd.linear.y = 0
    else:
        cmd.linear.x = 0
        cmd.angular.z = 0
        cmd.linear.y = 0

    pub2.publish(cmd)

    # Publishing
    

def get_rotation(Odom):
    orientation_q = Odom.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    r = R.from_quat(orientation_list)
    EuAn = r.as_euler('zyx', degrees=False)
    return EuAn 
    
if __name__ == "__main__":
    try:
        controller()
    except rospy.ROSInterruptException:

        pass
        