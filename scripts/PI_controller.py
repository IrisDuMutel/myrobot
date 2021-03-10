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
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from myrobot.msg import vect_msg, PWM
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
global psi_int
psi_int = 0


def controller():
    rospy.init_node('controller',anonymous=True)
    pub = rospy.Publisher('PWM_refer',PWM,queue_size=10)
    odom_sub   = message_filters.Subscriber('/odom', Odometry)
    ref_sub = message_filters.Subscriber('/simulink_references', Odometry)
    ts = message_filters.ApproximateTimeSynchronizer([ref_sub,odom_sub], queue_size=10, slop=0.5)
    ts.registerCallback(callback,pub)
    rospy.spin()
    

def callback(ref_sub, odom_sub, pub):
    
    # Initialization
    global psi_int
    cmd = Twist()
    pwm_refer = PWM()
    Refer = Odometry()
    Refer = ref_sub
    Odom = Odometry()
    Odom = odom_sub

    # Variable assignation:
    px_goal = Refer.pose.pose.position.x
    py_goal = Refer.pose.pose.position.y
    px_start = Refer.pose.pose.orientation.x
    py_start = Refer.pose.pose.orientation.y
    px_odom = Odom.pose.pose.position.x
    py_odom = Odom.pose.pose.position.y
    vel_ref = Refer.twist.twist.linear.x                   
    vel_odom = Odom.twist.twist.linear.x
    psi_ref = Refer.pose.pose.orientation.w
    
    # real orientation
    [yaw, pitch, roll] = get_rotation(Odom) 
    psi_est = yaw*180/math.pi

    # Distance differences: complete distance vs distance run
    tot_dist = np.hypot(px_goal-px_start,py_goal-py_start)
    rel_dist = np.hypot(px_odom-px_start,py_odom-py_start)

    # Error computation:
    dist_error = tot_dist-rel_dist
    vx_error = vel_ref-vel_odom
    psi_error = psi_ref-psi_est

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

    # Normalization
    psi_cmd = psi_cmd/180 #degrees
    vx_cmd = vx_cmd/1

    # Saturation
    if psi_cmd > 0.5:
        psi_cmd = 0.5
    if psi_cmd < -0.5:
        psi_cmd = -0.5

    if vx_cmd > 1:
        vx_cmd = 1
    if vx_cmd < -1:
        vx_cmd = -1
    
    # PWM commands
    pwm_refer.PWM_right = (vx_cmd+psi_cmd)*20000
    pwm_refer.PWM_left = (vx_cmd-psi_cmd)*20000

    # Publishing
    pub.publish(pwm_refer)

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
        