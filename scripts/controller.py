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
# // C/C++ source code generated on : Thu May 21 16:56:39 2020
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
from gazebo_devastator.msg import vector_msg, flag

class controller():
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
        rospy.init_node('controller',anonymous=True)
        self.rate = rospy.Rate(10) #10hz publishing rate
        self.U = vector_msg()
        self.Xest = Odometry()
        self.cmd = Twist()

    def input(self):
        u = rospy.wait_for_message("ref_vel",vector_msg,None)
        return u

    def estimated_state(self):
        x = rospy.wait_for_message("odom",Odometry,None)
        return x
        
    def get_rotation(self):
        orientation_q = self.Xest.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        r = R.from_quat(orientation_list)
        self.EuAn = r.as_euler('zyx', degrees=False)
        return self.EuAn

if __name__ == "__main__":
    try:
        C = controller()                     #Initialization of class
        while not rospy.is_shutdown():  
            C.U = C.input()                      #Get info from topic ref_vel
            C.Xest = C.estimated_state()         #Get estimated state from odom
            # Variable assignation:
            x_ref = C.U.value                    
            x_est = C.Xest.twist.twist.linear.x
            [yaw, pitch, roll] = C.get_rotation()
            psi_ref = C.U.angle
            psi_est = yaw*180/math.pi
    
            # Error computation:
            x_error = x_ref-x_est
            psi_error = psi_ref-psi_est
            print(psi_est)
            print(psi_ref)
            
            # Control
            x_cmd = x_error*1
            psi_cmd = psi_error*1
    
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
            C.cmd.linear.x = x_cmd
            C.cmd.angular.z = psi_cmd
            # Publishing
            C.pub.publish(C.cmd)
            C.rate.sleep()
            rospy.loginfo("In the loop")
    except rospy.ROSInterruptException:
        pass




