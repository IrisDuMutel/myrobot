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








def talker():
    pub = rospy.Publisher('/sg90h/cmd_servo', Twist, queue_size=10)
    rospy.init_node('plotter', anonymous=True)
    rate = rospy.Rate(30) # 10hz
    cmd=Twist()

    while not rospy.is_shutdown():
        # Transform into velocity commands
        # cmd.linear.x = 0
        cmd.angular.z = 0.5
        # Publishing
        pub.publish(cmd)
        rospy.loginfo("In the loop")
        rate.sleep()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        
        pass

    






