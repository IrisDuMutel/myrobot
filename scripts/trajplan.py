#!/usr/bin/env python

import rospy
import math
import random
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
# import numpy as np
from gazebo_devastator.msg import vector_msg, flag
# This code makes use of goal normal structure and the function wait_for_message()

rospy.init_node('trajplan',anonymous=True) # Create goal node called 'iris', that doesnt need to be anon
pub = rospy.Publisher('ref_vel',vector_msg,queue_size=10) #create goal publisher in the topic cmd_vel, with Twist msgs
# and goal queue_size (size of outgoing message) of 10
rate = rospy.Rate(1) # 10hz
rospy.loginfo("Status: Initialized") 


if  __name__ == '__main__':
    # Initialization
    vel      = vector_msg()  # Initialize publishing vector
    # Constants
    tolerance = 0.1  # imposed tolerance
    distance  = 1000 # big number to begin loop
    # Goal not reached yet, publish it
    try:
        while not rospy.is_shutdown():
            vel.value = 0.8
            vel.angle = random.uniform(0.01,2*math.pi)*180/math.pi #returns angles between 180 and -180 NOW IN DEG
            pub.publish(vel)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    