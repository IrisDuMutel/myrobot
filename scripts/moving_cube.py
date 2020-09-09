#!/usr/bin/env python

import rospy
import math
import random
from gazebo_msgs.msg import ModelState

rospy.init_node('moving_cube',anonymous=True) # Create goal node called 'iris', that doesnt need to be anon
pub = rospy.Publisher('gazebo/set_model_state',ModelState,queue_size=10) #create goal publisher in the topic cmd_status, with Twist msgs
# and goal queue_size (size of outgoing message) of 10
rate = rospy.Rate(5) # 10hz
rospy.loginfo("Status: Initialized") 


if  __name__ == '__main__':
    # Initialization
    status      = ModelState()  # Initialize publishing vector
    # Constants
    tolerance = 0.1  # imposed tolerance
    distance  = 1000 # big number to begin loop
    # Goal not reached yet, publish it
    try:
        while not rospy.is_shutdown():
            status.model_name = 'moving_cube'
            status.twist.angular.z = 1 #returns angles between 180 and -180 NOW IN DEG
            pub.publish(status)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    