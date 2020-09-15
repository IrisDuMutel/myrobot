#!/usr/bin/env python

import rospy
import math
import random
from gazebo_devastator.msg import vector_msg, flag

rospy.init_node('trajplan',anonymous=True) # Create goal node called 'trajplan'
pub = rospy.Publisher('ref_vel',vector_msg,queue_size=10) #create goal publisher in the topic ref_vel, with vector_msg msgs
# and goal queue_size (size of outgoing message) of 10
rate = rospy.Rate(1) # 1hz
rospy.loginfo("Status: Initialized") 


if  __name__ == '__main__':
    # Initialization
    vel      = vector_msg()  # Initialize publishing vector
    try:
        while not rospy.is_shutdown():
            vel.value = 0.8
            vel.angle = random.uniform(0.01,2*math.pi)*180/math.pi #returns angles between 180 and -180 NOW IN DEG
            pub.publish(vel)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    