#!/usr/bin/env python

import rospy
import math
import random
from myrobot.msg import vect_msg
import message_filters

msg = vect_msg()
def random():
    rospy.init_node('trajplan',anonymous=True) # Create goal node called 'trajplan'
    pub = rospy.Publisher('ref_vel',vect_msg,queue_size=10) #create goal publisher in the topic ref_vel, with vector_msg msgs
    # and goal queue_size (size of outgoing message) of 10
    # rate = rospy.Rate(1) # 1hz
    rospy.loginfo("Status: Initialized") 
    # Subscribe to messages
    rs_sub = message_filters.Subscriber('/rs_vect', vect_msg)
    # Syncronize
    ts = message_filters.TimeSynchronizer([rs_sub], queue_size=10)
    # Register callback and publisher
    ts.registerCallback(callback,pub)
    rospy.spin()

def callback(rs_sub,pub):
    # Collecting vectors
    rs_v  = [rs_sub.angle, rs_sub.value]
    # Sum vectors
    # bb_vect = vectfcn.sum_vect(vectfcn.sum_vect(vect_a,vect_b),vect_c)
    bb_vect =  rs_v
    # bb_vect = vectfcn.sum_vect(vect_a,vect_b)
    rospy.loginfo(bb_vect)
    print(rs_sub.header.seq) 
    msg.header.seq = rs_sub.header.seq
    msg.angle = bb_vect[0]
    msg.value = bb_vect[1]
    print('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n')
    pub.publish(msg)

if  __name__ == '__main__':
    try:
        random()
    except rospy.ROSInterruptException:
        pass