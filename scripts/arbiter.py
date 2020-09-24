#!/usr/bin/env python

import rospy
import math
import random
from myrobot.msg import vect_msg
import message_filters
import vector_fcn as vectfcn

msg = vect_msg()
def arbiter():
    rospy.init_node('arbiter',anonymous=True) # Create goal node called 'trajplan'
    pub = rospy.Publisher('ref_vel',vect_msg,queue_size=10) #create goal publisher in the topic ref_vel, with vector_msg msgs
    # and goal queue_size (size of outgoing message) of 10
    rospy.loginfo("Status: Arbiter initialized") 
    # Subscribe to topics
    rs_sub = message_filters.Subscriber('/rs_vect', vect_msg)
    light_sub = message_filters.Subscriber('/light_vect', vect_msg)
    force_sub = message_filters.Subscriber('/force_vect', vect_msg)
    # Syncronize
    ts = message_filters.TimeSynchronizer([rs_sub,light_sub,force_sub], queue_size=10)
    # Register callback and publisher
    ts.registerCallback(callback,pub)
    rospy.spin()

def callback(rs_sub,light_sub,contact_sub,pub):
    # Collecting vectors
    vect_a  = [rs_sub.angle, rs_sub.value]
    vect_b  = [force_sub.angle, force_sub.value]
    vect_c  = [light_sub.angle, light_sub.value] 
    # Sum vectors
    # TODO check if the vectfcn.sum works
    bb_vect = vectfcn.sum_vect(vectfcn.sum_vect(vectfcn.sum_vect(vectfcn.sum_vect(vect_a,vect_b),vect_c),vect_d),vect_e)
    rospy.loginfo(bb_vect) 
    # Publish message to topic
    msg.angle = bb_vect[0]
    msg.value = bb_vect[1]
    pub.publish(msg)

if  __name__ == '__main__':
    try:
        arbiter()
    except rospy.ROSInterruptException:
        pass