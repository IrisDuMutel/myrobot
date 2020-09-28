#!/usr/bin/env python

import rospy
import math
import random
from gazebo_msgs.msg import ContactsState
from myrobot.msg import vect_msg
import message_filters

msg = vect_msg()
def force():
    rospy.init_node('force',anonymous=True)
    rospy.loginfo("Status: Force Sensors Initialized") 
    # Subscribe to topics
    rightfront_sub = message_filters.Subscriber('/force/rightfront_sensor', ContactsState)
    leftfront_sub = message_filters.Subscriber('/force/leftfront_sensor', ContactsState)
    # Syncronize
    ts = message_filters.TimeSynchronizer([rightfront_sub,leftfront_sub], queue_size=10)
    # Register callback and publisher
    ts.registerCallback(callback)
    rospy.spin()

def callback(rightfront_sub,leftfront_sub):
    msg = vect_msg()
    #TODO define here a behavior according to the contact value
    # angle = (forceL - forceR) * 180/1023
    # force = (forceL+forceR)/2
    msg.value=0.4
    angle = 0
    if not rightfront_sub.states:
        pass
    else:
        rightfront_force = rightfront_sub.states[0].total_wrench.force.x
        rospy.loginfo('Ive been toched at the front F=%s\n', rightfront_force)
        # Fill vector message
        msg.angle = (- 1023) * 180/1023
        msg.value = -0.2 
    if not leftfront_sub.states:
        pass
    else:
        leftfront_force = leftfront_sub.states[0].total_wrench.force.x
        rospy.loginfo('Ive been toched at the back F=%s\n', leftfront_force) 
        # Fill vector message
        msg.angle = 1023* 180/1023
        msg.value = -0.2

    msg.header.stamp = rospy.Time.now()
    pub = rospy.Publisher('force_vect', vect_msg, queue_size=10)
    pub.publish(msg)
    rospy.loginfo('Force vector data sent')
if  __name__ == '__main__':
    try:
        force()
    except rospy.ROSInterruptException:
        pass