#!/usr/bin/env python

import rospy
import time
import math
import random
from gazebo_msgs.msg import ContactsState
from myrobot.msg import vect_msg
import message_filters

msg = vect_msg()
def force():
    rospy.init_node('force',anonymous=True)
    rospy.loginfo("Status: Force Sensors Initialized") 
    value = 1
    angle = 0
    # Subscribe to topics
    rightfront_sub = message_filters.Subscriber('/force/rightfront_sensor', ContactsState)
    leftfront_sub = message_filters.Subscriber('/force/leftfront_sensor', ContactsState)
    # Syncronize
    ts = message_filters.ApproximateTimeSynchronizer([rightfront_sub,leftfront_sub], queue_size=10, slop=0.1)
    ts.registerCallback(callback,value,angle)# Register callback and publisher
    rospy.spin()

def callback(rightfront_sub,leftfront_sub,value,angle):
    msg = vect_msg()
    #TODO define here a behavior according to the contact value
    # angle = (forceL - forceR) * 180/1023
    # force = (forceL+forceR)/2
    msg.value=value
    msg.angle=angle
    
    if not rightfront_sub.states:
        pass
    else:
        rightfront_force = rightfront_sub.states[0].total_wrench.force.x
        rospy.loginfo('Ive been toched from the right F=%s\n', rightfront_force)
        # Fill vector message
        angle = -90
        value = -1
        right_now = rospy.Time.now()
        seconds = rospy.Duration(2)
        endtime = right_now+seconds
        msg.header.stamp = rospy.Time.now()
        msg.value = value
        msg.angle = angle
        rospy.loginfo('Force vector data sent')
        pub = rospy.Publisher('force_vect', vect_msg, queue_size=10)
        while rospy.Time.now()<endtime:
            pub.publish(msg)

    if not leftfront_sub.states:
        pass
    else:
        leftfront_force = leftfront_sub.states[0].total_wrench.force.x
        rospy.loginfo('Ive been toched from the left F=%s\n', leftfront_force) 
        # Fill vector message
        angle = 90
        value = -1
        right_now = rospy.Time.now()
        seconds = rospy.Duration(2)
        endtime = right_now+seconds
        msg.header.stamp = rospy.Time.now()
        msg.value = value
        msg.angle = angle
        
        rospy.loginfo('Force vector data sent')
        pub = rospy.Publisher('force_vect', vect_msg, queue_size=10)
        while rospy.Time.now()<endtime:
            pub.publish(msg)
            

    
    msg.header.stamp = rospy.Time.now()
    msg.value = value
    msg.angle = angle
    pub = rospy.Publisher('force_vect', vect_msg, queue_size=10)
    pub.publish(msg)
    rospy.loginfo('Force vector data sent')
if  __name__ == '__main__':
    try:
        force()
    except rospy.ROSInterruptException:
        pass