#!/usr/bin/env python

import rospy
import math
import random
from gazebo_msgs.msg import ContactsState
from myrobot.msg import vect_msg
import message_filters

msg = vect_msg()
def contact():
    rospy.init_node('contact',anonymous=True) # Create goal node called 'trajplan'
    rospy.loginfo("Status: Contact Sensors Initialized") 
    # Subscribe to messages
    front_sub = message_filters.Subscriber('/contact/front_sensor', ContactsState)
    back_sub = message_filters.Subscriber('/contact/back_sensor', ContactsState)
    # Syncronize
    ts = message_filters.TimeSynchronizer([front_sub,back_sub], queue_size=10)
    # Register callback and publisher
    ts.registerCallback(callback)
    rospy.spin()

def callback(front_sub,back_sub):
    msg = vect_msg()
    #TODO define here a behavior according to the contact value
    angle = 0
    if not front_sub.states:
        pass
    else:
        front_force = front_sub.states[0].total_wrench.force.x
        rospy.loginfo('Ive been toched at the front F=%s\n', front_force)
        # Fill vector message
        msg.angle = angle
        msg.value = light_val 
    if not back_sub.states:
        pass
    else:
        back_force = back_sub.states[0].total_wrench.force.x
        rospy.loginfo('Ive been toched at the back F=%s\n', back_force) 
        # Fill vector message
        msg.angle = angle
        msg.value = light_val 

    msg.header.stamp = rospy.Time.now()
    pub = rospy.Publisher('contact_vect', vect_msg, queue_size=10)
    pub.publish(msg)
    rospy.loginfo('Contact vector sent')
if  __name__ == '__main__':
    try:
        contact()
    except rospy.ROSInterruptException:
        pass