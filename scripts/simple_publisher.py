#!/usr/bin/env python

import rospy
import time
import math
from std_msgs.msg import Float32


def simple_publisher():
    pub = rospy.Publisher('motor/voltage_norm', Float32, queue_size=10)
    rospy.init_node('simple_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    v = 0
    i = 0
    while not rospy.is_shutdown():
        if v == 0 and i == 2:
            v=0.2
            i = 0
        elif v==0.2 and i == 2:
            v = 0
            i = 0
        voltage = v
        rospy.loginfo(voltage)
        pub.publish(voltage)
        i+=1
        rate.sleep()

if __name__ == '__main__':
    try:
        simple_publisher()
    except rospy.ROSInterruptException:
        pass