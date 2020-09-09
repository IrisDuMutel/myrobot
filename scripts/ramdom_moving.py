#!/usr/bin/env python
import rospy
import math
import random
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import *
from datetime import datetime
rospy.init_node('random_movement')

def get_state_client():
    ##rospy.wait_for_service('GetModelState')
    try:
        get_serv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model_state1 = get_serv('model','cube_base')
        ##print('Moving cube')
        ##(model_state)
        return model_state1

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
def set_state_client(future_odom):
      try:
          set_serv = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
          result = set_serv(future_odom)
      except rospy.ServiceException as e:
        rospy.loginfo("Get Model State service call failed:  {0}".format(e))    


if __name__ == "__main__":
    current_odom = get_state_client()
    future_odom = SetModelStateRequest()
    try:
        while not rospy.is_shutdown():
            future_odom.model_state.model_name='model'
            future_odom.model_state.twist.linear.x = 0
            future_odom.model_state.twist.linear.y = 0
            future_odom.model_state.twist.linear.z = 0
            future_odom.model_state.twist.angular.x = 0
            future_odom.model_state.twist.angular.y = 0
            future_odom.model_state.twist.angular.z = 0
            future_odom.model_state.pose.position.x = 1
            future_odom.model_state.pose.position.y = 1
            future_odom.model_state.pose.position.z = 0
            future_odom.model_state.pose.orientation.w = 0
            future_odom.model_state.pose.orientation.x = 0
            future_odom.model_state.pose.orientation.y = 0
            future_odom.model_state.pose.orientation.z =  0
            set_state_client(future_odom)
            get_state_client()
    except rospy.ROSInterruptException:
        pass
