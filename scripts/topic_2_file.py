#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
import message_filters
import time


class SavePoses(object):
    def __init__(self):
        
        self._psides = Pose()
        self._psiref = Twist()
        self._psides.orientation.w = 0
        self._psiref.angular.x = 0
        self.pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
        self.odom_sub   = message_filters.Subscriber('/odom', Odometry)
        self.ref_sub = message_filters.Subscriber('/traj_plann', Odometry)
        ts = message_filters.ApproximateTimeSynchronizer([self.ref_sub,self.odom_sub], queue_size=10, slop=0.5)
        ts.registerCallback(self.callback,self.pub)
        self.write_to_file()

    def callback(self,ref_sub, odom_sub, pub):
        
        self._psides.orientation.w = ref_sub.pose.pose.orientation.w
        self._psiref.angular.x = ref_sub.twist.twist.angular.x
    
    def write_to_file(self):
        file1 = open("/home/iris/catkin_ws/src/myrobot/scripts/psi_des.txt","w+")
        file2 = open("/home/iris/catkin_ws/src/myrobot/scripts/psi_real.txt","w+")
        file3 = open("/home/iris/catkin_ws/src/myrobot/scripts/sim_time.txt","w+")
        file4 = open("/home/iris/catkin_ws/src/myrobot/scripts/x_des.txt","w+")
        file5 = open("/home/iris/catkin_ws/src/myrobot/scripts/x_real.txt","w+")
        while True:
            try:
                now = rospy.get_rostime()
                file1.write("%f\n" % (self._psides.orientation.w))
                file2.write("%f\n" % (self._psiref.angular.x))
                file3.write("%f\n" % (now.secs+(1/1000000000)*now.nsecs))
                time.sleep(0.001)
            except rospy.ROSInterruptException:
                file1.close()
                file2.close()
            
        
        # with open('poses.txt', 'w') as file:
            
        #     for key, value in self.poses_dict.iteritems():
        #         if value:
        #             file.write(str(key) + ':\n----------\n' + str(value) + '\n===========\n')
                    
                rospy.loginfo("Written all Poses to poses.txt file")
        


if __name__ == "__main__":
    rospy.init_node('spot_recorder', log_level=rospy.INFO) 
    save_spots_object = SavePoses()
    #rospy.spin() # mantain the service open.