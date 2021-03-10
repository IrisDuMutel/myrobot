#! usr/bin/env python
import rospy
import time
from tf import TransformBroadcaster
import tf2_ros
import tf2_msgs.msg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import message_filters


def definition():
    rospy.init_node('odometry_publisher',anonymous=False)
    odom_pub = rospy.Publisher('odom',Odometry,queue_size=10)
    odom_sub   = message_filters.Subscriber('/odom_diffdrive', Odometry)
    ref_sub = message_filters.Subscriber('/simulink_odometry', Twist)
    ts = message_filters.ApproximateTimeSynchronizer([ref_sub,odom_sub], queue_size=10, slop=0.5)
    ts.registerCallback(callback,odom_pub)
    rospy.spin()
    

def callback(ref_sub, odom_sub, odom_pub):

    odom_broadcaster = TransformBroadcaster()
    current_time = rospy.Time.now
    # last_time = rospy.Time.now

    current_time = rospy.Time.now
    # dt = rospy.Time.to_sec(current_time-last_time) 
    odom_trans = TransformStamped()
    odom_trans.header.stamp = current_time
    odom_trans.header.frame_id = "odom"
    odom_trans.child_frame_id = "base_footprint"

    odom_trans.transform.translation.x = ref_sub.pose.pose.position.x
    odom_trans.transform.translation.y = ref_sub.pose.pose.position.y
    odom_trans.transform.translation.z = 0.0
    odom_trans.transform.rotation = ref_sub.pose.pose.orientation

    odom_broadcaster.sendTransform(odom_trans)
    #next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    #set the position
    odom.pose.pose.position.x = ref_sub.pose.pose.position.x
    odom.pose.pose.position.y = ref_sub.pose.pose.position.y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation = ref_sub.pose.pose.orientation
    #set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist.linear.x = ref_sub.twist.twist.linear.x
    odom.twist.twist.linear.y = ref_sub.twist.twist.linear.y
    odom.twist.twist.angular.z = ref_sub.twist.twist.angular.z
    #publish the message
    odom_pub.publish(odom)
    last_time = current_time


if __name__ == '__main__':
    try:

        definition()
    except rospy.ROSInterruptException:
        pass


