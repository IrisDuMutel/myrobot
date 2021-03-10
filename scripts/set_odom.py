#! usr/bin/env python
import rospy
import time
from tf import TransformBroadcaster
import tf2_ros
import tf2_msgs.msg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

def definition():
    rospy.init_node('odometry_publisher',anonymous=False)
    odom_pub = rospy.Publisher('odom',Odometry,queue_size=10)
    rospy.Subscriber('/simulink_odometry', Odometry, callback)
    rospy.spin()

def callback(data):
    odom_broadcaster = TransformBroadcaster()
    odom_pub = rospy.Publisher('odom',Odometry,queue_size=10)

    current_time = rospy.Time.now
    last_time = rospy.Time.now

    current_time = rospy.Time.now
    dt = rospy.Time.to_sec(current_time-last_time) 
    odom_trans = TransformStamped()
    odom_trans.header.stamp = current_time
    odom_trans.header.frame_id = "odom"
    odom_trans.child_frame_id = "base_link"

    odom_trans.transform.translation.x = data.pose.pose.position.x
    odom_trans.transform.translation.y = data.pose.pose.position.y
    odom_trans.transform.translation.z = 0.0
    odom_trans.transform.rotation = data.pose.pose.orientation

    odom_broadcaster.sendTransform(odom_trans)
    #next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    #set the position
    odom.pose.pose.position.x = data.pose.pose.position.x
    odom.pose.pose.position.y = data.pose.pose.position.y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation = data.pose.pose.orientation
    #set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist.linear.x = data.twist.twist.linear.x
    odom.twist.twist.linear.y = data.twist.twist.linear.y
    odom.twist.twist.angular.z = data.twist.twist.angular.z
    #publish the message
    odom_pub.publish(odom)
    last_time = current_time


if __name__ == '__main__':
    try:

        definition()
    except rospy.ROSInterruptException:
        pass


