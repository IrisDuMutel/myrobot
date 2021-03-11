#! usr/bin/env python
import rospy
# import time
import tf
from tf import TransformBroadcaster
import tf2_ros
import tf2_msgs.msg
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import message_filters


def definition():
    rospy.init_node('odometry_publisher',anonymous=True)
    # odom_sub   = rospy.Subscriber('/odom', Odometry)
    pub = rospy.Publisher('/odom_setodom',Odometry,queue_size=10)
    odom_sub   = message_filters.Subscriber('/odom', Odometry)
    ref_sub = message_filters.Subscriber('/odom', Odometry)
    ts = message_filters.ApproximateTimeSynchronizer([ref_sub,odom_sub], queue_size=10, slop=2, allow_headerless=True)
    ts.registerCallback(callback,pub)
    rospy.spin()
    

def callback(ref_sub, odom_sub, odom_pub):
    Refer = Odometry()
    Refer = ref_sub
    # Odom = Odometry()
    # Odom = odom_sub
    odom_broadcaster = TransformBroadcaster()
    current_time = rospy.Time.now
    # last_time = rospy.Time.now
    rospy.loginfo('AAAAAAAAAAAA')
    current_time = rospy.Time.now
    # dt = rospy.Time.to_sec(current_time-last_time) 
    # odom_trans = TransformStamped()
    # odom_trans.header.stamp = current_time
    # odom_trans.header.frame_id = "odom"
    # odom_trans.child_frame_id = "base_footprint"

    # odom_trans.transform.translation.x = Refer.pose.pose.position.x
    # odom_trans.transform.translation.y = Refer.pose.pose.position.y
    # odom_trans.transform.translation.z = 0.0
    # odom_trans.transform.rotation = Refer.pose.pose.orientation
    odom_quat = Refer.pose.pose.orientation

    odom_broadcaster.sendTransform(
        (Refer.pose.pose.position.x, Refer.pose.pose.position.y, 0.),
        (Refer.pose.pose.orientation.x,Refer.pose.pose.orientation.y,Refer.pose.pose.orientation.z,Refer.pose.pose.orientation.w),
        current_time,
        "base_footprint",
        "odom"
    )
    # odom_broadcaster.sendTransform(odom_trans)
    #next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    print('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')
    #set the position
    odom.pose.pose.position.x = Refer.pose.pose.position.x
    odom.pose.pose.position.y = Refer.pose.pose.position.y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation = Refer.pose.pose.orientation
    #set the velocity
    odom.child_frame_id = "base_footprint"
    odom.twist.twist.linear.x = Refer.twist.twist.linear.x
    odom.twist.twist.linear.y = Refer.twist.twist.linear.y
    odom.twist.twist.angular.z = Refer.twist.twist.angular.z
    #publish the message
    odom_pub.publish(odom)
    # last_time = current_time


if __name__ == '__main__':
    try:

        definition()
    except rospy.ROSInterruptException:
        pass


