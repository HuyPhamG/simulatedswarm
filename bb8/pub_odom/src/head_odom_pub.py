#!/usr/bin/env python

import rospy
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion

odom_pub = rospy.Publisher('/odom_good', Odometry, queue_size=1)

if __name__ == '__main__':
    rospy.init_node('bb8_pub_good_odom')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/odom', '/head_link', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        current_time = rospy.Time.now()

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(trans[0], trans[1], trans[2]), Quaternion(rot[0], rot[1], rot[2], rot[3]))

        # set the velocity
        #odom.child_frame_id = "base_link"
        #odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        odom_pub.publish(odom)

        last_time = current_time
        rate.sleep()