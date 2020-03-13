#!/usr/bin/env python
import rospy

import tf2_ros
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rospy.init_node("tf2_turtle_listener")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("world", "panda_link8", rospy.Time())
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rate.sleep()
            continue

        # Transform trans to pose
        pose = PoseStamped()
        pose.header = trans.header
        pose.pose.orientation = trans.transform.rotation
        pose.pose.position = trans.transform.translation

        # Sleep
        rate.sleep()
