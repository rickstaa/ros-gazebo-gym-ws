#!/usr/bin/env python
import rospy
from rospy.exceptions import ROSException
from sensor_msgs.msg import JointState


def _joints_callback(data):
    pass


if __name__ == "__main__":
    rospy.init_node("tf2_turtle_listener")

    # Create Needed subscribers
    JOINT_STATES_SUBSCRIBER = "/joint_states"
    joints = JointState()

    # Test joint message
    try:
        joints = rospy.wait_for_message("/joint_states", JointState, timeout=1.0)
        rospy.logdebug("Current /joint_states READY=>" + str(joints))

    except ROSException as e:  # TODO: Fix exception type
        rospy.logerr(
            "Current /joint_states not ready yet, retrying for getting " "joint_states"
        )

    # Create subscriber
    joint_states_sub = rospy.Subscriber(
        JOINT_STATES_SUBSCRIBER, JointState, _joints_callback
    )
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        # Sleep
        rate.sleep()
