#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray


def jointEffortCommandPublisher():

    # Initiate ros node
    rospy.init_node("panda_joint_torques_controller", anonymous=True)

    # Create publisher
    pub = rospy.Publisher(
        "panda_arm_controller/command", Float64MultiArray, queue_size=100
    )
    rate = rospy.Rate(100)  # 950hz

    # Create joint_control message
    joint_effort_command_array = Float64MultiArray()

    # While loop to have joints follow a certain position, while rospy is not shutdown.
    while not rospy.is_shutdown():
        joint_effort_command_array.data = [-0.07, -30.6, 0.49, 16, 1, 1.5, 0.00]
        rospy.loginfo(joint_effort_command_array)
        pub.publish(joint_effort_command_array)
        rate.sleep()


if __name__ == "__main__":
    try:
        jointEffortCommandPublisher()
    except rospy.ROSInterruptException:
        pass
