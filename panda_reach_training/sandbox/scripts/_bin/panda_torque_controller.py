#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64


def jointEffortCommandPublisher():

    # Create publisher
    pub_j1 = rospy.Publisher(
        "panda_joint1_controller/command", Float64, queue_size=10
    )
    pub_j2 = rospy.Publisher(
        "panda_joint2_controller/command", Float64, queue_size=10
    )
    pub_j3 = rospy.Publisher(
        "panda_joint3_controller/command", Float64, queue_size=10
    )
    pub_j4 = rospy.Publisher(
        "panda_joint4_controller/command", Float64, queue_size=10
    )
    pub_j5 = rospy.Publisher(
        "panda_joint5_controller/command", Float64, queue_size=10
    )
    pub_j6 = rospy.Publisher(
        "panda_joint6_controller/command", Float64, queue_size=10
    )
    pub_j7 = rospy.Publisher(
        "panda_joint7_controller/command", Float64, queue_size=10
    )
    rospy.init_node("talker", anonymous=True)
    rate = rospy.Rate(950)  # 950hz

    # Create message
    j1_command = Float64()
    j2_command = Float64()
    j3_command = Float64()
    j4_command = Float64()
    j5_command = Float64()
    j6_command = Float64()
    j7_command = Float64()

    # Publish commands till ros is stopped
    while not rospy.is_shutdown():
        j1_command.data = -0.07
        j2_command.data = 70.6
        j3_command.data = 0.49
        j4_command.data = 16
        j5_command.data = 1
        j6_command.data = 1.5
        j7_command.data = 0.00
        pub_j1.publish(j1_command)
        pub_j2.publish(j2_command)
        pub_j3.publish(j3_command)
        pub_j4.publish(j4_command)
        pub_j5.publish(j5_command)
        pub_j6.publish(j6_command)
        pub_j7.publish(j7_command)
        rate.sleep()


if __name__ == "__main__":
    try:
        jointEffortCommandPublisher()
    except rospy.ROSInterruptException:
        pass
