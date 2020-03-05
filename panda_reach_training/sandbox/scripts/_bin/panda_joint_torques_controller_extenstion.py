#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import LinkStates
from tf.transformations import euler_from_quaternion
from math import (cos, sin, pi, atan2)
from std_msgs.msg import Float64


# Initiates Subscriber to gazebo/link_states
def gazebo_link_states():
    rospy.Subscriber("/gazebo/link_states", LinkStates, callback)  # subscribes to /gazebo/link_states of type LinkStates
    rospy.spin()


# Callback from Subscriber for Torque Balance with Added Mass
def callback(message):

    # Obtains 3rd revolute joint position and orientation from gazebo/link_states
    link4pose = message.pose[4]
    link4quaternion = link4pose.orientation
    q = [link4quaternion.x, link4quaternion.y, link4quaternion.z, link4quaternion.w]  # creates list from quaternion since it was not originally
    link4orientation = euler_from_quaternion(q)  # transfrom from quaternion to euler angles

    # Maps the end effector from 3rd revolute joint position and orientation
    end_effector_x = link4pose.position.x + sin(link4orientation[1])
    end_effector_y = link4pose.position.y
    end_effector_z = link4pose.position.z - cos(link4orientation[1])
    end_effector_position = [end_effector_x, end_effector_y, end_effector_z]

    # Proportional controller for torque balance with added mass near the camera
    theta_before_added_object = 21.62 # determined from the supplied torques before added mass
    theta_new = atan2((link4pose.position.z - end_effector_z), (link4pose.position.x - end_effector_x))  # new theta from horizion

    Gain = 0.4  # proportional gain

    torque_before_added_object = 9.81 #constant value was applied
    torque_new = (4.1 * torque_before_added_object / 9.81) * Gain * ((theta_new / pi*180) - theta_before_added_object) #Torque = (T_before)*K*(delta_theta)

    # Re-initializing publishers from before since in a different function
    pub1 = rospy.Publisher('/rrrbot/joint1_torque_controller/command', Float64, queue_size=100)
    pub2 = rospy.Publisher('/rrrbot/joint2_torque_controller/command', Float64, queue_size=100)
    pub3 = rospy.Publisher('/rrrbot/joint3_torque_controller/command', Float64, queue_size=100)

    # Publishing new torques to topics
    pub1.publish(2.5 * torque_before_added_object)  # no difference
    pub2.publish(1.2 * torque_before_added_object)  # no difference
    pub3.publish((4.1 * torque_before_added_object / 9.81) + torque_new)  # torque_before + torque_new

    # Debugging info - difference in theta & torque_new
    # print "Theta Diff: ", (theta_new / pi*180 - theta_before_added_object), "\t", "New Torque: ", torque_new
