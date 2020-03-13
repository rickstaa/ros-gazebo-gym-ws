#! /usr/bin/env python

import rospy
import time
import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)
from trajectory_msgs.msg import JointTrajectoryPoint

"""
class SimpleGoalState:
    PENDING    = 0   # The goal has yet to be processed by the action server
    ACTIVE     = 1   # The goal is currently being processed by the action server
    PREEMPTED  = 2   # The goal received a cancel request after it started executing
                     #   and has since completed its execution (Terminal State)
    SUCCEEDED  = 3   # The goal was achieved successfully by the action server (Terminal State)
    ABORTED    = 4   # The goal was aborted during execution by the action server due
                     #    to some failure (Terminal State)
    REJECTED   = 5   # The goal was rejected by the action server without being processed,
                     #    because the goal was unattainable or invalid (Terminal State)
    PREEMPTING = 6   # The goal received a cancel request after it started executing
                     #    and has not yet completed execution
    RECALLING  = 7   # The goal received a cancel request before it started executing,
                     #    but the action server has not yet confirmed that the goal is canceled
    RECALLED   = 8   # The goal received a cancel request before it started executing
                     #    and was successfully cancelled (Terminal State)
    LOST       = 9   # An action client can determine that a goal is LOST. This should not be
                     #    sent over the wire by an action server
"""
# We create some constants with the corresponing vaules from the SimpleGoalState class
PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

nImage = 1

# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received
def feedback_callback(feedback):
    """
    Error that might jump

    self._feedback.lastImage =
AttributeError: 'ArdroneAS' obj

    """
    pass
    rospy.loginfo("Robot Being Controlled")
    rospy.loginfo(feedback)


# initializes the action client node
rospy.init_node("joint_trajectory_action_test")

# Connect to robot control action server
control_action_server_name = "panda_arm_controller/follow_joint_trajectory/"
control_action_client = actionlib.SimpleActionClient(
    control_action_server_name, FollowJointTrajectoryAction
)

# waits until the action server is up and running
rospy.loginfo("Waiting for action Server " + control_action_server_name)
control_action_client.wait_for_server()
rospy.loginfo("Action Server Found..." + control_action_server_name)

# creates a goal to send to the action server
goal_msg = FollowJointTrajectoryGoal()
joint_states = JointTrajectoryPoint()
joint_states.time_from_start.secs = 1  # Time from start
joint_states.positions = [0, 0, 0, 0, 0, 0, 0]
goal_msg.trajectory.joint_names = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
]
goal_msg.trajectory.points.append(joint_states)
control_action_client.send_goal(goal_msg, feedback_cb=feedback_callback)

# You can access the SimpleAction Variable "simple_state", that will be 1 if active, and 2 when finished.
# Its a variable, better use a function like get_state.
# state = client.simple_state
# state_result will give the FINAL STATE. Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR
state_result = control_action_client.get_state()

rate = rospy.Rate(1)

rospy.loginfo("state_result: " + str(state_result))

while state_result < DONE:
    rospy.loginfo("Doing Stuff while waiting for the Server to give a result....")
    rate.sleep()
    state_result = control_action_client.get_state()
    rospy.loginfo("state_result: " + str(state_result))

rospy.loginfo("[Result] State: " + str(state_result))
if state_result == ERROR:
    rospy.logerr("Something went wrong in the Server Side")
if state_result == WARN:
    rospy.logwarn("There is a warning in the Server Side")

rospy.loginfo("[Result] State: " + str(control_action_client.get_result()))

