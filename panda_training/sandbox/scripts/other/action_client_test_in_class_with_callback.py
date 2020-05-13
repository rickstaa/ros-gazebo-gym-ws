#! /usr/bin/env python
from __future__ import print_function
import rospy
import sys
from std_msgs.msg import Header

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


class ActionClientTest(object):
    """Class to test two action clients
    """

    def __init__(self):

        # Create first action client
        self.client = actionlib.SimpleActionClient(
            "/panda_arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction
        )

        # Waits until the first action server has started up and started
        # listening for goals.
        retval = self.client.wait_for_server(timeout=rospy.Duration(5))
        if not retval:
            rospy.logerr("Shutting down")
            sys.exit(0)

        # Create second action client
        self.client2 = actionlib.SimpleActionClient(
            "/panda_hand_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )

        # Waits until the second action server has started up and started
        # listening for goals.
        retval2 = self.client2.wait_for_server(timeout=rospy.Duration(5))
        if not retval2:
            rospy.logerr("Shutting down")
            sys.exit(0)

    def start(self):
        """Start test by sending actions"""

        # Create action client 1 goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ]
        point = JointTrajectoryPoint()
        point.positions = [
            0.05,
            0.05,
            0.05,
            0.05,
            0.05,
            0.05,
            0.05,
        ]
        point.time_from_start.secs = 1.0
        goal.trajectory.points.append(point)

        # Create action client 2 goal
        header = Header()
        goal2 = FollowJointTrajectoryGoal()
        goal2.trajectory.joint_names = [
            "panda_finger_joint1",
            "panda_finger_joint2",
        ]
        point2 = JointTrajectoryPoint()
        point2.positions = [
            0.0,
            0.0,
        ]
        point2.time_from_start.secs = 1.0
        goal2.trajectory.points.append(point2)
        goal2.trajectory.header = header

        # Send action clients goals
        self.client.send_goal(goal, feedback_cb=self.feedback_cb1)
        self.client2.send_goal(goal2, feedback_cb=self.feedback_cb2)
        self.client.wait_for_result()
        self.client2.wait_for_result()
        result = self.client.get_result()
        state = self.client.get_state()
        result2 = self.client2.get_result()
        state2 = self.client2.get_state()
        return result, result2, state, state2

    def feedback_cb1(self, feedback):

        # publish the feedback
        print("jan")

    def feedback_cb2(self, feedback):

        # publish the feedback
        print("jan2")


if __name__ == "__main__":

    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node("test_action_client")

    # Test object uniqueness
    FollowGoal1 = FollowJointTrajectoryGoal()
    FollowGoal2 = FollowJointTrajectoryGoal()

    print(FollowGoal1 is FollowGoal2)

    # Create actionclienttest object
    action_test = ActionClientTest()

    # Start Test
    result, result2, state, state2 = action_test.start()

    # Check result
    print(result)
    print(result2)
    print(state)
    print(state2)
