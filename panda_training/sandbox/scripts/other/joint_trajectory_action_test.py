#! /usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction


class PandaTrajAction(object):
    def __init__(self, name):

        # Connect to panda joint trajectory controller action server
        _action_server_name = "panda_hand_controller/follow_joint_trajectory/"
        self._action_client = actionlib.SimpleActionClient(
            _action_server_name, FollowJointTrajectoryAction
        )

        # waits until the action server is up and running
        rospy.loginfo(
            "Waiting for panda joint trajectory action Server " + _action_server_name
        )
        self._action_client.wait_for_server()
        rospy.loginfo("Action Server Found..." + _action_server_name)

        # Setup panda joint traj action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            FollowJointTrajectoryAction,
            execute_cb=self._execute_cb,
            auto_start=False,
        )
        self._as.register_preempt_callback(self.preempt_cb)
        self._as.start()

    def _execute_cb(self, goal):

        # append the seeds for the fibonacci sequence
        self._action_client.send_goal(goal, feedback_cb=self._feedback_cb)

        # Waits for the server to finish performing the action.
        self._action_client.wait_for_result()

        # Get result
        self._result = self._action_client.get_result()
        self._result2 = self._action_client.get_state()
        if self._result == self._result.SUCCESSFUL:
            self._as.set_succeeded(self._result)
        else:
            self._as.set_aborted(self._result)

    def _feedback_cb(self, feedback):
        """
        Error that might jump

        self._feedback.lastImage =
        AttributeError: 'ArdroneAS' obj

        """

        # publish the feedback
        self._as.publish_feedback(feedback)

    def preempt_cb(self):

        # Stop action server
        self._action_client.cancel_goal()
        self._as.set_preempted()


if __name__ == "__main__":
    rospy.init_node("test_panda_traj_action")
    server = PandaTrajAction(rospy.get_name())
    rospy.spin()
