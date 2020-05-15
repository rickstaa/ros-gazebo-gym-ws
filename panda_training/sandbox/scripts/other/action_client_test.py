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

if __name__ == "__main__":

    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node("test_action_client")

    # Create first action client
    client = actionlib.SimpleActionClient(
        "/panda_arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction
    )

    # Waits until the first action server has started up and started
    # listening for goals.
    retval = client.wait_for_server(timeout=rospy.Duration(5))
    if not retval:
        rospy.logerr("Shutting down")
        sys.exit(0)

    # Create second action client
    client2 = actionlib.SimpleActionClient(
        "/panda_hand_controller/follow_joint_trajectory", FollowJointTrajectoryAction
    )

    # Waits until the second action server has started up and started
    # listening for goals.
    retval2 = client2.wait_for_server(timeout=rospy.Duration(5))
    if not retval2:
        rospy.logerr("Shutting down")
        sys.exit(0)

    # Create action client 1 goal
    header = Header()
    # header.stamp = rospy.get_rostime()
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
        0.5,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    point.time_from_start.secs = 1.0
    goal.trajectory.points.append(point)
    # goal.trajectory.header = header
    # goal.goal_time_tolerance.secs = 5

    # Create action client 2 goal
    header = Header()
    # header.stamp = rospy.get_rostime()
    goal2 = FollowJointTrajectoryGoal()
    goal2.trajectory.joint_names = [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ]
    point2 = JointTrajectoryPoint()
    point2.positions = [
        0.5,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    point2.time_from_start.secs = 1.0
    goal2.trajectory.points.append(point2)
    # goal.trajectory.header = header
    # goal.goal_time_tolerance.secs = 5

    # Send action clients goals
    client.send_goal(goal)
    client2.send_goal(goal2)
    client.wait_for_result()
    client2.wait_for_result()
    result = client.get_result()
    state = client.get_state()
    result2 = client.get_result()
    result2 = client.get_state()
    print("jan")
