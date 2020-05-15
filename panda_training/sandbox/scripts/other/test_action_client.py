from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header
import actionlib
from actionlib_msgs.msg import GoalStatusArray
import rospy
from rospy.exceptions import ROSException
import sys


def action_server_exists(topic_name):
    """Checks whether a topic contains an action server
    is running.

    Parameters
    ----------
    topic_name : str
        Action server topic name.
    """

    # Strip action server specific topics from topic name
    if topic_name.split("/")[-1] in ["cancel", "feedback", "goal", "result", "status"]:
        topic_name = "/".join(topic_name.split("/")[:-1])

    # Remove trailing /
    if topic_name[-1] == "/":
        topic_name = topic_name[:-1]

    # Validate if action server topic exists
    try:
        rospy.wait_for_message("%s/status" % topic_name, GoalStatusArray, timeout=5)
    except ROSException:
        return False

    # Check if topic contains action client
    exists = False
    for item in rospy.get_published_topics():
        if "%s/status" % topic_name in item[0]:
            if "actionlib_msgs" in item[1]:
                exists = True
            else:
                exists = False
    return exists


if __name__ == "__main__":

    rospy.init_node("test")
    
    # Validate if action client exists
    # retval = action_server_exists("/panda_arm_controller/follow_joint_trajectory/")

    # Connect to robot control action server
    robot_name_space = ""
    _arm_control_action_server_timeout = rospy.Duration(secs=6)
    ARM_CONTROL_ACTION_SERVER_NAME = (
        robot_name_space + "/panda_arm_controller/follow_joint_trajectory"
    )
    arm_control_action_client = actionlib.SimpleActionClient(
        ARM_CONTROL_ACTION_SERVER_NAME, FollowJointTrajectoryAction
    )

    # Waits until the action server has started up
    retval = arm_control_action_client.wait_for_server(timeout=rospy.Duration(5))
    if not retval:
        rospy.logerr(
            "Shutting down '%s' because action server was not started "
            "within the set startup duration %s. Please check whether "
            "the '%s' is functioning correctly."
            % (
                rospy.get_name(),
                _arm_control_action_server_timeout.to_sec(),
                ARM_CONTROL_ACTION_SERVER_NAME,
            )
        )
        sys.exit(0)

    # Create action client goal
    header = Header()
    header.stamp = rospy.get_rostime()
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
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.5,
    ]
    point.time_from_start.secs = 1.0
    goal.trajectory.points.append(point)
    # goal.trajectory.header = header

    # Send goal
    arm_control_action_client.send_goal(goal)
    arm_control_action_client.wait_for_result()
    result = arm_control_action_client.get_result()
    result2 = arm_control_action_client.get_state()
    print("kjan")
