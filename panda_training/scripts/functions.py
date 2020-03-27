"""Some additional helpfull helper functions for the panda_training package."""

# Main python imports
import copy

# Import ROS python packages
import rospy
from rospy.exceptions import ROSException

# ROS msgs and srvs
from actionlib_msgs.msg import GoalStatusArray
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


#################################################
# Additional functions ##########################
#################################################
def action_server_exists(topic_name):
    """Checks whether a topic contains an action server
    is running.

    Parameters
    ----------
    topic_name : str
        Action server topic name.

    Returns
    -------
    bool
        Bool specifying whether the action service exists.
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


def joint_positions_2_follow_joint_trajectory_goal(joint_positions, time_from_start=1):
    """Converts a dictionary of joint_positions into a FollowJointTrajectoryGoal
        msgs.

        Parameters
        ----------
        joint_positions : dict
            The joint positions of each of the robot joints.
        time_from_start : dict, optional
            The time from the start at which the joint position has to be achieved, by
            default 1 sec.
        """

    # creates a goal to send to the action server
    goal_msg = FollowJointTrajectoryGoal()
    joint_states = JointTrajectoryPoint()
    joint_states.time_from_start.secs = time_from_start
    for joint_name, joint_position in joint_positions.items():
        joint_states.positions.append(joint_position)
        goal_msg.trajectory.joint_names.append(joint_name)
    goal_msg.trajectory.points.append(joint_states)

    # Return goal msgs
    return goal_msg


def controller_list_array_2_dict(controller_list_msgs):
    """Converts a controller_manager list_controllers message into a
    controller information dictionary.

    Parameters
    ----------
    controller_list_msgs : controller_manager_msgs.srv.ListControllersResponse
        Controller_manager/list_controllers service response message.

    Returns
    -------
    dict
        Dictionary containing information about all the available controllers.
    """

    # Create controller_list dictionary
    controller_list_dict = {}
    for controller in controller_list_msgs.controller:
        controller_name = controller.name
        controller_list_dict[controller_name] = copy.deepcopy(controller)

    # Return dictionary
    return controller_list_dict


def flatten_list(input_list):
    """Function used to flatten a list containing sublists. It does this by calling
    itself recursively.

    Parameters
    ----------
    input_list : list of lists
        A list containing strings or other lists.

    Returns
    -------
    list
        The flattened list.
    """

    # Convert list of list to flattened lists
    flattened_list = []
    for list_item in input_list:
        if type(list_item) is list:
            flattened_list.extend(
                flatten_list(list_item)
            )  # NOTE: Calls itself recursively
        else:
            flattened_list.append(list_item)

    # Return flattened list
    return flattened_list


def dict_clean(input_dict):
    """Removes empty dictionary keys from a dictionary and returns a cleaned up
    dictionary. Empty meaning an empty list, string or dict or a None value.

    Parameters
    ----------
    input_dict : dict
        The input dictionary.

    Returns
    -------
    dict
        The cleaned dictionary
    """

    # Strip dictionary from empty keys
    stripped_dict = {}
    for k, v in input_dict.items():
        if isinstance(v, dict):
            v = dict_clean(v)
        if v not in (u"", None, {}, []):
            stripped_dict[k] = v
    return stripped_dict


def lower_first_char(string):
    """De-capitalize the first letter of a string.

    Parameters
    ----------
    string : str
        The input string.

    Returns
    -------
    string
        The de-capitalized string.

    .. note::
        This function is not the exact oposite of the capitalize function of the
        standard library. For example, capitalize('abC') returns Abc rather than AbC.
    """

    if not string:  # Added to handle case where s == None
        return
    else:
        return string[0].lower() + string[1:]


if __name__ == "__main__":

    test_joint_dict = {"panda_joint2": 4, "panda_joint1": 5}
    output = joint_positions_2_follow_joint_trajectory_goal(test_joint_dict)
    print("jan")
