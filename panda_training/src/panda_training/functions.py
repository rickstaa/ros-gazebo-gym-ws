"""Some additional helpfull helper functions for the panda_training package.
"""

# Main python imports
import copy
from collections import OrderedDict

# Import ROS python packages
import rospy
from rospy.exceptions import ROSException
from tf.transformations import euler_from_quaternion
from panda_training.extras import EulerAngles

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


def model_state_msg_2_link_state_dict(link_state_msgs):
    """Converts the a gazebo_msgs/ModelState message into a panda_state dictionary.
    Contrary to the original ModelState message, in the model_state dictionary the
    poses and twists are grouped per link/model.

    Parameters
    ----------
    link_state_msgs : gazebo_msgs.msg.ModelState
        A ModelState message.

    Returns
    -------
    dict
        A panda_training model_state dictionary.
    """

    # Create controller_list dictionary
    model_state_dict = {}
    for (joint_name, position, twist) in zip(
        link_state_msgs.name, link_state_msgs.pose, link_state_msgs.twist
    ):
        model_state_dict[joint_name] = {}
        model_state_dict[joint_name]["pose"] = copy.deepcopy(position)
        model_state_dict[joint_name]["twist"] = copy.deepcopy(twist)

    # Return dictionary
    return model_state_dict


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
        This function is not the exact opposite of the capitalize function of the
        standard library. For example, capitalize('abC') returns Abc rather than AbC.
    """

    if not string:  # Added to handle case where s == None
        return
    else:
        return string[0].lower() + string[1:]


def get_unique_list(input_list):
    """Removes non-unique items from a list

    Parameters
    ----------
    list : list
        The input list.

    Returns
    -------
    list
        The new list containing only unique items.
    """

    return list({item for item in input_list})


def get_duplicate_list(input_list):
    """Returns the duplicates in a list.

    Parameters
    ----------
    list : list
        The input list.

    Returns
    -------
    list
        The new list containing only the itesm that had duplicates.
    """

    return list(set([x for x in input_list if input_list.count(x) > 1]))


def get_orientation_euler(quaternion):
    """Converts pose (position, orientation) to euler angles.

    Parameters
    ----------
    quaternion : geometry_msgs.Pose
        Input quaternion

    Returns
    -------
    panda_training.EulerAngles
        Object containing the yaw (z), pitch (y) and roll (z) euler angles.
    """

    # Convert quaternion to euler
    orientation_list = [
        quaternion.orientation.x,
        quaternion.orientation.y,
        quaternion.orientation.z,
        quaternion.orientation.w,
    ]
    euler_resp = euler_from_quaternion(orientation_list, "rzyx")

    # Convert list to euler object
    euler = EulerAngles()
    euler.y = euler_resp[0]
    euler.p = euler_resp[1]
    euler.r = euler_resp[2]
    return euler


def action_list_2_action_dict(actions, joints):
    """Covert a list of joint actions to a action dictionary {joint: action}.

    Parameters
    ----------
    actions : list
        List containing a control action for each of the panda joints.
    joints : list
        List containing the robot joints.

    Returns
    -------
    dict
        Dictionary containing a control action for each of the panda joints.
    """

    action_dict = OrderedDict(zip(joints, actions))
    return action_dict


def translate_actionclient_result_error_code(actionclient_retval):
    """Translates the error code returned by the SimpleActionClient.get_result()
    function into a human readable error message.

    Parameters
    ----------
    actionclient_retval : control_msgs.msg.FollowJointTrajectoryResult
        The result that is returned by the
        actionlib.simple_action_client.SimpleActionClient.get_result() function.

    Returns
    -------
    str
        Error string that corresponds to the error code.
    """

    # Create error dictionary
    error_dict = {
        value: attr
        for attr, value in actionclient_retval.__class__.__dict__.items()
        if attr[0] != "_" and all(map(str.isupper, attr.replace("_", "")))
    }

    # Create error code message
    return (
        error_dict[actionclient_retval.error_code]
        .lower()
        .capitalize()
        .replace("_", " ")
        + "."
        if error_dict[actionclient_retval.error_code] != "SUCCESSFUL"
        else ""
    )


def list_2_human_text(input_list):
    """Function converts a list of values into human readable sentence.

    Example:
        Using this function a list of 4 items '[item1, item2, item3, item4]' becomes
        'item2, item3 and item4'.

    Parameters
    ----------
    input_list : list
        A input list.

    Returns
    -------
    str
        A human readable string that can be printed.
    """

    # Create human readable comma deliminated text
    if isinstance(input_list, list):
        if len(input_list) > 1:
            return ", ".join(input_list[:-1]) + " & " + input_list[-1]
        if len(input_list) == 0:
            return ""
        else:
            return str(input_list[0])
    else:
        return input_list
