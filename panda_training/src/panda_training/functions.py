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
from panda_training.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from panda_training.srv import SetJointPositionsRequest


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
    joint_positions : dict or panda_training.msg.SetJointPositionsRequest
        Dictionary or message containing the joint positions of each of the robot
        joints.
    time_from_start : dict, optional
        The time from the start at which the joint position has to be achieved, by
        default 1 sec.

    Returns
    -------
    panda_training.msg.FollowJointTrajectoryGoal
        New FollowJointTrajectoryGoal message.
    """

    # Initiate waypoints and new trajectory message
    goal_msg = FollowJointTrajectoryGoal()
    waypoint = JointTrajectoryPoint()
    waypoint.time_from_start.secs = time_from_start

    # creates waypoint from joint_posisitions
    if isinstance(joint_positions, SetJointPositionsRequest):
        waypoint.positions = joint_positions.joint_positions
        goal_msg.trajectory.joint_names = joint_positions.joint_names
    elif isinstance(joint_positions, dict):
        waypoint.positions = list(joint_positions.values())
        goal_msg.trajectory.joint_names = list(joint_positions.keys())
    else:
        TypeError(
            "FollowJointTrajectory message could not be created since the "
            "joint_positions argument has the %s type while the "
            "'joint_positions_2_follow_joint_trajectory_goal' function only accepts "
            "a dictionary or a SetJointPositions message." % type(joint_positions)
        )

    # Add waypoint to trajectory message
    goal_msg.trajectory.points.append(waypoint)

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


def wrap_space_around(text):
    """Wrap one additional space around text if it is not already present.

    Parameters
    ----------
    text : str
        Text

    Returns
    -------
    str
        Text with extra spaces around it.
    """
    if text[0] != " " and text[-1] != " ":
        return " " + text + " "
    elif text[0] != " ":
        return " " + text
    elif text[-1] != " ":
        return text + " "
    else:
        return text


def list_2_human_text(input_list, seperator=",", end_seperator="&"):
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

    # Add spaces around seperators if not present
    seperator = wrap_space_around(seperator)[1:]
    end_seperator = wrap_space_around(end_seperator)

    # Create human readable comma deliminated text
    if isinstance(input_list, list):
        if len(input_list) > 1:
            return (
                seperator.join([str(item) for item in input_list[:-1]])
                + end_seperator
                + str(input_list[-1])
            )
        if len(input_list) == 0:
            return ""
        else:
            return str(input_list[0])
    if isinstance(input_list, tuple):
        input_list = list(input_list)
        if len(input_list) > 1:
            return (
                seperator.join([str(item) for item in input_list[:-1]])
                + end_seperator
                + str(input_list[-1])
            )
        if len(input_list) == 0:
            return ""
        else:
            return str(input_list[0])
    else:
        return input_list


def has_invalid_type(variable, variable_types, items_types=None, depth=0):
    """Validates whether a variable or its attributes has an invalid type.

    Parameters
    ----------
    variable :
        The variable you want to check.
    variable_types : tuple
        The type the variable can have.
    items_types : tuple
        The types the dictionary or list values can have.

    Returns
    --------
    bool, depth, invalid_type
        A tuple containing whether the variable has an invalid type, the depth at which
        the type was invalid, and the type that was invalid.
    """

    # If one type was given make tuple
    if isinstance(variable_types, type):
        variable_types = (variable_types,)
    if isinstance(items_types, type):
        items_types = (items_types,)

    # Check variable type
    if type(variable) in variable_types:

        # Check list or dictionary values
        if items_types:
            if isinstance(variable, dict):

                # Check if the dictionary values are of the right type
                depth += 1
                for key, val in variable.items():
                    if type(val) in [dict, list]:
                        retval, depth, invalid_type = has_invalid_type(
                            val,
                            variable_types=items_types,
                            items_types=items_types,
                            depth=depth,
                        )
                    else:
                        retval, depth, invalid_type = has_invalid_type(
                            val, variable_types=items_types, depth=depth
                        )
                    if retval:
                        return retval, depth, type(val)
            elif isinstance(variable, list):

                # Check if the list values are of the right type
                depth += 1
                for val in variable:
                    if type(val) in [dict, list]:
                        retval, depth, invalid_type = has_invalid_type(
                            val,
                            variable_types=items_types,
                            items_types=items_types,
                            depth=depth,
                        )
                    else:
                        retval, depth, invalid_type = has_invalid_type(
                            val, variable_types=items_types, depth=depth
                        )
                    if retval:
                        return retval, depth, type(val)
    else:
        return True, depth, type(variable)

    # Return successbool if false was not returned
    return False, depth, []


def contains_keys(input_dict, required_keys, exclusive=True):
    """Function used to check if a dictionary contains keys. If 'required_keys' contains
    a nested list it checks whether at least of the nested_list elements is found.

    Parameters
    ----------
    input_dict : dict
        The input dictionary.
    required_keys : list
        List containing the keys you want to check.
    exclusive : bool, optional
        Whether the dictionary can contain other keys than those in the 'required_keys'
        argument, by default False.

    Returns
    -------
    bool, list, list
        A bool specifying whether the keys are present, a list containing the keys that
        were missing and a list containing the keys that were found in addition to the
        required keys.
    """

    # Check if dictionary contains missing keys
    missing_keys = []
    for key in required_keys:

        # If nested list check if one of the keys is present
        if isinstance(key, list):
            found_keys = [
                nested_key
                for nested_key in key
                if nested_key in list(input_dict.keys())
            ]
            if len(found_keys) == 0:
                missing_keys.extend(key)
        else:
            if key not in list(input_dict.keys()):
                missing_keys.append(key)

    # Check if the dictionary contains extra keys
    extra_keys = [
        key for key in input_dict.keys() if key not in flatten_list(required_keys)
    ]

    # Return result
    if len(missing_keys) > 0:
        return False, missing_keys, extra_keys
    elif exclusive and len(extra_keys) > 0:
        return False, missing_keys, extra_keys
    else:
        return True, missing_keys, extra_keys


def has_invalid_value(variable, valid_values):
    """Checks whether a string or list contains invalid values.

    Parameters
    ----------
    variable : list, str
        Input variable
    valid_values : list
        The values that are correct.

    Returns
    -------
    bool, list
        A bool specifying whether invalid values were found and a list that contains
        the invalid values if they were found.
    """

    # Check if list/string contains/has (a) valid value.
    if isinstance(variable, str):
        if variable not in valid_values:
            return True, variable
    elif isinstance(variable, list):
        invalid_values = [item for item in variable if item not in valid_values]
        if len(invalid_values) > 0:
            return True, invalid_values
    else:
        rospy.logwarn(
            "Variable could not be checked for invalid values as type '%s' "
            "is not supported." % type(variable)
        )
        return False, variable

    # Return success bool
    return False, []
