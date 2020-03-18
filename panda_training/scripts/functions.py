"""Some additional helpfull helper functions for the panda_training package."""

# Main python imports
import copy

# Import ROS python packages
import rospy
from rospy.exceptions import ROSException

# ROS msgs and srvs
from actionlib_msgs.msg import GoalStatusArray


#################################################
# Additional functions ##########################
#################################################
def action_server_exists(topic_name):
    """Checks whether a topic contains an action server
    is running.

    Parameters
    ----------
    topic_name : string
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
