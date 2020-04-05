"""Script used to create and test the gazebo model_states 2 model_states dict
convertion function"""

# Main python imports
import copy

# ROS python imports
import rospy

# ROS msgs and srvs
from gazebo_msgs.msg import ModelStates


# Model states conversion function
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
        model_state_dict[joint_name]["position"] = copy.deepcopy(position)
        model_state_dict[joint_name]["twist"] = copy.deepcopy(twist)

    # Return dictionary
    return model_state_dict


# Model state subscriber callback function
def link_states_callback(data):
    """Callback function for retrieving the link_state
    data from gazebo.
    """

    # Convert Gazebo link_states msgs to a link_states dictionary
    model_states_dict = model_state_msg_2_link_state_dict(data)
    print(model_states_dict)


# Main function
if __name__ == "__main__":

    # Initiate ros node
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

    # Create joint_states subscriber
    rospy.loginfo("Setting up sensor data subscribers.")
    joint_states_topic = "/gazebo/link_states"
    # joint_states_topic = "/gazebo/model_states"
    link_states = ModelStates()
    link_states_sub = rospy.Subscriber(
        joint_states_topic, ModelStates, link_states_callback
    )

    # Spin till rosnode is stoped
    rospy.spin()  # Maintain the service open.
