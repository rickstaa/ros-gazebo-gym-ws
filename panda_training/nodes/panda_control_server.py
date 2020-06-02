#!/usr/bin/env python
"""This node sets up a number of services that can be used to control
the Panda Emika Franka robot using the Moveit framework.
"""

# Import ROS packages
import rospy

# Panda_autograsp modules, msgs and srvs
from panda_training.core import PandaControlServer


#################################################
# Main script####################################
#################################################
if __name__ == "__main__":

    # Initiate ROS node
    rospy.init_node("panda_control_server")

    # Get ROS parameters
    try:  # Check end effector
        use_group_controller = rospy.get_param("~use_group_controller")
    except KeyError:
        use_group_controller = False
    try:  # Auto fill joint traj position field if left empty
        autofill_traj_positions = rospy.get_param("~autofill_traj_positions")
    except KeyError:
        autofill_traj_positions = False

    # Start control server
    control_server = PandaControlServer(
        use_group_controller=use_group_controller,
        autofill_traj_positions=autofill_traj_positions,
    )
    rospy.spin()  # Maintain the service open