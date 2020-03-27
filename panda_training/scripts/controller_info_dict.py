"""Class used to store information about the gazebo controllers.
"""

# Main python 2/3 compatibility imports
from __future__ import print_function
from builtins import super

# Script parameters
CONTROLLER_INFO_DICT = {
    "arm": {
        "running": {
            "joint_trajectory_control": [],
            "joint_position_control": [],
            "joint_effort_control": [],
            "joint_group_position_control": [],
            "joint_group_effort_control": [],
        },
        "loaded": {
            "joint_trajectory_control": [],
            "joint_position_control": [],
            "joint_effort_control": [],
            "joint_group_position_control": [],
            "joint_group_effort_control": [],
        },
        "stopped": {
            "joint_trajectory_control": [],
            "joint_position_control": [],
            "joint_effort_control": [],
            "joint_group_position_control": [],
            "joint_group_effort_control": [],
        },
    },
    "hand": {
        "running": {
            "joint_trajectory_control": [],
            "joint_position_control": [],
            "joint_effort_control": [],
            "joint_group_position_control": [],
            "joint_group_effort_control": [],
        },
        "loaded": {
            "joint_trajectory_control": [],
            "joint_position_control": [],
            "joint_effort_control": [],
            "joint_group_position_control": [],
            "joint_group_effort_control": [],
        },
        "stopped": {
            "joint_trajectory_control": [],
            "joint_position_control": [],
            "joint_effort_control": [],
            "joint_group_position_control": [],
            "joint_group_effort_control": [],
        },
    },
    "other": {
        "running": {
            "joint_trajectory_control": [],
            "positio_control": [],
            "joint_effort_control": [],
            "joint_group_position_control": [],
            "joint_group_effort_control": [],
        },
        "loaded": {
            "joint_trajectory_control": [],
            "joint_position_control": [],
            "joint_effort_control": [],
            "joint_group_position_control": [],
            "joint_group_effort_control": [],
        },
        "stopped": {
            "joint_trajectory_control": [],
            "joint_position_control": [],
            "joint_effort_control": [],
            "joint_group_position_control": [],
            "joint_group_effort_control": [],
        },
    },
}


#################################################
# Controller Info Dictionary ####################
#################################################
class ControllerInfoDict(dict):
    """Used for storing information about the gazebo robot controllers.
    This class overloads the normal `dict` class in order to pre-initialize the
    dictionary with the needed keys."""

    def __init__(self, *args, **kwargs):
        """Initiate the ControllerInfoDict"""
        super().__init__(*args, **kwargs)
        super().update(CONTROLLER_INFO_DICT)
