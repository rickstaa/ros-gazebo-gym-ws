#! /usr/bin/env python
"""A simple server for sending control commands to the controllers
of the Panda robot."""

# Main python imports
import sys
import numpy as np
from functions import (
    controller_list_array_2_dict,
    lower_first_char,
    flatten_list,
    dict_clean,
    get_duplicate_list,
)
from panda_exceptions import InputMessageInvalid
from group_publisher import GroupPublisher
from collections import OrderedDict
from itertools import compress
import copy

# ROS python imports
import rospy
from rospy.exceptions import ROSException, ROSInterruptException

# ROS msgs and srvs
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from panda_training.srv import (
    SetJointPositions,
    SetJointPositionsResponse,
    SetJointEfforts,
    SetJointEffortsResponse,
    ListControlType,
    ListControlTypeResponse,
    SwitchControlType,
    SwitchControlTypeResponse,
)
from controller_manager_msgs.srv import ListControllers, ListControllersRequest

# Script variables
ARM_POSITION_CONTROLLERS = [
    "panda_arm_joint1_position_controller",
    "panda_arm_joint2_position_controller",
    "panda_arm_joint3_position_controller",
    "panda_arm_joint4_position_controller",
    "panda_arm_joint5_position_controller",
    "panda_arm_joint6_position_controller",
    "panda_arm_joint7_position_controller",
]
ARM_POSITION_GROUP_CONTROLLERS = ["panda_arm_joint_group_position_controller"]
ARM_EFFORT_CONTROLLERS = [
    "panda_arm_joint1_effort_controller",
    "panda_arm_joint2_effort_controller",
    "panda_arm_joint3_effort_controller",
    "panda_arm_joint4_effort_controller",
    "panda_arm_joint5_effort_controller",
    "panda_arm_joint6_effort_controller",
    "panda_arm_joint7_effort_controller",
]
ARM_EFFORT_GROUP_CONTROLLERS = ["panda_arm_joint_group_effort_controller"]
HAND_POSITION_CONTROLLERS = [
    "panda_hand_finger1_position_controller",
    "panda_hand_finger2_position_controller",
]
HAND_POSITION_GROUP_CONTROLLERS = ["panda_hand_joint_group_position_controller"]
HAND_EFFORT_CONTROLLERS = [
    "panda_hand_finger1_effort_controller",
    "panda_hand_finger2_effort_controller",
]
HAND_EFFORT_GROUP_CONTROLLERS = ["panda_hand_joint_group_effort_controller"]

# TODO: Add action server wrapper for sending a arm and hand joint at the same time.


#################################################
# Joint Group Position Controller class #########
#################################################
class PandaControlServer(object):
    """Controller server used to send control commands to the simulated Panda Robot.

    Attributes
    ----------
    joints : sensor_msgs.JointState
        The current joint states.
    joint_positions_setpoint : dict
        Dictionary containing the last Panda arm and hand positions setpoint.
    joint_efforts_setpoint : dict
        Dictionary containing the last Panda arm and hand efforts setpoint.
    joint_positions_threshold : int
        The current threshold for determining whether the joint positions are within
        the given setpoint.
    joint_efforts_threshold : int
        Threshold for determining whether the joint efforts are within
        the given setpoint.
    use_group_controller : bool
        Whether we are using the group controllers or controlling the individual joints.
    arm_position_controllers : list
        List containing the names of the position controllers used for the arm.
    arm_effort_controllers : list
        List containing the names of the effort controllers used for the arm.
    hand_position_controllers  : list
        List containing the names of the position controllers used for the hand.
    hand_effort_controller : list
        List containing the names of the effort controllers used for the hand.
    """

    def __init__(self, use_group_controller=False, connection_timeout=10):
        """Initializes the PandaControlServer object

        Parameters
        ----------
        connection_timeout : int, optional
            The timeout for connecting to the controller_manager services,
            by default 3 sec.
        use_group_controller : bool, optional
            Whether you want to use the group controllers, by default False.
        """

        # Create class attributes
        self.joint_positions_setpoint = []
        self.joint_efforts_setpoint = []
        self.joint_positions_threshold = 0.01
        self.joint_efforts_threshold = 0.01
        self.use_group_controller = use_group_controller
        self._wait_till_done_timeout = rospy.Duration(8)
        self._joint_efforts_grad_threshold = 0.01
        self._joint_positions_grad_threshold = 0.01
        self._joint_state_topic = "/joint_states"

        #############################################
        # Create Panda control services #############
        #############################################
        rospy.loginfo("Creating '%s' services." % rospy.get_name())
        rospy.logdebug("Creating '%s/set_joint_positions' service." % rospy.get_name())
        self._set_joint_positions_srv = rospy.Service(
            "%s/set_joint_positions" % rospy.get_name()[1:],
            SetJointPositions,
            self._set_joint_positions_callback,
        )
        rospy.logdebug("Creating '%s/set_joint_efforts' service." % rospy.get_name())
        self._set_joint_efforts_srv = rospy.Service(
            "%s/set_joint_efforts" % rospy.get_name()[1:],
            SetJointEfforts,
            self._set_joint_efforts_callback,
        )
        rospy.logdebug(
            "Creating '%s/panda_arm/set_joint_positions' service." % rospy.get_name()
        )
        self._set_arm_joint_positions_srv = rospy.Service(
            "%s/panda_arm/set_joint_positions" % rospy.get_name()[1:],
            SetJointPositions,
            self._arm_set_joint_positions_callback,
        )
        rospy.logdebug(
            "Creating '%s/panda_arm/set_joint_efforts' service." % rospy.get_name()
        )
        self._set_arm_joint_efforts_srv = rospy.Service(
            "%s/panda_arm/set_joint_efforts" % rospy.get_name()[1:],
            SetJointEfforts,
            self._arm_set_joint_efforts_callback,
        )
        rospy.logdebug(
            "Creating '%s/panda_hand/set_joint_positions' service." % rospy.get_name()
        )
        self._set_hand_joint_positions_srv = rospy.Service(
            "%s/panda_hand/set_joint_positions" % rospy.get_name()[1:],
            SetJointPositions,
            self._hand_set_joint_positions_callback,
        )
        rospy.logdebug(
            "Creating '%s/panda_hand/set_joint_efforts' service." % rospy.get_name()
        )
        self._set_hand_joint_efforts_srv = rospy.Service(
            "%s/panda_hand/set_joint_efforts" % rospy.get_name()[1:],
            SetJointEfforts,
            self._hand_set_joint_efforts_callback,
        )
        rospy.logdebug("Creating '%s/switch_control_type' service." % rospy.get_name())
        self._switch_control_type_srv = rospy.Service(
            "%s/switch_control_type" % rospy.get_name()[1:],
            SwitchControlType,
            self._switch_control_type_callback,
        )
        rospy.logdebug("Creating '%s/list_control_type' service." % rospy.get_name())
        self._list_control_type_srv = rospy.Service(
            "%s/list_control_type" % rospy.get_name()[1:],
            ListControlType,
            self._list_control_type_callback,
        )
        rospy.loginfo("'%s' services created successfully." % rospy.get_name())

        #############################################
        # Create panda_control publishers and #######
        # and connect to required services.   #######
        #############################################

        # Create arm joint position group controller publisher
        self._arm_joint_positions_group_pub = GroupPublisher()
        for position_controller in ARM_POSITION_GROUP_CONTROLLERS:
            self._arm_joint_positions_group_pub.append(
                rospy.Publisher(
                    "%s/command" % position_controller, Float64MultiArray, queue_size=10
                )
            )

        # Create (individual) arm joint position controller publishers
        self._arm_joint_position_pub = GroupPublisher()
        for position_controller in ARM_POSITION_CONTROLLERS:
            self._arm_joint_position_pub.append(
                rospy.Publisher(
                    "%s/command" % position_controller, Float64, queue_size=10
                )
            )

        # Create arm joint effort group publisher
        self._arm_joint_efforts_group_pub = GroupPublisher()
        for effort_controller in ARM_EFFORT_GROUP_CONTROLLERS:
            self._arm_joint_efforts_group_pub.append(
                rospy.Publisher(
                    "%s/command" % effort_controller, Float64MultiArray, queue_size=10
                )
            )

        # Create (individual) arm joint effort publishers
        self._arm_joint_effort_pub = GroupPublisher()
        for effort_controller in ARM_EFFORT_CONTROLLERS:
            self._arm_joint_effort_pub.append(
                rospy.Publisher(
                    "%s/command" % effort_controller, Float64, queue_size=10
                )
            )

        # Create hand joint position group publishers
        self._hand_joint_positions_group_pub = GroupPublisher()
        for position_controller in HAND_POSITION_GROUP_CONTROLLERS:
            self._hand_joint_positions_group_pub.append(
                rospy.Publisher(
                    "%s/command" % position_controller, Float64MultiArray, queue_size=10
                )
            )

        # Create (individual) hand joint position publishers
        self._hand_joint_position_pub = GroupPublisher()
        for position_controller in HAND_POSITION_CONTROLLERS:
            self._hand_joint_position_pub.append(
                rospy.Publisher(
                    "%s/command" % position_controller, Float64, queue_size=10
                )
            )

        # Create hand joint group effort publishers
        self._hand_joint_efforts_group_pub = GroupPublisher()
        for effort_controller in HAND_EFFORT_GROUP_CONTROLLERS:
            self._hand_joint_efforts_group_pub.append(
                rospy.Publisher(
                    "%s/command" % effort_controller, Float64MultiArray, queue_size=10
                )
            )

        # Create hand joint group effort publishers
        self._hand_joint_effort_pub = GroupPublisher()
        for effort_controller in HAND_EFFORT_CONTROLLERS:
            self._hand_joint_effort_pub.append(
                rospy.Publisher(
                    "%s/command" % effort_controller, Float64, queue_size=10
                )
            )

        # Connect to controller_manager services
        try:

            # Connect to list service
            rospy.logdebug(
                "Connecting to '/controller_manager/list_controllers' service."
            )
            rospy.wait_for_service(
                "/controller_manager/list_controllers", timeout=connection_timeout
            )
            self.list_controllers_client = rospy.ServiceProxy(
                "/controller_manager/list_controllers", ListControllers
            )
            rospy.logdebug(
                "Connected to '/controller_manager/list_controllers' service!"
            )
        except (rospy.ServiceException, ROSException, ROSInterruptException) as e:
            rospy.logerr(
                "Shutting down '%s' because no connection could be established "
                "with the '%s' service and this service is needed "
                "when using 'joint_position_control'."
                % (
                    rospy.get_name(),
                    e.args[0].strip("timeout exceeded while waiting for service"),
                )
            )
            sys.exit(0)

        #############################################
        # Get controller information ################
        #############################################

        # Set panda_control to right controller type (Group or individual)
        if not self.use_group_controller:
            self.arm_position_controllers = ARM_POSITION_CONTROLLERS
            self.arm_effort_controllers = ARM_EFFORT_CONTROLLERS
            self.hand_position_controllers = HAND_POSITION_CONTROLLERS
            self.hand_effort_controllers = HAND_EFFORT_CONTROLLERS
            self._arm_position_pub = self._arm_joint_position_pub
            self._arm_effort_pub = self._arm_joint_effort_pub
            self._hand_position_pub = self._hand_joint_position_pub
            self._hand_effort_pub = self._hand_joint_effort_pub
            self._arm_position_controller_msg_type = Float64
            self._arm_effort_controller_msg_type = Float64
            self._hand_position_controller_msg_type = Float64
            self._hand_effort_controller_msg_type = Float64
        else:
            self.arm_position_controllers = ARM_POSITION_GROUP_CONTROLLERS
            self.arm_effort_controllers = ARM_EFFORT_GROUP_CONTROLLERS
            self.hand_position_controllers = HAND_POSITION_GROUP_CONTROLLERS
            self.hand_effort_controllers = HAND_EFFORT_GROUP_CONTROLLERS
            self._arm_position_pub = self._arm_joint_positions_group_pub
            self._arm_effort_pub = self._arm_joint_efforts_group_pub
            self._hand_position_pub = self._hand_joint_positions_group_pub
            self._hand_effort_pub = self._hand_joint_efforts_group_pub
            self._arm_position_controller_msg_type = Float64MultiArray
            self._arm_effort_controller_msg_type = Float64MultiArray
            self._hand_position_controller_msg_type = Float64MultiArray
            self._hand_effort_controller_msg_type = Float64MultiArray

        # Create combined position/effort controllers lists
        self._position_controllers = flatten_list(
            [self.arm_position_controllers, self.hand_position_controllers]
        )
        self._effort_controllers = flatten_list(
            [self.arm_effort_controllers, self.hand_effort_controllers]
        )

        # Retrieve informations about the controllers
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )

        #############################################
        # Connect joint state subscriber ############
        #############################################

        # Retrieve current robot joint state and effort information
        self.joint_states = None
        while self.joint_states is None and not rospy.is_shutdown():
            try:
                self.joint_states = rospy.wait_for_message(
                    self._joint_state_topic, JointState, timeout=1.0
                )

                # Set joint setpoint to current position
                self.joint_positions_setpoint = self.joint_states.position
                self.joint_efforts_setpoint = self.joint_states.effort
            except ROSException:
                rospy.logwarn(
                    "Current /joint_states not ready yet, retrying for getting %s"
                    % self._joint_state_topic
                )

        # Create joint_state subscriber
        self._joint_states_sub = rospy.Subscriber(
            self._joint_state_topic, JointState, self._joints_callback
        )

    ###############################################
    # Panda control member functions ##############
    ###############################################
    def _wait_till_done(
        self, control_type, control_group="both", timeout=None, controlled_joints=None
    ):
        """Wait control is finished. Meaning the robot state is within range of the
        Joint position and joint effort setpoints.

        Parameters
        ----------
        control_type : str
            The type of control that is being executed and on which we should wait.
            Options are 'effort_control' and 'position_control'.
        control_group : str, optional
            The control group  for which the control is being performed, defaults to
            "both".
        controlled_joints : dict, optional
            A dictionary containing the joints that are currently being controlled,
            these joints will be determined if no dictionary is given.
        connection_timeout : int, optional
            The timeout when waiting for the control to be done, by default
            self._wait_till_done_timeout.
        """

        # Validate control type and control group
        if control_type not in ["position_control", "effort_control"]:
            rospy.logwarn(
                "Please specify a valid control type. Valid values are %s."
                % ("['position_control', 'effort_control']")
            )
            return False
        else:

            # De-capitalize control type
            control_type = control_type.lower()
        if control_group not in ["arm", "hand", "both"]:
            rospy.logwarn(
                "The control group '%s' you specified is not valid. Valid values are "
                "%s. Control group 'both' used instead." % ("['arm', 'hand', 'both']")
            )
            control_group = "both"
        else:

            # De-capitalize control group
            control_group = control_group.lower()

        # Check if the controlled joints dictionary was given and compute the
        # state masks
        if controlled_joints:
            arm_states_mask = [
                joint in controlled_joints["arm"] for joint in self.joint_states.name
            ]
            hand_states_mask = [
                joint in controlled_joints["hand"] for joint in self.joint_states.name
            ]
        else:  # Try to determine the controlled joints and state mask
            controlled_joints = self._get_controlled_joints(control_type=control_type)
            arm_states_mask = [
                joint in controlled_joints["arm"] for joint in self.joint_states.name
            ]
            hand_states_mask = [
                joint in controlled_joints["hand"] for joint in self.joint_states.name
            ]

        # Get set input arguments
        if timeout:  # If not supplied
            timeout = rospy.Duration(timeout)
        else:
            timeout = self._wait_till_done_timeout

        # Select the right mask for the control_group
        if control_group == "arm":
            states_mask = arm_states_mask
        elif control_group == "hand":
            states_mask = hand_states_mask
        else:
            states_mask = [
                any(bool_tuple) for bool_tuple in zip(arm_states_mask, hand_states_mask)
            ]

        # Wait till robot positions/efforts are not changing anymore
        # NOTE: We have to use the std to determine whether the control was finished
        # as the velocity in the joint_states topic is wrong (see issue 14)
        timeout_time = rospy.get_rostime() + timeout
        positions_buffer = np.full((2, len(self.joint_states.position)), np.nan)
        positions_grad = np.full((2, len(self.joint_states.position)), np.nan)
        efforts_buffer = np.full((2, len(self.joint_states.effort)), np.nan)
        efforts_grad = np.full((2, len(self.joint_states.effort)), np.nan)
        while not rospy.is_shutdown() and rospy.get_rostime() < timeout_time:

            # Wait till joint positions are within range or arm not changing anymore
            if control_type == "position_control":

                # Retrieve positions setpoint
                joint_positions_setpoint = flatten_list(
                    [
                        self.joint_positions_setpoint["hand"],
                        self.joint_positions_setpoint["arm"],
                    ]
                    if hand_states_mask[0]
                    else [
                        self.joint_positions_setpoint["arm"],
                        self.joint_positions_setpoint["hand"],
                    ]
                )

                # Add state to position to buffer
                positions_buffer = np.append(
                    positions_buffer, [self.joint_states.position], axis=0
                )
                positions_buffer = np.delete(
                    positions_buffer, 0, axis=0
                )  # Delete oldest entry
                positions_grad = np.gradient(positions_buffer, axis=0)

                # Check if joint states are within setpoints
                if (
                    np.linalg.norm(
                        np.array(
                            list(compress(self.joint_states.position, states_mask))
                        )
                        - np.array(
                            list(compress(joint_positions_setpoint, states_mask))
                        )
                    )
                    <= self.joint_positions_threshold
                ) or all(
                    [
                        (
                            np.abs(val) <= self._joint_positions_grad_threshold
                            and val != 0.0
                        )
                        for val in list(compress(positions_grad[-1], states_mask))
                    ]
                ):  # Check if difference norm is within threshold
                    break

            # Check if joint effort states are within setpoint
            elif control_type == "effort_control":

                # Retrieve positions setpoint
                joint_efforts_setpoint = flatten_list(
                    [
                        self.joint_efforts_setpoint["hand"],
                        self.joint_efforts_setpoint["arm"],
                    ]
                    if hand_states_mask[0]
                    else [
                        self.joint_efforts_setpoint["arm"],
                        self.joint_efforts_setpoint["hand"],
                    ]
                )

                # Add state to effort to buffer
                efforts_buffer = np.append(
                    efforts_buffer, [self.joint_states.effort], axis=0
                )
                efforts_buffer = np.delete(
                    efforts_buffer, 0, axis=0
                )  # Delete oldest entry
                efforts_grad = np.gradient(efforts_buffer, axis=0)
                if (
                    np.linalg.norm(
                        np.array(list(compress(self.joint_states.effort, states_mask)))
                        - np.array(list(compress(joint_efforts_setpoint, states_mask)))
                    )
                    <= self.joint_efforts_threshold
                ) or all(
                    [
                        (
                            np.abs(val) <= self._joint_efforts_grad_threshold
                            and val != 0.0
                        )
                        for val in list(compress(efforts_grad[-1], states_mask))
                    ]
                ):  # Check if difference norm is within threshold
                    break
            else:
                rospy.loginfo(
                    "Not waiting for control to be completed as '%s' is not "
                    "a valid control type"
                )

        # Return value
        return True

    def _create_control_publisher_msg(
        self, input_msg, control_type, control_group, verbose=False
    ):
        """Converts the service input message into a control commands that is used by
        the control publishers. While doing this it also verifies whether the given
        input message is valid.

        Parameters
        ----------
        input_msg :
            The service input message we want to validate.
        control_type : str
            The type of control that is being executed and on which we should wait.
            Options are 'effort_control' and 'position_control'.
        control_group : str
            The robot control group which is being controlled. Options are "arm",
            "hand" or "both".
        verbose : bool
            Bool specifying whether you want to send a warning message to the ROS
            logger.

        Returns
        -------
        (Dict, Dict)
            Two dictionaries. The first dictionary contains the Panda arm and hand
            control commands in the order which is are required by the publishers.
            The second dictionary contains the joints that are being controlled.

        Raises
        ----------
        panda_training.exceptions.InputMessageInvalid
            Raised when the input_msg could not be converted into 'moveit_commander'
            arm hand joint position commands.
        """

        # Validate control_group
        if control_group.lower() not in ["arm", "hand", "both"]:

            # Log message and return result
            logwarn_message = (
                "Control group '%s' does not exist. Please specify a valid control "
                "group. Valid values are %s."
                % (control_group.lower(), "['arm', 'hand', 'both']")
            )
            if verbose:
                rospy.logwarn(logwarn_message)
            raise InputMessageInvalid(
                message="Control_group '%s' invalid." % control_group.lower(),
                log_message=logwarn_message,
            )
        else:

            # De-capitalize input arguments
            control_group = control_group.lower()
            control_type = control_type.lower()

        # Validate control type and extract information from input message
        if control_type == "position_control":
            joint_names = input_msg.joint_names
            control_input = input_msg.joint_positions.data
        elif control_type == "effort_control":
            joint_names = input_msg.joint_names
            control_input = input_msg.joint_efforts.data
        else:

            # Log message and return result
            logwarn_message = (
                "Please specify a valid control type. Valid values are %s."
                % ("['position_control', 'effort_control']")
            )
            if verbose:
                rospy.logwarn(logwarn_message)
            raise InputMessageInvalid(
                message="Control_type '%s' invalid." % control_type,
                log_message=logwarn_message,
            )

        # Compute controlled joints
        controlled_joints_dict = self._get_controlled_joints(
            control_type=control_type, verbose=verbose
        )

        # Get contolled joints information
        if control_group == "arm":
            controlled_joints = controlled_joints_dict["arm"]
        elif control_group == "hand":
            controlled_joints = controlled_joints_dict["hand"]
        else:
            controlled_joints = controlled_joints_dict["both"]
        controlled_joints_size = len(controlled_joints)

        # Retrieve state mask
        # NOTE: Used to determine which values in the /joint_states topic
        # are related to the arm and which to the hand.
        arm_states_mask = [
            joint in controlled_joints_dict["arm"] for joint in self.joint_states.name
        ]
        hand_states_mask = [
            joint in controlled_joints_dict["hand"] for joint in self.joint_states.name
        ]

        # Retrieve the current robot state
        state_list = list(
            self.joint_states.position
            if control_type == "position_control"
            else self.joint_states.effort
        )
        arm_state_dict = OrderedDict(
            zip(
                list(
                    compress(
                        self.joint_states.name,
                        [
                            item in flatten_list(controlled_joints_dict["arm"])
                            for item in self.joint_states.name
                        ],
                    )
                ),
                list(
                    compress(
                        state_list,
                        [
                            item in flatten_list(controlled_joints_dict["arm"])
                            for item in self.joint_states.name
                        ],
                    )
                ),
            )
        )
        hand_state_dict = OrderedDict(
            zip(
                list(
                    compress(
                        self.joint_states.name,
                        [
                            item in flatten_list(controlled_joints_dict["hand"])
                            for item in self.joint_states.name
                        ],
                    )
                ),
                list(
                    compress(
                        state_list,
                        [
                            item in flatten_list(controlled_joints_dict["hand"])
                            for item in self.joint_states.name
                        ],
                    )
                ),
            )
        )

        # Get control publisher message type
        if control_type == "position_control":
            arm_msg_type = self._arm_position_controller_msg_type
            hand_msg_type = self._hand_position_controller_msg_type
        elif control_type == "effort_control":
            arm_msg_type = self._arm_effort_controller_msg_type
            hand_msg_type = self._hand_effort_controller_msg_type

        # Check service request input
        if len(joint_names) == 0:

            # Check if enough joint position commands were given otherwise give warning
            if len(control_input) != controlled_joints_size:

                # Create log message strings
                if control_type == "position_control":
                    logwarn_msg_strings = [
                        "joint position"
                        if len(control_input) == 1
                        else "joint positions",
                        "joint" if controlled_joints_size == 1 else "joints",
                    ]
                else:
                    logwarn_msg_strings = [
                        "joint effort" if len(control_input) == 1 else "joint efforts",
                        "joint" if controlled_joints_size == 1 else "joints",
                    ]

                # Check if control input is bigger than controllable joints
                if len(control_input) > controlled_joints_size:

                    # Log message and raise exception
                    logwarn_message = "You specified %s while the Panda %s %s." % (
                        "%s %s" % (len(control_input), logwarn_msg_strings[0]),
                        control_group + " control group has"
                        if control_group in ["arm", "hand"]
                        else "arm and hand control groups have",
                        "%s %s" % (controlled_joints_size, logwarn_msg_strings[1]),
                    )
                    if verbose:
                        rospy.logwarn(logwarn_message)
                    raise InputMessageInvalid(
                        message="Invalid number of joint position commands.",
                        log_message=logwarn_message,
                        details={
                            "joint_positions_command_length": len(control_input),
                            "controlled_joints": controlled_joints_size,
                        },
                    )
                elif len(control_input) < controlled_joints_size:

                    # Display warning message
                    logwarn_message = "You specified %s while the Panda %s %s." % (
                        "%s %s" % (len(control_input), logwarn_msg_strings[0]),
                        control_group + " control group has"
                        if control_group in ["arm", "hand"]
                        else "arm and hand control groups have",
                        "%s %s" % (controlled_joints_size, logwarn_msg_strings[1]),
                    ) + " As a result only joints %s will be controlled." % (
                        controlled_joints[0 : len(control_input)]
                    )
                    rospy.logwarn(logwarn_message)

            # Update current state dictionary with given joint_position commands
            if control_group == "arm":
                arm_output_command = copy.deepcopy(arm_state_dict.values())
                arm_output_command[0 : len(control_input)] = control_input
                hand_output_command = hand_state_dict.values()
            elif control_group == "hand":
                hand_output_command = copy.deepcopy(hand_state_dict)
                arm_output_command = arm_state_dict.values()
            else:
                arm_output_command = copy.deepcopy(arm_state_dict.values())
                hand_output_command = copy.deepcopy(hand_state_dict.values())
                if self.joint_states.name[0] in controlled_joints_dict["arm"]:
                    if len(control_input) <= len(arm_output_command):
                        arm_output_command[0 : len(control_input)] = control_input
                    else:
                        arm_output_command[:] = control_input[
                            0 : len(arm_output_command)
                        ]
                        hand_output_command[
                            : len(control_input) - len(arm_output_command)
                        ] = control_input[len(arm_output_command) :]
                else:
                    if len(control_input) <= len(hand_output_command):
                        hand_output_command[0 : len(control_input)] = control_input
                    else:
                        hand_output_command[:] = control_input[
                            0 : len(hand_output_command)
                        ]
                        arm_output_command[
                            : len(control_input) - len(hand_output_command)
                        ] = control_input[len(hand_output_command) :]

            # Create publishers command dictionary
            if not self.use_group_controller:  # Non group controller
                control_commands = {
                    "arm": [arm_msg_type(item) for item in arm_output_command],
                    "hand": [hand_msg_type(item) for item in hand_output_command],
                }
            else:
                control_commands = {
                    "arm": arm_msg_type(data=arm_output_command),
                    "hand": hand_msg_type(data=hand_output_command),
                }

            # Return control commands
            return control_commands, controlled_joints_dict
        else:

            # Check if enough control values were given
            if len(joint_names) != len(control_input):

                # Create log message
                if control_type == "position_control":
                    logwarn_msg_strings = [
                        "joint position"
                        if len(control_input) == 1
                        else "joint positions",
                        "panda_training/SetJointPositions",
                        "joint" if len(joint_names) == 1 else "joints",
                        "joint position",
                        "joint_positions",
                    ]
                else:
                    logwarn_msg_strings = [
                        "joint effort" if len(control_input) == 1 else "joint efforts",
                        "panda_training/SetJointEfforts",
                        "joint effort" if len(joint_names) == 1 else "joint efforts",
                        "joint effort",
                        "joint_efforts",
                    ]
                logwarn_message = (
                    "You specified %s while the 'joint_names' field of the "
                    "'%s' message contains %s. Please make sure you supply "
                    "a %s for each of the joints contained in the 'joint_names' "
                    "field."
                    % (
                        "%s %s" % (len(control_input), logwarn_msg_strings[0]),
                        logwarn_msg_strings[1],
                        "%s %s" % (len(joint_names), logwarn_msg_strings[2]),
                        logwarn_msg_strings[3],
                    )
                )

                # Send log warn message and raise InputMessageInvalid exception
                if verbose:
                    rospy.logwarn(logwarn_message)
                raise InputMessageInvalid(
                    message=(
                        "The joint_names and %s fields of the input message are of "
                        "different lengths." % (logwarn_msg_strings[4])
                    ),
                    log_message=logwarn_message,
                    details={
                        logwarn_msg_strings[4] + "_length": len(control_input),
                        "joint_names_length": len(joint_names),
                    },
                )
            else:

                # Validate joint_names
                invalid_joint_names = [
                    joint_name
                    for joint_name in joint_names
                    if joint_name not in controlled_joints
                ]
                if len(invalid_joint_names) != 0:  # Joint names invalid

                    # Create joint names invalid warn message
                    if control_type == "position_control":
                        logwarn_msg_strings = [
                            "Joint" if len(invalid_joint_names) == 1 else "Joints",
                            "was" if len(invalid_joint_names) == 1 else "were",
                            "panda_training/SetJointPositions",
                        ]
                    else:
                        logwarn_msg_strings = [
                            "Joint" if len(invalid_joint_names) == 1 else "Joints",
                            "was" if len(invalid_joint_names) == 1 else "were",
                            "panda_training/SetJointEfforts",
                        ]
                    logwarn_message = (
                        "%s that %s specified in the 'joint_names' field of the '%s' "
                        "message %s invalid. Valid joint names for controlling the "
                        "Panda %s are %s."
                        % (
                            "%s %s" % (logwarn_msg_strings[0], invalid_joint_names),
                            logwarn_msg_strings[1],
                            logwarn_msg_strings[2],
                            logwarn_msg_strings[1],
                            control_group
                            if control_group in ["hand", "arm"]
                            else "arm & hand",
                            controlled_joints,
                        )
                    )

                    # Send log warn message and raise InputMessageInvalid exception
                    if verbose:
                        rospy.logwarn(logwarn_message)
                    raise InputMessageInvalid(
                        message="Invalid joint_names were given.",
                        log_message=logwarn_message,
                        details={"invalid_joint_names": invalid_joint_names},
                    )
                else:

                    # Create input command dictionary
                    input_command_dict = OrderedDict(zip(joint_names, control_input))

                    # Update current state dictionary with given joint_position commands
                    if control_group == "arm":
                        arm_output_command_dict = copy.deepcopy(arm_state_dict)
                        for (
                            joint,
                            position,
                        ) in input_command_dict.items():  # Update arm
                            if joint in arm_state_dict:
                                arm_output_command_dict[joint] = position
                        arm_output_command = arm_output_command_dict.values()
                        hand_output_command = hand_state_dict.values()
                    elif control_group == "hand":
                        hand_output_command_dict = copy.deepcopy(hand_state_dict)
                        for (
                            joint,
                            position,
                        ) in input_command_dict.items():  # Update hand
                            if joint in hand_state_dict:
                                hand_output_command_dict[joint] = position
                        arm_output_command = arm_state_dict.values()
                        hand_output_command = hand_output_command_dict.values()
                    else:
                        arm_output_command_dict = copy.deepcopy(arm_state_dict)
                        for (
                            joint,
                            position,
                        ) in input_command_dict.items():  # Update arm
                            if joint in arm_state_dict:
                                arm_output_command_dict[joint] = position
                        hand_output_command_dict = copy.deepcopy(hand_state_dict)
                        for (
                            joint,
                            position,
                        ) in input_command_dict.items():  # Update hand
                            if joint in hand_state_dict:
                                hand_output_command_dict[joint] = position
                        arm_output_command = arm_output_command_dict.values()
                        hand_output_command = hand_output_command_dict.values()

                    # Create publishers command dictionary
                    if not self.use_group_controller:  # Non group controller
                        control_commands = {
                            "arm": [arm_msg_type(item) for item in arm_output_command],
                            "hand": [
                                hand_msg_type(item) for item in hand_output_command
                            ],
                        }
                    else:
                        control_commands = {
                            "arm": arm_msg_type(data=arm_output_command),
                            "hand": hand_msg_type(data=hand_output_command),
                        }

                    # Return control publishers commands dictionary
                    return control_commands, controlled_joints_dict

    def _get_controlled_joints(self, control_type, verbose=False):
        """Returns the joints that are controlled by a given control type.

        Parameters
        ----------
        control_type : str
            The type of control that is being executed and on which we should wait.
            Options are 'effort_control' and 'position_control'.
        verbose : bool
            Bool specifying whether you want to send a warning message to the ROS
            logger.

        Returns
        -------
        dict
            A dictionary containing the joints that are controlled when using a given
            control type, grouped by control group ("arm" and "hand).

        Raises
        ----------
        panda_training.exceptions.InputMessageInvalid
            Raised when the input_msg could not be converted into 'moveit_commander'
            arm hand joint position commands.
        """

        # Get the joints which are contolled by a given control type
        if control_type == "position_control":

            # Get contolled joints information
            controlled_joints_dict = OrderedDict(
                zip(["arm", "hand", "both"], [[], [], []])
            )
            try:
                for position_controller in self._position_controllers:
                    for claimed_resources in self._controllers[
                        position_controller
                    ].claimed_resources:
                        for resource in claimed_resources.resources:
                            if position_controller in self.arm_position_controllers:
                                controlled_joints_dict["arm"].append(resource)
                            elif position_controller in self.hand_position_controllers:
                                controlled_joints_dict["hand"].append(resource)
            except KeyError:  # Controllers not initialized

                # Log message and return result
                logwarn_message = (
                    "The position control publisher messages could not be created as "
                    "the '%s' position controllers are not initialized. Initialization "
                    "of these controllers is needed to retrieve information about the "
                    "joints they control." % (self._position_controllers)
                )
                if verbose:
                    rospy.logwarn(logwarn_message)
                raise InputMessageInvalid(
                    message="Required controllers not initialised.",
                    details={"controlled_joints": self._position_controllers},
                    log_message=logwarn_message,
                )
        elif control_type == "effort_control":

            # Get controlled joints information
            controlled_joints_dict = OrderedDict(
                zip(["arm", "hand", "both"], [[], [], []])
            )
            try:
                for effort_controller in self._effort_controllers:
                    for claimed_resources in self._controllers[
                        effort_controller
                    ].claimed_resources:
                        for resource in claimed_resources.resources:
                            if effort_controller in self.arm_effort_controllers:
                                controlled_joints_dict["arm"].append(resource)
                            elif effort_controller in self.hand_effort_controllers:
                                controlled_joints_dict["hand"].append(resource)
            except KeyError:  # Controllers not initialized

                # Log message and return result
                logwarn_message = (
                    "The effort control publisher messages could not be created as "
                    "the '%s' effort controllers are not initialized. Initialization "
                    "of these controllers is needed to retrieve information about the "
                    "joints they control." % (self._effort_controllers)
                )
                if verbose:
                    rospy.logwarn(logwarn_message)
                raise InputMessageInvalid(
                    message="Required controllers not initialised.",
                    details={"controlled_joints": self._effort_controllers},
                    log_message=logwarn_message,
                )
        else:

            # Log message and return result
            logwarn_message = (
                "Please specify a valid control type. Valid values are %s."
                % ("['position_control', 'effort_control']")
            )
            if verbose:
                rospy.logwarn(logwarn_message)
            raise InputMessageInvalid(
                message="Control_type '%s' invalid." % control_type,
                log_message=logwarn_message,
            )

        # Fill controlled joints dict
        controlled_joints_dict["arm"] = flatten_list(controlled_joints_dict["arm"])
        controlled_joints_dict["hand"] = flatten_list(controlled_joints_dict["hand"])
        controlled_joints_dict["both"] = (
            flatten_list(
                [controlled_joints_dict["hand"], controlled_joints_dict["arm"]]
            )
            if self.joint_states.name[0] in controlled_joints_dict["hand"]
            else flatten_list(
                [controlled_joints_dict["arm"], controlled_joints_dict["hand"]]
            )
        )

        # Return controlled joints dict
        return controlled_joints_dict

    def _get_joint_controllers(self):
        """Retrieves the controllers which are currently initialized to work with a
        given joint.

        Returns
        -------
        dict
            Dictionary containing each panda joint and the controllers that are able to
            control these joints.
        """
        joint_contollers_dict = {}
        for (key, val) in self._controllers.items():
            for resources_item in val.claimed_resources:
                for resource in resources_item.resources:
                    if resource in joint_contollers_dict.keys():
                        joint_contollers_dict[resource].append(key)
                    else:
                        joint_contollers_dict[resource] = [key]
        return joint_contollers_dict

    ###############################################
    # Service callback functions ##################
    ###############################################
    def _set_joint_positions_callback(self, set_joint_positions_req):
        """Request arm and hand joint position control.

        Parameters
        ----------
        set_joint_positions_req : panda_training.srv.SetJointPositionsRequest
            Service request message specifying the positions for the robot arm joints.

        Returns
        -------
        panda_training.srv.SetJointPositionsResponse
            Service response.
        """

        # Check if set_joint_efforts_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(set_joint_positions_req.joint_names)
        if duplicate_list:

            # Print warning
            rospy.logwarn(
                "Multiple entries were found for %s '%s' in the '%s' message. "
                "Consequently, only the last occurrence was used in setting the joint "
                "positions."
                % (
                    "joints" if len(duplicate_list) > 1 else "joint",
                    duplicate_list,
                    "%s/set_joint_positions" % rospy.get_name(),
                )
            )

        # Create required variables and messages
        controllers_missing = {"arm": False, "hand": False}
        resp = SetJointPositionsResponse()

        # Retrieve controller information
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )

        # Check if all controllers are available and running
        stopped_controllers = {"arm": [], "hand": []}
        missing_controllers = {"arm": [], "hand": []}
        for (group, position_controllers) in {
            "arm": self.arm_position_controllers,
            "hand": self.hand_position_controllers,
        }.items():
            for position_controller in position_controllers:
                try:
                    if self._controllers[position_controller].state != "running":
                        stopped_controllers[group].append(position_controller)
                except KeyError:
                    missing_controllers[group].append(position_controller)

        # Return failed result if we miss a controller
        if len(flatten_list(missing_controllers.values())) >= 1:
            rospy.logwarn(
                "Panda arm and hand joint position command could not be send as the %s "
                "%s not initialized. Please make sure you load the controller "
                "parameters onto the ROS parameter server."
                % (
                    flatten_list(missing_controllers.values()),
                    "joint position controller is"
                    if len(flatten_list(missing_controllers.values())) == 1
                    else "joint position controllers are",
                )
            )
            missing_controllers_group_string = (
                "Arm and hand"
                if all(
                    [values != [] for (group, values) in missing_controllers.items()]
                )
                else ("Arm" if "arm" in dict_clean(missing_controllers) else "Hand")
            )
            resp.success = False
            resp.message = (
                "%s controllers not initialised." % missing_controllers_group_string
            )
            return resp

        # Validate request and create control publisher message
        try:
            control_pub_msgs, controlled_joints = self._create_control_publisher_msg(
                input_msg=set_joint_positions_req,
                control_type="position_control",
                control_group="both",
            )
        except InputMessageInvalid as e:

            # Print warning message and return result
            logwarn_msg = "Panda arm joint positions not set as " + lower_first_char(
                e.log_message
            )
            rospy.logwarn(logwarn_msg)
            resp.success = False
            resp.message = e.message
            return resp

        # Check if required joints are running
        if len(flatten_list(stopped_controllers.values())) >= 1:

            # Check from which group the controllers are missing
            controllers_missing = {
                group: (True if (position_controllers) else False)
                for (group, position_controllers) in stopped_controllers.items()
            }

            # Check if these controllers are required for the current command
            joint_controllers_dict = self._get_joint_controllers()
            req_missing_controllers = []
            if not set_joint_positions_req.joint_names:
                log_warning = True
                req_missing_controllers = flatten_list(stopped_controllers.values())
            else:
                for joint in set_joint_positions_req.joint_names:
                    if joint in joint_controllers_dict.keys():
                        req_missing_controllers.append(
                            [
                                stopped_controller
                                for stopped_controller in flatten_list(
                                    stopped_controllers.values()
                                )
                                if stopped_controller in joint_controllers_dict[joint]
                            ]
                        )
                req_missing_controllers = flatten_list(req_missing_controllers)
                if req_missing_controllers:
                    log_warning = True
                else:
                    log_warning = False

            # Log warning
            if log_warning:
                rospy.logwarn(
                    "Panda %s joint positions command send but probably not executed "
                    "as the %s %s not running."
                    % (
                        "arm and hand"
                        if all(controllers_missing.values())
                        else ("arm" if controllers_missing["arm"] else "hand"),
                        flatten_list(stopped_controllers.values()),
                        "joint position controller is"
                        if len(flatten_list(stopped_controllers.values())) == 1
                        else "joint position controllers are",
                    )
                )

        # Save position control setpoint
        joint_positions_setpoint_dict = {
            group: [command.data for command in control_list]
            for (group, control_list) in control_pub_msgs.items()
        }
        self.joint_positions_setpoint = joint_positions_setpoint_dict

        # Publish request
        rospy.logdebug("Publishing Panda arm and hand joint positions control message.")
        self._arm_position_pub.publish(control_pub_msgs["arm"])
        self._hand_position_pub.publish(control_pub_msgs["hand"])

        # Wait till control is finished or timeout has been reached
        if set_joint_positions_req.wait.data and not all(controllers_missing.values()):
            wait_control_group = (
                "both"
                if all([not bool_val for bool_val in controllers_missing.values()])
                else ("arm" if not controllers_missing["arm"] else "hand")
            )
            self._wait_till_done(
                control_type="position_control",
                control_group=wait_control_group,
                controlled_joints=controlled_joints,
            )

        # Return success message
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _set_joint_efforts_callback(self, set_joint_efforts_req):
        """Request arm and hand joint effort control.

        Parameters
        ----------
        set_joint_efforts_req : panda_training.srv.SetJointEffortsRequest
            Service request message specifying the efforts for the robot arm joints.

        Returns
        -------
        panda_training.srv.SetJointEffortsResponse
            Service response.
        """

        # Check if set_joint_efforts_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(set_joint_efforts_req.joint_names)
        if duplicate_list:

            # Print warning
            rospy.logwarn(
                "Multiple entries were found for %s '%s' in the '%s' message. "
                "Consequently, only the last occurrence was used in setting the joint "
                "efforts."
                % (
                    "joints" if len(duplicate_list) > 1 else "joint",
                    duplicate_list,
                    "%s/set_joint_efforts" % rospy.get_name(),
                )
            )

        # Create required variables and messages
        controllers_missing = {"arm": False, "hand": False}
        resp = SetJointEffortsResponse()

        # Retrieve controller information
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )

        # Check if all controllers are available and running
        stopped_controllers = {"arm": [], "hand": []}
        missing_controllers = {"arm": [], "hand": []}
        for (group, effort_controllers) in {
            "arm": self.arm_effort_controllers,
            "hand": self.hand_effort_controllers,
        }.items():
            for effort_controller in effort_controllers:
                try:
                    if self._controllers[effort_controller].state != "running":
                        stopped_controllers[group].append(effort_controller)
                except KeyError:
                    missing_controllers[group].append(effort_controller)

        # Return failed result if we miss a controller
        if len(flatten_list(missing_controllers.values())) >= 1:
            rospy.logwarn(
                "Panda arm and hand joint effort command could not be send as the %s "
                "%s not initialized. Please make sure you load the controller "
                "parameters onto the ROS parameter server."
                % (
                    flatten_list(missing_controllers.values()),
                    "joint effort controller is"
                    if len(flatten_list(missing_controllers.values())) == 1
                    else "joint effort controllers are",
                )
            )
            missing_controllers_group_string = (
                "Arm and hand"
                if all(
                    [values != [] for (group, values) in missing_controllers.items()]
                )
                else ("Arm" if "arm" in dict_clean(missing_controllers) else "Hand")
            )
            resp.success = False
            resp.message = (
                "%s controllers not initialised." % missing_controllers_group_string
            )
            return resp

        # Validate request and create control publisher message
        try:
            control_pub_msgs, controlled_joints = self._create_control_publisher_msg(
                input_msg=set_joint_efforts_req,
                control_type="effort_control",
                control_group="both",
            )
        except InputMessageInvalid as e:

            # Print warning message and return result
            logwarn_msg = "Panda arm joint efforts not set as " + lower_first_char(
                e.log_message
            )
            rospy.logwarn(logwarn_msg)
            resp.success = False
            resp.message = e.message
            return resp

        # Check if required joints are running
        if len(flatten_list(stopped_controllers.values())) >= 1:

            # Check from which group the controllers are missing
            controllers_missing = {
                group: (True if (effort_controllers) else False)
                for (group, effort_controllers) in stopped_controllers.items()
            }

            # Check if these controllers are required for the current command
            joint_controllers_dict = self._get_joint_controllers()
            req_missing_controllers = []
            if not set_joint_efforts_req.joint_names:
                log_warning = True
                req_missing_controllers = flatten_list(stopped_controllers.values())
            else:
                for joint in set_joint_efforts_req.joint_names:
                    if joint in joint_controllers_dict.keys():
                        req_missing_controllers.append(
                            [
                                stopped_controller
                                for stopped_controller in flatten_list(
                                    stopped_controllers.values()
                                )
                                if stopped_controller in joint_controllers_dict[joint]
                            ]
                        )
                req_missing_controllers = flatten_list(req_missing_controllers)
                if req_missing_controllers:
                    log_warning = True
                else:
                    log_warning = False

            # Log warning
            if log_warning:
                rospy.logwarn(
                    "Panda %s joint efforts command send but probably not executed as "
                    "the %s %s not running."
                    % (
                        "arm and hand"
                        if all(controllers_missing.values())
                        else ("arm" if controllers_missing["arm"] else "hand"),
                        req_missing_controllers,
                        "joint effort controller is"
                        if len(req_missing_controllers) == 1
                        else "joint effort controllers are",
                    )
                )

        # Save effort control setpoint
        joint_efforts_setpoint_dict = {
            group: [command.data for command in control_list]
            for (group, control_list) in control_pub_msgs.items()
        }
        self.joint_efforts_setpoint = joint_efforts_setpoint_dict

        # Publish request
        rospy.logdebug("Publishing Panda arm and hand joint efforts control message.")
        self._arm_effort_pub.publish(control_pub_msgs["arm"])
        self._hand_effort_pub.publish(control_pub_msgs["hand"])

        # Wait till control is finished or timeout has been reached
        if set_joint_efforts_req.wait.data and not all(controllers_missing.values()):
            wait_control_group = (
                "both"
                if all([not bool_val for bool_val in controllers_missing.values()])
                else ("arm" if not controllers_missing["arm"] else "hand")
            )
            self._wait_till_done(
                control_type="effort_control",
                control_group=wait_control_group,
                controlled_joints=controlled_joints,
            )

        # Return success message
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _arm_set_joint_positions_callback(self, set_joint_positions_req):
        """Request arm joint position control.

        Parameters
        ----------
        set_joint_positions_req : panda_training.srv.SetJointPositionsRequest
            Service request message specifying the positions for the robot arm joints.

        Returns
        -------
        panda_training.srv.SetJointPositionsResponse
            Service response.
        """

        # Check if set_joint_efforts_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(set_joint_positions_req.joint_names)
        if duplicate_list:

            # Print warning
            rospy.logwarn(
                "Multiple entries were found for %s '%s' in the '%s' message. "
                "Consequently, only the last occurrence was used in setting the joint "
                "positions."
                % (
                    "joints" if len(duplicate_list) > 1 else "joint",
                    duplicate_list,
                    "%s/panda_arm/set_joint_positions" % rospy.get_name(),
                )
            )

        # Create required variables and messages
        controllers_missing = False
        resp = SetJointPositionsResponse()

        # Retrieve controller information
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )

        # Check if all controllers are available and running
        stopped_controllers = []
        missing_controllers = []
        for position_controller in self.arm_position_controllers:
            try:
                if self._controllers[position_controller].state != "running":
                    stopped_controllers.append(position_controller)
            except KeyError:
                missing_controllers.append(position_controller)

        # Return failed result if we miss a controller
        if len(missing_controllers) >= 1:
            rospy.logwarn(
                "Panda arm joint position command could not be send as the %s %s "
                "not initialized. Please make sure you load the controller parameters "
                "onto the ROS parameter server."
                % (
                    missing_controllers,
                    "joint position controller is"
                    if len(missing_controllers) == 1
                    else "joint position controllers are",
                )
            )
            resp.success = False
            resp.message = "Arm controllers not initialised."
            return resp

        # Validate request and create control publisher message
        try:
            control_pub_msgs, controlled_joints = self._create_control_publisher_msg(
                input_msg=set_joint_positions_req,
                control_type="position_control",
                control_group="arm",
            )
        except InputMessageInvalid as e:

            # Print warning message and return result
            logwarn_msg = "Panda arm joint positions not set as " + lower_first_char(
                e.log_message
            )
            rospy.logwarn(logwarn_msg)
            resp.success = False
            resp.message = e.message
            return resp

        # Check if required joints are running
        if len(stopped_controllers) >= 1:

            # Set controllers missing boolean
            controllers_missing = True

            # Check if these controllers are required for the current command
            joint_controllers_dict = self._get_joint_controllers()
            req_missing_controllers = []
            if not set_joint_positions_req.joint_names:
                log_warning = True
                req_missing_controllers = flatten_list(stopped_controllers)
            else:
                for joint in set_joint_positions_req.joint_names:
                    if joint in joint_controllers_dict.keys():
                        req_missing_controllers.append(
                            [
                                stopped_controller
                                for stopped_controller in flatten_list(
                                    stopped_controllers
                                )
                                if stopped_controller in joint_controllers_dict[joint]
                            ]
                        )
                req_missing_controllers = flatten_list(req_missing_controllers)
                if req_missing_controllers:
                    log_warning = True
                else:
                    log_warning = False

            # Log warning
            if log_warning:
                rospy.logwarn(
                    "Panda arm joint positions command send but probably not executed "
                    "as the %s %s not running."
                    % (
                        stopped_controllers,
                        "joint position controller is"
                        if len(stopped_controllers) == 1
                        else "joint position controllers are",
                    )
                )

        # Save position control setpoint
        joint_positions_setpoint_dict = {
            group: [command.data for command in control_list]
            for (group, control_list) in control_pub_msgs.items()
        }
        self.joint_positions_setpoint = joint_positions_setpoint_dict

        # Publish request
        rospy.logdebug("Publishing Panda arm joint positions control message.")
        self._arm_position_pub.publish(control_pub_msgs["arm"])

        # Wait till control is finished or timeout has been reached
        if set_joint_positions_req.wait.data and not controllers_missing:
            self._wait_till_done(
                control_type="position_control",
                control_group="arm",
                controlled_joints=controlled_joints,
            )

        # Return success message
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _arm_set_joint_efforts_callback(self, set_joint_efforts_req):
        """Request arm Joint effort control.

        Parameters
        ----------
        set_joint_efforts_req : panda_training.srv.SetJointEffortsRequest
            Service request message specifying the efforts for the robot arm joints.

        Returns
        -------
        panda_training.srv.SetJointEffortsResponse
            Service response.
        """

        # Check if set_joint_efforts_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(set_joint_efforts_req.joint_names)
        if duplicate_list:

            # Print warning
            rospy.logwarn(
                "Multiple entries were found for %s '%s' in the '%s' message. "
                "Consequently, only the last occurrence was used in setting the joint "
                "efforts."
                % (
                    "joints" if len(duplicate_list) > 1 else "joint",
                    duplicate_list,
                    "%s/panda_arm/set_joint_efforts" % rospy.get_name(),
                )
            )

        # Create required variables and messages
        controllers_missing = False
        resp = SetJointEffortsResponse()

        # Retrieve controller information
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )

        # Check if all controllers are available and running
        stopped_controllers = []
        missing_controllers = []
        for effort_controller in self.arm_effort_controllers:
            try:
                if self._controllers[effort_controller].state != "running":
                    stopped_controllers.append(effort_controller)
            except KeyError:
                missing_controllers.append(effort_controller)

        # Return failed result if we miss a controller
        if len(missing_controllers) >= 1:
            rospy.logwarn(
                "Panda arm joint effort command could not be send as the %s %s "
                "not initialized. Please make sure you load the controller parameters "
                "onto the ROS parameter server."
                % (
                    missing_controllers,
                    "joint effort controller is"
                    if len(missing_controllers) == 1
                    else "joint effort controllers are",
                )
            )
            resp.success = False
            resp.message = "Arm controllers not initialised."
            return resp

        # Validate request and create control publisher message
        try:
            control_pub_msgs, controlled_joints = self._create_control_publisher_msg(
                input_msg=set_joint_efforts_req,
                control_type="effort_control",
                control_group="arm",
            )
        except InputMessageInvalid as e:

            # Print warning message and return result
            logwarn_msg = "Panda arm joint efforts not set as " + lower_first_char(
                e.log_message
            )
            rospy.logwarn(logwarn_msg)
            resp.success = False
            resp.message = e.message
            return resp

        # Check if required controllers are running
        if len(stopped_controllers) >= 1:

            # Set controllers missing boolean
            controllers_missing = True

            # Check if these controllers are required for the current command
            joint_controllers_dict = self._get_joint_controllers()
            req_missing_controllers = []
            if not set_joint_efforts_req.joint_names:
                log_warning = True
                req_missing_controllers = flatten_list(stopped_controllers)
            else:
                for joint in set_joint_efforts_req.joint_names:
                    if joint in joint_controllers_dict.keys():
                        req_missing_controllers.append(
                            [
                                stopped_controller
                                for stopped_controller in flatten_list(
                                    stopped_controllers
                                )
                                if stopped_controller in joint_controllers_dict[joint]
                            ]
                        )
                req_missing_controllers = flatten_list(req_missing_controllers)
                if req_missing_controllers:
                    log_warning = True
                else:
                    log_warning = False

            # Log warning
            if log_warning:
                rospy.logwarn(
                    "Panda arm joint efforts command send but probably not executed as "
                    "the %s %s not running."
                    % (
                        stopped_controllers,
                        "joint effort controller is"
                        if len(stopped_controllers) == 1
                        else "joint effort controllers are",
                    )
                )

        # Save effort control setpoint
        joint_efforts_setpoint_dict = {
            group: [command.data for command in control_list]
            for (group, control_list) in control_pub_msgs.items()
        }
        self.joint_efforts_setpoint = joint_efforts_setpoint_dict

        # Publish request
        rospy.logdebug("Publishing Panda arm joint efforts control message.")
        self._arm_effort_pub.publish(control_pub_msgs["arm"])

        # Wait till control is finished or timeout has been reached
        if set_joint_efforts_req.wait.data and not controllers_missing:
            self._wait_till_done(
                control_type="effort_control",
                control_group="arm",
                controlled_joints=controlled_joints,
            )

        # Return service response
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _hand_set_joint_positions_callback(self, set_joint_positions_req):
        """Request hand joint position control

        Parameters
        ----------
        set_joint_positions_req : panda_training.srv.SetJointPositionsRequest
            Service request message specifying the positions for the robot hand joints.

        Returns
        -------
        panda_training.srv.SetJointPositionsResponse
            Service response.
        """

        # Check if set_joint_efforts_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(set_joint_positions_req.joint_names)
        if duplicate_list:

            # Print warning
            rospy.logwarn(
                "Multiple entries were found for %s '%s' in the '%s' message. "
                "Consequently, only the last occurrence was used in setting the joint "
                "positions."
                % (
                    "joints" if len(duplicate_list) > 1 else "joint",
                    duplicate_list,
                    "%s/panda_hand/set_joint_positions" % rospy.get_name(),
                )
            )

        # Create required variables and messages
        controllers_missing = False
        resp = SetJointPositionsResponse()

        # Retrieve controller information
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )

        # Check if all controllers are available and running
        stopped_controllers = []
        missing_controllers = []
        for position_controller in self.hand_position_controllers:
            try:
                if self._controllers[position_controller].state != "running":
                    stopped_controllers.append(position_controller)
            except KeyError:
                missing_controllers.append(position_controller)

        # Return failed result if we miss a controller
        if len(missing_controllers) >= 1:
            rospy.logwarn(
                "Panda hand joint position command could not be send as the %s %s "
                "not initialized. Please make sure you load the controller parameters "
                "onto the ROS parameter server."
                % (
                    missing_controllers,
                    "joint position controller is"
                    if len(missing_controllers) == 1
                    else "joint position controllers are",
                )
            )
            resp.success = False
            resp.message = "Hand controllers not initialised."
            return resp

        # Validate request and create control publisher message
        try:
            control_pub_msgs, controlled_joints = self._create_control_publisher_msg(
                input_msg=set_joint_positions_req,
                control_type="position_control",
                control_group="hand",
            )
        except InputMessageInvalid as e:

            # Print warning message and return result
            logwarn_msg = "Panda hand joint positions not set as " + lower_first_char(
                e.log_message
            )
            rospy.logwarn(logwarn_msg)
            resp.success = False
            resp.message = e.message
            return resp

        # Check if required joints are running
        if len(stopped_controllers) >= 1:

            # Set controllers missing boolean
            controllers_missing = True

            # Check if these controllers are required for the current command
            joint_controllers_dict = self._get_joint_controllers()
            req_missing_controllers = []
            if not set_joint_positions_req.joint_names:
                log_warning = True
                req_missing_controllers = flatten_list(stopped_controllers)
            else:
                for joint in set_joint_positions_req.joint_names:
                    if joint in joint_controllers_dict.keys():
                        req_missing_controllers.append(
                            [
                                stopped_controller
                                for stopped_controller in flatten_list(
                                    stopped_controllers
                                )
                                if stopped_controller in joint_controllers_dict[joint]
                            ]
                        )
                req_missing_controllers = flatten_list(req_missing_controllers)
                if req_missing_controllers:
                    log_warning = True
                else:
                    log_warning = False

            # Log warning
            if log_warning:
                rospy.logwarn(
                    "Panda hand joint positions command send but probably not executed "
                    "as the %s %s not running."
                    % (
                        stopped_controllers,
                        "joint position controller is"
                        if len(stopped_controllers) == 1
                        else "joint position controllers are",
                    )
                )

        # Save position control setpoint
        joint_positions_setpoint_dict = {
            group: [command.data for command in control_list]
            for (group, control_list) in control_pub_msgs.items()
        }
        self.joint_positions_setpoint = joint_positions_setpoint_dict

        # Publish request
        rospy.logdebug("Publishing Panda hand joint positions control message.")
        self._hand_position_pub.publish(control_pub_msgs["hand"])

        # Wait till control is finished or timeout has been reached
        if set_joint_positions_req.wait.data and not controllers_missing:
            self._wait_till_done(
                control_type="position_control",
                control_group="hand",
                controlled_joints=controlled_joints,
            )

        # Return success message
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _hand_set_joint_efforts_callback(self, set_joint_efforts_req):
        """Request hand joint effort control.

        Parameters
        ----------
        set_joint_efforts_req : panda_training.srv.SetJointEffortsRequest
            Service request message specifying the efforts for the robot hand joints.

        Returns
        -------
        panda_training.srv.SetJointEffortsResponse
            Service response.
        """

        # Check if set_joint_efforts_req.joint_names contains duplicates
        duplicate_list = get_duplicate_list(set_joint_efforts_req.joint_names)
        if duplicate_list:

            # Print warning
            rospy.logwarn(
                "Multiple entries were found for %s '%s' in the '%s' message. "
                "Consequently, only the last occurrence was used in setting the joint "
                "efforts."
                % (
                    "joints" if len(duplicate_list) > 1 else "joint",
                    duplicate_list,
                    "%s/panda_hand/set_joint_efforts" % rospy.get_name(),
                )
            )

        # Create required variables and messages
        controllers_missing = False
        resp = SetJointEffortsResponse()

        # Retrieve controller information
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )

        # Check if all controllers are available and running
        stopped_controllers = []
        missing_controllers = []
        for effort_controller in self.hand_effort_controllers:
            try:
                if self._controllers[effort_controller].state != "running":
                    stopped_controllers.append(effort_controller)
            except KeyError:
                missing_controllers.append(effort_controller)

        # Return failed result if we miss a controller
        if len(missing_controllers) >= 1:
            rospy.logwarn(
                "Panda hand joint efforts command could not be send as the %s %s "
                "not initialized. Please make sure you load the controller parameters "
                "onto the ROS parameter server."
                % (
                    missing_controllers,
                    "joint effort controller is"
                    if len(missing_controllers) == 1
                    else "joint effort controllers are",
                )
            )
            resp.success = False
            resp.message = "Hand controllers not initialised."
            return resp

        # Validate request and create control publisher message
        try:
            control_pub_msgs, controlled_joints = self._create_control_publisher_msg(
                input_msg=set_joint_efforts_req,
                control_type="effort_control",
                control_group="hand",
            )
        except InputMessageInvalid as e:

            # Print warning message and return result
            logwarn_msg = "Panda hand joint efforts not set as " + lower_first_char(
                e.log_message
            )
            rospy.logwarn(logwarn_msg)
            resp.success = False
            resp.message = e.message
            return resp

        # Check if required controllers are running
        if len(stopped_controllers) >= 1:

            # Set controllers missing boolean
            controllers_missing = True

            # Check if these controllers are required for the current command
            joint_controllers_dict = self._get_joint_controllers()
            req_missing_controllers = []
            if not set_joint_efforts_req.joint_names:
                log_warning = True
                req_missing_controllers = flatten_list(stopped_controllers)
            else:
                for joint in set_joint_efforts_req.joint_names:
                    if joint in joint_controllers_dict.keys():
                        req_missing_controllers.append(
                            [
                                stopped_controller
                                for stopped_controller in flatten_list(
                                    stopped_controllers
                                )
                                if stopped_controller in joint_controllers_dict[joint]
                            ]
                        )
                req_missing_controllers = flatten_list(req_missing_controllers)
                if req_missing_controllers:
                    log_warning = True
                else:
                    log_warning = False

            # Log warning
            if log_warning:
                rospy.logwarn(
                    "Panda hand joint efforts command send but probably not executed "
                    "as the %s %s not running."
                    % (
                        stopped_controllers,
                        "joint effort controller is"
                        if len(stopped_controllers) == 1
                        else "joint effort controllers are",
                    )
                )

        # Save effort control setpoint
        joint_efforts_setpoint_dict = {
            group: [command.data for command in control_list]
            for (group, control_list) in control_pub_msgs.items()
        }
        self.joint_efforts_setpoint = joint_efforts_setpoint_dict

        # Publish request
        rospy.logdebug("Publishing Panda hand joint efforts control message.")
        self._hand_effort_pub.publish(control_pub_msgs["hand"])

        # Wait till control is finished or timeout has been reached
        if set_joint_efforts_req.wait.data and not controllers_missing:
            self._wait_till_done(
                control_type="effort_control",
                control_group="hand",
                controlled_joints=controlled_joints,
            )

        # Return service response
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _switch_control_type_callback(self, switch_control_type_req):
        """Request the controller type to switch from individual joint control to joint
        group control.

        Parameters
        ----------
        switch_control_type_req : panda_training.srv.SwitchControlTypeRequest
            Service request message to switch the control_type.

        Returns
        -------
        panda_training.srv.SwitchControlTypeResponse
            Service response.
        """

        # Switch use group controller bool
        self.use_group_controller = not self.use_group_controller

        # Switch controller publishers
        if not self.use_group_controller:
            self._arm_position_pub = self._arm_joint_position_pub
            self._arm_effort_pub = self._arm_joint_effort_pub
            self._hand_position_pub = self._hand_joint_position_pub
            self._hand_effort_pub = self._hand_joint_effort_pub
            self.arm_position_controllers = ARM_POSITION_CONTROLLERS
            self.arm_effort_controllers = ARM_EFFORT_CONTROLLERS
            self.hand_position_controllers = HAND_POSITION_CONTROLLERS
            self.hand_effort_controllers = HAND_EFFORT_CONTROLLERS
            self._arm_position_controller_msg_type = Float64
            self._arm_effort_controller_msg_type = Float64
            self._hand_position_controller_msg_type = Float64
            self._hand_effort_controller_msg_type = Float64
        else:
            self._arm_position_pub = self._arm_joint_positions_group_pub
            self._arm_effort_pub = self._arm_joint_efforts_group_pub
            self._hand_position_pub = self._hand_joint_positions_group_pub
            self._hand_effort_pub = self._hand_joint_efforts_group_pub
            self.arm_position_controllers = ARM_POSITION_GROUP_CONTROLLERS
            self.arm_effort_controllers = ARM_EFFORT_GROUP_CONTROLLERS
            self.hand_position_controllers = HAND_POSITION_GROUP_CONTROLLERS
            self.hand_effort_controllers = HAND_EFFORT_GROUP_CONTROLLERS
            self._arm_position_controller_msg_type = Float64MultiArray
            self._arm_effort_controller_msg_type = Float64MultiArray
            self._hand_position_controller_msg_type = Float64MultiArray
            self._hand_effort_controller_msg_type = Float64MultiArray

        # Return success bool
        resp = SwitchControlTypeResponse()
        resp.success = True
        return resp

    def _list_control_type_callback(self, list_control_type_req):
        """Returns the current controller type the 'panda_control_server' is using.

        Parameters
        ----------
        list_control_type_req : panda_training.srv.ListControlTypeRequest
            Service request message to list the control_type.

        Returns
        -------
        panda_training.srv.ListControlTypeResponse
            Service response. Options: 'joint_group_control' and 'joint_control'.
        """

        # Check if verbose was set to True
        verbose = list_control_type_req.verbose

        # Create response message
        resp = ListControlTypeResponse()

        # Get current control type
        if self.use_group_controller:
            if verbose:
                rospy.loginfo(
                    "'%s' is currently using the group controllers to control "
                    "the Panda robot" % rospy.get_name()
                )
            resp.control_type = "joint_group_control"
        else:
            if verbose:
                rospy.loginfo(
                    "'%s' is currently using the individual joint controllers to "
                    "control the Panda robot" % rospy.get_name()
                )
            resp.control_type = "joint_control"

        # Return control type
        return resp

    def _joints_callback(self, data):
        """Callback function for the joint data subscriber.
        """

        # Update joint_states
        self.joint_states = data


#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Initiate ROS nodetypes
    rospy.init_node("panda_control_server")

    # Get ROS parameters
    try:  # Check end effector
        use_group_controller = rospy.get_param("~use_group_controller")
    except KeyError:
        use_group_controller = False

    # Start control server
    control_server = PandaControlServer(use_group_controller=use_group_controller)
    rospy.spin()  # Maintain the service open.
