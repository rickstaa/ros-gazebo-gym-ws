#! /usr/bin/env python
"""A simple server for sending control commands to the controllers
of the Panda robot."""

# Main python imports
import sys
import numpy as np
from functions import controller_list_array_2_dict, lower_first_char
from panda_exceptions import InputMessageInvalid
from group_publisher import GroupPublisher

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

# TODO: Created combined joint_position publisher

#################################################
# Joint Group Position Controller class #########
#################################################
class PandaControlServer(object):
    """Controller server used to send control commands to the simulated Panda Robot.

    Attributes
    ----------
    joint_positions_setpoint : list
        The current joint positions setpoint.
    joint_efforts_setpoint : list
        The current joint positions setpoint.
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
        self._wait_till_done_timeout = rospy.Duration(3)
        self._joint_efforts_grad_threshold = 0.01
        self._joint_positions_grad_threshold = 0.01
        self._joint_state_topic = "/joint_states"

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

        # Retrieve current robot joint state and effort information
        self.joints = None
        while self.joints is None and not rospy.is_shutdown():
            try:
                self.joints = rospy.wait_for_message(
                    self._joint_state_topic, JointState, timeout=1.0
                )

                # Set joint setpoint to current position
                self.joint_positions_setpoint = list(self.joints.position)
                self.joint_efforts_setpoint = list(self.joints.effort)
            except ROSException:
                rospy.logwarn(
                    "Current /joint_states not ready yet, retrying for getting %s"
                    % self._joint_state_topic
                )

        # Create joint_state subscriber
        self.joint_states_sub = rospy.Subscriber(
            self._joint_state_topic, JointState, self._joints_callback
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

        # Retrieve informations about the controllers
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )

        # Create PandaControl services
        rospy.loginfo("Creating '%s' services." % rospy.get_name())
        rospy.logdebug(
            "Creating '%s/panda_arm/set_joint_positions' service." % rospy.get_name()
        )
        self.set_arm_joint_positions_srv = rospy.Service(
            "%s/panda_arm/set_joint_positions" % rospy.get_name()[1:],
            SetJointPositions,
            self._arm_set_joint_positions_callback,
        )
        rospy.logdebug(
            "Creating '%s/panda_arm/set_joint_efforts' service." % rospy.get_name()
        )
        self.set_arm_joint_efforts_srv = rospy.Service(
            "%s/panda_arm/set_joint_efforts" % rospy.get_name()[1:],
            SetJointEfforts,
            self._arm_set_joint_efforts_callback,
        )
        rospy.logdebug(
            "Creating '%s/panda_hand/set_joint_positions' service." % rospy.get_name()
        )
        self.set_hand_joint_positions_srv = rospy.Service(
            "%s/panda_hand/set_joint_positions" % rospy.get_name()[1:],
            SetJointPositions,
            self._hand_set_joint_positions_callback,
        )
        rospy.logdebug(
            "Creating '%s/panda_hand/set_joint_efforts' service." % rospy.get_name()
        )
        self.set_hand_joint_efforts_srv = rospy.Service(
            "%s/panda_hand/set_joint_efforts" % rospy.get_name()[1:],
            SetJointEfforts,
            self._hand_set_joint_efforts_callback,
        )
        rospy.logdebug("Creating '%s/switch_control_type' service." % rospy.get_name())
        self.switch_control_type_srv = rospy.Service(
            "%s/switch_control_type" % rospy.get_name()[1:],
            SwitchControlType,
            self._switch_control_type_callback,
        )
        rospy.logdebug("Creating '%s/list_control_type' service." % rospy.get_name())
        self.list_control_type_srv = rospy.Service(
            "%s/list_control_type" % rospy.get_name()[1:],
            ListControlType,
            self._list_control_type_callback,
        )
        rospy.loginfo("'%s' services created successfully." % rospy.get_name())

    ###############################################
    # Panda control member functions ##############
    ###############################################
    def _wait_till_done(self, control_type, timeout=None):
        """Wait control is finished. Meaning the robot state is within range of the
        Joint position and joint effort setpoints.

        Parameters
        ----------
        control_type : str
            The type of control that is being executed and on which we should wait.
            Options are 'effort_control' and 'position_control'.
        connection_timeout : int, optional
            The timeout when waiting for the control to be done, by default
            self._wait_till_done_timeout.
        """

        # Validate control type and control group
        if control_type.lower() not in ["position_control", "effort_control"]:
            rospy.logwarn(
                "Please specify a valid control type. Valid values are %s."
                % ("['position_control', 'effort_control']")
            )
            return False

        # Get set input arguments
        if not None:  # If not supplied
            timeout = self._wait_till_done_timeout
        else:
            timeout = rospy.Duration(timeout)

        # Wait till robot positions/efforts are not changing anymore
        # NOTE: We have to use the std to determine whether the control was finished
        # as the velocity in the joint_states topic is wrong (see issue 14)
        # TODO: Can be changed to /joint_states.velocity if #14 is fixed.
        timeout_time = rospy.get_rostime() + timeout
        positions_buffer = np.full((2, len(self.joints.position)), np.nan)
        positions_grad = np.full((2, len(self.joints.position)), np.nan)
        efforts_buffer = np.full((2, len(self.joints.effort)), np.nan)
        efforts_grad = np.full((2, len(self.joints.effort)), np.nan)
        while not rospy.is_shutdown() and rospy.get_rostime() < timeout_time:

            # Wait till joint positions are within range or arm not changing anymore
            if control_type.lower() == "position_control":

                # Add state to position to buffer
                positions_buffer = np.append(
                    positions_buffer, [self.joints.position], axis=0
                )
                np.delete(positions_buffer, 0, axis=0)  # Delete oldest entry
                positions_grad = np.gradient(positions_buffer, axis=0)

                # Check if joint states are within setpoints
                if (
                    np.linalg.norm(
                        np.array(self.joints.position)
                        - np.array(self.joint_positions_setpoint)
                    )
                    <= self.joint_positions_threshold
                ) or all(
                    [
                        (
                            np.abs(val) <= self._joint_positions_grad_threshold
                            and val != 0.0
                        )
                        for val in positions_grad[-1]
                    ]
                ):  # Check if difference norm is within threshold
                    break

            # Check if joint effort states are within setpoint
            elif control_type.lower() == "effort_control":

                # Add state to position to buffer
                efforts_buffer = np.append(
                    efforts_buffer, [self.joints.position], axis=0
                )
                np.delete(efforts_buffer, 0, axis=0)  # Delete oldest entry
                efforts_grad = np.gradient(efforts_buffer, axis=0)

                if (
                    np.linalg.norm(
                        np.array(self.joints.effort)
                        - np.array(self.joint_efforts_setpoint)
                    )
                    <= self.joint_efforts_threshold
                ) or all(
                    [
                        (
                            np.abs(val) <= self._joint_efforts_grad_threshold
                            and val != 0.0
                        )
                        for val in efforts_grad[-1]
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
            The robot control group which is being controlled. Options are "arm" or
            "hand".
        verbose : bool
            Bool specifying whether you want to send a warning message to the ROS
            logger.

        Returns
        -------
        list
            A list containing the control commands for each joint in the order which is
            are required by the publishers

        Raises
        ----------
        panda_training.exceptions.InputMessageInvalid
            Raised when the input_msg could not be converted into 'moveit_commander'
            arm hand joint position commands.
        """

        # Validate control type and control group
        if control_group.lower() not in ["arm", "hand"]:

            # Log message and return result
            logwarn_message = (
                "Control group '%s' does not exist. Please specify a valid control "
                "group. Valid values are %s."
                % (control_group.lower(), "['arm', 'hand']")
            )
            if verbose:
                rospy.logwarn(logwarn_message)
            raise InputMessageInvalid(
                message="Control_group '%s' invalid." % control_group.lower(),
                log_message=logwarn_message,
            )

        # Extract input message and get controller information
        if control_type.lower() == "position_control":

            # Extract input message
            joint_names = input_msg.joint_names
            control_input = input_msg.joint_positions.data

            # Get controller information
            used_controllers = (
                self.hand_position_controllers
                if control_group == "hand"
                else self.arm_position_controllers
            )
            controlled_joints = []
            for controller in used_controllers:
                for claimed_resources in self._controllers[
                    controller
                ].claimed_resources:
                    for resource in claimed_resources.resources:
                        controlled_joints.append(resource)
            controlled_joints_size = len(controlled_joints)
            control_msgs_type = (
                self._hand_position_controller_msg_type
                if control_group.lower() == "hand"
                else self._arm_position_controller_msg_type
            )
        elif control_type.lower() == "effort_control":

            # Extract input message
            joint_names = input_msg.joint_names
            control_input = input_msg.joint_efforts.data

            # Get controller information
            used_controllers = (
                self.hand_effort_controllers
                if control_group == "hand"
                else self.arm_effort_controllers
            )
            controlled_joints = []
            for controller in used_controllers:
                for claimed_resources in self._controllers[
                    controller
                ].claimed_resources:
                    for resource in claimed_resources.resources:
                        controlled_joints.append(resource)
            controlled_joints_size = len(controlled_joints)
            control_msgs_type = (
                self._hand_effort_controller_msg_type
                if control_group.lower() == "hand"
                else self._arm_effort_controller_msg_type
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
                message="Control_type '%s' invalid." % control_type.lower(),
                log_message=logwarn_message,
            )

        # Check service request input
        if len(joint_names) == 0:

            # Check if enough joint position commands were given
            if len(control_input) != controlled_joints_size:

                # Create log message
                if control_type.lower() == "position_control":
                    logwarn_msg_strings = [
                        "joint position"
                        if len(control_input) == 1
                        else "joint positions",
                        "joint" if controlled_joints_size == 1 else "joints",
                    ]
                else:
                    logwarn_msg_strings = [
                        "joint effort" if len(control_input) == 1 else "joint efforts",
                        "joint effort"
                        if controlled_joints_size == 1
                        else "joint efforts",
                    ]

                # Log message and return result
                logwarn_message = (
                    "You specified %s while the Panda %s control group has %s."
                    % (
                        "%s %s" % (len(control_input), logwarn_msg_strings[0]),
                        control_group,
                        "%s %s" % (controlled_joints_size, logwarn_msg_strings[1]),
                    )
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
            else:

                # Set control_commands equal to the control input as no joint_names
                # were given.
                control_commands = control_input
        else:

            # Check if enough control values were given
            if len(joint_names) != len(control_input):

                # Create log message
                if control_type.lower() == "position_control":
                    logwarn_msg_strings = [
                        "joint position"
                        if len(control_input) == 1
                        else "joint positions",
                        "joint" if len(joint_names) == 1 else "joints",
                        "joint position",
                        "panda_training/SetJointPositions",
                    ]
                else:
                    logwarn_msg_strings = [
                        "joint effort" if len(control_input) == 1 else "joint efforts",
                        "joint effort" if len(joint_names) == 1 else "joint efforts",
                        "joint effort",
                        "panda_training/SetJointEfforts",
                    ]

                # Send log message
                logwarn_message = (
                    "You specified %s while the 'joint_names' field of the "
                    "'%s' message contains %s. Please make sure you supply "
                    "a %s for each joint contained in the 'joint_names' field."
                    % (
                        "%s %s" % (len(control_input), logwarn_msg_strings[0]),
                        logwarn_msg_strings[3],
                        "%s %s" % (len(joint_names), logwarn_msg_strings[1]),
                        logwarn_msg_strings[2],
                    )
                )
                if verbose:
                    rospy.logwarn(logwarn_message)
                raise InputMessageInvalid(
                    message="Joint_names and joint_positions fields of the input "
                    "message are of different lengths.",
                    log_message=logwarn_message,
                    details={
                        "joint_positions_command_length": len(control_input),
                        "joint_names_length": len(joint_names),
                    },
                )

            # Validate joint_names
            invalid_joint_names = [
                joint_name
                for joint_name in joint_names
                if joint_name not in controlled_joints
            ]
            if len(invalid_joint_names) != 0:

                # Send log message
                logwarn_message = (
                    "%s %s that %s specified in the 'joint_names' field of the "
                    "'panda_training/SetJointPositions' message %s invalid. Valid "
                    "joint names are %s."
                    % (
                        "Joint" if len(invalid_joint_names) == 1 else "Joints",
                        invalid_joint_names,
                        "was" if len(invalid_joint_names) == 1 else "were",
                        "was" if len(invalid_joint_names) == 1 else "were",
                        controlled_joints,
                    )
                )
                if verbose:
                    rospy.logwarn(logwarn_message)
                raise InputMessageInvalid(
                    message="Invalid joint_names were given.",
                    log_message=logwarn_message,
                    details={"invalid_joint_names": invalid_joint_names},
                )
            else:

                # Fill joint positions message based on joint_names
                # FIX joint order
                control_commands = []
                for joint in controlled_joints:
                    try:
                        index = joint_names.index(joint)
                        control_commands.append(control_input[index])
                    except ValueError:
                        index = controlled_joints.index(joint)
                        control_commands.append(self.joints.position[index])

        # Create and return control publisher message
        if not self.use_group_controller:
            pub_msg = [control_msgs_type(item) for item in control_commands]
        else:
            pub_msg = control_msgs_type(data=control_commands)
        return pub_msg

    ###############################################
    # Service callback functions ##################
    ###############################################
    def _arm_set_joint_positions_callback(self, joint_positions_req):
        """Request arm joint position control.

        Parameters
        ----------
        joint_positions_req : panda_training.srv.SetJointPositionsRequest
            Service request message specifying the positions for the robot arm joints.

        Returns
        -------
        panda_training.srv.SetJointPositionsResponse
            Service response.
        """

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
            logwarn_msg_string = (
                "joint position controller is"
                if len(missing_controllers) == 1
                else "joint position controllers are"
            )
            rospy.logwarn(
                "Panda arm joint position command could not be send as the %s %s "
                "not initialized. Please make sure you load the controller parameters "
                "onto the ROS parameter server."
                % (missing_controllers, logwarn_msg_string)
            )
            resp.success = False
            resp.message = "Arm controllers not initialised."
            return resp

        # Validate request and create control publisher message
        try:
            control_pub_msgs = self._create_control_publisher_msg(
                input_msg=joint_positions_req,
                control_type="position_control",
                control_group="arm",
            )
        except InputMessageInvalid as e:

            # Print warning message and return result
            logwarn_message = (
                "Panda arm joint positions not set as "
                + lower_first_char(e.log_message)
            )
            rospy.logwarn(logwarn_message)
            resp.success = False
            resp.message = e.message
            return resp

        # Check if required joints are running
        if len(stopped_controllers) >= 1:
            logwarn_msg_string = (
                "joint position controller is"
                if len(stopped_controllers) == 1
                else "joint position controllers are"
            )
            rospy.logwarn(
                "Panda arm joint positions command send but probably not executed as "
                "the %s %s not running." % (stopped_controllers, logwarn_msg_string)
            )
            controllers_missing = True

        # Save position control setpoint
        self.joint_positions_setpoint[0:2] = list(
            self.joints.position[0:2]
        )  # Set Panda hand positions
        if not self.use_group_controller:
            self.joint_positions_setpoint[2:] = [
                item.data for item in control_pub_msgs
            ]  # Arm positions
        else:
            self.joint_positions_setpoint[2:] = [
                item for item in control_pub_msgs.data
            ]  # Arm positions

        # Publish request
        rospy.logdebug("Publishing Panda arm joint positions control message.")
        self._arm_position_pub.publish(control_pub_msgs)

        # Wait till control is finished or timeout has been reached
        if joint_positions_req.wait.data and not controllers_missing:
            self._wait_till_done("position_control")

        # Return success message
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _arm_set_joint_efforts_callback(self, joint_efforts_req):
        """Request arm Joint effort control.

        Parameters
        ----------
        joint_efforts_req : panda_training.srv.SetJointEffortsRequest
            Service request message specifying the efforts for the robot arm joints.

        Returns
        -------
        panda_training.srv.SetJointPositionsResponse
            Service response.
        """

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
            logwarn_msg_string = (
                "joint effort controller is"
                if len(missing_controllers) == 1
                else "joint effort controllers are"
            )
            rospy.logwarn(
                "Panda arm joint effort command could not be send as the %s %s "
                "not initialized. Please make sure you load the controller parameters "
                "onto the ROS parameter server."
                % (missing_controllers, logwarn_msg_string)
            )
            resp.success = False
            resp.message = "Arm controllers not initialised."
            return resp

        # Validate request and create control publisher message
        try:
            control_pub_msgs = self._create_control_publisher_msg(
                input_msg=joint_efforts_req,
                control_type="effort_control",
                control_group="arm",
            )
        except InputMessageInvalid as e:

            # Print warning message and return result
            logwarn_message = "Panda arm joint efforts not set as " + lower_first_char(
                e.log_message
            )
            rospy.logwarn(logwarn_message)
            resp.success = False
            resp.message = e.message
            return resp

        # Check if required controllers are running
        if len(stopped_controllers) >= 1:
            logwarn_msg_string = (
                "joint effort controller is"
                if len(stopped_controllers) == 1
                else "joint effort controllers are"
            )
            rospy.logwarn(
                "Panda arm joint efforts command send but probably not executed as "
                "the %s %s not running." % (stopped_controllers, logwarn_msg_string)
            )
            controllers_missing = True

        # Save effort control setpoint
        self.joint_efforts_setpoint[0:2] = list(
            self.joints.effort[0:2]
        )  # Set Panda hand efforts
        if not self.use_group_controller:
            self.joint_efforts_setpoint[2:] = [
                item.data for item in control_pub_msgs
            ]  # Arm efforts
        else:
            self.joint_efforts_setpoint[2:] = [
                item for item in control_pub_msgs.data
            ]  # Arm efforts

        # Publish request
        rospy.logdebug("Publishing Panda arm joint efforts control message.")
        self._arm_effort_pub.publish(control_pub_msgs)

        # Wait till control is finished or timeout has been reached
        if joint_efforts_req.wait.data and not controllers_missing:
            self._wait_till_done(control_type="effort_control")

        # Return service response
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _hand_set_joint_positions_callback(self, joint_positions_req):
        """Request hand joint position control

        Parameters
        ----------
        joint_positions_req : panda_training.srv.SetJointPositionsRequest
            Service request message specifying the positions for the robot hand joints.

        Returns
        -------
        panda_training.srv.SetJointPositionsResponse
            Service response.
        """

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
            logwarn_msg_string = (
                "joint position controller is"
                if len(missing_controllers) == 1
                else "joint position controllers are"
            )
            rospy.logwarn(
                "Panda hand joint position command could not be send as the %s %s "
                "not initialized. Please make sure you load the controller parameters "
                "onto the ROS parameter server."
                % (missing_controllers, logwarn_msg_string)
            )
            resp.success = False
            resp.message = "Hand controllers not initialised."
            return resp

        # Validate request and create control publisher message
        try:
            control_pub_msgs = self._create_control_publisher_msg(
                input_msg=joint_positions_req,
                control_type="position_control",
                control_group="hand",
            )
        except InputMessageInvalid as e:

            # Print warning message and return result
            logwarn_message = (
                "Panda hand joint positions not set as "
                + lower_first_char(e.log_message)
            )
            rospy.logwarn(logwarn_message)
            resp.success = False
            resp.message = e.message
            return resp

        # Check if required joints are running
        if len(stopped_controllers) >= 1:
            logwarn_msg_string = (
                "joint position controller is"
                if len(stopped_controllers) == 1
                else "joint position controllers are"
            )
            rospy.logwarn(
                "Panda hand joint positions command send but probably not executed as "
                "the %s %s not running." % (stopped_controllers, logwarn_msg_string)
            )
            controllers_missing = True

        # Save position control setpoint
        self.joint_positions_setpoint[2:] = list(
            self.joints.position[2:]
        )  # Set Panda hand positions
        if not self.use_group_controller:
            self.joint_positions_setpoint[:2] = [
                item.data for item in control_pub_msgs
            ]  # Arm positions
        else:
            self.joint_positions_setpoint[:2] = [
                item for item in control_pub_msgs.data
            ]  # Arm positions

        # Publish request
        rospy.logdebug("Publishing Panda hand joint positions control message.")
        self._hand_position_pub.publish(control_pub_msgs)

        # Wait till control is finished or timeout has been reached
        if joint_positions_req.wait.data and not controllers_missing:
            self._wait_till_done("position_control")

        # Return success message
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _hand_set_joint_efforts_callback(self, joint_efforts_req):
        """Request hand joint effort control.

        Parameters
        ----------
        joint_efforts_req : panda_training.srv.SetJointEffortsRequest
            Service request message specifying the efforts for the robot hand joints.

        Returns
        -------
        panda_training.srv.SetJointPositionsResponse
            Service response.
        """

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
            logwarn_msg_string = (
                "joint effort controller is"
                if len(missing_controllers) == 1
                else "joint effort controllers are"
            )
            rospy.logwarn(
                "Panda hand joint efforts command could not be send as the %s %s "
                "not initialized. Please make sure you load the controller parameters "
                "onto the ROS parameter server."
                % (missing_controllers, logwarn_msg_string)
            )
            resp.success = False
            resp.message = "Hand controllers not initialised."
            return resp

        # Validate request and create control publisher message
        try:
            control_pub_msgs = self._create_control_publisher_msg(
                input_msg=joint_efforts_req,
                control_type="effort_control",
                control_group="hand",
            )
        except InputMessageInvalid as e:

            # Print warning message and return result
            logwarn_message = "Panda hand joint efforts not set as " + lower_first_char(
                e.log_message
            )
            rospy.logwarn(logwarn_message)
            resp.success = False
            resp.message = e.message
            return resp

        # Check if required controllers are running
        if len(stopped_controllers) >= 1:
            logwarn_msg_string = (
                "joint effort controller is"
                if len(stopped_controllers) == 1
                else "joint effort controllers are"
            )
            rospy.logwarn(
                "Panda hand joint efforts command send but probably not executed as "
                "the %s %s not running." % (stopped_controllers, logwarn_msg_string)
            )
            controllers_missing = True

        # Save effort control setpoint
        self.joint_efforts_setpoint[2:] = list(
            self.joints.effort[2:]
        )  # Set Panda hand efforts
        if not self.use_group_controller:
            self.joint_efforts_setpoint[:2] = [
                item.data for item in control_pub_msgs
            ]  # Arm efforts
        else:
            self.joint_efforts_setpoint[:2] = [
                item for item in control_pub_msgs.data
            ]  # Arm efforts

        # Publish request
        rospy.logdebug("Publishing Panda hand joint efforts control message.")
        self._hand_effort_pub.publish(control_pub_msgs)

        # Wait till control is finished or timeout has been reached
        if joint_efforts_req.wait.data and not controllers_missing:
            self._wait_till_done(control_type="effort_control")

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

        # Create response message
        resp = ListControlTypeResponse()

        # Get current control type
        if self.use_group_controller:
            rospy.loginfo(
                "'%s' is currently using the group controllers to control "
                "the Panda robot" % rospy.get_name()
            )
            resp.control_type = "joint_group_control"
        else:
            rospy.loginfo(
                "'%s' is currently using the individual joint controllers to control "
                "the Panda robot" % rospy.get_name()
            )
            resp.control_type = "joint_control"

        # Return control type
        return resp

    def _joints_callback(self, data):
        """Callback function for the joint data subscriber.
        """

        # Update joint_states
        self.joints = data


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
    server = PandaControlServer(use_group_controller=use_group_controller)
    rospy.spin()  # Maintain the service open.
