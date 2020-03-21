#! /usr/bin/env python
"""A simple server for sending control commands to the controllers
of the panda robot."""

# Main python imports
import sys
import numpy as np
from functions import controller_list_array_2_dict
from group_publisher import GroupPublisher

# ROS python imports
import rospy
from rospy.exceptions import ROSException, ROSInterruptException

# ROS msgs and srvs
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from panda_training.srv import (
    setJointPositions,
    setJointPositionsResponse,
    setJointEfforts,
    setJointEffortsResponse,
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


#################################################
# Joint Group Position Controller class #########
#################################################
class PandaControlServer(object):
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
        self.joint_state_topic = "/joint_states"
        self.joint_positions_threshold = 0.01
        self.joint_positions_grad_threshold = 0.01
        self.joint_efforts_threshold = 0.01
        self.joint_efforts_grad_threshold = 0.01
        self.wait_till_done_timeout = rospy.Duration(3)
        self.use_group_controller = use_group_controller

        # Check which controllers should be used
        if not self.use_group_controller:
            self.arm_position_controllers = ARM_POSITION_CONTROLLERS
            self.arm_effort_controllers = ARM_EFFORT_CONTROLLERS
            self.hand_position_controllers = HAND_POSITION_CONTROLLERS
            self.hand_effort_controllers = HAND_EFFORT_CONTROLLERS
            self._arm_position_controller_msg_type = Float64
            self._arm_effort_controller_msg_type = Float64
            self._hand_position_controller_msg_type = Float64
            self._hand_effort_controller_msg_type = Float64
        else:
            self.arm_position_controllers = ARM_POSITION_GROUP_CONTROLLERS
            self.arm_effort_controllers = ARM_EFFORT_GROUP_CONTROLLERS
            self.hand_position_controllers = HAND_POSITION_GROUP_CONTROLLERS
            self.hand_effort_controllers = HAND_EFFORT_GROUP_CONTROLLERS
            self._arm_position_controller_msg_type = Float64MultiArray
            self._arm_effort_controller_msg_type = Float64MultiArray
            self._hand_position_controller_msg_type = Float64MultiArray
            self._hand_effort_controller_msg_type = Float64MultiArray

        # Create arm joint position publishers
        self._arm_position_pub = GroupPublisher()
        for effort_controller in self.arm_position_controllers:
            self._arm_position_pub.append(
                rospy.Publisher(
                    "%s/command" % effort_controller,
                    self._arm_position_controller_msg_type,
                    queue_size=10,
                )
            )

        # Create arm joint effort publishers
        self._arm_effort_pub = GroupPublisher()
        for effort_controller in self.arm_effort_controllers:
            self._arm_effort_pub.append(
                rospy.Publisher(
                    "%s/command" % effort_controller,
                    self._arm_effort_controller_msg_type,
                    queue_size=10,
                )
            )

        # Create hand joint position publishers
        self._hand_position_pub = GroupPublisher()
        for position_controller in self.hand_position_controllers:
            self._hand_position_pub.append(
                rospy.Publisher(
                    "%s/command" % position_controller,
                    self._hand_position_controller_msg_type,
                    queue_size=10,
                )
            )

        # Create hand joint effort publishers
        self._hand_effort_pub = GroupPublisher()
        for effort_controller in self.hand_effort_controllers:
            self._hand_effort_pub.append(
                rospy.Publisher(
                    "%s/command" % effort_controller,
                    self._hand_effort_controller_msg_type,
                    queue_size=10,
                )
            )

        # Retrieve current robot joint state and effort information
        self.joints = None
        while self.joints is None and not rospy.is_shutdown():
            try:
                self.joints = rospy.wait_for_message(
                    self.joint_state_topic, JointState, timeout=1.0
                )

                # Set joint setpoint to current position
                self.joint_positions_setpoint = list(self.joints.position)
                self.joint_efforts_setpoint = list(self.joints.effort)
            except ROSException:
                rospy.logwarn(
                    "Current /joint_states not ready yet, retrying for getting %s"
                    % self.joint_state_topic
                )

        # Create joint_state subscriber
        self.joint_states_sub = rospy.Subscriber(
            self.joint_state_topic, JointState, self._joints_callback
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

        # Set control input spaces
        try:
            self._arm_joint_position_input_size = len(
                self._controllers["panda_arm_joint_group_position_controller"]
                .claimed_resources[0]
                .resources
            )
        except KeyError:
            self._arm_joint_position_input_size = 7
        try:
            self._arm_joint_effort_input_size = len(
                self._controllers["panda_arm_joint_group_effort_controller"]
                .claimed_resources[0]
                .resources
            )
        except KeyError:
            self._arm_joint_effort_input_size = 7
        try:
            self._hand_joint_position_input_size = len(
                self._controllers["panda_hand_joint_group_position_controller"]
                .claimed_resources[0]
                .resources
            )
        except KeyError:
            self._hand_joint_position_input_size = 2
        try:
            self._hand_joint_effort_input_size = len(
                self._controllers["panda_hand_joint_group_effort_controller"]
                .claimed_resources[0]
                .resources
            )
        except KeyError:
            self._hand_joint_effort_input_size = 2

        # Create PandaControl services
        rospy.loginfo("Creating '%s' services." % rospy.get_name())
        rospy.logdebug(
            "Creating '%s/panda_arm/set_joint_positions' service." % rospy.get_name()
        )
        self.set_arm_joint_positions_srv = rospy.Service(
            "%s/panda_arm/set_joint_positions" % rospy.get_name()[1:],
            setJointPositions,
            self._arm_set_joint_positions_callback,
        )
        rospy.logdebug(
            "Creating '%s/panda_arm/set_joint_efforts' service." % rospy.get_name()
        )
        self.set_arm_joint_efforts_srv = rospy.Service(
            "%s/panda_arm/set_joint_efforts" % rospy.get_name()[1:],
            setJointEfforts,
            self._arm_set_joint_efforts_callback,
        )
        rospy.logdebug(
            "Creating '%s/panda_hand/set_joint_positions' service." % rospy.get_name()
        )
        self.set_hand_joint_positions_srv = rospy.Service(
            "%s/panda_hand/set_joint_positions" % rospy.get_name()[1:],
            setJointPositions,
            self._hand_set_joint_positions_callback,
        )
        rospy.logdebug(
            "Creating '%s/panda_hand/set_joint_efforts' service." % rospy.get_name()
        )
        self.set_hand_joint_efforts_srv = rospy.Service(
            "%s/panda_hand/set_joint_efforts" % rospy.get_name()[1:],
            setJointEfforts,
            self._hand_set_joint_efforts_callback,
        )
        rospy.loginfo("'%s' services created successfully." % rospy.get_name())

    ###############################################
    # panda control member functions ##############
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
            The timeout when waiting for the control to be done.
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
            timeout = self.wait_till_done_timeout

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
                            np.abs(val) <= self.joint_positions_grad_threshold
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
                            np.abs(val) <= self.joint_efforts_grad_threshold
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

    def _validate_input_msg(
        self, input_msg, control_type, control_group, controlled_joints
    ):
        """Checks whether a given control input message is valid.

        Parameters
        ----------
        input_msg :
            The service input message we want to validate.
        control_type : str
            The type of control that is being executed and on which we should wait.
            Options are 'effort_control' and 'position_control'.
        control_group : str
            The robot group which is being controlled. Options are "arm" or "hand".
        controlled_joints : list
            List containing the joints which are being controlled.

        Returns
        -------
        bool
            A boolean specifying whether the message was valid.
        """

        # Validate control type and control group
        if control_type.lower() not in ["position_control", "effort_control"]:
            rospy.logwarn(
                "Please specify a valid control type. Valid values are %s."
                % ("['position_control', 'effort_control']")
            )
            return False
        if control_group.lower() not in ["arm", "hand"]:
            rospy.logwarn(
                "Please specify a valid control group. Valid values are %s."
                % ("['arm', 'hand']")
            )
            return False

        # Get control_command information out of input message
        if control_type.lower() == "position_control":
            control_input = input_msg.joint_positions.data
            control_input_size = (
                self._hand_joint_position_input_size
                if control_group.lower() == "hand"
                else self._arm_joint_position_input_size
            )
        else:
            control_input = input_msg.joint_efforts.data
            control_input_size = (
                self._hand_joint_effort_input_size
                if control_group.lower() == "hand"
                else self._arm_joint_effort_input_size
            )

        # Check service request input
        if len(input_msg.joint_names) == 0:

            # Check if enough joint position commands were given
            if len(control_input) != control_input_size:

                # Create log message
                if control_type.lower() == "position_control":
                    log_warn_msg_strings = [
                        "joint position"
                        if len(control_input) == 1
                        else "joint positions",
                        "joint" if control_input_size == 1 else "joints",
                    ]
                else:
                    log_warn_msg_strings = [
                        "joint effort" if len(control_input) == 1 else "joint efforts",
                        "joint effort" if control_input_size == 1 else "joint efforts",
                    ]

                # Log message and return result
                rospy.logwarn(
                    "You specified %s while the panda %s control group has %s."
                    % (
                        "%s %s" % (len(control_input), log_warn_msg_strings[0]),
                        control_group,
                        "%s %s" % (control_input_size, log_warn_msg_strings[1]),
                    )
                )
                return False
        else:

            # Check if enough control values were given
            if len(input_msg.joint_names) != len(control_input):

                # Create log message
                if control_type.lower() == "position_control":
                    log_warn_msg_strings = [
                        "joint position"
                        if len(control_input) == 1
                        else "joint positions",
                        "joint" if len(input_msg.joint_names) == 1 else "joints",
                        "joint position",
                        "panda_training/setJointPositions",
                    ]
                else:
                    log_warn_msg_strings = [
                        "joint effort" if len(control_input) == 1 else "joint efforts",
                        "joint effort"
                        if len(input_msg.joint_names) == 1
                        else "joint efforts",
                        "joint effort",
                        "panda_training/setJointEfforts",
                    ]

                # Send log message
                rospy.logwarn(
                    "You specified %s while the 'joint_names' field of the "
                    "'%s' message contains %s. Please make sure you supply "
                    "a %s for each joint contained in the 'joint_names' field."
                    % (
                        "%s %s" % (len(control_input), log_warn_msg_strings[0]),
                        log_warn_msg_strings[3],
                        "%s %s" % (len(input_msg.joint_names), log_warn_msg_strings[1]),
                        log_warn_msg_strings[2],
                    )
                )
                return False

            # Validate joint_names
            invalid_joint_names = [
                joint_name
                for joint_name in input_msg.joint_names
                if joint_name not in controlled_joints
            ]
            if len(invalid_joint_names) != 0:

                # Create log message
                log_warn_msg_strings = (
                    ["Joint", "was"]
                    if len(invalid_joint_names) == 1
                    else ["Joints", "were"]
                )

                # Send log message
                rospy.logwarn(
                    "%s %s that %s specified in the 'joint_names' field of the "
                    "'panda_training/setJointPositions' message were invalid. Valid "
                    "joint names are %s."
                    % (
                        log_warn_msg_strings[0],
                        invalid_joint_names,
                        log_warn_msg_strings[1],
                        controlled_joints,
                    )
                )
                return False

        # Return success bool
        return True

    def _create_control_publisher_msg(
        self, input_msg, control_type, control_group, controlled_joints
    ):
        """Converts a service input message into a control commands that is used by the
        control publishers.

        Parameters
        ----------
        input_msg :
            The service input message we want to validate.
        control_type : str
            The type of control that is being executed and on which we should wait.
            Options are 'effort_control' and 'position_control'.
        control_group : str
            The robot group which is being controlled. Options are "arm" or "hand".
        controlled_joints : list
            List containing the joints which are being controlled.

        Returns
        -------
        list
            A list containing the control commands for each joint in the order which is
            are required by the publishers
        """

        # Get control_command information out of input message
        if control_type.lower() == "position_control":
            control_input = input_msg.joint_positions.data
            control_msgs_type = (
                self._hand_position_controller_msg_type
                if control_group.lower() == "hand"
                else self._arm_position_controller_msg_type
            )
        else:
            control_input = input_msg.joint_efforts.data
            control_msgs_type = (
                self._hand_effort_controller_msg_type
                if control_group.lower() == "hand"
                else self._arm_effort_controller_msg_type
            )

        # Create control_commands list
        if (
            len(input_msg.joint_names) == 0
        ):  # No joint_names were given index not important
            control_commands = control_input
        else:  # Make sure that each control command is added at the right index
            # Fill joint positions message based on joint_names
            control_commands = []
            for joint in controlled_joints:
                try:
                    index = input_msg.joint_names.index(joint)
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
        joint_positions_req : panda_training.srv.setJointPositionsRequest
            Service request message specifying the positions for the robot arm joints.

        Returns
        -------
        panda_training.srv.setJointPositionsResponse
            Service response.
        """

        # Create required variables and messages
        controllers_missing = False
        resp = setJointPositionsResponse()

        # Retrieve controller information
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )
        missing_controllers = [
            position_controller
            for position_controller in self.arm_position_controllers
            if self._controllers[position_controller].state != "running"
        ]
        controlled_joints = []
        for position_controller in self.arm_position_controllers:
            for claimed_resources in self._controllers[
                position_controller
            ].claimed_resources:
                for resource in claimed_resources.resources:
                    controlled_joints.append(resource)

        # Validate input message
        retval = self._validate_input_msg(
            input_msg=joint_positions_req,
            control_type="position_control",
            control_group="arm",
            controlled_joints=controlled_joints,
        )
        if not retval:
            resp.success = False
            return resp

        # Create control publisher message
        control_pub_msgs = self._create_control_publisher_msg(
            input_msg=joint_positions_req,
            control_type="position_control",
            control_group="arm",
            controlled_joints=controlled_joints,
        )

        # Check if required joints are running
        log_warn_msg_string = (
            "joint position controller is"
            if len(missing_controllers) == 1
            else "Joint position controllers are"
        )
        if len(missing_controllers) >= 1:
            rospy.logwarn(
                "Panda arm joint positions command send but probably not executed as "
                "the %s %s not running." % (missing_controllers, log_warn_msg_string)
            )
            controllers_missing = True

        # Save position control setpoint
        self.joint_positions_setpoint[0:2] = list(
            self.joints.position[0:2]
        )  # Set gripper positions
        if not self.use_group_controller:
            self.joint_positions_setpoint[2:] = [
                item.data for item in control_pub_msgs
            ]  # Arm positions
        else:
            self.joint_positions_setpoint[2:] = [
                item for item in control_pub_msgs.data
            ]  # Arm positions

        # Publish request
        rospy.logdebug("Publishing panda arm joint positions control message.")
        self._arm_position_pub.publish(control_pub_msgs)

        # Wait till control is finished or timeout has been reached
        if joint_positions_req.wait.data and not controllers_missing:
            self._wait_till_done("position_control")

        # Return success message
        resp.success = True
        return resp.success

    def _arm_set_joint_efforts_callback(self, joint_efforts_req):
        """Request arm Joint effort control.

        Parameters
        ----------
        joint_efforts_req : panda_training.srv.setJointEffortsRequest
            Service request message specifying the efforts for the robot arm joints.

        Returns
        -------
        panda_training.srv.setJointPositionsResponse
            Service response.
        """

        # Create required variables and messages
        controllers_missing = False
        resp = setJointEffortsResponse()

        # Retrieve controller information
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )
        missing_controllers = [
            effort_controller
            for effort_controller in self.arm_effort_controllers
            if self._controllers[effort_controller].state != "running"
        ]
        controlled_joints = []
        for effort_controller in self.arm_effort_controllers:
            for claimed_resources in self._controllers[
                effort_controller
            ].claimed_resources:
                for resource in claimed_resources.resources:
                    controlled_joints.append(resource)

        # Validate input message
        retval = self._validate_input_msg(
            input_msg=joint_efforts_req,
            control_type="effort_control",
            control_group="arm",
            controlled_joints=controlled_joints,
        )
        if not retval:
            resp.success = False
            return resp

        # Create control publisher message
        control_pub_msgs = self._create_control_publisher_msg(
            input_msg=joint_efforts_req,
            control_type="effort_control",
            control_group="arm",
            controlled_joints=controlled_joints,
        )

        # Check if required controllers are running
        log_warn_msg_string = (
            "joint effort controller is"
            if len(missing_controllers) == 1
            else "Joint effort controllers are"
        )
        if len(missing_controllers) >= 1:
            rospy.logwarn(
                "Panda arm joint efforts command send but probably not executed as "
                "the %s %s not running." % (missing_controllers, log_warn_msg_string)
            )
            controllers_missing = True

        # Save effort control setpoint
        self.joint_efforts_setpoint[0:2] = list(
            self.joints.effort[0:2]
        )  # Set gripper efforts
        if not self.use_group_controller:
            self.joint_efforts_setpoint[2:] = [
                item.data for item in control_pub_msgs
            ]  # Arm efforts
        else:
            self.joint_efforts_setpoint[2:] = [
                item for item in control_pub_msgs.data
            ]  # Arm efforts

        # Publish request
        rospy.logdebug("Publishing panda arm joint efforts control message.")
        self._arm_effort_pub.publish(control_pub_msgs)

        # Wait till control is finished or timeout has been reached
        if joint_efforts_req.wait.data and not controllers_missing:
            self._wait_till_done(control_type="effort_control")

        # Return service response
        resp.success = True
        return resp.success

    def _hand_set_joint_positions_callback(self, joint_positions_req):
        """Request hand joint position control

        Parameters
        ----------
        joint_positions_req : panda_training.srv.setJointPositionsRequest
            Service request message specifying the positions for the robot hand joints.

        Returns
        -------
        panda_training.srv.setJointPositionsResponse
            Service response.
        """

        # Create required variables and messages
        controllers_missing = False
        resp = setJointPositionsResponse()

        # Retrieve controller information
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )
        missing_controllers = [
            position_controller
            for position_controller in self.hand_position_controllers
            if self._controllers[position_controller].state != "running"
        ]
        controlled_joints = []
        for position_controller in self.hand_position_controllers:
            for claimed_resources in self._controllers[
                position_controller
            ].claimed_resources:
                for resource in claimed_resources.resources:
                    controlled_joints.append(resource)

        # Validate input message
        retval = self._validate_input_msg(
            input_msg=joint_positions_req,
            control_type="position_control",
            control_group="hand",
            controlled_joints=controlled_joints,
        )
        if not retval:
            resp.success = False
            return resp

        # Create control publisher message
        control_pub_msgs = self._create_control_publisher_msg(
            input_msg=joint_positions_req,
            control_type="position_control",
            control_group="hand",
            controlled_joints=controlled_joints,
        )

        # Check if required joints are running
        log_warn_msg_string = (
            "joint position controller is"
            if len(missing_controllers) == 1
            else "Joint position controllers are"
        )
        if len(missing_controllers) >= 1:
            rospy.logwarn(
                "Panda hand joint positions command send but probably not executed as "
                "the %s %s not running." % (missing_controllers, log_warn_msg_string)
            )
            controllers_missing = True

        # Save position control setpoint
        self.joint_positions_setpoint[2:] = list(
            self.joints.position[2:]
        )  # Set gripper positions
        if not self.use_group_controller:
            self.joint_positions_setpoint[:2] = [
                item.data for item in control_pub_msgs
            ]  # Arm positions
        else:
            self.joint_positions_setpoint[:2] = [
                item for item in control_pub_msgs.data
            ]  # Arm positions

        # Publish request
        rospy.logdebug("Publishing panda hand joint positions control message.")
        self._hand_position_pub.publish(control_pub_msgs)

        # Wait till control is finished or timeout has been reached
        if joint_positions_req.wait.data and not controllers_missing:
            self._wait_till_done("position_control")

        # Return success message
        resp.success = True
        return resp.success

    def _hand_set_joint_efforts_callback(self, joint_efforts_req):
        """Request hand joint effort control.

        Parameters
        ----------
        joint_efforts_req : panda_training.srv.setJointEffortsRequest
            Service request message specifying the efforts for the robot hand joints.

        Returns
        -------
        panda_training.srv.setJointPositionsResponse
            Service response.
        """

        # Create required variables and messages
        controllers_missing = False
        resp = setJointEffortsResponse()

        # Retrieve controller information
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )
        missing_controllers = [
            effort_controller
            for effort_controller in self.hand_effort_controllers
            if self._controllers[effort_controller].state != "running"
        ]
        controlled_joints = []
        for effort_controller in self.hand_effort_controllers:
            for claimed_resources in self._controllers[
                effort_controller
            ].claimed_resources:
                for resource in claimed_resources.resources:
                    controlled_joints.append(resource)

        # Validate input message
        retval = self._validate_input_msg(
            input_msg=joint_efforts_req,
            control_type="effort_control",
            control_group="hand",
            controlled_joints=controlled_joints,
        )
        if not retval:
            resp.success = False
            return resp

        # Create control publisher message
        control_pub_msgs = self._create_control_publisher_msg(
            input_msg=joint_efforts_req,
            control_type="effort_control",
            control_group="hand",
            controlled_joints=controlled_joints,
        )

        # Check if required controllers are running
        log_warn_msg_string = (
            "joint effort controller is"
            if len(missing_controllers) == 1
            else "Joint effort controllers are"
        )
        if len(missing_controllers) >= 1:
            rospy.logwarn(
                "Panda hand joint efforts command send but probably not executed as "
                "the %s %s not running." % (missing_controllers, log_warn_msg_string)
            )
            controllers_missing = True

        # Save effort control setpoint
        self.joint_efforts_setpoint[2:] = list(
            self.joints.effort[2:]
        )  # Set gripper efforts
        if not self.use_group_controller:
            self.joint_efforts_setpoint[:2] = [
                item.data for item in control_pub_msgs
            ]  # Arm efforts
        else:
            self.joint_efforts_setpoint[:2] = [
                item for item in control_pub_msgs.data
            ]  # Arm efforts

        # Publish request
        rospy.logdebug("Publishing panda hand joint efforts control message.")
        self._hand_effort_pub.publish(control_pub_msgs)

        # Wait till control is finished or timeout has been reached
        if joint_efforts_req.wait.data and not controllers_missing:
            self._wait_till_done(control_type="effort_control")

        # Return service response
        resp.success = True
        return resp.success

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
