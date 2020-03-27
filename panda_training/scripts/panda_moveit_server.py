#! /usr/bin/env python
"""A ros server that creates a number of Moveit services which can be used
to control the Panda robot or retrieve sensor data for the robot.
"""

# Main python imports
import sys
import os
import re
import copy
from collections import OrderedDict
from functions import flatten_list, lower_first_char
from panda_exceptions import InputMessageInvalid

# ROS python imports
import rospy
import moveit_commander
from moveit_commander.exception import MoveItCommanderException

# ROS msgs and srvs
import moveit_msgs.msg
import geometry_msgs.msg
from panda_training.srv import (
    GetEe,
    GetEeResponse,
    GetEePose,
    GetEePoseResponse,
    GetEeRpy,
    GetEeRpyResponse,
    SetEe,
    SetEeResponse,
    SetEePose,
    SetEePoseResponse,
    SetJointPositions,
    SetJointPositionsResponse,
)


#################################################
# Moveit Planner Server class ###################
#################################################
class PandaMoveitPlannerServer(object):
    """Used to control or request information from the Panda Robot. This is done using
    the Moveit `moveit_commander` module.

    Attributes
    ----------
    robot : moveit_commander.robot.RobotCommander
        The Moveit robot commander object.
    scene : moveit_commander.planning_scene_interface.PlanningSceneInterface
        The Moveit robot scene commander object.
    move_group_arm : moveit_commander.move_group.MoveGroupCommander
        The Moveit arm move group object.
    move_group_hand : moveit_commander.move_group.MoveGroupCommander
        The Moveit hand move group object.
    ee_pose_target : geometry_msgs.msg.Pose
        The last set ee pose.
    arm_joint_pose_target : geometry_msgs.msg.Pose()
        The last set arm joint pose.
    joint_positions_target : dict
        The last Panda joints positions setpoint.
    """

    def __init__(
        self,
        arm_move_group="panda_arm",
        arm_ee_link="panda_link8",
        hand_move_group="hand",
        services_load_type="all",
    ):
        """Initializes the PandaMoveitPlannerServer object.

        Parameters
        ----------
        arm_move_group : str, optional
            The name of the move group you want to use for controlling the Panda arm,
            by default "panda_arm".
        arm_ee_link : str, optional
            The end effector you want moveit to use when controlling
            the Panda arm by default "panda_link8".
        hand_move_group : str, optional
            The name of the move group you want to use for controlling the Panda
            hand, by default "hand".
        services : str, optional
            Specifies whether we want to create 'all' the available services or only the
            ones that are 'crucial' for the panda_training package, by default 'all'.
        """

        # Check which services should be loaded
        if services_load_type.lower() == "crucial":
            create_all_services = False
        else:
            create_all_services = True

        # Initialize Moveit/Robot/Scene and group commanders
        rospy.logdebug("Initialize Moveit Robot/Scene and group commanders.")
        try:

            # NOTE: Fix arguments to solve `C++ converter` error when debugging in ptvsd
            cleaned_args = [
                a
                for a in sys.argv
                if not os.path.basename(__file__) in os.path.basename(__file__)
            ]
            moveit_commander.roscpp_initialize(cleaned_args)
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
            self.move_group_arm = moveit_commander.MoveGroupCommander(arm_move_group)
            self.move_group_hand = moveit_commander.MoveGroupCommander(hand_move_group)
        except Exception as e:  # Shut down if something went wrong

            # Robot Description not found
            if "invalid robot mode" in e.args[0]:
                rospy.logerr(
                    "Shutting down '%s' because robot_description was not found."
                    % rospy.get_name()
                )
                sys.exit(0)

            # Move group not found
            elif len(re.findall("Group '(.*)' was not found", e.args[0])) >= 1:

                # Check if exception was thrown on arm or hand
                if hasattr(self, "move_group_arm"):
                    logerror_move_group = "hand"
                else:
                    logerror_move_group = "arm"

                # Send error message and shutdown node
                rospy.logerr(
                    "Shutting down '%s' because Panda %s move group '%s' was not found."
                    % (
                        rospy.get_name(),
                        logerror_move_group,
                        re.match(r"Group \'(.*)\' was not found.", e.args[0]).group(1),
                    )
                )
                sys.exit(0)
            else:
                rospy.logerr(
                    "Shutting down '%s' because %s" % (rospy.get_name(), e.message)
                )
                sys.exit(0)

        # Set end effector link
        self.move_group_arm.set_end_effector_link(arm_ee_link)

        # Create rviz trajectory publisher
        self._display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=10,
        )

        # Create main PandaMoveitPlannerServer services
        rospy.loginfo("Creating '%s' services." % rospy.get_name())
        rospy.logdebug("Creating '%s/panda_arm/set_ee_pos' service." % rospy.get_name())
        self._arm_set_ee_pose_srv = rospy.Service(
            "%s/panda_arm/set_ee_pos" % rospy.get_name()[1:],
            SetEePose,
            self._arm_set_ee_pose_callback,
        )
        rospy.logdebug("Creating '%s/set_joint_positions' service." % rospy.get_name())
        self._set_joint_positions_srv = rospy.Service(
            "%s/set_joint_positions" % rospy.get_name()[1:],
            SetJointPositions,
            self._set_joint_positions_callback,
        )
        rospy.logdebug("Creating '%s/panda_arm/get_ee' service." % rospy.get_name())
        self._arm_get_ee = rospy.Service(
            "%s/panda_arm/get_ee" % rospy.get_name()[1:],
            GetEe,
            self._arm_get_ee_callback,
        )
        rospy.logdebug("Creating '%s/panda_arm/set_ee' service." % rospy.get_name())
        self._arm_set_ee = rospy.Service(
            "%s/panda_arm/set_ee" % rospy.get_name()[1:],
            SetEe,
            self._arm_set_ee_callback,
        )

        # Create other services
        if create_all_services:
            rospy.logdebug(
                "Creating '%s/panda_arm/set_joint_positions' service."
                % rospy.get_name()
            )
            self._arm_set_joint_positions_srv = rospy.Service(
                "%s/panda_arm/set_joint_positions" % rospy.get_name()[1:],
                SetJointPositions,
                self._arm_set_joint_positions_callback,
            )
            rospy.logdebug(
                "Creating '%s/panda_hand/set_joint_positions' service."
                % rospy.get_name()
            )
            self._hand_set_joint_positions_srv = rospy.Service(
                "%s/panda_hand/set_joint_positions" % rospy.get_name()[1:],
                SetJointPositions,
                self._hand_set_joint_positions_callback,
            )
            rospy.logdebug(
                "Creating '%s/panda_arm/get_ee_pose' service." % rospy.get_name()
            )
            self._arm_get_ee_pose_srv = rospy.Service(
                "%s/panda_arm/get_ee_pose" % rospy.get_name()[1:],
                GetEePose,
                self._arm_get_ee_pose_callback,
            )
            rospy.logdebug(
                "Creating '%s/panda_arm/get_ee_rpy' service." % rospy.get_name()
            )
            self._arm_get_ee_rpy_srv = rospy.Service(
                "%s/panda_arm/get_ee_rpy" % rospy.get_name()[1:],
                GetEeRpy,
                self._arm_get_ee_rpy_callback,
            )
        rospy.loginfo("'%s' services created successfully." % rospy.get_name())

        # Initiate service msgs
        self.ee_pose_target = geometry_msgs.msg.Pose()
        self.arm_joint_pose_target = geometry_msgs.msg.Pose()
        self.joint_positions_target = {}

    ###############################################
    # Helper functions ############################
    ###############################################
    def _link_exists(self, link_name):
        """Function checks whether a given link exists in the robot_description.

        Parameters
        ----------
        link_name : str
            Name of link you want to check.

        Returns
        -------
        Bool
            Boolean specifying whether the link exists.
        """
        return link_name in self.robot.get_link_names()

    def _execute(self, control_group="both"):
        """Plan and execute a trajectory/pose or orientation setpoints

        Parameters
        ----------
        control_group : str, optional
            The robot control group for which you want to execute the control. Options
            are "arm" or "hand" or "both, by default "both".
        Returns
        -------
        list
            List specifying whether the arm and/or hand execution was successfull. If
            control_group == "both" then ["arm_success", "hand_success"].
        """

        # Plan and execute
        if control_group.lower() == "arm":
            self.arm_plan = self.move_group_arm.plan()
            arm_retval = self.move_group_arm.go(wait=True)
            retval = [arm_retval]
        elif control_group.lower() == "hand":
            self.hand_plan = self.move_group_hand.plan()
            hand_retval = self.move_group_hand.go(wait=True)
            retval = [hand_retval]
        elif control_group.lower() == "both":
            self.arm_plan = self.move_group_arm.plan()
            arm_retval = self.move_group_arm.go(wait=True)
            self.hand_plan = self.move_group_hand.plan()
            hand_retval = self.move_group_hand.go(wait=True)
            retval = [arm_retval, hand_retval]
        else:
            logwarn_message = (
                "Control group '%s' does not exist. Please specify a valid control "
                "group. Valid values are %s."
                % (control_group.lower(), "['arm', 'hand', 'both']")
            )
            rospy.logwarn(logwarn_message)
            retval = [False]

        # Return success bool dict
        return retval

    def _create_joint_positions_commands(
        self, input_msg, control_group="both", verbose=False
    ):
        """Converts the service input message in `moveit_commander` compatible joint
        position setpoint commands. While doing this it also verifies whether the given
        input message is valid.

        Parameters
        ----------
        input_msg :
            The service input message we want to validate.
        control_group : str, optional
            The robot control group for which you want to execute the control. Options
            are "arm" or "hand" or "both, by default "both".
        verbose : bool
            Bool specifying whether you want to send a warning message to the ROS
            logger.

        Returns
        -------
        Dict
            Dictionary that contains the 'moveit_commander' arm and hand joint
            position commands. Grouped by control group.

        Raises
        ----------
        panda_training.exceptions.InputMessageInvalid
            Raised when the input_msg could not be converted into 'moveit_commander'
            arm hand joint position commands.
        """

        # Retrieve control information out of input message
        joint_names = input_msg.joint_names
        joint_positions = input_msg.joint_positions.data

        # Get controlled joints
        if control_group.lower() == "arm":
            controlled_joints = [self.move_group_arm.get_active_joints()]
        elif control_group.lower() == "hand":
            controlled_joints = [self.move_group_hand.get_active_joints()]
        elif control_group.lower() == "both":
            controlled_joints = flatten_list(
                [
                    self.move_group_arm.get_active_joints(),
                    self.move_group_hand.get_active_joints(),
                ]
            )
        else:
            logwarn_message = (
                "Control group '%s' does not exist. Please specify a valid control "
                "group. Valid values are %s."
                % (control_group.lower(), "['arm', 'hand', 'both']")
            )
            if verbose:
                rospy.logwarn(logwarn_message)
            raise InputMessageInvalid(
                message="Invalid number of joint position commands.",
                log_message=logwarn_message,
            )

        # Get number of controlled joints
        controlled_joints_size = len(controlled_joints)

        # Check service request input
        if len(joint_names) == 0:

            # Check if enough joint position commands were given
            if len(joint_positions) != controlled_joints_size:

                # Create log message
                logwarn_msg_strings = [
                    "joint position"
                    if len(joint_positions) == 1
                    else "joint positions",
                    "arm and hand"
                    if control_group.lower() == "both"
                    else control_group.lower(),
                    "contain" if control_group.lower() == "both" else "contains",
                    "joint" if controlled_joints_size == 1 else "joints",
                ]
                logwarn_message = (
                    "You specified %s while the Panda Robot %s %s %s active "
                    "joints."
                    % (
                        "%s %s" % (len(joint_positions), logwarn_msg_strings[0]),
                        logwarn_msg_strings[1],
                        logwarn_msg_strings[2],
                        "%s %s" % (controlled_joints_size, logwarn_msg_strings[3]),
                    )
                )
                if verbose:
                    rospy.logwarn(logwarn_message)
                raise InputMessageInvalid(
                    message="Invalid number of joint position commands.",
                    log_message=logwarn_message,
                    details={
                        "joint_positions_command_length": len(joint_positions),
                        "controlled_joints": controlled_joints_size,
                    },
                )
            else:

                # Generate moveit_commander_control command dictionary
                if control_group.lower() == "arm":
                    control_commands = {"arm": joint_positions}
                elif control_group.lower() == "hand":
                    control_commands = {"hand": joint_positions}
                elif control_group.lower() == "both":
                    control_commands = {
                        "arm": joint_positions[
                            : len(self.move_group_arm.get_active_joints())
                        ],
                        "hand": joint_positions[
                            -len(self.move_group_hand.get_active_joints()) :
                        ],
                    }

                # Return control command dictionary
                return control_commands
        else:

            # Check if enough control values were given
            if len(joint_names) != len(joint_positions):

                # Create log message
                logwarn_msg_strings = [
                    "joint position"
                    if len(joint_positions) == 1
                    else "joint positions",
                    "joint" if len(joint_names) == 1 else "joints",
                    "joint position",
                    "panda_training/SetJointPositions",
                ]

                # Send log message
                logwarn_message = (
                    "You specified %s while the 'joint_names' field of the "
                    "'%s' message contains %s. Please make sure you supply "
                    "a %s for each joint contained in the 'joint_names' field."
                    % (
                        "%s %s" % (len(joint_positions), logwarn_msg_strings[0]),
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
                        "joint_positions_command_length": len(joint_positions),
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
                    "joint names for controlling the panda %s are %s."
                    % (
                        "Joint" if len(invalid_joint_names) == 1 else "Joints",
                        invalid_joint_names,
                        "was" if len(invalid_joint_names) == 1 else "were",
                        "was" if len(invalid_joint_names) == 1 else "were",
                        "arm and hand"
                        if control_group.lower() == "both"
                        else control_group.lower(),
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

                # Get the current state of the arm and hand
                arm_state_dict = OrderedDict(
                    zip(
                        self.move_group_arm.get_active_joints(),
                        self.move_group_arm.get_current_joint_values(),
                    )
                )
                hand_state_dict = OrderedDict(
                    zip(
                        self.move_group_hand.get_active_joints(),
                        self.move_group_hand.get_current_joint_values(),
                    )
                )
                input_command_dict = OrderedDict(zip(joint_names, joint_positions))

                # Update current state dictionary with given joint_position setpoints
                arm_output_command_dict = copy.deepcopy(arm_state_dict)
                hand_output_command_dict = copy.deepcopy(hand_state_dict)
                for (joint, position) in input_command_dict.items():  # Update arm
                    if joint in arm_state_dict:
                        arm_output_command_dict[joint] = position
                for (joint, position) in input_command_dict.items():  # Update hand
                    if joint in hand_state_dict:
                        hand_output_command_dict[joint] = position

                # Return moveit_commander comand dictionary
                if control_group.lower() == "arm":
                    control_commands = {"arm": arm_output_command_dict.values()}
                elif control_group.lower() == "hand":
                    control_commands = {"hand": hand_output_command_dict.values()}
                elif control_group.lower() == "both":
                    return {
                        "arm": arm_output_command_dict.values(),
                        "hand": hand_output_command_dict.values(),
                    }

    ###############################################
    # Service callback functions ##################
    ###############################################
    def _arm_set_ee_pose_callback(self, request):
        """Request the Panda arm to control to a given end effector
        (EE) pose.

        Parameters
        ----------
        request : geometry_msgs.msg.Pose
            The trajectory you want the EE to follow.

        Returns
        -------
        panda_train.srv.SetEePoseResponse
            Response message containing (success bool, message).
        """

        # Fill trajectory message
        rospy.logdebug("Setting ee pose.")
        resp = SetEePoseResponse()
        self.ee_pose_target.orientation.w = request.pose.orientation.w
        self.ee_pose_target.position.x = request.pose.position.x
        self.ee_pose_target.position.y = request.pose.position.y
        self.ee_pose_target.position.z = request.pose.position.z

        # Send trajectory message and return response
        try:
            self.move_group_arm.set_pose_target(self.ee_pose_target)
            self._execute()
        except MoveItCommanderException as e:
            rospy.logwarn(e.message)
            resp.success = False
            resp.message = e.message

        # Return success message
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _arm_set_joint_positions_callback(self, request):
        """Request the Panda arm to go to a given joint angle.

        Parameters
        ----------
        request : panda_train.srv.SetJointPositionRequest
            The joint positions you want to control the joints to.

        Returns
        -------
        panda_train.srv.SetJointPositionResponse
            Response message containing (success bool, message).
        """

        # Create response message
        rospy.logdebug("Setting arm joint position targets.")
        resp = SetJointPositionsResponse()

        # Validate request and create moveit_commander command
        try:
            moveit_commander_commands = self._create_joint_positions_commands(
                request, control_group="arm"
            )
        except InputMessageInvalid as e:

            # Print warning message and return result
            logwarn_message = "Arm joint Positions not set as " + lower_first_char(
                e.log_message
            )
            rospy.logwarn(logwarn_message)
            resp.success = False
            resp.message = e.message
            return resp

        # Log setpoint information
        arm_joint_states = self.move_group_arm.get_current_joint_values()
        rospy.logdebug("Current arm joint positions: %s" % arm_joint_states)
        rospy.logdebug(
            "Arm joint positions setpoint: %s" % moveit_commander_commands["arm"]
        )

        # Save Current setpoint to attribute
        self.joint_positions_target = moveit_commander_commands

        # Set joint positions setpoint, execute setpoint and return response
        rospy.logdebug("Setting hand setpoints.")
        try:
            self.move_group_arm.set_joint_value_target(moveit_commander_commands["arm"])
        except MoveItCommanderException:
            rospy.logwarn(
                "Setting joint position targets failed since the given arm joint "
                "positions were not within bounds."
            )
            resp.success = False
            resp.message = "Joint setpoint not within bounds."
            return resp

        # Return success boolean
        rospy.logdebug("Executing joint positions setpoint.")
        self._execute()
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _hand_set_joint_positions_callback(self, request):
        """Request the Panda arm to go to a given joint angle.

        Parameters
        ----------
        request : panda_train.srv.SetJointPositionRequest
            The joint positions you want to control the joints to.

        Returns
        -------
        panda_train.srv.SetJointPositionResponse
            Response message containing (success bool, message).
        """

        # Create response message
        rospy.logdebug("Setting hand joint position targets.")
        resp = SetJointPositionsResponse()

        # Validate request and create moveit_commander command
        try:
            moveit_commander_commands = self._create_joint_positions_commands(request)
        except InputMessageInvalid as e:

            # Print warning message and return result
            logwarn_message = "Hand joint Positions not set as " + lower_first_char(
                e.log_message
            )
            rospy.logwarn(logwarn_message)
            resp.success = False
            resp.message = e.message
            return resp

        # Log setpoint information
        hand_joint_states = self.move_group_hand.get_current_joint_values()
        rospy.logdebug("Current hand joint positions: %s" % hand_joint_states)
        rospy.logdebug(
            "Hand joint positions setpoint: %s" % moveit_commander_commands["hand"]
        )

        # Save Current setpoint to attribute
        self.joint_positions_target = moveit_commander_commands

        # Set joint positions setpoint, execute setpoint and return response
        rospy.logdebug("Setting hand joint position setpoints.")
        try:
            self.move_group_hand.set_joint_value_target(
                moveit_commander_commands["hand"]
            )
        except MoveItCommanderException:
            rospy.logwarn(
                "Setting joint position targets failed since the given hand joint "
                "positions were not within bounds."
            )
            resp.success = False
            resp.message = "Joint setpoint not within bounds."
            return resp

        # Return success boolean
        rospy.logdebug("Executing joint positions setpoint.")
        self._execute()
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _set_joint_positions_callback(self, request):
        """Request the Panda arm and hand to go to a given joint angle.

        Parameters
        ----------
        request : panda_train.srv.SetJointPositionRequest
            The joint positions you want to control the joints to.

        Returns
        -------
        panda_train.srv.SetJointPositionResponse
            Response message containing (success bool, message).
        """

        # Create response message
        rospy.logdebug("Setting joint position targets.")
        resp = SetJointPositionsResponse()

        # Validate request and create moveit_commander command
        try:
            moveit_commander_commands = self._create_joint_positions_commands(request)
        except InputMessageInvalid as e:

            # Print warning message and return result
            logwarn_message = (
                "Panda robot joint Positions not set as "
                + lower_first_char(e.log_message)
            )
            rospy.logwarn(logwarn_message)
            resp.success = False
            resp.message = e.message
            return resp

        # Log setpoint information
        arm_joint_states = self.move_group_arm.get_current_joint_values()
        hand_joint_states = self.move_group_hand.get_current_joint_values()
        rospy.logdebug("Current arm joint positions: %s" % arm_joint_states)
        rospy.logdebug(
            "Arm joint positions setpoint: %s" % moveit_commander_commands["arm"]
        )
        rospy.logdebug("Current hand joint positions: %s" % hand_joint_states)
        rospy.logdebug(
            "Hand joint positions setpoint: %s" % moveit_commander_commands["hand"]
        )

        # Save Current setpoint to attribute
        self.joint_positions_target = moveit_commander_commands

        # Set joint positions setpoint, execute setpoint and return response
        rospy.logdebug("Setting arm and hand setpoints.")
        command_success = []
        try:
            self.move_group_arm.set_joint_value_target(moveit_commander_commands["arm"])
            command_success.append(True)
        except MoveItCommanderException:
            command_success.append(False)
        try:
            self.move_group_hand.set_joint_value_target(
                moveit_commander_commands["hand"]
            )
            command_success.append(True)
        except MoveItCommanderException:
            command_success.append(False)

        # Print error message if an error occurred and return
        if not any(command_success):
            logwarn_string = (
                "arm and hand"
                if len(command_success) > 1
                else ("arm" if command_success[0] else "hand")
            )
            rospy.logwarn(
                "Setting joint position targets failed since the given %s joint "
                "positions were not within bounds." % logwarn_string
            )
            resp.success = False
            resp.message = "Joint setpoint not within bounds."
            return resp

        # Return success boolean
        rospy.logdebug("Executing joint positions setpoint.")
        self._execute()
        resp.success = True
        resp.message = "Everything went OK"
        return resp

    def _arm_get_ee_pose_callback(self, request):
        """Request end effector pose.

        Parameters
        ----------
        request : std_srvs.srv.Empty
            Empty request.

        Returns
        -------
        geometry_msgs.msg.PoseStamped
            The current end effector pose.
        """

        # Retrieve and return end effector pose
        rospy.logdebug("Retrieving ee pose.")
        gripper_pose = self.move_group_arm.get_current_pose()
        gripper_pose_res = GetEePoseResponse()
        gripper_pose_res = gripper_pose.pose
        return gripper_pose_res

    def _arm_get_ee_rpy_callback(self, request):
        """Request current end effector (EE) orientation.

        Parameters
        ----------
        request : std_srvs.srv.Empty
            Empty request.

        Returns
        -------
        panda_train.srv.GetEeResponse
            Response message containing containing the roll (z), yaw (y), pitch (x)
            euler angles.
        """

        # Retrieve and return end effector orientation
        rospy.logdebug("Retrieving ee orientation.")
        gripper_rpy = self.move_group_arm.get_current_rpy()
        gripper_rpy_res = GetEeRpyResponse()
        gripper_rpy_res.r = gripper_rpy[0]
        gripper_rpy_res.y = gripper_rpy[1]
        gripper_rpy_res.p = gripper_rpy[2]
        return gripper_rpy_res

    def _arm_get_ee_callback(self, request):
        """Request end effector (EE) name.

        Parameters
        ----------
        request : std_srvs.srv.Empty
            Empty request.

        Returns
        -------
        panda_train.srv.GetEeResponse
            Response message containing the name of the current EE.
        """

        # Return EE name
        rospy.logdebug("Retrieving ee name.")
        resp = GetEeResponse()
        resp.ee_name = self.move_group_arm.get_end_effector_link()
        return resp

    def _arm_set_ee_callback(self, request):
        """Request end effector (EE) change.

        Parameters
        ----------
        request : panda_train.srv.SetEe
            Request message containing the name of the end effector you want to be set.

        Returns
        -------
        panda_train.srv.SetEeResponse
            Response message containing (success bool, message).
        """

        # Set end effector and return response
        rospy.logdebug("Setting ee.")
        resp = SetEeResponse()
        if self._link_exists(request.ee_name):  # Check if vallid
            try:
                self.move_group_arm.set_end_effector_link(request.ee_name)
            except MoveItCommanderException as e:
                rospy.logwarn("Ee could not be set.")
                resp = False
                resp.message = e.message
            resp.success = True
            resp.message = "Everything went OK"
        else:
            rospy.logwarn(
                "Ee could not be as %s is not a valid ee link." % request.ee_name
            )
            resp.success = False
            resp.message = "%s is not a valid ee link." % request.ee_name
        return resp


#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Initiate Moveit Planner Server
    rospy.init_node("panda_moveit_planner_server")

    # Get private parameters specified in the launch file
    try:  # Check end effector
        arm_ee_link = rospy.get_param("~end_effector")
    except KeyError:
        arm_ee_link = "panda_grip_site"
    try:  # Check required services
        services_load_type = rospy.get_param("~services_load_type").lower()

        # Check if valid input
        if services_load_type.lower() not in ["all", "crucial"]:
            rospy.logerr(
                "Shutting down '%s' because ros argument 'services:=%s' is "
                "not valid. Options are 'all' and 'crucial'." % rospy.get_name()
            )
            sys.exit(0)
    except KeyError:
        services_load_type = "all"

    # Start moveit planner server
    moveit_planner_server = PandaMoveitPlannerServer(
        arm_ee_link=arm_ee_link, services_load_type=services_load_type
    )
    rospy.spin()  # Maintain the service open.
