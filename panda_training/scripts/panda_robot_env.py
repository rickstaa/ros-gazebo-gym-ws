"""This robot environment contains all the functions which are responsible
for interaction with the robot (Control and Sensors).
"""

# Main python imports
import sys
import panda_robot_gazebo_goal_env
from functions import action_server_exists
from panda_control_switcher import PandaControlSwitcher

# ROS python imports
import rospy
import actionlib
import tf2_ros

# from tf.transformations import euler_from_quaternion
from rospy.exceptions import ROSException, ROSInterruptException

# ROS msgs and srvs
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
from geometry_msgs.msg import PoseStamped
from panda_training.srv import (
    GetEe,
    GetEeRequest,
    GetEePose,
    GetEePoseRequest,
    GetEeRpy,
    GetEeRpyRequest,
    SetEe,
    SetEeRequest,
    SetEePose,
    SetJointEfforts,
    SetEePoseRequest,
    SetJointPositions,
)

# General script parameters
# TODO: Remove unused
ROBOT_CONTROL_TYPES = [
    "joint_trajectory_control",
    "joint_position_control",
    "joint_effort_control",
    "joint_group_position_control",
    "joint_group_effort_control",
    "ee_control",
]
MOVEIT_SET_EE_POSE_TOPIC = "panda_moveit_planner_server/panda_arm/set_ee_pos"
MOVEIT_GET_EE_POSE_TOPIC = "panda_moveit_planner_server/panda_arm/get_ee_pose"
MOVEIT_GET_EE_RPY_TOPIC = "panda_moveit_planner_server/panda_arm/get_ee_rpy"
MOVEIT_SET_EE_TOPIC = "panda_moveit_planner_server/panda_arm/set_ee"
MOVEIT_GET_EE_TOPIC = "panda_moveit_planner_server/panda_arm/get_ee"
MOVEIT_ARM_SET_JOINT_POSITIONS_TOPIC = (
    "panda_moveit_planner_server/panda_arm/set_joint_positions"
)
MOVEIT_HAND_SET_JOINT_POSITIONS_TOPIC = (
    "panda_moveit_planner_server/panda_hand/set_joint_positions"
)
ARM_JOINT_TRAJECTORY_CONTROL_TOPIC = "panda_arm_controller/follow_joint_trajectory"
ARM_JOINT_POSITIONS_CONTROL_TOPIC = "panda_control_server/panda_arm/set_joint_positions"
ARM_JOINT_EFFORTS_CONTROL_TOPIC = "panda_control_server/panda_arm/set_joint_efforts"
HAND_JOINT_TRAJECTORY_CONTROL_TOPIC = "panda_hand_controller/follow_joint_trajectory"
HAND_JOINT_POSITIONS_CONTROL_TOPIC = (
    "panda_control_server/panda_hand/set_joint_positions"
)
HAND_JOINT_EFFORTS_CONTROL_TOPIC = "panda_control_server/panda_hand/set_joint_efforts"

REQUIRED_SERVICES_DICT = {
    "arm": {
        "joint_trajectory_control": ["panda_arm_controller/follow_joint_trajectory"],
        "joint_position_control": [
            "panda_control_server/panda_arm/set_joint_positions"
        ],
        "joint_effort_control": ["panda_control_server/panda_arm/set_joint_efforts"],
        "joint_group_position_control": [
            "panda_control_server/panda_arm/set_joint_positions"
        ],
        "joint_group_effort_control": [
            "panda_control_server/panda_arm/set_joint_efforts"
        ],
        "ee_control": [
            "panda_moveit_planner_server/panda_arm/set_ee_pos",
            "panda_moveit_planner_server/panda_arm/set_joint_pose",
            "panda_moveit_planner_server/panda_arm/get_ee_pose",
            "panda_moveit_planner_server/panda_arm/get_ee_rpy",
            "panda_moveit_planner_server/panda_arm/set_ee",
            "panda_moveit_planner_server/panda_arm/get_ee",
        ],
    },
    "hand": {
        "joint_trajectory_control": ["panda_hand_controller/follow_joint_trajectory"],
        "joint_position_control": [
            "panda_control_server/panda_hand/set_joint_positions"
        ],
        "joint_effort_control": ["panda_control_server/panda_hand/set_joint_efforts"],
        "joint_group_position_control": [
            "panda_control_server/panda_hand/set_joint_positions"
        ],
        "joint_group_effort_control": [
            "panda_control_server/panda_hand/set_joint_efforts"
        ],
        "ee_control": [
            "panda_moveit_planner_server/panda_arm/set_ee_pos",
            "panda_moveit_planner_server/panda_arm/set_joint_pose",
            "panda_moveit_planner_server/panda_arm/get_ee_pose",
            "panda_moveit_planner_server/panda_arm/get_ee_rpy",
            "panda_moveit_planner_server/panda_arm/set_ee",
            "panda_moveit_planner_server/panda_arm/get_ee",
        ],
    },
}

# TODO: Add panda_hand_control_type
# TODO: Fix validation
# TODO: Remove unused services


#################################################
# Panda Robot Environment Class #################
#################################################
class PandaRobotEnv(panda_robot_gazebo_goal_env.RobotGazeboGoalEnv):
    """Used for controlling the panda robot and retrieving sensor data.

    Attributes
    ----------
    reset_control_list : list
        List of controllers to reset.
    robot_name_space : str
        The robot name space.
    robot_EE_link : str
        The robot end effector link.
    robot_arm_control_type : str
        The robot arm control type.
    robot_hand_control_type : str
        The robot hand control type.
    joints : sensor_msgs.msg.JointState
        The current joint states.

    Methods
    ----------
    get_ee_pose():
        Get end effector pose.
    get_ee_rpy():
        Get end effector orientation.
    get_joints():
        Get robot joint states.
    set_ee_pose(action):
        Set end effector pose.
    set_joint_positions(joint_positions):
        Set joint positions.
    """

    def __init__(
        self,
        robot_EE_link="panda_grip_site",
        robot_arm_control_type="joint_position_control",
        robot_hand_control_type="joint_position_control",
        robot_name_space="",
        reset_control_list=[],
    ):
        """Initializes a new Panda Robot environment.

        Parameters
        ----------
        robot_EE_link : str, optional
            Robot end effector link name, by default "panda_grip_site".
        robot_arm_control_type : str, optional
            The type of control you want to use for the robot arm. Options are
            'joint_trajectory_control', 'joint_position_control', 'joint_effort_control'
            'joint_group_position_control', 'joint_group_effort_control' or 'ee_control'
            , by default 'joint_position_control'.
        robot_hand_control_type : str, optional
            The type of control you want to use for the robot hand. Options are
            'joint_trajectory_control', 'joint_position_control', 'joint_effort_control'
            'joint_group_position_control', 'joint_group_effort_control' or 'ee_control'
            , by default 'joint_position_control'.
        robot_name_space : str, optional
            Robot namespace, by default "".
        reset_control_list : list, optional
            List containing the robot controllers you want to reset each time
            the simulation is reset, by default [].
        """

        # Environment initiation message
        rospy.loginfo("Initializing Panda Robot environment.")

        # Validate constructor input arguments
        control_type_invalid = {
            ("arm" if index == 0 else "hand"): control_type
            for index, control_type in enumerate(
                [robot_arm_control_type.lower(), robot_hand_control_type.lower()]
            )
            if control_type not in ROBOT_CONTROL_TYPES
        }
        if control_type_invalid:
            rospy.logerr(
                "Shutting down '%s' because the Panda robot %s control %s that %s "
                "specified %s %s invalid. Please use one of the following robot "
                " control types: "
                "%s."
                % (
                    rospy.get_name(),
                    "hand and arm"
                    if len(control_type_invalid.keys()) > 1
                    else list(control_type_invalid.keys())[0],
                    "types" if len(control_type_invalid.keys()) > 1 else "type",
                    "were" if len(control_type_invalid.keys()) > 1 else "was",
                    list(control_type_invalid.values())
                    if len(control_type_invalid.keys()) > 1
                    else "'" + list(control_type_invalid.values())[0] + "'",
                    "are" if len(control_type_invalid.keys()) > 1 else "is",
                    ROBOT_CONTROL_TYPES,
                )
            )
            sys.exit(0)

        # Set class attributes
        self.reset_control_list = reset_control_list
        self.robot_name_space = robot_name_space
        self.robot_EE_link = robot_EE_link
        self.robot_arm_control_type = robot_arm_control_type
        self.robot_hand_control_type = robot_hand_control_type
        self._panda_moveit_planner_connection_timeout = 5
        self._controller_switcher_connection_timeout = 5
        self._control_connection_timeout = rospy.Duration(secs=10)
        self._joint_traj_action_server_step_size = 1  # Time from send [sec]
        self._services_connection_status = {}

        # Create Needed subscribers
        rospy.loginfo("Setting up sensor data subscribers.")
        joint_states_topic = "joint_states"
        self.joints = JointState()
        self._joint_states_sub = rospy.Subscriber(
            joint_states_topic, JointState, self._joints_callback
        )

        # Create controller switcher
        self._controller_switcher = PandaControlSwitcher(
            connection_timeout=self._controller_switcher_connection_timeout
        )

        # Create transform listener
        self._tfBuffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tfBuffer)

        #############################################
        # Connect to control services ###############
        #############################################
        # NOTE: Depending on the type of control we require the Panda moveit services,
        # the joint trajectory action service, the joint effort control service or the
        # joint position control service. All these services are initiated here if they
        # are available.
        rospy.loginfo("Connecting to robot control services.")

        #################################
        # Moveit Control services #######
        #################################

        # Connect to moveit set ee pose topic
        try:
            rospy.logdebug("Connecting to '%s' service." % MOVEIT_SET_EE_POSE_TOPIC)
            rospy.wait_for_service(
                MOVEIT_SET_EE_POSE_TOPIC,
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._moveit_set_ee_pose_client = rospy.ServiceProxy(
                MOVEIT_SET_EE_POSE_TOPIC, SetEePose
            )
            rospy.logdebug("Connected to '%s' service!" % MOVEIT_SET_EE_POSE_TOPIC)
            self._services_connection_status[MOVEIT_SET_EE_POSE_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % MOVEIT_SET_EE_POSE_TOPIC
            )
            self._services_connection_status[MOVEIT_SET_EE_POSE_TOPIC] = False

        # Connect to Moveit get_ee_pose service
        try:
            rospy.logdebug("Connecting to '%s' service." % MOVEIT_GET_EE_POSE_TOPIC)
            rospy.wait_for_service(
                MOVEIT_GET_EE_POSE_TOPIC,
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._moveit_get_ee_pose_client = rospy.ServiceProxy(
                MOVEIT_GET_EE_POSE_TOPIC, GetEePose
            )
            rospy.logdebug("Connected to '%s' service!" % MOVEIT_GET_EE_POSE_TOPIC)
            self._services_connection_status[MOVEIT_GET_EE_POSE_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % MOVEIT_GET_EE_POSE_TOPIC
            )
            self._services_connection_status[MOVEIT_GET_EE_POSE_TOPIC] = False

        # Connect to Moveit get_ee_rpy service
        try:
            rospy.logdebug("Connecting to '%s' service." % MOVEIT_GET_EE_RPY_TOPIC)
            rospy.wait_for_service(
                MOVEIT_GET_EE_RPY_TOPIC,
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._moveit_get_ee_rpy_client = rospy.ServiceProxy(
                MOVEIT_GET_EE_RPY_TOPIC, GetEeRpy
            )
            rospy.logdebug("Connected to '%s' service!" % MOVEIT_GET_EE_RPY_TOPIC)
            self._services_connection_status[MOVEIT_GET_EE_RPY_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % MOVEIT_GET_EE_RPY_TOPIC
            )
            self._services_connection_status[MOVEIT_GET_EE_RPY_TOPIC] = False

        # Connect to Moveit set_ee service
        try:
            rospy.logdebug("Connecting to '%s' service." % MOVEIT_SET_EE_TOPIC)
            rospy.wait_for_service(
                MOVEIT_SET_EE_TOPIC,
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._moveit_set_ee_client = rospy.ServiceProxy(MOVEIT_SET_EE_TOPIC, SetEe)
            rospy.logdebug("Connected to '%s' service!" % MOVEIT_SET_EE_TOPIC)
            self._services_connection_status[MOVEIT_SET_EE_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn("Failed to connect to '%s' service!" % MOVEIT_SET_EE_TOPIC)
            self._services_connection_status[MOVEIT_SET_EE_TOPIC] = False

        # Connect to Moveit get_ee service
        try:
            rospy.logdebug("Connecting to '%s' service." % MOVEIT_GET_EE_TOPIC)
            rospy.wait_for_service(
                MOVEIT_GET_EE_TOPIC,
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._moveit_get_ee_client = rospy.ServiceProxy(MOVEIT_GET_EE_TOPIC, GetEe)
            rospy.logdebug("Connected to '%s' service!" % MOVEIT_GET_EE_TOPIC)

            # Validate class set EE with moveit end effector
            self._moveit_ee = self._moveit_get_ee_client(GetEeRequest()).ee_name
            if self._moveit_ee != self.robot_EE_link:
                rospy.logwarn(
                    "Moveit control end effector was set to '%s' while "
                    "panda_robot_env class end effector was set to '%s'."
                    "Changing moveit end effector to '%s'."
                    % (self._moveit_ee, self.robot_EE_link, self._moveit_ee)
                )

                # Set moveit EE
                req = SetEeRequest()
                req.ee_name = self.robot_EE_link
                resp = self._moveit_set_ee_client(req)

                # Check ee set result
                if not resp.success:
                    rospy.logerr(
                        "Shutting down '%s' because '%s' could not be set as "
                        "the Moveit move_group end effector (EE). Please "
                        "check that the EE you initiate the panda_robot_env "
                        "class with is vallid." % (rospy.get_name(), self.robot_EE_link)
                    )
                    sys.exit(0)
            else:
                self._services_connection_status[MOVEIT_GET_EE_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn("Failed to connect to '%s' service!" % MOVEIT_GET_EE_TOPIC)
            self._services_connection_status[MOVEIT_GET_EE_TOPIC] = False

        # Connect to Moveit arm set_joints_position service
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % MOVEIT_ARM_SET_JOINT_POSITIONS_TOPIC
            )
            rospy.wait_for_service(
                MOVEIT_ARM_SET_JOINT_POSITIONS_TOPIC,
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._moveit_arm_set_joint_positions_client = rospy.ServiceProxy(
                MOVEIT_ARM_SET_JOINT_POSITIONS_TOPIC, SetEe
            )
            rospy.logdebug(
                "Connected to '%s' service!" % MOVEIT_ARM_SET_JOINT_POSITIONS_TOPIC
            )
            self._services_connection_status[
                MOVEIT_ARM_SET_JOINT_POSITIONS_TOPIC
            ] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!"
                % MOVEIT_ARM_SET_JOINT_POSITIONS_TOPIC
            )
            self._services_connection_status[
                MOVEIT_ARM_SET_JOINT_POSITIONS_TOPIC
            ] = False

        # Connect to Moveit hand set_joints_position service
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % MOVEIT_HAND_SET_JOINT_POSITIONS_TOPIC
            )
            rospy.wait_for_service(
                MOVEIT_HAND_SET_JOINT_POSITIONS_TOPIC,
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._moveit_hand_set_joint_positions_client = rospy.ServiceProxy(
                MOVEIT_HAND_SET_JOINT_POSITIONS_TOPIC, SetEe
            )
            rospy.logdebug(
                "Connected to '%s' service!" % MOVEIT_HAND_SET_JOINT_POSITIONS_TOPIC
            )
            self._services_connection_status[
                MOVEIT_HAND_SET_JOINT_POSITIONS_TOPIC
            ] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!"
                % MOVEIT_HAND_SET_JOINT_POSITIONS_TOPIC
            )
            self._services_connection_status[
                MOVEIT_HAND_SET_JOINT_POSITIONS_TOPIC
            ] = False

        #################################
        # Trajectory action service #####
        #################################

        # Connect to Joint Trajectory Panda ARM Control (action) service
        rospy.logdebug(
            "Connecting to '%s' action service." % ARM_JOINT_TRAJECTORY_CONTROL_TOPIC
        )
        if action_server_exists(ARM_JOINT_TRAJECTORY_CONTROL_TOPIC):  # Check if exists

            # Connect to robot control action server
            self._arm_joint_traj_control_client = actionlib.SimpleActionClient(
                ARM_JOINT_TRAJECTORY_CONTROL_TOPIC, FollowJointTrajectoryAction
            )
            # Waits until the action server has started up
            retval = self._arm_joint_traj_control_client.wait_for_server(
                timeout=self._control_connection_timeout
            )
            if not retval:
                rospy.logwarn(
                    "No connection could be established with the '%s' service. "
                    "The Panda Robot Environment therefore can not use this action "
                    "service to control the Panda Robot."
                    % (ARM_JOINT_TRAJECTORY_CONTROL_TOPIC)
                )
                self._services_connection_status[
                    ARM_JOINT_TRAJECTORY_CONTROL_TOPIC
                ] = False
            else:
                self._services_connection_status[
                    ARM_JOINT_TRAJECTORY_CONTROL_TOPIC
                ] = True
        else:
            rospy.logwarn(
                "No connection could be established with the '%s' service. "
                "The Panda Robot Environment therefore can not use this action "
                "service to control the Panda Robot."
                % (ARM_JOINT_TRAJECTORY_CONTROL_TOPIC)
            )
            self._services_connection_status[ARM_JOINT_TRAJECTORY_CONTROL_TOPIC] = False

        # Connect to Joint Trajectory Panda HAND Control (action) service
        rospy.logdebug(
            "Connecting to '%s' action service." % HAND_JOINT_TRAJECTORY_CONTROL_TOPIC
        )
        if action_server_exists(HAND_JOINT_TRAJECTORY_CONTROL_TOPIC):  # Check if exists

            # Connect to robot control action server
            self._hand_joint_traj_control_client = actionlib.SimpleActionClient(
                HAND_JOINT_TRAJECTORY_CONTROL_TOPIC, FollowJointTrajectoryAction
            )
            # Waits until the action server has started up
            retval = self._hand_joint_traj_control_client.wait_for_server(
                timeout=self._control_connection_timeout
            )
            if not retval:
                rospy.logwarn(
                    "No connection could be established with the '%s' service. "
                    "The Panda Robot Environment therefore can not use this action "
                    "service to control the Panda Robot."
                    % (HAND_JOINT_TRAJECTORY_CONTROL_TOPIC)
                )
                self._services_connection_status[
                    HAND_JOINT_TRAJECTORY_CONTROL_TOPIC
                ] = False
            else:
                self._services_connection_status[
                    HAND_JOINT_TRAJECTORY_CONTROL_TOPIC
                ] = True
        else:
            rospy.logwarn(
                "No connection could be established with the '%s' service. "
                "The Panda Robot Environment therefore can not use this action "
                "service to control the Panda Robot."
                % (HAND_JOINT_TRAJECTORY_CONTROL_TOPIC)
            )
            self._services_connection_status[
                HAND_JOINT_TRAJECTORY_CONTROL_TOPIC
            ] = False

        #################################
        # Panda control services ########
        #################################

        # Connect to panda_control_server/panda_arm/set_joint_positions service
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % ARM_JOINT_POSITIONS_CONTROL_TOPIC
            )
            rospy.wait_for_service(
                ARM_JOINT_POSITIONS_CONTROL_TOPIC,
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._arm_set_joint_positions_client = rospy.ServiceProxy(
                ARM_JOINT_POSITIONS_CONTROL_TOPIC, SetJointPositions
            )
            rospy.logdebug(
                "Connected to '%s' service!" % ARM_JOINT_POSITIONS_CONTROL_TOPIC
            )
            self._services_connection_status[ARM_JOINT_POSITIONS_CONTROL_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "No connection could be established with the '%s' service. "
                "The Panda Robot Environment therefore can not use this service "
                "to control the Panda Robot." % ARM_JOINT_POSITIONS_CONTROL_TOPIC
            )
            self._services_connection_status[ARM_JOINT_POSITIONS_CONTROL_TOPIC] = False

        # Connect to panda_control_server/panda_arm/set_joint_efforts service
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % ARM_JOINT_EFFORTS_CONTROL_TOPIC
            )
            rospy.wait_for_service(
                ARM_JOINT_EFFORTS_CONTROL_TOPIC,
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._arm_set_joint_efforts_client = rospy.ServiceProxy(
                ARM_JOINT_EFFORTS_CONTROL_TOPIC, SetJointEfforts
            )
            rospy.logdebug(
                "Connected to '%s service!" % ARM_JOINT_EFFORTS_CONTROL_TOPIC
            )
            self._services_connection_status[ARM_JOINT_EFFORTS_CONTROL_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "No connection could be established with the '%s' service. "
                "The Panda Robot Environment therefore can not use this service "
                "to control the Panda Robot." % ARM_JOINT_EFFORTS_CONTROL_TOPIC
            )
            self._services_connection_status[ARM_JOINT_EFFORTS_CONTROL_TOPIC] = False

        # Connect to panda_control_server/panda_hand/set_joint_positions service
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % HAND_JOINT_POSITIONS_CONTROL_TOPIC
            )
            rospy.wait_for_service(
                HAND_JOINT_POSITIONS_CONTROL_TOPIC,
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._hand_set_joint_positions_client = rospy.ServiceProxy(
                HAND_JOINT_POSITIONS_CONTROL_TOPIC, SetJointPositions
            )
            rospy.logdebug(
                "Connected to '%s' service!" % HAND_JOINT_POSITIONS_CONTROL_TOPIC
            )
            self._services_connection_status[HAND_JOINT_POSITIONS_CONTROL_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "No connection could be established with the '%s' service. "
                "The Panda Robot Environment therefore can not use this service "
                "to control the Panda Robot." % HAND_JOINT_POSITIONS_CONTROL_TOPIC
            )
            self._services_connection_status[HAND_JOINT_POSITIONS_CONTROL_TOPIC] = False

        # Connect to panda_control_server/panda_hand/set_joint_efforts service
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % HAND_JOINT_EFFORTS_CONTROL_TOPIC
            )
            rospy.wait_for_service(
                HAND_JOINT_EFFORTS_CONTROL_TOPIC,
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._arm_set_joint_efforts_client = rospy.ServiceProxy(
                HAND_JOINT_EFFORTS_CONTROL_TOPIC, SetJointEfforts
            )
            rospy.logdebug(
                "Connected to '%s service!" % HAND_JOINT_EFFORTS_CONTROL_TOPIC
            )
            self._services_connection_status[HAND_JOINT_EFFORTS_CONTROL_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "No connection could be established with the '%s' service. "
                "The Panda Robot Environment therefore can not use this service "
                "to control the Panda Robot." % HAND_JOINT_EFFORTS_CONTROL_TOPIC
            )
            self._services_connection_status[HAND_JOINT_EFFORTS_CONTROL_TOPIC] = False

        #########################################
        # Validate service control connections ##
        #########################################
        # NOTE: Shut down ROS node if the services that are needed for a given control
        # type are not available.
        (retval, missing_services) = self._required_services_available()

        # Shut down ROS node if required services are not available
        if not retval:

            # Create error message and send log
            control_types = [
                self.robot_arm_control_type
                if key == "arm"
                else self.robot_hand_control_type
                for (key, val) in missing_services.items()
            ]
            logerror_msg_strings = [
                (
                    "{} and {}".format(*missing_services.keys())
                    if len(missing_services.keys()) > 1
                    else "{}".format(*missing_services.keys())
                ),
                (
                    "'{}' and '{}'".format(*control_types)
                    if len(control_types) > 1
                    else "'{}'".format(*control_types)
                ),
            ]
            rospy.logerr(
                "Shutting down '{}' node because the Panda robot services '{}' which "
                "are needed for controlling the Panda {} using {} control are not "
                "available.".format(
                    rospy.get_name(),
                    list(missing_services.values()),
                    logerror_msg_strings[0],
                    logerror_msg_strings[1],
                )
            )
            sys.exit(0)

        #########################################
        # Switch to right controller ############
        #########################################

        # Switch to the right controller
        rospy.loginfo("Switching to required controller.")
        arm_switch_resp = self._controller_switcher.switch(
            control_group="arm", control_type=self.robot_arm_control_type
        )
        hand_switch_resp = self._controller_switcher.switch(
            control_group="hand", control_type=self.robot_hand_control_type
        )

        # Check whether the controller was available
        control_switch_success = [arm_switch_resp.success, hand_switch_resp.success]
        if not all(control_switch_success):
            logerror_msg_strings = [
                "hand and arm control types"
                if not all(control_switch_success)
                else (
                    "arm control type"
                    if control_switch_success[0]
                    else "hand control type"
                ),
                [arm_switch_resp.prev_control_type, hand_switch_resp.prev_control_type],
                [self.robot_arm_control_type, self.robot_hand_control_type],
            ]
            rospy.logerr(
                "Shutting down '%s' node because the Panda %s could could not be "
                "switched from %s to %s."
                % (
                    rospy.get_name(),
                    logerror_msg_strings[0],
                    logerror_msg_strings[1],
                    logerror_msg_strings[2],
                )
            )
            sys.exit(0)

        #########################################
        # Initialize parent class ###############
        #########################################

        # Initialize parent Class to setup the Gazebo environment)
        super(PandaRobotEnv, self).__init__(
            robot_name_space=self.robot_name_space,
            reset_controls=False,
            reset_control_list=self.reset_control_list,
        )

        # Environment initiation complete message
        rospy.loginfo("Panda Robot environment initialized.")

    #############################################
    # Overload Gazebo env virtual methods #######
    #############################################
    def _check_all_systems_ready(self):
        """Checks that all the sensors, publishers and other simulation systems are
        operational.

        Returns
        -------
        Boolean
            Boolean specifying whether reset was successful.
        """
        self._check_all_sensors_ready()
        return True

    #############################################
    # Panda Robot env main methods ##############
    #############################################
    def get_joints(self):
        """Returns the robot joints.

        Returns
        -------
        np.array
            List containing the robot joint names.
        """
        return self.joints

    def get_ee_pose(self):
        """Returns the end effector EE pose.

        Returns
        -------
        geometry_msgs.msg.PoseStamped
            The current end effector pose.
        """

        # TODO: FIX
        # TODO: Use tf
        # Retrieve end effector pose
        grip_site_trans = self._tfBuffer.lookup_transform(
            "world", "panda_grip_site", rospy.Time()
        )

        # Transform trans to pose
        pose = PoseStamped()
        pose.header = grip_site_trans.header
        pose.pose.orientation = grip_site_trans.transform.rotation
        pose.pose.position = grip_site_trans.transform.translation

        # Retrieve end effector pose 2
        gripper_pose_req = GetEePoseRequest()
        gripper_pose = self.ee_pose_client(gripper_pose_req)

        return gripper_pose

    def get_ee_rpy(self):
        """Returns the end effector EE orientation.

        Returns
        -------
        list
            List containing the roll (z), yaw (y), pitch (x) euler angles.
        """
        # TODO: FIX
        # TODO: use tf
        gripper_rpy_req = GetEeRpyRequest()
        gripper_rpy = self._moveit_get_ee_pose_client(gripper_rpy_req)

        return gripper_rpy

    def set_ee_pose(self, action):
        """Wraps an action vector of joint angles into a JointTrajectory message.
        The velocities, accelerations, and effort do not control the arm motion
        """
        # TODO: FIX

        # Set up a trajectory message to publish.
        ee_target = SetEePoseRequest()
        ee_target.pose.orientation.w = 1.0
        ee_target.pose.position.x = action[0]
        ee_target.pose.position.y = action[1]
        ee_target.pose.position.z = action[2]
        self._moveit_set_ee_pose_client(ee_target)
        return True

    def set_joint_positions(self, joint_positions):
        """Wraps an action vector of joint angles into a JointTrajectory message.
        The velocities, accelerations, and effort do not control the arm motion.

        Parameters
        ----------
        joint_positions : dict
            The joint positions of each of the robot joints.

        Returns
        -------
        bool
            Boolean specifying if the joint_positions were set successfully.
        """

        # Set joint positions
        # NOTE: This is done by using the 'panda_control' joint_positions service, the
        # 'joint_trajectory' action service or the 'panda_moveit_server' service.
        self._controller_switcher.switch(
            control_group="arm", control_type="joint_position_control"
        )
        self._controller_switcher.switch(
            control_group="hand", control_type="joint_position_control"
        )
        self._arm_set_joint_positions_client.call()
        self._hand_set_joint_positions_client.call()
        self._controller_switcher.switch(control_group="arm", control_type="ee_control")
        self._controller_switcher.switch(
            control_group="hand", control_type="ee_control"
        )

        # # Add Joint_Trajectory control
        # # TODO: Add gripper control
        # if self.robot_arm_control_type == "ee_control":  # Use moveit service
        #     joint_point = SetJointPoseRequest()
        #     joint_point.point.positions = [None] * 7
        #     joint_point.point.positions[0] = joint_positions["panda_joint1"]
        #     joint_point.point.positions[1] = joint_positions["panda_joint2"]
        #     joint_point.point.positions[2] = joint_positions["panda_joint3"]
        #     joint_point.point.positions[3] = joint_positions["panda_joint4"]
        #     joint_point.point.positions[4] = joint_positions["panda_joint5"]
        #     joint_point.point.positions[5] = joint_positions["panda_joint6"]
        #     joint_point.point.positions[6] = joint_positions["panda_joint7"]
        #     self._set_joint_pose_client(joint_point)
        #     return True

        # if (
        #     self.robot_arm_control_type == "joint_trajectory_control"
        # ):  # Use joint traj action service

        #     # Convert joint_positions to joint_traj_action goal message
        #     joint_positions = dict.fromkeys(joint_positions, 0)
        #     goal_msg = self._joint_positions_2_follow_joint_trajectory_goal(
        #         joint_positions
        #     )

        #     # Send joint_traj action goal message
        #     self._arm_joint_traj_control_client.send_goal(
        #         goal_msg, feedback_cb=self._joint_traj_control_feedback_callback
        #     )
        #     return True
        # if (
        #     self.robot_arm_control_type == "joint_position_control"
        # ):  # Use joint traj action service

        #     # Send goal to the joint position service
        #     req = SetJointPositionsRequest()
        #     req.joint_positions.data = [
        #         joint_positions["panda_joint1"],
        #         joint_positions["panda_joint2"],
        #         joint_positions["panda_joint3"],
        #         joint_positions["panda_joint4"],
        #         joint_positions["panda_joint5"],
        #         joint_positions["panda_joint6"],
        #         joint_positions["panda_joint7"],
        #     ]
        #     self._arm_set_joint_positions_client.call(req)
        #     return True
        # else:  # Joint_effort control
        #     # NOTE: When joint_effort control is chosen we first try to control using
        #     # the joint_traj action client, then the joint_positions service client and
        #     # then the joint_pose moveit service client. In order to do this we first
        #     # switch to the right controller then perform the command and then switch
        #     # back to the joint_effort controller.

        #     # TODO See if this can be done easier
        #     # Check which control service is available
        #     if self._services_connection_status["arm_joint_traj_control_client"]:

        #         # Convert joint_positions to joint_traj_action goal message
        #         goal_msg = self._joint_positions_2_follow_joint_trajectory_goal(
        #             joint_positions
        #         )

        #         # Send joint_traj action goal message
        #         self._controller_switcher.switch(
        #             control_group="arm", controller="panda_arm_controller"
        #         )  # Switch to joint_trajectory controller
        #         self._arm_joint_traj_control_client.send_goal(
        #             goal_msg, feedback_cb=self._joint_traj_control_feedback_callback
        #         )
        #         self._arm_joint_traj_control_client.wait_for_result()
        #         self._controller_switcher.switch(
        #             control_group="arm",
        #             controller="panda_arm_joint_group_effort_controller",
        #         )  # Switch back to joint_effort controller
        #         return True
        #     elif self._services_connection_status["arm_set_joint_positions_client"]:

        #         # Send goal to the joint position service
        #         self._controller_switcher.switch(
        #             control_group="arm",
        #             controller="panda_arm_joint_group_positions_controller",
        #         )  # Switch to joint_trajectory controller
        #         req = SetJointPositionsRequest()
        #         req.joint_positions = [
        #             joint_positions["panda_joint1"],
        #             joint_positions["panda_joint2"],
        #             joint_positions["panda_joint3"],
        #             joint_positions["panda_joint4"],
        #             joint_positions["panda_joint5"],
        #             joint_positions["panda_joint6"],
        #             joint_positions["panda_joint7"],
        #         ]
        #         self._arm_set_joint_positions_client.call(req)
        #         self._controller_switcher.switch(
        #             control_group="arm",
        #             controller="panda_arm_joint_group_effort_controller",
        #         )  # Switch back to joint_effort controller
        #         return True
        #     elif self._services_connection_status["set_joint_pose_client"]:
        #         # TEST

        #         # Send joint positions goal to the moveit joint pose client
        #         joint_point = SetJointPoseRequest()
        #         joint_point.point.positions = [None] * 7
        #         joint_point.point.positions[0] = joint_positions["panda_joint1"]
        #         joint_point.point.positions[1] = joint_positions["panda_joint2"]
        #         joint_point.point.positions[2] = joint_positions["panda_joint3"]
        #         joint_point.point.positions[3] = joint_positions["panda_joint4"]
        #         joint_point.point.positions[4] = joint_positions["panda_joint5"]
        #         joint_point.point.positions[5] = joint_positions["panda_joint6"]
        #         joint_point.point.positions[6] = joint_positions["panda_joint7"]
        #         self._set_joint_pose_client(joint_point)
        #         return True

    #############################################
    # Panda Robot env helper methods ############
    #############################################
    def _required_services_available(self):
        """Checks if all services required for the current 'robot_arm_control_type' are
        available.

        Returns
        -------
        bool
            Boolean specifying if the required services are available.
        dict
            Dictionary containing the missing services grouped by control_group.
        """

        # Get missing required services
        required_services = {
            "arm": REQUIRED_SERVICES_DICT["arm"][self.robot_arm_control_type],
            "hand": REQUIRED_SERVICES_DICT["hand"][self.robot_hand_control_type],
        }
        connected_services = [
            key for (key, val) in self._services_connection_status.items() if val
        ]
        missing_services = {}
        for (key, req_srvs) in required_services.items():
            for req_srv in req_srvs:
                if req_srv not in connected_services:
                    missing_services.update({key: req_srv})

        # Check whether any services are missing and return result
        if len(missing_services.values()) >= 1:
            return (False, missing_services)
        else:
            return (True, missing_services)

    def _check_all_sensors_ready(self):
        """Checks whether we are receiving sensor data.
        """
        self._check_joint_states_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_joint_states_ready(self):
        """Checks if we are receiving joint state
        sensor data.

        Returns
        -------
        sensor_msgs.msgs.JointState
            Array containing the joint states.
        """

        self.joints = None
        while self.joints is None and not rospy.is_shutdown():
            try:
                self.joints = rospy.wait_for_message(
                    "/joint_states", JointState, timeout=1.0
                )
                rospy.logdebug("Current /joint_states READY=>" + str(self.joints))

            except ROSException:
                rospy.logwarn(
                    "Current /joint_states not ready yet, retrying for getting "
                    "joint_states"
                )
        return self.joints

    ###############################################
    # Callback functions ##########################
    ###############################################
    def _joints_callback(self, data):
        """Callback function for the joint data subscriber.
        """
        self.joints = data

    def _joint_traj_control_feedback_callback(self, feedback):
        """Callback function for the joint traj action client.

        Parameters
        ----------
        feedback : control_msgs.msg.FollowJointTrajectoryFeedback
            Feedback received from the action client.
        """
        rospy.logdebug("Controlling the robot through the joint_traj action server:")
        rospy.logdebug("=Feedback=")
        rospy.logdebug(feedback)

    #############################################
    # Setup virtual methods #####################
    #############################################
    # NOTE: These virtual methods can be overloaded by the Robot env
    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()

    def _get_obs(self):
        """Gets obervation data

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()
