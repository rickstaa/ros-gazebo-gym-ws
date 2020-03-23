#! /usr/bin/env python
"""PandaReach Robot environment
This environment contains all the functions which are responsible
for interaction with the robot (Control and Sensors).
"""

# Main python imports
import sys
import panda_robot_gazebo_goal_env
from functions import action_server_exists
from panda_controller_switcher import PandaControlSwitcher

# ROS python imports
import rospy
import actionlib
import tf2_ros
from tf.transformations import euler_from_quaternion
from rospy.exceptions import ROSException, ROSInterruptException

# ROS msgs and srvs
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
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
    SetJointEffortsRequest,
    SetEePoseRequest,
    SetJointPose,
    SetJointPoseRequest,
    SetJointPositions,
    SetJointPositionsRequest,
)

# General script parameters
ROBOT_CONTROL_TYPES = [
    "joint_trajectory_control",
    "joint_position_control",
    "joint_effort_control",
    "joint_group_position_control",
    "joint_group_effort_control",
    "ee_control",
]
REQUIRED_SERVICES_DICT = {
    "joint_trajectory_control": ["arm_joint_traj_control_client"],
    "joint_position_control": ["arm_set_joint_positions_client"],
    "joint_effort_control": ["arm_set_joint_efforts_client"],
    "joint_group_position_control": ["arm_set_joint_positions_client"],
    "joint_group_effort_control": ["arm_set_joint_efforts_client"],
    "ee_control": [
        "set_ee_pose_client",
        "set_joint_pose_client",
        "get_ee_pose_client",
        "get_ee_rpy_client",
        "set_ee_client",
        "get_ee_client",
    ],
}

# TODO: Add panda_hand_control_type


#################################################
# Panda Robot Environment Class #################
#################################################
class PandaRobotEnv(panda_robot_gazebo_goal_env.RobotGazeboGoalEnv):
    def __init__(
        self,
        robot_EE_link="panda_grip_site",
        robot_arm_control_type="joint_position_control",
        robot_hand_control_type="joint_position_control",
        robot_name_space="",
        controllers_list=[],
    ):
        """Initializes a new Panda Robot environment.

        Parameters
        ----------
        robot_EE_link : str, optional
            Robot end effector link name, by default "panda_grip_site"
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
        controllers_list : list, optional
            List containing the robot controllers you want to reset each time
            the simulation is reset, by default [].
        """

        # Environment initiation message
        rospy.loginfo("Initializing Panda Robot environment.")

        # Check constructor arguments
        # TODO: ADD HAND
        control_type_invallid = []
        if robot_arm_control_type.lower() not in ROBOT_CONTROL_TYPES:
            rospy.logerr(
                "Shutting down '%s' because robot control type that was specified '%s' "
                "is invalid please use one of the following robot control types: %s."
                % (
                    rospy.get_name(),
                    robot_arm_control_type.lower(),
                    ROBOT_CONTROL_TYPES,
                )
            )
            sys.exit(0)

        # Variables that we give through the constructor.
        self.controllers_list = controllers_list
        self.robot_name_space = robot_name_space
        self.robot_EE_link = robot_EE_link
        self.robot_arm_control_type = robot_arm_control_type
        self.robot_hand_control_type = robot_hand_control_type

        # Initiate other class members
        self._arm_joint_traj_control_topic = (
            "panda_arm_controller/follow_joint_trajectory"
        )
        self._arm_joint_efforts_control_topic = (
            "panda_control_server/panda_arm/set_joint_efforts"
        )
        self._arm_joint_positions_control_topic = (
            "panda_control_server/panda_arm/set_joint_positions"
        )
        self._panda_moveit_planner_connection_timeout = 5
        self._controller_switcher_connection_timeout = 5
        self._arm_control_connection_timeout = rospy.Duration(secs=10)
        self._joint_traj_action_server_step_size = 1  # Time from send [sec]
        self._moveit_services = []
        self._services_connected = {}

        # Create Needed subscribers
        rospy.loginfo("Setting up sensor data subscribers.")
        joint_states_topic = "joint_states"
        self.joints = JointState()
        self.joint_states_sub = rospy.Subscriber(
            joint_states_topic, JointState, self._joints_callback
        )

        # Create controller switcher
        self.controller_switcher = PandaControlSwitcher(
            connection_timeout=self._controller_switcher_connection_timeout
        )

        # Create transform listener
        self._tfBuffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tfBuffer)

        #########################################
        # Connect to control services ###########
        #########################################
        # NOTE: Depending on the type of control we require the panda moveit services,
        # the joint trajectory action service, the joint effort control service or the
        # joint position control service. All these services are initiated here if they
        # are available.
        rospy.loginfo("Connecting to robot control services.")
        self._moveit_services.append("set_ee_pose_client")
        try:
            rospy.logdebug(
                "Connecting to 'panda_moveit_planner_server/set_ee_pose' service."
            )
            rospy.wait_for_service(
                "panda_moveit_planner_server/set_ee_pose",
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._set_ee_pose_client = rospy.ServiceProxy(
                "panda_moveit_planner_server/set_ee_pose", SetEePose
            )
            rospy.logdebug(
                "Connected to 'panda_moveit_planner_server/set_ee_pose' service!"
            )
            self._services_connected["set_ee_pose_client"] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to 'panda_moveit_planner_server/set_ee_pose' "
                "service!"
            )
            self._services_connected["set_ee_pose_client"] = False

        # Connect to Moveit set_joint_pose service
        self._moveit_services.append("set_joint_pose_client")
        try:
            rospy.logdebug(
                "Connecting to 'panda_moveit_planner_server/set_joint_pose' " "service."
            )
            rospy.wait_for_service(
                "panda_moveit_planner_server/set_joint_pose",
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._set_joint_pose_client = rospy.ServiceProxy(
                "panda_moveit_planner_server/set_joint_pose", SetJointPose
            )
            rospy.logdebug(
                "Connected to 'panda_moveit_planner_server/set_joint_pose' service!"
            )
            self._services_connected["set_joint_pose_client"] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to 'panda_moveit_planner_server/set_joint_pose' "
                "service!"
            )
            self._services_connected["set_joint_pose_client"] = False

        # Connect to Moveit get_ee_pose service
        self._moveit_services.append("get_ee_pose_client")
        try:
            rospy.logdebug(
                "Connecting to 'panda_moveit_planner_server/get_ee_pose' service."
            )
            rospy.wait_for_service(
                "panda_moveit_planner_server/get_ee_pose",
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._get_ee_pose_client = rospy.ServiceProxy(
                "panda_moveit_planner_server/get_ee_pose", GetEePose
            )
            rospy.logdebug(
                "Connected to 'panda_moveit_planner_server/get_ee_pose' service!"
            )
            self._services_connected["get_ee_pose_client"] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to 'panda_moveit_planner_server/get_ee_pose' "
                "service!"
            )
            self._services_connected["get_ee_pose_client"] = False

        # Connect to Moveit get_ee_rpy service
        self._moveit_services.append("get_ee_rpy_client")
        try:
            rospy.logdebug(
                "Connecting to 'panda_moveit_planner_server/get_ee_rpy' service."
            )
            rospy.wait_for_service(
                "panda_moveit_planner_server/get_ee_rpy",
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._get_ee_pose_client = rospy.ServiceProxy(
                "panda_moveit_planner_server/get_ee_rpy", GetEeRpy
            )
            rospy.logdebug(
                "Connected to 'panda_moveit_planner_server/get_ee_rpy' service!"
            )
            self._services_connected["get_ee_rpy_client"] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to 'panda_moveit_planner_server/get_ee_rpy' "
                "service!"
            )
            self._services_connected["get_ee_rpy_client"] = False

        # Connect to Moveit set_ee service
        self._moveit_services.append("set_ee_client")
        try:
            rospy.logdebug(
                "Connecting to 'panda_moveit_planner_server/set_ee' service."
            )
            rospy.wait_for_service(
                "panda_moveit_planner_server/set_ee",
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._set_ee_client = rospy.ServiceProxy(
                "panda_moveit_planner_server/set_ee", SetEe
            )
            rospy.logdebug("Connected to 'panda_moveit_planner_server/set_ee' service!")
            self._services_connected["set_ee_client"] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to 'panda_moveit_planner_server/set_ee' service!"
            )
            self._services_connected["set_ee_client"] = False

        # Connect to Moveit get_ee service
        self._moveit_services.append("get_ee_client")
        try:
            rospy.logdebug(
                "Connecting to 'panda_moveit_planner_server/get_ee' service."
            )
            rospy.wait_for_service(
                "panda_moveit_planner_server/get_ee",
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._get_ee_client = rospy.ServiceProxy(
                "panda_moveit_planner_server/get_ee", GetEe
            )
            rospy.logdebug("Connected to 'panda_moveit_planner_server/get_ee' service!")

            # Validate class set EE with moveit end effector
            self._moveit_ee = self._get_ee_client(GetEeRequest()).ee_name
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
                response = self._set_ee_client(req)

                # Check ee set result
                if not response.success:
                    rospy.logerr(
                        "Shutting down '%s' because '%s' could not be set as "
                        "the Moveit move_group end effector (EE). Please "
                        "check that the EE you initiate the panda_robot_env "
                        "class with is vallid." % (rospy.get_name(), self.robot_EE_link)
                    )
                    sys.exit(0)
            else:
                self._services_connected["get_ee_client"] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to 'panda_moveit_planner_server/get_ee' service!"
            )
            self._services_connected["get_ee_client"] = False

        # Connect to Joint Trajectory Panda Arm Control (action) service
        rospy.logdebug(
            "Connecting to '%s' action service." % self._arm_joint_traj_control_topic
        )
        if action_server_exists(self._arm_joint_traj_control_topic):  # Check if exists

            # Connect to robot control action server
            self._arm_joint_traj_control_client = actionlib.SimpleActionClient(
                self._arm_joint_traj_control_topic, FollowJointTrajectoryAction
            )
            # Waits until the action server has started up
            retval = self._arm_joint_traj_control_client.wait_for_server(
                timeout=self._arm_control_connection_timeout
            )
            if not retval:
                rospy.logwarn(
                    "No connection could be established with the '%s' service. "
                    "The Panda Robot Environment therefore can not use this action "
                    "service to control the Panda Robot."
                    % (self._arm_joint_traj_control_topic)
                )
                self._services_connected["arm_joint_traj_control_client"] = False
            else:
                self._services_connected["arm_joint_traj_control_client"] = True
        else:
            rospy.logwarn(
                "No connection could be established with the '%s' service. "
                "The Panda Robot Environment therefore can not use this action "
                "service to control the Panda Robot."
                % (self._arm_joint_traj_control_topic)
            )
            self._services_connected["arm_joint_traj_control_client"] = False

        # Connect to panda_control_server/set_joint_positions service
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % self._arm_joint_positions_control_topic
            )
            rospy.wait_for_service(
                self._arm_joint_positions_control_topic,
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._arm_set_joint_positions_client = rospy.ServiceProxy(
                self._arm_joint_positions_control_topic, SetJointPositions
            )
            rospy.logdebug(
                "Connected to '%s' service!" % self._arm_joint_positions_control_topic
            )
            self._services_connected["arm_set_joint_positions_client"] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "No connection could be established with the '%s' service. "
                "The Panda Robot Environment therefore can not use this service "
                "to control the Panda Robot." % self._arm_joint_positions_control_topic
            )
            self._services_connected["arm_set_joint_positions_client"] = False

        # Connect to panda_control_server/set_joint_efforts service
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % self._arm_joint_efforts_control_topic
            )
            rospy.wait_for_service(
                self._arm_joint_efforts_control_topic,
                timeout=self._panda_moveit_planner_connection_timeout,
            )
            self._arm_set_joint_efforts_client = rospy.ServiceProxy(
                self._arm_joint_efforts_control_topic, SetJointEfforts
            )
            rospy.logdebug(
                "Connected to '%s service!" % self._arm_joint_efforts_control_topic
            )
            self._services_connected["arm_set_joint_efforts_client"] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "No connection could be established with the '%s' service. "
                "The Panda Robot Environment therefore can not use this service "
                "to control the Panda Robot." % self._arm_joint_efforts_control_topic
            )
            self._services_connected["arm_set_joint_efforts_client"] = False

        #########################################
        # Validate service control connections ##
        #########################################
        # NOTE: Shut down ROS node if the services that are needed for a given control
        # type are not available.
        (retval, missing_srvs) = self._required_services_available()

        # Shut down ROS node if required services are not available
        if not retval:

            # Create error message and send log
            rospy.logerr(
                "Shutting down '%s' because services '%s' which are needed for "
                "controlling the robot using '%s' control are not available."
                % (rospy.get_name(), missing_srvs, self.robot_arm_control_type)
            )
            sys.exit(0)

        #########################################
        # Switch to right controller parent #####
        #########################################

        # Switch to the right controller
        rospy.loginfo("Switching to required controller.")
        self.controller_switcher.switch(
            control_group="arm", controller=self.robot_arm_control_type
        )
        self.controller_switcher.switch(
            control_group="hand", controller=self.robot_hand_control_type
        )

        #########################################
        # Initialize parent class ###############
        #########################################

        # Initialize parent Class to setup the Gazebo environment)
        super(PandaRobotEnv, self).__init__(
            controllers_list=self.controllers_list,
            robot_name_space=self.robot_name_space,
            reset_controls=False,
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

        # TODO: use tf
        gripper_rpy_req = GetEeRpyRequest()
        gripper_rpy = self._get_ee_pose_client(gripper_rpy_req)

        return gripper_rpy

    def set_ee_pose(self, action):
        """Wraps an action vector of joint angles into a JointTrajectory message.
        The velocities, accelerations, and effort do not control the arm motion
        """

        # Set up a trajectory message to publish.
        ee_target = SetEePoseRequest()
        ee_target.pose.orientation.w = 1.0
        ee_target.pose.position.x = action[0]
        ee_target.pose.position.y = action[1]
        ee_target.pose.position.z = action[2]
        self._set_ee_pose_client(ee_target)
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

        # Set up a trajectory message to publish
        # Add Joint_Trajectory control
        # TODO: Add gripper control
        if self.robot_arm_control_type == "ee_control":  # Use moveit service
            joint_point = SetJointPoseRequest()
            joint_point.point.positions = [None] * 7
            joint_point.point.positions[0] = joint_positions["panda_joint1"]
            joint_point.point.positions[1] = joint_positions["panda_joint2"]
            joint_point.point.positions[2] = joint_positions["panda_joint3"]
            joint_point.point.positions[3] = joint_positions["panda_joint4"]
            joint_point.point.positions[4] = joint_positions["panda_joint5"]
            joint_point.point.positions[5] = joint_positions["panda_joint6"]
            joint_point.point.positions[6] = joint_positions["panda_joint7"]
            self._set_joint_pose_client(joint_point)
            return True
        elif (
            self.robot_arm_control_type == "joint_trajectory_control"
        ):  # Use joint traj action service

            # Convert joint_positions to joint_traj_action goal message
            joint_positions = dict.fromkeys(joint_positions, 0)
            goal_msg = self._joint_positions_2_follow_joint_trajectory_goal(
                joint_positions
            )

            # Send joint_traj action goal message
            self._arm_joint_traj_control_client.send_goal(
                goal_msg, feedback_cb=self._joint_traj_control_feedback_callback
            )
            return True
        if (
            self.robot_arm_control_type == "joint_position_control"
        ):  # Use joint traj action service

            # Send goal to the joint position service
            req = SetJointPositionsRequest()
            req.joint_positions.data = [
                joint_positions["panda_joint1"],
                joint_positions["panda_joint2"],
                joint_positions["panda_joint3"],
                joint_positions["panda_joint4"],
                joint_positions["panda_joint5"],
                joint_positions["panda_joint6"],
                joint_positions["panda_joint7"],
            ]
            self._arm_set_joint_positions_client.call(req)
            return True
        else:  # Joint_effort control
            # NOTE: When joint_effort control is chosen we first try to control using
            # the joint_traj action client, then the joint_positions service client and
            # then the joint_pose moveit service client. In order to do this we first
            # switch to the right controller then perform the command and then switch
            # back to the joint_effort controller.

            # TODO See if this can be done easier
            # Check which control service is available
            if self._services_connected["arm_joint_traj_control_client"]:

                # Convert joint_positions to joint_traj_action goal message
                goal_msg = self._joint_positions_2_follow_joint_trajectory_goal(
                    joint_positions
                )

                # Send joint_traj action goal message
                self.controller_switcher.switch(
                    control_group="arm", controller="panda_arm_controller"
                )  # Switch to joint_trajectory controller
                self._arm_joint_traj_control_client.send_goal(
                    goal_msg, feedback_cb=self._joint_traj_control_feedback_callback
                )
                self._arm_joint_traj_control_client.wait_for_result()
                self.controller_switcher.switch(
                    control_group="arm",
                    controller="panda_arm_joint_group_effort_controller",
                )  # Switch back to joint_effort controller
                return True
            elif self._services_connected["arm_set_joint_positions_client"]:

                # Send goal to the joint position service
                self.controller_switcher.switch(
                    control_group="arm",
                    controller="panda_arm_joint_group_positions_controller",
                )  # Switch to joint_trajectory controller
                req = SetJointPositionsRequest()
                req.joint_positions = [
                    joint_positions["panda_joint1"],
                    joint_positions["panda_joint2"],
                    joint_positions["panda_joint3"],
                    joint_positions["panda_joint4"],
                    joint_positions["panda_joint5"],
                    joint_positions["panda_joint6"],
                    joint_positions["panda_joint7"],
                ]
                self._arm_set_joint_positions_client.call(req)
                self.controller_switcher.switch(
                    control_group="arm",
                    controller="panda_arm_joint_group_effort_controller",
                )  # Switch back to joint_effort controller
                return True
            elif self._services_connected["set_joint_pose_client"]:
                # TEST

                # Send joint positions goal to the moveit joint pose client
                joint_point = SetJointPoseRequest()
                joint_point.point.positions = [None] * 7
                joint_point.point.positions[0] = joint_positions["panda_joint1"]
                joint_point.point.positions[1] = joint_positions["panda_joint2"]
                joint_point.point.positions[2] = joint_positions["panda_joint3"]
                joint_point.point.positions[3] = joint_positions["panda_joint4"]
                joint_point.point.positions[4] = joint_positions["panda_joint5"]
                joint_point.point.positions[5] = joint_positions["panda_joint6"]
                joint_point.point.positions[6] = joint_positions["panda_joint7"]
                self._set_joint_pose_client(joint_point)
                return True

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
        list
            List containing the missing services.
        """

        # Check services for each control type
        if self.robot_arm_control_type == "ee_control":

            # Check if all needed services for 'ee_control' are ready
            missing_srvs = [
                key
                for (key, val) in self._services_connected.items()
                if key in self._moveit_services and not val
            ]
            if len(missing_srvs) >= 1:
                return (False, missing_srvs)
            else:
                return (True, missing_srvs)
        elif self.robot_arm_control_type == "joint_trajectory_control":

            # Check if all needed services for 'joint_trajectory_control' are ready
            if not self._services_connected["arm_joint_traj_control_client"]:
                return (False, ["arm_joint_traj_control_client"])
            else:
                return (True, [])
        elif self.robot_arm_control_type == "joint_position_control":

            # Check if all needed services for 'joint_position_control' are ready
            if not self._services_connected["arm_set_joint_positions_client"]:
                return (False, ["arm_set_joint_positions_client"])
            else:
                return (True, [])
        elif self.robot_arm_control_type == "joint_effort_control":

            # Check if all needed services for 'joint_position_control' are ready
            if not self._services_connected["arm_set_joint_efforts_client"]:
                return (False, "arm_set_joint_efforts_client")
            else:
                if not any(
                    [
                        val
                        for (key, val) in self._services_connected.items()
                        if key
                        in [
                            "set_joint_pose_client",
                            "arm_joint_traj_control_client",
                            "arm_set_joint_positions_client",
                        ]
                    ]
                ):
                    return (
                        False,
                        [
                            "set_joint_pose_client",
                            "arm_joint_traj_control_client",
                            "arm_set_joint_positions_client",
                        ],
                    )
                else:
                    return (True, [])

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

    def _joint_positions_2_follow_joint_trajectory_goal(self, joint_positions):
        """Converts a dictionary of joint_positions into a FollowJointTrajectoryGoal
        msgs.

        Parameters
        ----------
        joint_positions : dict
            The joint positions of each of the robot joints.
        """

        # creates a goal to send to the action server
        goal_msg = FollowJointTrajectoryGoal()
        joint_states = JointTrajectoryPoint()
        joint_states.time_from_start.secs = (
            self._joint_traj_action_server_step_size
        )  # Time from start
        joint_states.positions = [
            joint_positions["panda_joint1"],
            joint_positions["panda_joint2"],
            joint_positions["panda_joint3"],
            joint_positions["panda_joint4"],
            joint_positions["panda_joint5"],
            joint_positions["panda_joint6"],
            joint_positions["panda_joint7"],
        ]
        goal_msg.trajectory.joint_names = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ]
        goal_msg.trajectory.points.append(joint_states)

        # Return goal msgs
        return goal_msg

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
