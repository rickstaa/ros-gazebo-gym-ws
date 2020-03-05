#! /usr/bin/env python
"""PandaReach Robot environment
This environment contains all the functions which are responsible
for interaction with the robot (Control and Sensors).
"""

import rospy

# import panda_robot_gazebo_goal_env # Needed for HER
import panda_robot_gazebo_env  # Needed for DDPG
import actionlib
import tf2_ros
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped


#################################################
# Panda Robot Environment Class##################
#################################################
class PandaRobotEnv(panda_robot_gazebo_env.RobotGazeboEnv):
    def __init__(
        self, robot_EE_link="panda_link8", robot_name_space="", controllers_list=[]
    ):
        """Initializes a new Panda Robot environment.

        Parameters
        ----------
        robot_EE_link : str, optional
            Robot end effector link name, by default "panda_link8"
        robot_name_space : str, optional
            Robot namespace, by default "".
        controllers_list : list, optional
            List containing the robot controllers you want to reset each time
            the simulation is reset, by default [].
        """

        # Environment initiation message
        rospy.loginfo("Initializing Panda Robot environment...")

        # Variables that we give through the constructor.
        self.controllers_list = controllers_list
        self.robot_name_space = robot_name_space
        self.robot_EE_link = robot_EE_link

        # Other class member variables
        self.panda_arm_joints = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ]

        # Create Needed subscribers
        JOINT_STATES_SUBSCRIBER = self.robot_name_space + "/joint_states"
        self.joint_states_sub = rospy.Subscriber(
            JOINT_STATES_SUBSCRIBER, JointState, self._joints_callback
        )
        self.joints = JointState()

        # Create transform listener
        self._tfBuffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tfBuffer)

        # Connect to robot control action server
        ARM_CONTROL_ACTION_SERVER_NAME = (
            robot_name_space + "/panda_arm_controller/follow_joint_trajectory/"
        )
        self._arm_control_action_client = actionlib.SimpleActionClient(
            ARM_CONTROL_ACTION_SERVER_NAME, FollowJointTrajectoryAction
        )

        # Setup action server parameters
        self._arm_control_action_client_time_from_start = 1

        # waits until the action server is up and running
        rospy.loginfo(
            "Waiting for robot control action Server " + ARM_CONTROL_ACTION_SERVER_NAME
        )
        self._arm_control_action_client.wait_for_server()
        rospy.loginfo("Action Server Found..." + ARM_CONTROL_ACTION_SERVER_NAME)

        # Initialize parent Class to setup the Gazebo environment)
        super(PandaRobotEnv, self).__init__(
            controllers_list=self.controllers_list,
            robot_name_space=self.robot_name_space,
            reset_controls=False,
        )

        # Environment initiation message
        rospy.loginfo("Panda Robot environment initialized")

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

    def set_arm_joints_positions(self, initial_qpos):
        """Set joint positions of the panda arm.

        Returns
        -------
        Boolean
            Boolean specifying whether reset was successful.
        """

        # Initiate action server msgs
        arm_control_joints_goal_msg = FollowJointTrajectoryGoal()
        arm_control_joints_goal_msg.trajectory.joint_names = self.panda_arm_joints

        # Fill joint state msg
        joint_states_msg = JointTrajectoryPoint()
        joint_states_msg.time_from_start.secs = (
            self._arm_control_action_client_time_from_start
        )
        joint_states_msg.positions = [
            initial_qpos["joint1"],
            initial_qpos["joint2"],
            initial_qpos["joint3"],
            initial_qpos["joint4"],
            initial_qpos["joint5"],
            initial_qpos["joint6"],
        ]

        # Send joint_trajectory_msg to arm control action server
        arm_control_joints_goal_msg.trajectory.points.append(joint_states_msg)
        self._arm_control_action_client.send_goal(
            arm_control_joints_goal_msg,
            feedback_cb=self._arm_action_server_feedback_callback,
        )
        return True

    def get_ee_pose(self):
        """Returns the current end effector pose.

        Returns
        -------
        geometry_msgs.PoseStamped
            Current end effector pose.
        """

        # Retrieve EE transform w.r.t. world
        try:
            ee_tf = self._tfBuffer.lookup_transform(
                "world", self.robot_EE_link, rospy.Time()
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn("Could not retrieve end effector pose.")
            rospy.logdebug(e)

        # Transform tf to pose
        ee_pose = PoseStamped()
        ee_pose.header = ee_tf.header
        ee_pose.pose.orientation = ee_tf.transform.rotation
        ee_pose.pose.position = ee_tf.transform.translation

        # Fix x-axis sign
        # NOTE: I noticed that the sign of the x axes was off
        # when requesting a transform using ht tf2_ros library
        ee_pose.pose.position.x = -ee_pose.pose.position.x

        # Return EE pose
        return ee_pose

    def get_ee_rpy(self):
        """Returns the current end effector rotation.
        """

        # Retrieve EE transform w.r.t. world
        try:
            ee_tf = self._tfBuffer.lookup_transform(
                "world", self.robot_EE_link, rospy.Time()
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn("Could not retrieve end effector pose.")
            rospy.logdebug(e)

        # Retrieve end effector rotation
        ee_rot = ee_tf.transform.rotation  # Quaternion
        ee_rpy = self._get_orientation_euler([ee_rot.x, ee_rot.y, ee_rot.z, ee_rot.w])
        return ee_rpy

    #############################################
    # Panda Robot env helper methods ############
    #############################################
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

            except:  # TODO: Fix exception type
                rospy.logerr(
                    "Current /joint_states not ready yet, retrying for getting "
                    "joint_states"
                )
        return self.joints

    def _get_orientation_euler(self, orientations):
        """Converts orientations to euler angles.

        Parameters
        ----------
        orientations : np.array
            Orientation quaternion (x, y, z, w).

        Returns
        -------
        np.array
            Array containing euler angles (roll, pitch, yaw).
        """
        # We convert from quaternions to euler
        roll, pitch, yaw = euler_from_quaternion(orientations)
        return roll, pitch, yaw

    def _arm_action_server_feedback_callback(self, feedback):
        """Callback function for the feedback call of the arm
        control action server.

        Parameters
        ----------
        feedback : control_msgs.msg.FollowJointTrajectoryFeedback
            Action server feedback message.
        """

        # Print action server feedback message
        rospy.logdebug("Robot Being Controlled")
        rospy.logdebug(feedback)

    def _joints_callback(self, data):
        """Callback function for the joint data subscriber.
        """
        self.joints = data

    #############################################
    # Setup virtual methods #####################
    #############################################
    # NOTE: These virtual methods can be overloaded by the Robot env
    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()
