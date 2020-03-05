#! /usr/bin/env python
"""PandaReach Robot environment
This environment contains all the functions which are responsible
for interaction with the robot (Control and Sensors).
"""

# Main python imports
import rospy
import panda_robot_gazebo_goal_env

# ROS python imports
import actionlib
import tf2_ros
from tf.transformations import euler_from_quaternion

# Ros msgs and srvs
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from panda_reach_training.srv import (
    getEePose,
    getEePoseRequest,
    getEeRpy,
    getEeRpyRequest,
    setEePose,
    setEePoseRequest,
    setJointPose,
    setJointPoseRequest,
)


#################################################
# Panda Robot Environment Class##################
#################################################
class PandaRobotEnv(panda_robot_gazebo_goal_env.RobotGazeboGoalEnv):
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

        # Create Moveit services
        self.ee_traj_client = rospy.ServiceProxy(
            "panda_moveit_planner_server/set_ee_pose", setEePose
        )
        self.joint_traj_client = rospy.ServiceProxy(
            "panda_moveit_planner_server/set_joint_pose", setJointPose
        )
        self.ee_pose_client = rospy.ServiceProxy(
            "panda_moveit_planner_server/get_ee_pose", getEePose
        )
        self.ee_rpy_client = rospy.ServiceProxy(
            "panda_moveit_planner_server/get_ee_rpy", getEeRpy
        )

        # Create transform listener
        self._tfBuffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tfBuffer)

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

    def get_ee_pose(self):
        """Returns the end effector EE pose.

        Returns
        -------
        geometry_msgs.msg.PoseStamped
            The current end effector pose.
        """

        # Retrieve end effector pose
        self._tfBuffer.lookup_transform("world", "panda_grip_site", rospy.Time())

        ## Retrieve end effector pose 2
        gripper_pose_req = getEePoseRequest()
        gripper_pose = self.ee_pose_client(gripper_pose_req)

        return gripper_pose

    def get_ee_rpy(self):
        """Returns the end effector EE orientation.

        Returns
        -------
        list
            List containing the roll (z), yaw (y), pitch (x) euler angles.
        """
        gripper_rpy_req = getEeRpyRequest()
        gripper_rpy = self.ee_rpy_client(gripper_rpy_req)

        return gripper_rpy

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

    def _joints_callback(self, data):
        """Callback function for the joint data subscriber.
        """
        self.joints = data

    def set_trajectory_ee(self, action):
        """
        Helper function.
        Wraps an action vector of joint angles into a JointTrajectory message.
        The velocities, accelerations, and effort do not control the arm motion
        """
        # Set up a trajectory message to publish.

        ee_target = setEePoseRequest()
        ee_target.pose.orientation.w = 1.0
        ee_target.pose.position.x = action[0]
        ee_target.pose.position.y = action[1]
        ee_target.pose.position.z = action[2]
        result = self.ee_traj_client(ee_target)

        return True

    def set_trajectory_joints(self, initial_qpos):
        """
        Helper function.
        Wraps an action vector of joint angles into a JointTrajectory message.
        The velocities, accelerations, and effort do not control the arm motion
        """
        # Set up a trajectory message to publish.

        joint_point = setJointPoseRequest()

        joint_point.point.positions = [None] * 7
        joint_point.point.positions[0] = initial_qpos["joint0"]
        joint_point.point.positions[1] = initial_qpos["joint1"]
        joint_point.point.positions[2] = initial_qpos["joint2"]
        joint_point.point.positions[3] = initial_qpos["joint3"]
        joint_point.point.positions[4] = initial_qpos["joint4"]
        joint_point.point.positions[5] = initial_qpos["joint5"]
        joint_point.point.positions[6] = initial_qpos["joint6"]

        result = self.joint_traj_client(joint_point)

        return True

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
