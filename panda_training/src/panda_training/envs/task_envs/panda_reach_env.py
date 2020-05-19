"""This Task Environment will be in charge of providing all the necessary
functions and methods related to this specific training.

NOTE
------
Euler Angles are all relative with 'xyz' axes ordering
"""

# Main python imports
from gym import utils
from gym import spaces
from gym.envs.registration import register
import numpy as np
import sys
from panda_training.envs.robot_envs import PandaRobotEnv
from panda_training.exceptions import EePoseLookupError
from panda_training.functions import (
    flatten_list,
    get_orientation_euler,
    action_list_2_action_dict,
)

# ROS python imports
import rospy
from panda_training.extras import GoalMarker

# ROS msgs and srvs
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose

# Register Openai gym environment
register(
    id="PandaReach-v0",
    entry_point="panda_training.envs.task_envs:PandaReachTaskEnv",
    max_episode_steps=1000,
)

# CLEAN: Cleanup code
# TODO: Create robot arm and hand control
# TODO: Add visual goal position and in inference
# FIXME: Add a way for the robot to stop (if goal not valid --> Choose new goal).

# Script parameters
# TODO: ADD goal chose region to ROS PARAMETER FILE.
# TODO: Create min max object or use existing type
GOAL_XMIN = 0.3
GOAL_YMIN = -1
GOAL_ZMIN = 0.4
GOAL_XMAX = 1
GOAL_YMAX = 1
GOAL_ZMAX = 2


#################################################
# Panda Robot Environment Class #################
#################################################
class PandaReachTaskEnv(PandaRobotEnv, utils.EzPickle):
    """Class that provides all the methods used for the algorithm training.

    Attributes
    ----------
    action_space : gym.spaces.box.Box
        Gym action space object.
    observation_space : gym.spaces.dict.Dict
        Gym observation space object.

    Methods
    ----------
    get_params():
        Get training and simulation parameters from the ROS parameter server.
    goal_distance(goal_a, goal_b):
        Calculates the perpendicular distance to the goal.
    robot_get_obs(data):
        Returns all joint positions and velocities associated with a robot.
    """

    def __init__(
        self,
        robot_EE_link="panda_grip_site",
        robot_arm_control_type="joint_position_control",
        robot_hand_control_type="joint_position_control",
    ):
        """Initializes a Panda Reach task environment.


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
        """

        # Create class attributes
        self._sim_time = rospy.get_time()
        # TODO: Transform to twists
        self._prev_grip_pos = np.zeros(3)
        self._prev_object_pos = np.zeros(3)
        self._prev_object_rot = np.zeros(3)
        self._robot_arm_control_type = robot_arm_control_type.lower()
        self._robot_hand_control_type = robot_hand_control_type.lower()

        # Wait for the simulation to be started
        wait_for_sim_timeout = 60
        simulation_check_timeout_time = rospy.get_rostime() + rospy.Duration(
            wait_for_sim_timeout
        )
        while (
            not rospy.is_shutdown()
            and rospy.get_rostime() < simulation_check_timeout_time
        ):
            if any(
                [
                    "/gazebo" in topic
                    for topic in flatten_list(rospy.get_published_topics())
                ]
            ):
                break
            else:
                rospy.logwarn_once(
                    "Waiting for the Panda Gazebo robot simulation to be " "started."
                )
        else:
            rospy.logerr(
                "Shutting down '%s' since the Panda Gazebo simulation was not "
                "started within the set timeout period of %s seconds."
                % (rospy.get_name(), wait_for_sim_timeout)
            )
            sys.exit(0)

        # Retrieve parameters from parameter server
        rospy.loginfo("Initializing Panda task environment.")
        self.get_params()

        # Initialize parent Class to setup the Robot environment)
        super(PandaReachTaskEnv, self).__init__(
            robot_EE_link=robot_EE_link,
            robot_arm_control_type=self._robot_arm_control_type,
            robot_hand_control_type=self._robot_hand_control_type,
        )
        utils.EzPickle.__init__(self)

        # Create publisher to publish the place pose
        rospy.logdebug("Creating goal pose publisher.")
        self._goal_pose_pub = rospy.Publisher(
            "panda_training/current_goal", Marker, queue_size=10
        )
        rospy.logdebug("Goal pose publisher created.")
        rospy.logdebug("Creating goal pose publisher.")
        self._goal_pose_pub2 = rospy.Publisher(
            "panda_training/current_goal2", PointStamped, queue_size=10
        )
        rospy.logdebug("Goal pose publisher created.")

        # Setup task environment
        rospy.logdebug("Setup initial environment state.")
        self._env_setup(init_qpose=self.init_qpose)

        # Get observations
        rospy.logdebug("Get initial observation.")
        obs = self._get_obs()

        # Set gym action and observation space
        rospy.logdebug("Setup gym action and observation space.")
        # QUESTION: Clip action space
        # Create action space
        # TODO: Add option for hand asswell
        # TODO: Use parameter server or only here!
        if self._robot_arm_control_type == "ee_control":
            self.action_space = spaces.Box(
                -1.0, 1.0, shape=(self.n_actions,), dtype="float32"
            )
        elif self._robot_arm_control_type == "joint_position_control":
            self.action_space = spaces.Box(-1.0, 1.0, shape=(7,), dtype="float32")
            # TODO: Add gripper joint later
            # Question: Do we want to learn gripper control

        # Create observation space
        self.observation_space = spaces.Dict(
            dict(
                desired_goal=spaces.Box(
                    -np.inf, np.inf, shape=obs["achieved_goal"].shape, dtype="float32"
                ),
                achieved_goal=spaces.Box(
                    -np.inf, np.inf, shape=obs["achieved_goal"].shape, dtype="float32"
                ),
                observation=spaces.Box(
                    -np.inf, np.inf, shape=obs["observation"].shape, dtype="float32"
                ),
            )
        )

        # Environment initiation complete message
        rospy.loginfo("Panda Panda task environment initialized.")

    #############################################
    # Panda Robot env main methods ##############
    #############################################
    def get_params(self):
        """Get training and simulation parameters. These parameters have to be loaded on
        the parameter server using a yaml file.
        """
        # Check if parameters were loaded onto the parameter server

        # Retrieve parameters
        self.n_actions = self._get_params("panda_training/n_actions")
        self.has_object = rospy.get_param("panda_training/has_object")
        self.block_gripper = rospy.get_param("panda_training/block_gripper")
        self.n_substeps = rospy.get_param("panda_training/n_substeps")
        self.gripper_extra_height = rospy.get_param(
            "panda_training/gripper_extra_height"
        )
        self.target_in_the_air = rospy.get_param("panda_training/target_in_the_air")
        self.target_offset = rospy.get_param("panda_training/target_offset")
        self.obj_range = rospy.get_param("panda_training/obj_range")
        self.target_range = rospy.get_param("panda_training/target_range")
        self.distance_threshold = rospy.get_param("panda_training/distance_threshold")
        self.reward_type = rospy.get_param("panda_training/reward_type")
        self.init_qpose = rospy.get_param("panda_training/init_qpose")
        # self.

    def goal_distance(self, goal_a, goal_b):
        """Calculates the perpendicular distance to the goal.

        Parameters
        ----------
        goal_a : np.array
            List containing a gripper and object pose.
        goal_b : np.array
            List containing a gripper and object pose.

        Returns
        -------
        np.float32
            Perpendicular distance to the goal.
        """
        assert goal_a.shape == goal_b.shape
        return np.linalg.norm(goal_a - goal_b, axis=-1)

    def _sample_goal(self):
        """Sample a grasping goal. Sample from objects if environment has object
        otherwise create random goal.

        Returns
        -------
        geometry_msgs.PoseStamped
            A goal pose.
        """
        if self.has_object:  # Environment has object

            # Set random goal pose
            # Note: Takes into account gripper and target offset
            goal = self.initial_gripper_pos + self.np_random.uniform(
                -self.target_range, self.target_range, size=3
            )
            goal += self.target_offset
            goal[2] = self.height_offset
            if self.target_in_the_air and self.np_random.uniform() < 0.5:
                goal[2] += self.np_random.uniform(0, 0.45)
        else:

            # Get a goal
            goal = self.initial_gripper_pos + self.np_random.uniform(
                -0.15, 0.15, size=3
            )

            # Clip goal
            # QUESTION: Do we want to clip the goal?
            goal = self.clip_goal_position(goal)

        # Generate Rviz marker
        goal_maker_pose = Pose()
        goal_maker_pose.position.x = goal[0]
        goal_maker_pose.position.y = goal[1]
        goal_maker_pose.position.z = goal[2]
        goal_marker = GoalMarker(pose=goal_maker_pose)

        # Publish goal marker
        self._goal_pose_pub.publish(goal_marker)

        # return goal.copy()
        return goal

    def _sample_achieved_goal(self, grip_pos, object_pos):
        # TODO: Update docstring
        """Retrieve current position. If gripper has object
        return object position.

        Parameters
        ----------
        grip_pos : np.array
            Gripper position.
        object_pos : np.array
            Object position.

        Returns
        -------
        geometry_msgs.PoseStamped
            The achieved pose.
        """

        # Retrieve gripper end pose
        if not self.has_object:  # Environment has no object
            achieved_goal = grip_pos.copy()
        else:  # Has object
            achieved_goal = np.squeeze(object_pos.copy())

        # return achieved_goal.copy()
        return achieved_goal

    def robot_get_obs(self, data):
        """Returns all joint positions and velocities associated with a robot.

        Parameters
        ----------
        param : sensor_msgs/JointState
            Joint states message.

        Returns
        -------
        np.array
           Robot Positions, Robot Velocities
        """

        # Retrieve positions and velocity out of sensor_msgs/JointState msgs
        if data.position is not None and data.name:
            names = [n for n in data.name]
            return (
                np.array([data.position[i] for i in range(len(names))]),
                np.array([data.velocity[i] for i in range(len(names))]),
            )
        else:
            return np.zeros(0), np.zeros(0)

    #############################################
    # Overload Gazebo env virtual methods #######
    #############################################
    def _set_init_pose(self, init_qpose=None):
        """Sets the Robot in its init pose.

        Parameters
        ----------
        init_qpose : dict, optional
            Dictionary containing the initial joint positions for the Panda joints,
            by default :attr:`self.init_qpose`. Example: {"panda_link0": 1.5}
        Returns
        -------
        Boolean
            Boolean specifying whether reset was successful.
        """

        # Check if init_qpose was supplied
        if init_qpose is None:
            init_qpose = self.init_qpose

        # Set initial pose
        self.gazebo.unpauseSim()
        self.set_joint_positions(init_qpose, wait=True)
        return True

    def _get_obs(self):
        """Get robot state observation.

        Returns
        -------
        dict
            A dictionary containing the {observation, achieved_goal, desired_goal):

            observation (22x1):
                - End effector x position
                - End effector y position
                - End effector z position
                - Object x position
                - Object y position
                - Object z position
                - Object/gripper rel. x position
                - Object/gripper rel. y position
                - Object/gripper rel. z position
                - Gripper finger 1 x position
                - Gripper finger 2 x position
                - Object pitch (y)
                - Object yaw (z)
                - Object roll (x)
                - Object x velocity
                - Object y velocity
                - Object z velocity
                - Object x angular velocity
                - Object y angular velocity
                - Object z angular velocity
                - Gripper finger 1 x velocity
                - Gripper finger 2 x velocity
        """

        # Retrieve robot end effector pose and orientation
        # CLEAN: Clean up code
        # IMPROVE: Change to TF if moveit not available
        grip_pose = self._moveit_get_ee_pose_client()
        grip_pos = np.array(
            [
                grip_pose.pose.position.x,
                grip_pose.pose.position.y,
                grip_pose.pose.position.z,
            ]
        )
        # grip_rpy = self._moveit_get_ee_rpy_client()  # Retrieve as euler angles

        # Retrieve robot joint pose and velocity
        # IMPROVE: Retrieve real velocity from gazebo
        dt = self.get_elapsed_time()
        grip_velp = (
            grip_pos - self._prev_grip_pos
        ) / dt  # Velocity(position) = Distance/Time
        robot_qpos, robot_qvel = self.robot_get_obs(self.joint_states)

        # Get gripper pose and (angular)velocity
        # TODO: Fix to gripper_grasp_site
        gripper_state = robot_qpos[0:2]
        gripper_vel = robot_qvel[0:2]

        # Get object pose and (angular)velocity
        if self.has_object:

            # Get object pose
            object_pos = [
                self.model_states["object0"]["pose"].position.x,
                self.model_states["object0"]["pose"].position.y,
                self.model_states["object0"]["pose"].position.z,
            ]

            # Get object orientation
            object_quat = [self.model_states["object0"]["pose"].orientation]
            object_rot_resp = get_orientation_euler(object_quat)
            object_rot = [object_rot_resp.y, object_rot_resp.p, object_rot_resp.r]

            # Get object velocity
            object_velp = (
                object_pos - self.prev_object_pos
            ) / dt  # Velocity(position) = Distance/Time
            object_velr = (
                object_rot - self.prev_object_rot
            ) / dt  # Velocity(rotation) = Rotation/Time

            # Get relative position and velocity
            object_rel_pos = object_pos - grip_pos
            object_velp -= grip_velp
        else:
            object_pos = (
                object_rot
            ) = object_velp = object_velr = object_rel_pos = np.zeros(0)

        # Get achieved goal
        achieved_goal = self._sample_achieved_goal(grip_pos, object_pos)

        # Concatenate observations
        obs = np.concatenate(
            [
                grip_pos,
                object_pos.ravel(),
                object_rel_pos.ravel(),
                gripper_state,
                object_rot.ravel(),
                object_velp.ravel(),
                object_velr.ravel(),
                gripper_vel,
            ]
        )

        # Save current gripper and object positions
        self._prev_grip_pos = grip_pos
        self._prev_object_pos = object_pos
        self._prev_object_rot = object_rot

        # Return goal env observation dictionary
        return {
            "observation": obs.copy(),
            "achieved_goal": achieved_goal.copy(),
            "desired_goal": self.goal.copy(),
        }

    def _init_env_variables(self):
        """Inits variables needed to be initialized each time we reset at the start
        of an episode.
        """
        pass

    def _set_action(self, action):
        # TODO UPDATE DOCSTRING
        """Take robot action.
        """

        # QUESTION: Algorithm starts trying in postion where it is in collision. Maybe
        # help a little?

        if self._robot_arm_control_type == "ee_control":

            # Validate action
            # TODO: Get action shape from __init__ instead of hardcode
            assert action.shape == (4,)
            action = (
                action.copy()
            )  # ensure that we don't change the action outside of this scope
            pos_ctrl, gripper_ctrl = action[:3], action[3]
            # pos_ctrl *= 0.05  # limit maximum change in position
            rot_ctrl = [
                1.0,
                0.0,
                1.0,
                0.0,
            ]  # fixed rotation of the end effector, expressed as a quaternion
            gripper_ctrl = np.array([gripper_ctrl, gripper_ctrl])
            assert gripper_ctrl.shape == (2,)
            if self.block_gripper:
                gripper_ctrl = np.zeros_like(gripper_ctrl)
            action = np.concatenate([pos_ctrl, rot_ctrl, gripper_ctrl])

            # Print Debug info
            rospy.logdebug("=Action set info=")
            rospy.logdebug("Action that is set:")
            rospy.logdebug(action)

            # Take action
            self.set_ee_pose(action)
        elif self._robot_arm_control_type == "joint_position_control":

            # Validate action
            # TODO: Get action shape from __init__ instead of hardcode
            assert action.shape == (7,)
            action = (
                action.copy()
            )  # ensure that we don't change the action outside of this scope

            # Print Debug info
            rospy.logdebug("=Action set info=")
            rospy.logdebug("Action that is set:")
            rospy.logdebug(action)

            # Convert action to joint_position set dict
            action_dict = action_list_2_action_dict(action)
            self.set_joint_positions(action_dict)

    def _is_done(self, observations):
        """Check if task is done."""

        # Check if gripper is within range of the goal
        d = self.goal_distance(observations["achieved_goal"], self.goal)

        # Print Debug info
        rospy.logdebug("=Task is done info=")
        if (d < self.distance_threshold).astype(np.float32):
            rospy.logdebug("Taks is done.")
        else:
            rospy.logdebug("Task is not done.")

        # Return result
        return (d < self.distance_threshold).astype(np.float32)

    def _compute_reward(self, observations, done):
        """Compute reward.

        Parameters
        ----------
        observations : [type]
            [description]
        done : function
            [description]

        Returns
        -------
        np.float32
            Reward that is received by the agent.
        """

        # Calculate the rewards based on the distance from the goal
        d = self.goal_distance(observations["achieved_goal"], self.goal)
        if self.reward_type == "sparse":

            # Print Debug info
            rospy.logdebug("=Reward info=")
            rospy.logdebug("Reward type: Non sparse")
            rospy.logdebug("Goal: %s", self.goal)
            rospy.logdebug("Achieved goal: %s", observations["achieved_goal"])
            rospy.logdebug("Perpendicular distance: %s", d)
            rospy.logdebug("Threshold: %s", self.distance_threshold)
            rospy.logdebug(
                "Received reward: %s", -(d > self.distance_threshold).astype(np.float32)
            )

            # Return result
            return -(d > self.distance_threshold).astype(np.float32)
        else:

            # Print Debug info
            rospy.logdebug("=Reward info=")
            rospy.logdebug("Reward type: Sparse")
            rospy.logdebug("Goal: %s", self.goal)
            rospy.logdebug("Achieved goal: %s", observations["achieved_goal"])
            rospy.logdebug("Perpendicular distance: %s", d)
            rospy.logdebug("Threshold: %s", self.distance_threshold)
            rospy.logdebug("Received reward: %s", -d)

            # Return result
            return -d

    def _env_setup(self, init_qpose=None):
        """Sets up initial configuration of the environment.
        Can be used to configure initial state and extract information
        from the simulation.

        Parameters
        ----------
        init_qpose : dict, optional
            Dictionary containing the initial joint positions for the Panda joints,
            by default :attr:`self.init_qpose`. Example: {"panda_link0": 1.5}
        """

        # IMPROVE: Make sure that we can choice to set EE pose or Joint pose
        # Check if init_qpose was supplied
        if init_qpose is None:
            init_qpose = self.init_qpose

        # Set initial joint positions
        rospy.loginfo("Init joint pose:")
        rospy.loginfo(init_qpose)
        self._set_init_pose(init_qpose)

        # Move end effector into position.
        gripper_target = np.array(
            [
                0.4632999742667715,
                0.29213747997030426,
                0.35038435384729955 + self.gripper_extra_height,
            ]
        )  # + self.sim.data.get_site_xpos('robot0:grip')
        gripper_rotation = np.array(
            [
                0.7813464975093422,
                0.6240963025419627,
                -0.0012058302957410503,
                4.42338382302794e-05,
            ]
        )
        action = np.concatenate([gripper_target, gripper_rotation])
        self.set_ee_pose(action)

        # Retrieve EE pose for sampling goals
        try:
            gripper_pose = self.get_ee_pose()
        except EePoseLookupError:
            rospy.logerr(
                "Shutting down '%s' since EE pose which is needed for sampling the "
                "goals could not be retrieved." % (rospy.get_name())
            )
            sys.exit(0)

        # Extract gripper position from pose
        gripper_pos = np.array(
            [
                gripper_pose.pose.position.x,
                gripper_pose.pose.position.y,
                gripper_pose.pose.position.z,
            ]
        )
        self.initial_gripper_pos = gripper_pos.copy()
        if self.has_object:  # Calculate gripper offset if object is present
            self.height_offset = self.model_states["object0"]["pose"].position.y

        # Sample a reaching goal
        self.goal = self._sample_goal()
        self._get_obs()

    #############################################
    # Task env helper methods ###################
    #############################################
    def _get_params(self, param_name):
        """Retrieve parameters from the parameter server or
        display an error message if they do not exist.

        Parameters
        ----------
        param_name : str
            Parameter name.
        """

        # Retrieve parameter from parameter server
        try:
            return rospy.get_param(param_name)
        except KeyError:
            rospy.logwarn(
                "Parameter '%s' was not found on the ROS parameter server. "
                "Please make sure this parameter is present in the "
                "<ALGO>config.yaml and that this yaml file is loaded." % param_name
            )

            # Shut down python script
            rospy.logerr(
                "Shutting down '%s' since parameter '%s' was missing on "
                "the ROS parameter server." % (rospy.get_name(), param_name)
            )
            sys.exit(0)

    def get_elapsed_time(self):
        """Returns the elapsed time since the last time this function was called.
        """
        current_time = rospy.get_time()
        dt = self._sim_time - current_time
        self._sim_time = current_time
        return dt

    def clip_goal_position(self, goal_pose):
        # TODO: IMPROVE DOCSTRING
        """Function used to limit the possible goal positions to a certian region.
        """

        # Clip goal
        goal_pose[0] = np.clip(goal_pose[0], GOAL_XMIN, GOAL_XMAX)
        goal_pose[1] = np.clip(goal_pose[1], GOAL_YMIN, GOAL_YMAX)
        goal_pose[2] = np.clip(goal_pose[2], GOAL_ZMIN, GOAL_ZMAX)

        # Return goal
        return goal_pose
