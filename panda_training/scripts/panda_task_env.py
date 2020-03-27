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
from gym.envs.robotics import rotations
import numpy as np
import sys
import panda_robot_env
from functions import flatten_list

# ROS python imports
import rospy

# Register openai gym environment
register(
    id="PandaReach-v1",
    entry_point="panda_task_env:PandaReachTaskEnv",
    max_episode_steps=1000,
)


#################################################
# Panda Robot Environment Class #################
#################################################
class PandaReachTaskEnv(panda_robot_env.PandaRobotEnv, utils.EzPickle):
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
                    "Waiting for the Panda gazebo robot simulation to be " "started."
                )
        else:
            rospy.logerr(
                "Shutting down '%s' since the Panda gazebo simulation was not "
                "started within the set timeout period of %s seconds."
                % (rospy.get_name(), wait_for_sim_timeout)
            )
            sys.exit(0)

        # Retrieve parameters from parameter server
        rospy.loginfo("Initializing Panda task environment.")
        self.get_params()

        # Initialize parent Class to setup the Robot environment)
        panda_robot_env.PandaRobotEnv.__init__(
            self,
            robot_EE_link=robot_EE_link,
            robot_arm_control_type=robot_arm_control_type.lower(),
            robot_hand_control_type=robot_hand_control_type.lower(),
        )
        utils.EzPickle.__init__(self)

        # Setup task environment
        rospy.logdebug("Setup initial environment state.")
        self._env_setup(initial_qpos=self.init_pos)

        # Get observations
        rospy.logdebug("Get initial observation.")
        obs = self._get_obs()

        # Set gym action and observation space
        rospy.logdebug("Setup gym action and observation space.")
        # TODO: Change action space based on control_type
        # TODO: Clip action space
        self.action_space = spaces.Box(
            -1.0, 1.0, shape=(self.n_actions,), dtype="float32"
        )
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
        self.n_actions = self._get_params("/reach_sim/n_actions")
        self.has_object = rospy.get_param("/reach_sim/has_object")
        self.block_gripper = rospy.get_param("/reach_sim/block_gripper")
        self.n_substeps = rospy.get_param("/reach_sim/n_substeps")
        self.gripper_extra_height = rospy.get_param("/reach_sim/gripper_extra_height")
        self.target_in_the_air = rospy.get_param("/reach_sim/target_in_the_air")
        self.target_offset = rospy.get_param("/reach_sim/target_offset")
        self.obj_range = rospy.get_param("/reach_sim/obj_range")
        self.target_range = rospy.get_param("/reach_sim/target_range")
        self.distance_threshold = rospy.get_param("/reach_sim/distance_threshold")
        self.reward_type = rospy.get_param("/reach_sim/reward_type")
        self.init_pos = rospy.get_param("/reach_sim/init_pos")

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
            goal = self.initial_gripper_xpos[:3] + self.np_random.uniform(
                -self.target_range, self.target_range, size=3
            )
            goal += self.target_offset
            goal[2] = self.height_offset
            if self.target_in_the_air and self.np_random.uniform() < 0.5:
                goal[2] += self.np_random.uniform(0, 0.45)
        else:
            goal = self.initial_gripper_xpos[:3] + self.np_random.uniform(
                -0.15, 0.15, size=3
            )

        # return goal.copy()
        return goal

    def _sample_achieved_goal(self, grip_pos_array, object_pos):
        """Retrieve current position. If gripper has object
        return object position.

        Returns
        -------
        geometry_msgs.PoseStamped
            The achieved pose.
        """

        # Retrieve gripper end pose
        if not self.has_object:  # Environment has no object
            achieved_goal = grip_pos_array.copy()
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
    def _set_init_pose(self, initial_qpos=None):
        """Sets the Robot in its init pose.

        Parameters
        ----------
        initial_qpos : dict, optional
            Dictionary containing the initial joint positions for the Panda joints,
            by default :attr:`self.init_pose`. Example: {"panda_link0": 1.5}
        Returns
        -------
        Boolean
            Boolean specifying whether reset was successful.
        """

        # Check if initial_qpos was supplied
        if initial_qpos is None:
            initial_qpos = self.init_pose

        # Set initial pose
        self.gazebo.unpauseSim()
        self.set_joint_positions(initial_qpos)
        return True

    def _get_obs(self):
        """Get robot state observation.

        Returns
        -------
        dict
            A dictionary containing the {observation, achieved_goal, desired_goal):

            observation (22x1):
                - End effector x pos
                - End effector y pos
                - End effector z pos
                - Object x pos
                - Object y pos
                - Object z pos
                - Object/gripper rel. x pos
                - Object/gripper rel. y pos
                - Object/gripper rel. z pos
                - Gripper finger 1 x pos
                - Gripper finger 2 x pos
                - Object pitch (x)
                - Object yaw (y)
                - Object roll (z)
                - Object x vel
                - Object y vel
                - Object z vel
                - Object x angular vel
                - Object y angular vel
                - Object z angular vel
                - Gripper finger 1 x vel
                - Gripper finger 2 x vel
        """

        # Retrieve robot end effector pose and orientation
        grip_pos = self.get_ee_pose()
        grip_pos_array = np.array(
            [
                grip_pos.pose.position.x,
                grip_pos.pose.position.y,
                grip_pos.pose.position.z,
            ]
        )
        grip_rpy = self.get_ee_rpy()  # Retrieve as euler angles

        # Retrieve robot joint pose and velocity
        grip_velp = np.array([grip_rpy.y, grip_rpy.y])
        robot_qpos, robot_qvel = self.robot_get_obs(self.joints)

        # Get gripper pose and (angular)velocity
        gripper_state = robot_qpos[0:2]
        gripper_vel = robot_qvel[0:2]

        # Get object pose and (angular)velocity
        if self.has_object:
            object_pos = self.sim.data.get_site_xpos("object0")
            # rotations
            object_rot = rotations.mat2euler(self.sim.data.get_site_xmat("object0"))
            # velocities
            object_velp = self.sim.data.get_site_xvelp("object0") * dt
            object_velr = self.sim.data.get_site_xvelr("object0") * dt
            # gripper state
            object_rel_pos = object_pos - grip_pos
            object_velp -= grip_velp
        else:
            object_pos = (
                object_rot
            ) = object_velp = object_velr = object_rel_pos = np.zeros(0)

        # Get achieved goal
        achieved_goal = self._sample_achieved_goal(grip_pos_array, object_pos)

        # Concatenate observations
        obs = np.concatenate(
            [
                grip_pos_array,
                object_pos.ravel(),
                object_rel_pos.ravel(),
                gripper_state,
                object_rot.ravel(),
                object_velp.ravel(),
                object_velr.ravel(),
                gripper_vel,
            ]
        )

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
        """Take robot action.
        """

        # Take action
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

        # DEBUG:
        rospy.logdebug("=Action set info=")
        rospy.logdebug("Action that is set:")
        rospy.logdebug(action)
        # DEBUG:

        # Apply action to simulation.
        self.set_ee_pose(action)
        # self.set_arm_joints_positions(action)

    def _is_done(self, observations):
        """Check if task is done."""

        # Check if gripper is within range of the goal
        d = self.goal_distance(observations["achieved_goal"], self.goal)

        # DEBUG:
        rospy.logdebug("=Task is done info=")
        if (d < self.distance_threshold).astype(np.float32):
            rospy.logdebug("Taks is done.")
        else:
            rospy.logdebug("Task is not done.")
        # DEBUG:
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

            # DEBUG
            rospy.logdebug("=Reward info=")
            rospy.logdebug("Reward type: Non sparse")
            rospy.logdebug("Goal: %s", self.goal)
            rospy.logdebug("Achieved goal: %s", observations["achieved_goal"])
            rospy.logdebug("Perpendicular distance: %s", d)
            rospy.logdebug("Threshold: %s", self.distance_threshold)
            rospy.logdebug(
                "Received reward: %s", -(d > self.distance_threshold).astype(np.float32)
            )
            # DEBUG

            return -(d > self.distance_threshold).astype(np.float32)
        else:

            # DEBUG
            rospy.logdebug("=Reward info=")
            rospy.logdebug("Reward type: Sparse")
            rospy.logdebug("Goal: %s", self.goal)
            rospy.logdebug("Achieved goal: %s", observations["achieved_goal"])
            rospy.logdebug("Perpendicular distance: %s", d)
            rospy.logdebug("Threshold: %s", self.distance_threshold)
            rospy.logdebug("Received reward: %s", -d)
            # DEBUG

            return -d

    def _env_setup(self, initial_qpos=None):
        """Sets up initial configuration of the environment.
        Can be used to configure initial state and extract information
        from the simulation.

        Parameters
        ----------
        initial_qpos : dict, optional
            Dictionary containing the initial joint positions for the Panda joints,
            by default :attr:`self.init_pose`. Example: {"panda_link0": 1.5}
        """

        # Check if initial_qpos was supplied
        if initial_qpos is None:
            initial_qpos = self.init_pose

        # Set initial joint positions
        rospy.loginfo("Init Pos:")
        rospy.loginfo(initial_qpos)
        self._set_init_pose(initial_qpos)

        # Move end effector into position.
        gripper_target = np.array(
            [0.498, 0.005, 0.431 + self.gripper_extra_height]
        )  # + self.sim.data.get_site_xpos('robot0:grip')
        gripper_rotation = np.array([1.0, 0.0, 1.0, 0.0])
        action = np.concatenate([gripper_target, gripper_rotation])
        self.set_ee_pose(action)

        # Extract information for sampling goals
        gripper_pos = self.get_ee_pose()
        gripper_pose_array = np.array(
            [
                gripper_pos.pose.position.x,
                gripper_pos.pose.position.y,
                gripper_pos.pose.position.z,
            ]
        )
        self.initial_gripper_xpos = gripper_pose_array.copy()
        if self.has_object:  # Calculate gripper offset if object is present
            self.height_offset = self.sim.data.get_site_xpos("object0")[
                2
            ]  # TODO: Change to object we want to grasp

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
            rospy.loginfo(
                "Shutting down simulation since parameter %s was missing on "
                "the ROS parameter server." % param_name
            )
            sys.exit(0)
