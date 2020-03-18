#! /usr/bin/env python
"""Panda RobotGazeboGoal environment
This class is responsible for creating the link between the gazebo
simulator and the openAI package. It is different from the RobotGazebo env
as here the gym.GoalEnv is used instead of the gym.Env. This is done
since the goal of the robot task now is changing with each episode.
"""

# Main python imports
import gym
from gym.utils import seeding

# ROS python imports
import rospy
from openai_ros.gazebo_connection import GazeboConnection
from openai_ros.controllers_connection import ControllersConnection

# ROS msgs and srvs
# NOTE: Found at https://bitbucket.org/theconstructcore/theconstruct_msgs/src/master/msg/RLExperimentInfo.msg
from theconstruct_msgs.msg import RLExperimentInfo


#################################################
# Panda Robot Gazebo Environment Class ##########
#################################################
class RobotGazeboGoalEnv(gym.GoalEnv):
    def __init__(self, robot_name_space, controllers_list, reset_controls):
        """Initializes a new Panda Robot Gazebo Goal environment.

        Parameters
        ----------
        robot_name_space : str
            Namespace of the robot.
        controllers_list : np.array
            Names of the controllers of the robot.
        reset_controls : bool
            Boolean specifying whether to reset the controllers when the simulation
            is reset.
        """

        # To reset Simulations
        rospy.loginfo("Initializing Panda RobotGazeboGoal environment.")
        self.gazebo = GazeboConnection(
            start_init_physics_parameters=False, reset_world_or_sim="WORLD"
        )
        self.controllers_object = ControllersConnection(
            namespace=robot_name_space, controllers_list=controllers_list
        )
        self.reset_controls = reset_controls
        rospy.loginfo(self.reset_controls)
        self.seed()

        # Set up ROS related variables
        self.episode_num = 0
        self.reward_pub = rospy.Publisher(
            "/openai/reward", RLExperimentInfo, queue_size=10
        )

        # Environment initiation complete message
        rospy.loginfo("Panda RobotGazeboGoal environment initialized.")

    #############################################
    # Panda Robot env main methods ##############
    #############################################
    def seed(self, seed=None):
        """Create gym random seed.

        Parameters
        ----------
        seed : int, optional
            Random seed, by default None (seeds from an operating system
            specific randomness source).

        Returns
        -------
        list
            List containing an opengym random seed (randomstate, seed).
        """
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        """Function executed each time step.
        Here we get the action execute it in a time step and retrieve the
        observations generated by that action. Here a action num is converted
        to a movement action, executed in the simulation and after which
        the observations result of performing that action is returned.

        Parameters
        ----------
        action :
            The the robot has to perform.

        Returns
        -------
            obs, reward, done, info :
                The step observation, setp reward, whether the task is done and some
                additional debug info.
        """

        # Execute action in the simulation
        rospy.loginfo("Taking step.")
        rospy.logdebug("Unpause sim.")
        self.gazebo.unpauseSim()
        rospy.logdebug("Set action.")
        rospy.logdebug("Action: %s" % action)
        self._set_action(action)

        # Retrieve observation
        rospy.loginfo("Get observation.")
        obs = self._get_obs()
        rospy.logdebug("Check if episode is done.")
        done = self._is_done(obs)
        info = {}
        reward = self._compute_reward(obs, done)

        # Publish reward and return action result
        rospy.logdebug("Publishing reward.")
        self._publish_reward_topic(reward, self.episode_num)
        return obs, reward, done, info

    def reset(self):
        """Function used for resetting the simulation.

        Returns
        -------
            list :
                An observation of the initial state.
        """
        rospy.loginfo("Resetting Panda RobotGazeboGoal environment.")
        self._reset_sim()
        self._init_env_variables()
        self._update_episode()
        obs = self._get_obs()
        return obs

    def close(self):
        """Function executed when closing the environment.
        Use it for closing GUIS and other systems that need closing.
        """
        rospy.logdebug("Closing Panda RobotGazeboGoal environment.")
        rospy.signal_shutdown("Closing Panda RobotGazeboGoal environment.")

    #############################################
    # Setup extension methods methods ###########
    #############################################
    # NOTE: These methods can be overloaded by robot or task env)
    def _reset_sim(self):
        """Resets a simulation.

        Returns
        -------
        bool
            Boolean specifying whether reset was successful.
        """

        # Reset simulation (and controls)
        if self.reset_controls:
            self.gazebo.unpauseSim()
            self.controllers_object.reset_controllers()
            self._check_all_systems_ready()
            self._set_init_pose()
            self.gazebo.pauseSim()
            self.gazebo.resetSim()
            self.gazebo.unpauseSim()
            self.controllers_object.reset_controllers()
            self._check_all_systems_ready()
            self.gazebo.pauseSim()

        else:
            self.gazebo.unpauseSim()
            self._check_all_systems_ready()
            self._set_init_pose()
            self.gazebo.resetWorld()
            self._check_all_systems_ready()

        # Return result bool
        return True

    #############################################
    # Panda GAzebo env helper methods ###########
    #############################################
    def _update_episode(self):
        """Increases the episode number by one.
        """
        self.episode_num += 1

    def _publish_reward_topic(self, reward, episode_number=1):
        """This function publishes the given reward in the reward topic for
        easy access from ROS infrastructure.


        Parameters
        ----------
        reward : np.float32
            The episode reward.
        episode_number : int, optional
            The episode number, by default 1.
        """
        reward_msg = RLExperimentInfo()
        reward_msg.episode_number = episode_number
        reward_msg.episode_reward = reward
        self.reward_pub.publish(reward_msg)

    #############################################
    # Setup virtual methods #####################
    #############################################
    # NOTE: These virtual methods need to be overloaded by the Robot and Task env
    def _check_all_systems_ready(self):
        """Checks that all the sensors, publishers and other simulation systems are
        operational.

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()

    def _set_init_pose(self):
        """Sets the Robot in its init pose.

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()

    def _get_obs(self):
        """Returns the observation.

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.

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

    def _is_done(self, observations):
        """Indicates whether or not the episode is done ( the robot has fallen for example).

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

    def _env_setup(self, initial_qpos):
        """Initial configuration of the environment. Can be used to configure initial state
        and extract information from the simulation.

        Raises
        ------
        NotImplementedError
        """
        raise NotImplementedError()
