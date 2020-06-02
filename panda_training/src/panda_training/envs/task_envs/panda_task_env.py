"""This Task Environment will be in charge of providing all the necessary
functions and methods related to this specific training.
"""

# Main python imports
from gym import utils
from gym import spaces
from gym.envs.registration import register
import numpy as np
import sys
import os
from panda_training.envs.robot_envs import PandaRobotEnv
from panda_training.exceptions import (
    EePoseLookupError,
    RandomJointPositionsError,
    RandomEePoseError,
    SpawnModelError,
    SetModelStateError,
)
from panda_training.errors import (
    arg_type_error,
    arg_keys_error,
    arg_value_error,
)
from panda_training.functions import (
    flatten_list,
    get_orientation_euler,
    lower_first_char,
    has_invalid_type,
    contains_keys,
    has_invalid_value,
    pose_dict_2_pose_msg,
    pose_msg_2_pose_dict,
    split_bounds_dict,
    split_pose_dict,
    translate_gripper_width_2_finger_joint_commands,
    log_pose_dict,
    model_state_msg_2_link_state_dict,
)
import yaml
from collections import OrderedDict

# ROS python imports
import rospy
from panda_training.extras import TargetMarker, SampleRegionMarker
from rospy.exceptions import ROSException, ROSInterruptException

# ROS msgs and srvs
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, PointStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from panda_training.msg import FollowJointTrajectoryGoal, BoundingRegion, JointLimits
from panda_training.srv import (
    GetRandomEePose,
    GetRandomEePoseRequest,
    GetRandomJointPositions,
    GetRandomJointPositionsRequest,
    GetControlledJoints,
    GetControlledJointsRequest,
)

# Register Openai gym environment
register(
    id="PandaReach-v0",
    entry_point="panda_training.envs.task_envs:PandaTaskEnv",
    max_episode_steps=1000,
)

# TODO: Check control switcher
# TODO: Add attributes to docstring
# TODO: Change action bounds to an array
# TODO: Adjust n_actions to control type if n_actions was not supplied
# TODO: Nakijken gripper_extra_height
# TODO: Add gripper width ensure prestep
# TODO: Add iterations_done such that impossible positions are eventually stoped see openai_sim
# TODO: Check why finger joints can not be controlled

# Script Parameters
MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC = (
    "panda_moveit_planner_server/get_random_joint_positions"
)
MOVEIT_GET_RANDOM_EE_POSE_TOPIC = "panda_moveit_planner_server/get_random_ee_pose"
GET_CONTROLLED_JOINTS_TOPIC = "panda_control_server/get_controlled_joints"
DIRNAME = os.path.dirname(__file__)
DEFAULT_CONFIG_PATH = os.path.abspath(
    os.path.join(DIRNAME, "../../../../cfg/env_config.yaml")
)
PANDA_JOINTS = {
    "arm": [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ],
    "hand": ["panda_finger_joint1", "panda_finger_joint2"],
    "both": [
        "panda_finger_joint1",
        "panda_finger_joint2",
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ],
}
POSITION_CONTROL_TYPES = [
    "joint_position_control",
    "joint_group_position_control",
]
EFFORT_CONTROL_TYPES = [
    "joint_effort_control",
    "joint_group_effort_control",
]
# TODO: CLEANUP both present here and in control environment
REWARD_TYPES = ["sparse", "dense"]
ROBOT_CONTROL_TYPES = {
    "arm": [
        "joint_trajectory_control",
        "joint_position_control",
        "joint_effort_control",
        "joint_group_position_control",
        "joint_group_effort_control",
        "ee_control",
    ],
    "hand": [
        "joint_trajectory_control",
        "joint_position_control",
        "joint_effort_control",
        "joint_group_position_control",
        "joint_group_effort_control",
    ],
}
GOAL_SAMPLING_STRATEGIES = ["global", "local"]
GOAL_SAMPLING_REFERENCES = ["initial", "current"]
INIT_POSE_TYPES = ["ee_pose", "qpose"]
GRASP_OBJECT_NAME = "grasp_object_0"


#################################################
# Panda Robot Environment Class #################
#################################################
class PandaTaskEnv(PandaRobotEnv, utils.EzPickle):
    """Class that provides all the methods used for the algorithm training.

    Attributes
    ----------
    action_space : gym.spaces.box.Box
        Gym action space object.
    observation_space : gym.spaces.dict.Dict
        Gym observation space object.

    Methods
    ----------
    get_config():
        Retrieve default values from the panda task environment configuration file.
    goal_distance(goal_a, goal_b):
        Calculates the perpendicular distance to the goal.
    robot_get_obs(data):
        Returns all joint positions and velocities associated with a robot.
    """

    def __init__(
        self,
        gripper_extra_height=None,  # FIXME: NOt implemented
        block_gripper=None,
        has_object=None,
        obj_target_in_the_air=None,
        target_offset=None,
        target_bounds=None,
        distance_threshold=None,
        init_pose=None,
        init_pose_bounds=None,
        init_obj_pose=None,
        obj_bounds=None,
        reward_type=None,
        robot_arm_control_type=None,
        robot_hand_control_type=None,
        n_actions=None,
        controlled_joints=None,
        use_gripper_width=None,
    ):
        # TODO: UPDATE DOCSTRING
        # TODO: Add contsructor elements
        """Initializes a Panda Reach task environment.


        Parameters
        ----------
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
        n_actions : int, optional
            The size of the action space, by default the value given in the
            <ALGO>config.yaml is used. If the n_actions parameter is not found in this
            config the maximum possible action space is used.
        """

        # Log message
        # FIXME: ACtion calculation
        # TODO: Calculate action space based on controlled joints
        rospy.loginfo("Initializing Panda task environment.")

        # Load task environment configuration settings as class attributes
        self.get_config()

        # Process constructor input arguments
        # Overload default settings if constructor arguments are supplied
        if gripper_extra_height:
            self.gripper_extra_height = gripper_extra_height
        if block_gripper:
            self.block_gripper = block_gripper
        if has_object:
            self.has_object = has_object
        if obj_target_in_the_air:
            self.obj_target_in_the_air = obj_target_in_the_air
        if target_offset:

            # If list convert to dictionary
            if isinstance(target_offset, list):
                self.target_offset = {
                    "x": target_offset[0],
                    "y": target_offset[1],
                    "z": target_offset[2],
                }
        if target_bounds:
            self._target_bounds[self.target_sampling_strategy] = target_bounds
        if distance_threshold:
            self.distance_threshold = distance_threshold
        if init_pose:
            self.init_pose = init_pose
        if init_pose_bounds:
            self.init_pose_bounds = init_pose_bounds
        if init_obj_pose:
            self.init_obj_pose = init_obj_pose
        if obj_bounds:
            self.obj_bounds = obj_bounds
        if reward_type:
            self.reward_type = reward_type.lower()
        if robot_arm_control_type:
            self.robot_arm_control_type = robot_arm_control_type.lower()
        if robot_hand_control_type:
            self.robot_hand_control_type = robot_hand_control_type.lower()
        if n_actions:
            self.n_actions = n_actions
        if controlled_joints:
            self.action_space_joints = controlled_joints
        else:
            self.action_space_joints = []  # Initialize as list
        if use_gripper_width:
            self.use_gripper_width = use_gripper_width

        # Validate input arguments
        self._validate_input_args()

        # Convert init pose and init pose bounds dictionaries into the right format
        self.init_obj_pose = pose_dict_2_pose_msg(self.init_obj_pose)
        self.init_pose = translate_gripper_width_2_finger_joint_commands(self.init_pose)
        if hasattr(self, "init_pose_bounds"):
            self.init_pose_bounds = translate_gripper_width_2_finger_joint_commands(
                self.init_pose_bounds
            )

        # Create other class attributes
        self._sim_time = rospy.get_time()
        self._prev_grip_pos = np.zeros(3)
        self._prev_object_pos = np.zeros(3)
        self._prev_object_rot = np.zeros(3)
        self._services_connection_status = {}

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

        # Initialize parent Class to setup the Robot environment
        super(PandaTaskEnv, self).__init__(
            robot_EE_link=self._ee_link,
            robot_arm_control_type=self.robot_arm_control_type,
            robot_hand_control_type=self.robot_hand_control_type,
        )
        utils.EzPickle.__init__(self)

        #########################################
        # Connect to required services, #########
        # subscribers and publishers. ###########
        #########################################

        # Connect to Moveit 'get_random_joint_positions' service
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC
            )
            rospy.wait_for_service(
                MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC,
                timeout=self._panda_moveit_server_connection_timeout,
            )
            self._moveit_get_random_joint_positions_client = rospy.ServiceProxy(
                MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC, GetRandomJointPositions
            )
            rospy.logdebug(
                "Connected to '%s' service!" % MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC
            )
            self._services_connection_status[
                MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC
            ] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!"
                % MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC
            )
            self._services_connection_status[
                MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC
            ] = False

        # Connect to Moveit 'get_random_ee_pose' service
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % MOVEIT_GET_RANDOM_EE_POSE_TOPIC
            )
            rospy.wait_for_service(
                MOVEIT_GET_RANDOM_EE_POSE_TOPIC,
                timeout=self._panda_moveit_server_connection_timeout,
            )
            self._moveit_get_random_ee_pose_client = rospy.ServiceProxy(
                MOVEIT_GET_RANDOM_EE_POSE_TOPIC, GetRandomEePose
            )
            rospy.logdebug(
                "Connected to '%s' service!" % MOVEIT_GET_RANDOM_EE_POSE_TOPIC
            )
            self._services_connection_status[MOVEIT_GET_RANDOM_EE_POSE_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % MOVEIT_GET_RANDOM_EE_POSE_TOPIC
            )
            self._services_connection_status[MOVEIT_GET_RANDOM_EE_POSE_TOPIC] = False

        # Connect to Panda control server 'get_controlled_joints' service
        try:
            rospy.logdebug("Connecting to '%s' service." % GET_CONTROLLED_JOINTS_TOPIC)
            rospy.wait_for_service(
                GET_CONTROLLED_JOINTS_TOPIC,
                timeout=self._panda_moveit_server_connection_timeout,
            )
            self._moveit_get_controlled_joints_client = rospy.ServiceProxy(
                GET_CONTROLLED_JOINTS_TOPIC, GetControlledJoints
            )
            rospy.logdebug("Connected to '%s' service!" % GET_CONTROLLED_JOINTS_TOPIC)
            self._services_connection_status[GET_CONTROLLED_JOINTS_TOPIC] = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % GET_CONTROLLED_JOINTS_TOPIC
            )
            self._services_connection_status[GET_CONTROLLED_JOINTS_TOPIC] = False

        # Create current target publisher
        rospy.logdebug("Creating target pose publisher.")
        self._target_pose_pub = rospy.Publisher(
            "panda_training/current_target", Marker, queue_size=10
        )
        rospy.logdebug("Goal target publisher created.")

        # Create target bounding region publisher
        rospy.logdebug("Creating target bounding region publisher.")
        self._target_sample_region_pub = rospy.Publisher(
            "panda_training/target_sample_region", Marker, queue_size=10
        )
        rospy.logdebug("Target bounding region publisher created.")

        # Create initial pose bounding region publisher
        rospy.logdebug("Creating initial pose sample region publisher.")
        self._init_pose_sample_region_pub = rospy.Publisher(
            "panda_training/init_pose_sample_region", Marker, queue_size=10
        )
        rospy.logdebug("Initial pose sample region publisher created.")

        # Create initial pose bounding region publisher
        rospy.logdebug("Creating initial object pose bounding region publisher.")
        self._obj_pose_sample_region_pub = rospy.Publisher(
            "panda_training/obj_pose_sample_region", Marker, queue_size=10
        )
        rospy.logdebug("Initial object pose bounding region publisher created.")

        #########################################
        # Initialize rviz and Gazebo envs #######
        #########################################

        # Initiate task environment
        rospy.logdebug("Setup initial environment state.")
        self._env_setup()

        # Get observations
        rospy.logdebug("Get initial observation.")
        obs = self._get_obs()

        # Display goal target sample region in rviz
        if self._visualize_target_bounds:

            # Create goal sampling region marker
            goal_sample_region_marker_msg = SampleRegionMarker(
                x_min=self._target_bounds["global"]["x_min"],
                y_min=self._target_bounds["global"]["y_min"],
                z_min=self._target_bounds["global"]["z_min"],
                x_max=self._target_bounds["global"]["x_max"],
                y_max=self._target_bounds["global"]["y_max"],
                z_max=self._target_bounds["global"]["z_max"],
            )

            # Publish goal sample region marker for rviz visualization
            self._target_sample_region_pub.publish(goal_sample_region_marker_msg)

        # Display object sampling region in rviz
        if self._visualize_obj_bounds:

            # Create goal sampling region marker
            obj_sample_region_marker_msg = SampleRegionMarker(
                x_min=self.obj_bounds["x_min"],
                y_min=self.obj_bounds["y_min"],
                z_min=self.init_obj_pose.position.z,
                x_max=self.obj_bounds["x_max"],
                y_max=self.obj_bounds["y_max"],
                z_max=self.init_obj_pose.position.z + 10 ** -19,
            )
            obj_region_color = ColorRGBA()
            obj_region_color.a = 0.15
            obj_region_color.r = 0.0
            obj_region_color.g = 1.0
            obj_region_color.b = 0.0
            obj_sample_region_marker_msg.color = obj_region_color

            # Publish goal sample region marker for rviz visualization
            self._obj_pose_sample_region_pub.publish(obj_sample_region_marker_msg)

        # Display object sampling region in rviz
        if self._visualize_init_pose_bounds and hasattr(self, "init_pose_bounds"):

            # Create goal sampling region marker
            init_pose_sample_region_marker_msg = SampleRegionMarker(
                x_min=self.init_pose_bounds["x_min"],
                y_min=self.init_pose_bounds["y_min"],
                z_min=self.init_pose_bounds["z_min"],
                x_max=self.init_pose_bounds["x_max"],
                y_max=self.init_pose_bounds["y_max"],
                z_max=self.init_pose_bounds["z_max"],
            )
            init_pose_region_color = ColorRGBA()
            init_pose_region_color.a = 0.15
            init_pose_region_color.r = 0.0
            init_pose_region_color.g = 0.0
            init_pose_region_color.b = 1.0
            init_pose_sample_region_marker_msg.color = init_pose_region_color

            # Publish goal sample region marker for rviz visualization
            self._init_pose_sample_region_pub.publish(
                init_pose_sample_region_marker_msg
            )

        #########################################
        # Create action and observation space ###
        #########################################
        rospy.logdebug("Setup gym action and observation space.")

        # Create action space
        self.action_space = self._create_action_space()

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

        #########################################
        # Validate constructor arguments ########
        #########################################

        # Validate controlled_joints argument and create if not supplied
        if self.action_space_joints:  # If exists
            self._validate_action_space_joints(self.action_space_joints)
        else:
            self.action_space_joints = self._get_action_space_joints()

        # Environment initiation complete message
        rospy.loginfo("Panda Panda task environment initialized.")

    #############################################
    # Panda Robot env main methods ##############
    #############################################
    def get_config(self):
        """Retrieve default values from the defaults configuration file.
        """

        # Try to load default values from default configuration file
        try:
            with open(DEFAULT_CONFIG_PATH, "r") as stream:
                try:
                    config = yaml.safe_load(stream)
                except yaml.YAMLError as e:
                    rospy.logwarn(
                        "Shutting down '%s' as the task environment configuration "
                        "values could not be loaded from the configuration file '%s' "
                        "as the following error was thrown: %s"
                        % (rospy.get_name(), DEFAULT_CONFIG_PATH, e)
                    )
                    sys.exit(0)
        except FileNotFoundError:
            rospy.logwarn(
                "Shutting down '%s' as the task environment configuration values could "
                "not be loaded since the configuration file '%s' was not found. Please "
                "make sure the configuration file is present."
                % (rospy.get_name(), DEFAULT_CONFIG_PATH)
            )
            sys.exit(0)

        # configuration values that can be overloaded using the constructor
        self.gripper_extra_height = config["simulation"]["control"][
            "gripper_extra_height"
        ]
        self.block_gripper = config["simulation"]["control"]["block_gripper"]
        self.has_object = config["training"]["has_object"]
        self.obj_target_in_the_air = config["training"]["target_sampling"][
            "obj_target_in_the_air"
        ]
        self.target_offset = config["training"]["target_sampling"]["target_offset"]
        self.target_sampling_strategy = config["training"]["target_sampling"][
            "strategy"
        ].lower()
        self.target_bounds = config["training"]["target_sampling"]["bounds"][
            self.target_sampling_strategy
        ]
        self.distance_threshold = config["training"]["distance_threshold"]
        self.init_pose = config["simulation"]["init_pose_sampling"]["init_robot_pose"]
        self.init_obj_pose = config["training"]["object_sampling"]["init_obj_pose"]
        self.obj_bounds = config["training"]["object_sampling"]["bounds"]
        self.reward_type = config["training"]["reward_type"].lower()
        self.robot_arm_control_type = config["simulation"]["control"][
            "robot_arm_control_type"
        ].lower()
        self.robot_hand_control_type = config["simulation"]["control"][
            "robot_hand_control_type"
        ].lower()
        self.n_actions = config["action_space"]["n_actions"]
        self.use_gripper_width = config["action_space"]["use_gripper_width"]

        # Other configuration values
        self._obj_sampling_distance_threshold = config["training"]["object_sampling"][
            "distance_threshold"
        ]
        self._reset_robot_pose = config["simulation"]["reset_robot_pose"]
        self._random_init_pose = config["simulation"]["init_pose_sampling"][
            "random_init_pose"
        ]
        self._randomize_first_episode = config["simulation"]["init_pose_sampling"][
            "randomize_first_episode"
        ]
        self._target_bounds = config["training"]["target_sampling"]["bounds"]
        self._visualize_target = config["training"]["target_sampling"][
            "visualize_target"
        ]
        self._visualize_target_bounds = config["training"]["target_sampling"][
            "visualize_bounds"
        ]
        self._visualize_obj_bounds = config["training"]["object_sampling"][
            "visualize_bounds"
        ]
        self._visualize_init_pose_bounds = config["simulation"]["init_pose_sampling"][
            "visualize_bounds"
        ]
        self._ee_link = config["simulation"]["control"]["ee_link"]
        self._action_bound_low = config["action_space"]["bounds"]["low"]
        self._action_bound_high = config["action_space"]["bounds"]["high"]

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

        Raises
        ------
        EePoseLookupError
            Error thrown when error occurred while trying to retrieve the EE pose using
            the 'get_ee_pose' service.
        """

        # Sample goal from goal region based on the target_sampling_strategy
        if self.target_sampling_strategy == "global":

            # Sample goal within the global bounds
            goal = self.np_random.uniform(
                [
                    self._target_bounds["global"]["x_min"],
                    self._target_bounds["global"]["y_min"],
                    self._target_bounds["global"]["z_min"],
                ],
                [
                    self._target_bounds["global"]["x_max"],
                    self._target_bounds["global"]["y_max"],
                    self._target_bounds["global"]["z_max"],
                ],
                size=3,
            )
        elif self.target_sampling_strategy == "local":  # Rel to current EE pose

            # Retrieve current end effector pose
            try:
                cur_ee_pose = self.get_ee_pose()  # Get ee pose
                cur_ee_pos = np.array(
                    [
                        cur_ee_pose.pose.position.x,
                        cur_ee_pose.pose.position.y,
                        cur_ee_pose.pose.position.z,
                    ]
                )  # Retrieve position
            except EePoseLookupError:
                rospy.logerr(
                    "Shutting down '%s' since the current end effector pose "
                    "which is needed for sampling the goals could not be "
                    "retrieved." % (rospy.get_name())
                )
                sys.exit(0)

            # Sample goal relative to end effector pose
            goal = cur_ee_pos + self.np_random.uniform(
                [
                    self._target_bounds["local"]["x_min"],
                    self._target_bounds["local"]["y_min"],
                    self._target_bounds["local"]["z_min"],
                ],
                [
                    self._target_bounds["local"]["x_max"],
                    self._target_bounds["local"]["y_max"],
                    self._target_bounds["local"]["z_max"],
                ],
                size=3,
            )
        else:  # Thrown error if goal could not be sampled
            rospy.logerr(
                "Shutting down '%s' since no goal could be sampled as '%s' is not "
                "a valid goal sampling strategy. Options are 'global' and 'local'."
                % (rospy.get_name(), self.target_sampling_strategy)
            )
            sys.exit(0)

        # Apply offsets if the task environment has an object
        if self.has_object:  # Environment has object

            # Apply target offset (Required in the panda push task)
            goal += [
                self.target_offset["x"],
                self.target_offset["y"],
                self.target_offset["z"],
            ]
            goal[2] = self.object_height_offset

            # If object is in the air add an additional height offset
            if self.obj_target_in_the_air and self.np_random.uniform() < 0.5:
                goal[2] += self.np_random.uniform(0, 0.45)

        # Make sure the goal is always within the global goal sampling region
        goal = self._clip_goal_position(goal)

        # Visualize goal marker
        if self._visualize_target:

            # Generate Rviz marker
            goal_maker_pose = Pose()
            goal_maker_pose.position.x = goal[0]
            goal_maker_pose.position.y = goal[1]
            goal_maker_pose.position.z = goal[2]
            goal_marker_msg = TargetMarker(pose=goal_maker_pose)

            # Publish goal marker for rviz visualization
            self._target_pose_pub.publish(goal_marker_msg)

        # return goal.copy()
        return goal

    def _sample_achieved_goal(self, ee_pos, object_pos):
        """Retrieve currently achieved goal. If gripper has object return object
        position.

        Parameters
        ----------
        ee_pos : np.array
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
            achieved_goal = np.squeeze(ee_pos.copy())
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
    # Overload Robot/Gazebo env virtual methods #
    #############################################
    def _set_init_pose(self):
        """Sets the Robot in its init pose.

        Returns
        -------
        Boolean
            Boolean specifying whether reset was successful.
        """

        # Retrieve initial pose (Random or fixed)
        if (
            self._randomize_first_episode or self.episode_num != 0
        ) and self._random_init_pose:

            # Retrieve random ee pose
            rospy.logdebug("Retrieve random ee_pose.")
            try:

                # Retrieve random ee and gripper pose
                init_ee_pose = self._get_random_ee_pose()
                init_ee_pose = pose_msg_2_pose_dict(init_ee_pose)  # Conv. to pose dict
            except (RandomEePoseError) as e:
                # Change warn message
                logwarn_msg = (
                    "Random ee_pose could not be retrieved as %s. Initial pose used "
                    "instead." % lower_first_char(e.args[0])
                )
                rospy.logwarn(logwarn_msg)
                init_ee_pose, _ = split_pose_dict(self.init_pose)

            # Retrieve random gripper pose
            rospy.logdebug("Retrieve random gripper pose.")
            try:

                # Retrieve random gripper pose
                init_joint_pose = self._get_random_joint_positions()
                init_grip_joints_pose = {
                    key: val
                    for key, val in init_joint_pose.items()
                    if key
                    in ["panda_finger_joint1", "panda_finger_joint2", "gripper_width"]
                }
            except (RandomJointPositionsError) as e:

                # Change warn message
                logwarn_msg = (
                    "Random gripper pose could not be retrieved as %s. Initial pose "
                    "used instead." % lower_first_char(e.args[0])
                )
                rospy.logwarn(logwarn_msg)
                _, init_joint_pose = split_pose_dict(self.init_pose)
                init_grip_joints_pose = {
                    key: val
                    for key, val in init_joint_pose.items()
                    if key
                    in ["panda_finger_joint1", "panda_finger_joint2", "gripper_width"]
                }

        else:

            # Clip init pose within bounds if requested
            init_ee_pose, init_joint_pose = split_pose_dict(self.init_pose)
            if hasattr(self, "init_pose_bounds"):
                init_ee_pose = self._clip_init_pose(init_ee_pose)
                init_joint_pose = self._clip_init_pose(init_joint_pose)
            init_grip_joints_pose = {
                key: val
                for key, val in init_joint_pose.items()
                if key
                in ["panda_finger_joint1", "panda_finger_joint2", "gripper_width"]
            }

        # Put finger joints in the right position if use_gripper_width is True
        if self.use_gripper_width:
            if "gripper_width" not in init_grip_joints_pose.keys():
                rospy.logwarn(
                    "The joint position of 'panda_finger_joint2' was set to be "
                    "be equal to the value of 'panda_finger_joint1' as the "
                    "'use_gripper_width' variable is set to True."
                )
                init_grip_joints_pose["panda_finger_joint2"] = init_grip_joints_pose[
                    "panda_finger_joint1"
                ]
            else:
                init_grip_joints_pose = translate_gripper_width_2_finger_joint_commands(
                    init_grip_joints_pose
                )

        # Log messages
        rospy.loginfo("Setting initial robot pose.")
        rospy.logdebug("Init ee pose:")
        rospy.logdebug(pose_dict_2_pose_msg(init_ee_pose))
        rospy.logdebug("init gripper pose:")
        log_pose_dict(init_grip_joints_pose, header="gripper_pose")

        # Set initial pose
        self.gazebo.unpauseSim()
        ee_pose_retval = self.set_ee_pose(init_ee_pose)
        gripper_pose_retval = self.set_joint_positions(init_grip_joints_pose, wait=True)

        # Throw warning and return result
        if not ee_pose_retval and not gripper_pose_retval:
            rospy.logwarn("Setting initial robot pose failed.")
        elif not ee_pose_retval:
            rospy.logwarn("Setting initial ee pose failed.")
        elif not gripper_pose_retval:
            rospy.logwarn("Setting initial gripper pose failed.")
        return any([ee_pose_retval, gripper_pose_retval])

    def _set_init_obj_pose(self):
        """Sets the grasp object to its initial pose.

        Returns
        -------
        bool
            Success boolean.
        """

        # Set the grasp object pose
        rospy.loginfo("Setting initial object position.")
        if self.has_object:

            # Retrieve x,y positions of the current and initial object pose
            obj_pose = self.model_states[GRASP_OBJECT_NAME]["pose"]
            obj_xy_positions = np.array(
                [
                    self.model_states[GRASP_OBJECT_NAME]["pose"].position.x,
                    self.model_states[GRASP_OBJECT_NAME]["pose"].position.y,
                ]
            )
            init_obj_xy_positions = np.array(
                [self.init_obj_pose.position.x, self.init_obj_pose.position.y,]
            )

            # Sample an object initial object (x, y) position
            # NOTE: This is done relative to the 'init_obj_pose' that is set in the
            # that is supplied through the class constructor
            while (
                np.linalg.norm(
                    np.array([obj_pose.position.x, obj_pose.position.y])
                    - obj_xy_positions
                )
                < self._obj_sampling_distance_threshold
            ):  # Sample till is different enough from the current object pose
                obj_xy_positions = init_obj_xy_positions + self.np_random.uniform(
                    [self.obj_bounds["x_min"], self.obj_bounds["y_min"]],
                    [self.obj_bounds["x_max"], self.obj_bounds["y_max"]],
                    size=2,
                )

            # Set the sampled object x and y positions to the object pose
            obj_pose.position.x = obj_xy_positions[0]
            obj_pose.position.y = obj_xy_positions[1]

            # Set init object pose
            rospy.logdebug("Init object pose:")
            rospy.logdebug(obj_pose)
            try:
                retval = self._set_model_state(GRASP_OBJECT_NAME, obj_pose)
            except SetModelStateError:
                rospy.logerr(
                    "Shutting down '%s' since the state of the grasp object could not "
                    "be set." % (rospy.get_name())
                )
                sys.exit(0)

            # Return result
            if not retval:
                rospy.logwarn("setting initial object position failed.")
            return retval

    def _get_obs(self):
        """Get robot state observation.

        Returns
        -------
        dict
            A dictionary containing the {observation, achieved_goal, desired_goal):

            observation list (22x1):
                - End effector x position
                - End effector y position
                - End effector z position
                - Object x position
                - Object y position
                - Object z position
                - Object/gripper rel. x position
                - Object/gripper rel. y position
                - Object/gripper rel. z position
                - EE joints positions
                - Object pitch (y)
                - Object yaw (z)
                - Object roll (x)
                - Object x velocity
                - Object y velocity
                - Object z velocity
                - Object x angular velocity
                - Object y angular velocity
                - Object z angular velocity
                - EE joints velocities
        """

        # Retrieve robot end effector pose and orientation
        ee_pose = self.get_ee_pose()
        ee_pos = np.array(
            [ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z]
        )

        # Retrieve robot joint pose and velocity
        # IMPROVE: Retrieve real velocity from gazebo
        dt = self._get_elapsed_time()
        grip_velp = (
            ee_pos - self._prev_grip_pos
        ) / dt  # Velocity(position) = Distance/Time
        robot_qpos, robot_qvel = self.robot_get_obs(self.joint_states)

        # Get ee joint positions and velocities
        # NOTE: For the parallel jaw grippper this are the finger joints
        ee_state = robot_qpos[0:2]
        ee_vel = robot_qvel[0:2]

        # Get object pose and (angular)velocity
        if self.has_object:

            # Get object pose
            object_pos = np.array(
                [
                    self.model_states[GRASP_OBJECT_NAME]["pose"].position.x,
                    self.model_states[GRASP_OBJECT_NAME]["pose"].position.y,
                    self.model_states[GRASP_OBJECT_NAME]["pose"].position.z,
                ]
            )

            # Get object orientation
            object_rot_resp = get_orientation_euler(
                self.model_states[GRASP_OBJECT_NAME]["pose"]
            )
            object_rot = np.array(
                [object_rot_resp.y, object_rot_resp.p, object_rot_resp.r]
            )

            # Get object velocity
            object_velp = (
                object_pos - self._prev_object_pos
            ) / dt  # Velocity(position) = Distance/Time
            object_velr = (
                object_rot - self._prev_object_rot
            ) / dt  # Velocity(rotation) = Rotation/Time

            # Get relative position and velocity
            object_rel_pos = object_pos - ee_pos
            object_velp -= grip_velp
        else:
            object_pos = (
                object_rot
            ) = object_velp = object_velr = object_rel_pos = np.zeros(0)

        # Get achieved goal
        achieved_goal = self._sample_achieved_goal(ee_pos, object_pos)

        # Concatenate observations
        obs = np.concatenate(
            [
                ee_pos,
                object_pos.ravel(),
                object_rel_pos.ravel(),
                ee_state,
                object_rot.ravel(),
                object_velp.ravel(),
                object_velr.ravel(),
                ee_vel,
            ]
        )

        # Save current gripper and object positions
        self._prev_grip_pos = ee_pos
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
        # TODO: Add iteratnions done (stepcount)
        pass

    def _set_action(self, action):
        # TODO: ADD section headers
        # TODO: Validate
        """Take robot action.

        Parameters
        ----------
        action : list
            List containing joint or ee action commands.
        """

        # Throw error and shutdown if action space is not the right size
        if not action.shape == self.action_space.shape:
            rospy.logerr(
                "Shutting down '%s' since the shape of the supplied action %s while "
                "the gym action space has shape %s."
                % (rospy.get_name(), action.shape, self.action_space.shape)
            )
            sys.exit(0)

        # ensure that we don't change the action outside of this scope
        action = action.copy()

        # Send action commands to the controllers based on control type
        if self.robot_arm_control_type == "ee_control":

            # Create action dictionary
            action_dict = OrderedDict(zip(self.action_space_joints, action))

            # Convert gripper_width command into finger joints commands
            if (
                self.robot_hand_control_type in POSITION_CONTROL_TYPES
                and self.use_gripper_width
            ):
                action_dict = translate_gripper_width_2_finger_joint_commands(
                    action_dict
                )

            # Make sure the gripper is not moved if block_gripper == True
            # Note: When the gripper joint_commands are not present the current gripper
            # state will be used as a setpoint and thus the gripper is locked
            # TODO: Check if block gripper is implemented the right way
            if self.block_gripper:
                for joint in self._controlled_joints["hand"]:
                    action_dict.pop(joint)

            # Print Debug info
            rospy.logdebug("=Action set info=")
            rospy.logdebug("Action that is set:")
            rospy.logdebug(list(action_dict.values()))

            # Split action_dict into hand and arm control_msgs
            arm_action_dict = {
                key: val
                for key, val in action_dict.items()
                if key in ["x", "y", "z", "rx", "ry", "rz", "rw"]
            }
            hand_action_dict = {
                key: val
                for key, val in action_dict.items()
                if key in self._controlled_joints["hand"]
            }

            # Take action
            self.set_ee_pose(arm_action_dict)
            if self.robot_hand_control_type in POSITION_CONTROL_TYPES:
                self.set_joint_positions(hand_action_dict)
            elif EFFORT_CONTROL_TYPES:  # If hand uses effort control
                self.set_joint_efforts(hand_action_dict)
            else:  # If hand uses joint_trajectory control
                hand_traj_msg = self._action_dict_2_joint_trajectory_msg(
                    hand_action_dict
                )
                self.set_joint_trajectory(hand_traj_msg)
        else:

            # Create action dictionary
            action_dict = OrderedDict(zip(self.action_space_joints, action))

            # Convert gripper_width command into finger joints commands
            if (
                self.robot_hand_control_type in POSITION_CONTROL_TYPES
                and self.use_gripper_width
            ):
                action_dict = translate_gripper_width_2_finger_joint_commands(
                    action_dict
                )

            # Make sure the gripper is not moved if block_gripper == True
            # Note: When the gripper joint_commands are not present the current gripper
            # state will be used as a setpoint and thus the gripper is locked
            if self.block_gripper:
                for joint in self._controlled_joints["hand"]:
                    action_dict.pop(joint)

            # Print Debug info
            rospy.logdebug("=Action set info=")
            rospy.logdebug("Action that is set:")
            rospy.logdebug(list(action_dict.values()))

            # Take action
            if (
                self.robot_arm_control_type in POSITION_CONTROL_TYPES
                and self.robot_hand_control_type in POSITION_CONTROL_TYPES
            ):
                self.set_joint_positions(action_dict)
            elif (
                self.robot_arm_control_type in EFFORT_CONTROL_TYPES
                and self.robot_hand_control_type in EFFORT_CONTROL_TYPES
            ):
                self.set_joint_efforts(action_dict)
            elif (
                self.robot_arm_control_type == "joint_trajectory_control"
                and self.robot_hand_control_type == "joint_trajectory_control"
            ):
                traj_msg = self._action_dict_2_joint_trajectory_msg(action_dict)
                self.set_joint_trajectory(traj_msg)
            else:  # If arm and hand have different control types

                # Split action_dict into hand and arm control_msgs
                arm_action_dict = {
                    key: val
                    for key, val in action_dict.items()
                    if key in self._controlled_joints["arm"]
                }
                hand_action_dict = {
                    key: val
                    for key, val in action_dict.items()
                    if key in self._controlled_joints["hand"]
                }

                # Take arm action
                if self.robot_arm_control_type in POSITION_CONTROL_TYPES:
                    self.set_joint_positions(arm_action_dict)
                elif self.robot_arm_control_type in EFFORT_CONTROL_TYPES:
                    self.set_joint_efforts(arm_action_dict)
                else:
                    arm_traj_msg = self._action_dict_2_joint_trajectory_msg(
                        arm_action_dict
                    )
                    self.set_joint_trajectory(arm_traj_msg)

                # Take hand action
                if self.robot_hand_control_type in POSITION_CONTROL_TYPES:
                    self.set_joint_positions(hand_action_dict)
                elif self.robot_hand_control_type in EFFORT_CONTROL_TYPES:
                    self.set_joint_efforts(hand_action_dict)
                else:
                    hand_traj_msg = self._action_dict_2_joint_trajectory_msg(
                        hand_action_dict
                    )
                    self.set_joint_trajectory(hand_traj_msg)

    def _is_done(self, observations):
        # TODO: ADD max iterations
        """Check if task is done.

        Parameters
        ----------
        observations : dict
            Dictionary containing the observations

        Returns
        -------
        bool
            Bool specifying whether the distance to the goal is within the distance
            threshold and thus the episode is completed.
        """

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
        """Compute the reward.

        Parameters
        ----------
        observations : dict
            Dictionary containing the observations
        done : bool
            Bool specifying whether an episode is terminated.

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

    def _env_setup(self):
        """Sets up initial configuration of the environment. Can be used to configure
        the initial robot state and extract information from the simulation.
        """

        # Move robot joints into their initial position
        self._set_init_pose()

        # Spawn grasp object, set initial pose and calculate the gripper height offset
        if self.has_object:

            # Spawn the object
            rospy.loginfo("Spawning '%s' object." % GRASP_OBJECT_NAME)
            try:
                self._spawn_object(
                    GRASP_OBJECT_NAME, "grasp_cube", pose=self.init_obj_pose
                )
            except SpawnModelError:
                rospy.logerr(
                    "Shutting down '%s' since the grasp object could not be spawned."
                    % (rospy.get_name(),)
                )
                sys.exit(0)

            # Set initial object pose
            self._set_init_obj_pose()

            # Retrieve the object height
            self.object_height_offset = self.model_states[GRASP_OBJECT_NAME][
                "pose"
            ].position.y

        # Store initial EE and qpose
        try:
            cur_ee_pose = self.get_ee_pose()
        except EePoseLookupError:
            rospy.logwarn(
                "Initial EE pose not stored since it could not be retrieved."
                % (rospy.get_name())
            )
        cur_ee_pos = np.array(
            [
                cur_ee_pose.pose.position.x,
                cur_ee_pose.pose.position.y,
                cur_ee_pose.pose.position.z,
            ]
        )
        self.initial_ee_pos = cur_ee_pos.copy()
        try:
            cur_qpose = list(self.joint_states.position)
        except EePoseLookupError:
            rospy.logwarn(
                "Initial generalized robot pose (qpose) not stored since it could not "
                "be retrieved." % (rospy.get_name())
            )
        cur_qpose = np.array(
            [
                cur_ee_pose.pose.position.x,
                cur_ee_pose.pose.position.y,
                cur_ee_pose.pose.position.z,
            ]
        )
        self.initial_qpose = cur_qpose.copy()

        # Sample a reaching goal
        self.goal = self._sample_goal()
        self._get_obs()

    def _step_callback(self):
        """A custom callback that is called after stepping the simulation. Used
        to enforce additional constraints on the simulation state.
        """
        # TODO: MAke sure the gripper width is respected
        # Make sure the gripper joint positions are equal to the initial positions
        if self.block_gripper:
            self.set
            self.sim.data.set_joint_qpos("robot0:l_gripper_finger_joint", 0.0)
            self.sim.data.set_joint_qpos("robot0:r_gripper_finger_joint", 0.0)

    #############################################
    # Task env helper methods ###################
    #############################################

    def _get_elapsed_time(self):
        """Returns the elapsed time since the last time this function was called.
        """
        current_time = rospy.get_time()
        dt = self._sim_time - current_time
        self._sim_time = current_time
        return dt

    def _get_random_joint_positions(self):
        """Get valid joint position commands for the Panda arm and hand.

        Returns
        -------
        dict
            Dictionary containing a valid joint position for each joint.

        Raises
        ------
        RandomJointPositionsError
            Error thrown when 'get_random_joint_positions' service is not available.
        """

        # Create GetRandomJointPositionsRequest message
        req = GetRandomJointPositionsRequest()
        if hasattr(self, "init_pose_bounds"):  # If the user supplied bounding region

            # Retrieve joint positions bounding regions
            _, joint_pose_bound_region = split_bounds_dict(self.init_pose_bounds)

            # Translate gripper_with bounds to finger bounds
            joint_pose_bound_region = translate_gripper_width_2_finger_joint_commands(
                joint_pose_bound_region
            )

            # Add bounding region to GetRandomEePoseRequest message
            joint_limits = JointLimits()
            joint_limits.names = list(joint_pose_bound_region.keys())
            joint_limits.values = list(joint_pose_bound_region.values())
            req.joint_limits = joint_limits
        if self._services_connection_status[MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC]:

            # Request random pose
            resp = self._moveit_get_random_joint_positions_client(req)

            # Convert to joint_position dictionary and return
            if resp.success:
                joint_positions_dict = OrderedDict(
                    zip(resp.joint_names, resp.joint_positions)
                )
                return joint_positions_dict
            else:
                raise RandomJointPositionsError(
                    message=(
                        "A MoveItCommanderException error occurred in the '%s' service "
                        "when trying to retrieve random (valid) joint positions."
                        % MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC
                    ),
                )
        else:
            raise RandomJointPositionsError(
                message="No random (valid) joint positions could not be retrieved as "
                "the '%s' service is not available."
                % MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC,
            )

    def _get_random_ee_pose(self):
        """Get a valid ee pose command for controlling the Panda Arm end effector.

        Returns
        -------
        geometry_msgs.msg.Pose
            Pose message containing a valid ee pose.

        Raises
        ------
        RandomEePoseError
            Error thrown when 'get_random_ee_pose' service is not available.
        """

        # Create GetRandomEePoseRequest message
        req = GetRandomEePoseRequest()
        if hasattr(self, "init_pose_bounds"):  # If the user supplied bounding region

            # Retrieve end effector pose bounding regions
            ee_pose_bound_region, _ = split_bounds_dict(self.init_pose_bounds)

            # Add bounding region to GetRandomEePoseRequest message
            req.bounding_region = BoundingRegion(**ee_pose_bound_region)

        # Get random pose using moveit get_random_pose service
        if self._services_connection_status[MOVEIT_GET_RANDOM_EE_POSE_TOPIC]:

            # Request random pose
            resp = self._moveit_get_random_ee_pose_client(req)

            # Convert to joint_position dictionary and return
            if resp.success:
                return resp.ee_pose
            else:
                raise RandomEePoseError(
                    message=(
                        "A MoveItCommanderException error occurred in the '%s' service "
                        "when trying to retrieve a random (valid) end effector pose."
                        % MOVEIT_GET_RANDOM_EE_POSE_TOPIC
                    ),
                )
        else:
            raise RandomEePoseError(
                message="A random (valid) end effector pose could not be retrieved as "
                "the '%s' service is not available." % MOVEIT_GET_RANDOM_EE_POSE_TOPIC,
            )

    def _clip_goal_position(self, goal_pose):
        # TODO: check if neeeded
        """Limit the possible goal position x, y and z values to a certian range.

        Parameters
        ----------
        goal_pose : numpy.ndarray
            A numpy array containing the goal x,y and z values.
        """

        # Clip goal using the goal bounds
        goal_pose[0] = np.clip(
            goal_pose[0],
            self._target_bounds[self.target_sampling_strategy]["x_min"],
            self._target_bounds[self.target_sampling_strategy]["x_max"],
        )
        goal_pose[1] = np.clip(
            goal_pose[1],
            self._target_bounds[self.target_sampling_strategy]["y_min"],
            self._target_bounds[self.target_sampling_strategy]["y_max"],
        )
        goal_pose[2] = np.clip(
            goal_pose[2],
            self._target_bounds[self.target_sampling_strategy]["z_min"],
            self._target_bounds[self.target_sampling_strategy]["z_max"],
        )

        # Return goal
        return goal_pose

    def _clip_init_pose(self, init_pose):
        """Limit the possible initial end effector (EE) and Robot joint positions to
        a certain range.

        Parameters
        ----------
        init_pose : dict
            A dictionary containing the init pose values for each of the Robot or
            end effector joints.
        """

        # Clip goal
        if hasattr(self, "init_pose_bounds"):
            init_pose_clipped = {}
            for key, val in init_pose.items():
                try:
                    init_pose_clipped[key] = np.clip(
                        val,
                        self.init_pose_bounds[key + "_min"],
                        self.init_pose_bounds[key + "_max"],
                    )
                except KeyError:
                    init_pose_clipped[key] = val
        else:
            rospy.logwarn(
                "Init pose could not be clipped since no 'init_pose_bounds' were given."
            )
            init_pose_clipped = init_pose

        # Return goal
        return init_pose_clipped

    def _get_controlled_joints(self):
        """Get the joints that can be controlled when using the current Panda arm and
        hand control types.

        Returns
        -------
        dict
            Arm and Hand joints that are available for control.
        """

        # Retrieve all the joints that can be controlled by the control types
        controlled_joints = {}
        resp_arm = self._get_controlled_joints_client(
            GetControlledJointsRequest(control_type=self.robot_arm_control_type)
        )
        resp_hand = self._get_controlled_joints_client(
            GetControlledJointsRequest(control_type=self.robot_hand_control_type)
        )
        controlled_joints["arm"] = (
            resp_arm.controlled_joints_arm if resp_arm.success else PANDA_JOINTS["arm"]
        )
        controlled_joints["hand"] = (
            resp_hand.controlled_joints_hand
            if resp_hand.success
            else PANDA_JOINTS["hand"]
        )
        controlled_joints["both"] = (
            flatten_list([controlled_joints["arm"], controlled_joints["hand"]])
            if self.joint_states.name[0] in controlled_joints["arm"]
            else flatten_list([controlled_joints["hand"], controlled_joints["arm"]])
        )

        # Return currently controlled joints
        return controlled_joints

    def _create_action_space(self):
        # TODO: Add trajectory control option
        """Create the action space for the supplied Panda arm and hand control_types.

        Returns
        -------
        gym.spaces.Box
            The gym action space.
        """

        # Create action space based on control_types
        if self.robot_arm_control_type == "ee_control":

            # Retrieve max action space
            if self.robot_hand_control_type in [
                "joint_position_control",
                "joint_group_position_control",
            ]:
                max_action_space = 9 if not self.use_gripper_width else 8
                actions_string = "EE position(x,y,z), EE-orientation(x,y,z,w), %s" % (
                    "panda_finger_joint1 position, panda_finger_joint2 position"
                    if not self.use_gripper_width
                    else "gripper_width"
                )
            else:
                max_action_space = 9
                actions_string = (
                    "EE position(x,y,z), EE-orientation(x,y,z,w), panda_finger_joint1 "
                    "effort, panda_finger_joint2 effort"
                )

            # Throw warning if more actions are requested than are possible
            if self.n_actions > max_action_space:

                # Log warning message
                rospy.logwarn(
                    "You specified an action space of size %s while the maximum size "
                    "of the action space while using 'ee_control' for the arm and '%s' "
                    "for the hand is %s %s([%s]). Because of this an action space of "
                    "%s will be used during training."
                    % (
                        self.n_actions,
                        self.robot_hand_control_type,
                        max_action_space,
                        "when 'gripper_width' is used "
                        if self.use_gripper_width
                        else "",
                        actions_string,
                        max_action_space,
                    )
                )
                return spaces.Box(
                    self._action_bound_low,
                    self._action_bound_high,
                    shape=(max_action_space,),
                    dtype="float32",
                )
            else:
                return spaces.Box(
                    self._action_bound_low,
                    self._action_bound_high,
                    shape=(self.n_actions,),
                    dtype="float32",
                )
        else:

            # Retrieve max action space
            if self.robot_hand_control_type in [
                "joint_position_control",
                "joint_group_position_control",
            ]:
                max_action_space = (
                    len(self.joint_states.name)
                    if not self.use_gripper_width
                    else len(self.joint_states.name) - 1
                )
                actions_string = (
                    (
                        " position, ".join(
                            map(
                                str,
                                [
                                    item
                                    for item in self.joint_states.name
                                    if item
                                    not in [
                                        "panda_finger_joint1",
                                        "panda_finger_joint2",
                                    ]
                                ],
                            )
                        )
                        + " position, gripper_width"
                        if self.use_gripper_width
                        else " position, ".join(map(str, self.joint_states.name))
                        + " position"
                    )
                    if self.use_gripper_width
                    else str(self.joint_states.name)[1:-1].replace("'", "")
                )
            else:
                max_action_space = len(self.joint_states.name)
                actions_string = (
                    str(self.joint_states.name)[1:-1]
                    .replace("'", "")
                    .replace(",", " effort,")
                    + " effort,"
                    + ("panda_finger_joint1 effort, " "panda_finger_joint2 effort")
                )

            # Throw warning if more actions are requested than are possible
            if self.n_actions > max_action_space:

                # Log warning message
                rospy.logwarn(
                    "You specified an action space of size %s while the maximum size "
                    "of the action space while using 'ee_control' for the arm and '%s' "
                    "for the hand is %s %s([%s]). Because of this an action space of "
                    "%s will used during training."
                    % (
                        self.n_actions,
                        self.robot_hand_control_type,
                        max_action_space,
                        "when 'gripper_width' is used "
                        if self.use_gripper_width
                        else "",
                        actions_string,
                        max_action_space,
                    )
                )
                return spaces.Box(
                    self._action_bound_low,
                    self._action_bound_high,
                    shape=(max_action_space,),
                    dtype="float32",
                )
            else:
                return spaces.Box(
                    self._action_bound_low,
                    self._action_bound_high,
                    shape=(self.n_actions,),
                    dtype="float32",
                )

    def _get_action_space_joints(self):
        """Retrieves the joints that are being controlled when we sample from the action
        space.

        Returns
        -------
        list
            Joints that are controlled.
        """

        # Retrieve controlled joints given the action space size and control_type
        if self.robot_arm_control_type == "ee_control":
            if self.use_gripper_width:

                # Replace gripper_width item with panda finger joints
                action_space_joints = flatten_list(
                    [
                        ["x", "y", "z", "rx", "ry", "rz", "rw"][
                            0 : (self.action_space.shape[0] - 1)
                        ],
                        "gripper_width",
                    ]
                )
            else:
                action_space_joints = flatten_list(
                    [
                        ["x", "y", "z", "rx", "ry", "rz", "rw"],
                        self._controlled_joints["hand"],
                    ]
                )[0 : self.action_space.shape[0]]
        else:  # All other control types
            if self.use_gripper_width:

                # Replace gripper_width item with panda finger joints
                action_space_joints = flatten_list(
                    [
                        "gripper_width",
                        self._controlled_joints["arm"][
                            0 : (self.action_space.shape[0] - 1)
                        ],
                    ]
                    if self._controlled_joints["both"][0]
                    in self._controlled_joints["hand"]
                    else [
                        self._controlled_joints["arm"][
                            0 : (self.action_space.shape[0] - 1)
                        ],
                        "gripper_width",
                    ]
                )
            else:
                action_space_joints = self._controlled_joints["both"][
                    0 : self.action_space.shape[0]
                ]

        # Return action space joints
        return action_space_joints

    def _action_dict_2_joint_trajectory_msg(self, action_dict):
        """Converts an action dictionary into a FollowJointTrajectoryGoal
        msgs.

        Parameters
        ----------
        action_dict : dict
            Dictionary containing actions and joints.

        Returns
        -------
        panda_training.msg.FollowJointTrajectoryGoal
            New FollowJointTrajectoryGoal message.
        """

        # Initiate waypoints and new trajectory message
        goal_msg = FollowJointTrajectoryGoal()
        goal_msg.create_time_axis = True
        goal_msg.time_axis_step = 0.01
        waypoint = JointTrajectoryPoint()

        # creates waypoint from joint_positions
        waypoint.positions = list(action_dict.values())
        goal_msg.trajectory.joint_names = list(action_dict.keys())

        # Add waypoint to trajectory message
        goal_msg.trajectory.points.append(waypoint)

        # Return goal msgs
        return goal_msg

    def _validate_action_space_joints(self, action_space_joints):
        # TODO: UPDATE DOCSTRING
        """Checks whether the joints in the 'controlled_joints' arguments are valid. If
        this is not the case a ROS error will be thrown and the node will be shutdown.
        """

        # Validate if the number of action_space_joints is equal to the action space
        if self._controlled_joints and self.n_actions != len(self._controlled_joints):
            rospy.logerr(
                "Shutting down '%s' since the action space size 'n_actions' (%s) is "
                "unequal to the number of joints that was specified in the "
                "'action_space_joints' argument (%s). Please only supply one of these "
                "arguments or make sure that their size is equal."
                % (rospy.get_name(), len(self.n_actions), len(self._controlled_joints))
            )
            sys.exit(0)

        # Validate if the joints in the action_space_joints list exist
        invalid_joints = []
        for joint in action_space_joints:

            # Check if joint_name is vallid based on the control type
            if self.robot_arm_control_type == "ee_control":
                if joint not in flatten_list(
                    ["x", "y", "z", "rx", "ry", "rz", "rw", "gripper_width"]
                ):
                    invalid_joints.append(joint)
            else:
                if joint not in flatten_list([self.joint_states.name, "gripper_width"]):
                    invalid_joints.append(joint)
        if invalid_joints:
            rospy.logerr(
                "Shutting down '%s' since the %s %s. Please supply the "
                "'action_space_joints' argument with valid Panda joints (%s)."
                % (
                    rospy.get_name(),
                    str(invalid_joints)[1:-1]
                    if len(invalid_joints) == 1
                    else str(invalid_joints),
                    "is not a valid joint"
                    if len(invalid_joints) == 1
                    else "are not valid joints",
                    str(self._controlled_joints["both"])[1:-1],
                )
            )
            sys.exit(0)

    def _validate_input_args(self):
        """Checks wether the input arguments are valid. If this is not the case a ROS error
        will be thrown and the node will be shutdown.
        """

        # Gripper_extra_height
        valid_types = (float, int)
        retval, depth, invalid_types = has_invalid_type(
            self.gripper_extra_height, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "gripper_extra_height", depth, invalid_types, valid_types,
            )

        # Block_gripper
        valid_types = (bool,)
        retval, depth, invalid_types = has_invalid_type(
            self.block_gripper, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "block_gripper", depth, invalid_types, valid_types,
            )

        # Has_object
        valid_types = (bool,)
        retval, depth, invalid_types = has_invalid_type(
            self.has_object, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "has_object", depth, invalid_types, valid_types,
            )

        # Target in the air
        valid_types = (bool,)
        retval, depth, invalid_types = has_invalid_type(
            self.obj_target_in_the_air, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "obj_target_in_the_air", depth, invalid_types, valid_types,
            )

        # Target offset
        valid_types = dict
        valid_items_types = (float, int)
        retval, depth, invalid_types = has_invalid_type(
            self.target_offset, variable_types=valid_types,
        )
        if retval:  # Validate type
            arg_type_error(
                "target_offset", depth, invalid_types, valid_types,
            )
        retval, missing_keys, extra_keys = contains_keys(
            self.target_offset, required_keys=["x", "y", "z"], exclusive=True,
        )
        if not retval:  # Validate keys
            arg_keys_error(
                "target_offset", missing_keys=missing_keys, extra_keys=extra_keys,
            )

        # Target sampling strategy
        valid_types = (str,)
        valid_values = GOAL_SAMPLING_STRATEGIES
        retval, depth, invalid_types = has_invalid_type(
            self.target_sampling_strategy, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "target_sampling_strategy", depth, invalid_types, valid_types,
            )
        retval, invalid_values = has_invalid_value(
            self.target_sampling_strategy, valid_values=valid_values
        )
        if retval:  # Validate values
            arg_value_error(
                "target_sampling_strategy",
                invalid_values=invalid_values,
                valid_values=valid_values,
            )

        # Target bounds
        valid_types = (dict,)
        valid_items_types = (float, int)
        retval, depth, invalid_types = has_invalid_type(
            self._target_bounds[self.target_sampling_strategy],
            variable_types=valid_types,
        )
        if retval:  # Validate type
            arg_type_error(
                "target_bounds", depth, invalid_types, valid_types,
            )
        retval, missing_keys, extra_keys = contains_keys(
            self._target_bounds[self.target_sampling_strategy],
            required_keys=["x_min", "y_min", "z_min", "x_max", "y_max", "z_max"],
            exclusive=True,
        )
        if not retval:  # Validate keys
            arg_keys_error(
                "target_bounds", missing_keys=missing_keys, extra_keys=extra_keys,
            )

        # Distance threshold
        valid_types = (float, int)
        retval, depth, invalid_types = has_invalid_type(
            self.distance_threshold, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "distance_threshold", depth, invalid_types, valid_types,
            )

        # Init pose
        valid_types = (dict,)
        valid_items_types = (float, int)
        required_keys = [
            "x",
            "y",
            "z",
            "rx",
            "ry",
            "rz",
            "rw",
            ["gripper_width", "panda_finger_joint1"],
            ["gripper_width", "panda_finger_joint2"],
        ]
        retval, depth, invalid_types = has_invalid_type(
            self.init_pose, variable_types=valid_types, items_types=valid_items_types
        )
        if retval:  # Validate type
            arg_type_error(
                "init_pose", depth, invalid_types, valid_types,
            )
        retval, missing_keys, extra_keys = contains_keys(
            self.init_pose, required_keys=required_keys, exclusive=True,
        )
        if not retval:  # Validate keys
            arg_keys_error(
                "init_pose", missing_keys=missing_keys, extra_keys=extra_keys,
            )

        # Init pose bounds
        if hasattr(self, "init_pose_bounds"):  # If init_pose_bounds were supplied
            valid_types = (dict,)
            valid_items_types = (float, int)
            required_keys = [
                "x_min",
                "y_min",
                "z_min",
                "x_max",
                "y_max",
                "z_max",
                ["gripper_width_min", "panda_finger_joint1_min"],
                ["gripper_width_min", "panda_finger_joint2_min"],
                ["gripper_width_max", "panda_finger_joint1_max"],
                ["gripper_width_max", "panda_finger_joint2_max"],
            ]
            retval, depth, invalid_types = has_invalid_type(
                self.init_pose_bounds,
                variable_types=valid_types,
                items_types=valid_items_types,
            )
            if retval:  # Validate type
                arg_type_error(
                    "init_pose_bounds", depth, invalid_types, valid_types,
                )
            retval, missing_keys, extra_keys = contains_keys(
                self.init_pose_bounds, required_keys=required_keys, exclusive=True,
            )
            if not retval:  # Validate keys
                arg_keys_error(
                    "init_pose_bounds",
                    missing_keys=missing_keys,
                    extra_keys=extra_keys,
                )

        # Init obj pose
        valid_types = (dict,)
        valid_items_types = (float, int)
        required_keys = ["x", "y", "z", "rx", "ry", "rz", "rw"]
        retval, depth, invalid_types = has_invalid_type(
            self.init_obj_pose,
            variable_types=valid_types,
            items_types=valid_items_types,
        )
        if retval:  # Validate type
            arg_type_error(
                "init_obj_pose", depth, invalid_types, valid_types,
            )
        retval, missing_keys, extra_keys = contains_keys(
            self.init_obj_pose, required_keys=required_keys, exclusive=True,
        )
        if not retval:  # Validate keys
            arg_keys_error(
                "init_obj_pose", missing_keys=missing_keys, extra_keys=extra_keys,
            )

        # Object bounds
        valid_types = (dict,)
        valid_items_types = (float, int)
        required_keys = ["x_min", "y_min", "x_max", "y_max"]
        retval, depth, invalid_types = has_invalid_type(
            self.obj_bounds, variable_types=valid_types, items_types=valid_items_types
        )
        if retval:  # Validate type
            arg_type_error(
                "obj_bounds", depth, invalid_types, valid_types,
            )
        retval, missing_keys, extra_keys = contains_keys(
            self.obj_bounds, required_keys=required_keys, exclusive=True,
        )
        if not retval:  # Validate keys
            arg_keys_error(
                "obj_bounds", missing_keys=missing_keys, extra_keys=extra_keys,
            )

        # Reward type
        valid_types = (str,)
        valid_values = REWARD_TYPES
        retval, depth, invalid_types = has_invalid_type(
            self.reward_type, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "reward_type", depth, invalid_types, valid_types,
            )
        retval, invalid_values = has_invalid_value(
            self.reward_type, valid_values=valid_values
        )
        if retval:  # Validate values
            arg_value_error(
                "reward_type", invalid_values=invalid_values, valid_values=valid_values,
            )

        # Robot arm control type
        valid_types = (str,)
        valid_values = ROBOT_CONTROL_TYPES["arm"]
        retval, depth, invalid_types = has_invalid_type(
            self.robot_arm_control_type, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "robot_arm_control_type", depth, invalid_types, valid_types,
            )
        retval, invalid_values = has_invalid_value(
            self.robot_arm_control_type, valid_values=valid_values
        )
        if retval:  # Validate values
            arg_value_error(
                "robot_arm_control_type",
                invalid_values=invalid_values,
                valid_values=valid_values,
            )

        # Robot hand control type
        valid_types = (str,)
        valid_values = ROBOT_CONTROL_TYPES["hand"]
        retval, depth, invalid_types = has_invalid_type(
            self.robot_hand_control_type, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "robot_hand_control_type", depth, invalid_types, valid_types,
            )
        retval, invalid_values = has_invalid_value(
            self.robot_hand_control_type, valid_values=valid_values
        )
        if retval:  # Validate values
            arg_value_error(
                "robot_hand_control_type",
                invalid_values=invalid_values,
                valid_values=valid_values,
            )

        # Robot n actions
        valid_types = (int,)
        retval, depth, invalid_types = has_invalid_type(
            self.n_actions, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "n_actions", depth, invalid_types, valid_types,
            )

        # Action space joints
        # NOTE: The joint names in the action_Space_joints variable are validated at
        # the end of the constructor
        valid_types = (list,)
        retval, depth, invalid_types = has_invalid_type(
            self.action_space_joints, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "action_space_joints", depth, invalid_types, valid_types,
            )

        # Use gripper width
        valid_types = (bool,)
        retval, depth, invalid_types = has_invalid_type(
            self.use_gripper_width, variable_types=valid_types
        )
        if retval:  # Validate type
            arg_type_error(
                "use_gripper_width", depth, invalid_types, valid_types,
            )
