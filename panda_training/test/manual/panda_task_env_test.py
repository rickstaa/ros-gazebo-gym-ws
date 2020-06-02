"""Script used for testing features of the panda_training task environment during
develeopment.
"""

# ROS python imports
import rospy
import gym

# Import panda gym environment
from panda_training.envs.task_envs import PandaTaskEnv

#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("panda_training_her", log_level=rospy.DEBUG)

    # Create environment6
    env = gym.make(
        "PandaReach-v0",
        robot_arm_control_type="ee_control",
        robot_hand_control_type="joint_position_control",
        # n_actions=3,7
        # init_pose_type="ee_pose",
        init_pose_bounds={
            "x_min": 0,
            "x_max": 0.4,
            "y_min": 0,
            "y_max": 0.4,
            "z_min": 0,
            "z_max": 1,
            "gripper_width_min": 0,
            "gripper_width_max": 0.4,
        },
        has_object=True,
        # gripper_extra_height="jan",
        # controlled_joints=["panda_joint1", "panda_joint2"],
    )

    # NOTE: Simply wrap the goal-based environment using FlattenDictWrapper
    # and specify the keys that you would like to use.
    env = gym.wrappers.FlattenObservation(env)

    # -- TEST action function --
    # -- Inference --
    # Visualize results
    obs = env.reset()
    obs, reward, done, _ = env.step(env.action_space.sample())
