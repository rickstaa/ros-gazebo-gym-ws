"""Train the Panda Robot using the HER method of stable_baselines.
"""

# Main python imports
from stable_baselines import HER, DDPG  # ,DQN, SAC, TD3

# from stable_baselines.her import GoalSelectionStrategy, HERGoalEnvWrapper
import gym
import time
import inspect
import sys
import os

# ROS python imports
import rospy

# Import panda gym environment
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
scriptsdir = os.path.abspath(os.path.join(currentdir, "../../../scripts"))
sys.path.insert(0, scriptsdir)
from panda_training.envs.task_envs import PandaReachTaskEnv

# Model parameters
MODEL_POLICY = "MlpPolicy"
MODEL_CLASS = DDPG  # works also with SAC, DDPG and TD3
GOAL_SELECTION_STRATEGY = (
    "future"  # Available strategies (cf paper): future, final, episode, random
)
NAME = "her-panda-reach-{}".format(int(time.time()))
TB_LOGDIR = "./panda_training/logs//{}".format(NAME)
# VIDEO_DIR = "./videos/"
MODEL_DIR = "./panda_training/models/{}".format(NAME)
N_STEPS = 1e5

#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Initialize ros node
    rospy.init_node("panda_training_her", log_level=rospy.DEBUG)

    # Print log directory
    rospy.loginfo("RL results logged to: %s", os.path.abspath(TB_LOGDIR))
    rospy.loginfo("RL results saved to: %s", os.path.abspath(MODEL_DIR) + ".zip")
    rospy.sleep(2)

    # Make environment
    env = gym.make(
        "PandaReach-v0",
        robot_EE_link="panda_hand",
        robot_arm_control_type="joint_position_control",
        robot_hand_control_type="joint_position_control",
    )

    # Wrap the model
    model = HER(
        MODEL_POLICY,
        env,
        MODEL_CLASS,
        n_sampled_goal=4,
        goal_selection_strategy=GOAL_SELECTION_STRATEGY,
        verbose=1,
        tensorboard_log=TB_LOGDIR,
    )

    # -- Train --
    # Train the model
    model.learn(int(N_STEPS))
    model.save(MODEL_DIR)

    # -- Inference --
    # Load model
    del model  # remove to demonstrate saving and loading
    model = HER.load(MODEL_DIR, env=env)

    # Visualize results
    obs = env.reset()
    for _ in range(10):
        action, _ = model.predict(obs)
        obs, reward, done, _ = env.step(action)

        if done:
            obs = env.reset()
