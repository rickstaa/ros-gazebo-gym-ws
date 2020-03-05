"""Example of training a fetch robot model using the HER method of
stable_baselines.

VIZUALIZE:
    tensorboard --logdir=""
"""

# RL
from stable_baselines import HER, DQN, SAC, DDPG, TD3
from stable_baselines.her import GoalSelectionStrategy, HERGoalEnvWrapper
from stable_baselines.common.bit_flipping_env import BitFlippingEnv
import gym
import rospy

# Other
import time
import os
import sys
import inspect

# Import custom environment
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
scriptsdir = os.path.abspath(os.path.join(currentdir, "../../../scripts"))
sys.path.insert(0, scriptsdir)
import panda_task_env

# Model parameters
MODEL_POLICY = "MlpPolicy"
MODEL_CLASS = DDPG  # works also with SAC, DDPG and TD3
GOAL_SELECTION_STRATEGY = (
    "future"
)  # Available strategies (cf paper): future, final, episode, random
NAME = "her_fetch_reach-{}".format(int(time.time()))
TB_LOGDIR = "./logs/{}".format(NAME)
VIDEO_DIR = "./videos/"
MODEL_DIR = "./models/{}".format(NAME)
N_STEPS = 1e5

# Main
if __name__ == "__main__":

    # Initialize ros node
    rospy.init_node("panda_reach_training_her", log_level=rospy.DEBUG)

    # Print log directory
    rospy.loginfo("RL results logged to: %s", os.path.abspath(TB_LOGDIR))

    # Make environment
    env = gym.make("PandaReach-v1")

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

    # WARNING: you must pass an env
    # or wrap your environment with HERGoalEnvWrapper to use the predict method
    del model  # remove to demonstrate saving and loading
    model = HER.load(MODEL_DIR, env=env)

    # -- Inference --
    # Visualize results
    obs = env.reset()
    for _ in range(500):
        action, _ = model.predict(obs)
        obs, reward, done, _ = env.step(action)
        env.render()

        if done:
            obs = env.reset()
