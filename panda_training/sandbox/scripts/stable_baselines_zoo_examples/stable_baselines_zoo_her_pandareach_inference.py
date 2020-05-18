"""This script shows how to use the her fetchreach model that is
found in the stable_baselines_zoo to control a Panda robot.
NOTE: Does not work due to a different observation space size.
"""


import gym
import time
from stable_baselines import HER, DQN, SAC, DDPG, TD3
import os
import sys
import inspect
import rospy

# Import custom environment
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
scriptsdir = os.path.abspath(os.path.join(currentdir, "../../../scripts"))
sys.path.insert(0, scriptsdir)
import panda_training.envs.task_envs import PandaReachTaskEnv

# Model parameters
MODEL_DIR = "rl-baselines-zoo/trained_agents/her/FetchReach-v1.pkl"

# Main
if __name__ == "__main__":

    # Initialize ros node
    rospy.init_node("panda_training_her", log_level=rospy.DEBUG)

    # Print log directory
    rospy.loginfo("RL model loaded from: %s", os.path.abspath(MODEL_DIR))

    # Make environment
    env = gym.make("PandaReach-v0")

    # Initialize model
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
