"""Train the Panda Robot using the DDPG method of stable_baselines.
"""

# Environment
import gym
import numpy as np
from stable_baselines.ddpg.policies import MlpPolicy
from stable_baselines.common.noise import (
    # NormalActionNoise,
    OrnsteinUhlenbeckActionNoise,
    # AdaptiveParamNoiseSpec,
)
from stable_baselines import DDPG
import time
import os
import inspect
import sys

# ROS python imports
import rospy

# Import panda openai sim task environments
import panda_openai_sim.envs

# Model parameters
TASK_ENV_NAME = "PandaPush-v0"  # ("PandaReach-v0", "PandaPickAndPlace-v0","PandaSlide-v0","PandaPush-v0")
MODEL_POLICY = MlpPolicy
NAME = "ddpg-panda-reach-{}".format(int(time.time()))
TB_LOGDIR = "./panda_training/logs/{}".format(NAME)
# VIDEO_DIR = "./videos/"
MODEL_DIR = "./panda_training/models/{}".format(NAME)
N_STEPS = 3  # 1e5

#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Initialize ros node
    rospy.init_node("panda_openai_sim_ddpg", log_level=rospy.DEBUG)

    # Print log directory
    rospy.loginfo("RL results logged to: %s", os.path.abspath(TB_LOGDIR))
    rospy.loginfo("RL results saved to: %s", os.path.abspath(MODEL_DIR) + ".zip")
    rospy.sleep(2)

    # Create environment
    env = gym.make(TASK_ENV_NAME,)

    # NOTE: Simply wrap the goal-based environment using FlattenDictWrapper
    # and specify the keys that you would like to use.
    env = gym.wrappers.FlattenObservation(env)

    # the noise objects for DDPG
    n_actions = env.action_space.shape[-1]
    param_noise = None
    action_noise = OrnsteinUhlenbeckActionNoise(
        mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions)
    )

    # Create DDPG model
    model = DDPG(
        MODEL_POLICY,
        env,
        verbose=1,
        param_noise=param_noise,
        action_noise=action_noise,
        tensorboard_log=TB_LOGDIR,
    )

    # -- Train --
    # Learn model
    model.learn(total_timesteps=int(N_STEPS))
    model.save(MODEL_DIR)

    # -- Inference --
    # Load model
    del model  # remove to demonstrate saving and loading
    model = DDPG.load(MODEL_DIR, env=env)

    # Visualize results
    obs = env.reset()
    for _ in range(10):
        action, _ = model.predict(obs)
        obs, reward, done, _ = env.step(action)

        if done:
            obs = env.reset()
