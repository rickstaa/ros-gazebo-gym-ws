"""Example of training a mountain car control algorithm using the DDPG method of
stable_baselines.

VIZUALIZE:
    tensorboard --logdir=""
"""

# Environment
import gym
import numpy as np

# Other
import time
import os

# RL
from stable_baselines.ddpg.policies import MlpPolicy
from stable_baselines.common.noise import (
    NormalActionNoise,
    OrnsteinUhlenbeckActionNoise,
    AdaptiveParamNoiseSpec,
)
from stable_baselines import DDPG

# Model parameters
MODEL_POLICY = MlpPolicy
NAME = "ddpg_fetchtrain-{}".format(int(time.time()))
TB_LOGDIR = "./panda_training/logs//{}".format(NAME)
VIDEO_DIR = "./videos/"
MODEL_DIR = "./panda_training/models/{}".format(NAME)
N_STEPS = 1e5

# Main
if __name__ == "__main__":

    # Print log directory
    print("")
    print("RL results logged to: %s", os.path.abspath(TB_LOGDIR))
    print("")

    # Create environment
    env = gym.make("FetchReach-v1")

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
    model = DDPG.load(MODEL_DIR)

    # -- Inference --
    # Visualize results
    obs = env.reset()
    for _ in range(500):
        action, _ = model.predict(obs)
        obs, reward, done, _ = env.step(action)
        env.render()

        if done:
            obs = env.reset()
