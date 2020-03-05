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
NAME = "ddpg_mountain_car-{}".format(int(time.time()))
TB_LOGDIR = "./logs/{}".format(NAME)
VIDEO_DIR = "./videos/"
MODEL_DIR = "./models/{}".format(NAME)

# Main
if __name__ == "__main__":

    # Print log directory
    print("")
    print("RL results logged to: %s", os.path.abspath(TB_LOGDIR))
    print("")

    # Create environment
    env = gym.make("MountainCarContinuous-v0")

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
    model.learn(total_timesteps=int(4e5))
    model.save(MODEL_DIR)

    # -- Inference --

    # Load model
    del model  # remove to demonstrate saving and loading
    model = DDPG.load(MODEL_DIR)

    # Visualize results interference
    obs = env.reset()
    for _ in range(int(2e2)):

        # Slow down animation
        time.sleep(1 / 1e2)

        action, _states = model.predict(obs)
        obs, rewards, done, info = env.step(action)
        env.render()

        if done:
            time.sleep(2)
            obs = env.reset()
