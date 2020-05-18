"""This script shows how we can use transfer learning to learn robot control.
As no DDPG model was present in the rl-baslines-zoo for the fetchreach env I tried
using the pretrained HER model.
BUG: Not working as the her model is to different from the DDPG. No actor can be created.
"""

import gym
import time
import numpy as np
from stable_baselines import HER, DQN, SAC, DDPG, TD3
from stable_baselines.common.noise import (
    NormalActionNoise,
    OrnsteinUhlenbeckActionNoise,
    AdaptiveParamNoiseSpec,
)
import os
import yaml
from stable_baselines.ddpg.policies import DDPGPolicy

# Model parameters
MODEL_CLASS = DDPG  # works also with SAC, DDPG and TD
NAME = "{}-fetch_reach-transferred-{}".format(
    MODEL_CLASS.__module__.split(".")[-1], int(time.time())
)
TB_LOGDIR = "./panda_training/logs//{}".format(NAME)

# Transferlearning pretrained model
# NOTE: Her is not working since policy was trained with sac
# We therfore first use transfer learning on the HER model
# to change SAC to DDPG and then use this new model as a
# Pretrained model in this new transfer learning
MODEL_DIR = "./panda_training/models/her-ddpg-fetch_reach-transferred-1583237666.pkl"
TRANSFER_MODEL_DIR = "./panda_training/models/{}".format(NAME)

N_STEPS = 5e4

# Main
if __name__ == "__main__":

    # Print log directory
    print("")
    print("RL results logged to: %s", os.path.abspath(TB_LOGDIR))
    print("")

    # Create environment
    env = gym.make("FetchReach-v1")

    # Retrieve model parameters
    # NOTE: Here we load the her parameters and later change them.
    MODEL_PARAMS = os.path.join(
        os.path.abspath(os.path.join(os.path.splitext(MODEL_DIR)[0], os.pardir))
        + "/PandaReach-v0/ddpg-config.yml"
    )
    with open(MODEL_PARAMS, "r") as f:
        hyperparams = yaml.load(f, Loader=yaml.UnsafeLoader)

    # NOTE: Simply wrap the goal-based environment using FlattenDictWrapper
    # and specify the keys that you would like to use.
    env = gym.wrappers.FlattenObservation(env)

    # Add additional variables that are needed for the DDPG case
    # NOTE: I took the rl-baseliens-zoo biped-walking params as a starting point
    hyperparams["model_class"] = MODEL_CLASS  # Change from SAC to other
    hyperparams["actor_lr"] = hyperparams["learning_rate"]
    hyperparams["memory_limit"] = 100000
    hyperparams["noise_std"] = 0.287
    hyperparams["noise_type"] = "adaptive-param"
    hyperparams["normalize_observations"] = True
    hyperparams["normalize_returns"] = False
    hyperparams["random_exploration"] = 0.0
    hyperparams["policy"] = DDPG

    # Delete Variables that are not used in the DDPG case
    del hyperparams["policy"]
    del hyperparams["model_class"]
    del hyperparams["learning_rate"]
    del hyperparams["buffer_size"]
    del hyperparams["ent_coef"]
    del hyperparams["n_sampled_goal"]
    del hyperparams["learning_starts"]
    del hyperparams["goal_selection_strategy"]

    # the noise objects for DDPG
    n_actions = env.action_space.shape[-1]
    param_noise = None
    action_noise = OrnsteinUhlenbeckActionNoise(
        mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions)
    )

    # Create DDPG model
    model = DDPG.load(
        MODEL_DIR,
        custom_objects={"policy": DDPGPolicy},  # Overload original SAC policy
        env=env,
        tensorboard_log=TB_LOGDIR,
        verbose=1,
        param_noise=param_noise,
        action_noise=action_noise,
        **hyperparams,
    )

    # -- Train --
    # Learn model
    model.learn(total_timesteps=int(N_STEPS))
    model.save(TRANSFER_MODEL_DIR)

    # -- Inference --

    # Load model
    del model  # remove to demonstrate saving and loading
    model = DDPG.load(TRANSFER_MODEL_DIR)

    # -- Inference --
    # Visualize results
    obs = env.reset()
    for _ in range(500):
        action, _ = model.predict(obs)
        obs, reward, done, _ = env.step(action)
        env.render()

        if done:
            obs = env.reset()
