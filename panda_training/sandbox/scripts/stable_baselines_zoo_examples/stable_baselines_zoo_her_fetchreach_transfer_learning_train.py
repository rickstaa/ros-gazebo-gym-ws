"""This script shows how we can use transfer learning to learn robot control.
To do this the pre-trained model of stable-baseliens-rl-zoo was used."""

import gym
import time
from stable_baselines import HER, DQN, SAC, DDPG, TD3
import os
import yaml

# Model parameters
MODEL_CLASS = DDPG  # works also with SAC, DDPG and TD
NAME = "her-{}-fetch_reach-transferred-{}".format(
    MODEL_CLASS.__module__.split(".")[-1], int(time.time())
)
TB_LOGDIR = "./panda_training/logs/{}".format(NAME)
MODEL_DIR = "./rl-baselines-zoo/trained_agents/her/FetchReach-v1.pkl"
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
    MODEL_PARAMS = os.path.abspath(os.path.splitext(MODEL_DIR)[0] + "/config.yml")
    with open(MODEL_PARAMS, "r") as f:
        hyperparams = yaml.load(f, Loader=yaml.UnsafeLoader)

    # Overload hyperparameters if we want to
    # hyperparams["batch_size"] = 256
    hyperparams["model_class"] = MODEL_CLASS  # Change from SAC to other

    # Policy should not be changed (NN structure)
    del hyperparams["policy"]

    # Initialize model
    model = HER.load(
        MODEL_DIR, env=env, tensorboard_log=TB_LOGDIR, verbose=1, **hyperparams
    )

    # -- Train --
    # Train the model
    model.learn(int(N_STEPS))
    model.save(TRANSFER_MODEL_DIR)

    # WARNING: you must pass an env
    # or wrap your environment with HERGoalEnvWrapper to use the predict method
    del model  # remove to demonstrate saving and loading
    model = HER.load(TRANSFER_MODEL_DIR, env=env)

    # -- Inference --
    # Visualize results
    obs = env.reset()
    for _ in range(500):
        action, _ = model.predict(obs)
        obs, reward, done, _ = env.step(action)
        env.render()

        if done:
            obs = env.reset()
