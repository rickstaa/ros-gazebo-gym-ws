"""This script shows how to use the her fetchreach model that is
found in the stable_baselines_zoo to control a fetch robot."""

import gym
import time
from stable_baselines import HER, DQN, SAC, DDPG, TD3
import os

# Model parameters
MODEL_DIR = "rl-baselines-zoo/trained_agents/her/FetchReach-v1.pkl"

# Main
if __name__ == "__main__":

    # Print log directory
    print("")
    print("RL loaded from: %s", os.path.abspath(MODEL_DIR))
    print("")

    # Create environment
    env = gym.make("FetchReach-v1")

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
