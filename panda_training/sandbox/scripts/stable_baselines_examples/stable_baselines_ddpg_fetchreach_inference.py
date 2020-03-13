"""Example of using a trained modedl to control a mountain car"""
import gym
from stable_baselines import DDPG
import time
import os

# Parameters
MODEL_DIR = "./models/ddpg_fetchtrain-1583160184.zip"

# Main
if __name__ == "__main__":

    # Print log directory
    print("")
    print("RL loaded from: %s", os.path.abspath(MODEL_DIR))
    print("")

    # Create environment
    env = gym.make("FetchReach-v1")

    # NOTE: Simply wrap the goal-based environment using FlattenDictWrapper
    # and specify the keys that you would like to use.
    env = gym.wrappers.FlattenObservation(env)

    # Load model
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
