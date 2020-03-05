"""Example of using a trained modedl to control a mountain car"""
import gym
from stable_baselines import DDPG
import time
import os

# Parameters
MODEL_DIR = "./models/ddpg_mountain_car-1583152054.zip"

# Main
if __name__ == "__main__":

    # Print log directory
    print("")
    print("RL loaded from: %s", os.path.abspath(MODEL_DIR))
    print("")

    # Make environment
    env = gym.make("MountainCarContinuous-v0")

    # Load model
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
