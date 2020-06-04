"""Example of training a bitflip model using the HER method of
stable_baselines.

VIZUALIZE:
    tensorboard --logdir=""
"""

from stable_baselines import HER, DQN, SAC, DDPG, TD3
from stable_baselines.her import GoalSelectionStrategy, HERGoalEnvWrapper
from stable_baselines.common.bit_flipping_env import BitFlippingEnv
import time
import os

# Model parameters
MODEL_POLICY = "MlpPolicy"
MODEL_CLASS = DDPG  # works also with SAC, DDPG and TD3
GOAL_SELECTION_STRATEGY = (
    "future"  # Available strategies (cf paper): future, final, episode, random
)
NAME = "her_bit_flip-{}".format(int(time.time()))
TB_LOGDIR = "./panda_training/logs/{}".format(NAME)
VIDEO_DIR = "./videos/"
MODEL_DIR = "./panda_training/models/{}".format(NAME)
N_BITS = 1

if __name__ == "__main__":

    # Print log directory
    print("")
    print("RL results logged to: %s", os.path.abspath(TB_LOGDIR))
    print("")

    # Create environment
    env = BitFlippingEnv(
        N_BITS, continuous=MODEL_CLASS in [DDPG, SAC, TD3], max_steps=N_BITS
    )

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

    # Train the model
    model.learn(int(2e3))
    model.save(MODEL_DIR)

    # WARNING: you must pass an env
    # or wrap your environment with HERGoalEnvWrapper to use the predict method
    del model  # remove to demonstrate saving and loading
    model = HER.load(MODEL_DIR, env=env)

    obs = env.reset()
    for _ in range(2):
        action, _ = model.predict(obs)
        obs, reward, done, _ = env.step(action)
        env.render()

        if done:
            obs = env.reset()
