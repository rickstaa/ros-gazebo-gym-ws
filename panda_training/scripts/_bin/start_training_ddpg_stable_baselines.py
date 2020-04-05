import gym
import numpy as np

from stable_baselines.ddpg.policies import MlpPolicy
from stable_baselines.common.noise import (
    NormalActionNoise,
    OrnsteinUhlenbeckActionNoise,
    AdaptiveParamNoiseSpec,
)
import rospy
import panda_task_env
from stable_baselines import DDPG

if __name__ == "__main__":

    # Initialize rosnode
    rospy.init_node("reach_training_ddpg", anonymous=True, log_level=rospy.WARN)

    env = gym.make("PandaReach-v0")

    # the noise objects for DDPG
    n_actions = env.action_space.shape[-1]
    param_noise = None
    action_noise = OrnsteinUhlenbeckActionNoise(
        mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions)
    )

    model = DDPG(
        MlpPolicy, env, verbose=1, param_noise=param_noise, action_noise=action_noise
    )
    model.learn(total_timesteps=400000)
    model.save("ddpg_pandareach")

    del model  # remove to demonstrate saving and loading

    model = DDPG.load("ddpg_pandareach")

    obs = env.reset()
    while True:
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)
        env.render()
