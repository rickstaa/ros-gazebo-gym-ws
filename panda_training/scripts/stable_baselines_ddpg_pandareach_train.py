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

# Import panda gym environment
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
scriptsdir = os.path.abspath(os.path.join(currentdir, "../../../scripts"))
sys.path.insert(0, scriptsdir)
import panda_training.envs.task_envs import PandaReachTaskEnv

# Model parameters
MODEL_POLICY = MlpPolicy
NAME = "ddpg-panda-reach-{}".format(int(time.time()))
TB_LOGDIR = "./panda_training/logs//{}".format(NAME)
# VIDEO_DIR = "./videos/"
MODEL_DIR = "./panda_training/models/{}".format(NAME)
N_STEPS = 3  # 1e5

#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Initialize ros node
    rospy.init_node("panda_training_ddpg", log_level=rospy.DEBUG)

    # Print log directory
    rospy.loginfo("RL results logged to: %s", os.path.abspath(TB_LOGDIR))
    rospy.loginfo("RL results saved to: %s", os.path.abspath(MODEL_DIR) + ".zip")
    rospy.sleep(2)

    # Create environment
    env = gym.make(
        "PandaReach-v0",
        robot_EE_link="panda_grip_site",
        robot_arm_control_type="joint_position_control",
        robot_hand_control_type="joint_position_control",
    )

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
