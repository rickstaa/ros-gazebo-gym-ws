#!/usr/bin/env python3
"""Small script that can be used to profile the training frequency.

You can get the training frequency by checking the frequency of the
`/panda_arm_joint1_effort_controller/command` topic. This is done using the
`rostopic hz /panda_arm_joint1_effort_controller/command` command.
"""

import gym
import numpy
import rospkg
import rospy
import torch
from openai_ros.core import start_openai_ros_env

# Script parameters
# CONTROL_TYPE = "trajectory"
# CONTROL_TYPE = "end_effector"
# CONTROL_TYPE = "position"
CONTROL_TYPE = "effort"
TASK_ENV = "PandaReach-v0"
# TASK_ENV = "PandaPickAndPlace-v0"
# TASK_ENV = "PandaPush-v0"
# TASK_ENV = "PandaSlide-v0"
N_STEPS = 5000
N_EPISODES = 5

if __name__ == "__main__":
    rospy.init_node("panda_train_freq_test", anonymous=True, log_level=rospy.WARN)

    # Create openai_ros gym environments
    env = gym.make(TASK_ENV, control_type=CONTROL_TYPE)

    # Create the Gym environment
    rospy.loginfo("Gym environment done")
    rospy.loginfo("Starting Learning")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("panda_openai_ros_example")
    outdir = pkg_path + "/training_results"
    last_time_steps = numpy.ndarray(0)

    # Set max_episode_steps
    env._max_episode_steps = N_STEPS

    # Convert goal gym env to normal gym env
    env = gym.wrappers.FlattenObservation(env)

    # Perform simulated training loop
    # NOTE: Here we use random actions and do not train a network
    torch.cuda.empty_cache()
    rospy.logwarn("Starting training loop")
    obs = env.reset()
    for x in range(N_EPISODES):
        done = False
        for i in range(N_STEPS):
            action = env.action_space.sample()
            obs, reward, done, info = env.step(action)
            if done:
                obs = env.reset()

    env.close()
