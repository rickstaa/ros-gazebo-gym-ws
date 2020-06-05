"""Control the Panda Robot via model inference of a model trained with the DDPG
algorithm
"""

# Main python imports
import os
import time
from tkinter import Tk
from tkinter.filedialog import askopenfilename
import inspect
import sys
import gym
from stable_baselines import DDPG  # , DQN, SAC, DDPG, TD3

# ROS python imports
import rospy

# Import panda openai sim task environments
import panda_openai_sim.envsv

# Get current folder
FILE_PATH = os.path.dirname(os.path.realpath(__file__))
MODEL_FOLDER_PATH = os.path.abspath(
    os.path.join(FILE_PATH, "../../panda_training/models")
)

# Script parameters
TASK_ENV_NAME = "PandaPush-v0"  # ("PandaReach-v0", "PandaPickAndPlace-v0","PandaSlide-v0","PandaPush-v0")

#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Ask for the filename
    root = Tk()
    root.withdraw()  # we don't want a full GUI, so keep the root window from appearing
    root.call("wm", "attributes", ".", "-topmost", True)
    root.update()
    model_path = askopenfilename(
        initialdir=MODEL_FOLDER_PATH,
        title="Choose the model you want to use for the interference",
    )
    root.update()
    root.destroy()
    # show an "Open" dialog box and return the path to the selected file

    # Print log directory
    print("")
    print("RL loaded from: %s" % os.path.abspath(model_path))
    print("")

    # Sleep to make sure thinker window is destroyed
    time.sleep(2)

    # Initialize ros node
    rospy.init_node("panda_inference_ddpg", log_level=rospy.DEBUG)

    # Create environment
    env = gym.make(TASK_ENV_NAME)

    # NOTE: Simply wrap the goal-based environment using FlattenDictWrapper
    # and specify the keys that you would like to use.
    env = gym.wrappers.FlattenObservation(env)

    # Initialize model
    model = DDPG.load(model_path, env=env)

    # -- Inference --
    # Visualize results
    obs = env.reset()
    for _ in range(10):
        action, _ = model.predict(obs)
        obs, reward, done, _ = env.step(action)

        if done:
            obs = env.reset()

    # Sys exit
    sys.exit(0)  # To make sure tkinter is closed
