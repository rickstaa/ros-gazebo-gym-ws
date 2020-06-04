"""Control the Panda Robot via model inference of a model trained with the DDPG algorithm
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

# Import panda gym environment
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
scriptsdir = os.path.abspath(os.path.join(currentdir, "../../../scripts"))
sys.path.insert(0, scriptsdir)
import panda_openai_sim.envs.task_envs import PandaReachEnv

# Get current folder
FILE_PATH = os.path.dirname(os.path.realpath(__file__))
MODEL_FOLDER_PATH = os.path.abspath(
    os.path.join(FILE_PATH, "../../panda_training/models")
)


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
    env = gym.make("PandaReach-v0")

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
