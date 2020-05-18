"""Example of training a highway model using the HER method of
stable_baselines.

VIZUALIZE:
    tensorboard --logdir=""
"""

# Environment
import gym
import highway_env
import os

# Agent
from stable_baselines import HER, SAC, DDPG
from IPython import display as ipythondisplay

# Visualization
from pyvirtualdisplay import Display
from IPython import display as ipythondisplay
from gym.wrappers import Monitor
from pathlib import Path
import base64
from tqdm import tnrange
import time

# Model parameters
MODEL_POLICY = "MlpPolicy"
MODEL_CLASS = DDPG  # works also with SAC, DDPG and TD3
GOAL_SELECTION_STRATEGY = (
    "future"  # Available strategies (cf paper): future, final, episode, random
)
NAME = "her_highway_park-{}".format(int(time.time()))
TB_LOGDIR = "./panda_training/logs//{}".format(NAME)
VIDEO_DIR = "./videos/"
MODEL_DIR = "./panda_training/models/{}.zip".format(NAME)


# Helper functions
def show_video():
    html = []
    for mp4 in Path("video").glob("*.mp4"):
        video_b64 = base64.b64encode(mp4.read_bytes())
        html.append(
            """<video alt="{}" autoplay
                      loop controls style="height: 400px;">
                      <source src="data:video/mp4;base64,{}" type="video/mp4" />
                 </video>""".format(
                mp4, video_b64.decode("ascii")
            )
        )
    ipythondisplay.display(ipythondisplay.HTML(data="<br>".join(html)))


# Main
if __name__ == "__main__":

    # Print log directory
    print("")
    print("RL results logged to: %s", os.path.abspath(TB_LOGDIR))
    print("")

    # Train
    env = gym.make("parking-ActionRepeat-v0")
    model = HER(
        MODEL_POLICY,
        env,
        MODEL_CLASS,
        n_sampled_goal=4,
        goal_selection_strategy=GOAL_SELECTION_STRATEGY,
        verbose=1,
        buffer_size=int(1e6),
        # learning_rate=1e-3,
        gamma=0.9,
        batch_size=256,
        policy_kwargs=dict(layers=[256, 256, 256]),
        tensorboard_log=TB_LOGDIR,
    )

    # model.learn(int(2e4))
    model.learn(int(2e3))
    model.save(MODEL_DIR)

    # LOAD MODEL
    del model  # remove to demonstrate saving and loading
    model = HER.load(MODEL_DIR, env=env)

    # Visualize
    display = Display(visible=0, size=(1400, 900))
    display.start()

    # Test the policy
    env = gym.make("parking-ActionRepeat-v0")
    env = Monitor(env, VIDEO_DIR, force=True, video_callable=lambda episode: True)
    for episode in tnrange(3, desc="Test episodes"):
        obs, done = env.reset(), False
        env.unwrapped.automatic_rendering_callback = env.video_recorder.capture_frame
        while not done:
            action, _ = model.predict(obs)
            obs, reward, done, info = env.step(action)
    env.close()
    show_video()
