"""Train a RL algorithm on the Panda robot of the 'panda_openai_sim' package using the
`stable_baselines <https://stable-baselines.readthedocs.io/en/master/index.html>`_
`DDPG <https://stable-baselines.readthedocs.io/en/master/modules/ddpg.html>`_ model
class.
"""

# main python imports
import os
import sys
import gym
import numpy as np

# from stable_baselines.ddpg.policies import MlpPolicy
from stable_baselines.common.noise import (
    # NormalActionNoise,
    OrnsteinUhlenbeckActionNoise,
    # AdaptiveParamNoiseSpec,
)
from stable_baselines.common.callbacks import CheckpointCallback
from stable_baselines import DDPG  # HER, TD3, SAC

from panda_training.functions import (
    get_unique_file_suffix,
    backup_model,
)

# ROS python imports
import rospy

# Import panda openai sim task environments
import panda_openai_sim.envs

# Retrieve file path
FILE_PATH = os.path.dirname(os.path.realpath(__file__))

#################################################
# script settings ###############################
#################################################

# Main settings
TASK_ENV_NAME = "PandaPush-v0"  # task environment name
LOAD = False  # Whether to load a model. If false new model will be trained.
TRANSFER = True  # Whether to use transfer learning to retrain pretrained model
MODEL_LOAD_FILE_NAME = (
    "ddpg-pandapush-v0-2.zip"  # The file name of the model you want to load
)
MAX_EPISODES = 600  # max number of training episodes
MAX_EP_STEPS = 200  # max number of steps per
MODEL_POLICY = "MlpPolicy"  # NOTE: Use MlpPolicy obj if you want to change parameters

# Other settings
INFERENCE_STEPS = 10  # The number of steps you want to perform during the inference
FILE_SUFFIX_TYPE = "number"  # The model suffix type "timestamp" or "number".
SAVE_CHECKPOINTS = False  # Whether to save checkpoints of the model during training
CHECKPOINTS_SAVE_FREQ = 100  # After how many steps you want to save a checkpoint
BACKUP_MODEL = True  # Whether to backup the model if transfer learning
BACKUP_CHECKPOINTS = False  # Whether to backup the checkpoints if transfer learning

#############################################
# Create model save/load path ###############
#############################################
if LOAD:  # If the user want to load an existing model

    # Retrieve full model path
    MODEL_NAME = "-".join(os.path.splitext(MODEL_LOAD_FILE_NAME)[0].split("-"))
    MODEL_DIR = os.path.abspath(
        os.path.join(
            FILE_PATH,
            "../models/{}/stable_baselines/ddpg".format(TASK_ENV_NAME.lower()),
        )
    )  # Retrieve model directory path
    MODEL_FILE = os.path.abspath(
        os.path.join(MODEL_DIR, MODEL_LOAD_FILE_NAME,)
    )  # Retrieve model file path

    # Validate model model_file if LOAD is enabled
    if LOAD:
        if not os.path.isfile(MODEL_FILE):
            rospy.logerr(
                "Trained model could not be loaded as the specified model file '%s' "
                "does not exist. Please verify the 'MODEL_LOAD_FILE_NAME' and try "
                "again." % MODEL_FILE
            )
            sys.exit(0)
else:  # If the user wants to create a new model

    # Create model save path
    MODEL_DIR = os.path.abspath(
        os.path.join(
            FILE_PATH,
            "../models/{}/stable_baselines/ddpg".format(TASK_ENV_NAME.lower()),
        )
    )  # Create model save folder path

    # Create save directory if it does not yet exist
    if not os.path.isdir(MODEL_DIR):
        os.makedirs(MODEL_DIR)
        print("Created model save folder '%s' as it did not yet exits." % MODEL_DIR)

    # Create model name, model file path and tensorboard log directory
    MODEL_NAME = "ddpg-{}-".format(TASK_ENV_NAME.lower()) + get_unique_file_suffix(
        MODEL_DIR, FILE_SUFFIX_TYPE
    )  # Create model name
    MODEL_FILE = os.path.abspath(
        os.path.join(MODEL_DIR, MODEL_NAME + ".zip")
    )  # Create model file path

# Create tensorboard log dir
TB_LOGDIR = os.path.abspath(
    os.path.join(
        FILE_PATH, "../logs/{}/stable-baselines/ddpg".format(TASK_ENV_NAME.lower())
    )
)


#################################################
# Helper functions ##############################
#################################################
def train():
    """Function used for training a new RL model.
    """
    global model

    # Create checkpoint callback
    checkpoint_callbacks = []
    if SAVE_CHECKPOINTS:
        checkpoints_dir = os.path.join(MODEL_DIR, "checkpoints", MODEL_NAME)
        checkpoint_callback = CheckpointCallback(
            save_freq=CHECKPOINTS_SAVE_FREQ,
            save_path=checkpoints_dir,
            name_prefix=MODEL_NAME,
        )  # Save a checkpoint every CHECKPOINTS_SAVE_FREQ steps
        checkpoint_callbacks.append(checkpoint_callback)  # Add callback to callbacks

    # Create backup if transfer learning is used
    if LOAD and TRANSFER:  # If transfer learning is enabled
        backup_model(
            MODEL_FILE, backup_model=BACKUP_MODEL, backup_checkpoints=BACKUP_CHECKPOINTS
        )

    # Train model
    model.learn(
        total_timesteps=int(MAX_EPISODES),
        tb_log_name=MODEL_NAME,
        callback=checkpoint_callbacks,
        reset_num_timesteps=(
            not all([TRANSFER, LOAD])
        ),  # If transfer don't reset timesteps
    )

    # Log message
    rospy.loginfo("Model training finished.")

    # Save model
    model.save(MODEL_FILE)
    rospy.loginfo("RL model saved to: %s" % os.path.abspath(MODEL_FILE))


def eval():
    """Function used to run the model inference.
    """
    global model
    obs = env.reset()
    for _ in range(INFERENCE_STEPS):
        action, _ = model.predict(obs)
        obs, reward, done, _ = env.step(action)

        # Check if done
        if done:
            obs = env.reset()

    # Inference finished message
    rospy.loginfo("Model inference finished.")


#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Initialize ros node
    rospy.init_node(
        "stable_baselines_ddpg_panda_train_and_inference", log_level=rospy.INFO
    )

    # Create environment
    env = gym.make(TASK_ENV_NAME)

    # Set max_episode_steps
    env._max_episode_steps = MAX_EP_STEPS

    # NOTE: As the DDPG algorithm only accepts a Gazebo Env and not a Gazebo Goal env
    # we need to wrap the goal-based environment using FlattenDictWrapper. This will
    # convert the observations dictionary from a dict to a flat list.
    env = gym.wrappers.FlattenObservation(env)

    # Load model if inference otherwise create new model
    if LOAD:
        rospy.loginfo("RL model will be loaded from: %s" % os.path.abspath(MODEL_DIR))
        rospy.sleep(2)
        model = DDPG.load(MODEL_FILE, env=env, tensorboard_log=TB_LOGDIR)
    else:

        # Create the noise objects for DDPG
        n_actions = env.action_space.shape[-1]
        param_noise = None
        action_noise = OrnsteinUhlenbeckActionNoise(
            mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions)
        )

        # Create the DDPG model
        model = DDPG(
            MODEL_POLICY,
            env,
            verbose=1,
            param_noise=param_noise,
            action_noise=action_noise,
            tensorboard_log=TB_LOGDIR,
        )

    # Choose between load (inference, transfer) or train
    if LOAD and not TRANSFER:
        rospy.loginfo("Run model inference.")
        rospy.sleep(2)
        eval()
    else:

        # Print log and save directories
        rospy.loginfo("RL results will be logged to: %s" % os.path.abspath(TB_LOGDIR))
        rospy.loginfo("RL model will be saved as: %s" % (os.path.abspath(MODEL_FILE)),)

        # Train model
        if TRANSFER:
            log_str = "Train model using transfer learning."
        else:
            log_str = "Train model."
        rospy.loginfo(log_str)
        rospy.sleep(2)
        train()
