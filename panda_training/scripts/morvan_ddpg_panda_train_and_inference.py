"""Train a RL algorithm on the Panda robot of the 'panda_openai_sim' package using the
DDPG classes o
`Morvanzhou <https://github.com/MorvanZhou/Reinforcement-learning-with-tensorflow/>`_.

.. note:
    You can train this RL by using LOAD = False, after training, this model will be
    store in the a local folder. Using LOAD = True to reload the trained model for
    playing. You can customize this script in a way you want.
    View more on https://morvanzhou.github.io/tutorials/
    Requirement:
    pyglet >= 1.2.4
    numpy >= 1.12.1
    tensorflow >= 1.0.1
"""

# Main python imports
import tensorflow as tf
import os
import shutil
import gym
import sys
import glob
import numpy as np

from panda_training.functions import get_unique_file_suffix

# ROS python imports
import rospy

# Import panda openai sim task environments
import panda_openai_sim.envs

# Create random seed
np.random.seed(1)
tf.set_random_seed(1)

# Retrieve file path
FILE_PATH = os.path.dirname(os.path.realpath(__file__))

#################################################
# script settings ###############################
#################################################
TASK_ENV_NAME = "PandaPush-v0"  # task environment name
LOAD = False  # Whether to load a pretrained model or train a new model
TRANSFER = False  # Whether to use transfer learning to retrain pretrained model
MAX_EPISODES = 600  # max number of training episodes
MAX_EP_STEPS = 200  # max number of steps per
LR_A = 1e-4  # learning rate for actor
LR_C = 1e-4  # learning rate for critic
GAMMA = 0.9  # reward discount
REPLACE_ITER_A = 1100  # The actor update rate (After how many experiences)
REPLACE_ITER_C = 1000  # The critic update rate (After how many experiences)
MEMORY_CAPACITY = 5000  # The capacity of the replay memory
BATCH_SIZE = 16  # The number of samples you want to sample from the replay buffer
VAR_START = 3  # The variance value the training is started on
VAR_MIN = 0.1  # Minimum exploration rate

# Other settings
INFERENCE_STEPS = 10  # The number of steps you want to perform during the inference
FILE_SUFFIX_TYPE = "number"  # The model folder suffix type "timestamp" or "number".

#############################################
# Model name and save path ##################
#############################################
MODEL_DIR = os.path.abspath(
    os.path.join(FILE_PATH, "../models/{}/movran/ddpg".format(TASK_ENV_NAME.lower()),)
)  # Create model save folder pat
if not os.path.isdir(MODEL_DIR):  # Create folder if it does not yet exist
    os.makedirs(MODEL_DIR)
    print("Created model save folder '%s' as it did not yet exits." % MODEL_DIR)
MODEL_NAME = "ddpg-{}-{}".format(
    TASK_ENV_NAME.lower(), get_unique_file_suffix(MODEL_DIR, FILE_SUFFIX_TYPE)
)  # Model save name
MODEL_FILE = os.path.abspath(
    os.path.join(MODEL_DIR, MODEL_NAME)
)  # Create model file path

# Create tensorboard log dir
TB_LOGDIR = os.path.abspath(
    os.path.join(
        FILE_PATH, "../logs/{}/morvan/{}".format(TASK_ENV_NAME.lower(), MODEL_NAME),
    )
)  # Tensorboard log path

#############################################
# Model load path ###########################
#############################################

# Specify the model you want to load
MODEL_LOAD_DIR = os.path.join("./panda_training/models/morvan/ddpg-pandapush-v0-1/")
MODEL_LOAD_DIR = os.path.abspath(
    os.path.join(FILE_PATH, MODEL_LOAD_DIR)
)  # Create absolute path

# Validate model MODEL_LOAD_DIR if LOAD is enabled
if LOAD:
    MODEL_FILE = os.path.join(MODEL_LOAD_DIR, "DDPG.ckpt*")
    FILES_LIST = glob.glob(MODEL_FILE)
    if not FILES_LIST:
        print(
            "Trained model could not be loaded as the specified model file folder '%s' "
            "does not exist. Please verify the 'MODEL_LOAD_DIR' and try again."
            % MODEL_LOAD_DIR
        )
        sys.exit(0)


#################################################
# Actor class  ##################################
#################################################
class Actor(object):
    """Actor class.
    """

    def __init__(self, sess, action_dim, action_bound, learning_rate, t_replace_iter):
        self.sess = sess
        self.a_dim = action_dim
        self.action_bound = action_bound
        self.lr = learning_rate
        self.t_replace_iter = t_replace_iter
        self.t_replace_counter = 0

        with tf.variable_scope("Actor"):
            # input s, output a
            self.a = self._build_net(S, scope="eval_net", trainable=True)

            # input s_, output a, get a_ for critic
            self.a_ = self._build_net(S_, scope="target_net", trainable=False)

        self.e_params = tf.get_collection(
            tf.GraphKeys.GLOBAL_VARIABLES, scope="Actor/eval_net"
        )
        self.t_params = tf.get_collection(
            tf.GraphKeys.GLOBAL_VARIABLES, scope="Actor/target_net"
        )
        self.replace = [tf.assign(t, e) for t, e in zip(self.t_params, self.e_params)]

    def _build_net(self, s, scope, trainable):
        with tf.variable_scope(scope):
            init_w = tf.contrib.layers.xavier_initializer()
            init_b = tf.constant_initializer(0.001)
            net = tf.layers.dense(
                s,
                200,
                activation=tf.nn.relu6,
                kernel_initializer=init_w,
                bias_initializer=init_b,
                name="l1",
                trainable=trainable,
            )
            net = tf.layers.dense(
                net,
                200,
                activation=tf.nn.relu6,
                kernel_initializer=init_w,
                bias_initializer=init_b,
                name="l2",
                trainable=trainable,
            )
            net = tf.layers.dense(
                net,
                10,
                activation=tf.nn.relu,
                kernel_initializer=init_w,
                bias_initializer=init_b,
                name="l3",
                trainable=trainable,
            )
            with tf.variable_scope("a"):
                actions = tf.layers.dense(
                    net,
                    self.a_dim,
                    activation=tf.nn.tanh,
                    kernel_initializer=init_w,
                    name="a",
                    trainable=trainable,
                )
                self.action_bound = (
                    self.action_bound.T
                    if self.action_bound.shape[1]
                    != [val.value for val in actions.shape][1]
                    else self.action_bound
                )  # Make sure the shape is valid
                scaled_a = tf.multiply(actions, self.action_bound, name="scaled_a")
        return scaled_a

    def learn(self, s):  # batch update
        self.sess.run(self.train_op, feed_dict={S: s})
        if self.t_replace_counter % self.t_replace_iter == 0:
            self.sess.run(self.replace)
        self.t_replace_counter += 1

    def choose_action(self, s):
        s = s[np.newaxis, :]  # single state
        return self.sess.run(self.a, feed_dict={S: s})[0]  # single action

    def add_grad_to_graph(self, a_grads):
        with tf.variable_scope("policy_grads"):
            self.policy_grads = tf.gradients(
                ys=self.a, xs=self.e_params, grad_ys=a_grads
            )

        with tf.variable_scope("A_train"):
            opt = tf.train.RMSPropOptimizer(
                -self.lr
            )  # (- learning rate) for ascent policy
            self.train_op = opt.apply_gradients(zip(self.policy_grads, self.e_params))


#################################################
# Critic class  #################################
#################################################
class Critic(object):
    """Critic class.
    """

    def __init__(
        self, sess, state_dim, action_dim, learning_rate, gamma, t_replace_iter, a, a_
    ):
        self.sess = sess
        self.s_dim = state_dim
        self.a_dim = action_dim
        self.lr = learning_rate
        self.gamma = gamma
        self.t_replace_iter = t_replace_iter
        self.t_replace_counter = 0

        # Add variable summaries

        with tf.variable_scope("Critic"):
            # Input (s, a), output q
            self.a = a
            self.q = self._build_net(S, self.a, "eval_net", trainable=True)

            # Input (s_, a_), output q_ for q_target
            self.q_ = self._build_net(
                S_, a_, "target_net", trainable=False
            )  # target_q is based on a_ from Actor's target_net

            self.e_params = tf.get_collection(
                tf.GraphKeys.GLOBAL_VARIABLES, scope="Critic/eval_net"
            )
            self.t_params = tf.get_collection(
                tf.GraphKeys.GLOBAL_VARIABLES, scope="Critic/target_net"
            )

        with tf.variable_scope("target_q"):
            self.target_q = R + self.gamma * self.q_

        with tf.variable_scope("TD_error"):
            self.loss = tf.reduce_mean(tf.squared_difference(self.target_q, self.q))

        with tf.variable_scope("C_train"):
            self.train_op = tf.train.RMSPropOptimizer(self.lr).minimize(self.loss)

        with tf.variable_scope("a_grad"):
            self.a_grads = tf.gradients(self.q, a)[
                0
            ]  # tensor of gradients of each sample (None, a_dim)
        self.replace = [tf.assign(t, e) for t, e in zip(self.t_params, self.e_params)]

    def _build_net(self, s, a, scope, trainable):
        with tf.variable_scope(scope):
            init_w = tf.contrib.layers.xavier_initializer()
            init_b = tf.constant_initializer(0.01)

            with tf.variable_scope("l1"):
                n_l1 = 200
                w1_s = tf.get_variable(
                    "w1_s", [self.s_dim, n_l1], initializer=init_w, trainable=trainable
                )
                w1_a = tf.get_variable(
                    "w1_a", [self.a_dim, n_l1], initializer=init_w, trainable=trainable
                )
                b1 = tf.get_variable(
                    "b1", [1, n_l1], initializer=init_b, trainable=trainable
                )
                net = tf.nn.relu6(tf.matmul(s, w1_s) + tf.matmul(a, w1_a) + b1)
            net = tf.layers.dense(
                net,
                200,
                activation=tf.nn.relu6,
                kernel_initializer=init_w,
                bias_initializer=init_b,
                name="l2",
                trainable=trainable,
            )
            net = tf.layers.dense(
                net,
                10,
                activation=tf.nn.relu,
                kernel_initializer=init_w,
                bias_initializer=init_b,
                name="l3",
                trainable=trainable,
            )
            with tf.variable_scope("q"):
                q = tf.layers.dense(
                    net,
                    1,
                    kernel_initializer=init_w,
                    bias_initializer=init_b,
                    trainable=trainable,
                )  # Q(s,a)
        return q

    def learn(self, s, a, r, s_):
        self.sess.run(self.train_op, feed_dict={S: s, self.a: a, R: r, S_: s_})
        if self.t_replace_counter % self.t_replace_iter == 0:
            self.sess.run(self.replace)
        self.t_replace_counter += 1


#################################################
# Memory class  #################################
#################################################
class Memory(object):
    """Memory buffer class.
    """

    def __init__(self, capacity, dims):
        self.capacity = capacity
        self.data = np.zeros((capacity, dims))
        self.pointer = 0

    def store_transition(self, s, a, r, s_):
        transition = np.hstack((s, a, [r], s_))
        index = self.pointer % self.capacity  # replace the old memory with new memory
        self.data[index, :] = transition
        self.pointer += 1

    def sample(self, n):
        assert self.pointer >= self.capacity, "Memory has not been fulfilled"
        indices = np.random.choice(self.capacity, size=n)
        return self.data[indices, :]


#################################################
# Additional helper functions ###################
#################################################
def train():
    """Function used to start the training.
    """

    # Variables
    var = VAR_START  # control exploration

    # Create tensorboard variables
    control_exp_pl = tf.placeholder(tf.float32)
    mean_ep_rw_pl = tf.placeholder(tf.float32)
    ep_rw_pl = tf.placeholder(tf.float32)
    with tf.name_scope("reward"):
        tf.summary.scalar("episode_reward", ep_rw_pl)
        tf.summary.scalar("mean_episode_reward", mean_ep_rw_pl)
    tf.summary.scalar("control_exploration", control_exp_pl)
    merged_summaries = tf.summary.merge_all()
    episodes_rewards = []

    # Train model
    for ep in range(MAX_EPISODES):
        s = env.reset()
        ep_reward = 0

        # Episode loop
        for t in range(MAX_EP_STEPS):

            # Choose action
            a = actor.choose_action(s)

            # Add exploration noise
            a = a + var * np.random.uniform(
                env.action_space.low, env.action_space.high,
            )

            # add randomness to action selection for exploration
            a = np.clip(np.random.normal(a, var), ACTION_BOUND[0], ACTION_BOUND[1])

            # Clip the action to make sure they are within the set action bounds
            a = np.clip(a, ACTION_BOUND[0], ACTION_BOUND[1])

            # Take step and check result
            s_, r, done, _ = env.step(a)
            M.store_transition(s, a, r, s_)
            if M.pointer > MEMORY_CAPACITY:
                var = max([var * 0.9999, VAR_MIN])  # decay the action randomness
                b_M = M.sample(BATCH_SIZE)
                b_s = b_M[:, :STATE_DIM]
                b_a = b_M[:, STATE_DIM : STATE_DIM + ACTION_DIM]
                b_r = b_M[:, -STATE_DIM - 1 : -STATE_DIM]
                b_s_ = b_M[:, -STATE_DIM:]

                critic.learn(b_s, b_a, b_r, b_s_)
                actor.learn(b_s)
            s = s_
            ep_reward += r

            # Check if max episodes has been reached or episode is done
            if t == MAX_EP_STEPS - 1 or done:
                # if done:
                result = "| done" if done else "| ----"
                print(
                    "Ep:",
                    ep,
                    result,
                    "| R: %i" % int(ep_reward),
                    "| Explore: %.2f" % var,
                )
                break

        # Add mean episode reward to summary
        episodes_rewards.append(ep_reward)
        summary = sess.run(
            merged_summaries,
            feed_dict={
                ep_rw_pl: ep_reward,
                mean_ep_rw_pl: sum(episodes_rewards) / len(episodes_rewards),
                control_exp_pl: var,
            },
        )
        writer.add_summary(summary, ep)

    # Save model
    if os.path.isdir(MODEL_FILE):
        shutil.rmtree(MODEL_FILE)
    os.mkdir(MODEL_FILE)
    ckpt_path = os.path.join(MODEL_FILE, "DDPG.ckpt")
    save_path = saver.save(sess, ckpt_path, write_meta_graph=False)
    rospy.loginfo("RL model saved to: %s", os.path.abspath(save_path))


def eval():
    """Function used to run the model inference.
    """

    s = env.reset()
    for _ in range(INFERENCE_STEPS):
        a = actor.choose_action(s)
        s_, r, done, _ = env.step(a)
        s = s_

        # Check if done
        if done:
            s = env.reset()


#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Initialize ros node
    rospy.init_node("morvan_ddpg_panda_train_and_inference", log_level=rospy.DEBUG)

    # Create environment6
    env = gym.make(TASK_ENV_NAME,)

    # Set max_episode_steps
    env._max_episode_steps = MAX_EP_STEPS

    # NOTE: Simply wrap the goal-based environment using FlattenDictWrapper
    # and specify the keys that you would like to use.
    env = gym.wrappers.FlattenObservation(env)
    STATE_DIM = env.observation_space.shape[-1]
    ACTION_DIM = env.action_space.shape[-1]
    ACTION_BOUND = np.array([env.action_space.low, env.action_space.high])

    # Tensorflow reset default graph
    tf.reset_default_graph()

    # all placeholder for tf
    with tf.name_scope("S"):
        S = tf.placeholder(tf.float32, shape=[None, STATE_DIM], name="s")
    with tf.name_scope("R"):
        R = tf.placeholder(tf.float32, [None, 1], name="r")
    with tf.name_scope("S_"):
        S_ = tf.placeholder(tf.float32, shape=[None, STATE_DIM], name="s_")

    # Create tensorflow session
    sess = tf.Session()

    # Create actor and critic and memory objects
    actor = Actor(sess, ACTION_DIM, ACTION_BOUND, LR_A, REPLACE_ITER_A)
    critic = Critic(
        sess, STATE_DIM, ACTION_DIM, LR_C, GAMMA, REPLACE_ITER_C, actor.a, actor.a_
    )
    actor.add_grad_to_graph(critic.a_grads)
    M = Memory(MEMORY_CAPACITY, dims=2 * STATE_DIM + ACTION_DIM + 1)

    # Create tensorboard writer
    writer = tf.summary.FileWriter(TB_LOGDIR, sess.graph)

    # Restore session if inference otherwise run new session
    saver = tf.train.Saver()
    if LOAD:
        rospy.loginfo(
            "RL model will be loaded from: %s",
            os.path.abspath(os.path.join(MODEL_LOAD_DIR, "DDPG.ckpt")),
        )
        rospy.sleep(2)
        saver.restore(sess, tf.train.latest_checkpoint(MODEL_LOAD_DIR))
    else:

        # Run session
        sess.run(tf.global_variables_initializer())

    # Choose between load (inference, transfer) or train
    if LOAD and not TRANSFER:
        rospy.loginfo("Run model inference.")
        rospy.sleep(2)
        eval()
    else:

        # Print log and save directories
        rospy.loginfo("RL results will be logged to: %s", os.path.abspath(TB_LOGDIR))
        rospy.loginfo(
            "RL model will be saved as: %s", os.path.join(MODEL_FILE, "DDPG.ckpt")
        )

        # Train model
        rospy.loginfo("Start model training.")
        rospy.sleep(2)
        train()

    # Close writer and session
    writer.flush()  # make sure everything is written to disk
    writer.close()  # not really needed, but good habit
    sess.close()
