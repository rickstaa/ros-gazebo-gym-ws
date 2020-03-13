from stable_baselines import HER, DQN, SAC, DDPG, TD3
from stable_baselines.her import GoalSelectionStrategy, HERGoalEnvWrapper
from stable_baselines.common.bit_flipping_env import BitFlippingEnv
import rospy
import gym
import panda_task_env

model_class = DDPG  # works also with SAC, DDPG and TD3

if __name__ == "__main__":

    # Available strategies (cf paper): future, final, episode, random
    goal_selection_strategy = "future"  # equivalent to GoalSelectionStrategy.FUTURE

    # Initialize rosnode
    rospy.init_node("reach_training_her", anonymous=True, log_level=rospy.WARN)

    env = gym.make("PandaReach-v1")

    # Wrap the model
    model = HER(
        "MlpPolicy",
        env,
        model_class,
        n_sampled_goal=4,
        goal_selection_strategy=goal_selection_strategy,
        verbose=1,
    )
    # Train the model
    model.learn(1000)

    model.save("./her_bit_env")

    # WARNING: you must pass an env
    # or wrap your environment with HERGoalEnvWrapper to use the predict method
    model = HER.load("./her_bit_env", env=env)

    obs = env.reset()
    for _ in range(100):
        action, _ = model.predict(obs)
        rospy.loginfo(action)
        rospy.loginfo("Take new step")
        obs, reward, done, _ = env.step(action)

        if done:
            obs = env.reset()
