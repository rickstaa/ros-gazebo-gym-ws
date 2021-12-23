# Openai\_ros workspace

[![GitHub release (latest by date)](https://img.shields.io/github/v/release/rickstaa/gazebo-panda-gym)](https://github.com/rickstaa/gazebo-panda-gym/releases)
[![Python 3](https://img.shields.io/badge/Python-3.8%20%7C%203.7%20%7C%203.6-green)](https://www.python.org/)
[![ROS version](https://img.shields.io/badge/ROS%20versions-Noetic-brightgreen)](https://wiki.ros.org)

This repository contains the workspace for the [openai\_ros](https://wiki.ros.org/openai_ros) ROS package. It
includes all the components (submodules and code) to create a create a
Openai gym environment for the Panda Emika Franka robot. This workspace consists of two
main components the [openai\_ros](https://wiki.ros.org/openai_ros) package and the [openai\_examples\_projects](https://bitbucket.org/rickstaa/openai_examples_projects/src/master/) package. The first package contains all the OpenAi ROS gym environments, and the second package several examples of RL training scripts in which these environments are used.

## Clone instructions

To use this workspace, clone the repository inside your a catkin workspace folder. Since the repository contains several git submodules to use all the features, it needs to be cloned using the `--recurse-submodules` argument:

```bash
git clone --recurse-submodules https://github.com/rickstaa/bayesian-learning-control.git
```

If you already cloned the repository and forgot the `--recurse-submodule` argument you
can pull the submodules using the following git command:

```bash
git submodule update --init --recursive
```

## Installation instructions

After you cloned the repository, you have to install the system dependencies using the `rosdep install --from-path src --ignore-src -r -y` command. After these dependencies are installed, you can build the ROS packages inside the catkin workspace using the following build command:

```bash
catkin build
```

## Usage instructions

To see the Openai Ros gym environments in action, you can pick any of the examples found in the [openai\_examples\_projects](https://bitbucket.org/rickstaa/openai_examples_projects/src/master/) package. These examples can be launched using the `roslaunch` command. The example below uses the [SAC algorithm of the stable-baselines](https://stable-baselines3.readthedocs.io/en/master/modules/sac.html) package to train a reaching task on a (simulated) [Panda Emika Franka](https://www.franka.de/) robot.

```bash
roslaunch panda_openai_ros_example start_training.launch
```
