# Gazebo Panda Gym workspace

[![GitHub release (latest by date)](https://img.shields.io/github/v/release/rickstaa/gazebo-panda-gym)](https://github.com/rickstaa/gazebo-panda-gym/releases)
[![Python 3](https://img.shields.io/badge/python%203-3.7%20%7C%203.6%20%7C%203.5-brightgreen.svg)](https://www.python.org/)
[![Python 2](https://img.shields.io/badge/python%202-2.7%20%7C%202.6%20%7C%202.5-brightgreen.svg)](https://www.python.org/)
[![ROS versions](https://img.shields.io/badge/ROS%20versions-Melodic-brightgreen)](https://wiki.ros.org)

This repository contains the workspace for the `gazebo_panda_gym` ROS package. It
includes all the components (submodules and code) to create a create a
Openai gym environment for the Panda Emika Franka robot. This workspace consists of two
main components the `panda_gazebo` package and the `panda_training` package. The
the first package (`panda_gazebo`) contains a in Gazebo simulated version of the Panda
robot together with a Panda gym environment that can be used to train RL algorithms as is done
with the original [openai_gym robotics environments](https://gym.openai.com/envs/#robotics).
The second package (`panda_training`) contains several examples of RL training scripts
that can be used together with the simulation and Openai gym environments of the
`panda_gazebo` package.

## Environments

The `panda_gazebo` package currently contains the following task environments:

-   **PandaPickAndPlace-v0:** Lift a block into the air.
-   **PandaPush-v0:** Push a block to a goal position.
-   **PandaReach-v0:** Move fetch to a goal position.
-   **PandaSlide-v0:** Slide a puck to a goal position.

These environments were based on the original [openai_gym robotics environments](https://gym.openai.com/envs/#robotics).

## Installation and Usage

Please see the [docs](https://rickstaa.github.io/gazebo-panda-gym/) for installation
and usage instructions.
