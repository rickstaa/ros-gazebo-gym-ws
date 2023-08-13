# Ros\_gazebo\_gym workspace

[![GitHub release (latest by date)](https://img.shields.io/github/v/release/rickstaa/ros-gazebo-gym-ws)](https://github.com/rickstaa/panda-gazebo/releases)
[![Python 3](https://img.shields.io/badge/Python->=3.8-brightgreen)](https://www.python.org/)
[![ROS version](https://img.shields.io/badge/ROS%20versions-Noetic-brightgreen)](https://wiki.ros.org)
[![Contributions](https://img.shields.io/badge/contributions-welcome-brightgreen.svg)](https://github.com/rickstaa/ros-gazebo-gym/blob/noetic/contributing.md)

This repository contains the workspace for the [ros\_gazebo\_gym](https://github.com/rickstaa/ros-gazebo-gym) framework. It
includes all the components (submodules and code) to create
all of the [ros\_gazebo\_gym](https://github.com/rickstaa/ros-gazebo-gym) gym task environment. This workspace consists of two
main components the [ros\_gazebo\_gym](https://github.com/rickstaa/ros-gazebo-gym) package and the [ros\_gazebo\_gym\_examples](https://github.com/rickstaa/ros-gazebo-gym-examples) package. The first package contains all the ros\_gazebo\_gym ROS gym environments, and the second package several examples of RL training scripts in which these environments are used.

## Clone instructions

To use this workspace, clone the repository inside a catkin workspace folder. Since the repository contains several git submodules to use all the features, it needs to be cloned using the `--recurse-submodules` argument:

```bash
git clone --recurse-submodules https://github.com/rickstaa/ros-gazebo-gym-ws.git
```

If you already cloned the repository and forgot the `--recurse-submodule` argument you
can pull the submodules using the following git command:

```bash
git submodule update --init --recursive
```

## Installation and Usage

Please see the [docs](https://rickstaa.dev/ros-gazebo-gym/) for installation and usage instructions.

## Contributing

We use [husky](https://github.com/typicode/husky) pre-commit hooks and github actions to enforce high code quality. Please check the [contributing.md](https://github.com/rickstaa/ros-gazebo-gym/blob/noetic/contributing.md) before contributing to this repository.
