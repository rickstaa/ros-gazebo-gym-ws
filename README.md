# ROS Gazebo Gym Workspace

[![ROS Test](https://github.com/rickstaa/ros-gazebo-gym-ws/actions/workflows/ros_test.yml/badge.svg?branch=noetic)](https://github.com/rickstaa/ros-gazebo-gym-ws/actions/workflows/ros_test.yml)
[![GitHub release (latest by date)](https://img.shields.io/github/v/release/rickstaa/ros-gazebo-gym-ws)](https://github.com/rickstaa/ros-gazebo-gym-ws/releases)
[![Python 3](https://img.shields.io/badge/Python->=3.8-brightgreen)](https://www.python.org/)
[![ROS version](https://img.shields.io/badge/ROS%20versions-Noetic-brightgreen)](https://wiki.ros.org)
[![Contributions](https://img.shields.io/badge/contributions-welcome-brightgreen.svg)](https://github.com/rickstaa/ros-gazebo-gym/blob/noetic/CONTRIBUTING.md)
[![DOI](https://zenodo.org/badge/453634930.svg)](https://zenodo.org/badge/latestdoi/453634930)

Welcome to the ROS Gazebo Gym Workspace repository! This repository is designed to simplify the installation and usage of the [ros\_gazebo\_gym](https://github.com/rickstaa/ros-gazebo-gym) framework, providing a convenient workspace for experimenting with various ROS gymnasium task environments.

## Repository Overview

The ROS Gazebo Gym Workspace comprises two essential components:

1.  [ros\_gazebo\_gym](https://github.com/rickstaa/ros-gazebo-gym) package: This package encapsulates the entire collection of ROS gymnasium environments provided by the [ros\_gazebo\_gym](https://github.com/rickstaa/ros-gazebo-gym) framework.

2.  [ros\_gazebo\_gym\_examples](https://github.com/rickstaa/ros-gazebo-gym-examples) package: Inside this package, you'll discover a set of reinforcement learning (RL) training scripts that make use of the environments available in the [ros\_gazebo\_gym](https://github.com/rickstaa/ros-gazebo-gym) package.

### Clone the repository

To use this workspace repository, clone the repository inside a Catkin workspace folder. Since the repository contains several git submodules to use all the features, it needs to be cloned using the `--recurse-submodules` argument:

```bash
git clone --recurse-submodules https://github.com/rickstaa/ros-gazebo-gym-ws.git
```

If you already cloned the repository and forgot the `--recurse-submodule` argument, you
can pull the submodules using the following git command:

```bash
git submodule update --init --recursive
```

## How to use

For comprehensive instructions on utilizing the [ros-gazebo-gym](https://github.com/rickstaa/ros-gazebo-gym) framework and running the provided examples from the [ros-gazebo-gym-examples](https://github.com/rickstaa/ros-gazebo-gym-examples) package, refer to the official [ros\_gazebo\_gym documentation](https://rickstaa.dev/ros-gazebo-gym).

## Contributing

We use [husky](https://github.com/typicode/husky) pre-commit hooks and github actions to enforce high code quality. Please check the [contributing guidelines](https://github.com/rickstaa/ros-gazebo-gym/blob/noetic/CONTRIBUTING.md) in the [ros-gazebo-gym](https://github.com/rickstaa/ros-gazebo-gym) package before contributing to this repository.

> \[!NOTE]\
> We used [husky](https://github.com/typicode/husky) instead of [pre-commit](https://pre-commit.com/), which is more commonly used with Python projects. This was done because only some tools we wanted to use were possible to integrate the Please feel free to open a [PR](https://github.com/rickstaa/ros-gazebo-gym-ws/pulls) if you want to switch to pre-commit if this is no longer the case.
