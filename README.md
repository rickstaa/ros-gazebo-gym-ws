# Ros-gazebo-gym workspace

[![GitHub release (latest by date)](https://img.shields.io/github/v/release/rickstaa/ros-gazebo-gym-ws)](https://github.com/rickstaa/ros-gazebo-gym-ws/releases)
[![Python 3](https://img.shields.io/badge/Python->=3.8-brightgreen)](https://www.python.org/)
[![ROS version](https://img.shields.io/badge/ROS%20versions-Noetic-brightgreen)](https://wiki.ros.org)
[![Contributions](https://img.shields.io/badge/contributions-welcome-brightgreen.svg)](https://github.com/rickstaa/ros-gazebo-gym/blob/noetic/contributing.md)

This repository contains the catkin workspace for the [ros\_gazebo\_gym](https://github.com/rickstaa/ros-gazebo-gym) framework. It includes all the components, such as submodules and code, necessary to create various [ros\_gazebo\_gym](https://github.com/rickstaa/ros-gazebo-gym) gymnasium task environments.

The workspace comprises two primary components:

*   The [ros\_gazebo\_gym](https://github.com/rickstaa/ros-gazebo-gym) package: This package encompasses all the ROS gymnasium environments provided by ros\_gazebo\_gym.
*   The [ros\_gazebo\_gym\_examples](https://github.com/rickstaa/ros-gazebo-gym-examples) package: This package contains a collection of reinforcement learning (RL) training scripts that utilize the aforementioned environments.

This workspace repository is offered to facilitate the installation of the [ros\_gazebo\_gym](https://github.com/rickstaa/ros-gazebo-gym) framework, as not all components have been released as ROS packages yet.

## How to use

Below you can find instructions on how to use the examples in this repository. For more comprehensive instructions on how to use this package or the [ros\_gazebo\_gym](https://github.com/rickstaa/ros-gazebo-gym) framework, please refer to the [ros\_gazebo\_gym](https://rickstaa.dev/ros-gazebo-gym) documentation.

### Clone the repository

To use this workspace, clone the repository inside a Catkin workspace folder. Since the repository contains several git submodules to use all the features, it needs to be cloned using the `--recurse-submodules` argument:

```bash
git clone --recurse-submodules https://github.com/rickstaa/ros-gazebo-gym-ws.git
```

If you already cloned the repository and forgot the `--recurse-submodule` argument you
can pull the submodules using the following git command:

```bash
git submodule update --init --recursive
```

### Install the system dependencies

After cloning the repository, you need to install the system dependencies. This can be achieved by using the following [rosdep](http://wiki.ros.org/rosdep) command:

```bash
rosdep install --from-path src --ignore-src -r -y
```

### Build the catkin workspace

After you successfully cloned the repository, you can build the catkin workspace using the following command:

```bash
catkin_make
```

or

```bash
catkin build
```

### Run an example

After the workspace has successfully been built and sourced, you can run any of the examples found in the [ros\_gazebo\_gym\_examples](https://github.com/rickstaa/ros-gazebo-gym-examples) package by launching the `start_training` launch file in the project folder:

```bash
roslaunch ros_gazebo_gym_examples start_training.launch
```

The `ros-gazebo-gym` package will download all the required dependencies for a given example and run the gazebo simulator. After this, the agent will start training in the environment. Each example project uses the Soft-Actor Critic algorithm of the [stablebaselines3](https://stable-baselines3.readthedocs.io/en/master/) package.

> \[!WARNING]
> If you're attempting to run the package on Ubuntu 20.04, you might encounter issues stemming from conflicting versions of the gymnasium and Numpy packages (see [this issue](https://github.com/ros/rosdistro/pull/38242)). You can address this by installing specific versions of gymnasium and Numpy using pip3:
>
> ```bash
> pip install -r requirements/requirements.txt
> ```

### Using virtual environments

If you want to try out the examples inside a virtual environment, you are you are recommended you to use a native Python virtual environment (using the [venv](https://docs.python.org/3/library/venv.html) package) since [Anaconda](https://www.anaconda.com/) is not yet fully compatible with ROS (see [this issue](https://answers.ros.org/question/256886/conflict-anaconda-vs-ros-catking_pkg-not-found/)).

#### Setup virtual environment

First, make sure that [ROS noetic](https://wiki.ros.org/noetic) has been installed on your system (See the [ROS site](https://wiki.ros.org/noetic) for instructions). Following, install the `python3-venv` package and create a virtual python3 environment using the following command:

```bash
python3 -m venv ./venvs/ore --system-site-packages
```

After this environment is created, you can activate it using:

```bash
source ./venv/ore/bin/activate
```

> \[!IMPORTANT]\
> Please note that the `--system-site-packages` flag in the command above is required for the ROS system packages to be available inside the virtual environment.

## Contributing

We use [husky](https://github.com/typicode/husky) pre-commit hooks and github actions to enforce high code quality. Please check the [contributing guidelines](https://github.com/rickstaa/ros-gazebo-gym/blob/noetic/contributing.md) in the [ros-gazebo-gym](https://github.com/rickstaa/ros-gazebo-gym) package before contributing to this repository.
