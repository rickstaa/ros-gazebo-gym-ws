# Panda Grasp Simulator

Grasp simulator for the Panda Emika Franka robot that can be used to test and train DL/RL based grasping algorithms.

## Dependencies
- [ROS Melodic - Desktop full](https://wiki.ros.org/melodic/Installation/Ubuntu)
- Several system dependencies

### System dependencies

The system dependencies can be installed using the following command:

```bash
sudo apt-get install ros-melodic-moveit-ros-move-group ros-melodic-controller-manager* ros-melodic-moveit* ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller ros-melodic-gazebo-ros* ros-melodic-rviz* libboost-filesystem-dev libjsoncpp-dev python3-pycryptodome python3-gnupg python3-tk
```

## How to build

Since ROS does not yet fully support python3 (see #17), we need to separate the training script (python3) and the ROS gazebo simulation (python2). To do this first create a virtual environment:

```bash
pip install virtualenv 
virtualenv ~/.catkin_ws_python3/openai_venv --python=python3 
```

Then build the compile the [geometry2](https://github.com/ros/geometry2), [ros_comm](https://github.com/ros/ros_comm) and [geometry_msgs](https://github.com/ros/common_msgs) for python3:


```bash
cd ~/.catkin_ws_python3
mkdir src
ROS_PYTHON_VERSION=3
wstool init src
rosinstall_generator ros_comm common_msgs  geometry2 --rosdistro melodic --deps | wstool merge -t src -
wstool update -t src -j8
catkin build --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
```

After this is done you can clone and build the `panda_training` package by running:

```
mkdir ~/panda_training_ws
cd ~/panda_training_ws
git clone --recursive https://github.com/rickstaa/panda_openai_sim.git src
rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys libfranka
catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build -Dfreenect2_DIR=/opt/freenect2/lib/cmake/freenect2"
```

## How to train

1. Open two terminals or two panels in [tmux](https://github.com/tmux/tmux/wiki).
2. Source the ROS setup.bash file.
3. Source the panda_training setup.bash file `source ~/panda_training/devel/setup.bash`
4. In one panel start the training simulation environment using the `roslaunch panda_training train_her.launch` or `roslaunch panda_training train_ddpg.launch`.
6. In the other terminal/panel activate the virtual environment `source ~/.catkin_ws_python3/openai_venv/bin/activate`.
6. Source the required python3 ROS packages `source ~/.catkin_ws_python3/devel/setup.bash`.
7. Start the training `python3 ~/panda_training_ws/src/panda_training/scripts/stable_baselines_her_pandareach_train.py`

## How to configure the algorithms

The algorithm parameters can be found in the `./panda_training/cfg/algorithms` folder.

## How to see the progress

You can use [tensorboard](https://www.tensorflow.org/tensorboard/) to visualize the model training in progress:

```bash
cd ~/panda_training_ws/src
tensorboard --logdir ./logs
```

The file name will be displayed when starting the training.

## How to use the trained model

You can use the trained model by running one of the inference scripts.

**Example:**

```
cd ~/panda_training_ws/src/panda_training/scripts
source ~/.catkin_ws_python3/openai_venv/bin/activate
python stable_baselines_her_pandareach_inference.py model:=<MODEL_NAME>
```
