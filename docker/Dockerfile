FROM osrf/ros:noetic-desktop-full

# Install system dependencies.
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

# Copy the source code into the container.
WORKDIR /ros_gazebo_gym_ws
COPY . /ros_gazebo_gym_ws/src

# Ensure ROS is sourced in the bashrc.
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Install the panda-gazebo package.
RUN git clone https://github.com/rickstaa/panda-gazebo.git src/rosdeps/panda-gazebo

# Overwrite the rosdep sources list to fix compatibility issues with Ubuntu 20.04 and install ROS dependencies.
# See: https://rickstaa.dev/ros-gazebo-gym/get_started/usage.html#ubuntu-20-04-compatibility-issues
RUN cp src/ros-gazebo-gym/rosdep/19-ros-gazebo-gym.list /etc/ros/rosdep/sources.list.d/
RUN apt-get update && rosdep update && rosdep install --reinstall --from-path src --ignore-src -r -y && rm -rf /var/lib/apt/lists/*

# Build the catkin workspace.
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

# Source the workspace in the bashrc.
RUN echo "source /ros_gazebo_gym_ws/devel/setup.bash" >> /root/.bashrc
