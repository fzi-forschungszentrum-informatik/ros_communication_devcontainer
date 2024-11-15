# Use the official image for ROS Noetic under Ubuntu 20.04
FROM osrf/ros:noetic-desktop-full-focal

# Install Catmux
RUN sudo apt-get update && sudo apt-get -y install \
    tmux \
    python3-pip \
  && sudo rm -rf /var/lib/apt/lists/*
RUN pip install catmux
RUN pip install rospy-message-converter

# Foxglove
RUN sudo apt-get update && sudo apt-get -y install \
    ros-$ROS_DISTRO-foxglove-bridge \
  && sudo rm -rf /var/lib/apt/lists/*