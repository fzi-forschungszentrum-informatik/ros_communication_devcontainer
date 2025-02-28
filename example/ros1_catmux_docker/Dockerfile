# Use the official image for ROS Noetic under Ubuntu 20.04
FROM osrf/ros:noetic-desktop-full-focal

ENV USERNAME=myuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && apt-get update \
  && apt-get -y install sudo \
   # [Optional] Add sudo support for the non-root user
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
   # Clean up
  && rm -rf /var/lib/apt/lists/*

# Use the non-root user in this script
USER ${USERNAME}:${USERNAME}

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