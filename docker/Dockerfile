# Use the official image for ROS Noetic under Ubuntu 20.04
FROM osrf/ros:noetic-desktop-full-focal

# Add non-root user
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

WORKDIR /home/${USERNAME}/

# dev tools
RUN sudo apt-get update && sudo apt-get -y install \
    git \
    python-is-python3 \
  && sudo rm -rf /var/lib/apt/lists/*

## Source ROS in bash
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /home/${USERNAME}/.bashrc
RUN echo '[ -f ~/catkin_ws/devel/setup.bash ] && source ~/catkin_ws/devel/setup.bash' >> /home/${USERNAME}/.bashrc

# catmux
RUN sudo apt-get update && sudo apt-get -y install \
    tmux \
    python3-pip \
  && sudo rm -rf /var/lib/apt/lists/*
# version 0.3.4 has problems with catmux_create_session
RUN pip install --user catmux
RUN echo 'alias catmux="tmux -L catmux"' >> /home/${USERNAME}/.bashrc

RUN pip install --user PyYAML

# network tools
RUN sudo apt-get update && sudo apt-get -y install \
    ssh \
    openvpn \
    iputils-ping \
    psmisc \
    net-tools \
    snmp \
    wireless-tools \
    wakeonlan \
  && sudo rm -rf /var/lib/apt/lists/*
RUN pip install --user pysnmp
RUN pip install --user speedtest-cli
RUN pip install --user psutil
RUN pip install --user zstandard

# ros-multi-master
RUN sudo apt-get update && sudo apt-get -y install \
    ros-noetic-fkie-multimaster \
  && sudo rm -rf /var/lib/apt/lists/*

# Install gnome-terminal to be able to launch a terminal from inside the docker
# - install dependencies
    # dbus: 
        # neccessary for the gnome-terminal to work
        # see: https://askubuntu.com/a/1407915
    # dconf-cli: for setting the gnome-terminal preferences via cli
    # libglib2.0-bin for using gsettings to fix a bug later
RUN sudo apt-get update && sudo apt-get install -y \
    gnome-terminal \
    dbus-x11 \
    dconf-cli \
    libglib2.0-bin

# Configure GNOME Terminal settings: hide menubar, don't use theme colors
RUN dbus-launch gsettings set org.gnome.Terminal.Legacy.Settings default-show-menubar false && \
    profile_ids=$(dbus-launch gsettings get org.gnome.Terminal.ProfilesList list | tr -d "[]',") && \
    for id in $profile_ids; do \
        dbus-launch gsettings set org.gnome.Terminal.Legacy.Profile:/org/gnome/terminal/legacy/profiles:/:$id/ use-theme-colors false; \
        dbus-launch gsettings set org.gnome.Terminal.Legacy.Profile:/org/gnome/terminal/legacy/profiles:/:$id/ scrollbar-policy 'never'; \
    done
# Note about dbus-launch: It's used to start a D-Bus session bus instance. This is necessary because gsettings
# and dconf require an active D-Bus session to communicate with the gsettings backend.
