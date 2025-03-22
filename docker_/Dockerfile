# syntax=docker/dockerfile:1.4

FROM ubuntu:24.04

# Add metadata labels
LABEL maintainer="Martin Gontscharow <m.gont94@gmail.com>"
LABEL description="ROS1 Noetic + ROS2 Jazzy bridge container"
LABEL version="1.0"

# Use ARG for configurable values
ARG DEBIAN_FRONTEND=noninteractive
ARG TZ=Europe/Berlin
ARG PYTHON_VENV_PATH=/opt/ros_venv

##############################################################################
# 1) Base environment
##############################################################################
ENV TZ=${TZ}
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=${DEBIAN_FRONTEND}

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

##############################################################################
# 2) Install apt dependencies
##############################################################################
# - Combine apt-get update & install
RUN apt-get update && apt-get install -y \
    # Basic dev tools & libs
    python3 python3-pip python3-venv python3-dev python3-empy \
    cmake build-essential git wget ffmpeg \
    libsm6 libxext6 ninja-build libglib2.0-0 libxrender-dev \
    vim ssh emacs curl gnupg lsb-release sudo \
    tmux gosu openvpn iputils-ping psmisc \
    net-tools snmp wireless-tools wakeonlan \
    libboost-iostreams-dev libboost-all-dev libyaml-cpp-dev \
 && rm -rf /var/lib/apt/lists/*

##############################################################################
# 3) Install pip dependencies in virtual environment
##############################################################################
RUN python3 -m venv ${PYTHON_VENV_PATH} --system-site-packages && \
    ${PYTHON_VENV_PATH}/bin/pip install --no-cache-dir --upgrade pip && \
    . ${PYTHON_VENV_PATH}/bin/activate && \
    pip install --no-cache-dir \
      catkin_tools \
      rosdep \
      rosinstall_generator \
      vcstool \
      PyYAML \
      catmux \
      lz4 \
      pysnmp \
      speedtest-cli \
      psutil \
      zstandard

##############################################################################
# 4) Install ROS 1 (Noetic)
##############################################################################
# Add the ROS 2 repository for certain packages on 24.04
# This helps rosdep find dependencies that might be missing for Noetic
RUN curl -sSL "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key" \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
 && apt-get update

# Initialize rosdep & fetch Noetic
RUN mkdir -p /ros1_ws/src
RUN . ${PYTHON_VENV_PATH}/bin/activate && \
    rosdep init && \
    rosdep update && \
    rosinstall_generator ros_core --rosdistro noetic --deps --tar > noetic.core.rosinstall && \
    vcs import --input noetic.core.rosinstall /ros1_ws/src && \
    rosdep install --from-paths /ros1_ws/src \
      --ignore-packages-from-source \
      --rosdistro noetic \
      --skip-keys="python3-catkin-pkg" \
      -y

# Patch rosconsole
RUN mkdir -p /ros1_ws/src/rosconsole && \
    wget -O /ros1_ws/src/rosconsole/rosconsole.patch https://gist.githubusercontent.com/Crcodlus/111f25ac48fed5004a86aae7a9d758c7/raw/e21c3fb9ddd04e67f1873855eeef2bcb50ac883a/rosconsole.patch && \
    git -C /ros1_ws/src/rosconsole apply --ignore-whitespace /ros1_ws/src/rosconsole/rosconsole.patch
COPY rosconsole_log4cxx.cpp /ros1_ws/src/rosconsole/src/rosconsole/impl/
COPY roslogging.py /ros1_ws/src/ros_comm/rosgraph/src/rosgraph/

# Clone geometry2 (includes tf2_msgs)
RUN . ${PYTHON_VENV_PATH}/bin/activate && \
    cd /ros1_ws/src && \
    git clone --branch noetic-devel https://github.com/ros/geometry2.git geometry2 && \
    cd geometry2 && \
    rm -rf tf2_geometry_msgs tf2_sensor_msgs geometry2 tf2 tf2_bullet tf2_eigen tf2_kdl tf2_py tf2_ros tf2_tools test_tf2  # DELETE EVERYTHING EXCEPT tf2_msgs

# Build Noetic
RUN ${PYTHON_VENV_PATH}/bin/catkin build --workspace /ros1_ws

##############################################################################
# 5) Install and Setup ROS2 (Jazzy)
##############################################################################
# Install ROS2 (Jazzy) + DDS (system-level apt)
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-base \
    python3-colcon-common-extensions \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rmw-fastrtps-cpp \
 && rm -rf /var/lib/apt/lists/*

# Setup ros2-ros1-bridge
# Create workspace and clone specific version of ros1_bridge
RUN mkdir -p /ros2_ros1_bridge_ws/src && \
    cd /ros2_ros1_bridge_ws/src && \
    git clone https://github.com/ros2/ros1_bridge.git

# Build ros1_bridge with all dependencies sourced in correct order
RUN /bin/bash -c "source ${PYTHON_VENV_PATH}/bin/activate && \
    source /ros1_ws/devel/setup.bash && \
    source /opt/ros/jazzy/setup.bash && \
    cd /ros2_ros1_bridge_ws && \
    colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure"

##############################################################################
# 6) Environment settings
##############################################################################
# We'll store a script that sources the venv + /ros1_ws/devel + jazzy
RUN echo "source ${PYTHON_VENV_PATH}/bin/activate" >> /root/.bashrc
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc
RUN echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> /root/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc
RUN echo 'export PYTHONPATH="'"${PYTHON_VENV_PATH}"'/lib/python3.12/site-packages:$PYTHONPATH"' >> /root/.bashrc
RUN echo 'alias catmux="tmux -L catmux"' >> /root/.bashrc

##############################################################################
# 7) Clean up
##############################################################################
# Delete unused user which is created by default in the ubuntu image
RUN userdel -r $(id -un 1000)

##############################################################################
# 8) EntryPoint
##############################################################################
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]