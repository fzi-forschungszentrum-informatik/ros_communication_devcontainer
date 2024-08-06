#!/bin/bash

# Print commands and their arguments as they are executed.
set -x

# Somehow it is required to init the gnome-terminal, so that it will work, when calling with docker exec
gnome-terminal -- bash -c "echo '' | dconf load /org/gnome/terminal/"

# catkin make
source /opt/ros/${ROS_DISTRO}/setup.bash
# chown necessary since otherwise catkin_make fails because the folder is owned by root.
# this is always the case when mounting folders in docker
# see: https://github.com/moby/moby/issues/2259
sudo chown -R myuser ~/catkin_ws
cd ~/catkin_ws
rm -f ~/catkin_ws/src/CMakeLists.txt 
catkin_make

source ~/.bashrc

# run the CMD passed as command-line arguments
exec "$@"
