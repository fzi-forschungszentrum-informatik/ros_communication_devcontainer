#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e
# Print commands and their arguments as they are executed.
set -x

CONFIG_FOLDER="folderName"
DIRECTORY_OF_THIS_SCRIPT="$(dirname "$(realpath "$0")")" # variable for using paths relative to the file location independently of it gets invoked from

config_path=$DIRECTORY_OF_THIS_SCRIPT/$CONFIG_FOLDER/config.ovpn
auth_path=$DIRECTORY_OF_THIS_SCRIPT/$CONFIG_FOLDER/pw.txt

sudo openvpn --config $config_path --auth-user-pass $auth_path