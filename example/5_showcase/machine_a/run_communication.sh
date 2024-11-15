#!/bin/bash

# Print commands and their arguments as they are executed.
set -x

REPOSITORY_PATH=$(dirname "$(realpath "$0")")/../../..
$REPOSITORY_PATH/run_session_in_container.py --session-dir /ws/example/5_showcase/machine_a