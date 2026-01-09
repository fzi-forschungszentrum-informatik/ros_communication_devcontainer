#!/bin/bash

# Print commands and their arguments as they are executed.
set -x

SESSION_DIR=/ws/example/2_native_chatter
IDENTITY=a
rosotacom --session-dir $SESSION_DIR --identity $IDENTITY --force