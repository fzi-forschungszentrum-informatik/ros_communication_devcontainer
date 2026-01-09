#!/bin/bash

# Print commands and their arguments as they are executed.
set -x

SESSION_DIR=/ws/example/1_heartbeat
IDENTITY=a
rosotacom --session-dir $SESSION_DIR --identity $IDENTITY --force