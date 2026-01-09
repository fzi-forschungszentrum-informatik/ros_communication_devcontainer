#!/bin/bash

# Print commands and their arguments as they are executed.
set -x

SESSION_DIR=/ws/example/4_comp_occ_grid_zen
IDENTITY=a
rosotacom --session-dir $SESSION_DIR --identity $IDENTITY --force