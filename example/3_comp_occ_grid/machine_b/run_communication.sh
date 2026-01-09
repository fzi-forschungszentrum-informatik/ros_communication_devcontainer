#!/bin/bash

# Print commands and their arguments as they are executed.
set -x

SESSION_DIR=/ws/example/3_comp_occ_grid
IDENTITY=b
rosotacom --session-dir $SESSION_DIR --identity $IDENTITY --force