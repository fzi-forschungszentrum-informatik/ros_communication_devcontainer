#!/bin/bash

# Print commands and their arguments as they are executed.
set -x

DIRECTORY_OF_THIS_SCRIPT="$(dirname "$(realpath "$0")")" # variable for using paths relative to the file location independently of it gets invoked from

$DIRECTORY_OF_THIS_SCRIPT/test_machine_script.py -c -e -a 'test_arg_a=lorem test_arg_b=ipsum'