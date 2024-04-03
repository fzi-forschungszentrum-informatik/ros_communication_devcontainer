#!/usr/bin/env python3

import os
import socket
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))))) # add workspace to sys
import utility.machine_utils as machine_utils

def test_machine_script(connect_to_target=False, external_terminal=False, test_arg_a="default_a", test_arg_b="default_b"):
    additional_args = {"test_arg_a":test_arg_a, "test_arg_b":test_arg_b}
    machine_utils.ensure_machine_environment(__file__, connect_to_target, external_terminal, None, **additional_args)

    print(f'''
          Environment after calling of ensure_environment.py:
          Am I inside-container? {os.getenv("INSIDE_CONTAINER")}
          Hostname: {socket.gethostname()}
          test_arg_a: {test_arg_a}
          test_arg_b: {test_arg_b}''')
    
    time.sleep(90)

if __name__ == "__main__":
    machine_utils.machine_main(test_machine_script)