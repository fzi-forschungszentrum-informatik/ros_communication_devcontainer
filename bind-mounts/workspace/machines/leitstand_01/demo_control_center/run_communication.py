#!/usr/bin/env python3

import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))))) # add workspace to sys
from utility.machine_utils import ensure_machine_environment, machine_main
from communication_session.create_session import create_session

def run_communication(connect_to_target=False, external_terminal=False):
    ensure_machine_environment(__file__, connect_to_target, external_terminal)
    create_session(os.path.dirname(os.path.realpath(__file__)))

if __name__ == "__main__":
    machine_main(run_communication)