#!/usr/bin/env python3

import os
import sys

current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(current_dir))) # add workspace to sys
from compositions.run_composition import run_composition

def run():
    run_composition(current_dir)

if __name__ == "__main__":
    run()