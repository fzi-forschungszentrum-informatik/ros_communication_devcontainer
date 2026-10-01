#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

import os

from ros2docker.api import build_run

directory_of_this_script = os.path.dirname(os.path.realpath(__file__))

def run():
    os.chdir(directory_of_this_script)
    config = f'{directory_of_this_script}/external.ros2docker.json'
    build_run(config_file=config)

if __name__ == "__main__":
    run()
