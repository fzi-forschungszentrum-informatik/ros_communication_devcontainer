# SPDX-License-Identifier: Apache-2.0
# Author: Open Source Robotics Foundation, Inc.
# Date: 2015 (upstream copyright year)
# Modified: 2026-10-01, publication metadata and copyright check.

# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path


def test_copyright():
    """Every Python source carries its license and attribution, without ROS."""
    root = Path(__file__).resolve().parents[1]
    paths = [root / 'setup.py', *root.glob('com_py/**/*.py'), *root.glob('test/*.py')]
    for path in paths:
        header = '\n'.join(path.read_text().splitlines()[:25])
        license_id = 'Apache-2.0' if path.parent == root / 'test' else 'BSD-3-Clause'
        assert 'SPDX-License-Identifier: ' + license_id in header, str(path)
        assert 'Copyright' in header, str(path)
        assert 'Author:' in header and 'Date:' in header, str(path)
