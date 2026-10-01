# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""The packaged image profile must install every ROS package the sessions invoke.

The session templates emit concrete commands — `ros2 run topic_tools throttle`,
`ros2 run domain_bridge domain_bridge`, and so on. If the packaged image profile
does not install the package behind one of them, the stage dies with
"Package '<name>' not found" and the pipeline stops there.

That failure is quiet where it matters. The producing side reports the topic as
present at its native stage; only the *next* stage is missing, so a receiver
simply never sees the processed topic and waits for something that will never
arrive. Reviewing a diff does not catch it, because the command and the package
list live in different files and neither mentions the other.

So the coupling is asserted here instead of trusted: every package name the
session templates run has to appear in the profile that builds the image those
templates run in.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest
from ros2docker.config import load_config

REPO_ROOT = Path(__file__).resolve().parents[2]
RESOURCES = REPO_ROOT / "src" / "rosotacom" / "resources"
SESSION_CONTENT = RESOURCES / "ws" / "session" / "content"
PROFILES = (
    RESOURCES / "examples" / "ros2docker.json",
    RESOURCES / "examples" / "ros2docker.lyrical.json",
)

#: `ros2 run <package> <executable>` in a template.
ROS2_RUN = re.compile(r"\bros2\s+run\s+([A-Za-z_][A-Za-z0-9_]*)\b")

#: Packages that ship with the base ROS image rather than through APT_PACKAGES.
#: Keep this list short and justified: every entry is a package the profile does
#: NOT install, so an entry added carelessly re-opens the hole this test closes.
BASE_IMAGE_PACKAGES = {
    # rosotacom's own workspace package, built into the image from source.
    "com_py",
    # ros2cli and friends are part of every ROS 2 base image.
    "ros2cli",
}


def _invoked_packages() -> dict[str, set[str]]:
    """Every package name run by a session template, mapped to where it appears."""
    found: dict[str, set[str]] = {}
    for template in sorted(SESSION_CONTENT.rglob("*.yaml")):
        for package in ROS2_RUN.findall(template.read_text(encoding="utf-8")):
            found.setdefault(package, set()).add(str(template.relative_to(RESOURCES)))
    return found


@pytest.mark.parametrize("profile", PROFILES, ids=("kilted", "lyrical"))
def test_profile_installs_every_package_the_sessions_run(profile: Path) -> None:
    # Read it through ros2docker itself, so the test sees exactly what the
    # tool that builds the image sees rather than a second parser's idea of it.
    build_args = load_config(profile)["build_args"]
    installed = str(build_args["APT_PACKAGES"]).split()
    # `ros-<distro>-topic-tools` installs the `topic_tools` package; compare on
    # the ROS package name rather than the apt name so a distro bump does not
    # silently turn this test green.
    installed_ros_packages = {
        name.split("-", 2)[2].replace("-", "_")
        for name in installed
        if name.startswith("ros-") and name.count("-") >= 2
    }
    # Lyrical's domain_bridge is provided by ros2docker's pinned source profile
    # until an official binary package exists.
    if build_args.get("INSTALL_DOMAIN_BRIDGE") == "1":
        installed_ros_packages.add("domain_bridge")

    missing = {
        package: sorted(where)
        for package, where in _invoked_packages().items()
        if package not in installed_ros_packages and package not in BASE_IMAGE_PACKAGES
    }

    assert not missing, (
        "session templates run packages the example image profile does not "
        "install; the stage will fail with \"Package '<name>' not found\" and "
        "the topic will stop at its previous stage:\n"
        + "\n".join(f"  {package}: invoked in {', '.join(where)}" for package, where in sorted(missing.items()))
    )
