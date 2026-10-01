# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""The content-addressed image name, and the build it replaces.

The whole safety argument for skipping a build is that the name of a published
image is derived from its build inputs and never accepted from a caller. These
tests are that argument: identical inputs agree wherever they are evaluated, any
input change moves the name, and a build only turns into an adoption when a
repository is configured.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import subprocess
from collections.abc import Iterator
from pathlib import Path

import pytest

import rosotacom.cli as rosotacom
from rosotacom import image_cache


@pytest.fixture(autouse=True)
def no_ambient_cache(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(image_cache.IMAGE_CACHE_ENV, raising=False)


def _project(tmp_path: Path, name: str, build_args: dict[str, str]) -> Path:
    """A minimal rosotacom project whose ros2docker config carries build_args."""
    root = tmp_path / name
    root.mkdir()
    (root / "ros2docker.json").write_text(
        json.dumps({"image_name": "ros-communication", "build_args": build_args}),
        encoding="utf-8",
    )
    (root / "rosotacom.yaml").write_text("ros2docker_config: ros2docker.json\n", encoding="utf-8")
    return root


_BUILD_ARGS = {"BASE_IMAGE": "osrf/ros:kilted-desktop-full-noble", "APT_PACKAGES": "iputils-ping"}


def test_fingerprint_ignores_where_the_project_lives(tmp_path: Path) -> None:
    """A copied project is the same image.

    Not a detail: every e2e test runs against its own copy of the packaged
    example, and the publisher runs against the packaged original. If the path
    reached the hash, the two would never agree and nothing would ever be
    adopted.
    """
    here = _project(tmp_path, "here", _BUILD_ARGS)
    there = _project(tmp_path, "there", _BUILD_ARGS)

    assert image_cache.image_fingerprint(here / "ros2docker.json") == image_cache.image_fingerprint(
        there / "ros2docker.json"
    )


def test_fingerprint_ignores_the_image_name(tmp_path: Path) -> None:
    """rosotacom scopes the image name per install; the contents do not change with it."""
    project = _project(tmp_path, "scoped", _BUILD_ARGS)
    config = project / "ros2docker.json"

    assert image_cache.image_fingerprint(config) == image_cache.image_fingerprint(
        config, {"image_name": "ros-communication-deadbeef", "container_name": "whatever"}
    )


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("BASE_IMAGE", "ros:kilted-ros-base"),
        ("DIGEST", "@sha256:0000000000000000000000000000000000000000000000000000000000000000"),
        ("APT_PACKAGES", "iputils-ping curl"),
        ("PIP_PACKAGES", "numpy"),
    ],
)
def test_fingerprint_moves_with_every_build_input(tmp_path: Path, key: str, value: str) -> None:
    baseline = image_cache.image_fingerprint(_project(tmp_path, "base", _BUILD_ARGS) / "ros2docker.json")
    changed = _project(tmp_path, f"changed-{key}", {**_BUILD_ARGS, key: value})

    assert image_cache.image_fingerprint(changed / "ros2docker.json") != baseline


def test_fingerprint_moves_with_the_build_context(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """The Dockerfile and entrypoint ros2docker stages are inputs too.

    They come from the installed ros2docker rather than from this repository, so
    a ros2docker release that changes either one has to change the name — that is
    what keeps the published image from outliving the package that built it.
    """
    import ros2docker.api

    config = _project(tmp_path, "context", _BUILD_ARGS) / "ros2docker.json"
    baseline = image_cache.image_fingerprint(config)
    original = ros2docker.api.build_context

    @contextlib.contextmanager
    def with_a_changed_entrypoint(*args: object, **kwargs: object) -> Iterator[Path]:
        with original(*args, **kwargs) as context_dir:  # type: ignore[arg-type]
            entrypoint = Path(context_dir) / "entrypoint.sh"
            entrypoint.write_text(entrypoint.read_text(encoding="utf-8") + "# changed\n", encoding="utf-8")
            yield Path(context_dir)

    monkeypatch.setattr(ros2docker.api, "build_context", with_a_changed_entrypoint)

    assert image_cache.image_fingerprint(config) != baseline


def test_reference_is_repository_plus_fingerprint() -> None:
    assert image_cache.cached_reference("ghcr.io/owner/name", "abc123") == "ghcr.io/owner/name:abc123"
    assert image_cache.reference_tag("ghcr.io/owner/name:abc123") == "abc123"


def test_repository_may_not_carry_a_tag() -> None:
    """The tag is the content hash. A caller that picks one has defeated the point."""
    with pytest.raises(image_cache.ImageCacheError, match="without a tag"):
        image_cache.cached_reference("ghcr.io/owner/name:latest", "abc123")


def test_cache_repository_reads_the_environment() -> None:
    assert image_cache.cache_repository({image_cache.IMAGE_CACHE_ENV: " ghcr.io/owner/name "}) == "ghcr.io/owner/name"
    assert image_cache.cache_repository({image_cache.IMAGE_CACHE_ENV: "  "}) is None
    assert image_cache.cache_repository({}) is None


def test_adopt_pulls_only_when_the_image_is_absent(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[list[str]] = []

    def fake_run(command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        calls.append(command)
        present = command[1:3] == ["image", "inspect"]
        return subprocess.CompletedProcess(command, 0 if not present else 1, "", "")

    monkeypatch.setattr(image_cache.subprocess, "run", fake_run)
    image_cache.adopt_cached_image("ghcr.io/owner/name:abc", "ros-communication-1234")

    assert calls == [
        ["docker", "image", "inspect", "ghcr.io/owner/name:abc"],
        ["docker", "pull", "ghcr.io/owner/name:abc"],
        ["docker", "tag", "ghcr.io/owner/name:abc", "ros-communication-1234"],
    ]


def test_adopt_fails_loudly_when_the_image_is_not_published(monkeypatch: pytest.MonkeyPatch) -> None:
    """No silent fallback to building.

    A miss means the publisher never ran, or ran against a different tree. Both
    are worth a red job: falling back would quietly restore the 154s per job
    this exists to remove, and it would do so invisibly.
    """
    monkeypatch.setattr(
        image_cache.subprocess,
        "run",
        lambda command, **_: subprocess.CompletedProcess(command, 1, "", "manifest unknown"),
    )
    monkeypatch.setattr(image_cache.time, "sleep", lambda _seconds: None)

    with pytest.raises(image_cache.ImageCacheError, match="Could not pull"):
        image_cache.adopt_cached_image("ghcr.io/owner/name:abc", "ros-communication-1234", attempts=2)


def test_build_runs_ros2docker_when_no_repository_is_configured(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config = _project(tmp_path, "plain", _BUILD_ARGS) / "ros2docker.json"
    built: list[object] = []
    monkeypatch.setattr(rosotacom, "ros2docker_build", lambda **kwargs: built.append(kwargs))

    rosotacom._build_image(config, {"image_name": "ros-communication-1234"})

    assert built == [{"config_file": config, "override": {"image_name": "ros-communication-1234"}}]


def test_build_adopts_the_published_image_when_a_repository_is_configured(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config = _project(tmp_path, "cached", _BUILD_ARGS) / "ros2docker.json"
    monkeypatch.setenv(image_cache.IMAGE_CACHE_ENV, "ghcr.io/owner/name")
    monkeypatch.setattr(
        rosotacom,
        "ros2docker_build",
        lambda **_: pytest.fail("a configured repository must replace the build, not add to it"),
    )
    adopted: list[tuple[str, str]] = []
    monkeypatch.setattr(rosotacom, "adopt_cached_image", lambda ref, name: adopted.append((ref, name)))

    rosotacom._build_image(config, {"image_name": "ros-communication-1234"})

    expected = f"ghcr.io/owner/name:{image_cache.image_fingerprint(config)}"
    assert adopted == [(expected, "ros-communication-1234")]


def test_references_lists_every_image_the_project_builds(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """Publisher and consumer read one list.

    `2_native_chatter` builds a second image from a different package list. If
    the publisher only knew about the project image, the chatter slice would
    reach a strict adopt path for something nobody published.
    """
    project = _example_project(tmp_path, monkeypatch)
    capsys.readouterr()

    rosotacom.image_references_command(
        argparse.Namespace(rosotacom_config=str(project / "rosotacom.yaml"), repository="ghcr.io/owner/name")
    )

    references = capsys.readouterr().out.split()
    assert len(references) == 2, references
    assert len(set(references)) == 2, "two different package lists must not share one name"
    assert all(reference.startswith("ghcr.io/owner/name:") for reference in references)


def test_references_selects_scenario_image_for_active_ros_distro(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    project = _example_project(tmp_path, monkeypatch)
    capsys.readouterr()
    lyrical_project_config = project / "ros2docker.lyrical.json"
    monkeypatch.setenv("ROSOTACOM_ROS2DOCKER_CONFIG", str(lyrical_project_config))

    rosotacom.image_references_command(
        argparse.Namespace(rosotacom_config=str(project / "rosotacom.yaml"), repository="ghcr.io/owner/name")
    )

    references = set(capsys.readouterr().out.split())
    lyrical_application = project / "scripts/2_native_chatter/machine_b/external.lyrical.ros2docker.json"
    kilted_application = project / "scripts/2_native_chatter/machine_b/external.ros2docker.json"
    expected_lyrical = image_cache.cached_reference(
        "ghcr.io/owner/name", image_cache.image_fingerprint(lyrical_application)
    )
    default_kilted = image_cache.cached_reference(
        "ghcr.io/owner/name", image_cache.image_fingerprint(kilted_application)
    )

    assert expected_lyrical in references
    assert default_kilted not in references


def test_build_refuses_a_reference_no_input_hashes_to(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """The publisher may push only what its own inputs produced."""
    project = _example_project(tmp_path, monkeypatch)
    monkeypatch.setattr(
        rosotacom,
        "ros2docker_build",
        lambda **_: pytest.fail("nothing may be built under a name that describes other inputs"),
    )

    with pytest.raises(image_cache.ImageCacheError, match="No image this project builds"):
        rosotacom.image_build_command(
            argparse.Namespace(
                rosotacom_config=str(project / "rosotacom.yaml"),
                reference="ghcr.io/owner/name:" + "0" * 64,
            )
        )


def test_build_builds_the_input_a_reference_names(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    project = _example_project(tmp_path, monkeypatch)
    capsys.readouterr()
    rosotacom.image_references_command(
        argparse.Namespace(rosotacom_config=str(project / "rosotacom.yaml"), repository="ghcr.io/owner/name")
    )
    reference = capsys.readouterr().out.split()[0]

    built: list[object] = []
    monkeypatch.setattr(rosotacom, "ros2docker_build", lambda **kwargs: built.append(kwargs))
    rosotacom.image_build_command(
        argparse.Namespace(rosotacom_config=str(project / "rosotacom.yaml"), reference=reference)
    )

    assert len(built) == 1
    assert built[0]["override"] == {"image_name": reference}  # type: ignore[index]


def _example_project(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """A copy of the packaged example — the project the e2e suite actually runs."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / ".config"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / ".run"))
    target = tmp_path / "example"
    rosotacom.examples_create_command(argparse.Namespace(target=str(target), force=False))
    return target
