# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Content-addressed names for the images this project builds.

Every e2e job used to rebuild the same project image from scratch, and #236
measured what that cost: 154s per job across the thirteen jobs in that run.
Publishing the built image once and adopting it in each job removes the build —
but only if "adopt" can never mean "adopt something else". A cached image that
no longer matches its build inputs is a silent wrong-result machine, which is why #226
declined to do this inline.

So the name of a published image is a hash of everything that determines it:
the ``docker build`` command ros2docker renders from a config, and the build
context it stages. Nothing else decides what the image contains. A changed base
digest, apt package, pip package, Dockerfile, entrypoint, or ros2docker version
is a different hash and therefore a different name, so "the published image is
stale" is not a state this can be in rather than a state it is unlikely to be
in. A consumer never receives a reference to trust; it derives the one name its
own inputs allow and pulls exactly that.

The image name itself is deliberately excluded from the hash. It says what the
caller wants the result called, not what is in it, and rosotacom scopes it per
install (``ros-communication-<install_id>``), so including it would give two
checkouts of the same tree two different hashes for one image.
"""

from __future__ import annotations

import hashlib
import os
import subprocess
import time
from collections.abc import Mapping
from pathlib import Path

#: Docker repository holding published images, e.g.
#: ``ghcr.io/<owner>/rosotacom-e2e``. Set it and every build this project would
#: run is replaced by a pull of the content-addressed tag; leave it unset and
#: nothing changes.
IMAGE_CACHE_ENV = "ROSOTACOM_IMAGE_CACHE"

_FINGERPRINT_IMAGE_NAME = "rosotacom-image-fingerprint"

_PULL_ATTEMPTS = 3
_PULL_RETRY_SECONDS = 5.0


class ImageCacheError(RuntimeError):
    """A published image could not be resolved, pulled, or adopted."""


def cache_repository(env: Mapping[str, str] | None = None) -> str | None:
    """The configured publish repository, or ``None`` when there is none."""
    source = os.environ if env is None else env
    value = source.get(IMAGE_CACHE_ENV, "").strip()
    return value or None


def image_fingerprint(
    config_file: str | os.PathLike[str] | None = None,
    override: Mapping[str, object] | None = None,
) -> str:
    """Hash everything that decides what ``ros2docker build`` would produce.

    Both halves are taken from ros2docker rather than re-derived here: the
    rendered command carries the build args (``BASE_IMAGE``, its ``DIGEST``,
    ``APT_PACKAGES``, ``PIP_PACKAGES``, ``USER_UID``/``USER_GID``) and the
    staged context carries the Dockerfile, the entrypoint, and any baked
    packages. Hashing what ros2docker actually produces means a change in
    ros2docker itself — a new build arg, a changed Dockerfile — moves the hash
    without anything here knowing it happened.
    """
    from ros2docker.api import build_context
    from ros2docker.commands import make_build_command

    fingerprint_override: dict[str, object] = dict(override or {})
    fingerprint_override["image_name"] = _FINGERPRINT_IMAGE_NAME

    digest = hashlib.sha256()
    with build_context(config_file, fingerprint_override) as context_dir:
        command = make_build_command(config_file, fingerprint_override, context_dir=context_dir)
        # The final token is the context path, a temporary directory whose name
        # differs every run. What is *in* it is hashed below instead.
        for token in command[:-1]:
            _feed(digest, token.encode("utf-8"))
        _feed_tree(digest, Path(context_dir))
    return digest.hexdigest()


def cached_reference(repository: str, fingerprint: str) -> str:
    """``<repository>:<fingerprint>`` — the only name these inputs may be published under."""
    repository = repository.strip().rstrip("/")
    if not repository:
        raise ImageCacheError(f"{IMAGE_CACHE_ENV} is set to an empty repository.")
    if ":" in repository.rsplit("/", 1)[-1]:
        raise ImageCacheError(
            f"{IMAGE_CACHE_ENV} must name a repository without a tag, got {repository!r}. "
            "The tag is the content hash and is never chosen by the caller."
        )
    return f"{repository}:{fingerprint}"


def reference_tag(reference: str) -> str:
    """The tag part of a full image reference."""
    name = reference.rsplit("/", 1)[-1]
    if ":" not in name:
        raise ImageCacheError(f"Image reference {reference!r} carries no tag.")
    return name.rsplit(":", 1)[1]


def adopt_cached_image(reference: str, image_name: str, *, attempts: int = _PULL_ATTEMPTS) -> None:
    """Make ``image_name`` resolve to the published image, pulling it if needed.

    Failure is an error rather than a fallback to building. The reference was
    derived from this machine's own build inputs, so a miss means the publish
    step did not run or did not agree with this tree — both worth a red job,
    and neither worth hiding behind a build that silently costs what this
    change exists to remove.
    """
    if not image_present(reference):
        _pull(reference, attempts=attempts)
    result = _docker(["tag", reference, image_name])
    if result.returncode != 0:
        raise ImageCacheError(f"Could not tag {reference} as {image_name}: {_message(result)}")


def image_present(reference: str) -> bool:
    """Whether the local Docker daemon already holds this image."""
    return _docker(["image", "inspect", reference], quiet=True).returncode == 0


def _pull(reference: str, *, attempts: int) -> None:
    last = ""
    for attempt in range(1, attempts + 1):
        result = _docker(["pull", reference])
        if result.returncode == 0:
            return
        last = _message(result)
        if attempt < attempts:
            time.sleep(_PULL_RETRY_SECONDS)
    raise ImageCacheError(f"Could not pull the published image {reference} after {attempts} attempts: {last}")


def _docker(args: list[str], *, quiet: bool = False) -> subprocess.CompletedProcess[str]:
    if quiet:
        return subprocess.run(
            ["docker", *args],
            text=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    return subprocess.run(["docker", *args], text=True, capture_output=True, check=False)


def _message(result: subprocess.CompletedProcess[str]) -> str:
    return ((result.stderr or "") + (result.stdout or "")).strip()


def _feed(digest: hashlib._Hash, payload: bytes) -> None:
    digest.update(payload)
    digest.update(b"\0")


def _feed_tree(digest: hashlib._Hash, root: Path) -> None:
    for path in sorted(entry for entry in root.rglob("*") if entry.is_file()):
        _feed(digest, str(path.relative_to(root)).encode("utf-8"))
        # The executable bit is part of the context: an entrypoint that loses it
        # builds an image that cannot start.
        _feed(digest, b"x" if path.stat().st_mode & 0o111 else b"-")
        _feed(digest, hashlib.sha256(path.read_bytes()).digest())
