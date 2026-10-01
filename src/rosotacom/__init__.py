# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""rosotacom package."""

from __future__ import annotations

from importlib.metadata import PackageNotFoundError, packages_distributions, version
from typing import Any


def _package_version() -> str:
    """Version of the distribution that ships this package.

    The distribution name is looked up, not hardcoded. This source is published
    under more than one distribution name, so
    a hardcoded name resolves under one of them and silently degrades to
    "0+unknown" under the other — which is what happened up to 2.3: every
    install under the other name misreported its own version, so `--version` could
    not be compared against a pin, and the OTA source bundle wrote an invalid
    PEP 440 version into its generated metadata.
    """
    for distribution in packages_distributions().get(__name__, ()):
        try:
            return version(distribution)
        except PackageNotFoundError:
            continue
    return "0+unknown"


__version__ = _package_version()


def __getattr__(name: str) -> Any:
    """Lazily expose the historical root-module CLI attributes."""

    from . import cli

    try:
        return getattr(cli, name)
    except AttributeError as exc:
        raise AttributeError(f"module 'rosotacom' has no attribute {name!r}") from exc


__all__ = ["__version__"]
