#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Strip ANSI/VT100 control sequences from stdin to stdout."""

from __future__ import annotations

import re
import sys


_ANSI_PATTERN = re.compile(
    rb"\x1b\[[0-9;?]*[ -/]*[@-~]"
    rb"|\x1b\][^\x07\x1b]*(?:\x07|\x1b\\)"
    rb"|\x1b[@-Z\\-_]"
)


def _strip(chunk: bytes) -> bytes:
    return _ANSI_PATTERN.sub(b"", chunk)


def main() -> None:
    src = sys.stdin.buffer
    dst = sys.stdout.buffer
    pending = b""
    while True:
        chunk = src.read1(4096)
        if not chunk:
            break
        pending += chunk
        cut = pending.rfind(b"\n")
        if cut < 0:
            continue
        dst.write(_strip(pending[: cut + 1]))
        dst.flush()
        pending = pending[cut + 1:]
    if pending:
        dst.write(_strip(pending))
        dst.flush()


if __name__ == "__main__":
    main()
