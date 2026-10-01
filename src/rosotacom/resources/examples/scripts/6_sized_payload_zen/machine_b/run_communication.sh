#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

set -euo pipefail
set -x

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
EXAMPLE_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
CONFIG="${ROSOTACOM_CONFIG:-$EXAMPLE_ROOT/rosotacom.yaml}"

rosotacom start --rosotacom-config "$CONFIG" 6_sized_payload_zen --identity b --force
