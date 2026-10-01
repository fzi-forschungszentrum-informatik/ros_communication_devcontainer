#!/bin/bash
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

# Wait until the topic is available and has a known type
until topic_info="$(ros2 topic info /chatter 2>/dev/null)" && grep -q "Type:" <<<"$topic_info"; do
    echo "[INFO] Waiting for /chatter to become available..."
    sleep 1
done

# Now start echoing the topic
exec ros2 topic echo /chatter
