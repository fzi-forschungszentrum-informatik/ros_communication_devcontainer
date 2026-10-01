# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

std_msgs/Header header
uint64 seq

# The same fixed-rate message is both a fresh probe and the response to the
# newest peer probe. On receipt, t4 is the local receive time.
bool echo_valid
uint64 echo_seq
builtin_interfaces/Time echo_t1
builtin_interfaces/Time echo_t2
builtin_interfaces/Time echo_t3
