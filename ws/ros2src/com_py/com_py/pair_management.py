#!/usr/bin/env python3

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
# 
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
# 
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.                                                
# -- END LICENSE BLOCK ------------------------------------------------
#
# ---------------------------------------------------------------------
# !\file
#
# \author  Martin Gontscharow <gontscharow@fzi.de>
# \date    2026-01-09
#
# ---------------------------------------------------------------------

from __future__ import annotations

"""Shared utilities for nodes that manage a list of PubSub-like pairs.

Several nodes (bridge/relay, frame_bridge, etc.) create a list of pairs that may
initially be invalid because the ROS graph does not yet contain the topics.
This mixin provides a standard periodic re-initialization loop with consistent
logging.
"""


class PairRefreshMixin:
    """Mixin for rclpy Nodes that maintain `self.pairs` with `try_initialize()`."""

    def init_pair_refresh(
        self,
        *,
        period_s: float = 5.0,
        pairs_attr: str = "pairs",
        log_prefix: str = "PairRefresh",
    ) -> None:
        self._pair_refresh_pairs_attr = pairs_attr
        self._pair_refresh_log_prefix = log_prefix
        # Keep handle in case callers want to cancel it later.
        self._pair_refresh_timer = self.create_timer(period_s, self._pair_refresh_tick)

    def _pair_refresh_tick(self) -> None:
        pairs = getattr(self, getattr(self, "_pair_refresh_pairs_attr", "pairs"), [])

        pending = 0
        initialized_now = 0

        for pair in pairs:
            if not getattr(pair, "is_valid", False):
                pending += 1
                try:
                    if pair.try_initialize():
                        initialized_now += 1
                except Exception as exc:  # defensive: never crash the timer
                    self.get_logger().error(
                        f"[{getattr(self, '_pair_refresh_log_prefix', 'PairRefresh')}] "
                        f"pair.try_initialize() failed: {exc}"
                    )

        if initialized_now > 0:
            self.get_logger().info(
                f"[{getattr(self, '_pair_refresh_log_prefix', 'PairRefresh')}] "
                f"Initialized {initialized_now} pair(s). Pending={max(pending - initialized_now, 0)}"
            )
