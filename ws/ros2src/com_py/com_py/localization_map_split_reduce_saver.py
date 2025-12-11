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
# \date    2025-05-16
#
# ---------------------------------------------------------------------

"""ROS 2 node that lowers, vertically splits, downsamples and stores an
OccupancyGrid arriving on `/localization/map`.

Parameters (set via `--ros-args -p …`):
  • **num_slices** (int, default 2) – number of vertical slices.
  • **downsample_factor** (int, default 2) – block size used for averaging.
  • **output_file** (string) – path for the output pickle (default
    `<script>/map_data/map_processed.pkl`).

Only the first map message is processed. The node shuts itself down after
writing the file.
"""

from __future__ import annotations

import os
import sys
import time
import copy
import pickle
import array

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def human_readable_size(bytes_: int) -> str:  # noqa: D401, ANN001 (simple util)
    """Return `bytes_` as human‑readable string (B, KB, MB)."""
    if bytes_ < 1024:
        return f"{bytes_} B"
    if bytes_ < 1024 ** 2:
        return f"{bytes_ / 1024:.2f} KB"
    return f"{bytes_ / 1024 ** 2:.2f} MB"


# ---------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------

class MapProcessorReducer(Node):
    """Lower Z, split, downsample then pickle the resulting slices."""

    def __init__(self) -> None:
        super().__init__('map_processor_reducer')

        # ---------------- Parameters ----------------
        self.declare_parameter('num_slices', 2)
        self.declare_parameter('downsample_factor', 2)
        default_out = '/ws/map_processed.pkl'
        self.declare_parameter('output_file', default_out)

        self.num_slices: int = max(
            1,
            self.get_parameter('num_slices')
            .get_parameter_value().integer_value,
        )
        self.factor: int = max(
            1,
            self.get_parameter('downsample_factor')
            .get_parameter_value().integer_value,
        )
        self.output_file: str = (
            self.get_parameter('output_file')
            .get_parameter_value().string_value
        )

        # --------------- Subscription --------------
        self._got_map = False
        self.create_subscription(
            OccupancyGrid,
            '/localization/map',
            self._map_callback,
            1,
        )

        self.get_logger().info(
            f"Waiting for /localization/map… (num_slices={self.num_slices}, "
            f"factor={self.factor})"
        )

    # ------------------------------------------------
    # Callback
    # ------------------------------------------------

    def _map_callback(self, msg: OccupancyGrid) -> None:  # noqa: D401
        if self._got_map:
            return  # ignore further messages
        self._got_map = True

        self.get_logger().info(
            f"Received map: {msg.info.width}×{msg.info.height}, "
            f"len(data)={len(msg.data)}"
        )
        tic = time.time()

        # 1) Lower map by 1 m along Z
        msg.info.origin.position.z -= 1.0

        # 2) Split → 3) Downsample
        reduced_slices = [
            self._downsample_slice(slice_msg, self.factor, i + 1)
            for i, slice_msg in enumerate(self._split_grid(msg, self.num_slices))
        ]

        # 4) Store
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        with open(self.output_file, 'wb') as fp:
            pickle.dump(reduced_slices, fp)
        self.get_logger().info(
            f"Saved {len(reduced_slices)} slice(s) to '{self.output_file}' in "
            f"{time.time() - tic:.3f} s. Shutting down."
        )

        rclpy.shutdown()

    # ------------------------------------------------
    # Helpers
    # ------------------------------------------------

    def _split_grid(self, msg: OccupancyGrid, n: int) -> list[OccupancyGrid]:
        """Return *n* vertical sub‑grids of *msg*."""
        ow, oh, res = msg.info.width, msg.info.height, msg.info.resolution
        data = msg.data

        if ow % n:
            self.get_logger().warn(
                f"Width {ow} not divisible by num_slices={n}. Final slice wider."
            )

        slice_w = ow // n
        slices: list[OccupancyGrid] = []

        for i in range(n):
            start_c = i * slice_w
            end_c = (i + 1) * slice_w if i < n - 1 else ow
            width = end_c - start_c

            new_data = [0] * (width * oh)
            for r in range(oh):
                base_old = r * ow
                base_new = r * width
                for c in range(start_c, end_c):
                    new_data[base_new + (c - start_c)] = data[base_old + c]

            sub = OccupancyGrid()
            sub.header = copy.deepcopy(msg.header)
            sub.info.resolution = res
            sub.info.width = width
            sub.info.height = oh
            sub.info.origin = copy.deepcopy(msg.info.origin)
            sub.info.origin.position.x += start_c * res
            sub.data = new_data
            slices.append(sub)

            self.get_logger().info(
                f"Slice {i + 1}/{n}: {width}×{oh}, len={len(new_data)}"
            )

        return slices

    def _downsample_slice(self, msg: OccupancyGrid, f: int, idx: int) -> OccupancyGrid:
        ow, oh, res = msg.info.width, msg.info.height, msg.info.resolution
        data = msg.data
        self.get_logger().info(
            f"Slice #{idx} BEFORE: {ow}×{oh}, size≈{human_readable_size(sys.getsizeof(data))}"
        )

        nw, nh = ow // f, oh // f
        new_res = res * f
        tgt = [0] * (nw * nh)

        for nr in range(nh):
            for nc in range(nw):
                accum = valid = 0
                for dr in range(f):
                    for dc in range(f):
                        v = data[(nr * f + dr) * ow + (nc * f + dc)]
                        if v >= 0:
                            accum += v
                            valid += 1
                tgt[nr * nw + nc] = -1 if valid == 0 else accum // valid

        new_data = array.array('b', tgt)
        self.get_logger().info(
            f"Slice #{idx} AFTER:  {nw}×{nh}, size≈{human_readable_size(sys.getsizeof(new_data))}"
        )

        reduced = OccupancyGrid()
        reduced.header = copy.deepcopy(msg.header)
        reduced.info.resolution = new_res
        reduced.info.width = nw
        reduced.info.height = nh
        reduced.info.origin = copy.deepcopy(msg.info.origin)
        reduced.data = new_data
        return reduced


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def main(args: list[str] | None = None) -> None:  # noqa: D401
    rclpy.init(args=args)
    node = MapProcessorReducer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
