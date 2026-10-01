#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2024-11-13
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

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
#    * Neither the name of the copyright holder nor the names of its
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
# \date    2024-11-13
#
#
# ---------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from com_py.topic_resolution import resolve_topics
from com_py.qos import load_qos_config, load_topic_types
from com_py.pub_sub_pair import PubSubPair
from com_py.pair_management import PairRefreshMixin

class BaseBridgeRelay(Node, PairRefreshMixin):
    """Base class for bridge and relay nodes with common functionality."""

    def __init__(self, node_name: str, node_role: str, sub_role: str, pub_role: str):
        super().__init__(node_name)

        self.node_role = node_role
        self.sub_role = sub_role
        self.pub_role = pub_role
        self.pairs = []

        # Common parameters
        self.base_topic_files = self.declare_parameter('base_topic_files', rclpy.Parameter.Type.STRING_ARRAY).value
        self.host_name = self.declare_parameter('local_name', '').value
        self.qos_config_file = self.declare_parameter('qos_config_file', '').value
        # `<stage topic>: <message type>` for every stage of this session, so a
        # pair can create its endpoints without waiting for somebody else to
        # create the matching one first. Empty is allowed: the graph is still
        # consulted, and was the only source before this existed.
        self.topic_types_file = self.declare_parameter('topic_types_file', '').value

        # Optional parameters with defaults
        self.source_names = self.declare_optional_string_array('source_names')
        self.target_names = self.declare_optional_string_array('target_names')

        # Derived classes can declare additional parameters here (must happen before initialize_node()).
        self.declare_additional_parameters()

        # Initialize node
        self.initialize_node()

        # Create timer for periodic refresh of invalid pairs
        self.init_pair_refresh(period_s=5.0, log_prefix=self.node_role)

    def declare_optional_string_array(self, name: str) -> list:
        """Declare an optional STRING_ARRAY that reads as [] rather than as unset.

        Declaring a type without a value leaves the parameter
        PARAMETER_NOT_SET. This node never notices — it reads the value once,
        here, and an unset parameter simply yields None. Everything *else*
        notices: reading a not-set parameter raises, and rclpy's parameter
        service logs one warning per read against this node,

            [WARN] [ota_relay_out]: Failed to get parameters: The parameter
            'source_names' is not initialized: source_names

        which anything that enumerates parameters then produces on a timer. A
        Foxglove bridge with a parameter panel open put 107 of those lines into
        a single pane in one session — the kind of noise that hides the message
        somebody actually needs to see.

        A plain default cannot express this: `declare_parameter(name, [])`
        infers the type from the value, and an empty list satisfies the
        BYTE_ARRAY test first (`all(...)` is true for an empty sequence). The
        parameter would be typed BYTE_ARRAY and the real string array refused
        later with "Wrong parameter type, expected '5' got '9'". Declaring the
        type and then setting the empty value keeps it STRING_ARRAY, so a wrong
        type is still rejected and an override still wins.
        """
        self.declare_parameter(name, rclpy.Parameter.Type.STRING_ARRAY)
        if self.get_parameter_or(name).type_ == rclpy.Parameter.Type.NOT_SET:
            self.set_parameters([rclpy.Parameter(name, rclpy.Parameter.Type.STRING_ARRAY, [])])
        return self.get_parameter(name).value

    def initialize_node(self):
        """Initialize node with resolved topics and QoS configuration."""
        try:
            triplets = self.resolve_topic_triplets()
        except (ValueError, FileNotFoundError) as e:
            self.get_logger().error(f"[{self.node_role}] resolve_topics failed: {e}")
            return

        self.qos_config = load_qos_config(self.get_logger(), self.qos_config_file)
        self.topic_types = load_topic_types(self.get_logger(), self.topic_types_file)

        for (sub_t, pub_t, base_t) in triplets:
            pair = PubSubPair(
                node=self,
                base_topic_name=base_t,
                sub_topic=sub_t,
                pub_topic=pub_t,
                sub_role=self.sub_role,
                pub_role=self.pub_role,
                qos_config=self.qos_config,
                topic_types=self.topic_types
            )
            self.pairs.append(pair)

        self.get_logger().info(f"[{self.node_role}] Created {len(self.pairs)} pairs.")

    def resolve_topic_triplets(self):
        """Override in derived classes to implement specific topic resolution logic."""
        raise NotImplementedError

    def declare_additional_parameters(self):
        """Hook for derived classes to declare extra parameters before topic resolution."""
        return
