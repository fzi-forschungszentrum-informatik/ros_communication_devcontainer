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
# \date    2024-11-13
#
#
# ---------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from com_py.topic_resolution import resolve_topics
from com_py.qos import load_qos_config
from com_py.pub_sub_pair import PubSubPair

class BaseBridgeRelay(Node):
    """Base class for bridge and relay nodes with common functionality."""

    def __init__(self, node_name: str, node_role: str, sub_role: str, pub_role: str):
        super().__init__(node_name)
        
        self.node_role = node_role
        self.sub_role = sub_role
        self.pub_role = pub_role
        self.pairs = []

        # Common parameters
        self.base_topic_files = self.declare_parameter('base_topic_files', rclpy.Parameter.Type.STRING_ARRAY).value
        self.host_name = self.declare_parameter('host_name', '').value
        self.qos_config_file = self.declare_parameter('qos_config_file', '').value

        # Optional parameters with defaults
        self.source_names = self.declare_parameter('source_names', rclpy.Parameter.Type.STRING_ARRAY).value or []
        self.target_names = self.declare_parameter('target_names', rclpy.Parameter.Type.STRING_ARRAY).value or []
        self.explicitly_adressed = bool(self.declare_parameter('explicitly_adressed', False).value)
        self.locally_used_source_name = bool(self.declare_parameter('locally_used_source_name', False).value)

        # Initialize node
        self.initialize_node()
        
        # Create timer for periodic refresh of invalid pairs
        self.create_timer(5.0, self.refresh_pairs)

    def initialize_node(self):
        """Initialize node with resolved topics and QoS configuration."""
        try:
            triplets = self.resolve_topic_triplets()
        except (ValueError, FileNotFoundError) as e:
            self.get_logger().error(f"[{self.node_role}] resolve_topics failed: {e}")
            return

        self.qos_config = load_qos_config(self.get_logger(), self.qos_config_file)
        
        for (sub_t, pub_t, base_t) in triplets:
            pair = PubSubPair(
                node=self,
                base_topic_name=base_t,
                sub_topic=sub_t,
                pub_topic=pub_t,
                sub_role=self.sub_role,
                pub_role=self.pub_role,
                qos_config=self.qos_config
            )
            self.pairs.append(pair)

        self.get_logger().info(f"[{self.node_role}] Created {len(self.pairs)} pairs.")

    def resolve_topic_triplets(self):
        """Override in derived classes to implement specific topic resolution logic."""
        raise NotImplementedError

    def refresh_pairs(self):
        """Attempt to initialize any invalid pairs."""
        for pair in self.pairs:
            if not pair.is_valid:
                pair.try_initialize() 