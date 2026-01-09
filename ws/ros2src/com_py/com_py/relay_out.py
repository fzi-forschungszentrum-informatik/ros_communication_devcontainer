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
from com_py.base_bridge_relay import BaseBridgeRelay
from com_py.topic_resolution import resolve_topics

class OtaRelayOut(BaseBridgeRelay):
    """
    Relay out node: sub=, pub=
    QoS roles: sub_role=local_sub, pub_role=forward_pub
    """
    def __init__(self):
        super().__init__(
            node_name='ota_relay_out',
            node_role='relay_out',
            sub_role='local_sub',
            pub_role='forward_pub'
        )

    def declare_additional_parameters(self):
        # Semantics: if true, local side uses /{source_name}/ prefix.
        self.app_has_source_name = bool(self.declare_parameter('app_has_source_name', False).value)

    def resolve_topic_triplets(self):
        return resolve_topics(
            self.node_role,
            self.base_topic_files,
            [self.host_name],
            self.target_names,
            locally_used_source_name=self.app_has_source_name
        )

def main(args=None):
    rclpy.init(args=args)
    node = OtaRelayOut()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
