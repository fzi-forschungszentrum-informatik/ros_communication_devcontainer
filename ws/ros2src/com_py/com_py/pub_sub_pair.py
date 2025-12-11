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
from rosidl_runtime_py.utilities import get_message
from com_py.qos import get_topic_qos

VALID_QOS_ROLES = {
    "ota_sub", "ota_pub", 
    "forward_sub", "forward_pub",
    "relay_sub", "relay_pub",
    "local_sub", "local_pub"
    # etc. define all you want
}

class PubSubPair:
    """
    A single subscription->publication pair for a single base topic.
    sub_role, pub_role must be in VALID_QOS_ROLES or we error out.
    No fallback if missing in the config => We'll still build a minimal QoS from the merged defaults or empty => standard ROS 2 defaults?
    """

    def __init__(self, node: Node,
                 base_topic_name: str,
                 sub_topic: str,
                 pub_topic: str,
                 sub_role: str,
                 pub_role: str,
                 qos_config: dict):

        self.node = node
        self.base_topic_name = base_topic_name
        self.sub_topic = sub_topic
        self.pub_topic = pub_topic
        self.sub_role = sub_role
        self.pub_role = pub_role
        self.first_msg = True
        self.is_valid = False
        self.logger = node.get_logger()
        self.publisher = None
        self.subscription = None

        # Validate roles
        if sub_role not in VALID_QOS_ROLES or pub_role not in VALID_QOS_ROLES:
            self.logger.error(f"[PubSubPair] Invalid roles: sub={sub_role}, pub={pub_role}")
            return

        # Build QoS profiles
        self.sub_qos = get_topic_qos(self.logger, qos_config, base_topic_name, sub_role)
        self.pub_qos = get_topic_qos(self.logger, qos_config, base_topic_name, pub_role)

        # Try initial setup
        self.try_initialize()

        # Log summary after creation
        self.logger.info(f"[PubSubPair] Created: base_topic='{self.base_topic_name}', "
                         f"sub_topic='{self.sub_topic}', pub_topic='{self.pub_topic}', "
                         f"sub_role='{self.sub_role}', pub_role='{self.pub_role}', "
                         f"is_valid={self.is_valid}")

    def try_initialize(self) -> bool:
        """
        Attempt to initialize publisher and subscription.
        Returns True if successful, False otherwise.
        """
        if self.is_valid:
            return True

        # Get message type from topic info
        topic_types = dict(self.node.get_topic_names_and_types())
        if self.sub_topic not in topic_types:
            return False

        msg_type_str = topic_types[self.sub_topic][0]
        msg_type = get_message(msg_type_str)
        if not msg_type:
            self.logger.error(f"[PubSubPair] Could not load message type '{msg_type_str}'!")
            return False

        # Create publisher and subscription
        self.publisher = self.node.create_publisher(msg_type, self.pub_topic, self.pub_qos)
        self.subscription = self.node.create_subscription(
            msg_type,
            self.sub_topic,
            self._callback,
            self.sub_qos
        )

        self.logger.info(f"[PubSubPair] Initialized: sub='{self.sub_topic}'(role={self.sub_role}), "
                        f"pub='{self.pub_topic}'(role={self.pub_role}), type='{msg_type_str}'")
        self.is_valid = True
        return True

    def _callback(self, msg):
        if self.first_msg:
            self.logger.info(f"[PubSubPair] FIRST msg on sub='{self.sub_topic}' => pub='{self.pub_topic}'")
            self.first_msg = False
        self.publisher.publish(msg)
