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
# \date    2025-04-03
#
#
# ---------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    DurabilityPolicy,
)
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class TFRemapper(Node):
    """
    Prefixes all frame_ids and child_frame_ids in /tf *and* /tf_static
    and republishes the result on /tf_remapped and /tf_static_remapped.
    """

    def __init__(self) -> None:
        super().__init__("tf_remapper")

        # ROS parameter: prefix added in front of every frame id
        self.declare_parameter("remap_prefix", "")
        self.remap_prefix: str = (
            self.get_parameter("remap_prefix").get_parameter_value().string_value
        )

        # QoS profiles ------------------------------------------------------
        # Dynamic TFs: default QoS is fine (depth 10, volatile)
        tf_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Static TFs: latched (= TRANSIENT_LOCAL) so new subscribers get them
        static_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Publishers --------------------------------------------------------
        self.pub_tf = self.create_publisher(TFMessage, "/tf_remapped", tf_qos)
        self.pub_static = self.create_publisher(
            TFMessage, "/tf_static_remapped", static_qos
        )

        # Subscriptions -----------------------------------------------------
        self.create_subscription(TFMessage, "/tf", self._cb_tf, tf_qos)
        self.create_subscription(TFMessage, "/tf_static", self._cb_static, static_qos)

        self.get_logger().info(
            f"TFRemapper ready – prefixing frames with '{self.remap_prefix}_'"
        )

    # ------------------------------------------------------------------ #
    # Callbacks                                                          #
    # ------------------------------------------------------------------ #

    def _cb_tf(self, msg: TFMessage) -> None:
        """Dynamic TFs – update the time stamp so it is fresh."""
        self.pub_tf.publish(self._remap(msg, update_stamp=True))

    def _cb_static(self, msg: TFMessage) -> None:
        """Static TFs – keep original (usually zero) time stamp."""
        self.pub_static.publish(self._remap(msg, update_stamp=False))

    # ------------------------------------------------------------------ #
    # Helpers                                                            #
    # ------------------------------------------------------------------ #

    def _remap(self, msg: TFMessage, *, update_stamp: bool) -> TFMessage:
        """Return a new TFMessage with all frames prefix-remapped."""
        now = self.get_clock().now().to_msg() if update_stamp else None
        remapped = []

        for tf in msg.transforms:
            new_tf = TransformStamped()
            new_tf.header = tf.header
            if update_stamp:
                new_tf.header.stamp = now

            # Only add prefix if the field is non-empty (defensive)
            new_tf.header.frame_id = (
                f"{self.remap_prefix}_{tf.header.frame_id}"
                if tf.header.frame_id
                else tf.header.frame_id
            )
            new_tf.child_frame_id = (
                f"{self.remap_prefix}_{tf.child_frame_id}"
                if tf.child_frame_id
                else tf.child_frame_id
            )

            new_tf.transform = tf.transform
            remapped.append(new_tf)

        return TFMessage(transforms=remapped)


# ---------------------------------------------------------------------- #
# Main                                                                   #
# ---------------------------------------------------------------------- #


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TFRemapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TFRemapper …")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
