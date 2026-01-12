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
# ---------------------------------------------------------------------

import math
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage, Image

from com_py.qos import get_topic_qos, load_qos_config


# Presets for max pixel budget (given as WxH for readability)
RESOLUTION_PRESETS: Dict[str, str] = {
    # -------------------------------------------------
    # 4:3 (klassisch, VGA-Familie)
    # -------------------------------------------------
    "qqvga": "160x120",        # 4:3  | max_pixels = 19_200
    "qvga": "320x240",         # 4:3  | max_pixels = 76_800
    "480x360": "480x360",      # 4:3  | max_pixels = 172_800
    "vga": "640x480",          # 4:3  | max_pixels = 307_200
    "svga": "800x600",         # 4:3  | max_pixels = 480_000

    # -------------------------------------------------
    # 16:10 (small custom resolutions)
    # -------------------------------------------------
    "qqvga_16_10": "160x100",  # 16:10 | max_pixels = 16_000
    "qvga_16_10": "320x200",   # 16:10 | max_pixels = 64_000
    "480x300": "480x300",      # 16:10 | max_pixels = 144_000
    "mcga": "640x400",         # 16:10 | max_pixels = 256_000
    "wvga_16_10": "800x500",   # 16:10 | max_pixels = 400_000

    # -------------------------------------------------
    # 16:9
    # -------------------------------------------------
    "hd_720p": "1280x720",     # 16:9  | max_pixels = 921_600
    "hd": "1366x768",          # ~16:9 | max_pixels = 1_049_088
    "fhd": "1920x1080",        # 16:9  | max_pixels = 2_073_600
    "qhd": "2560x1440",        # 16:9  | max_pixels = 3_686_400
    "res_4k": "3840x2160",     # 16:9  | max_pixels = 8_294_400
    "res_8k": "7680x4320",     # 16:9  | max_pixels = 33_177_600

    # -------------------------------------------------
    # 16:10
    # -------------------------------------------------
    "wxga": "1280x800",        # 16:10 | max_pixels = 1_024_000
    "wxga_plus": "1440x900",   # 16:10 | max_pixels = 1_296_000
    "wsxga_plus": "1680x1050", # 16:10 | max_pixels = 1_764_000
    "wuxga": "1920x1200",      # 16:10 | max_pixels = 2_304_000
    "wqxga": "2560x1600",      # 16:10 | max_pixels = 4_096_000
    "res_4k_16_10": "3840x2400", # 16:10 | max_pixels = 9_216_000
    "res_8k_16_10": "7680x4800", # 16:10 | max_pixels = 36_864_000
}

def _parse_wh(raw: str) -> Tuple[int, int]:
    s = raw.strip().lower().replace(" ", "")
    if "x" not in s:
        raise ValueError(f"Cannot parse WxH from '{raw}'")
    a, b = s.split("x", 1)
    if not a.isdigit() or not b.isdigit():
        raise ValueError(f"Non-integer WxH in '{raw}'")
    w, h = int(a), int(b)
    if w <= 0 or h <= 0:
        raise ValueError(f"Invalid WxH '{raw}'")
    return w, h


def _encoding_from_cv_image(image_np: np.ndarray) -> str:
    """Best-effort encoding guess for OpenCV decoded images."""
    if image_np.ndim == 2:
        return "mono8"
    if image_np.ndim == 3 and image_np.shape[2] == 3:
        return "bgr8"
    if image_np.ndim == 3 and image_np.shape[2] == 4:
        return "bgra8"
    return "bgr8"


def _resize_interpolation(src_w: int, src_h: int, dst_w: int, dst_h: int) -> int:
    """Pick a reasonable interpolation depending on up/down-scaling."""
    if dst_w <= src_w and dst_h <= src_h:
        return cv2.INTER_AREA
    return cv2.INTER_LINEAR


def _resolve_max_pixels_preset(
    *,
    logger,
    param: Parameter,
) -> Tuple[str, int, Tuple[int, int]]:
    """
    Resolve required 'max_pixels_preset' into:
      (preset_name, max_pixels, (preset_w, preset_h))

    Only preset names are allowed.
    """
    if param.type_ == Parameter.Type.NOT_SET or param.value is None:
        raise RuntimeError(
            "Missing required parameter 'max_pixels_preset'. "
            "It must be one of: " + ", ".join(sorted(RESOLUTION_PRESETS.keys()))
        )

    preset = str(param.value).strip()
    if preset == "" or preset not in RESOLUTION_PRESETS:
        raise RuntimeError(
            "Invalid 'max_pixels_preset'. It must be one of: "
            + ", ".join(sorted(RESOLUTION_PRESETS.keys()))
            + f". Got: {param.value!r}"
        )

    w, h = _parse_wh(RESOLUTION_PRESETS[preset])
    max_pixels = w * h

    logger.info(
        f"Resolved max pixel budget preset: '{preset}' -> {w}x{h} = {max_pixels} pixels."
    )
    return preset, max_pixels, (w, h)


class ImagePixelCapper(Node):
    """
    Cap images to satisfy a max pixel budget, while preserving aspect ratio.

    Interface:
      - max_pixels_preset (required): one of RESOLUTION_PRESETS keys
      - only_downscale (default True): never upscale
      - topics (string array): list of topics to subscribe to
      - output_suffix (default '/{max_pixels_preset}'): appended to each input topic for output
      - qos_config_file/sub_role/pub_role: QoS role mapping
    """

    def __init__(self):
        super().__init__("image_pixel_capper")
        self.bridge = CvBridge()
        self.pubs = {}

        # Logging: once per output topic (first seen frame)
        self._logged_topics: set[str] = set()
        self._frame_count: Dict[str, int] = {}

        # Required preset (name only)
        self.declare_parameter("max_pixels_preset", Parameter.Type.STRING)
        self.only_downscale = self.declare_parameter("only_downscale", True).value

        # Topics + naming
        self.declare_parameter("topics", rclpy.Parameter.Type.STRING_ARRAY)

        # QoS config + roles (configured via qos.py YAML)
        self.qos_config_file = self.declare_parameter("qos_config_file", "").value
        self.sub_role = "image_reduce_sub"
        self.pub_role = "image_reduce_pub"
        self.qos_config = load_qos_config(self.get_logger(), self.qos_config_file)

        # Resolve max pixel budget
        self.max_pixels_preset, self.max_pixels, self.max_pixels_preset_wh = _resolve_max_pixels_preset(
            logger=self.get_logger(),
            param=self.get_parameter("max_pixels_preset"),
        )

        self.output_suffix = self.declare_parameter("output_suffix", f"/{self.max_pixels_preset}").value

        topics_param = self.get_parameter("topics").value
        if not isinstance(topics_param, list):
            topics_param = [topics_param]
        self.topics = topics_param

        self.get_logger().info(
            "Initialized ImagePixelCapper with "
            f"max_pixels_preset='{self.max_pixels_preset}' "
            f"({self.max_pixels_preset_wh[0]}x{self.max_pixels_preset_wh[1]} -> {self.max_pixels} px), "
            f"only_downscale={self.only_downscale}, "
            f"output_suffix='{self.output_suffix}', "
            f"qos_config_file='{self.qos_config_file}', "
            f"roles(sub={self.sub_role}, pub={self.pub_role})"
        )

        # Create subscribers and publishers for each topic
        for topic in self.topics:
            out_topic = f"{topic}{self.output_suffix}"

            pub_qos = get_topic_qos(self.get_logger(), self.qos_config, out_topic, self.pub_role)
            sub_qos = get_topic_qos(self.get_logger(), self.qos_config, topic, self.sub_role)

            # Always publish raw Image. Compression is expected to be handled by transport.
            self.pubs[out_topic] = self.create_publisher(Image, out_topic, pub_qos)
            self._frame_count[out_topic] = 0

            # Heuristic: if 'compressed' appears in topic, assume CompressedImage; else raw Image.
            msg_type = CompressedImage if "compressed" in topic else Image

            self.create_subscription(
                msg_type,
                topic,
                lambda msg, ot=out_topic: self.callback(msg, ot),
                sub_qos,
            )

            self.get_logger().info(
                f"Subscribed: {topic} ({msg_type.__name__}) -> publishing pixel-capped Image: {out_topic}"
            )

    def _log_once(
        self,
        out_topic: str,
        *,
        in_w: int,
        in_h: int,
        in_encoding: str,
        out_w: int,
        out_h: int,
        out_encoding: str,
        is_noop: bool,
        scale: float,
        decode_note: Optional[str],
    ) -> None:
        if out_topic in self._logged_topics:
            return
        self._logged_topics.add(out_topic)

        in_px = in_w * in_h
        out_px = out_w * out_h
        reduction_pct = (1.0 - (out_px / float(in_px))) * 100.0 if in_px > 0 else 0.0

        preset_w, preset_h = self.max_pixels_preset_wh
        preset_px = self.max_pixels

        action = "NO-OP (within budget)" if is_noop else "PIXEL-CAPPED"

        msg = (
            f"[{out_topic}] {action}. "
            f"Budget: preset='{self.max_pixels_preset}' ({preset_w}x{preset_h} => {preset_px} px). "
            f"Input: {in_w}x{in_h} ({in_px} px), encoding='{in_encoding}'. "
            f"Output: {out_w}x{out_h} ({out_px} px), encoding='{out_encoding}'. "
            f"Scale={scale:.4f}, pixel reduction={reduction_pct:.1f}%. "
        )
        if decode_note:
            msg += decode_note

        self.get_logger().info(msg.strip())

    def callback(self, data, out_topic: str):
        try:
            decode_note = None

            # Convert ROS message to OpenCV image + decide output encoding
            if isinstance(data, CompressedImage):
                np_arr = np.frombuffer(data.data, np.uint8)
                image_np = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
                if image_np is None:
                    raise RuntimeError("Failed to decode CompressedImage")

                in_h, in_w = image_np.shape[:2]
                in_encoding = "(compressed)"
                out_encoding = _encoding_from_cv_image(image_np)

                decode_note = (
                    "Note: input was CompressedImage; output encoding is derived from decoded pixels "
                    f"('{out_encoding}') and is not guaranteed to match the original pre-compression format."
                )

            elif isinstance(data, Image):
                image_np = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
                in_h, in_w = image_np.shape[:2]
                in_encoding = str(data.encoding)
                out_encoding = str(data.encoding)  # hardcoded: keep original encoding for Image

            else:
                raise ValueError("Unsupported message type received")

            in_px = in_w * in_h
            if in_px <= 0:
                raise RuntimeError("Invalid source image size (0 pixels).")

            # Compute scale to satisfy pixel budget while preserving aspect ratio
            scale = math.sqrt(self.max_pixels / float(in_px))
            if self.only_downscale:
                scale = min(1.0, scale)

            # Determine output size
            if scale >= 1.0:
                out_w, out_h = in_w, in_h
                is_noop = True
            else:
                out_w = max(1, int(round(in_w * scale)))
                out_h = max(1, int(round(in_h * scale)))
                is_noop = False

            # Log once per topic (first seen frame)
            self._log_once(
                out_topic,
                in_w=in_w,
                in_h=in_h,
                in_encoding=in_encoding,
                out_w=out_w,
                out_h=out_h,
                out_encoding=out_encoding,
                is_noop=is_noop,
                scale=scale,
                decode_note=decode_note,
            )

            # Fast path: no resize needed and raw Image input -> republish unchanged
            if is_noop and isinstance(data, Image):
                self.pubs[out_topic].publish(data)
                return

            # Resize if needed
            if not is_noop:
                interp = _resize_interpolation(in_w, in_h, out_w, out_h)
                image_np = cv2.resize(image_np, (out_w, out_h), interpolation=interp)

            # Publish raw Image
            out_msg = self.bridge.cv2_to_imgmsg(image_np, encoding=out_encoding)
            out_msg.header = data.header
            self.pubs[out_topic].publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"[{out_topic}] Failed to pixel-cap/publish: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImagePixelCapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
