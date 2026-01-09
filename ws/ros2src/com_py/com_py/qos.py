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

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
from rclpy.duration import Duration
from rosidl_runtime_py.utilities import get_message

def _normalize_topic_name(topic_name: str) -> str:
    """
    Normalize a ROS topic name for matching:
      - ensure leading '/'
      - strip trailing '/' (except for root '/')
    """
    if topic_name is None:
        return ""
    t = str(topic_name).strip()
    if t == "":
        return ""
    if not t.startswith("/"):
        t = "/" + t
    if len(t) > 1:
        t = t.rstrip("/")
    return t

def _topic_contains_base_topic(topic_name: str, base_topic: str) -> bool:
    """
    True iff base_topic matches topic_name as a path segment, e.g.:
      base='/tf' matches '/tf', '/ns/tf', '/ns/tf/restamped'
      base='/tf' does NOT match '/tf_static', '/wtf'
    """
    t = _normalize_topic_name(topic_name)
    b = _normalize_topic_name(base_topic)
    if not t or not b:
        return False
    if b == "/":
        return False  # too broad / ambiguous

    # Exact
    if t == b:
        return True

    # Base is a suffix segment: ".../<base>"
    if t.endswith(b):
        # b is normalized to start with '/', so this is already a path-segment boundary.
        return True

    # Base appears with a following path segment: ".../<base>/..."
    return (b + "/") in t

def _find_best_base_topic_match(topics_cfg: dict, topic_name: str) -> str | None:
    """
    Return the most specific (longest) configured base topic that matches topic_name.
    """
    if not isinstance(topics_cfg, dict) or not topics_cfg:
        return None
    candidates: list[str] = []
    for k in topics_cfg.keys():
        if _topic_contains_base_topic(topic_name, k):
            candidates.append(k)
    if not candidates:
        return None
    candidates.sort(key=lambda s: len(_normalize_topic_name(s)), reverse=True)
    return candidates[0]

def load_qos_config(logger, config_file: str):
    """
    Load a QoS YAML config file.
    If the file is missing or invalid, return an empty dictionary.
    """
    if not config_file:
        logger.warn("[dds_qos] No qos_config_file param => Using empty config => rclpy defaults only.")
        return {}
    if not os.path.isfile(config_file):
        logger.error(f"[dds_qos] QoS config file '{config_file}' not found => no QoS loaded.")
        return {}

    logger.info(f"[dds_qos] Loading QoS config from '{config_file}'")
    try:
        with open(config_file, 'r') as f:
            data = yaml.safe_load(f)
        if not isinstance(data, dict):
            logger.error(f"[dds_qos] Invalid QoS config: top-level is not a dict.")
            return {}
        logger.debug(f"[dds_qos] Loaded QoS config: {data}")
        return data
    except Exception as e:
        logger.error(f"[dds_qos] Failed to load QoS config '{config_file}': {e}")
        return {}

def get_topic_qos(logger, qos_config: dict, topic_name: str, role_name: str):
    """
    Build QoS for (topic_name, role_name) by merging (in this order):
      - qos_config['default']                   (global defaults)
      - qos_config['role_defaults'][role_name]  (per-role defaults)
      - qos_config['topics'][topic].<fields>    (topic defaults for ALL roles)
      - qos_config['topics'][topic]['roles'][role_name] (optional per-role overrides)
    Then parse each field and return a QoSProfile.

    Additionally supports a fallback lookup: if no exact match for topic_name exists
    in qos_config['topics'], we look for the most specific configured base topic key
    that matches as a path segment within topic_name (e.g. '/tf' for
    '/shuttle_ella/tf/restamped'). If such a fallback is used, it is always logged.
    """
    default = qos_config.get('default', {})

    role_defaults_dict = qos_config.get('role_defaults', {})
    role_defaults = role_defaults_dict.get(role_name, {}) if isinstance(role_defaults_dict.get(role_name), dict) else {}

    topics_cfg = qos_config.get('topics', {}) if isinstance(qos_config.get('topics', {}), dict) else {}
    effective_topic = topic_name
    if topic_name not in topics_cfg:
        base_match = _find_best_base_topic_match(topics_cfg, topic_name)
        if base_match is not None:
            logger.warn(
                f"[dds_qos] No exact QoS match for topic='{topic_name}'. "
                f"Falling back to base topic='{base_match}'."
            )
            effective_topic = base_match

    topic_cfg = topics_cfg.get(effective_topic, {}) if isinstance(topics_cfg.get(effective_topic, {}), dict) else {}
    # Topic-level fields apply to ALL roles (new desired interface).
    # Keep backwards compatibility: a topic may also contain a nested 'roles' dict for per-role overrides.
    topic_fields = {k: v for k, v in topic_cfg.items() if k != 'roles'}

    topic_roles = topic_cfg.get('roles', {}) if isinstance(topic_cfg.get('roles', {}), dict) else {}
    role_override = topic_roles.get(role_name, {}) if isinstance(topic_roles.get(role_name), dict) else {}

    merged = {**default, **role_defaults, **topic_fields, **role_override}
    logger.debug(
        f"[dds_qos] build_qos topic='{topic_name}' (effective='{effective_topic}'), "
        f"role='{role_name}', merged={merged}"
    )

    return dict_to_qos(logger, merged)

def dict_to_qos(logger, fields: dict) -> QoSProfile:
    """
    Convert a dictionary of QoS parameters into a QoSProfile.
    """
    qos = QoSProfile(depth=10)  # Only provided since it is required. the depth will be overridden.

    # RELIABILITY
    if 'reliability' in fields:
        rel_str = fields['reliability'].lower()
        if rel_str == 'best_effort':
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
        elif rel_str == 'reliable':
            qos.reliability = ReliabilityPolicy.RELIABLE
        else:
            raise ValueError(f"[dict_to_qos] Unknown reliability='{rel_str}'.")

    # HISTORY & DEPTH
    if 'history' in fields:
        hist_str = fields['history'].lower()
        if hist_str == 'keep_all':
            qos.history = HistoryPolicy.KEEP_ALL
        elif hist_str == 'keep_last':
            qos.history = HistoryPolicy.KEEP_LAST
        else:
            raise ValueError(f"[dict_to_qos] Unknown history='{hist_str}'.")
    if 'depth' in fields:
        depth_val = fields['depth']
        if isinstance(depth_val, int):
            qos.depth = depth_val
        else:
            raise ValueError(f"[dict_to_qos] depth must be an integer, got {depth_val}")

    # DURABILITY
    if 'durability' in fields:
        dur_str = fields['durability'].lower()
        if dur_str == 'volatile':
            qos.durability = DurabilityPolicy.VOLATILE
        elif dur_str == 'transient_local':
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        else:
            raise ValueError(f"[dict_to_qos] Unknown durability='{dur_str}'.")

    # LIVELINESS
    if 'liveliness' in fields:
        liv_str = fields['liveliness'].lower()
        if liv_str == 'automatic':
            qos.liveliness = LivelinessPolicy.AUTOMATIC
        elif liv_str == 'manual_by_topic':
            qos.liveliness = LivelinessPolicy.MANUAL_BY_TOPIC
        elif liv_str == 'manual_by_node':
            qos.liveliness = LivelinessPolicy.MANUAL_BY_NODE
        else:
            raise ValueError(f"[dict_to_qos] Unknown liveliness='{liv_str}'.")

    # DEADLINE
    if 'deadline' in fields:
        dl = float(fields['deadline'])
        if dl > 0.0:
            qos.deadline = Duration(seconds=dl)
        else:
            raise ValueError("[dict_to_qos] 'deadline' must be positive.")

    # LIFESPAN
    if 'lifespan' in fields:
        lf = float(fields['lifespan'])
        if lf > 0.0:
            qos.lifespan = Duration(seconds=lf)
        else:
            raise ValueError("[dict_to_qos] 'lifespan' must be positive.")

    # LIVELINESS_LEASE
    if 'liveliness_lease' in fields:
        lease = float(fields['liveliness_lease'])
        if lease > 0.0:
            qos.liveliness_lease_duration = Duration(seconds=lease)
        else:
            raise ValueError("[dict_to_qos] 'liveliness_lease' must be positive.")

    return qos
