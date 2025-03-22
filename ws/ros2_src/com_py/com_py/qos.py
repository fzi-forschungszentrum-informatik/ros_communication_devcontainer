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
    Merge 'defaults[role_name]' + 'topics[topic_name].roles[role_name]'.
    Then parse each field and return a QoSProfile.
    """
    default = qos_config.get('default', {})

    role_defaults_dict = qos_config.get('role_defaults', {})
    role_defaults = role_defaults_dict.get(role_name, {}) if isinstance(role_defaults_dict.get(role_name), dict) else {}

    topic_roles = qos_config.get('topics', {}).get(topic_name, {}).get('roles', {})
    role_override = topic_roles.get(role_name, {}) if isinstance(topic_roles.get(role_name), dict) else {}

    merged = {**default, **role_defaults, **role_override}
    logger.debug(f"[build_role_qos] for topic='{topic_name}', role='{role_name}', merged={merged}")

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
