#!/usr/bin/env python3

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

def infer_topic_types(node: Node):
    """
    Retrieves all available topics and their corresponding message types dynamically.
    Returns a dictionary {topic_name: topic_type}.
    """
    topic_map = {}
    all_topics = node.get_topic_names_and_types()
    for topic_name, type_list in all_topics:
        if type_list:
            topic_map[topic_name] = type_list[0]  # Assume the first type if multiple exist
    return topic_map

def get_topic_type(qos_config: dict, topic_name: str, node: Node):
    """
    Tries to retrieve the topic type from QoS config.
    If not available, it attempts to infer it dynamically.
    """
    topics = qos_config.get('topics', {})
    if topic_name in topics and 'type' in topics[topic_name]:
        return topics[topic_name]['type']

    # Fallback: Try to infer the type dynamically
    inferred_types = infer_topic_types(node)
    topic_type = inferred_types.get(topic_name, None)

    if topic_type is None:
        node.get_logger().warn(f"[dds_qos] Could not determine type for topic '{topic_name}', skipping QoS config.")
    return topic_type

def build_role_qos(logger, qos_config: dict, topic_name: str, role_name: str):
    """
    Merge 'defaults[role_name]' + 'topics[topic_name].roles[role_name]'.
    Then parse each field and return a QoSProfile.
    """
    defaults = qos_config.get('defaults', {})
    role_defaults = defaults.get(role_name, {}) if isinstance(defaults.get(role_name), dict) else {}

    topic_roles = qos_config.get('topics', {}).get(topic_name, {}).get('roles', {})
    role_override = topic_roles.get(role_name, {}) if isinstance(topic_roles.get(role_name), dict) else {}

    merged = {**role_defaults, **role_override}
    logger.debug(f"[build_role_qos] for topic='{topic_name}', role='{role_name}', merged={merged}")

    return dict_to_qos(logger, merged)

def dict_to_qos(logger, fields: dict) -> QoSProfile:
    """
    Convert a dictionary of QoS parameters into a QoSProfile.
    """
    qos = QoSProfile(depth=10)  # Default depth, may be overridden.

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
