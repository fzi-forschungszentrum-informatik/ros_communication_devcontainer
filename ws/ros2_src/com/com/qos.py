#!/usr/bin/env python3

import os
import yaml
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
from rclpy.duration import Duration

def load_qos_config(logger, config_file: str):
    """
    Load a layered QoS YAML file of the form:
      defaults:
        <role>:
          reliability: "best_effort" or "reliable"
          depth: <int>
          ... (deadline, lifespan, etc.)
      topics:
        "/some_topic":
          type: "std_msgs/msg/String"
          roles:
            <role>:
              reliability: ...
              depth: ...
              ...
    Return the raw dict or empty if errors. 
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

def get_topic_type(qos_config: dict, topic_name: str):
    """
    Return the 'type' field from config['topics'][topic_name]. If missing, return None.
    """
    topics = qos_config.get('topics', {})
    tentry = topics.get(topic_name, {})
    return tentry.get('type', None)

def build_role_qos(logger, qos_config: dict, topic_name: str, role_name: str):
    """
    Merge 'defaults[role_name]' + 'topics[topic_name].roles[role_name]'.
    Then parse each field. If user sets an unknown reliability/durability, raise error.
    If user doesn't set it, do NOT override => let rclpy default stand.

    Return a QoSProfile with partial fields set and others left to default.
    """
    defaults = qos_config.get('defaults', {})
    role_defaults = defaults.get(role_name, {})
    if not isinstance(role_defaults, dict):
        role_defaults = {}

    topic_overrides = qos_config.get('topics', {}).get(topic_name, {}).get('roles', {})
    role_override = topic_overrides.get(role_name, {})
    if not isinstance(role_override, dict):
        role_override = {}

    merged = dict(role_defaults)
    for k,v in role_override.items():
        merged[k] = v

    logger.debug(f"[build_role_qos] for topic='{topic_name}', role='{role_name}', merged={merged}")

    return dict_to_qos(logger, merged)

def dict_to_qos(logger, fields: dict) -> QoSProfile:
    """
    Build a QoSProfile from the partial 'fields' dict. 
    If a field is not present, we do NOT set it => rclpy keeps its default.

    Strict checks for reliability, durability, etc.: if user sets an unknown value, raise ValueError.
    """

    # We'll construct a QoSProfile with default constructor => everything is default.
    # Then we'll selectively set each field if user provided it, with strict checking.

    qos = QoSProfile(depth=10) # hofix: provide default depth since it is required. Might be overridden later.

    # RELIABILITY
    if 'reliability' in fields:
        rel_str = fields['reliability'].lower()
        if rel_str == 'best_effort':
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
        elif rel_str == 'reliable':
            qos.reliability = ReliabilityPolicy.RELIABLE
        else:
            raise ValueError(f"[dict_to_qos] Unknown reliability='{rel_str}'. Must be 'best_effort' or 'reliable'.")

    # HISTORY & DEPTH
    if 'history' in fields:
        hist_str = fields['history'].lower()
        if hist_str == 'keep_all':
            qos.history = HistoryPolicy.KEEP_ALL
        elif hist_str == 'keep_last':
            qos.history = HistoryPolicy.KEEP_LAST
        else:
            raise ValueError(f"[dict_to_qos] Unknown history='{hist_str}'. Must be 'keep_all' or 'keep_last'.")
    if 'depth' in fields:
        depth_val = fields['depth']
        if not isinstance(depth_val, int):
            raise ValueError(f"[dict_to_qos] depth must be an integer, got {depth_val}")
        qos.depth = depth_val

    # DURABILITY
    if 'durability' in fields:
        dur_str = fields['durability'].lower()
        if dur_str == 'volatile':
            qos.durability = DurabilityPolicy.VOLATILE
        elif dur_str == 'transient_local':
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        else:
            raise ValueError(f"[dict_to_qos] Unknown durability='{dur_str}'. Must be 'volatile' or 'transient_local'.")

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
            raise ValueError(f"[dict_to_qos] Unknown liveliness='{liv_str}'. Must be 'automatic','manual_by_topic','manual_by_node'.")

    # DEADLINE
    if 'deadline' in fields:
        dl = float(fields['deadline'])
        if dl <= 0.0:
            raise ValueError("[dict_to_qos] 'deadline' must be positive if set.")
        qos.deadline = Duration(seconds=dl)

    # LIFESPAN
    if 'lifespan' in fields:
        lf = float(fields['lifespan'])
        if lf <= 0.0:
            raise ValueError("[dict_to_qos] 'lifespan' must be positive if set.")
        qos.lifespan = Duration(seconds=lf)

    # LIVELINESS_LEASE
    if 'liveliness_lease' in fields:
        lease = float(fields['liveliness_lease'])
        if lease <= 0.0:
            raise ValueError("[dict_to_qos] 'liveliness_lease' must be positive if set.")
        qos.liveliness_lease_duration = Duration(seconds=lease)

    return qos
