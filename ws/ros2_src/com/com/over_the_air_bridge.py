#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
from rclpy.duration import Duration
import rclpy.parameter
import yaml
import os

class OverTheAirBridge(Node):
    """
    A single node that:
      - Creates 'incoming' relay pairs: (OTA subscriber -> local publisher)
      - Creates 'outgoing' relay pairs: (local subscriber -> OTA publisher)
      - Loads a YAML QoS config with sections for [over_the_air_subscriber, local_publisher,
        over_the_air_publisher, local_subscriber].
      - Enhanced with debug & info logging for better visibility.
    """

    def __init__(self):
        super().__init__('over_the_air_bridge')

        # --------------------------------------------------------------------
        # 1) Declare & Read Parameters
        # --------------------------------------------------------------------
        # Using Type.STRING_ARRAY ensures we can pass arrays from the CLI
        self.declare_parameter('topic_list_paths_incoming', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('topic_list_prefixes_incoming', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('remove_target_prefix_incoming', '')

        self.declare_parameter('topic_list_paths_outgoing', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('topic_list_prefixes_outgoing', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('target_prefix_outgoing', '')

        self.declare_parameter('qos_config_file', '')
        self.declare_parameter('sync_suffix', '_sync')

        # Retrieve param values
        self.topic_list_paths_incoming = self.get_array_param('topic_list_paths_incoming')
        self.topic_list_prefixes_incoming = self.get_array_param('topic_list_prefixes_incoming')
        self.remove_target_prefix_incoming = self.get_parameter('remove_target_prefix_incoming').value

        self.topic_list_paths_outgoing = self.get_array_param('topic_list_paths_outgoing')
        self.topic_list_prefixes_outgoing = self.get_array_param('topic_list_prefixes_outgoing')
        self.target_prefix_outgoing = self.get_parameter('target_prefix_outgoing').value

        self.sync_suffix = self.get_parameter('sync_suffix').value
        self.qos_config_file = self.get_parameter('qos_config_file').value
        self.qos_config_dict = self._load_qos_config(self.qos_config_file)

        self.get_logger().info(f"Parameter [topic_list_paths_incoming]: {self.topic_list_paths_incoming}")
        self.get_logger().info(f"Parameter [topic_list_prefixes_incoming]: {self.topic_list_prefixes_incoming}")
        self.get_logger().info(f"Parameter [remove_target_prefix_incoming]: '{self.remove_target_prefix_incoming}'")

        self.get_logger().info(f"Parameter [topic_list_paths_outgoing]: {self.topic_list_paths_outgoing}")
        self.get_logger().info(f"Parameter [topic_list_prefixes_outgoing]: {self.topic_list_prefixes_outgoing}")
        self.get_logger().info(f"Parameter [target_prefix_outgoing]: '{self.target_prefix_outgoing}'")

        self.get_logger().info(f"Parameter [sync_suffix]: '{self.sync_suffix}'")
        self.get_logger().info(f"Parameter [qos_config_file]: '{self.qos_config_file}'")

        # --------------------------------------------------------------------
        # 2) Load topic lists from files
        # --------------------------------------------------------------------
        incoming_base_topics = self.load_topics_from_files(self.topic_list_paths_incoming)
        outgoing_base_topics = self.load_topics_from_files(self.topic_list_paths_outgoing)

        self.get_logger().info(f"Loaded {len(incoming_base_topics)} base topic(s) for INCOMING: {incoming_base_topics}")
        self.get_logger().info(f"Loaded {len(outgoing_base_topics)} base topic(s) for OUTGOING: {outgoing_base_topics}")

        # --------------------------------------------------------------------
        # 3) Create "incoming" relay pairs (OTA sub => local pub)
        # --------------------------------------------------------------------
        self.incoming_relays = []
        if not self.topic_list_prefixes_incoming: self.topic_list_prefixes_incoming = ['']
        for prefix in self.topic_list_prefixes_incoming:
            # Log which prefix we are using
            self.get_logger().debug(f"Processing incoming prefix='{prefix}'...")
            for base_topic in incoming_base_topics:
                remote_topic = prefix + base_topic + self.sync_suffix
                local_topic = self.build_local_topic_incoming(prefix, base_topic)

                self.get_logger().debug(f"Trying incoming relay for base_topic='{base_topic}', "
                                        f"remote_topic='{remote_topic}' -> local_topic='{local_topic}'")

                # If no local_topic returned (mismatch prefix removal), skip
                if not local_topic:
                    continue

                # Check that the base_topic is in our QoS config
                if base_topic not in self.qos_config_dict:
                    self.get_logger().warn(
                        f"No QoS config for base_topic='{base_topic}' in incoming relay. Skipping."
                    )
                    continue

                # Attempt to create the relay
                relay = QoSRelayPair(
                    node=self,
                    base_topic=base_topic,
                    sub_role='over_the_air_subscriber',
                    pub_role='local_publisher',
                    sub_topic=remote_topic,
                    pub_topic=local_topic,
                    config_entry=self.qos_config_dict[base_topic]
                )
                if relay.is_valid:
                    self.incoming_relays.append(relay)
                else:
                    # The relay logs detailed errors internally, we can also log here
                    self.get_logger().error(
                        f"Failed to create incoming relay for base_topic='{base_topic}'."
                    )

        # --------------------------------------------------------------------
        # 4) Create "outgoing" relay pairs (local sub => OTA pub)
        # --------------------------------------------------------------------
        self.outgoing_relays = []
        if not self.topic_list_prefixes_outgoing: self.topic_list_prefixes_outgoing = ['']
        for prefix in self.topic_list_prefixes_outgoing:
            self.get_logger().debug(f"Processing outgoing prefix='{prefix}'...")
            for base_topic in outgoing_base_topics:
                local_topic = prefix + base_topic
                remote_topic = f"{self.target_prefix_outgoing}{prefix}{base_topic}{self.sync_suffix}"

                self.get_logger().debug(f"Trying outgoing relay for base_topic='{base_topic}', "
                                        f"local_topic='{local_topic}' -> remote_topic='{remote_topic}'")

                if base_topic not in self.qos_config_dict:
                    self.get_logger().warn(
                        f"No QoS config for base_topic='{base_topic}' in outgoing relay. Skipping."
                    )
                    continue

                relay = QoSRelayPair(
                    node=self,
                    base_topic=base_topic,
                    sub_role='local_subscriber',
                    pub_role='over_the_air_publisher',
                    sub_topic=local_topic,
                    pub_topic=remote_topic,
                    config_entry=self.qos_config_dict[base_topic]
                )
                if relay.is_valid:
                    self.outgoing_relays.append(relay)
                else:
                    self.get_logger().error(
                        f"Failed to create outgoing relay for base_topic='{base_topic}'."
                    )

        # Summaries
        self.get_logger().info(f"Created {len(self.incoming_relays)} incoming relay(s).")
        self.get_logger().info(f"Created {len(self.outgoing_relays)} outgoing relay(s).")

    def build_local_topic_incoming(self, prefix, base_topic):
        """Helper to build the local topic name for an incoming bridge, optionally stripping prefix."""
        full_prefixed = prefix + base_topic
        if self.remove_target_prefix_incoming:
            if not full_prefixed.startswith(self.remove_target_prefix_incoming):
                self.get_logger().error(
                    f"Path '{full_prefixed}' does not start with '{self.remove_target_prefix_incoming}' -> SKIPPING!"
                )
                return None
            return full_prefixed[len(self.remove_target_prefix_incoming):]
        else:
            return full_prefixed

    def get_array_param(self, param_name):
        """
        Retrieve a parameter as a string list, with fallback to an empty list.
        Using get_parameter_or() to provide a default if uninitialized.
        """
        param_value = self.get_parameter_or(
            param_name,
            rclpy.parameter.Parameter(
                param_name,
                type_=rclpy.parameter.Parameter.Type.STRING_ARRAY,
                value=[]
            )
        ).value
        return list(param_value)

    def load_topics_from_files(self, paths):
        """
        Load line-by-line topics from each file in 'paths'. If a file doesn't exist,
        logs an error and continues. Returns a combined list of topics.
        """
        all_topics = []
        for p in paths:
            if not os.path.isfile(p):
                self.get_logger().error(f"Topic list file not found: '{p}'. Skipping it.")
                continue
            self.get_logger().info(f"Loading topics from file: '{p}'")
            with open(p, 'r') as f:
                lines = [ln.strip() for ln in f if ln.strip()]
                self.get_logger().debug(f"File '{p}' => found lines: {lines}")
                all_topics.extend(lines)
        return all_topics

    def _load_qos_config(self, config_file):
        """
        Load the YAML config for QoS, expecting each key to have:
            'over_the_air_subscriber', 'local_publisher',
            'over_the_air_publisher', 'local_subscriber'.

        Example:
        "/chatter":
          over_the_air_subscriber:
            type: "std_msgs/msg/String"
            qos:
              reliability: "reliable"
              ...
          local_publisher:
            type: "std_msgs/msg/String"
            qos:
              reliability: "best_effort"
              ...
          over_the_air_publisher:
            ...
          local_subscriber:
            ...
        """
        if not config_file:
            self.get_logger().warn("No qos_config_file param provided. Using empty QoS config.")
            return {}

        if not os.path.isfile(config_file):
            self.get_logger().error(f"QoS config file '{config_file}' not found! No QoS loaded.")
            return {}

        self.get_logger().info(f"Loading QoS config from '{config_file}'")
        try:
            with open(config_file, 'r') as f:
                data = yaml.safe_load(f)
            if not isinstance(data, dict):
                self.get_logger().error(f"Invalid QoS config format in '{config_file}', must be a dict.")
                return {}
            self.get_logger().debug(f"Qos config contents: {data}")
            return data
        except Exception as e:
            self.get_logger().error(f"Failed to load QoS config '{config_file}': {e}")
            return {}

def main(args=None):
    rclpy.init(args=args)
    node = OverTheAirBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

class QoSRelayPair:
    """
    Sets up a single bridging subscription->publisher pair with:
      - sub_role (e.g. 'over_the_air_subscriber' or 'local_subscriber')
      - pub_role (e.g. 'local_publisher' or 'over_the_air_publisher')
    The final subscription topic = sub_topic
    The final publisher topic = pub_topic

    Uses the QoS settings from config_entry[sub_role] and config_entry[pub_role].
    Logs the first message relayed for debugging/troubleshooting.
    """

    def __init__(self, node: Node,
                 base_topic: str,
                 sub_role: str,
                 pub_role: str,
                 sub_topic: str,
                 pub_topic: str,
                 config_entry: dict):
        self.node = node
        self.base_topic = base_topic
        self.sub_role = sub_role
        self.pub_role = pub_role
        self.sub_topic = sub_topic
        self.pub_topic = pub_topic
        self.is_valid = False
        self.first_message_received = False
        self.message_count = 0

        sub_cfg = config_entry.get(sub_role, None)
        pub_cfg = config_entry.get(pub_role, None)

        # Check for missing roles in config
        if not sub_cfg:
            node.get_logger().error(
                f"No '{sub_role}' config found for base_topic='{base_topic}'. Skipping relay creation."
            )
            return
        if not pub_cfg:
            node.get_logger().error(
                f"No '{pub_role}' config found for base_topic='{base_topic}'. Skipping relay creation."
            )
            return

        # Check for missing type
        from rosidl_runtime_py.utilities import get_message
        sub_type_str = sub_cfg.get('type')
        pub_type_str = pub_cfg.get('type')
        if not sub_type_str:
            node.get_logger().error(
                f"No 'type' specified under '{sub_role}' for base_topic='{base_topic}'."
            )
            return
        if not pub_type_str:
            node.get_logger().error(
                f"No 'type' specified under '{pub_role}' for base_topic='{base_topic}'."
            )
            return

        try:
            msg_type_sub = get_message(sub_type_str)
            msg_type_pub = get_message(pub_type_str)
            sub_qos = self.build_qos_profile(sub_cfg.get('qos', {}))
            pub_qos = self.build_qos_profile(pub_cfg.get('qos', {}))
        except Exception as ex:
            node.get_logger().error(f"Error building QoS or message type for base_topic='{base_topic}': {ex}")
            return

        if not msg_type_sub:
            node.get_logger().error(
                f"Could not load subscriber message type '{sub_type_str}' for base_topic='{base_topic}'!"
            )
            return
        if not msg_type_pub:
            node.get_logger().error(
                f"Could not load publisher message type '{pub_type_str}' for base_topic='{base_topic}'!"
            )
            return

        self.subscription = node.create_subscription(
            msg_type_sub,
            self.sub_topic,
            self.callback,
            sub_qos
        )
        node.get_logger().info(
            f"Created Subscription (base_topic='{base_topic}', role='{sub_role}') => "
            f"topic='{self.sub_topic}', type='{sub_type_str}', QoS='{sub_qos}'"
        )

        self.publisher = node.create_publisher(
            msg_type_pub,
            self.pub_topic,
            pub_qos
        )
        node.get_logger().info(
            f"Created Publisher (base_topic='{base_topic}', role='{pub_role}') => "
            f"topic='{self.pub_topic}', type='{pub_type_str}', QoS='{pub_qos}'"
        )

        node.get_logger().info(
            f"RelayPair READY [base_topic='{base_topic}'] sub='{self.sub_topic}' -> pub='{self.pub_topic}'"
        )
        self.is_valid = True

    def callback(self, msg):
        # Simple pass-through from subscriber to publisher
        if not self.first_message_received:
            self.first_message_received = True
            self.node.get_logger().info(
                f"First message received on sub_topic='{self.sub_topic}', bridging to pub_topic='{self.pub_topic}'"
            )

        self.message_count += 1
        self.publisher.publish(msg)

    def build_qos_profile(self, qos_dict: dict) -> QoSProfile:
        """
        Convert YAML-based QoS settings into an rclpy QoSProfile.
        """
        reliability_str = qos_dict.get('reliability', 'reliable').lower()
        history_str = qos_dict.get('history', 'keep_last').lower()
        depth = qos_dict.get('depth', 10)
        durability_str = qos_dict.get('durability', 'volatile').lower()
        liveliness_str = qos_dict.get('liveliness', 'automatic').lower()
        deadline_sec = qos_dict.get('deadline', 0.0)
        lifespan_sec = qos_dict.get('lifespan', 0.0)
        liveliness_lease_sec = qos_dict.get('liveliness_lease', 0.0)

        # Reliability
        reliability = ReliabilityPolicy.RELIABLE
        if reliability_str == 'best_effort':
            reliability = ReliabilityPolicy.BEST_EFFORT

        # History
        history = HistoryPolicy.KEEP_LAST
        if history_str == 'keep_all':
            history = HistoryPolicy.KEEP_ALL

        # Durability
        durability = DurabilityPolicy.VOLATILE
        if durability_str == 'transient_local':
            durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Liveliness
        liveliness = LivelinessPolicy.AUTOMATIC
        if liveliness_str == 'manual_by_topic':
            liveliness = LivelinessPolicy.MANUAL_BY_TOPIC
        elif liveliness_str == 'manual_by_node':
            liveliness = LivelinessPolicy.MANUAL_BY_NODE

        qos = QoSProfile(
            reliability=reliability,
            history=history,
            depth=depth,
            durability=durability,
            liveliness=liveliness
        )

        if deadline_sec > 0.0:
            qos.deadline = Duration(seconds=deadline_sec)
        if lifespan_sec > 0.0:
            qos.lifespan = Duration(seconds=lifespan_sec)
        if liveliness_lease_sec > 0.0:
            qos.liveliness_lease_duration = Duration(seconds=liveliness_lease_sec)

        return qos
