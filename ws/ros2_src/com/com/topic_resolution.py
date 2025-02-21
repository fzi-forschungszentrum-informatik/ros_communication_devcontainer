#!/usr/bin/env python3

import os

VALID_NODE_ROLES = {"relay_out", "relay_in", "bridge_out", "bridge_in"}

def load_base_topics(files):
    all_topics = []
    for f in files:
        if not os.path.isfile(f):
            # Instead of skipping silently, we raise an error or log a message:
            raise FileNotFoundError(f"Base topic file not found: '{f}'")
        with open(f, 'r') as fh:
            lines = [ln.strip() for ln in fh if ln.strip()]
            all_topics.extend(lines)
    return all_topics

def resolve_topics(
    node_role: str,
    base_topic_files: list,
    host_name: str,
    target_names: list
):
    """
    Produce a list of (sub_topic, pub_topic, base_topic) for each base topic (and each target).
    If node_role is invalid => raise ValueError (no fallback).
    If base topic files missing => raise FileNotFoundError or similar.
    """

    if node_role not in VALID_NODE_ROLES:
        raise ValueError(f"[resolve_topics] Unknown node_role='{node_role}', must be one of {VALID_NODE_ROLES}")

    base_topics = load_base_topics(base_topic_files)

    triplets = []  # list of (sub_topic, pub_topic, base_topic_name)

    for bt in base_topics:
        # For each base topic, we might produce multiple expansions if target_names is non-empty
        expansions = []
        if target_names:
            # e.g. user wants to prefix local topics with /to_shuttle_ella
            # We'll do: /to_shuttle_ella + base_topic
            for t in target_names:
                # Insert after a slash if the user’s base topic might have a slash
                # We do e.g. "/to_shuttle_ella" + bt
                # ensure no double slash
                expansions.append(f"/to_{t}{bt}")  # e.g. "/to_shuttle_ella/move_base_free/goal"
        else:
            expansions.append(bt)  # no target => just the base topic

        for expanded_topic in expansions:
            # Now generate final sub, pub
            sub_t, pub_t = compute_sub_pub(node_role, expanded_topic, host_name)
            # We store the original base topic name so the bridging node
            # can do the QoS lookup in the YAML config
            # The base topic name is still 'bt' (the original from the file),
            # not 'expanded_topic'
            triplets.append( (sub_t, pub_t, bt) )

    return triplets

def compute_sub_pub(node_role: str, topic: str, host: str):
    """
    Decide how we map local vs. forward vs. ota topics for each node_role.
    Strict checks: if the role is unknown, we raise an error (handled above).
    """
    # remove leading slash
    no_slash = topic.lstrip('/')

    if node_role == 'relay_out':
        # local => forward out
        sub_t = f"/{no_slash}"  # local user
        pub_t = f"/com/out/{host}/{no_slash}"
    elif node_role == 'bridge_out':
        # forward => OTA
        sub_t = f"/com/out/{host}/{no_slash}"
        pub_t = f"/ota/{host}/{no_slash}"
    elif node_role == 'bridge_in':
        # OTA => forward in
        sub_t = f"/ota/{host}/{no_slash}"
        pub_t = f"/com/in/{host}/{no_slash}"
    elif node_role == 'relay_in':
        # forward in => local
        sub_t = f"/com/in/{host}/{no_slash}"
        pub_t = f"/{no_slash}"
    else:
        # we should never get here because we validated node_role above
        raise ValueError(f"[compute_sub_pub] Unexpected node_role='{node_role}'")

    return (sub_t, pub_t)
