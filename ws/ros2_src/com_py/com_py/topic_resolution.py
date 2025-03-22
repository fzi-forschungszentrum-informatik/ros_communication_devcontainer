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
import re

VALID_NODE_ROLES = {"relay_out", "relay_in", "bridge_out", "bridge_in"}

def load_base_topics(files):
    """
    Load base topics from file(s). Raise FileNotFoundError if any file is missing.
    """
    all_topics = []
    for f in files:
        if not os.path.isfile(f):
            raise FileNotFoundError(f"[load_base_topics] File not found: '{f}'")
        with open(f, 'r') as fh:
            lines = [ln.strip() for ln in fh if ln.strip()]
            all_topics.extend(lines)
    return all_topics

def resolve_topics(
    node_role: str,
    base_topic_files: list,
    source_names: list,
    target_names: list,
    locally_used_source_name: bool = False,
    locally_used_expansion: bool = True
):
    """
    Produce a list of (sub_topic, pub_topic, base_topic).
      - If target_names is non-empty, we typically create expansions "/to_{t}{base_topic}" 
        for the forward side. The local side usage depends on 'locally_used_expansion'.
    """
    if node_role not in VALID_NODE_ROLES:
        raise ValueError(f"[resolve_topics] Unknown node_role='{node_role}', must be one of {VALID_NODE_ROLES}")

    base_topics = load_base_topics(base_topic_files)
    triplets = []

    for bt in base_topics:
        for src in source_names:
            # If we have target_names, create expansions for forward side
            if target_names:
                for t in target_names:
                    # The "forward side" always uses expansions:
                    forward_expanded = f"/to_{t}{bt}"
                    sub_t, pub_t = compute_sub_pub(
                        node_role=node_role,
                        forward_topic=forward_expanded, 
                        base_topic=bt,
                        source_name=src,
                        locally_used_source_name=locally_used_source_name,
                        locally_used_expansion=locally_used_expansion
                    )
                    triplets.append((sub_t, pub_t, bt))
            else:
                # No target-based expansions at all
                sub_t, pub_t = compute_sub_pub(
                    node_role=node_role,
                    forward_topic=bt, 
                    base_topic=bt,
                    source_name=src,
                    locally_used_source_name=locally_used_source_name,
                    locally_used_expansion=locally_used_expansion
                )
                triplets.append((sub_t, pub_t, bt))

    return triplets

def compute_sub_pub(
    node_role: str,
    forward_topic: str,
    base_topic: str,
    source_name: str,
    locally_used_source_name: bool,
    locally_used_expansion: bool
):
    """
    Decide final subscription and publication topics for each node_role.

    'forward_topic' is the topic with expansions for the forward side (if any).
    'base_topic' is the original from file, used if we need to strip expansions for local side.
    'source_name' is the machine name
    'locally_used_source_name' => whether local side uses /{source_name}/ prefix
    'locally_used_expansion' => if false => we remove any "/to_{target}" from local side
    """

    # Remove leading slash
    forward_ns = forward_topic.lstrip('/')
    base_ns = base_topic.lstrip('/')

    if node_role == 'relay_out':
        # Local => Forward
        # sub = local side
        # pub = forward side
        local_topic_ns = forward_ns if locally_used_expansion else base_ns
        sub_t = f"/{source_name}/{local_topic_ns}" if locally_used_source_name else f"/{local_topic_ns}"
        pub_t = f"/com/out/{source_name}/{forward_ns}" 

    elif node_role == 'relay_in':
        # Forward => Local
        # sub = forward side, pub = local side
        sub_t = f"/com/in/{source_name}/{forward_ns}"
        
        # Now for the local side, we either keep expansions or remove them:
        if locally_used_expansion:
            # local side uses the same expanded topic
            pub_t = f"/{source_name}/{forward_ns}" if locally_used_source_name else f"/{forward_ns}"
        else:
            # strip any leading "to_{target}" if present
            # e.g. "to_shuttle_ella/move_base_free/goal" => "move_base_free/goal"
            # then add local prefix if requested
            # note: we assume "to_xxx" is always at the start
            # let's do a quick approach:
            no_slash = re.sub(r'^to_[^/]+', '', forward_ns)
            no_slash = no_slash.lstrip('/')
            pub_t = f"/{source_name}/{no_slash}" if locally_used_source_name else f"/{no_slash}"

    elif node_role == 'bridge_out':
        # Forward => OTA
        sub_t = f"/com/out/{source_name}/{forward_ns}"
        pub_t = f"/ota/{source_name}/{forward_ns}"
        # expansions for local side are irrelevant here, because there's no local side in a 'bridge_out'.

    elif node_role == 'bridge_in':
        # OTA => Forward
        sub_t = f"/ota/{source_name}/{forward_ns}"
        pub_t = f"/com/in/{source_name}/{forward_ns}"
        # expansions for local side are also typically irrelevant for 'bridge_in'.

    else:
        raise ValueError(f"[compute_sub_pub] Unexpected node_role='{node_role}'")

    return (sub_t, pub_t)
