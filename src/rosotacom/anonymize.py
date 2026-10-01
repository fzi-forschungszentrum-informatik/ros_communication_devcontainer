# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import copy
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class HandoffTopic:
    source_peer: str
    target_peer: str
    direction: str
    source_topic: str
    source_type: str | None
    handoff_topic: str
    handoff_type: str | None
    generic_topic: str
    expect: dict[str, Any] | None
    qos: dict[str, Any] | None
    zen_qos: dict[str, Any] | None


@dataclass(frozen=True)
class PlaybackTopic:
    source_peer: str
    bag_topic: str
    publish_topic: str


def metadata_path_for_bag(path: Path) -> Path:
    return path / "metadata.yaml" if path.is_dir() else path.parent / "metadata.yaml"


def load_bag_metadata(path: Path) -> dict[str, Any]:
    metadata_path = metadata_path_for_bag(path)
    if not metadata_path.exists():
        raise FileNotFoundError(f"metadata.yaml not found for bag at {path}")
    loaded = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    if not isinstance(loaded, dict):
        raise RuntimeError(f"Bag metadata must contain a mapping: {metadata_path}")
    return loaded


def bag_storage_id(metadata: dict[str, Any]) -> str:
    return str((metadata.get("rosbag2_bagfile_information") or {}).get("storage_identifier") or "mcap")


def bag_topics_info(metadata: dict[str, Any]) -> dict[str, dict[str, Any]]:
    info = metadata.get("rosbag2_bagfile_information") or {}
    topics: dict[str, dict[str, Any]] = {}
    for entry in info.get("topics_with_message_count") or []:
        meta = entry.get("topic_metadata") or {}
        name = meta.get("name")
        if not name:
            continue
        topics[str(name)] = {
            "type": meta.get("type"),
            "serialization_format": meta.get("serialization_format", "cdr"),
            "offered_qos_profiles": meta.get("offered_qos_profiles", []),
            "message_count": int(entry.get("message_count") or 0),
        }
    return topics


def _topic_entry_topic(entry: Any) -> str | None:
    if isinstance(entry, str):
        return entry
    if isinstance(entry, dict) and isinstance(entry.get("topic"), str):
        return str(entry["topic"])
    return None


def _topic_entry_type(entry: Any) -> str | None:
    if isinstance(entry, dict) and entry.get("type") is not None:
        return str(entry["type"])
    return None


def _topic_entry_mapping(entry: Any) -> dict[str, Any]:
    if isinstance(entry, dict):
        return dict(entry)
    if isinstance(entry, str):
        return {"topic": entry}
    return {}


def _peer_keys(session_cfg: dict[str, Any]) -> list[str]:
    peers = session_cfg.get("peers")
    if not isinstance(peers, dict) or not peers:
        raise RuntimeError("session configuration must define peers before anonymization planning")
    keys = list(peers.keys())
    if len(keys) != 2:
        raise RuntimeError(f"anonymization planning currently supports exactly 2 peers, got {keys}")
    return keys


def _planning_config(session_cfg: dict[str, Any]) -> dict[str, Any]:
    cfg = copy.deepcopy(session_cfg)
    shared = cfg.get("shared") or {}
    if not isinstance(shared, dict):
        shared = {}
    shared["use_status_overview"] = True
    cfg["shared"] = shared
    return cfg


def _dummy_peer_addresses(peer_keys: list[str]) -> dict[str, str]:
    return {peer: f"127.99.0.{idx + 1}" for idx, peer in enumerate(peer_keys)}


def _load_pipeline_spec(path: Path) -> dict[str, Any]:
    loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(loaded, dict):
        raise RuntimeError(f"pipeline spec must contain a mapping: {path}")
    return loaded


def _handoff_stage(topic_spec: dict[str, Any]) -> dict[str, Any]:
    stages = topic_spec.get("stages") or []
    if not isinstance(stages, list):
        raise RuntimeError(f"pipeline topic stages must be a list for {topic_spec.get('base')!r}")
    for stage_name in ("processed", "native"):
        for stage in stages:
            if isinstance(stage, dict) and stage.get("stage") == stage_name:
                return stage
    raise RuntimeError(f"no outbound handoff stage found for {topic_spec.get('base')!r}")


def _outbound_specs_by_base(spec: dict[str, Any]) -> dict[str, list[dict[str, Any]]]:
    by_base: dict[str, list[dict[str, Any]]] = {}
    for topic_spec in spec.get("topics") or []:
        if not isinstance(topic_spec, dict) or topic_spec.get("direction") != "outbound":
            continue
        base = topic_spec.get("base")
        if not isinstance(base, str):
            continue
        by_base.setdefault(base, []).append(topic_spec)
    return by_base


def plan_handoff_topics(session_cfg: dict[str, Any], session_gen: Any) -> list[HandoffTopic]:
    peer_keys = _peer_keys(session_cfg)
    topics_cfg = session_cfg.get("topics") or {}
    if not isinstance(topics_cfg, dict):
        raise RuntimeError("session configuration topics must be a mapping")

    planned: list[HandoffTopic] = []
    with tempfile.TemporaryDirectory(prefix="rosotacom-anonymize-plan-") as tmp:
        tmp_path = Path(tmp)
        session_gen.func(
            session_config_obj=_planning_config(session_cfg),
            output_dir=str(tmp_path),
            peer_addresses=_dummy_peer_addresses(peer_keys),
        )

        for source_peer in peer_keys:
            target_peer = peer_keys[0] if source_peer == peer_keys[1] else peer_keys[1]
            direction = f"{source_peer}_to_{target_peer}"
            entries = topics_cfg.get(direction) or []
            if not isinstance(entries, list) or not entries:
                continue

            spec_path = tmp_path / source_peer / "pipeline_spec.yaml"
            if not spec_path.exists():
                raise RuntimeError(f"pipeline spec missing for peer {source_peer}: {spec_path}")
            by_base = _outbound_specs_by_base(_load_pipeline_spec(spec_path))

            for entry in entries:
                source_topic = _topic_entry_topic(entry)
                if not source_topic:
                    continue
                candidates = by_base.get(source_topic) or []
                if not candidates:
                    raise RuntimeError(
                        f"generated pipeline has no outbound topic for {direction} source topic {source_topic!r}"
                    )
                topic_spec = candidates.pop(0)
                handoff = _handoff_stage(topic_spec)
                generic_topic = f"/topic{len(planned) + 1}"
                entry_map = _topic_entry_mapping(entry)
                expect = entry_map.get("expect")
                qos = entry_map.get("qos")
                zen_qos = entry_map.get("zen_qos")
                planned.append(
                    HandoffTopic(
                        source_peer=source_peer,
                        target_peer=target_peer,
                        direction=direction,
                        source_topic=source_topic,
                        source_type=_topic_entry_type(entry),
                        handoff_topic=str(handoff["topic"]),
                        handoff_type=(str(handoff["type"]) if handoff.get("type") is not None else None),
                        generic_topic=generic_topic,
                        expect=copy.deepcopy(expect) if isinstance(expect, dict) else None,
                        qos=copy.deepcopy(qos) if isinstance(qos, dict) else None,
                        zen_qos=copy.deepcopy(zen_qos) if isinstance(zen_qos, dict) else None,
                    )
                )

    if not planned:
        raise RuntimeError("session configuration does not define any anonymizable outbound topics")
    return planned


def topics_map(plan: list[HandoffTopic]) -> dict[str, str]:
    return {item.handoff_topic: item.generic_topic for item in plan}


def source_topics_by_peer(plan: list[HandoffTopic]) -> dict[str, list[str]]:
    by_peer: dict[str, list[str]] = {}
    for item in plan:
        by_peer.setdefault(item.source_peer, []).append(item.generic_topic)
    return by_peer


def playback_topics_by_peer(
    replay_session_cfg: dict[str, Any],
    plan: list[HandoffTopic],
    session_gen: Any,
) -> dict[str, list[PlaybackTopic]]:
    replay_plan = plan_handoff_topics(replay_session_cfg, session_gen)
    replay_publish_topics = {(item.source_peer, item.source_topic): item.handoff_topic for item in replay_plan}

    by_peer: dict[str, list[PlaybackTopic]] = {}
    for item in plan:
        publish_topic = replay_publish_topics.get((item.source_peer, item.generic_topic))
        if publish_topic is None:
            raise RuntimeError(
                "generated replay session has no outbound handoff topic for "
                f"{item.source_peer} bag topic {item.generic_topic!r}"
            )
        by_peer.setdefault(item.source_peer, []).append(
            PlaybackTopic(
                source_peer=item.source_peer,
                bag_topic=item.generic_topic,
                publish_topic=publish_topic,
            )
        )
    return by_peer


def missing_handoff_topics(plan: list[HandoffTopic], info_by_topic: dict[str, dict[str, Any]]) -> list[HandoffTopic]:
    return [item for item in plan if item.handoff_topic not in info_by_topic]


def _replay_expect(expect: Any) -> dict[str, Any] | None:
    if not isinstance(expect, dict):
        return None
    replay = copy.deepcopy(expect)
    replay.pop("latency_ms", None)
    return replay or None


def build_replay_session_config(session_cfg: dict[str, Any], plan: list[HandoffTopic]) -> dict[str, Any]:
    cfg = copy.deepcopy(session_cfg)
    plan_by_direction: dict[str, list[HandoffTopic]] = {}
    for item in plan:
        plan_by_direction.setdefault(item.direction, []).append(item)

    replay_topics: dict[str, list[dict[str, Any]]] = {}
    topics_cfg = session_cfg.get("topics") or {}
    if not isinstance(topics_cfg, dict):
        topics_cfg = {}

    for direction, entries in topics_cfg.items():
        if not isinstance(entries, list):
            continue
        direction_plan = list(plan_by_direction.get(direction) or [])
        rewritten: list[dict[str, Any]] = []
        for entry in entries:
            if not direction_plan:
                continue
            item = direction_plan.pop(0)
            new_entry = _topic_entry_mapping(entry)
            new_entry["topic"] = item.generic_topic
            if item.handoff_type:
                new_entry["type"] = item.handoff_type
            new_entry.pop("processing", None)
            replay_expect = _replay_expect(new_entry.get("expect"))
            if replay_expect is None:
                new_entry.pop("expect", None)
            else:
                new_entry["expect"] = replay_expect
            rewritten.append(new_entry)
        replay_topics[direction] = rewritten

    cfg["topics"] = replay_topics
    return cfg


def _first_qos_profile(raw: Any) -> dict[str, Any]:
    offered = raw
    if isinstance(offered, str):
        try:
            offered = yaml.safe_load(offered)
        except yaml.YAMLError:
            return {}
    if isinstance(offered, list) and offered and isinstance(offered[0], dict):
        return dict(offered[0])
    if isinstance(offered, dict):
        return dict(offered)
    return {}


def playback_qos_overrides(
    plan: list[HandoffTopic],
    info_by_topic: dict[str, dict[str, Any]],
) -> dict[str, dict[str, Any]]:
    allowed = {
        "history",
        "depth",
        "reliability",
        "durability",
        "deadline",
        "lifespan",
        "liveliness",
        "liveliness_lease_duration",
    }
    overrides: dict[str, dict[str, Any]] = {}
    for item in plan:
        info = info_by_topic.get(item.handoff_topic) or {}
        qos = _first_qos_profile(info.get("offered_qos_profiles"))
        clean = {key: value for key, value in qos.items() if key in allowed and value is not None}
        if clean:
            overrides[item.generic_topic] = clean
    return overrides


def anonymization_manifest(
    plan: list[HandoffTopic],
    info_by_topic: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    topics: list[dict[str, Any]] = []
    for item in plan:
        info = info_by_topic.get(item.handoff_topic) or {}
        topics.append(
            {
                "source_peer": item.source_peer,
                "target_peer": item.target_peer,
                "direction": item.direction,
                "source_topic": item.source_topic,
                "source_type": item.source_type,
                "handoff_topic": item.handoff_topic,
                "handoff_type": item.handoff_type,
                "generic_topic": item.generic_topic,
                "message_count": info.get("message_count"),
                "serialization_format": info.get("serialization_format"),
                "qos_source": "bag.offered_qos_profiles" if info.get("offered_qos_profiles") else "none",
                "offered_qos_profiles": info.get("offered_qos_profiles") or [],
            }
        )
    return {
        "schema_version": 1,
        "mode": "processed_handoff_replay",
        "topics": topics,
    }
