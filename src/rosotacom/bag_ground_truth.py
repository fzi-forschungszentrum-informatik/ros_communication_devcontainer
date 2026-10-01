# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Ground truth from a recorded rosbag2, for replay (rosbag-driven) OTA tests.

A live test only knows what it observes; a *replay* test additionally knows its
source exactly -- the bag records, per topic, how many messages exist, over what
duration, at what QoS. That known-future lets a replay test make assertions a
live test cannot (see docs/reference/expectation-concepts.md "Live vs replay"): here we read the bag's
own ``metadata.yaml`` (pure YAML -- no rosbag2/ROS, no mcap decode) to extract
per-topic ground truth, and validate a session's declared ``expect`` against it.

The most common authoring bug this catches: an ``hz.min`` floor above the topic's
native publish rate -- the OTA link can only ever thin a stream, never speed it
up, so such a floor is unsatisfiable no matter how healthy the link.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

import yaml

DEFAULT_EXPECT_MIN_RATIO = 0.9
DEFAULT_STREAM_MIN_HZ = 0.1


def _native_hz(count: int, duration_s: float) -> float | None:
    if duration_s <= 0 or count <= 0:
        return None
    # N messages span N-1 intervals over the duration; for large N this ~= count/duration.
    return (count - 1) / duration_s if count > 1 else None


def _first_qos(offered: Any) -> dict[str, Any]:
    """rosbag2 stores offered_qos_profiles as a YAML string or an already-parsed
    list of per-publisher profiles. Return the first profile as a dict."""
    if isinstance(offered, str):
        try:
            offered = yaml.safe_load(offered)
        except yaml.YAMLError:
            return {}
    if isinstance(offered, list) and offered and isinstance(offered[0], dict):
        return offered[0]
    if isinstance(offered, dict):
        return offered
    return {}


def bag_ground_truth(metadata_path: str | Path) -> dict[str, dict[str, Any]]:
    """Parse a rosbag2 ``metadata.yaml`` into ``{topic: ground-truth}``.

    Accepts either the metadata.yaml path or its containing bag directory. Each
    value has: ``count``, ``duration_s``, ``native_hz`` (None if not derivable),
    ``msg_type``, ``durability``, ``reliability``.
    """
    path = Path(metadata_path)
    if path.is_dir():
        path = path / "metadata.yaml"
    doc = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    info = doc.get("rosbag2_bagfile_information") or {}
    duration_s = float((info.get("duration") or {}).get("nanoseconds", 0)) / 1e9

    out: dict[str, dict[str, Any]] = {}
    for entry in info.get("topics_with_message_count") or []:
        meta = entry.get("topic_metadata") or {}
        name = meta.get("name")
        if not name:
            continue
        count = int(entry.get("message_count") or 0)
        qos = _first_qos(meta.get("offered_qos_profiles"))
        out[str(name)] = {
            "count": count,
            "duration_s": round(duration_s, 3),
            "native_hz": _native_hz(count, duration_s),
            "msg_type": meta.get("type"),
            "durability": qos.get("durability"),
            "reliability": qos.get("reliability"),
        }
    return out


def validate_expect_against_bag(
    expect_by_topic: dict[str, dict[str, Any]], ground_truth: dict[str, dict[str, Any]]
) -> list[str]:
    """Warnings where a session's per-topic ``expect`` contradicts the bag.

    Pure and order-stable. Only emits a warning when the contradiction is certain
    from the ground truth alone (the OTA link can thin but never amplify a rate)."""
    warnings: list[str] = []
    for topic, expect in expect_by_topic.items():
        gt = ground_truth.get(topic)
        if not gt or not isinstance(expect, dict):
            continue
        native_hz = gt.get("native_hz")
        hz = expect.get("hz") or {}
        hz_min = hz.get("min") if isinstance(hz, dict) else None
        if hz_min is not None and native_hz is not None and float(hz_min) > native_hz:
            warnings.append(
                f"{topic}: expect.hz.min {hz_min} exceeds the bag's native rate "
                f"{native_hz:.1f} Hz ({gt['count']} msgs / {gt['duration_s']}s) -- the OTA "
                f"link can only thin a stream, so this floor is unsatisfiable."
            )
        # A bag topic published essentially once and held (transient_local) is a
        # static/latched topic; a stream contract (hz floor) misdescribes it.
        if (
            gt.get("durability") == "transient_local"
            and (gt.get("count") or 0) <= 3
            and hz_min is not None
            and str(expect.get("mode", "stream")).strip().lower() == "stream"
        ):
            warnings.append(
                f"{topic}: bag publishes it {gt['count']}x as transient_local (static/held), "
                f"but expect is a stream with an hz floor -- consider `mode: latched`."
            )
    return warnings


@dataclass(frozen=True)
class GeneratedTopicExpect:
    direction: str
    topic: str
    expect: dict[str, Any]
    comment: str


@dataclass(frozen=True)
class WholeBagExpectFragment:
    topics: dict[str, tuple[GeneratedTopicExpect, ...]]
    missing_session_topics: tuple[str, ...]
    uncarried_bag_topics: tuple[str, ...]


@dataclass(frozen=True)
class _Shaping:
    expected_count: float
    expected_hz: float | None
    vs_bag_factor: float | None
    comments: tuple[str, ...]


def _topic_name(item: Any) -> str | None:
    if isinstance(item, str):
        return item
    if isinstance(item, dict) and isinstance(item.get("topic"), str):
        return cast(str, item["topic"])
    return None


def _processing(item: Any) -> dict[str, Any]:
    if isinstance(item, dict) and isinstance(item.get("processing"), dict):
        return cast(dict[str, Any], item["processing"])
    return {}


def _qos_is_transient_local(value: Any) -> bool:
    if not isinstance(value, dict):
        return False
    durability = value.get("durability")
    if isinstance(durability, str) and durability.strip().lower() == "transient_local":
        return True
    for nested in value.values():
        if _qos_is_transient_local(nested):
            return True
    return False


def _is_latched(item: Any, gt: dict[str, Any]) -> tuple[bool, str | None]:
    proc = _processing(item)
    if bool(proc.get("latch")):
        return True, "session processing.latch=true"
    if gt.get("durability") == "transient_local":
        return True, "bag QoS durability=transient_local"
    if isinstance(item, dict) and _qos_is_transient_local(item.get("qos")):
        return True, "session qos durability=transient_local"
    return False, None


def _float_or_none(value: Any) -> float | None:
    if value is None:
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _round_metric(value: float, digits: int = 3) -> float:
    rounded = round(float(value), digits)
    if rounded == 0 and value > 0:
        return float(f"{value:.3g}")
    return rounded


def _shape_expected_delivery(item: Any, gt: dict[str, Any]) -> _Shaping:
    count = float(int(gt.get("count") or 0))
    duration_s = _float_or_none(gt.get("duration_s")) or 0.0
    native_hz = _float_or_none(gt.get("native_hz"))
    expected_count = count
    expected_hz = native_hz
    comments: list[str] = []

    proc = _processing(item)
    drop = proc.get("drop")
    if isinstance(drop, dict) and drop.get("drop_count") is not None and drop.get("window_size") is not None:
        drop_count = int(drop.get("drop_count") or 0)
        window_size = int(drop.get("window_size") or 0)
        if window_size > 0 and 0 <= drop_count < window_size:
            keep_ratio = (window_size - drop_count) / window_size
            expected_count *= keep_ratio
            if expected_hz is not None:
                expected_hz *= keep_ratio
            comments.append(f"drop {drop_count}/{window_size} keeps x{keep_ratio:.3g}")

    throttle = proc.get("throttle_hz")
    if throttle is not None:
        throttle_hz = float(throttle)
        if throttle_hz > 0:
            if expected_hz is None:
                if duration_s > 0:
                    expected_count = min(expected_count, throttle_hz * duration_s + 1)
                comments.append(f"throttle_hz {throttle_hz:g}")
            elif expected_hz > throttle_hz:
                before = expected_hz
                expected_hz = throttle_hz
                if duration_s > 0:
                    expected_count = min(expected_count, throttle_hz * duration_s + 1)
                comments.append(f"throttle_hz {throttle_hz:g} caps {before:.3g} Hz")
            else:
                comments.append(f"throttle_hz {throttle_hz:g} is not limiting")

    vs_bag_factor = expected_hz / native_hz if native_hz and native_hz > 0 and expected_hz is not None else None
    return _Shaping(
        expected_count=expected_count,
        expected_hz=expected_hz,
        vs_bag_factor=vs_bag_factor,
        comments=tuple(comments),
    )


def _mode_for_topic(
    item: Any,
    gt: dict[str, Any],
    *,
    stream_min_hz: float,
) -> tuple[str, str]:
    latched, reason = _is_latched(item, gt)
    if latched:
        return "latched", str(reason)

    count = int(gt.get("count") or 0)
    native_hz = _float_or_none(gt.get("native_hz"))
    if count <= 1:
        return "existence", "bag has at most one message and is not transient_local"
    if native_hz is None or native_hz < stream_min_hz:
        hz_text = "unknown" if native_hz is None else f"{native_hz:.3g} Hz"
        return "existence", f"native rate {hz_text} is below stream_min_hz {stream_min_hz:g}"
    return "stream", f"native rate {native_hz:.3g} Hz is stream-like"


def _base_derivation(topic: str, gt: dict[str, Any]) -> str:
    native_hz = _float_or_none(gt.get("native_hz"))
    hz_text = "unknown Hz" if native_hz is None else f"{native_hz:.3g} Hz"
    return (
        f"{topic}: bag_count={int(gt.get('count') or 0)}, "
        f"duration={float(gt.get('duration_s') or 0):g}s, native={hz_text}"
    )


def generate_whole_bag_expectations(
    cfg: dict[str, Any],
    ground_truth: dict[str, dict[str, Any]],
    *,
    min_ratio: float = DEFAULT_EXPECT_MIN_RATIO,
    stream_min_hz: float = DEFAULT_STREAM_MIN_HZ,
) -> WholeBagExpectFragment:
    """Generate mergeable per-topic ``expect`` blocks from bag metadata.

    The result covers session-carried topics that are present in the bag ground
    truth. Stream thresholds are scaled for intended send-side shaping
    (``drop`` and ``throttle_hz``), so configured decimation is not counted as
    OTA loss.
    """
    if not 0 < min_ratio <= 1:
        raise ValueError("min_ratio must be > 0 and <= 1")
    if stream_min_hz < 0:
        raise ValueError("stream_min_hz must be >= 0")

    topics = cfg.get("topics")
    if not isinstance(topics, dict):
        return WholeBagExpectFragment(
            topics={},
            missing_session_topics=(),
            uncarried_bag_topics=tuple(sorted(ground_truth)),
        )

    generated: dict[str, list[GeneratedTopicExpect]] = {}
    missing: list[str] = []
    carried_topics: set[str] = set()
    for direction, entries in topics.items():
        if not isinstance(entries, list):
            continue
        direction_key = str(direction)
        for item in entries:
            topic = _topic_name(item)
            if topic is None:
                continue
            carried_topics.add(topic)
            gt = ground_truth.get(topic)
            if gt is None:
                missing.append(f"{direction_key}:{topic}")
                continue

            mode, mode_reason = _mode_for_topic(item, gt, stream_min_hz=stream_min_hz)
            expect: dict[str, Any] = {"presence": "required", "mode": mode}
            derivation = [_base_derivation(topic, gt), f"mode={mode} ({mode_reason})"]
            if mode == "stream":
                shaping = _shape_expected_delivery(item, gt)
                min_count = max(1, int(math.floor(shaping.expected_count * min_ratio)))
                completeness: dict[str, Any] = {"min_ratio": _round_metric(min_ratio)}
                if shaping.vs_bag_factor is not None:
                    completeness["vs_bag_ratio"] = _round_metric(shaping.vs_bag_factor * min_ratio)
                expect["min_count"] = min_count
                expect["completeness"] = completeness
                shape_text = "; ".join(shaping.comments) if shaping.comments else "none"
                expected_hz = "unknown" if shaping.expected_hz is None else f"{shaping.expected_hz:.3g} Hz"
                derivation.append(
                    f"shaping={shape_text}; expected={shaping.expected_count:.3g} msgs/{expected_hz}; "
                    f"min_ratio={min_ratio:g} -> min_count={min_count}"
                )
            elif mode == "latched":
                derivation.append("latched mode checks held-value delivery, not rate or full count")
            else:
                derivation.append("existence mode checks graph presence for sparse/irregular non-latched topics")

            generated.setdefault(direction_key, []).append(
                GeneratedTopicExpect(
                    direction=direction_key,
                    topic=topic,
                    expect=expect,
                    comment="; ".join(derivation),
                )
            )

    uncarried = tuple(sorted(set(ground_truth) - carried_topics))
    return WholeBagExpectFragment(
        topics={direction: tuple(items) for direction, items in generated.items()},
        missing_session_topics=tuple(sorted(missing)),
        uncarried_bag_topics=uncarried,
    )


def _yaml_scalar(value: str) -> str:
    dumped = str(yaml.safe_dump(value, default_flow_style=True, sort_keys=False)).strip()
    if dumped.endswith("\n..."):
        dumped = dumped[:-4].strip()
    return dumped


def _indent(text: str, spaces: int) -> list[str]:
    pad = " " * spaces
    return [pad + line if line else line for line in text.splitlines()]


def render_whole_bag_expect_fragment(
    fragment: WholeBagExpectFragment,
    *,
    bag: str | Path | None = None,
    session: str | Path | None = None,
) -> str:
    """Render a parseable YAML fragment with derivation comments."""
    lines = ["# Generated by `rosotacom expect from-bag`."]
    if bag is not None:
        lines.append(f"# Bag: {bag}")
    if session is not None:
        lines.append(f"# Session: {session}")
    if fragment.missing_session_topics:
        lines.append("# Session topics not found in the bag and skipped: " + ", ".join(fragment.missing_session_topics))
    if fragment.uncarried_bag_topics:
        lines.append("# Bag topics not carried by this session: " + ", ".join(fragment.uncarried_bag_topics))
    lines.append("topics:")
    for direction, entries in fragment.topics.items():
        lines.append(f"  {direction}:")
        for entry in entries:
            lines.append(f"    - topic: {_yaml_scalar(entry.topic)}")
            lines.append(f"      # {entry.comment}")
            dumped = yaml.safe_dump(
                {"expect": entry.expect},
                sort_keys=False,
                default_flow_style=False,
                allow_unicode=True,
            ).rstrip()
            lines.extend(_indent(dumped, 6))
    return "\n".join(lines)
