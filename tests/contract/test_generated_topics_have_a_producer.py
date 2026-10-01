# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""A generated session may not tell a node to read a topic nothing publishes.

This repository's characteristic defect has now shipped four times, always in
the same shape and never with a symptom:

* #12   a wrapper renamed the delivered topic; the centre stayed subscribed to
        the old name and every panel stayed green.
* #259  the wrapper ran before the encoder, so the envelope carried an image
        while the receiver decoded a name that never appeared.
* #276  the reverse republish decoded to `raw` while the consumer subscribed to
        `.../compressed`; every intervention snapshot was written without its
        camera.
* #304  a decode was pointed at `<encoded>/paced` on a link that paces nothing.

The mechanism is the same every time and it is why none of them failed loudly:
a ROS node advertises its output whether or not its input exists. Point it at a
name nobody publishes and it starts, stays up, reports healthy, and delivers
nothing. Only #304 was caught by a check at all, and that check was a
six-minute Docker slice that exists for one example session.

So the invariant is checked here, in the fast lane, against every example:

    every topic a node is configured to READ is either written by another node
    the same peer configures, or a stage that peer's own pipeline spec knows

Writes are deliberately *not* required to be stages. A node may publish for a
human -- the NOR window restores `/tf` for a local TF tree, the pacer publishes
`budget_ms` -- and none of that is something the link promises to deliver. It
is the read side that can be wrong in silence.

`pipeline_spec.yaml` is the right authority for the other half because the
generator builds it on a different path from the plugin parameters: the spec is
the model, the parameters are what the nodes are actually told. #304 was
precisely a divergence between the two, with each half self-consistent.

Two guards keep this check honest as the template grows:

* every parameter the template declares must be classified here, so a new
  window cannot arrive unmodelled and silently covered;
* every topic name this file composes (`<topic>/paced`, `<topic>/drop1of2`, ...)
  is asserted to be the form the template itself composes, so a renamed suffix
  fails here instead of drifting apart.

Offline: no Docker, no ROS, no running session.
"""

from __future__ import annotations

import importlib.util
import re
import sys
from collections.abc import Iterator
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
WS = REPO_ROOT / "src" / "rosotacom" / "resources" / "ws"
GENERATOR_PY = WS / "session" / "creation" / "generate_session_files.py"
PLUGIN_BASE = WS / "session" / "content" / "base" / "session_plugin_base.yaml"
PACER_PY = WS / "ros2src" / "com_py" / "com_py" / "playout_pacer.py"
EXAMPLES = REPO_ROOT / "src" / "rosotacom" / "resources" / "examples" / "sessions"


def _load_generator() -> Any:
    spec = importlib.util.spec_from_file_location("rosotacom_generate_session_files_topics", GENERATOR_PY)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


generator = _load_generator()


# ---------------------------------------------------------------------------
# reading the generated files
# ---------------------------------------------------------------------------

#: catmux renders an unset parameter as the literal string "None".
UNSET = ("", "None", None)


def _one(params: dict, key: str) -> str | None:
    value = params.get(key)
    if isinstance(value, bool) or value in UNSET:
        return None
    return str(value)


def _many(params: dict, key: str) -> list[str]:
    """A comma-separated parameter, as the template hands it to a node."""
    value = _one(params, key)
    return [item for item in (value or "").split(",") if item]


def _set(params: dict, key: str) -> bool:
    """Is a flag-shaped parameter present at all (the template tests != None)."""
    return _one(params, key) is not None or params.get(key) is True


#: `^...$` around a literal name, which is how the compressor/wrapper config
#: files list their topics. Anything with real regex syntax is a pattern rather
#: than a name and cannot be resolved statically.
_ANCHORED_LITERAL = re.compile(r"^\^(?P<topic>/[A-Za-z0-9_/]*)\$$")


def _config_file_topics(peer_dir: Path, params: dict, key: str) -> list[str]:
    """The topics a `*_config_file` node subscribes to."""
    path = _one(params, key)
    if path is None:
        return []
    resolved = peer_dir / Path(path).name
    if not resolved.is_file():
        return []
    document = yaml.safe_load(resolved.read_text(encoding="utf-8")) or {}
    topics: list[str] = []
    for entries in document.values():
        for entry in entries or []:
            match = _ANCHORED_LITERAL.match(str((entry or {}).get("topic_regex", "")))
            if match:
                topics.append(match.group("topic"))
    return topics


# ---------------------------------------------------------------------------
# what each window touches
# ---------------------------------------------------------------------------


@dataclass
class Touches:
    reads: set[str] = field(default_factory=set)
    writes: set[str] = field(default_factory=set)

    def through(self, source: str, target: str) -> None:
        self.reads.add(source)
        self.writes.add(target)


def _slots(params: dict, template: str, limit: int = 8) -> Iterator[int]:
    """Indices whose slot is filled, for the indexed window families."""
    for index in range(1, limit + 1):
        if _one(params, template.format(index)) is not None:
            yield index


def touches(params: dict, peer_dir: Path) -> Touches:
    """Every topic this peer's parameters put on a node's input or output.

    One function rather than a rule table, because most windows are a single
    `source -> target` pair and reading them next to each other is how a wrong
    one is spotted. The composed suffixes are asserted against the template in
    `test_the_composed_names_are_the_ones_the_template_composes`.
    """
    result = Touches()

    # BEAT: this peer answers the remote heartbeat with its own.
    if remote := _one(params, "heartbeat_remote_topic"):
        result.reads.add(remote)
    if local := _one(params, "heartbeat_local_topic"):
        result.writes.add(local)

    # RS / LAT / TRICKLE: same shape, each appends its own suffix.
    for topics_key, suffix_key in (
        ("rs_restamp_topics", "rs_topic_suffix"),
        ("lat_topics", "lat_topic_suffix"),
        ("trickle_topics", "trickle_topic_suffix"),
    ):
        suffix = _one(params, suffix_key) or ""
        for topic in _many(params, topics_key):
            result.through(topic, topic + suffix)

    # FB: one direction adds the global suffix, the other takes it off again.
    global_suffix = _one(params, "fb_global_topic_suffix") or ""
    for topic in _many(params, "fb_local_to_global_topics"):
        result.through(topic, topic + global_suffix)
    for topic in _many(params, "fb_global_to_local_topics"):
        result.through(topic + global_suffix, topic)

    # DRP / THR: topic_tools, output named after the parameters themselves.
    for index in _slots(params, "drp_topic_{}"):
        topic = _one(params, f"drp_topic_{index}")
        count = _one(params, f"drp_drop_count_{index}")
        window = _one(params, f"drp_window_size_{index}")
        result.through(str(topic), f"{topic}/drop{count}of{window}")
    for index in _slots(params, "thr_topic_{}", limit=4):
        topic = _one(params, f"thr_topic_{index}")
        rate = _one(params, f"thr_rate_{index}")
        result.through(str(topic), f"{topic}/max{rate}hz")

    # IPX: the preset names the output.
    for index in _slots(params, "ipx_{}_topics", limit=4):
        preset = _one(params, f"ipx_{index}_preset")
        for topic in _many(params, f"ipx_{index}_topics"):
            result.through(topic, f"{topic}/{preset}")

    # IT: the sender's encoder.
    for index in _slots(params, "it_{}_topic", limit=4):
        topic = _one(params, f"it_{index}_topic")
        outgoing = _one(params, f"it_{index}_outgoing_suffix") or _one(params, f"it_{index}_transport")
        result.through(str(topic), f"{topic}/{outgoing}")

    # IRT: the reverse republish. Its input moves to the paced copy when the
    # link paces, and its output name is overridable so a second slot can decode
    # the same stream for comparison.
    for index in _slots(params, "irt_{}_topic", limit=4):
        topic = _one(params, f"irt_{index}_topic")
        source = f"{topic}/paced" if _set(params, f"irt_{index}_paced") else str(topic)
        target = _one(params, f"irt_{index}_out_topic") or f"{topic}/{_one(params, f'irt_{index}_out_transport')}"
        result.through(source, target)

    # PACE: the playout pacer, plus the two debug topics it publishes.
    for index in _slots(params, "pace_{}_topic", limit=4):
        topic = _one(params, f"pace_{index}_topic")
        result.through(str(topic), f"{topic}/paced")
        result.writes.update({f"{topic}/paced/budget_ms", f"{topic}/paced/queue_depth", f"{topic}/paced/hold_ms"})

    # NOR: relays a prefixed, suffixed name back onto the bare base, so a local
    # consumer (a TF tree) finds the name it expects.
    for index in _slots(params, "nor_topic_{}_base", limit=4):
        prefix = _one(params, f"nor_topic_{index}_prefix") or ""
        base = _one(params, f"nor_topic_{index}_base")
        suffix = _one(params, f"nor_topic_{index}_suffix") or ""
        result.through(f"{prefix}/{base}{suffix}", f"/{base}")

    # COMP / DECO / OTAW / OTAU: the topic list lives in a config file, and each
    # node adds or removes its own suffix.
    for config_key, suffix_key, adds in (
        ("comp_config_file", "comp_algorithm", True),
        ("deco_config_file", "deco_algorithm", False),
        ("otaw_config_file", "otaw_suffix", True),
        ("otau_config_file", "otau_suffix", False),
    ):
        raw = _one(params, suffix_key) or ""
        suffix = raw if raw.startswith("/") else f"/{raw}"
        for topic in _config_file_topics(peer_dir, params, config_key):
            # The listed topic is the node's *input* either way; `adds` says
            # whether the output grows the suffix or is the input without it.
            result.through(topic, topic + suffix if adds else topic[: -len(suffix)])

    # Observability: reads only, never republishes.
    result.reads.update(_many(params, "metric_stage_topics"))
    # STAT: the status overview reads the pacing hold a stage names, to tell
    # this session's deliberate delay from the network's. It is configured out
    # of the pipeline spec rather than out of these parameters, but it is a read
    # by a node this peer starts, so it belongs here -- a hold topic no pacer
    # publishes would leave the status silently subtracting nothing.
    result.reads.update(paced_hold_topics(peer_dir))

    return result


def _spec(peer_dir: Path) -> dict:
    path = peer_dir / "pipeline_spec.yaml"
    if not path.is_file():
        return {}
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def _stages(peer_dir: Path) -> Iterator[dict]:
    for topic in _spec(peer_dir).get("topics", []):
        yield from topic.get("stages", [])


def paced_hold_topics(peer_dir: Path) -> set[str]:
    return {stage["paced_hold_topic"] for stage in _stages(peer_dir) if stage.get("paced_hold_topic")}


def spec_topics(peer_dir: Path) -> set[str]:
    """Every topic this peer's pipeline spec models, at any stage."""
    return {stage["topic"] for stage in _stages(peer_dir)}


# ---------------------------------------------------------------------------
# the parameter classification, so nothing arrives unmodelled
# ---------------------------------------------------------------------------

#: Parameters whose value is a topic (or composes one) and which `touches`
#: therefore has to account for.
CARRY_TOPICS = (
    r"heartbeat_(local|remote)_topic",
    r"(rs_restamp|lat|trickle)_topics",
    r"(rs|lat|trickle)_topic_suffix",
    r"fb_(local_to_global|global_to_local)_topics",
    r"fb_global_topic_suffix",
    r"drp_topic_\d+",
    r"drp_(drop_count|window_size)_\d+",
    r"thr_topic_\d+",
    r"thr_rate_\d+",
    r"ipx_\d+_(topics|preset)",
    r"it_\d+_(topic|transport|outgoing_suffix)",
    r"irt_\d+_(topic|out_topic|out_transport|paced)",
    r"pace_\d+_topic",
    r"nor_topic_\d+_(prefix|base|suffix)",
    r"metric_stage_topics",
    r"(comp|deco|otaw|otau)_(config_file|algorithm|suffix)",
)

#: Parameters that name topics whose producers the *pipeline spec* owns: the
#: bridge and relay stages are computed from the topic list files by the same
#: naming rules the spec is built from, so re-deriving them here would compare
#: the generator with itself.
SPEC_OWNS = (
    r"topic_list_paths_(inbound|outbound)",
    r"(local_explicitly_targeted_inbound|app_keep_source_name_inbound)",
    r"(app_has_source_name_outbound|remote_explicitly_targeted_name_outbound)",
)

#: Everything else: addresses, files, rates, flags, window switches, encoder
#: options. Listed by pattern so an unclassified newcomer fails the guard.
NO_TOPIC = (
    r"dir_path|before_command|peer_dir|session_dir",
    r"(rmw|ota|local)_(local|ota|config_template|config_file|easy_mode_ip|spdp_interval|fragment_size|domain_id|name)",
    r"domain_bridge_config_file|use_domain_bridge",
    r"ip_(local|remote)|remote_name|qos_config_file|topic_types_file",
    r"in|out|use_zenoh_(rmw|ros2dds)",
    r"zen_.*",
    r"topic_monitor|tm_.*",
    # The two link instruments write files, not topics: the trace's JSONL and
    # the header-only capture's pcap both land in logs/<peer>/status/.
    r"status_.*|link_trace.*|ota_pcap.*|metric_stage_bag|network_monitor",
    r"heartbeat|heartbeat_(out_hz|delay_bad_ms|loss3_bad_pct)",
    r"rs|lat|trickle|fb|comp|deco|otaw|otau|nor|drp|drp2|thr|ipx|it|irt|pace",
    r"lat_keepalive_hz|trickle_rate_hz",
    r"fb_(global_frame_prefix|prefix_exclude_frames|tf_filter_frames|tf_throttle_links)",
    r"it_\d+_(compressed_jpeg_quality|ffmpeg_\w+|foxglove_\w+)",
    r"irt_\d+_transport",
    r"pace_\d+_(msg_type|adaptive|target_ms|min_ms|max_ms)",
)


def _matches(patterns: tuple[str, ...], name: str) -> int:
    return sum(1 for pattern in patterns if re.fullmatch(pattern, name))


def declared_parameters() -> list[str]:
    return list(yaml.safe_load(PLUGIN_BASE.read_text(encoding="utf-8"))["parameters"])


def test_every_template_parameter_is_classified() -> None:
    """A new window may not arrive unmodelled and be silently covered."""
    unclassified = [name for name in declared_parameters() if _matches(CARRY_TOPICS + SPEC_OWNS + NO_TOPIC, name) == 0]

    assert not unclassified, (
        "these template parameters are in neither CARRY_TOPICS, SPEC_OWNS nor "
        "NO_TOPIC, so this check does not know whether they name a topic:\n  " + "\n  ".join(unclassified)
    )


def test_no_parameter_is_classified_twice() -> None:
    """Two answers for one parameter means one of them is not being read."""
    ambiguous = [name for name in declared_parameters() if _matches(CARRY_TOPICS + SPEC_OWNS + NO_TOPIC, name) > 1]

    assert not ambiguous, "classified by more than one pattern:\n  " + "\n  ".join(ambiguous)


def test_the_composed_names_are_the_ones_the_template_composes() -> None:
    """`touches` composes names; the template composes the same ones, or this drifts."""
    template = PLUGIN_BASE.read_text(encoding="utf-8")
    for composed in (
        "${irt_1_topic}/paced",
        "${irt_1_topic}/${irt_1_out_transport}",
        "${drp_topic_1}/drop${drp_drop_count_1}of${drp_window_size_1}",
        "${thr_topic_1}/max${thr_rate_1}hz",
        "${nor_topic_1_prefix}/${nor_topic_1_base}${nor_topic_1_suffix}",
    ):
        assert composed in template, f"the template no longer composes {composed}"

    # The pacer names its own output and its debug topics. The IRT window above
    # reads that name, and `touches` claims those two writes, so all three have
    # to agree -- they are three files apart and nothing else compares them.
    pacer = PACER_PY.read_text(encoding="utf-8")
    for composed in (
        'declare_parameter("topic_suffix", "/paced")',
        '+ "/budget_ms"',
        '+ "/queue_depth"',
        '+ "/hold_ms"',
    ):
        assert composed in pacer, f"the pacer no longer composes {composed}"


# ---------------------------------------------------------------------------
# the check itself
# ---------------------------------------------------------------------------


def example_sessions() -> list[Path]:
    return sorted(path for path in EXAMPLES.iterdir() if (path / "session-definition.yaml").is_file())


def _generate(definition: Path, output_dir: Path) -> list[str]:
    """Generate both peers of a session, and say which peers they are."""
    document = yaml.safe_load((definition / "session-definition.yaml").read_text(encoding="utf-8"))
    peers = list(document.get("peers") or document.get("peer_settings") or {})
    generator.func(
        session_config_yaml=str(definition / "session-definition.yaml"),
        output_dir=str(output_dir),
        force=True,
        peer_addresses={peer: f"127.0.0.{index + 1}" for index, peer in enumerate(peers)},
    )
    return peers


@pytest.mark.parametrize("definition", example_sessions(), ids=lambda path: path.name)
def test_every_read_topic_has_something_that_publishes_it(definition: Path, tmp_path: Path) -> None:
    peers = _generate(definition, tmp_path)
    assert peers, f"{definition.name} declares no peers"

    for peer in peers:
        peer_dir = tmp_path / peer
        params = yaml.safe_load((peer_dir / "plugin.yaml").read_text(encoding="utf-8"))["parameters"]
        touched = touches(params, peer_dir)
        produced = touched.writes | spec_topics(peer_dir)

        orphans = sorted(topic for topic in touched.reads if topic not in produced)

        assert not orphans, (
            f"{definition.name}/{peer}: configured to read topics that neither "
            f"another node here nor this peer's pipeline spec publishes. A node "
            f"pointed at one of these advertises its output and delivers "
            f"nothing:\n  " + "\n  ".join(orphans)
        )


#: Which window families the example corpus actually puts a topic on. Pinned
#: rather than computed-and-ignored, because the value of this check is bounded
#: by it: a family no example exercises is a family this check never sees.
#:
#: Growing the set is good news -- update it. Shrinking it means an example
#: stopped exercising something and the check quietly got weaker, which is the
#: failure this pin exists to make loud.
FAMILIES_THE_EXAMPLES_EXERCISE = frozenset(
    {"comp", "deco", "drp", "heartbeat", "irt", "it", "lat", "metric", "otau", "otaw", "rs", "thr", "trickle"}
)

#: The rest of what the template can emit. Each is configuration this check
#: models but no example ever produces, so the modelling is unverified:
#: `pace` most of all, since the pacer is deployed on a real link and #304 was
#: its wiring. A known gap.
FAMILIES_NO_EXAMPLE_EXERCISES = frozenset({"fb", "ipx", "nor", "pace"})


def _families_with_topics(params: dict) -> set[str]:
    return {
        key.split("_")[0]
        for key, value in params.items()
        if _one(params, key) is not None and _matches(CARRY_TOPICS, key)
    }


def test_the_examples_exercise_the_windows_this_check_claims_to_cover(tmp_path: Path) -> None:
    exercised: set[str] = set()
    for index, definition in enumerate(example_sessions()):
        output = tmp_path / str(index)
        for peer in _generate(definition, output):
            params = yaml.safe_load((output / peer / "plugin.yaml").read_text(encoding="utf-8"))["parameters"]
            exercised |= _families_with_topics(params)

    assert exercised == set(FAMILIES_THE_EXAMPLES_EXERCISE), (
        "the example corpus no longer exercises what this pin records.\n"
        f"  gained: {sorted(exercised - FAMILIES_THE_EXAMPLES_EXERCISE)}\n"
        f"  lost:   {sorted(FAMILIES_THE_EXAMPLES_EXERCISE - exercised)}\n"
        "Gained is good news: add them here. Lost means this check got weaker."
    )
    assert not exercised & FAMILIES_NO_EXAMPLE_EXERCISES


def test_the_check_catches_the_defect_it_was_written_for(tmp_path: Path) -> None:
    """#304 on a real session: a decode pointed at a paced copy, no pacer running.

    Mutating the generated parameters rather than hand-writing a dict, so the
    same path runs that the check runs -- `touches` against `spec_topics` of a
    session that really exists. `17_synthetic_camera_quality` is the example the
    defect actually broke.
    """
    definition = EXAMPLES / "17_synthetic_camera_quality"
    peers = _generate(definition, tmp_path)
    peer_dir = tmp_path / peers[0]
    params = yaml.safe_load((peer_dir / "plugin.yaml").read_text(encoding="utf-8"))["parameters"]
    assert params.get("irt_1_topic"), "this example is expected to decode on the receiver"
    assert not _set(params, "irt_1_paced"), "this example declares no playout"

    healthy = touches(params, peer_dir)
    assert not healthy.reads - (healthy.writes | spec_topics(peer_dir))

    params["irt_1_paced"] = "true"  # what the generator wrongly emitted in #304
    broken = touches(params, peer_dir)
    orphans = broken.reads - (broken.writes | spec_topics(peer_dir))

    assert orphans == {f"{params['irt_1_topic']}/paced"}
