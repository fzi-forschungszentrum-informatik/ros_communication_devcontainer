# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""What a two-host benchmark must carry to its peers, and where it looks after.

Three defects this pins, all found by running one: the same session under the
same profile was shaped in opposite directions on one host and on two; the
benchmark's load never left the orchestrator; and the collection looked for the
peer's artifacts in a directory the project does not use, then reported the
symptom rather than the cause.
"""

from __future__ import annotations

import argparse

import pytest

from rosotacom.benchmark import parse_size_pattern_load
from rosotacom.cli import (
    _ota_instances_rel,
    _ota_publish_parts,
    _profile_directions_for,
    _publish_load_override,
)
from rosotacom.cli_benchmark import _sized_publisher_param_args


class _Target:
    target_type = "session"
    name = "bench_1_1_capacity"


def _cfg(a_to_b: int, b_to_a: int, **shared) -> dict:
    return {
        "topics": {
            "a_to_b": [{"topic": f"/a{i}"} for i in range(a_to_b)],
            "b_to_a": [{"topic": f"/b{i}"} for i in range(b_to_a)],
        },
        "shared": shared,
    }


# ------------------------------------------------- which peer meets `uplink`


def test_the_sender_of_the_loaded_direction_shapes_uplink():
    """A profile's impaired leg must land on the egress that carries the load.

    `bench_1_1_capacity` sends a->b. The OTA runner used to hand `uplink` to b
    on the a=centre/b=vehicle convention, so a run under a loss profile shaped
    the idle direction and delivered every message -- a green run measuring
    nothing.
    """
    assert _profile_directions_for(["a", "b"], _cfg(1, 0)) == {"a": "uplink", "b": "downlink"}


def test_a_vehicle_to_centre_session_still_shapes_the_vehicle():
    assert _profile_directions_for(["a", "b"], _cfg(0, 1)) == {"b": "uplink", "a": "downlink"}


def test_a_symmetric_session_falls_back_to_the_a_b_convention():
    """Neither direction dominates, so the session does not say; b sends."""
    assert _profile_directions_for(["a", "b"], _cfg(2, 2)) == {"b": "uplink", "a": "downlink"}


def test_an_explicit_declaration_wins_over_both():
    cfg = _cfg(1, 0, profile_directions={"a": "downlink", "b": "uplink"})
    assert _profile_directions_for(["a", "b"], cfg) == {"a": "downlink", "b": "uplink"}


# ------------------------------------------------------- the load must travel


def test_the_benchmark_load_reaches_the_peer_command():
    parts = _ota_publish_parts(_Target(), "a", [], load={"size": 12000, "rate": 10.0, "streams": 2})
    assert "--load-size" in parts and parts[parts.index("--load-size") + 1] == "12000"
    assert "--load-rate-hz" in parts and parts[parts.index("--load-rate-hz") + 1] == "10.0"
    assert "--load-streams" in parts and parts[parts.index("--load-streams") + 1] == "2"


def test_a_size_pattern_travels_as_the_string_it_was_given():
    """The source pattern, not the expanded token form it was compiled into.

    `parse_size_pattern_load` turns one string into `sizes`, `pattern`,
    `size_pattern` and a `size_<label>` per distinct size, and only three of
    those used to cross the hop. A two-size pattern arrived as `a*6,b*1` with a
    `size_a` and no `size_b`, so the peer's publisher died on `Pattern
    references size 'b' but it was not provided` -- inside a detached
    `docker exec`, which the orchestrator could only report as the topic never
    advertising. Sending the source string cannot lose a size.
    """
    load = parse_size_pattern_load("6x200B+1x28000B")
    parts = _ota_publish_parts(_Target(), "a", [], load=load)

    assert parts[parts.index("--load-size-pattern") + 1] == "6x200B+1x28000B"
    assert "a*6,b*1" not in parts
    # `size_a` is the pattern's own base size and must reach the peer as --load-size.
    assert parts[parts.index("--load-size") + 1] == "200"


def test_a_hand_built_pattern_load_is_refused_rather_than_silently_truncated():
    with pytest.raises(RuntimeError, match="without the size pattern it came from"):
        _ota_publish_parts(_Target(), "a", [], load={"pattern": "a*4,b*1", "size_a": 5000})


def test_every_size_of_a_pattern_survives_the_round_trip_to_the_publisher():
    """Orchestrator load -> peer flags -> peer load -> publisher parameters.

    The end the defect lived at: each step looked right on its own, and only
    the whole chain shows that the second size never arrives.
    """
    parts = _ota_publish_parts(_Target(), "a", [], load=parse_size_pattern_load("6x200B+1x28000B"))
    args = argparse.Namespace(
        load_size=int(parts[parts.index("--load-size") + 1]),
        load_size_pattern=parts[parts.index("--load-size-pattern") + 1],
        load_rate_hz=None,
        load_streams=None,
        load_interval_jitter_ms=None,
        load_interval_jitter_seed=None,
    )
    peer_load = _publish_load_override(args)
    assert peer_load is not None

    params = _sized_publisher_param_args("/bench_capacity", dict(peer_load))

    assert "size_a:=200" in params
    assert "size_b:=28000" in params
    assert "pattern:=a*6,b*1" in params


def test_a_peer_newer_than_its_orchestrator_says_so_instead_of_failing_obscurely():
    args = argparse.Namespace(
        load_size=5000,
        load_size_pattern="a*4,b*1",
        load_rate_hz=None,
        load_streams=None,
        load_interval_jitter_ms=None,
        load_interval_jitter_seed=None,
    )
    with pytest.raises(RuntimeError, match="same rosotacom"):
        _publish_load_override(args)


def test_no_load_leaves_the_session_to_say_what_it_expects():
    parts = _ota_publish_parts(_Target(), "a", [])
    assert not [p for p in parts if p.startswith("--load-")]


def test_stopping_carries_no_load():
    parts = _ota_publish_parts(_Target(), "a", [], stop=True, load={"size": 1})
    assert "--stop" in parts
    assert not [p for p in parts if p.startswith("--load-")]


def test_the_peer_turns_the_flags_back_into_a_load():
    args = argparse.Namespace(
        load_size=12000,
        load_size_pattern=None,
        load_rate_hz=10.0,
        load_streams=None,
        load_interval_jitter_ms=None,
        load_interval_jitter_seed=None,
    )
    assert _publish_load_override(args) == {"size": 12000, "size_a": 12000, "rate": 10.0}


def test_an_empty_namespace_means_use_the_session():
    args = argparse.Namespace(
        load_size=None,
        load_size_pattern=None,
        load_rate_hz=None,
        load_streams=None,
        load_interval_jitter_ms=None,
        load_interval_jitter_seed=None,
    )
    assert _publish_load_override(args) is None


# --------------------------------------------- where the artifacts are looked for


class _Runtime:
    def __init__(self, path):
        self.rosotacom_config = path


def test_the_collection_follows_the_project_not_a_convention(tmp_path):
    project = tmp_path / "rosotacom.yaml"
    project.write_text("session_instances_dir: ../instances\n", encoding="utf-8")
    assert _ota_instances_rel(_Runtime(project)) == "../instances"


def test_a_project_that_says_nothing_gets_the_default(tmp_path):
    project = tmp_path / "rosotacom.yaml"
    project.write_text("session_configs_dir:\n  - sessions\n", encoding="utf-8")
    assert _ota_instances_rel(_Runtime(project)) == "session-instances"
