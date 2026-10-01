# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""network profiles reference validation checklist — host tests for the network-profile schema and
``tc``/``netem`` command generation (the pure half; privileged arming is a bench check)."""

from __future__ import annotations

import subprocess

import pytest

from rosotacom.network_profiles import (
    OUTAGE_CATCHUP,
    OUTAGE_RECONNECT,
    DirectionShaping,
    GemodelLoss,
    Profile,
    TimelineSegment,
    expand_timeline,
    load_profiles_file,
    parse_direction,
    parse_ms,
    parse_pct,
    parse_profile,
    parse_profiles,
    parse_rate_bps,
    parse_seconds,
    parse_seed,
    required_dist_installs,
    resolve_profile_selection,
    shaping_commands,
    tc_lib_dir_probe,
    teardown_command,
)

# --- value parsing --------------------------------------------------------- #


def test_rate_parsing_is_bit_oriented() -> None:
    assert parse_rate_bps("4mbit") == 4_000_000
    assert parse_rate_bps("500kbit") == 500_000
    assert parse_rate_bps("2.5mbit") == 2_500_000
    assert parse_rate_bps(4_000_000) == 4_000_000
    with pytest.raises(ValueError):
        parse_rate_bps("4mbps")  # byte units rejected — ambiguous on the wire
    with pytest.raises(ValueError):
        parse_rate_bps("0mbit")


def test_duration_and_pct_parsing() -> None:
    assert parse_ms("120ms") == 120.0
    assert parse_ms("0.5s") == 500.0
    assert parse_ms(30) == 30.0
    assert parse_seconds("30s") == 30.0
    assert parse_seconds("500ms") == 0.5
    assert parse_pct("2%") == 2.0
    assert parse_pct(100) == 100.0
    assert parse_seed("12345") == 12345
    with pytest.raises(ValueError):
        parse_pct("150%")
    with pytest.raises(ValueError):
        parse_seconds("0s")
    with pytest.raises(ValueError):
        parse_seed(0)


# --- direction schema + validation ----------------------------------------- #


def test_parse_direction_full_block() -> None:
    shaping = parse_direction(
        {"rate": "4mbit", "delay": "120ms", "jitter": "30ms", "distribution": "normal", "loss": "2%"}
    )
    assert shaping is not None
    assert shaping.rate_bps == 4_000_000
    assert shaping.delay_ms == 120.0
    assert shaping.jitter_ms == 30.0
    assert shaping.distribution == "normal"
    assert shaping.loss_pct == 2.0
    assert not shaping.is_empty and shaping.has_netem


def test_parse_direction_rejects_unknown_keys_and_invalid_combos() -> None:
    with pytest.raises(ValueError):
        parse_direction({"bandwidth": "4mbit"})  # unknown key
    with pytest.raises(ValueError):
        DirectionShaping(jitter_ms=30.0)  # jitter without delay
    with pytest.raises(ValueError):
        DirectionShaping(delay_ms=120.0, distribution="normal")  # distribution without jitter
    with pytest.raises(ValueError):
        DirectionShaping(loss_correlation_pct=25.0)  # correlation without loss
    with pytest.raises(ValueError):
        DirectionShaping(reorder_pct=5.0)  # reorder without delay


# --- tc/netem command generation ------------------------------------------- #


def test_shaping_commands_tbf_root_with_netem_child() -> None:
    # The canonical static cellular profile from the calibrated baseline. Emitted
    # as `replace` (add-or-change): creates from a clean slate, updates in place.
    shaping = DirectionShaping(rate_bps=4_000_000, delay_ms=120.0, jitter_ms=30.0, distribution="normal", loss_pct=2.0)
    commands = shaping_commands("tun0", shaping)
    assert commands == [
        [
            "tc",
            "qdisc",
            "replace",
            "dev",
            "tun0",
            "root",
            "handle",
            "1:",
            "tbf",
            "rate",
            "4000000bit",
            "burst",
            "32kbit",
            "latency",
            "100ms",
        ],
        [
            "tc",
            "qdisc",
            "replace",
            "dev",
            "tun0",
            "parent",
            "1:",
            "handle",
            "10:",
            "netem",
            "delay",
            "120ms",
            "30ms",
            "distribution",
            "normal",
            "loss",
            "2%",
        ],
    ]


def test_shaping_commands_netem_only_becomes_root() -> None:
    commands = shaping_commands("tun0", DirectionShaping(delay_ms=60.0, loss_pct=1.0))
    assert commands == [
        ["tc", "qdisc", "replace", "dev", "tun0", "root", "handle", "10:", "netem", "delay", "60ms", "loss", "1%"]
    ]


def test_shaping_commands_rate_only_and_empty() -> None:
    rate_only = shaping_commands("eth0", DirectionShaping(rate_bps=1_000_000))
    assert rate_only == [
        [
            "tc",
            "qdisc",
            "replace",
            "dev",
            "eth0",
            "root",
            "handle",
            "1:",
            "tbf",
            "rate",
            "1000000bit",
            "burst",
            "32kbit",
            "latency",
            "100ms",
        ]
    ]
    assert shaping_commands("eth0", DirectionShaping()) == []


def test_netem_arg_ordering_is_valid() -> None:
    # netem requires delay before reorder, and the correlation value follows its base loss.
    shaping = DirectionShaping(
        delay_ms=100.0, jitter_ms=20.0, loss_pct=3.0, loss_correlation_pct=25.0, duplicate_pct=1.0, reorder_pct=5.0
    )
    netem = shaping_commands("tun0", shaping)[0]
    assert netem[netem.index("netem") :] == [
        "netem",
        "delay",
        "100ms",
        "20ms",
        "loss",
        "3%",
        "25%",
        "duplicate",
        "1%",
        "reorder",
        "5%",
    ]


def test_shaping_commands_emit_netem_seed() -> None:
    commands = shaping_commands(
        "tun0",
        DirectionShaping(delay_ms=100.0, jitter_ms=20.0, distribution="normal", loss_pct=1.0, seed=12345),
    )
    assert commands == [
        [
            "tc",
            "qdisc",
            "replace",
            "dev",
            "tun0",
            "root",
            "handle",
            "10:",
            "netem",
            "delay",
            "100ms",
            "20ms",
            "distribution",
            "normal",
            "loss",
            "1%",
            "seed",
            "12345",
        ]
    ]


def test_teardown_is_root_delete() -> None:
    assert teardown_command("tun0") == ["tc", "qdisc", "del", "dev", "tun0", "root"]


# --- outage kinds (both named variants, in the timeline tree) --------------- #


def test_timeline_catchup_is_in_place_full_loss_interface_up() -> None:
    profile = parse_profile(
        "o",
        {
            "timeline": [
                {"for": "5s", "uplink": {"delay": "50ms"}},
                {"for": "5s", "outage": "catchup"},
                {"for": "5s", "uplink": {"delay": "50ms"}},
            ]
        },
    )
    steps = expand_timeline(profile, "tun0")
    assert steps[1].commands == [
        ["tc", "qdisc", "replace", "dev", "tun0", "root", "handle", "10:", "netem", "loss", "100%"]
    ]
    # The interface never goes down, and the exit is the same in-place change back.
    assert all(cmd[0] == "tc" for step in steps for cmd in step.commands)
    assert steps[2].commands == steps[0].commands


def test_timeline_reconnect_downs_link_and_next_step_restores_it_first() -> None:
    profile = parse_profile(
        "o",
        {
            "timeline": [
                {"for": "5s", "uplink": {"rate": "4mbit"}},
                {"for": "5s", "outage": "reconnect"},
                {"for": "5s", "uplink": {"rate": "4mbit"}},
            ]
        },
    )
    steps = expand_timeline(profile, "tun0")
    assert steps[1].commands == [["ip", "link", "set", "dev", "tun0", "down"]]
    assert steps[2].commands[0] == ["ip", "link", "set", "dev", "tun0", "up"]
    assert steps[2].commands[1:] == steps[0].commands


# --- profile parsing (static + timeline) ----------------------------------- #


def test_parse_static_profile() -> None:
    profile = parse_profile(
        "cellular-typical",
        {"uplink": {"rate": "4mbit", "delay": "120ms", "loss": "2%"}, "downlink": {"rate": "25mbit", "delay": "60ms"}},
    )
    assert profile.kind == "static" and not profile.is_timeline
    assert profile.uplink is not None and profile.uplink.rate_bps == 4_000_000
    assert profile.downlink is not None and profile.downlink.rate_bps == 25_000_000


def test_parse_profiles_document() -> None:
    profiles = parse_profiles(
        {"profiles": {"none-ish": {"uplink": {"delay": "10ms"}}, "cellular": {"uplink": {"rate": "4mbit"}}}}
    )
    assert set(profiles) == {"none-ish", "cellular"}
    assert profiles["cellular"].uplink is not None and profiles["cellular"].uplink.rate_bps == 4_000_000


def test_timeline_outage_kinds_and_default() -> None:
    profile = parse_profile(
        "handover",
        {
            "timeline": [
                {"for": "30s", "uplink": {"rate": "4mbit", "delay": "60ms", "loss": "0.1%"}},
                {"for": "15s", "uplink": {"rate": "1mbit", "delay": "180ms", "loss": "3%"}},
                {"for": "5s", "outage": "reconnect"},
                {"for": "40s", "uplink": {"rate": "2.5mbit", "delay": "90ms", "loss": "1%"}},
            ]
        },
    )
    assert profile.is_timeline and profile.total_duration_s == 90.0
    assert profile.timeline[2].outage == OUTAGE_RECONNECT
    # A bare `outage: true` defaults to the milder catchup kind.
    assert parse_profile("o", {"timeline": [{"for": "5s", "outage": True}]}).timeline[0].outage == OUTAGE_CATCHUP


def test_parse_profile_rejects_mixed_and_unknown() -> None:
    with pytest.raises(ValueError):
        parse_profile("bad", {"timeline": [{"for": "5s"}], "uplink": {"rate": "4mbit"}})  # timeline + static
    with pytest.raises(ValueError):
        parse_profile("bad", {"sidelink": {"rate": "4mbit"}})  # unknown direction key
    with pytest.raises(ValueError):
        parse_profile("bad", {"timeline": [{"for": "5s", "outage": "reconnect", "uplink": {"rate": "4mbit"}}]})


# --- timeline expansion ----------------------------------------------------- #


def test_expand_timeline_places_steps_on_absolute_clock() -> None:
    profile = parse_profile(
        "handover",
        {
            "timeline": [
                {"for": "30s", "uplink": {"rate": "4mbit", "delay": "60ms"}},
                {"for": "5s", "outage": "catchup"},
                {"for": "40s", "uplink": {"rate": "2.5mbit", "delay": "90ms"}},
            ]
        },
    )
    steps = expand_timeline(profile, "tun0", direction="uplink")
    assert [(step.start_s, step.end_s, step.outage) for step in steps] == [
        (0.0, 30.0, None),
        (30.0, 35.0, OUTAGE_CATCHUP),
        (35.0, 75.0, None),
    ]
    # Every step replaces the same tbf(1:)+netem(10:) tree in place — including the
    # catchup outage, which is a full-loss netem on an unlimited tbf stage.
    assert steps[0].commands[0][:9] == ["tc", "qdisc", "replace", "dev", "tun0", "root", "handle", "1:", "tbf"]
    netem_prefix = ["tc", "qdisc", "replace", "dev", "tun0", "parent", "1:", "handle", "10:", "netem"]
    assert steps[0].commands[1][:10] == netem_prefix
    assert steps[1].commands[0][8:10] == ["tbf", "rate"] and "10gbit" in steps[1].commands[0]
    assert steps[1].commands[1][-3:] == ["netem", "loss", "100%"]


def test_expand_timeline_replaces_one_constant_tree_no_teardown() -> None:
    # The seamlessness contract: a del+add between steps drops netem's queue and
    # opens an unshaped window, so steps may only `replace` an unchanging tree.
    profile = parse_profile(
        "stepping",
        {
            "timeline": [
                {"for": "5s", "uplink": {"rate": "8mbit", "delay": "50ms", "jitter": "15ms", "distribution": "normal"}},
                {"for": "5s", "uplink": {"rate": "8mbit", "delay": "50ms", "jitter": "15ms", "distribution": "normal"}},
                {"for": "5s", "uplink": {"delay": "50ms"}},
                {"for": "5s", "uplink": {"rate": "3mbit"}},
            ]
        },
    )
    steps = expand_timeline(profile, "tun0", direction="uplink")
    assert {cmd[2] for step in steps for cmd in step.commands} == {"replace"}
    # Identical consecutive segments produce identical (idempotent) commands.
    assert steps[1].commands == steps[0].commands
    # The tree shape is constant: every step arms both stages. A rate-less step
    # keeps the tbf stage as effectively-unlimited; a netem-less step keeps the
    # netem stage as a bare pass-through (netem change resets omitted params).
    assert [len(step.commands) for step in steps] == [2, 2, 2, 2]
    assert "10gbit" in steps[2].commands[0]
    assert steps[3].commands[1][-1] == "netem"


def test_expand_timeline_unshaped_direction_arms_nothing() -> None:
    profile = parse_profile(
        "uplink-only",
        {
            "timeline": [
                {"for": "5s", "uplink": {"rate": "4mbit"}},
                {"for": "5s", "uplink": {"rate": "2mbit"}},
            ]
        },
    )
    assert all(step.commands == [] for step in expand_timeline(profile, "tun0", direction="downlink"))


def test_expand_timeline_uses_the_requested_direction() -> None:
    profile = Profile(
        name="asym",
        kind="timeline",
        timeline=(
            TimelineSegment(
                for_s=10.0,
                uplink=DirectionShaping(rate_bps=1_000_000),
                downlink=DirectionShaping(rate_bps=25_000_000),
            ),
        ),
    )
    up = expand_timeline(profile, "tun0", direction="uplink")[0]
    down = expand_timeline(profile, "tun0", direction="downlink")[0]
    assert "1000000bit" in up.commands[0]
    assert "25000000bit" in down.commands[0]
    with pytest.raises(ValueError):
        expand_timeline(profile, "tun0", direction="sideways")


# --- profile file loading + selection resolution --------------------------- #


def test_load_profiles_file(tmp_path) -> None:
    path = tmp_path / "profiles.yaml"
    path.write_text(
        "profiles:\n"
        "  cellular-typical:\n"
        "    uplink: { rate: 4mbit, delay: 120ms, loss: 2% }\n"
        "  handover:\n"
        "    timeline:\n"
        "      - { for: 30s, uplink: { rate: 4mbit } }\n"
        "      - { for: 5s, outage: reconnect }\n",
        encoding="utf-8",
    )
    profiles = load_profiles_file(path)
    assert set(profiles) == {"cellular-typical", "handover"}
    assert profiles["handover"].is_timeline


def test_resolve_profile_selection() -> None:
    available = {"cellular", "handover"}
    # Explicit selection wins and is validated against the configured set.
    assert resolve_profile_selection("cellular", available=available) == "cellular"
    # Omitted → the shared default; 'none' and omission with no default → unshaped.
    assert resolve_profile_selection(None, shared_default="handover", available=available) == "handover"
    assert resolve_profile_selection(None) is None
    assert resolve_profile_selection("none", available=available) is None
    # A typo is caught when a profiles file is configured.
    with pytest.raises(ValueError):
        resolve_profile_selection("celluar", available=available)
    # A profile on a run that may not be shaped (the real fleet) is an error.
    with pytest.raises(ValueError):
        resolve_profile_selection("cellular", available=available, allow_shaping=False)
    # ...but 'none' is always fine, shaping or not.
    assert resolve_profile_selection("none", allow_shaping=False) is None


# --- Gilbert-Elliott loss (loss gemodel) ----------------------------------- #


def test_gemodel_argv_emits_all_four_probabilities() -> None:
    # Field-fitted burst loss (2026-08-17 cellular drive): rare entry into a short
    # intense bad state, light independent loss in the good state. The plain
    # 2-arg correlated loss cannot express this shape.
    shaping = DirectionShaping(
        delay_ms=25.0,
        jitter_ms=18.0,
        loss_gemodel=GemodelLoss(p_pct=0.12, r_pct=2.9, loss_bad_pct=2.7, loss_good_pct=0.098),
    )
    commands = shaping_commands("eth0", shaping)
    assert len(commands) == 1
    netem = commands[0]
    i = netem.index("loss")
    # All four positional values are always emitted, so the argv never depends
    # on iproute2 defaults (the 3rd/4th args ARE the bad/good loss probabilities).
    assert netem[i : i + 6] == ["loss", "gemodel", "0.12%", "2.9%", "2.7%", "0.098%"]


def test_gemodel_parse_and_exclusivity() -> None:
    shaping = parse_direction({"delay": "25ms", "loss_gemodel": {"p": "0.12%", "r": "2.9%", "loss_bad": "2.7%"}})
    assert shaping is not None and shaping.loss_gemodel is not None
    assert shaping.loss_gemodel.loss_good_pct == 0.0  # default: clean good state
    with pytest.raises(ValueError):
        parse_direction({"loss": "2%", "loss_gemodel": {"p": 1, "r": 50}})  # one loss model only
    with pytest.raises(ValueError):
        parse_direction({"loss_gemodel": {"p": 1}})  # r required
    with pytest.raises(ValueError):
        parse_direction({"loss_gemodel": {"p": 1, "r": 50, "h": 3}})  # unknown key
    with pytest.raises(ValueError):
        GemodelLoss(p_pct=0.0, r_pct=50.0, loss_good_pct=0.0)  # can never lose anything


# --- custom delay-distribution tables -------------------------------------- #


def test_custom_distribution_requires_a_file_and_builtin_does_not() -> None:
    assert parse_direction({"delay": "25ms", "jitter": "18ms", "distribution": "paretonormal"}) is not None
    with pytest.raises(ValueError):
        DirectionShaping(delay_ms=25.0, jitter_ms=18.0, distribution="fieldlink")  # no file
    with pytest.raises(ValueError):
        DirectionShaping(delay_ms=25.0, jitter_ms=18.0, distribution="Bad Name", distribution_file="x.dist")
    with pytest.raises(ValueError):
        DirectionShaping(distribution_file="x.dist")  # file without a name
    shaping = DirectionShaping(
        delay_ms=25.0, jitter_ms=18.0, distribution="fieldlink", distribution_file="tables/fieldlink.dist"
    )
    commands = shaping_commands("eth0", shaping)
    i = commands[0].index("distribution")
    assert commands[0][i : i + 2] == ["distribution", "fieldlink"]


def test_load_profiles_file_resolves_distribution_files_and_lists_installs(tmp_path) -> None:
    (tmp_path / "tables").mkdir()
    (tmp_path / "tables" / "fieldlink.dist").write_text("# test\n0 0 0 0\n", encoding="utf-8")
    path = tmp_path / "profiles.yaml"
    path.write_text(
        "profiles:\n"
        "  field:\n"
        "    uplink:\n"
        "      delay: 25ms\n"
        "      jitter: 18ms\n"
        "      distribution: fieldlink\n"
        "      distribution_file: tables/fieldlink.dist\n"
        "      loss_gemodel: { p: 0.12%, r: 2.9%, loss_bad: 2.7%, loss_good: 0.098% }\n",
        encoding="utf-8",
    )
    profiles = load_profiles_file(path)
    field = profiles["field"]
    assert field.uplink is not None
    # Relative distribution_file entries resolve against the profiles file.
    assert field.uplink.distribution_file == str(tmp_path / "tables" / "fieldlink.dist")
    installs = required_dist_installs(field)
    assert installs == [("fieldlink", str(tmp_path / "tables" / "fieldlink.dist"))]
    # Builtin distributions need no install.
    assert (
        required_dist_installs(
            parse_profile("p", {"uplink": {"delay": "10ms", "jitter": "5ms", "distribution": "normal"}})
        )
        == []
    )


def _probe(root) -> str:
    return subprocess.run(
        ["sh", "-c", tc_lib_dir_probe(str(root))],
        capture_output=True,
        text=True,
        check=True,
    ).stdout


def test_the_distribution_table_goes_where_tc_reads_it(tmp_path):
    """A table installed into `/usr/lib/tc` on a multiarch image is invisible.

    iproute2 compiles its library directory in, so a Debian or Ubuntu build
    reads `/usr/lib/<triplet>/tc` and never looks at `/usr/lib/tc`. netem then
    fails at arming time with "No distribution data", which reads like a broken
    profile rather than a misplaced file -- so the directory is resolved from
    the image instead of assumed.
    """
    multiarch = tmp_path / "usr/lib/x86_64-linux-gnu/tc"
    multiarch.mkdir(parents=True)
    (multiarch / "normal.dist").write_text("0\n", encoding="utf-8")
    (tmp_path / "usr/lib/tc").mkdir(parents=True)

    # Both exist; the one carrying iproute2's own table is the one tc reads.
    assert _probe(tmp_path) == str(multiarch)


def test_a_directory_that_merely_exists_is_the_second_choice(tmp_path):
    multiarch = tmp_path / "usr/lib/aarch64-linux-gnu/tc"
    multiarch.mkdir(parents=True)
    assert _probe(tmp_path) == str(multiarch)


def test_an_image_with_neither_gets_one_created(tmp_path):
    (tmp_path / "usr/lib").mkdir(parents=True)
    out = _probe(tmp_path)
    assert out == str(tmp_path / "usr/lib/tc")
    assert (tmp_path / "usr/lib/tc").is_dir()
