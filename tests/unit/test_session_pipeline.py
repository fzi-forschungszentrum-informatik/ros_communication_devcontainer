# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Stage composition in the session generator.

The pipeline is a chain of topic-name transformations, and two things have to
stay true of it: what the wrapper wraps is what actually crosses the link, and
every receiver-side stage is computed from the same delivered name. Otherwise a
topic is renamed under a receiver that subscribes by a fixed name and simply
receives nothing, with every panel still green (#259).
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
GENERATOR_PY = (
    REPO_ROOT / "src" / "rosotacom" / "resources" / "ws" / "session" / "creation" / "generate_session_files.py"
)


def _load_module(path: Path, name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


generator = _load_module(GENERATOR_PY, "rosotacom_generate_session_files_pipeline")

SUFFIXES = {
    "restamped_suffix": "/restamped",
    "latched_suffix": "/latched",
    "globalframe_suffix": "/globalframe",
    "comp_alg_suffix": "/bz2",
    "ota_suffix": "/ota_stamped",
}


def _pipeline(processing: dict, base: str = "/camera/image", msg_type: str = "sensor_msgs/msg/Image"):
    entry = generator.TopicEntry(
        base=base,
        msg_type=msg_type,
        processing=processing,
        qos=None,
        zen_qos=None,
        index=0,
    )
    pipe = generator._compute_pipeline(entry, {}, **SUFFIXES)
    return entry, pipe


def _ffmpeg(**extra: object) -> dict:
    spec: dict = {"type": "ffmpeg", "local_republish": "raw", "remote_republish": "raw"}
    spec.update(extra)
    return {"transport": {k: v for k, v in spec.items() if v is not None}}


# ---------------------------------------------------------------------------
# the wrapper is the last sender-side stage
# ---------------------------------------------------------------------------


def test_transport_is_encoded_before_it_is_wrapped() -> None:
    """#259: the envelope must carry what crosses the link, i.e. the packet."""
    entry, pipe = _pipeline({**_ffmpeg(), "use_ota_wrapper": True})

    assert pipe["it_in"] == "/camera/image"  # encoder input: the raw image
    assert pipe["ota_in"] == "/camera/image/ffmpeg"  # wrapper input: the packet
    assert pipe["final"] == "/camera/image/ffmpeg/ota_stamped"
    assert generator._final_topic_type(entry, pipe) == generator.OTA_STAMPED_MSG_TYPE


def test_wrapping_a_transport_does_not_rename_the_delivered_topic() -> None:
    """The unwrapper republishes on the pre-wrap name -- where the decoder listens."""
    _, wrapped = _pipeline({**_ffmpeg(), "use_ota_wrapper": True})
    _, plain = _pipeline(_ffmpeg())

    assert wrapped["irt_in"] == plain["irt_in"] == "/camera/image/ffmpeg"
    delivered = generator._postprocessed_topic(wrapped, wrapped["final"])
    assert delivered == generator._postprocessed_topic(plain, plain["final"]) == "/camera/image/ffmpeg/raw"


def test_transport_without_republish_delivers_the_unwrapped_packet() -> None:
    _, pipe = _pipeline(
        {**_ffmpeg(local_republish=None, remote_republish=None), "use_ota_wrapper": True},
    )

    assert pipe["irt_in"] is None
    assert generator._postprocessed_topic(pipe, pipe["final"]) == "/camera/image/ffmpeg"


# ---------------------------------------------------------------------------
# the two reverse republishes are separate decisions (#276)
# ---------------------------------------------------------------------------


def test_the_receiver_decode_names_the_delivered_topic_and_type() -> None:
    """`remote_republish` decides what the receiving application subscribes to."""
    entry, pipe = _pipeline(_ffmpeg(remote_republish="compressed"))

    assert generator._postprocessed_topic(pipe, pipe["final"]) == "/camera/image/ffmpeg/compressed"
    assert generator.REPUBLISH_OUTPUT_TYPES["compressed"] == "sensor_msgs/msg/CompressedImage"
    # What crosses the link is still the encoded packet; only the decode changed.
    assert generator._final_topic_type(entry, pipe) == generator.TRANSPORT_OUTPUT_TYPES["ffmpeg"]


def test_the_senders_preview_does_not_decide_the_delivered_topic() -> None:
    """`local_republish` runs on the sending machine and never reaches the receiver."""
    _, pipe = _pipeline(_ffmpeg(local_republish="raw", remote_republish="compressed"))

    assert generator._postprocessed_topic(pipe, pipe["final"]) == "/camera/image/ffmpeg/compressed"


def test_a_sender_only_preview_delivers_the_packet() -> None:
    """A preview on the sender leaves the receiver with what crossed the link."""
    _, pipe = _pipeline(_ffmpeg(remote_republish=None))

    # The encoded topic still exists -- the sender decodes it locally --
    # but nothing decodes it on the receiving side.
    assert pipe["irt_in"] == "/camera/image/ffmpeg"
    assert generator._postprocessed_topic(pipe, pipe["final"]) == "/camera/image/ffmpeg"


def test_a_republish_must_name_a_known_image_transport() -> None:
    with pytest.raises(ValueError, match="remote_republish"):
        _pipeline(_ffmpeg(remote_republish="jpeg"))


def test_trickle_republishes_the_unwrapped_topic() -> None:
    """#259: `<latched>/ota_stamped/trickle` is a name no application subscribes to."""
    _, pipe = _pipeline(
        {"latch": True, "trickle_hz": 1, "use_ota_wrapper": True},
        base="/can/autonomous",
        msg_type="std_msgs/msg/Bool",
    )

    assert pipe["final"] == "/can/autonomous/latched/ota_stamped"
    assert generator._delivered_topic(pipe, pipe["final"]) == "/can/autonomous/latched"
    assert generator._postprocessed_topic(pipe, pipe["final"]) == "/can/autonomous/latched/trickle"


# ---------------------------------------------------------------------------
# every combination that existed before keeps its names
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("processing", "final", "delivered"),
    [
        ({}, "/t", "/t"),
        ({"use_ota_wrapper": True}, "/t/ota_stamped", "/t"),
        ({"compress": True}, "/t/bz2", "/t"),
        ({"compress": True, "use_ota_wrapper": True}, "/t/bz2/ota_stamped", "/t"),
        ({"latch": True, "trickle_hz": 1}, "/t/latched", "/t/latched/trickle"),
        ({"framebridge": "local_to_global"}, "/t/globalframe", "/t/globalframe"),
        ({"framebridge": "global_to_local"}, "/t/globalframe", "/t"),
        (
            {"framebridge": "local_to_global", "compress": True, "use_ota_wrapper": True},
            "/t/globalframe/bz2/ota_stamped",
            "/t/globalframe",
        ),
        ({"throttle_hz": 10, "use_ota_wrapper": True}, "/t/max10hz/ota_stamped", "/t/max10hz"),
        ({"drop": {"drop_count": 1, "window_size": 2}}, "/t/drop1of2", "/t/drop1of2"),
    ],
)
def test_existing_combinations_are_unchanged(processing: dict, final: str, delivered: str) -> None:
    _, pipe = _pipeline(processing, base="/t", msg_type="std_msgs/msg/Bool")

    assert pipe["final"] == final
    assert generator._postprocessed_topic(pipe, pipe["final"]) == delivered


# ---------------------------------------------------------------------------
# end to end: the generated peer configuration
# ---------------------------------------------------------------------------


def _camera_cfg(processing: dict) -> dict:
    return {
        "peers": {"a": {}, "b": {}},
        "peer_settings": {"a": {"domain_id": 46}, "b": {"domain_id": 47}},
        "shared": {
            "use_heartbeat": False,
            "use_status_overview": True,
            "rmw": {
                "local": {"cyclone": {"config": "cyclonedds_minimal.xml"}},
                "ota": {"cyclone": {"config": "cyclonedds_minimal.xml"}},
            },
            "ota_domain_id": 48,
        },
        "topics": {
            "b_to_a": [
                {
                    "topic": "/camera/image",
                    "type": "sensor_msgs/msg/Image",
                    "processing": processing,
                    "expect": {"hz": {"min": 3, "max": 8}},
                }
            ]
        },
    }


def test_generated_peer_files_wire_the_wrapped_camera(tmp_path: Path) -> None:
    generator.func(
        session_config_obj=_camera_cfg({**_ffmpeg(gop_size=4), "use_ota_wrapper": True}),
        output_dir=str(tmp_path),
        force=True,
        peer_addresses={"a": "127.0.0.1", "b": "127.0.0.2"},
    )

    # Sender wraps the encoded packet stream, not the image.
    sender = yaml.safe_load((tmp_path / "b" / "ota_wrapper.yaml").read_text(encoding="utf-8"))
    assert [item["topic_regex"] for item in sender["ota_wrapper"]] == ["^/camera/image/ffmpeg$"]

    # Receiver unwraps back onto the name the decoder subscribes to.
    receiver = yaml.safe_load((tmp_path / "a" / "ota_unwrapper.yaml").read_text(encoding="utf-8"))
    assert [item["topic_regex"] for item in receiver["ota_unwrapper"]] == ["^/camera/image/ffmpeg/ota_stamped$"]

    spec = yaml.safe_load((tmp_path / "a" / "pipeline_spec.yaml").read_text(encoding="utf-8"))
    topic = next(t for t in spec["topics"] if t["base"] == "/camera/image")
    stages = {stage["stage"]: stage for stage in topic["stages"]}
    # com_in is where loss and OTA transit are read, so it has to be the envelope.
    assert stages["com_in"]["topic"] == "/com/in/b/camera/image/ffmpeg/ota_stamped"
    assert stages["com_in"]["type"] == generator.OTA_STAMPED_MSG_TYPE
    # ... and what the application finally reads is the decoded image.
    assert stages["native_in"]["topic"] == "/camera/image/ffmpeg/raw"
    assert stages["native_in"]["type"] == "sensor_msgs/msg/Image"


# ---------------------------------------------------------------------------
# receiver-side playout pacing (transport.playout, issue #284)
# ---------------------------------------------------------------------------


def test_playout_requires_a_receiver_republish() -> None:
    """Without a reverse republish the pacer would rename the delivered topic."""
    with pytest.raises(ValueError, match="requires remote_republish"):
        _pipeline({"transport": {"type": "ffmpeg", "playout": {"min_ms": 100}}})


def test_playout_rejects_unknown_keys_and_bad_numbers() -> None:
    with pytest.raises(ValueError, match="unknown keys"):
        _pipeline(_ffmpeg(playout={"budget_ms": 350}))
    with pytest.raises(ValueError, match="positive number"):
        _pipeline(_ffmpeg(playout={"target_ms": -1}))
    with pytest.raises(ValueError, match="max_ms must be >= min_ms"):
        _pipeline(_ffmpeg(playout={"min_ms": 500, "max_ms": 100}))


def test_playout_never_leaks_into_encoder_params() -> None:
    _entry, pipe = _pipeline(_ffmpeg(gop_size=3, playout={"min_ms": 100, "max_ms": 800}))
    tspec = pipe["transport"]
    assert tspec.playout == {"min_ms": 100, "max_ms": 800}
    assert "playout" not in tspec.params
    assert tspec.params["gop_size"] == 3


def test_playout_does_not_rename_the_delivered_topic() -> None:
    """The pacer sits between unwrapper and decode; what the application reads keeps its name."""
    _entry, plain = _pipeline(_ffmpeg())
    _entry, paced = _pipeline(_ffmpeg(playout={}))
    assert generator._delivered_topic(paced, paced["final"]) == generator._delivered_topic(plain, plain["final"])


def test_generated_peer_files_wire_the_pacer(tmp_path: Path) -> None:
    processing = {
        "transport": {
            "type": "ffmpeg",
            "gop_size": 3,
            "remote_republish": "compressed",
            "playout": {"adaptive": True, "min_ms": 120, "max_ms": 600},
        },
        "use_ota_wrapper": True,
    }
    generator.func(
        session_config_obj=_camera_cfg(processing),
        output_dir=str(tmp_path),
        force=True,
        peer_addresses={"a": "127.0.0.1", "b": "127.0.0.2"},
    )

    receiver = (tmp_path / "a" / "plugin.yaml").read_text(encoding="utf-8")
    # The pacer runs on the receiving peer against the arrived encoded stream...
    assert "pace_1_topic: /camera/image/ffmpeg" in receiver
    assert "pace_1_msg_type: ffmpeg_image_transport_msgs/msg/FFMPEGPacket" in receiver
    assert (
        "pace_1_adaptive: 'true'" in receiver
        or 'pace_1_adaptive: "true"' in receiver
        or "pace_1_adaptive: true" in receiver
    )
    assert "pace_1_min_ms: 120.0" in receiver
    assert "pace_1_max_ms: 600.0" in receiver
    # ...and the decode reads the paced copy.
    assert "irt_1_paced" in receiver

    # The sender has no pacer: its preview (if any) reads its own encode, and
    # this session declares no local_republish at all.
    sender = (tmp_path / "b" / "plugin.yaml").read_text(encoding="utf-8")
    assert "pace" not in sender or "pace_1_topic" not in sender
    assert "irt_1_paced" not in sender


# `transport.playout.republish` — which stream the receiver decodes (#302).
#
# The improvement a pacer makes is invisible while it replaces the picture under
# the same name: there is no moment where paced and unpaced are on screen
# together. `both` puts them there. What must hold across all three modes is
# that the *delivered* name never moves — an application subscribes to
# `<encoded>/<out_transport>` whatever the mode, which is the rule #276 exists
# to protect and exactly what a comparison switch could break in silence.


def _receiver_plugin(tmp_path: Path, playout: dict | None) -> str:
    transport: dict = {"type": "ffmpeg", "remote_republish": "compressed"}
    if playout is not None:
        transport["playout"] = playout
    generator.func(
        session_config_obj=_camera_cfg({"transport": transport, "use_ota_wrapper": True}),
        output_dir=str(tmp_path),
        force=True,
        peer_addresses={"a": "127.0.0.1", "b": "127.0.0.2"},
    )
    return (tmp_path / "a" / "plugin.yaml").read_text(encoding="utf-8")


def test_a_receiver_without_playout_decodes_what_arrived(tmp_path: Path) -> None:
    """A link that paces nothing has no '/paced' for its decode to read.

    Worth its own test because of how it fails: `image_transport republish`
    advertises its output whether or not its input exists, so a decode pointed
    at a name nobody publishes looks alive and delivers nothing. That is what
    the `17_synthetic_camera_quality` slice caught while the whole unit suite
    stayed green — the receiver's *unpaced* case had no assertion at all.
    """
    receiver = _receiver_plugin(tmp_path, None)

    assert "irt_1_topic: /camera/image/ffmpeg" in receiver
    assert "irt_1_paced" not in receiver
    assert "irt_1_out_topic" not in receiver
    assert "pace_1_topic" not in receiver


def test_republish_rejects_a_mode_that_is_not_one_of_the_three() -> None:
    with pytest.raises(ValueError, match="allowed: \\['paced', 'unpaced', 'both'\\]"):
        _pipeline(_ffmpeg(playout={"republish": "compare"}))


def test_paced_is_the_default_and_needs_no_output_override() -> None:
    _entry, pipe = _pipeline(_ffmpeg(playout={}))
    assert generator.playout_republish(pipe["transport"].playout) == "paced"


def test_unpaced_runs_the_pacer_but_keeps_it_off_the_display_path(tmp_path: Path) -> None:
    """The measurement mode: '/paced' and its debug topics exist, nothing reads them."""
    receiver = _receiver_plugin(tmp_path, {"republish": "unpaced"})

    assert "pace_1_topic: /camera/image/ffmpeg" in receiver
    assert "irt_1_paced" not in receiver
    assert "irt_1_out_topic" not in receiver


def test_both_adds_a_second_decode_under_a_name_of_its_own(tmp_path: Path) -> None:
    receiver = _receiver_plugin(tmp_path, {"republish": "both"})

    # Slot 1 is the delivered one: paced input, unchanged output name.
    assert "irt_1_topic: /camera/image/ffmpeg" in receiver
    assert "irt_1_paced" in receiver
    assert "irt_1_out_topic" not in receiver
    # Slot 2 is the comparison: same stream, undelayed, qualified output.
    assert "irt_2_topic: /camera/image/ffmpeg" in receiver
    assert "irt_2_out_topic: /camera/image/ffmpeg/unpaced/compressed" in receiver
    assert "irt_2_paced" not in receiver
    # One pacer, not two -- the second decode reads what the first one's pacer
    # was fed, and a second pacer on the same topic would publish onto the same
    # '/paced' name.
    assert "pace_2_topic" not in receiver


def test_every_mode_delivers_the_same_topic_name(tmp_path: Path) -> None:
    """The one invariant a comparison switch must not break."""
    delivered = set()
    for mode in ("paced", "unpaced", "both"):
        _entry, pipe = _pipeline(_ffmpeg(playout={"republish": mode}))
        delivered.add(generator._delivered_topic(pipe, pipe["final"]))
    _entry, unpaced_link = _pipeline(_ffmpeg())
    delivered.add(generator._delivered_topic(unpaced_link, unpaced_link["final"]))

    assert len(delivered) == 1, delivered


# The status has to be told which pacer belongs to which stage (#301).
#
# A decoded stage's age is the link plus a hold this session asked for, and the
# status overview cannot tell them apart by looking at one number. The generator
# knows both nodes, so it names the pacer's report on the stage it explains --
# rather than the status guessing from a topic-name convention.


def _native_in(receiver: str) -> dict:
    spec = yaml.safe_load(receiver)
    for topic in spec["topics"]:
        if topic["direction"] == "inbound":
            for stage in topic["stages"]:
                if stage["stage"] == "native_in":
                    return stage
    raise AssertionError("no inbound native_in stage")


def _receiver_spec(tmp_path: Path, playout: dict | None) -> str:
    transport: dict = {"type": "ffmpeg", "remote_republish": "compressed"}
    if playout is not None:
        transport["playout"] = playout
    generator.func(
        session_config_obj=_camera_cfg({"transport": transport, "use_ota_wrapper": True}),
        output_dir=str(tmp_path),
        force=True,
        peer_addresses={"a": "127.0.0.1", "b": "127.0.0.2"},
    )
    return (tmp_path / "a" / "pipeline_spec.yaml").read_text(encoding="utf-8")


def test_a_paced_decode_names_the_pacer_that_explains_its_age(tmp_path: Path) -> None:
    stage = _native_in(_receiver_spec(tmp_path, {"adaptive": True, "max_ms": 300}))

    assert stage["paced_hold_topic"] == "/camera/image/ffmpeg/paced/hold_ms"


def test_an_unpaced_link_names_no_pacer(tmp_path: Path) -> None:
    """Nothing holds this stream, so nothing may be subtracted from its age."""
    stage = _native_in(_receiver_spec(tmp_path, None))

    assert "paced_hold_topic" not in stage


def test_a_pacer_kept_off_the_display_path_explains_nothing(tmp_path: Path) -> None:
    """`republish: unpaced` decodes the arrived stream; its age is all link."""
    stage = _native_in(_receiver_spec(tmp_path, {"adaptive": True, "republish": "unpaced"}))

    assert "paced_hold_topic" not in stage
