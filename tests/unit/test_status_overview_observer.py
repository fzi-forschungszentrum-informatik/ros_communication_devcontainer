# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Latency and keyframe measurement in the status overview's stage observer.

Three layers: the ROS-independent `stamp_delay` in `status_overview_core`, the
observer callback that decides *whether* a stamp is comparable to the local
clock at all, and the OtaStamped branch that reads the FFMPEG keyframe bit out
of the wrapped payload. The node module imports rclpy, which the host suite
does not have, so its ROS surface is stubbed and the real core and
ffmpeg_flags modules are registered under the names the node imports
(`tests/unit/test_ota_wrapper.py` uses the same approach).
"""

from __future__ import annotations

import importlib.util
import struct
import sys
from array import array
from collections.abc import Iterator
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
COM_PY = REPO_ROOT / "src" / "rosotacom" / "resources" / "ws" / "ros2src" / "com_py" / "com_py"
CORE_PY = COM_PY / "status_overview_core.py"
NODE_PY = COM_PY / "status_overview.py"

#: Modules the node imports at load time and the host suite does not have.
_STUBBED = (
    "rclpy",
    "rclpy.context",
    "rclpy.executors",
    "rclpy.node",
    "rclpy.qos",
    "rclpy.serialization",
    "rosidl_runtime_py",
    "rosidl_runtime_py.utilities",
    "com_py",
    "com_py.ffmpeg_flags",
    "com_py.link_bytes",
    "com_py.link_trace",
    "com_py.status_overview_core",
)


def _load(path: Path, name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class _FakeNode:
    pass


def _stub_module(name: str, **attrs: object) -> ModuleType:
    module = ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    sys.modules[name] = module
    return module


core = _load(CORE_PY, "rosotacom_status_overview_core_observer")
ffmpeg_flags = _load(COM_PY / "ffmpeg_flags.py", "rosotacom_status_overview_ffmpeg_flags")


# ---------------------------------------------------------------------------
# stamp_delay (ROS-independent)
# ---------------------------------------------------------------------------


def test_stamp_delay_without_an_estimate_is_the_raw_difference() -> None:
    assert core.stamp_delay(0.25) == pytest.approx(0.25)


def test_stamp_delay_adds_the_peer_offset() -> None:
    """offset_s is peer minus local, so a peer running late shortens the delay."""
    assert core.stamp_delay(0.25, 0.10) == pytest.approx(0.35)
    assert core.stamp_delay(0.25, -0.10) == pytest.approx(0.15)


def test_stamp_delay_guards_the_corrected_value_not_the_raw_one() -> None:
    """#258: a stamp the raw guard rejects is reported once the offset explains it.

    A peer clock 5 s ahead makes every raw delay negative; the old guard dropped
    the sample and the topic reported no latency at all.
    """
    assert core.stamp_delay(-4.9) is None
    assert core.stamp_delay(-4.9, 5.0) == pytest.approx(0.1)


def test_stamp_delay_still_rejects_an_unset_stamp() -> None:
    """An epoch-0 stamp is decades, not a latency -- with or without an offset."""
    assert core.stamp_delay(1.7e9) is None
    assert core.stamp_delay(1.7e9, 0.01) is None


# ---------------------------------------------------------------------------
# the observer callback
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def node_module() -> Iterator[ModuleType]:
    saved = {name: sys.modules.get(name) for name in _STUBBED}
    _stub_module("rclpy", init=lambda *a, **k: None, shutdown=lambda *a, **k: None)
    _stub_module("rclpy.context", Context=object)
    _stub_module("rclpy.executors", MultiThreadedExecutor=object)
    _stub_module("rclpy.node", Node=_FakeNode)
    _stub_module(
        "rclpy.qos",
        DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1, VOLATILE=0),
        HistoryPolicy=SimpleNamespace(KEEP_LAST=1),
        QoSProfile=lambda **k: SimpleNamespace(**k),
        ReliabilityPolicy=SimpleNamespace(RELIABLE=1, BEST_EFFORT=0),
    )
    _stub_module("rclpy.serialization", serialize_message=lambda msg: b"\x00" * 16)
    _stub_module("rosidl_runtime_py", utilities=None)
    _stub_module("rosidl_runtime_py.utilities", get_message=lambda type_str: None)
    _stub_module("com_py", link_bytes=None)
    _stub_module(
        "com_py.link_bytes",
        LinkByteSampler=object,
        resolve_link_interface=lambda explicit, host_ip: (explicit or "tun0", "stubbed"),
    )
    _stub_module("com_py.link_trace", LinkTraceRecorder=object)
    sys.modules["com_py.status_overview_core"] = core
    sys.modules["com_py.ffmpeg_flags"] = ffmpeg_flags

    module = _load(NODE_PY, "rosotacom_status_overview_node")

    yield module

    sys.modules.pop("rosotacom_status_overview_node", None)
    for name, previous in saved.items():
        if previous is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = previous


_NOW_S = 1_000.0


class _Estimator:
    def __init__(self, offset_s: float | None, rtt_s: float = 0.04) -> None:
        self._offset_s = offset_s
        self._rtt_s = rtt_s

    def estimate(self, now_mono: float | None = None) -> dict | None:
        if self._offset_s is None:
            return None
        return {"offset_s": self._offset_s, "rtt_s": self._rtt_s, "age_s": 0.1, "samples": 12.0}


def _observer(node_module: ModuleType, topic: str, *, direction: str, offset_s: float | None):
    node = object.__new__(node_module.StageObserver)
    node.observations = {topic: core.StageObservation(type_str="sensor_msgs/msg/CompressedImage")}
    node._stage_metadata = {topic: [{"direction": direction, "stage": "com_in"}]}
    node.clock_estimator = _Estimator(offset_s)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(_NOW_S * 1e9)))
    node.get_logger = lambda: SimpleNamespace(error=lambda *a, **k: None, warning=lambda *a, **k: None)
    return node


def _stamped(delay_s: float):
    stamp_s = _NOW_S - delay_s
    sec = int(stamp_s)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=int((stamp_s - sec) * 1e9))))


def test_inbound_stamp_is_corrected_by_the_peer_offset(node_module: ModuleType) -> None:
    topic = "/vehicle/camera/compressed"
    node = _observer(node_module, topic, direction="inbound", offset_s=0.12)

    node._make_cb(topic)(_stamped(0.03))

    obs = node.observations[topic]
    assert obs.last_delay_s == pytest.approx(0.15, abs=1e-3)
    assert obs.last_raw_delay_s == pytest.approx(0.03, abs=1e-3)
    assert obs.last_clock_offset_s == pytest.approx(0.12)
    assert obs.last_rtt_s == pytest.approx(0.04)


def test_local_stage_stamp_is_left_alone(node_module: ModuleType) -> None:
    """A stage this machine published was stamped by this machine's clock.

    Correcting it would inject the peer offset into a purely local measurement.
    """
    topic = "/camera/compressed"
    node = _observer(node_module, topic, direction="outbound", offset_s=0.12)

    node._make_cb(topic)(_stamped(0.03))

    obs = node.observations[topic]
    assert obs.last_delay_s == pytest.approx(0.03, abs=1e-3)
    assert obs.last_clock_offset_s is None
    assert obs.last_raw_delay_s is None


def test_inbound_without_an_estimate_falls_back_to_the_raw_delay(node_module: ModuleType) -> None:
    topic = "/vehicle/camera/compressed"
    node = _observer(node_module, topic, direction="inbound", offset_s=None)

    node._make_cb(topic)(_stamped(0.03))

    obs = node.observations[topic]
    assert obs.last_delay_s == pytest.approx(0.03, abs=1e-3)
    assert obs.last_clock_offset_s is None


def test_offset_rescues_a_sample_the_raw_guard_would_drop(node_module: ModuleType) -> None:
    """#258 regression: the whole point -- a peer 5 s ahead used to report nothing."""
    topic = "/vehicle/camera/compressed"
    node = _observer(node_module, topic, direction="inbound", offset_s=5.0)

    node._make_cb(topic)(_stamped(-4.9))

    obs = node.observations[topic]
    assert obs.last_delay_s == pytest.approx(0.1, abs=1e-3)


def test_a_message_without_a_header_reports_no_latency(node_module: ModuleType) -> None:
    topic = "/vehicle/tf"
    node = _observer(node_module, topic, direction="inbound", offset_s=0.12)

    node._make_cb(topic)(SimpleNamespace())

    obs = node.observations[topic]
    assert obs.last_delay_s is None
    assert obs.msg_total == 1


# ---------------------------------------------------------------------------
# the OtaStamped branch: FFMPEG keyframe flag into the transit record
# ---------------------------------------------------------------------------


def _cdr_string(offset: int, value: str) -> tuple[bytes, int]:
    pad = (-offset) % 4
    raw = value.encode() + b"\x00"
    chunk = b"\x00" * pad + struct.pack("<I", len(raw)) + raw
    return chunk, offset + len(chunk)


def _ffmpeg_packet(flags: int) -> bytes:
    """A minimal serialized FFMPEGPacket (XCDR1 little-endian)."""
    body = struct.pack("<iI", 7, 9)  # header.stamp
    chunk, offset = _cdr_string(8, "camera")
    body += chunk
    body += b"\x00" * ((-offset) % 4) + struct.pack("<ii", 640, 480)
    offset += (-offset) % 4 + 8
    chunk, offset = _cdr_string(offset, "h264")
    body += chunk
    body += b"\x00" * ((-offset) % 8) + struct.pack("<QBB", 42, flags, 0)
    return b"\x00\x01\x00\x00" + body


def _ota_observer(node_module: ModuleType, topic: str):
    node = object.__new__(node_module.StageObserver)
    node.observations = {topic: core.StageObservation(type_str="com_msgs/msg/OtaStamped")}
    node._stage_metadata = {
        topic: [
            {
                "peer": "b",
                "source": "a",
                "target": "b",
                "base": "/cam",
                "direction": "inbound",
                "stage": "com_in",
            }
        ]
    }
    node.clock_estimator = _Estimator(0.0)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(_NOW_S * 1e9)))
    node.get_logger = lambda: SimpleNamespace(error=lambda *a, **k: None, warning=lambda *a, **k: None)
    return node


def _ota_stamped(seq: int, *, msg_type: str, serialized_msg) -> SimpleNamespace:
    stamp_s = _NOW_S - 0.03
    sec = int(stamp_s)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=int((stamp_s - sec) * 1e9))),
        seq=seq,
        msg_type=msg_type,
        serialized_msg=serialized_msg,
    )


def test_ota_ffmpeg_payload_marks_the_transit_record_keyframe(node_module: ModuleType) -> None:
    topic = "/cam/ffmpeg/ota"
    node = _ota_observer(node_module, topic)
    cb = node._make_cb(topic)

    # rclpy delivers uint8[] as array('B'); the branch must take it as-is.
    cb(_ota_stamped(0, msg_type=ffmpeg_flags.FFMPEG_PACKET_TYPE, serialized_msg=array("B", _ffmpeg_packet(1))))
    cb(_ota_stamped(1, msg_type=ffmpeg_flags.FFMPEG_PACKET_TYPE, serialized_msg=_ffmpeg_packet(0)))

    records = node.observations[topic].drain_transit_records()
    assert [(record["seq"], record["keyframe"]) for record in records] == [(0, True), (1, False)]


def test_ota_non_ffmpeg_payload_gets_no_keyframe_field(node_module: ModuleType) -> None:
    topic = "/twist/ota"
    node = _ota_observer(node_module, topic)

    node._make_cb(topic)(_ota_stamped(0, msg_type="geometry_msgs/msg/Twist", serialized_msg=_ffmpeg_packet(1)))

    (record,) = node.observations[topic].drain_transit_records()
    assert record["topic"] == "/cam"
    assert "keyframe" not in record


def test_ota_unparseable_ffmpeg_payload_omits_the_flag(node_module: ModuleType) -> None:
    topic = "/cam/ffmpeg/ota"
    node = _ota_observer(node_module, topic)
    cb = node._make_cb(topic)

    cb(_ota_stamped(0, msg_type=ffmpeg_flags.FFMPEG_PACKET_TYPE, serialized_msg=b"\x00\x01\x00\x00\x07"))
    cb(_ota_stamped(1, msg_type=ffmpeg_flags.FFMPEG_PACKET_TYPE, serialized_msg=None))

    records = node.observations[topic].drain_transit_records()
    assert [record["seq"] for record in records] == [0, 1]
    assert all("keyframe" not in record for record in records)


# ---------------------------------------------------------------------------
# the sender-side com_out branch: what the source handed to the link (#293)
# ---------------------------------------------------------------------------


def _ota_out_observer(node_module: ModuleType, topic: str):
    node = object.__new__(node_module.StageObserver)
    node.observations = {topic: core.StageObservation(type_str="com_msgs/msg/OtaStamped")}
    node._stage_metadata = {
        topic: [
            {
                "peer": "a",
                "source": "a",
                "target": "b",
                "base": "/cam",
                "direction": "outbound",
                "stage": "com_out",
            }
        ]
    }
    # A live peer estimate exists; the branch must not apply it to a local hop.
    node.clock_estimator = _Estimator(5.0)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(_NOW_S * 1e9)))
    node.get_logger = lambda: SimpleNamespace(error=lambda *a, **k: None, warning=lambda *a, **k: None)
    return node


def test_com_out_ota_payload_produces_a_sent_transit_record(node_module: ModuleType) -> None:
    topic = "/com/out/a/cam/ffmpeg/ota_stamped"
    node = _ota_out_observer(node_module, topic)
    cb = node._make_cb(topic)

    cb(_ota_stamped(0, msg_type=ffmpeg_flags.FFMPEG_PACKET_TYPE, serialized_msg=_ffmpeg_packet(1)))
    cb(_ota_stamped(1, msg_type=ffmpeg_flags.FFMPEG_PACKET_TYPE, serialized_msg=_ffmpeg_packet(0)))

    records = node.observations[topic].drain_transit_records()
    assert [(r["seq"], r["status"], r["keyframe"]) for r in records] == [
        (0, "sent", True),
        (1, "sent", False),
    ]
    first = records[0]
    assert first["stage"] == "com_out"
    assert first["direction"] == "outbound"
    assert first["t_wrap"] == pytest.approx(_NOW_S - 0.03, abs=1e-6)
    assert first["t_com_out"] == pytest.approx(_NOW_S, abs=1e-6)
    assert "t_com_in" not in first
    # Wrapper and observer share one clock, so the peer offset must not leak in.
    assert first["clock_offset_ms"] is None
    assert first["sections"]["wrap_to_com_out_ms"] == pytest.approx(30.0, abs=0.5)
    assert "ota_hop_ms" not in first["sections"]


def test_com_out_sequence_gap_is_unobserved_not_lost(node_module: ModuleType) -> None:
    """A gap at com_out means the local observer missed the message, not the link."""
    topic = "/com/out/a/cam/ffmpeg/ota_stamped"
    node = _ota_out_observer(node_module, topic)
    cb = node._make_cb(topic)

    cb(_ota_stamped(0, msg_type=ffmpeg_flags.FFMPEG_PACKET_TYPE, serialized_msg=_ffmpeg_packet(1)))
    cb(_ota_stamped(3, msg_type=ffmpeg_flags.FFMPEG_PACKET_TYPE, serialized_msg=_ffmpeg_packet(0)))

    records = node.observations[topic].drain_transit_records()
    assert [(r["seq"], r["status"]) for r in records] == [
        (0, "sent"),
        (1, "unobserved"),
        (2, "unobserved"),
        (3, "sent"),
    ]
    gap = records[1]
    assert gap["t_wrap"] is None
    assert gap["t_com_out"] is None
    assert "t_com_in" not in gap
    assert gap["sections"] == {"wrap_to_com_out_ms": None}
