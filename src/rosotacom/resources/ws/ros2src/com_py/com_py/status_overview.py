#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

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
#    * Neither the name of the copyright holder nor the names of its
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
# \brief  rosotacom status / debugging overview node.
#
# Tracks, for every configured topic, the furthest pipeline stage it has
# reached and the first stage that is missing/broken, plus live metrics
# (last-message age, Hz, mean size, latency). Writes a machine-readable
# status.json (source of truth), a rendered status.txt, and an append-only
# events.jsonl (one line per state transition).
#
# Phase 1 observes only what the local peer's ROS graph exposes:
#   * local-domain stages are sampled directly for live metrics
#   * OTA-domain stages use graph metadata only and never create DataReaders;
#     activity is inferred from the adjacent local stage where possible
# End-to-end remote confirmation is reserved for a later phase; remote-side
# fields are reported as "unknown".
#
# The pure classification/rollup/rendering logic lives in
# status_overview_core (ROS-independent, unit-tested on the host); this file
# wires it to live ROS graph observers (one per ROS domain).
# ---------------------------------------------------------------------

from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading
from pathlib import Path
from typing import Dict, Optional

import yaml

import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

from com_py.ffmpeg_flags import FFMPEG_PACKET_TYPE, parse_keyframe_flag
from com_py.link_bytes import LinkByteSampler, resolve_link_interface
from com_py.link_trace import LinkTraceRecorder
from com_py.status_overview_core import (
    SCALAR_MSG_TYPE,
    STARTUP_GRACE_S,
    ClockOffsetEstimator,
    StageObservation,
    StatusAggregator,
    collect_stage_metadata,
    collect_stage_topics,
    stamp_delay,
)

ECHO_HEARTBEAT_TYPE = "com_msgs/msg/EchoHeartbeat"
OTA_STAMPED_TYPE = "com_msgs/msg/OtaStamped"


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


class StageObserver(Node):
    """
    Observes a set of stage topics within a single ROS domain context.

    Performs periodic graph introspection (publisher/subscriber counts). For
    local-domain topics it also dynamically subscribes to measure size and
    header-based latency. OTA observers are graph-only so status monitoring can
    never create an additional OTA payload stream.
    """

    def __init__(self, node_name: str, context: Optional[Context],
                 topics: Dict[str, Optional[str]], refresh_interval_s: float,
                 *, subscribe_to_messages: bool = True,
                 stage_metadata: Optional[Dict[str, list[dict]]] = None,
                 clock_estimator: Optional[ClockOffsetEstimator] = None):
        super().__init__(node_name, context=context)
        self.observations: Dict[str, StageObservation] = {
            t: StageObservation(type_str=hint, graph_only=not subscribe_to_messages)
            for t, hint in topics.items()
        }
        self._refresh_interval_s = refresh_interval_s
        self._subscribe_to_messages = subscribe_to_messages
        self._stage_metadata = stage_metadata or {}
        self.clock_estimator = clock_estimator or ClockOffsetEstimator()
        self.create_timer(self._refresh_interval_s, self._refresh)
        self._refresh()

    def _refresh(self) -> None:
        try:
            graph = {name: types for name, types in self.get_topic_names_and_types()}
        except Exception:  # pragma: no cover - defensive
            graph = {}

        for topic, obs in self.observations.items():
            try:
                obs.pub_count = self.count_publishers(topic)
                obs.sub_count = self.count_subscribers(topic)
            except Exception:  # pragma: no cover - defensive
                pass

            if not self._subscribe_to_messages:
                continue
            if obs.subscribed:
                continue
            type_str = obs.type_str
            if not type_str:
                graph_types = graph.get(topic)
                type_str = graph_types[0] if graph_types else None
            if not type_str or obs.pub_count == 0:
                continue
            try:
                msg_class = get_message(type_str)
            except Exception:
                msg_class = None
            if msg_class is None:
                continue
            qos = self._observation_qos(topic)
            try:
                self.create_subscription(msg_class, topic, self._make_cb(topic), qos)
                obs.subscribed = True
                obs.type_str = type_str
            except Exception:  # pragma: no cover - defensive
                continue

    def _observation_qos(self, topic: str) -> QoSProfile:
        """QoS for the observation subscription, matched to the live publisher.

        A VOLATILE reader only receives samples sent *after* it matches, so it
        cannot read a TRANSIENT_LOCAL writer's already-published held sample. A
        genuinely static latched topic publishes exactly once at startup, so a
        hardcoded best-effort/volatile observer turns stage observation into a
        startup race -- it intermittently misses that single sample and reports a
        false STALLED even though the pipeline delivered and durably holds the
        value. Adopting the publisher's offered durability/reliability (request ==
        offered) is always compatible and replays the durable history. Falls back
        to best-effort/volatile when no publisher QoS is available.

        Adopting the *publisher's* offered durability (rather than forcing
        TRANSIENT_LOCAL for every stage of a declared latched topic) is
        deliberate: a latched topic's own source stage -- the vehicle's native
        publish -- is VOLATILE (the ROS 1 bridge offers no durability), and a
        TRANSIENT_LOCAL reader is incompatible with a VOLATILE writer and would
        receive nothing, turning a healthy native stage into a false STALLED.
        Only the delivered stages (com_in/app_in) offer TRANSIENT_LOCAL, and the
        per-publisher match already requests it there -- which is the
        "subscribe transient_local on latched roles" of issue #231, made
        per-stage-correct. The matched subscription is observable on the graph
        (it raises the topic's subscriber count).
        """
        durability = DurabilityPolicy.VOLATILE
        reliability = ReliabilityPolicy.BEST_EFFORT
        try:
            infos = self.get_publishers_info_by_topic(topic)
        except Exception:  # pragma: no cover - defensive
            infos = []
        for info in infos:
            pub_qos = getattr(info, "qos_profile", None)
            if pub_qos is None:
                continue
            if pub_qos.durability == DurabilityPolicy.TRANSIENT_LOCAL:
                durability = DurabilityPolicy.TRANSIENT_LOCAL
            if pub_qos.reliability == ReliabilityPolicy.RELIABLE:
                reliability = ReliabilityPolicy.RELIABLE
        return QoSProfile(
            reliability=reliability,
            durability=durability,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

    def _make_cb(self, topic: str):
        def cb(msg):
            obs = self.observations[topic]
            try:
                size = len(serialize_message(msg))
            except Exception:
                size = 0
            now_ros = self.get_clock().now()
            now_ros_s = now_ros.nanoseconds / 1e9
            delay_s: Optional[float] = None
            raw_delay_s: Optional[float] = None
            rtt_s: Optional[float] = None
            clock_offset_s: Optional[float] = None
            seq: Optional[int] = None
            transit = None

            if obs.type_str == ECHO_HEARTBEAT_TYPE:
                contexts = self._stage_metadata.get(topic, [])
                is_inbound = any(
                    context.get("direction") == "inbound" for context in contexts
                )
                if is_inbound:
                    seq = int(msg.seq)
                    if bool(msg.echo_valid):
                        self.clock_estimator.update(
                            t1_s=_stamp_seconds(msg.echo_t1),
                            t2_s=_stamp_seconds(msg.echo_t2),
                            t3_s=_stamp_seconds(msg.echo_t3),
                            t4_s=now_ros_s,
                            sample_id=int(msg.echo_seq),
                        )
                    estimate = self.clock_estimator.estimate()
                    if estimate is not None:
                        clock_offset_s = estimate["offset_s"]
                        rtt_s = estimate["rtt_s"]
                        delay_s = (
                            now_ros_s
                            + clock_offset_s
                            - _stamp_seconds(msg.header.stamp)
                        )
                        if delay_s < 0.0:
                            delay_s = None
                else:
                    delay_s = now_ros_s - _stamp_seconds(msg.header.stamp)
            elif obs.type_str == OTA_STAMPED_TYPE:
                contexts = self._stage_metadata.get(topic, [])
                com_in = next(
                    (
                        context
                        for context in contexts
                        if context.get("stage") == "com_in"
                        and context.get("direction") == "inbound"
                    ),
                    None,
                )
                com_out = next(
                    (
                        context
                        for context in contexts
                        if context.get("stage") == "com_out"
                        and context.get("direction") == "outbound"
                    ),
                    None,
                )
                keyframe: Optional[bool] = None
                if com_in is not None or com_out is not None:
                    try:
                        if msg.msg_type == FFMPEG_PACKET_TYPE:
                            keyframe = parse_keyframe_flag(bytes(msg.serialized_msg))
                    except Exception:
                        keyframe = None
                if com_in is not None:
                    seq = int(msg.seq)
                    t_wrap_s = _stamp_seconds(msg.header.stamp)
                    raw_delay_s = now_ros_s - t_wrap_s
                    estimate = self.clock_estimator.estimate()
                    if estimate is not None:
                        clock_offset_s = estimate["offset_s"]
                        rtt_s = estimate["rtt_s"]
                        delay_s = now_ros_s + clock_offset_s - t_wrap_s
                    transit = {
                        "peer": com_in.get("peer"),
                        "source": com_in.get("source"),
                        "target": com_in.get("target"),
                        "topic": com_in.get("base"),
                        "direction": com_in.get("direction"),
                        "stage": com_in.get("stage"),
                        "t_wrap": t_wrap_s,
                        "t_com_in": now_ros_s,
                        "keyframe": keyframe,
                    }
                elif com_out is not None:
                    # The sending peer's own view of the same message: what the
                    # source handed to the link, before any OTA hop. The wrapper
                    # stamped `t_wrap` on THIS machine, so no peer offset applies
                    # (same rule as the generic local-stage branch below) and the
                    # delay is the wrap->com_out relay hop, not an OTA number.
                    seq = int(msg.seq)
                    t_wrap_s = _stamp_seconds(msg.header.stamp)
                    delay_s = now_ros_s - t_wrap_s
                    transit = {
                        "peer": com_out.get("peer"),
                        "source": com_out.get("source"),
                        "target": com_out.get("target"),
                        "topic": com_out.get("base"),
                        "direction": com_out.get("direction"),
                        "stage": com_out.get("stage"),
                        "t_wrap": t_wrap_s,
                        "t_com_out": now_ros_s,
                        "keyframe": keyframe,
                    }
            elif obs.type_str == SCALAR_MSG_TYPE:
                # Not a stage: a number this node reads to interpret one. The
                # pacing hold arrives here (`.../paced/hold_ms`); it carries no
                # header, so there is no latency to record, only the value.
                obs.last_value = float(msg.data)
            else:
                header = getattr(msg, "header", None)
                stamp = getattr(header, "stamp", None) if header is not None else None
                if stamp is not None:
                    try:
                        stamp_s = _stamp_seconds(stamp)
                    except Exception:
                        stamp_s = None
                    if stamp_s is not None:
                        d = now_ros_s - stamp_s
                        # An inbound stage carries a stamp written by the peer's
                        # clock, so it is comparable to ours only once the offset
                        # the heartbeat measures is applied -- exactly what the
                        # OtaStamped branch above does with `t_wrap`. A local
                        # stage was stamped by this machine and must stay raw.
                        contexts = self._stage_metadata.get(topic, [])
                        peer_stamped = any(
                            context.get("direction") == "inbound" for context in contexts
                        )
                        estimate = self.clock_estimator.estimate() if peer_stamped else None
                        if estimate is not None:
                            clock_offset_s = estimate["offset_s"]
                            rtt_s = estimate["rtt_s"]
                            raw_delay_s = d
                        delay_s = stamp_delay(d, clock_offset_s)

            obs.record(
                size,
                delay_s,
                seq=seq,
                raw_delay_s=raw_delay_s,
                rtt_s=rtt_s,
                clock_offset_s=clock_offset_s,
                transit=transit,
            )

        return cb


class StatusOverview(Node):
    """Coordinator node: owns the local-domain observer, the aggregator timer,
    and (when domains are split) a second observer running in the OTA context."""

    def __init__(self) -> None:
        super().__init__("status_overview")

        self.declare_parameter("status_spec_file", "")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("write_interval_s", 2.0)
        self.declare_parameter("liveness_window_s", 3.0)
        self.declare_parameter("stale_after_s", 3.0)
        self.declare_parameter("refresh_interval_s", 5.0)
        self.declare_parameter("delay_good_ms", 100.0)
        self.declare_parameter("delay_bad_ms", 200.0)
        # OTA link interface for link-overhead measurement (e.g. the VPN tunnel).
        # Prefer ota_local_ip (the peer's OTA address) and resolve the interface
        # from it; ota_interface is an explicit override. Both empty -> disabled
        # (the snapshot's `link` block is null).
        self.declare_parameter("ota_interface", "")
        self.declare_parameter("ota_local_ip", "")
        # Header-only capture of the OTA link, beside the link trace. Off
        # unless a session definition asks for it; it needs the same interface
        # resolution as the link sampler above and nothing else.
        self.declare_parameter("ota_pcap", False)
        self.declare_parameter("ota_pcap_snaplen", 96)
        self.declare_parameter("ota_pcap_max_mb", 2000)
        self.declare_parameter("ota_pcap_peer_filter", True)
        self.declare_parameter("ota_pcap_peer_ip", "")
        self.declare_parameter("link_trace", False)
        self.declare_parameter("link_trace_interval_s", 1.0)
        self.declare_parameter("link_trace_modem_command", "")
        self.declare_parameter("link_trace_modem_timeout_s", 2.0)
        # Grace before the one-shot startup verdict (#266). 0 disables it.
        self.declare_parameter("startup_grace_s", STARTUP_GRACE_S)

        spec_file = str(self.get_parameter("status_spec_file").value or "").strip()
        output_dir = str(self.get_parameter("output_dir").value or "").strip()
        if not spec_file:
            raise ValueError("Parameter 'status_spec_file' is required")
        if not output_dir:
            output_dir = os.path.join(os.path.dirname(spec_file), "status")

        with open(spec_file, "r", encoding="utf-8") as fp:
            self.spec = yaml.safe_load(fp) or {}

        self.write_interval_s = float(self.get_parameter("write_interval_s").value)
        self.refresh_interval_s = float(self.get_parameter("refresh_interval_s").value)

        local_domain_id = self.spec.get("local_domain_id")
        ota_domain_id = self.spec.get("ota_domain_id")
        need_ota_ctx = ota_domain_id is not None and ota_domain_id != local_domain_id

        by_domain = collect_stage_topics(self.spec)
        metadata_by_domain = collect_stage_metadata(self.spec)
        clock_estimator = ClockOffsetEstimator()

        self._ota_context: Optional[Context] = None
        self._ota_executor: Optional[MultiThreadedExecutor] = None
        self._ota_thread: Optional[threading.Thread] = None
        self._ota_observer: Optional[StageObserver] = None
        self._ota_observer_uses_separate_context = False
        observers: Dict[str, StageObserver] = {}

        # Local-domain stages are sampled for payload metrics.
        local_topics = dict(by_domain.get("local", {}))
        self._local_observer = StageObserver(
            "status_overview_local_obs",
            None,
            local_topics,
            self.refresh_interval_s,
            stage_metadata=metadata_by_domain.get("local"),
            clock_estimator=clock_estimator,
        )
        observers["local"] = self._local_observer

        # OTA stages always have a distinct graph-only observer, even if the
        # local and OTA domain IDs are equal. This makes the no-OTA-DataReader
        # guarantee independent of session topology.
        ota_topics = dict(by_domain.get("ota", {}))
        if ota_topics:
            ota_context: Optional[Context] = None
            if need_ota_ctx:
                self._ota_context = Context()
                rclpy.init(context=self._ota_context, domain_id=int(ota_domain_id))
                ota_context = self._ota_context
                self._ota_observer_uses_separate_context = True
            self._ota_observer = StageObserver(
                "status_overview_ota_obs",
                ota_context,
                ota_topics,
                self.refresh_interval_s,
                subscribe_to_messages=False,
            )
            observers["ota"] = self._ota_observer
            if self._ota_observer_uses_separate_context:
                self._ota_executor = MultiThreadedExecutor(
                    num_threads=2, context=self._ota_context
                )
                self._ota_executor.add_node(self._ota_observer)
                self._ota_thread = threading.Thread(
                    target=self._ota_executor.spin, daemon=True
                )
                self._ota_thread.start()
            self.get_logger().info(
                f"status_overview: graph-only OTA observer running in domain "
                f"{ota_domain_id} (no payload subscriptions)"
            )

        ota_interface = str(self.get_parameter("ota_interface").value or "").strip()
        ota_local_ip = str(self.get_parameter("ota_local_ip").value or "").strip()
        link_sampler = None
        if ota_interface or ota_local_ip:
            # Same resolution and the same loopback refusal as topic_monitor: a
            # link number nobody can tell apart from loopback traffic is worse
            # than no link number, so this reports nothing rather than that.
            try:
                ota_interface, provenance = resolve_link_interface(ota_interface, ota_local_ip)
            except ValueError as exc:
                self.get_logger().warning(
                    f"status_overview: {exc}; link-overhead measurement disabled"
                )
            else:
                link_sampler = LinkByteSampler(ota_interface)
                self.get_logger().info(
                    f"status_overview: link-overhead sampling on interface "
                    f"'{ota_interface}' ({provenance})"
                )

        self._pcap: Optional[subprocess.Popen] = None
        if bool(self.get_parameter("ota_pcap").value):
            self._start_ota_pcap(output_dir, ota_interface if link_sampler else "")

        link_trace_recorder = None
        if bool(self.get_parameter("link_trace").value):
            link_trace_path = os.path.join(output_dir, "link_trace.jsonl")
            link_trace_recorder = LinkTraceRecorder(
                link_trace_path,
                interval_s=float(self.get_parameter("link_trace_interval_s").value),
                modem_metrics_command=str(self.get_parameter("link_trace_modem_command").value or ""),
                modem_metrics_timeout_s=float(self.get_parameter("link_trace_modem_timeout_s").value),
            )
            self.get_logger().info(
                f"status_overview: link trace enabled; writing {link_trace_path}"
            )

        self.aggregator = StatusAggregator(
            self.get_logger(),
            self.spec,
            output_dir,
            observers,
            liveness_window_s=float(self.get_parameter("liveness_window_s").value),
            stale_after_s=float(self.get_parameter("stale_after_s").value),
            delay_good_ms=float(self.get_parameter("delay_good_ms").value),
            delay_bad_ms=float(self.get_parameter("delay_bad_ms").value),
            link_sampler=link_sampler,
            clock_estimator=clock_estimator,
            link_trace_recorder=link_trace_recorder,
            startup_grace_s=float(self.get_parameter("startup_grace_s").value),
        )

        self.create_timer(self.write_interval_s, self._on_write)
        self.get_logger().info(
            f"status_overview started: peer={self.spec.get('peer')} "
            f"topics={len(self.spec.get('topics', []))} output={output_dir}"
        )

    def add_local_nodes(self, executor: MultiThreadedExecutor) -> None:
        executor.add_node(self)
        executor.add_node(self._local_observer)
        if (
            self._ota_observer is not None
            and not self._ota_observer_uses_separate_context
        ):
            executor.add_node(self._ota_observer)

    def _start_ota_pcap(self, output_dir: str, interface: str) -> None:
        """Begin the header-only capture, or say why there is none.

        A separate process, not a thread: this node writes the status snapshot
        every interval and is the thing an operator watches, so a capture that
        misbehaves must not be able to reach it. It is still the same container
        — no capability, no image and no `docker ps` line is added by the
        option — which is the whole reason the capture lives here rather than
        beside a recorder.

        Never raises. The capture is a sidecar to the link, not a precondition
        for it, and a session that cannot capture must still run.
        """
        if not interface:
            # `ota_local_ip`/`ota_interface` are what resolve it, and the
            # resolution above already said why it failed — including the
            # loopback refusal, which is the failure worth repeating (#267).
            self.get_logger().warning(
                "status_overview: ota_pcap is enabled but no OTA interface was "
                "resolved; capturing nothing. Set shared.ota_pcap only where "
                "ota_local_ip resolves, and read the warning above for why."
            )
            return

        path = os.path.join(output_dir, "ota.pcap")
        # By file path, not `-m com_py.ota_pcap`. sudo resets the environment,
        # so PYTHONPATH does not survive and the package is not importable on
        # the other side of it — a two-host run answered exactly that,
        # "No module named 'com_py'", after the permission problem was fixed.
        # `ota_pcap.py` imports nothing but the standard library (a test pins
        # that), so a path needs no package at all.
        script = Path(__file__).resolve().with_name("ota_pcap.py")
        if not script.is_file():
            self.get_logger().warning(
                f"status_overview: ota_pcap is enabled but {script} is not here; "
                "capturing nothing"
            )
            return
        argv = [
            sys.executable, str(script),
            "--iface", interface,
            "--out", path,
            "--snaplen", str(int(self.get_parameter("ota_pcap_snaplen").value)),
            "--max-mb", str(int(self.get_parameter("ota_pcap_max_mb").value)),
            # Root opens the socket; this hands it straight back, so the pcap
            # belongs to the same user as link_trace.jsonl beside it.
            "--drop-to", f"{os.getuid()}:{os.getgid()}",
        ]
        # Narrowed to the remote peer by default. On a VPN tunnel the whole
        # device is the link and the filter changes nothing; on a LAN port it
        # is the difference between the link and every other conversation the
        # machine is having.
        peer_ip = str(self.get_parameter("ota_pcap_peer_ip").value or "").strip()
        if bool(self.get_parameter("ota_pcap_peer_filter").value) and peer_ip:
            argv += ["--peer", peer_ip]

        # An `AF_PACKET` socket needs root, and this node is `containeruser`:
        # its `CapEff` is zero, so no `--cap-add` on the container would help.
        # `containeruser` has passwordless sudo here — the NET pane's
        # `sudo iftop` already depends on it — so the capture goes through it
        # and gives root back as soon as the socket exists. Where there is no
        # sudo (this module run somewhere else, or already as root) the plain
        # command is tried instead rather than refused.
        if os.geteuid() != 0 and self._sudo_available():
            argv = ["sudo", "-n", *argv]

        try:
            self._pcap = subprocess.Popen(argv, start_new_session=True)
        except OSError as exc:  # noqa: BLE001 - a sidecar may not stop a session
            self.get_logger().warning(
                f"status_overview: ota_pcap could not start ({exc}); "
                "the session runs without a capture"
            )
            self._pcap = None
            return
        self.get_logger().info(
            f"status_overview: ota_pcap capturing {interface} -> {path}"
            + (f" (to/from {peer_ip})" if peer_ip and
               bool(self.get_parameter("ota_pcap_peer_filter").value) else "")
        )

    @staticmethod
    def _sudo_available() -> bool:
        """Can this process become root without being asked for a password?"""
        try:
            return subprocess.run(
                ["sudo", "-n", "true"], capture_output=True, timeout=5.0
            ).returncode == 0
        except (OSError, subprocess.SubprocessError):
            return False

    def _stop_ota_pcap(self) -> None:
        """SIGINT and wait, so the sidecar JSON and the drop counts are written.

        Killed instead, the pcap on disk is still readable — it is written as
        it goes — but `kernel_drops` never lands, and that is the one number
        that separates a gap in the file from a gap on the link.
        """
        child, self._pcap = self._pcap, None
        if child is None or child.poll() is not None:
            return
        try:
            # Through sudo the child is root, so an ordinary kill would be
            # refused; `sudo -n kill` is how a non-root parent signals it.
            # SIGINT and not SIGTERM, because SIGINT is what makes the capture
            # write ota.json on the way out.
            if os.geteuid() != 0:
                subprocess.run(["sudo", "-n", "kill", "-INT", str(child.pid)],
                               capture_output=True, timeout=5.0)
            else:
                child.send_signal(signal.SIGINT)
            child.wait(timeout=20.0)
        except subprocess.TimeoutExpired:
            child.kill()
            try:
                child.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                pass
        except OSError:
            pass

    def _on_write(self) -> None:
        self.aggregator.write()

    def shutdown(self) -> None:
        self._stop_ota_pcap()
        if self._ota_executor is not None:
            try:
                self._ota_executor.shutdown()
            except Exception:
                pass
        if self._ota_context is not None:
            try:
                self._ota_context.try_shutdown()
            except Exception:
                pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StatusOverview()
    executor = MultiThreadedExecutor(num_threads=4)
    node.add_local_nodes(executor)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
