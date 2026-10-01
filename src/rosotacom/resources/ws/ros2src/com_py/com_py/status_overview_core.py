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
# \brief  Pure (ROS-independent) logic for the rosotacom status overview.
#
# This module holds the per-stage observation accumulator, the stage/topic
# classification, the rollup ("where is the topic now?") and the artifact
# rendering. It deliberately has no rclpy dependency so it can be unit-tested
# on the host. status_overview.py wires it to live ROS observers.
# ---------------------------------------------------------------------

from __future__ import annotations

import datetime
import json
import os
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, Hashable, List, Optional, Tuple

# --- State vocabulary shared with heartbeat health ---
ABSENT = "ABSENT"        # no publisher and never received a message
IDLE = "IDLE"            # publisher present but no message observed yet
FLOWING = "FLOWING"      # messages within the liveness window
STALE = "STALE"          # was flowing, last message older than stale threshold

# Quality sub-classification while FLOWING
GOOD = "GOOD"
DEGRADED = "DEGRADED"
BAD = "BAD"

#: A stage whose age includes a delay this session asked for names the topic on
#: which the node responsible reports what it actually held. Set by the
#: generator for a decode that reads a playout pacer's output.
PACED_HOLD_KEY = "paced_hold_topic"
SCALAR_MSG_TYPE = "std_msgs/msg/Float64"

# Topic-level rollup
OK = "OK"
PARTIAL = "PARTIAL"
STALLED = "STALLED"


def _pct(numerator: int, denominator: int) -> float:
    return (100.0 * numerator / denominator) if denominator > 0 else 0.0


class ClockOffsetEstimator:
    """Minimum-RTT NTP-style peer-clock estimator.

    ``offset_s`` is peer clock minus local clock. A timestamp received locally
    from the peer is therefore corrected with ``local_time + offset_s`` before
    comparing it with a peer timestamp.
    """

    def __init__(self, window_s: float = 60.0) -> None:
        self.window_s = window_s
        self._samples: Deque[Tuple[float, float, float]] = deque()
        self._seen: set[Hashable] = set()
        self._seen_order: Deque[Hashable] = deque()
        self._best_rtt_s: Optional[float] = None
        self._offset_s: Optional[float] = None
        self._updated_mono: Optional[float] = None
        self._lock = threading.Lock()

    def update(
        self,
        *,
        t1_s: float,
        t2_s: float,
        t3_s: float,
        t4_s: float,
        now_mono: Optional[float] = None,
        sample_id: Optional[Hashable] = None,
    ) -> bool:
        now = time.monotonic() if now_mono is None else now_mono
        rtt_s = (t4_s - t1_s) - (t3_s - t2_s)
        if rtt_s < 0.0:
            return False
        offset_s = ((t2_s - t1_s) + (t3_s - t4_s)) / 2.0
        with self._lock:
            if sample_id is not None:
                if sample_id in self._seen:
                    return False
                self._seen.add(sample_id)
                self._seen_order.append(sample_id)
                while len(self._seen_order) > 4096:
                    self._seen.discard(self._seen_order.popleft())
            self._samples.append((now, rtt_s, offset_s))
            self._prune_locked(now)
            self._select_best_locked()
        return True

    def _prune_locked(self, now_mono: float) -> None:
        cutoff = now_mono - self.window_s
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()

    def _select_best_locked(self) -> None:
        if not self._samples:
            self._best_rtt_s = None
            self._offset_s = None
            self._updated_mono = None
            return
        at, rtt_s, offset_s = min(self._samples, key=lambda sample: sample[1])
        self._best_rtt_s = rtt_s
        self._offset_s = offset_s
        self._updated_mono = at

    def estimate(self, now_mono: Optional[float] = None) -> Optional[Dict[str, float]]:
        now = time.monotonic() if now_mono is None else now_mono
        with self._lock:
            self._prune_locked(now)
            self._select_best_locked()
            if self._offset_s is None or self._best_rtt_s is None or self._updated_mono is None:
                return None
            return {
                "offset_s": self._offset_s,
                "rtt_s": self._best_rtt_s,
                "age_s": max(0.0, now - self._updated_mono),
                "samples": float(len(self._samples)),
            }


#: Plausibility window (seconds) for a latency derived from a message header
#: stamp. Outside it the number is not a latency but an unset stamp (epoch 0) or
#: a clock difference no offset estimate explains.
STAMP_DELAY_MIN_S = -1.0
STAMP_DELAY_MAX_S = 1000.0


def stamp_delay(raw_delay_s: float, clock_offset_s: Optional[float] = None) -> Optional[float]:
    """Latency from a header stamp, corrected by the estimated peer clock offset.

    ``raw_delay_s`` is ``local_now - stamp``. ``clock_offset_s`` is peer clock
    minus local clock (as ``ClockOffsetEstimator`` reports it), so the corrected
    delay is their sum -- the same arithmetic the OtaStamped path uses on its
    ``t_wrap``.

    Returns ``None`` when the result falls outside the plausibility window. The
    guard is applied to the corrected value on purpose: a topic whose stamp is
    written by the peer used to be dropped for an offset this node has already
    measured, which reads as "no latency available" instead of "latency, once
    the clocks are reconciled".
    """
    corrected = raw_delay_s if clock_offset_s is None else raw_delay_s + clock_offset_s
    if STAMP_DELAY_MIN_S < corrected < STAMP_DELAY_MAX_S:
        return corrected
    return None


#: How long a peer is given to bring its own publishers up before their absence
#: is called a defect. Node start, DDS discovery and the first `refresh_interval`
#: of the observers all happen inside it; 20 s is roughly four times what a
#: healthy bring-up needs.
STARTUP_GRACE_S = 20.0


#: Stage producers rosotacom starts itself, and can therefore promise. The
#: application that feeds a link is the user's, and a session that declares a
#: topic says nothing about whether its publisher is running right now -- a
#: smoke run legitimately exercises the peers without the data source.
ROSOTACOM_STAGE_PRODUCERS = frozenset({"heartbeat_echo", "preprocessing", "relay_out", "bridge_out"})


def outbound_startup_gaps(snapshot: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Where each outbound topic first has no publisher at all, and whose fault it is.

    Only outbound topics: their stages are produced on this machine. Inbound
    stages depend on the peer and the link, which the per-topic rollup already
    diagnoses.

    Only the *first* absent stage per topic, because the ones behind it are
    absent for the trivial reason that nothing reached them, and reporting a
    cascade as four findings buries the one that matters.

    `owner` is what makes the finding actionable, and it takes three values that
    look identical in the graph:

    * ``rosotacom`` -- a stage this peer starts itself, whose input is arriving.
      It had something to forward and forwarded nothing. This is the defect.
    * ``application`` -- the native stage of a topic the user's application (or a
      replay) publishes. Not this peer's promise.
    * ``upstream`` -- a stage this peer starts whose input has not delivered a
      message yet. Its publisher appears on discovery of the input, so there is
      nothing wrong: `universal_ota_wrapper` has no `…/ota_stamped` publisher
      until something publishes the topic it wraps.

    This exists because of a failure that produced no error anywhere. On
    2026-08-13 the centre's `heartbeat_echo` and one `topic_monitor` died two
    seconds after start -- Cyclone had no free participant index left on the
    local domain -- and catmux left both panes sitting at a shell prompt. The
    session ran for two hours with no heartbeat publisher, so the run has no RTT
    and no clock-offset measurement, and the far side reported `status=LOST
    reason=age hz=0.00` about a peer that was otherwise healthy. The heartbeat is
    the one outbound topic rosotacom publishes itself, which is why the
    generator marks its native stage `heartbeat_echo` rather than `application`.
    """
    gaps: List[Dict[str, Any]] = []
    for topic in snapshot.get("topics", []):
        if topic.get("direction") != "outbound":
            continue
        upstream: Optional[Dict[str, Any]] = None
        for stage in topic.get("stages", []):
            if stage.get("state") != ABSENT:
                upstream = stage
                continue
            producer = stage.get("produced_by")
            if producer not in ROSOTACOM_STAGE_PRODUCERS:
                owner = "application"
                reason = "the application that publishes this topic is not running"
            elif upstream is not None and upstream.get("state") not in (FLOWING, STALE):
                owner = "upstream"
                reason = (
                    f"nothing has arrived on {upstream.get('topic')} "
                    f"(stage {upstream.get('stage')}), so there is nothing to publish yet"
                )
            else:
                owner = "rosotacom"
                reason = f"{producer} did not come up"
            gaps.append(
                {
                    "base": topic.get("base"),
                    "stage": stage.get("stage"),
                    "topic": stage.get("topic"),
                    "domain": stage.get("domain"),
                    "produced_by": producer,
                    "owner": owner,
                    "reason": reason,
                }
            )
            break
    return gaps


#: How far back a sequence number may jump and still be called reordering.
#: `universal_ota_wrapper` counts per topic from zero and increments by one, so
#: a receiver sees a backward jump only when the wire reordered a message or
#: when the publishing peer restarted. Reordering is bounded by the transport's
#: history depth (single digits for the QoS this stack uses); a restart lands
#: thousands of sequence numbers back. 64 sits between the two by two orders of
#: magnitude on both sides.
SEQUENCE_REORDER_TOLERANCE = 64


def is_sequence_epoch_reset(
    seq: int,
    expected_next_seq: Optional[int],
    tolerance: int = SEQUENCE_REORDER_TOLERANCE,
) -> bool:
    """Does ``seq`` begin a new sequence epoch (the peer's wrapper restarted)?

    Zero is unambiguous and was the only case handled before. It is also the
    case that almost never arrives: DDS discovery takes long enough that the
    first tens of messages of a fresh publisher are written before the reader
    has matched. In the 2026-08-13 field instance the peer restarted four times
    and the first arrival after each restart carried seq 37, 39, 39 and 41 --
    never 0. Every one of them was therefore counted as `reordered`, and
    because the baseline was never reset, so was every message after it: 26,453
    of 55,437 transits on one 10 Hz topic, for the rest of the instance.

    So a large backward jump counts as a restart too. It is corroborated by an
    arrival gap (105 s, 32 s, 44 s and 195 s in that instance), but the gap is
    not required: a fast restart is still a restart, and the jump alone already
    separates the two causes by orders of magnitude.
    """
    if expected_next_seq is None or seq >= expected_next_seq:
        return False
    if seq == 0:
        return True
    return (expected_next_seq - seq) > tolerance


# --- Link overhead (session-level) -----------------------------------------

def _payload_kbps(stage: Optional[Dict[str, Any]]) -> float:
    """Application payload bandwidth at a stage: mean size x rate, in Kbit/s."""
    if not stage or stage.get("state") not in (FLOWING, STALE):
        return 0.0
    size = float(stage.get("mean_size_bytes") or 0.0)
    hz = float(stage.get("hz") or 0.0)
    return size * 8.0 / 1024.0 * hz


def _stage_named(topic: Dict[str, Any], stage_name: str) -> Optional[Dict[str, Any]]:
    for s in topic.get("stages") or []:
        if s.get("stage") == stage_name:
            return s
    return None


def compute_link_overview(
    topics: List[Dict[str, Any]], link_sample: Optional[Dict[str, Any]]
) -> Optional[Dict[str, Any]]:
    """Session-level link overhead: wire bandwidth (from a LinkByteSampler) vs the
    ROS application payload crossing the OTA boundary -- summed from the per-stage
    sizes/rates the overview *already* measures, so there is no second measurement
    path. Returns None when no link sample is available.

    Directional: outbound payload is summed at each topic's `com_out` stage (the
    last local stage before bridge_out -> OTA) and compared with the interface tx
    rate; inbound payload at `com_in` (just after bridge_in) vs the rx rate. The
    ratio link/payload is ~1 when the wire carries little beyond the payload and
    grows with retransmits / shadow connections / discovery chatter / bad QoS.
    (Small messages have a naturally higher ratio from per-packet RTPS/UDP/IP
    headers, so the assertion in status_eval is an upper bound, not equality.)
    """
    if not link_sample:
        return None
    payload_out = sum(
        _payload_kbps(_stage_named(t, "com_out")) for t in topics if t.get("direction") == "outbound"
    )
    payload_in = sum(
        _payload_kbps(_stage_named(t, "com_in")) for t in topics if t.get("direction") == "inbound"
    )
    tx = float(link_sample.get("tx_kbps") or 0.0)
    rx = float(link_sample.get("rx_kbps") or 0.0)

    def _ratio(link_kbps: float, payload: float) -> Optional[float]:
        return round(link_kbps / payload, 3) if payload > 0.0 else None

    return {
        "interface": link_sample.get("interface"),
        "window_s": round(float(link_sample.get("window_s") or 0.0), 3),
        "link_tx_kbps": round(tx, 3),
        "link_rx_kbps": round(rx, 3),
        "ros_payload_out_kbps": round(payload_out, 3),
        "ros_payload_in_kbps": round(payload_in, 3),
        "overhead_ratio_out": _ratio(tx, payload_out),
        "overhead_ratio_in": _ratio(rx, payload_in),
    }


@dataclass
class StageObservation:
    """Live observation of a single stage topic within one ROS domain."""

    type_str: Optional[str] = None
    graph_only: bool = False
    subscribed: bool = False
    pub_count: int = 0
    sub_count: int = 0
    msg_total: int = 0
    last_recv_mono: Optional[float] = None
    last_recv_wall: Optional[float] = None
    last_delay_s: Optional[float] = None
    last_raw_delay_s: Optional[float] = None
    #: Payload of the last message, for the scalar topics this node reads as
    #: numbers rather than observes as stages (the pacing hold).
    last_value: Optional[float] = None
    last_rtt_s: Optional[float] = None
    last_clock_offset_s: Optional[float] = None
    # (t_mono, size_bytes, delay_s_or_None)
    events: Deque[Tuple[float, int, Optional[float]]] = field(default_factory=deque)
    # (t_mono, expected_delta, missing_delta, reordered_delta, burst_missing)
    sequence_events: Deque[Tuple[float, int, int, int, int]] = field(default_factory=deque)
    expected_next_seq: Optional[int] = None
    last_seq: Optional[int] = None
    tracks_sequence: bool = False
    epoch: int = 0
    epoch_resets: int = 0
    total_expected: int = 0
    total_missing: int = 0
    total_reordered: int = 0
    max_burst_missing: int = 0
    transit_records: Deque[Dict[str, Any]] = field(default_factory=deque)
    last_transit_recv_wall: Optional[float] = None
    last_inter_arrival_s: Optional[float] = None
    lock: threading.Lock = field(default_factory=threading.Lock)

    def record(
        self,
        size: int,
        delay_s: Optional[float],
        now_mono: Optional[float] = None,
        now_wall: Optional[float] = None,
        *,
        seq: Optional[int] = None,
        raw_delay_s: Optional[float] = None,
        rtt_s: Optional[float] = None,
        clock_offset_s: Optional[float] = None,
        transit: Optional[Dict[str, Any]] = None,
    ) -> None:
        now = time.monotonic() if now_mono is None else now_mono
        wall = time.time() if now_wall is None else now_wall
        with self.lock:
            self.msg_total += 1
            self.last_recv_mono = now
            self.last_recv_wall = wall
            self.last_delay_s = delay_s
            self.last_raw_delay_s = raw_delay_s
            self.last_rtt_s = rtt_s
            self.last_clock_offset_s = clock_offset_s
            self.events.append((now, size, delay_s))
            sequence_status = "delivered"
            missing_start: Optional[int] = None
            missing = 0
            reordered = 0
            expected_delta = 0
            if seq is not None:
                self.tracks_sequence = True
                seq = int(seq)
                if self.expected_next_seq is None:
                    expected_delta = 1
                    self.expected_next_seq = seq + 1
                elif is_sequence_epoch_reset(seq, self.expected_next_seq):
                    # A publisher restart begins a new sequence epoch: the
                    # wrapper counts from zero again. See
                    # `is_sequence_epoch_reset` for why the first arrival after
                    # a restart is almost never seq 0, and what that cost.
                    self.epoch += 1
                    self.epoch_resets += 1
                    expected_delta = 1
                    self.expected_next_seq = seq + 1
                elif seq >= self.expected_next_seq:
                    missing = seq - self.expected_next_seq
                    missing_start = self.expected_next_seq if missing else None
                    expected_delta = 1 + missing
                    self.expected_next_seq = seq + 1
                else:
                    reordered = 1
                    sequence_status = "reordered"
                self.last_seq = seq
                self.total_expected += expected_delta
                self.total_missing += missing
                self.total_reordered += reordered
                self.max_burst_missing = max(self.max_burst_missing, missing)
                self.sequence_events.append((now, expected_delta, missing, reordered, missing))

            if transit is not None and seq is not None:
                # A sender-side com_out row observes its own machine's wrapper:
                # a sequence gap there means the local observer missed the
                # message, not that the link lost it, and an observed message
                # was *sent*, not delivered. The distinct status values also
                # keep the offline join honest: `join_transit_records` ranks
                # only lost < reordered < delivered, so a sender row can never
                # overwrite the receiving side's `lost` verdict for the same
                # (source, topic, epoch, seq).
                outbound = transit.get("direction") == "outbound"
                common = {
                    "kind": "transit",
                    "peer": transit.get("peer"),
                    "source": transit.get("source"),
                    "target": transit.get("target"),
                    "topic": transit.get("topic"),
                    "direction": transit.get("direction"),
                    "stage": transit.get("stage"),
                    # Which run of the peer's wrapper produced this sequence
                    # number. Without it the offline join collapses seq 0..N of
                    # every epoch after the first onto the first epoch's rows.
                    "epoch": self.epoch,
                }
                if missing_start is not None:
                    gap_time_field = "t_com_out" if outbound else "t_com_in"
                    for missing_seq in range(missing_start, int(seq)):
                        self.transit_records.append(
                            {
                                **common,
                                "seq": missing_seq,
                                "status": "unobserved" if outbound else "lost",
                                "t_wrap": None,
                                gap_time_field: None,
                                "clock_offset_ms": None,
                                "sections": (
                                    {"wrap_to_com_out_ms": None}
                                    if outbound
                                    else {"ota_hop_ms": None}
                                ),
                                "size_bytes": None,
                                "inter_arrival_ms": None,
                                "jitter_ms": None,
                            }
                        )
                inter_arrival_s = (
                    wall - self.last_transit_recv_wall
                    if self.last_transit_recv_wall is not None
                    else None
                )
                jitter_s = (
                    abs(inter_arrival_s - self.last_inter_arrival_s)
                    if inter_arrival_s is not None and self.last_inter_arrival_s is not None
                    else None
                )
                if inter_arrival_s is not None:
                    self.last_inter_arrival_s = inter_arrival_s
                self.last_transit_recv_wall = wall
                if outbound:
                    # Same clock on both ends of this hop, so there is no
                    # corrected/uncorrected pair and no OTA section: the only
                    # honest number is the wrap->com_out relay hop.
                    status = "sent"
                    time_fields = {"t_com_out": transit.get("t_com_out")}
                    sections: Dict[str, Any] = {
                        "wrap_to_com_out_ms": (
                            round(delay_s * 1000.0, 3) if delay_s is not None else None
                        ),
                    }
                else:
                    status = sequence_status
                    time_fields = {"t_com_in": transit.get("t_com_in")}
                    sections = {
                        "ota_hop_ms": (
                            round(delay_s * 1000.0, 3) if delay_s is not None else None
                        ),
                        "ota_hop_uncorrected_ms": (
                            round(raw_delay_s * 1000.0, 3)
                            if raw_delay_s is not None
                            else None
                        ),
                    }
                row = {
                    **common,
                    "seq": int(seq),
                    "status": status,
                    "t_wrap": transit.get("t_wrap"),
                    **time_fields,
                    "clock_offset_ms": (
                        round(clock_offset_s * 1000.0, 3)
                        if clock_offset_s is not None
                        else None
                    ),
                    "sections": sections,
                    "size_bytes": size,
                    "inter_arrival_ms": (
                        round(inter_arrival_s * 1000.0, 3)
                        if inter_arrival_s is not None
                        else None
                    ),
                    "jitter_ms": (
                        round(jitter_s * 1000.0, 3) if jitter_s is not None else None
                    ),
                }
                # The FFMPEG keyframe bit (AV_PKT_FLAG_KEY) exists only for
                # recognized FFMPEGPacket payloads; other streams' rows omit
                # the field rather than carrying a null on every message.
                keyframe = transit.get("keyframe")
                if keyframe is not None:
                    row["keyframe"] = bool(keyframe)
                self.transit_records.append(row)

    def metrics(self, now_mono: float, window_s: float) -> Dict[str, Any]:
        with self.lock:
            cutoff = now_mono - window_s
            while self.events and self.events[0][0] < cutoff:
                self.events.popleft()
            while self.sequence_events and self.sequence_events[0][0] < cutoff:
                self.sequence_events.popleft()
            count = len(self.events)
            total_bytes = sum(e[1] for e in self.events)
            delays = [e[2] for e in self.events if e[2] is not None]
            expected = sum(e[1] for e in self.sequence_events)
            missing = sum(e[2] for e in self.sequence_events)
            reordered = sum(e[3] for e in self.sequence_events)
            max_burst_missing = max((e[4] for e in self.sequence_events), default=0)
            last_recv_mono = self.last_recv_mono
            last_recv_wall = self.last_recv_wall
            last_delay = self.last_delay_s
            last_raw_delay = self.last_raw_delay_s
            last_rtt = self.last_rtt_s
            last_clock_offset = self.last_clock_offset_s
            msg_total = self.msg_total
            tracks_sequence = self.tracks_sequence
            total_expected = self.total_expected
            total_missing = self.total_missing
            total_reordered = self.total_reordered
            total_max_burst = self.max_burst_missing
            epoch = self.epoch
            epoch_resets = self.epoch_resets
        hz = count / window_s if window_s > 0 else 0.0
        mean_size = (total_bytes / count) if count > 0 else 0.0
        age = (now_mono - last_recv_mono) if last_recv_mono is not None else None
        return {
            "hz": hz,
            "mean_size_bytes": mean_size,
            "last_delay_s": last_delay,
            "last_raw_delay_s": last_raw_delay,
            "last_rtt_s": last_rtt,
            "last_clock_offset_s": last_clock_offset,
            "delays_in_window": delays,
            "age_s": age,
            "last_recv_wall": last_recv_wall,
            "msg_total": msg_total,
            "loss_pct": _pct(missing, expected) if tracks_sequence else None,
            "reordered": reordered if tracks_sequence else None,
            "max_burst_missing": max_burst_missing if tracks_sequence else None,
            "loss_total_pct": _pct(total_missing, total_expected) if tracks_sequence else None,
            "reordered_total": total_reordered if tracks_sequence else None,
            "max_burst_missing_total": total_max_burst if tracks_sequence else None,
            "epoch": epoch if tracks_sequence else None,
            "epoch_resets": epoch_resets if tracks_sequence else None,
        }

    def drain_transit_records(self) -> List[Dict[str, Any]]:
        with self.lock:
            records = list(self.transit_records)
            self.transit_records.clear()
        return records


class StatusAggregator:
    """
    Merges observations from the per-domain observers with the static pipeline
    spec into the unified status artifacts.

    ``observers_by_domain`` maps a domain key ("local"/"ota") to any object that
    exposes an ``observations`` dict mapping topic name -> StageObservation.
    """

    def __init__(self, logger, spec: Dict[str, Any], output_dir: str,
                 observers_by_domain: Dict[str, Any],
                 *, liveness_window_s: float = 3.0, stale_after_s: float = 3.0,
                 delay_good_ms: float = 100.0, delay_bad_ms: float = 200.0,
                 link_sampler: Any = None, clock_estimator: Any = None,
                 link_trace_recorder: Any = None,
                 startup_grace_s: float = STARTUP_GRACE_S,
                 started_mono: Optional[float] = None):
        self._log = logger
        self._spec = spec
        self._output_dir = output_dir
        self._observers = observers_by_domain
        self._liveness_window_s = liveness_window_s
        self._stale_after_s = stale_after_s
        self._delay_good_ms = delay_good_ms
        self._delay_bad_ms = delay_bad_ms
        # Optional object with a .sample() -> {interface, rx_kbps, tx_kbps,
        # window_s} | None (a link_bytes.LinkByteSampler). Injected so the core
        # stays free of any I/O and is unit-testable.
        self._link_sampler = link_sampler
        self._clock_estimator = clock_estimator
        self._link_trace_recorder = link_trace_recorder
        self._prev_states: Dict[str, Dict[str, Any]] = {}
        os.makedirs(self._output_dir, exist_ok=True)
        self._json_path = os.path.join(self._output_dir, "status.json")
        self._txt_path = os.path.join(self._output_dir, "status.txt")
        self._events_path = os.path.join(self._output_dir, "events.jsonl")
        self._startup_path = os.path.join(self._output_dir, "startup_check.json")
        # `pane_failures.log` is written by catmux_log_setup.sh one level up, in
        # logs/<peer>/. The verdict quotes it because a pane that exited is the
        # most common reason a promised publisher never appears, and the two
        # facts are useless apart.
        self._pane_failure_path = os.path.join(
            os.path.dirname(os.path.abspath(self._output_dir)), "pane_failures.log"
        )
        self._startup_grace_s = startup_grace_s
        self._started_mono = time.monotonic() if started_mono is None else started_mono
        self._startup_check: Optional[Dict[str, Any]] = None
        #: (when, topics) of the first evaluation that found a defect. A verdict
        #: is only committed once the same defect survives a second look; see
        #: `_maybe_run_startup_check`.
        self._startup_pending: Optional[Tuple[float, set]] = None

    # -- observation lookup --
    def _pacing_hold_ms(self, stage: Dict[str, Any]) -> Optional[float]:
        """How long this session deliberately held the last message of a stage.

        None whenever the answer is not actually known -- no pacer named, or the
        pacer has not reported yet. Guessing zero would read a paced picture as
        a slow link; guessing the configured budget would do the opposite and
        credit a hold that a late message never received.
        """
        hold_topic = stage.get(PACED_HOLD_KEY)
        if not hold_topic:
            return None
        observer = self._observers.get("local")
        observation = observer.observations.get(str(hold_topic)) if observer else None
        return None if observation is None else observation.last_value

    def _obs_for(self, stage: Dict[str, Any]) -> Optional[StageObservation]:
        domain = stage.get("domain", "local")
        observer = self._observers.get(domain) or self._observers.get("local")
        if observer is None:
            return None
        return observer.observations.get(stage["topic"])

    # -- classification --
    def classify_stage(self, stage: Dict[str, Any], expected_hz: Optional[float],
                       now_mono: float, expect: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        obs = self._obs_for(stage)
        result: Dict[str, Any] = {
            "stage": stage["stage"],
            "topic": stage["topic"],
            "domain": stage.get("domain", "local"),
            "produced_by": stage.get("produced_by"),
            "publishers": 0,
            "subscribers": 0,
            "hz": 0.0,
            "mean_size_bytes": 0.0,
            "latency_ms": None,
            "latency_uncorrected_ms": None,
            "rtt_ms": None,
            "clock_offset_ms": None,
            "loss_pct": None,
            "loss_total_pct": None,
            "reordered": None,
            "reordered_total": None,
            "max_burst_missing": None,
            "max_burst_missing_total": None,
            "epoch": None,
            "epoch_resets": None,
            "age_s": None,
            "last_message_wall": None,
            "messages_total": 0,
            "state": ABSENT,
            "quality": None,
            "quality_reason": None,
            "observation": "graph" if obs is not None and obs.graph_only else "payload",
            "inferred_from": None,
            "latched": str((expect or {}).get("mode", "")).strip().lower() == "latched",
            # Split of `latency_ms` for a stage this session delays on purpose.
            "pacing_hold_ms": None,
            "link_latency_ms": None,
        }
        if obs is None:
            return result

        m = obs.metrics(now_mono, self._liveness_window_s)
        result["publishers"] = obs.pub_count
        result["subscribers"] = obs.sub_count
        result["hz"] = round(m["hz"], 3)
        result["mean_size_bytes"] = round(m["mean_size_bytes"], 1)
        result["messages_total"] = m["msg_total"]
        result["age_s"] = round(m["age_s"], 3) if m["age_s"] is not None else None
        if m["last_delay_s"] is not None:
            result["latency_ms"] = round(m["last_delay_s"] * 1000.0, 1)
            hold_ms = self._pacing_hold_ms(stage)
            if hold_ms is not None:
                result["pacing_hold_ms"] = round(hold_ms, 1)
                result["link_latency_ms"] = round(max(0.0, result["latency_ms"] - hold_ms), 1)
        if m["last_raw_delay_s"] is not None:
            result["latency_uncorrected_ms"] = round(m["last_raw_delay_s"] * 1000.0, 1)
        if m["last_rtt_s"] is not None:
            result["rtt_ms"] = round(m["last_rtt_s"] * 1000.0, 1)
        if m["last_clock_offset_s"] is not None:
            result["clock_offset_ms"] = round(m["last_clock_offset_s"] * 1000.0, 3)
        for key in (
            "loss_pct",
            "loss_total_pct",
            "reordered",
            "reordered_total",
            "max_burst_missing",
            "max_burst_missing_total",
            "epoch",
            "epoch_resets",
        ):
            result[key] = m[key]
        if m["last_recv_wall"] is not None:
            result["last_message_wall"] = datetime.datetime.fromtimestamp(
                m["last_recv_wall"]
            ).isoformat(timespec="milliseconds")

        if m["msg_total"] == 0:
            result["state"] = IDLE if obs.pub_count > 0 else ABSENT
            return result

        age = m["age_s"]
        if age is not None and age > self._stale_after_s:
            result["state"] = STALE
            return result

        result["state"] = FLOWING
        quality, reason = self._classify_quality(
            m, expected_hz, expect, pacing_hold_ms=result["pacing_hold_ms"]
        )
        result["quality"] = quality
        result["quality_reason"] = reason
        return result

    @staticmethod
    def _copy_inferred_activity(target: Dict[str, Any], source: Dict[str, Any]) -> None:
        """Copy payload-derived activity while retaining OTA graph counts/topic."""
        for key in (
            "hz",
            "mean_size_bytes",
            "latency_ms",
            "latency_uncorrected_ms",
            "rtt_ms",
            "clock_offset_ms",
            "loss_pct",
            "loss_total_pct",
            "reordered",
            "reordered_total",
            "max_burst_missing",
            "max_burst_missing_total",
            "epoch",
            "epoch_resets",
            "age_s",
            "last_message_wall",
            "messages_total",
            "state",
            "quality",
            "quality_reason",
        ):
            target[key] = source[key]
        target["inferred_from"] = source["topic"]

    def infer_graph_only_ota_stages(
        self, topic_spec: Dict[str, Any], stage_results: List[Dict[str, Any]]
    ) -> None:
        """
        Infer OTA activity without creating an OTA DataReader.

        Outbound activity requires both an OTA publisher endpoint and flowing
        input at the preceding local stage. Inbound receipt is proven by flow at
        the following local stage, which can only exist after the OTA sample has
        passed through bridge_in/domain_bridge.
        """
        direction = topic_spec.get("direction")
        for idx, stage_result in enumerate(stage_results):
            if (
                stage_result.get("domain") != "ota"
                or stage_result.get("observation") != "graph"
            ):
                continue

            source: Optional[Dict[str, Any]] = None
            if direction == "outbound":
                if stage_result["publishers"] == 0 or idx == 0:
                    continue
                source = stage_results[idx - 1]
            elif direction == "inbound" and idx + 1 < len(stage_results):
                source = stage_results[idx + 1]

            if source is not None and source["state"] in (FLOWING, STALE):
                self._copy_inferred_activity(stage_result, source)

    def _classify_quality(
        self,
        m: Dict[str, Any],
        expected_hz: Optional[float],
        expect: Optional[Dict[str, Any]] = None,
        *,
        pacing_hold_ms: Optional[float] = None,
    ) -> Tuple[str, Optional[str]]:
        expect = expect or {}
        hz_exp = expect.get("hz") or {}
        lat_exp = expect.get("latency_ms") or {}
        loss_exp = expect.get("loss_pct") or {}

        # Latency: a declared `expect.latency_ms.max` overrides the global bad
        # threshold; the good band stays the default so a tight contract still
        # surfaces a DEGRADED before BAD where it makes sense.
        #
        # A stage the session delays on purpose is judged on what is left after
        # that delay is taken back out. Otherwise a threshold calibrated for a
        # link measures a buffer, which reads as a fault on a healthy stream and
        # teaches an operator to stop believing the panel. The subtracted value
        # is the hold actually applied, so a late message -- which the pacer
        # passes straight through -- is credited nothing and still flags.
        bad_lat = float(lat_exp["max"]) if "max" in lat_exp else self._delay_bad_ms
        delay_ms = (m["last_delay_s"] * 1000.0) if m["last_delay_s"] is not None else None
        if delay_ms is not None and pacing_hold_ms is not None:
            delay_ms = max(0.0, delay_ms - pacing_hold_ms)
        if delay_ms is not None:
            if delay_ms >= bad_lat:
                return BAD, "latency"
            if delay_ms > self._delay_good_ms:
                return DEGRADED, "latency"

        loss_pct = m.get("loss_pct")
        if loss_pct is not None and "max" in loss_exp:
            if loss_pct > float(loss_exp["max"]):
                return BAD, "loss"

        # Hz: a declared `expect.hz` {min,max} is a hard contract; otherwise fall
        # back to the derived expected_hz heuristic.
        hz = m["hz"]
        if "min" in hz_exp or "max" in hz_exp:
            if "min" in hz_exp and hz < float(hz_exp["min"]):
                return BAD, "hz"
            if "max" in hz_exp and hz > float(hz_exp["max"]):
                return BAD, "hz"
        elif expected_hz and expected_hz > 0:
            if hz < 0.5 * expected_hz:
                return BAD, "hz"
            if hz < 0.8 * expected_hz:
                return DEGRADED, "hz"
        return GOOD, None

    # -- rollup --
    def rollup(self, topic_spec: Dict[str, Any], stage_results: List[Dict[str, Any]]) -> Dict[str, Any]:
        states = [s["state"] for s in stage_results]
        n = len(stage_results)
        seen_idx = [i for i, st in enumerate(states) if st in (FLOWING, STALE)]
        last_seen = max(seen_idx) if seen_idx else -1
        any_publisher = any(s["publishers"] > 0 for s in stage_results)

        reached_stage = stage_results[last_seen]["stage"] if last_seen >= 0 else None
        blocked_at = None
        next_topic = None
        diagnosis = ""

        final_idx = n - 1
        if n == 0:
            overall = ABSENT
        elif last_seen == final_idx and states[final_idx] == FLOWING:
            overall = OK
            diagnosis = f"all observable stages flowing (reached '{reached_stage}')"
        elif last_seen == -1:
            blocked = stage_results[0]
            blocked_at = blocked["stage"]
            next_topic = blocked["topic"]
            overall = ABSENT if not any_publisher else STALLED
            diagnosis = self._stage_diagnosis(blocked)
        else:
            reached = stage_results[last_seen]
            if reached["state"] == STALE:
                overall = STALLED
                age = reached.get("age_s")
                diagnosis = (
                    f"'{reached_stage}' ({reached['topic']}) stopped"
                    + (f" {age:.1f}s ago" if isinstance(age, (int, float)) else "")
                )
                blocked_at = reached["stage"]
                next_topic = reached["topic"]
            else:
                overall = PARTIAL
                blocked = stage_results[last_seen + 1]
                blocked_at = blocked["stage"]
                next_topic = blocked["topic"]
                diagnosis = (
                    f"reached '{reached_stage}', but '{blocked_at}' is not getting it: "
                    + self._stage_diagnosis(blocked)
                )

        # Mode-aware reinterpretation (mirrors status_eval.evaluate_report so the
        # live display agrees with `rosotacom test`). A latched/static topic that
        # delivered its value and now idles is OK, not STALLED; an existence-only
        # topic is OK as soon as it is present in the graph. See expectation concepts reference.
        expect = topic_spec.get("expect") or {}
        mode = str(expect.get("mode", "stream")).strip().lower() or "stream"
        if overall != OK and mode in ("latched", "existence"):
            if mode == "latched":
                # Receiver (inbound): the held value must have reached the final
                # stage. Sender (outbound): a one-shot held value need only have
                # been produced/latched -- its OTA send is not continuously observed.
                if topic_spec.get("direction") == "outbound":
                    relaxed = any(s["state"] in (FLOWING, STALE) for s in stage_results)
                    msg = "latched value produced and held; not expected to tick"
                else:
                    relaxed = bool(stage_results) and stage_results[-1]["state"] in (FLOWING, STALE)
                    msg = f"latched value delivered (reached '{reached_stage}'); not expected to tick"
                if relaxed:
                    overall = OK
                    blocked_at = None
                    next_topic = None
                    diagnosis = msg
            elif mode == "existence" and any_publisher:
                overall = OK
                blocked_at = None
                next_topic = None
                diagnosis = "present in graph (existence)"

        if topic_spec.get("direction") == "outbound" and overall == OK:
            diagnosis += " | OTA activity inferred locally; remote delivery not observed (Phase 1)"

        inferred = [s for s in stage_results if s.get("inferred_from")]
        if inferred:
            detail = ", ".join(
                f"{s['stage']} from {s['inferred_from']}" for s in inferred
            )
            diagnosis += f" | inferred without OTA payload subscription: {detail}"

        return {
            "overall": overall,
            "reached_stage": reached_stage,
            "blocked_at": blocked_at,
            "next_missing_topic": next_topic,
            "diagnosis": diagnosis,
        }

    def _stage_diagnosis(self, stage_result: Dict[str, Any]) -> str:
        st = stage_result["state"]
        topic = stage_result["topic"]
        producer = stage_result.get("produced_by") or "producer"
        if st == ABSENT:
            return f"{producer} is not producing {topic} (no publisher)"
        if st == IDLE:
            if stage_result.get("observation") == "graph":
                return (
                    f"{topic} has a publisher; payload is intentionally not "
                    "subscribed and adjacent local flow is not observed"
                )
            if stage_result.get("latched"):
                # The third state issue #231 names: distinct from "no publisher"
                # (ABSENT) and "stopped" (STALE). The observation subscribes
                # transient_local for latched roles, so a value published before
                # it matched would have been replayed; holding none means nothing
                # was ever latched (empty/broken latch), not a late-joining reader.
                return (
                    f"{topic} has a publisher but holds no retained latched "
                    "value (nothing was latched, or the latch was lost)"
                )
            return f"{topic} has a publisher but no messages observed yet"
        if st == STALE:
            return f"{topic} stopped receiving messages"
        return f"{topic}: {st}"

    # -- snapshot + write --
    def build_snapshot(
        self, now_mono: Optional[float] = None, link_sample: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        if now_mono is None:
            now_mono = time.monotonic()
        now_wall = datetime.datetime.now().isoformat(timespec="milliseconds")
        topics_out: List[Dict[str, Any]] = []
        counts = {OK: 0, PARTIAL: 0, STALLED: 0, ABSENT: 0}

        for topic_spec in self._spec.get("topics", []):
            expected_hz = topic_spec.get("expected_hz")
            expect = topic_spec.get("expect")
            stage_results = [
                self.classify_stage(stage, expected_hz, now_mono, expect)
                for stage in topic_spec.get("stages", [])
            ]
            self.infer_graph_only_ota_stages(topic_spec, stage_results)
            roll = self.rollup(topic_spec, stage_results)
            counts[roll["overall"]] = counts.get(roll["overall"], 0) + 1
            topics_out.append(
                {
                    "base": topic_spec.get("base"),
                    "direction": topic_spec.get("direction"),
                    "source": topic_spec.get("source"),
                    "target": topic_spec.get("target"),
                    "type": topic_spec.get("type"),
                    "expected_hz": expected_hz,
                    "expect": expect,
                    "overall": roll["overall"],
                    "reached_stage": roll["reached_stage"],
                    "blocked_at": roll["blocked_at"],
                    "next_missing_topic": roll["next_missing_topic"],
                    "diagnosis": roll["diagnosis"],
                    "remote_observation": None,  # Phase 1: not observed
                    "stages": stage_results,
                }
            )

        return {
            "schema_version": 2,
            "phase": 1,
            "generated_at": now_wall,
            "peer": self._spec.get("peer"),
            "remote": self._spec.get("remote"),
            "local_domain_id": self._spec.get("local_domain_id"),
            "ota_domain_id": self._spec.get("ota_domain_id"),
            "uses_domain_bridge": self._spec.get("uses_domain_bridge"),
            "summary": counts,
            "link": compute_link_overview(topics_out, link_sample),
            "clock_sync": self._clock_snapshot(now_mono),
            "startup_check": self._startup_check,
            "topics": topics_out,
        }

    def _clock_snapshot(self, now_mono: float) -> Optional[Dict[str, Any]]:
        if self._clock_estimator is None:
            return None
        estimate = self._clock_estimator.estimate(now_mono)
        if estimate is None:
            return None
        return {
            "method": "echo_min_rtt",
            "peer_offset_ms": round(estimate["offset_s"] * 1000.0, 3),
            "rtt_ms": round(estimate["rtt_s"] * 1000.0, 3),
            "sample_age_s": round(estimate["age_s"], 3),
            "samples": int(estimate["samples"]),
            "assumption": "symmetric_path",
        }

    def detect_transitions(self, snapshot: Dict[str, Any]) -> List[Dict[str, Any]]:
        events: List[Dict[str, Any]] = []
        for t in snapshot["topics"]:
            key = f"{t['direction']}:{t['base']}"
            cur = {
                "overall": t["overall"],
                "reached_stage": t["reached_stage"],
                "blocked_at": t["blocked_at"],
                "stages": {s["stage"]: s["state"] for s in t["stages"]},
            }
            prev = self._prev_states.get(key)
            if prev != cur:
                if prev is not None:
                    events.append(
                        {
                            "kind": "state_transition",
                            "at": snapshot["generated_at"],
                            "peer": snapshot["peer"],
                            "topic": t["base"],
                            "direction": t["direction"],
                            "from": {
                                "overall": prev.get("overall"),
                                "reached_stage": prev.get("reached_stage"),
                                "blocked_at": prev.get("blocked_at"),
                            },
                            "to": {
                                "overall": cur["overall"],
                                "reached_stage": cur["reached_stage"],
                                "blocked_at": cur["blocked_at"],
                            },
                            "diagnosis": t["diagnosis"],
                        }
                    )
                self._prev_states[key] = cur
        return events

    def collect_transit_records(self) -> List[Dict[str, Any]]:
        records: List[Dict[str, Any]] = []
        seen: set[int] = set()
        for observer in self._observers.values():
            for obs in observer.observations.values():
                identity = id(obs)
                if identity in seen:
                    continue
                seen.add(identity)
                records.extend(obs.drain_transit_records())
        return records

    def render_text(self, snapshot: Dict[str, Any]) -> str:
        lines: List[str] = []
        s = snapshot["summary"]
        lines.append(
            f"rosotacom status  peer={snapshot['peer']} remote={snapshot['remote']}  "
            f"{snapshot['generated_at']}"
        )
        lines.append(
            f"summary: OK={s.get('OK', 0)} PARTIAL={s.get('PARTIAL', 0)} "
            f"STALLED={s.get('STALLED', 0)} ABSENT={s.get('ABSENT', 0)}   (Phase 1: local observation)"
        )
        # The STAT watcher pane shows this file, so a failed startup check has to
        # be readable here or it is not surfaced where anybody looks.
        check = snapshot.get("startup_check")
        if check and check.get("verdict") != "ok":
            gaps = check.get("missing_outbound_stages") or []
            panes = check.get("pane_failures") or []
            lines.append(
                f"STARTUP CHECK FAILED after {check.get('checked_after_s')}s: "
                f"{len(gaps)} outbound stage(s) without a publisher, {len(panes)} pane(s) exited"
            )
            for gap in gaps:
                lines.append(f"    no publisher: {gap.get('topic')} ({gap.get('produced_by')})")
            for line in panes:
                lines.append(f"    pane exited: {line}")
        lines.append("")
        for t in snapshot["topics"]:
            arrow = "->" if t["direction"] == "outbound" else "<-"
            lines.append(
                f"[{t['overall']:<7}] {arrow} {t['base']}  "
                f"(reached: {t['reached_stage']}, blocked: {t['blocked_at']})"
            )
            for st in t["stages"]:
                state = st["state"]
                if st.get("inferred_from"):
                    state = f"{state}/INFERRED"
                elif st.get("observation") == "graph":
                    state = f"{state}/GRAPH"
                elif state == FLOWING and st.get("quality") and st["quality"] != GOOD:
                    state = f"{state}/{st['quality']}"
                metric = ""
                if st["state"] in (FLOWING, STALE):
                    parts = [f"{st['hz']:.1f}Hz"]
                    if st["latency_ms"] is not None:
                        # A paced stage reads as one big number unless the split
                        # is on the same line: what the link cost, and what this
                        # session chose to add on top of it.
                        if st.get("pacing_hold_ms") is not None:
                            parts.append(
                                f"{st['latency_ms']:.0f}ms"
                                f"(link {st['link_latency_ms']:.0f}+paced {st['pacing_hold_ms']:.0f})"
                            )
                        else:
                            parts.append(f"{st['latency_ms']:.0f}ms")
                    elif st["latency_uncorrected_ms"] is not None:
                        parts.append(f"{st['latency_uncorrected_ms']:.0f}ms(raw)")
                    if st["loss_pct"] is not None:
                        parts.append(f"loss {st['loss_pct']:.1f}%")
                    if st["mean_size_bytes"]:
                        parts.append(f"{st['mean_size_bytes']:.0f}B")
                    if st["age_s"] is not None:
                        parts.append(f"age {st['age_s']:.1f}s")
                    if st.get("inferred_from"):
                        parts.append(f"from {st['inferred_from']}")
                    metric = "  " + " ".join(parts)
                lines.append(
                    f"    {st['stage']:<10} {state:<14} "
                    f"pub={st['publishers']} sub={st['subscribers']}  {st['topic']}{metric}"
                )
            if t["diagnosis"]:
                lines.append(f"    -> {t['diagnosis']}")
            lines.append("")
        return "\n".join(lines) + "\n"

    @staticmethod
    def _atomic_write(path: str, text: str) -> None:
        tmp = f"{path}.tmp"
        with open(tmp, "w", encoding="utf-8") as fp:
            fp.write(text)
        os.replace(tmp, path)

    def _read_pane_failures(self) -> List[str]:
        try:
            with open(self._pane_failure_path, "r", encoding="utf-8", errors="replace") as fp:
                return [line.rstrip("\n") for line in fp if line.strip()]
        except OSError:
            return []

    def _maybe_run_startup_check(
        self, snapshot: Dict[str, Any], now_mono: Optional[float]
    ) -> None:
        """Once, after the grace period: did this peer bring up what it promised?

        Written as a verdict at a defined moment rather than left implicit in a
        table that is refreshed every two seconds. `status.json` already carried
        the evidence on 2026-08-13 -- 13 stages STALLED, `clock_sync: null` --
        and nobody read it, because nothing said "this is wrong now".
        """
        if self._startup_check is not None or self._startup_grace_s <= 0.0:
            return
        now = time.monotonic() if now_mono is None else now_mono
        if now - self._started_mono < self._startup_grace_s:
            return

        gaps = outbound_startup_gaps(snapshot)
        mine = [gap for gap in gaps if gap["owner"] == "rosotacom"]
        theirs = [gap for gap in gaps if gap["owner"] != "rosotacom"]
        pane_failures = self._read_pane_failures()

        # A stage can be late rather than dead. `relay_out` and the wrapper
        # create their publisher on *discovering* their input, on a refresh
        # interval, so an input that started publishing shortly before this
        # moment leaves a gap that is about to close on its own. Waiting a
        # second grace period and reporting only what is still missing
        # separates the two without weakening either: a node that exited never
        # recovers. CI found this the way it should be found -- the smoke that
        # starts its payload publisher last failed on a topic whose relay was
        # working seconds later.
        if mine and self._startup_pending is None:
            self._startup_pending = (now, {gap["topic"] for gap in mine})
            return
        if mine and self._startup_pending is not None:
            first_seen_at, first_seen = self._startup_pending
            if now - first_seen_at < self._startup_grace_s:
                return
            mine = [gap for gap in mine if gap["topic"] in first_seen]
        verdict = "ok" if not mine and not pane_failures else "incomplete"
        self._startup_check = {
            "verdict": verdict,
            "checked_after_s": round(now - self._started_mono, 1),
            "generated_at": snapshot.get("generated_at"),
            "peer": snapshot.get("peer"),
            "missing_outbound_stages": mine,
            "waiting_for_input": theirs,
            "pane_failures": pane_failures,
        }
        snapshot["startup_check"] = self._startup_check
        try:
            self._atomic_write(
                self._startup_path, json.dumps(self._startup_check, indent=2) + "\n"
            )
        except Exception as exc:  # pragma: no cover - defensive
            if self._log is not None:
                self._log.warning(f"status_overview: failed to write startup check: {exc}")
        if self._log is None:
            return
        if theirs:
            # Not a defect and deliberately not an error: whether the data source
            # is publishing is the application's business, not this peer's, and a
            # stage with no input has nothing to publish.
            self._log.info(
                f"status_overview: startup check -- {len(theirs)} outbound topic(s) are "
                "waiting for input: "
                + ", ".join(gap["topic"] for gap in theirs[:8])
                + (" ..." if len(theirs) > 8 else "")
            )
        if verdict == "ok":
            self._log.info(
                "status_overview: startup check OK -- every stage this peer produces "
                f"for {snapshot.get('peer')} has a publisher"
            )
            return
        self._log.error(
            f"status_overview: STARTUP CHECK FAILED for peer {snapshot.get('peer')} "
            f"after {self._startup_check['checked_after_s']}s -- "
            f"{len(mine)} stage(s) this peer produces have no publisher, "
            f"{len(pane_failures)} pane(s) exited. See {self._startup_path}"
        )
        for gap in mine:
            self._log.error(
                f"status_overview:   no publisher on {gap['topic']} "
                f"(stage {gap['stage']}, produced_by {gap['produced_by']})"
            )
        for line in pane_failures:
            self._log.error(f"status_overview:   pane exited: {line}")

    def write(self, now_mono: Optional[float] = None) -> int:
        """Build a snapshot, write artifacts, and return number of transition events."""
        link_sample = None
        if self._link_sampler is not None:
            try:
                link_sample = self._link_sampler.sample()
            except Exception as exc:  # pragma: no cover - defensive
                if self._log is not None:
                    self._log.warning(f"status_overview: link sampling failed: {exc}")
        snapshot = self.build_snapshot(now_mono, link_sample=link_sample)
        self._maybe_run_startup_check(snapshot, now_mono)
        if self._link_trace_recorder is not None:
            try:
                self._link_trace_recorder.maybe_write(snapshot, link_sample)
            except Exception as exc:  # pragma: no cover - defensive
                if self._log is not None:
                    self._log.warning(f"status_overview: failed to write link trace: {exc}")
        events = self.detect_transitions(snapshot)
        transit_records = self.collect_transit_records()
        try:
            self._atomic_write(self._json_path, json.dumps(snapshot, indent=2) + "\n")
            self._atomic_write(self._txt_path, self.render_text(snapshot))
            if events or transit_records:
                with open(self._events_path, "a", encoding="utf-8") as fp:
                    for ev in [*events, *transit_records]:
                        fp.write(json.dumps(ev) + "\n")
        except Exception as exc:  # pragma: no cover - defensive
            if self._log is not None:
                self._log.warning(f"status_overview: failed to write artifacts: {exc}")
        return len(events)


def collect_stage_topics(spec: Dict[str, Any]) -> Dict[str, Dict[str, Optional[str]]]:
    """Return {domain: {topic: type_hint}} from the pipeline spec.

    Includes the pacing-hold topics a stage names: they are not stages of the
    link -- nothing is delivered on them -- but the aggregator has to read them
    to tell this session's deliberate delay from the network's.
    """
    by_domain: Dict[str, Dict[str, Optional[str]]] = {"local": {}, "ota": {}}
    for topic_spec in spec.get("topics", []):
        type_hint = topic_spec.get("type")
        for stage in topic_spec.get("stages", []):
            domain = stage.get("domain", "local")
            by_domain.setdefault(domain, {})
            existing = by_domain[domain].get(stage["topic"])
            by_domain[domain][stage["topic"]] = existing or stage.get("type") or type_hint
            hold_topic = stage.get(PACED_HOLD_KEY)
            if hold_topic:
                by_domain["local"].setdefault(str(hold_topic), SCALAR_MSG_TYPE)
    return by_domain


def collect_stage_metadata(spec: Dict[str, Any]) -> Dict[str, Dict[str, List[Dict[str, Any]]]]:
    """Return observation context for each concrete stage topic."""
    by_domain: Dict[str, Dict[str, List[Dict[str, Any]]]] = {"local": {}, "ota": {}}
    for topic_spec in spec.get("topics", []):
        for stage in topic_spec.get("stages", []):
            domain = stage.get("domain", "local")
            by_domain.setdefault(domain, {})
            by_domain[domain].setdefault(stage["topic"], []).append(
                {
                    "base": topic_spec.get("base"),
                    "peer": spec.get("peer"),
                    "direction": topic_spec.get("direction"),
                    "source": topic_spec.get("source"),
                    "target": topic_spec.get("target"),
                    "stage": stage.get("stage"),
                    "type": stage.get("type") or topic_spec.get("type"),
                }
            )
    return by_domain
