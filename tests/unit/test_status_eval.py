# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import pytest

from rosotacom.status_eval import (
    evaluate_report,
    evaluate_reports,
    expectations_from_cfg,
    resolve_expect_for_profile,
    topics_requiring_bag,
)


def _stage(stage: str, hz: float, latency_ms: float | None, state: str = "FLOWING", publishers: int = 1) -> dict:
    return {"stage": stage, "hz": hz, "latency_ms": latency_ms, "state": state, "publishers": publishers}


def _topic(base: str, direction: str, overall: str, stages: list[dict]) -> dict:
    return {"base": base, "direction": direction, "overall": overall, "stages": stages}


# Mirrors a settled 1_heartbeat status.json: inbound delivered through app_in,
# outbound observed locally at the native publish stage.
OK_REPORT = {
    "peer": "b",
    "topics": [
        _topic("/heartbeat_a", "inbound", "OK", [_stage("com_in", 10.0, 5.3), _stage("app_in", 10.0, 6.9)]),
        _topic("/heartbeat_b", "outbound", "OK", [_stage("native", 10.0, None)]),
    ],
}


def test_all_ok_without_expectations_passes() -> None:
    assert evaluate_report(OK_REPORT, {}) == []


def test_non_ok_overall_fails_with_diagnosis() -> None:
    report = {"peer": "a", "topics": [_topic("/x", "inbound", "STALLED", [_stage("com_in", 0.0, None, "STALE")])]}
    report["topics"][0]["diagnosis"] = "stopped 4s ago"
    failures = evaluate_report(report, {})
    assert len(failures) == 1 and "STALLED" in failures[0] and "stopped 4s ago" in failures[0]


def test_smoke_probe_false_skips_contract_only_topic() -> None:
    report = {"peer": "a", "topics": [_topic("/contract_only", "inbound", "STALLED", [])]}

    assert evaluate_report(report, {"/contract_only": {"smoke_probe": False, "hz": {"min": 1}}}) == []


def test_empty_report_fails() -> None:
    assert evaluate_report({"peer": "a", "topics": []}, {})


def test_expect_within_bounds_passes() -> None:
    expect = {"/heartbeat_a": {"hz": {"min": 8, "max": 12}, "latency_ms": {"max": 200}}}
    assert evaluate_report(OK_REPORT, expect) == []


def test_expect_hz_too_low_fails() -> None:
    failures = evaluate_report(OK_REPORT, {"/heartbeat_a": {"hz": {"min": 50}}})
    assert len(failures) == 1 and "hz" in failures[0]


def test_expect_latency_exceeded_fails() -> None:
    failures = evaluate_report(OK_REPORT, {"/heartbeat_a": {"latency_ms": {"max": 1}}})
    assert len(failures) == 1 and "latency" in failures[0]


def test_expect_only_checked_on_inbound_side() -> None:
    # Delivery (and thus the contract) is observed inbound; an outbound topic's
    # expect is not asserted against its local publish stage.
    assert evaluate_report(OK_REPORT, {"/heartbeat_b": {"hz": {"min": 50}}}) == []


def test_bad_outbound_quality_does_not_fail_receiver_side_contract() -> None:
    # 11_trickle is the concrete regression: the sender publishes the base topic
    # at 1 Hz, while the receiver-side trickle output is contracted at 2..8 Hz.
    # The sender report may therefore classify the outbound base topic as BAD,
    # but the contract is asserted on the receiver's inbound final stage.
    bad_outbound_stage = {
        "stage": "ota_sent",
        "hz": 1.0,
        "latency_ms": None,
        "state": "FLOWING",
        "quality": "BAD",
        "quality_reason": "hz",
    }
    report = {"peer": "b", "topics": [_topic("/trickle_demo", "outbound", "OK", [bad_outbound_stage])]}
    assert evaluate_report(report, {"/trickle_demo": {"hz": {"min": 2, "max": 8}}}) == []


def test_expectations_from_cfg_collects_only_expect_blocks() -> None:
    cfg = {"topics": {"b_to_a": [{"topic": "/c", "expect": {"hz": {"min": 1}}}, {"topic": "/d"}, "/e"]}}
    assert expectations_from_cfg(cfg) == {"/c": {"hz": {"min": 1}}}


def test_evaluate_reports_aggregates_across_peers() -> None:
    bad = {"peer": "a", "topics": [_topic("/x", "inbound", "ABSENT", [])]}
    failures = evaluate_reports({"a": bad, "b": OK_REPORT}, {})
    assert len(failures) == 1 and "ABSENT" in failures[0]


def test_bad_quality_on_final_stage_fails_even_when_delivered() -> None:
    # The status overview now classifies stages against `expect`; a delivered
    # (overall OK) topic whose final stage is quality BAD violates its contract.
    bad_stage = {
        "stage": "app_in",
        "hz": 3.0,
        "latency_ms": 5.0,
        "state": "FLOWING",
        "quality": "BAD",
        "quality_reason": "hz",
    }
    report = {"peer": "b", "topics": [_topic("/heartbeat_a", "inbound", "OK", [bad_stage])]}
    failures = evaluate_report(report, {})
    assert len(failures) == 1 and "contract violated" in failures[0] and "hz" in failures[0]


# --- presence: required | optional ------------------------------------------


def _stalled(base: str, *, state: str, publishers: int) -> dict:
    t = _topic(base, "inbound", "STALLED", [_stage("app_in", 0.0, None, state, publishers)])
    t["diagnosis"] = "no publisher" if publishers == 0 else "stopped 9s ago"
    return t


def test_optional_topic_not_delivered_passes() -> None:
    # An optional topic (e.g. an a_to_b command with no source in a one-way
    # replay test) that never delivers is not a failure.
    report = {"peer": "a", "topics": [_stalled("/cmd", state="ABSENT", publishers=0)]}
    assert evaluate_report(report, {"/cmd": {"presence": "optional"}}) == []


def test_required_topic_not_delivered_still_fails() -> None:
    report = {"peer": "a", "topics": [_stalled("/cmd", state="ABSENT", publishers=0)]}
    failures = evaluate_report(report, {"/cmd": {"presence": "required"}})
    assert len(failures) == 1 and "STALLED" in failures[0]


# --- mode: latched ----------------------------------------------------------


def test_latched_delivered_then_idle_passes() -> None:
    # A latched/static topic delivers its value once and then stops ticking; the
    # final stage is STALE, overall is STALLED, but the held value arrived.
    report = {"peer": "a", "topics": [_stalled("/site", state="STALE", publishers=1)]}
    assert evaluate_report(report, {"/site": {"mode": "latched"}}) == []


def test_latched_never_delivered_fails() -> None:
    # Publisher present but no message ever observed -> the latch never arrived.
    report = {"peer": "a", "topics": [_stalled("/site", state="IDLE", publishers=1)]}
    failures = evaluate_report(report, {"/site": {"mode": "latched"}})
    assert len(failures) == 1 and "STALLED" in failures[0]


def test_latched_outbound_produced_passes_even_if_send_stage_idle() -> None:
    # On the sender a one-shot held value shows at native/processed but the
    # inferred OTA-send stage may not keep ticking; producing+latching is enough.
    topic = {
        "base": "/site",
        "direction": "outbound",
        "overall": "STALLED",
        "diagnosis": "stopped",
        "stages": [
            _stage("native", 1.0, None, "FLOWING"),
            _stage("processed", 0.0, None, "STALE"),
            _stage("ota_sent", 0.0, None, "IDLE"),
        ],
    }
    assert evaluate_report({"peer": "b", "topics": [topic]}, {"/site": {"mode": "latched"}}) == []


def test_latched_outbound_never_produced_fails() -> None:
    topic = {
        "base": "/site",
        "direction": "outbound",
        "overall": "STALLED",
        "diagnosis": "no publisher",
        "stages": [_stage("native", 0.0, None, "IDLE"), _stage("ota_sent", 0.0, None, "IDLE")],
    }
    assert len(evaluate_report({"peer": "b", "topics": [topic]}, {"/site": {"mode": "latched"}})) == 1


def test_latched_does_not_assert_rate_when_ok() -> None:
    # A latched topic that happens to be FLOWING must not fail on a stream-style
    # hz contract (it has none); only stream mode asserts hz/quality.
    bad_stage = {
        "stage": "app_in",
        "hz": 1.0,
        "latency_ms": 5.0,
        "state": "FLOWING",
        "quality": "BAD",
        "quality_reason": "hz",
        "publishers": 1,
    }
    report = {"peer": "a", "topics": [_topic("/site", "inbound", "OK", [bad_stage])]}
    assert evaluate_report(report, {"/site": {"mode": "latched"}}) == []


# --- mode: existence --------------------------------------------------------


def test_existence_with_publisher_passes() -> None:
    report = {"peer": "a", "topics": [_stalled("/diag", state="IDLE", publishers=1)]}
    assert evaluate_report(report, {"/diag": {"mode": "existence"}}) == []


def test_existence_without_publisher_fails() -> None:
    report = {"peer": "a", "topics": [_stalled("/diag", state="ABSENT", publishers=0)]}
    failures = evaluate_report(report, {"/diag": {"mode": "existence"}})
    assert len(failures) == 1


def test_invalid_mode_raises() -> None:
    report = {"peer": "a", "topics": [_stalled("/x", state="STALE", publishers=1)]}
    with pytest.raises(ValueError):
        evaluate_report(report, {"/x": {"mode": "bogus"}})


# --- completeness: min_count + completeness.min_ratio -----------------------


def _counted_stage(stage: str, count: int, state: str = "FLOWING") -> dict:
    return {"stage": stage, "hz": 10.0, "latency_ms": 5.0, "state": state, "publishers": 1, "messages_total": count}


def _counted_inbound(base: str, counts: list[tuple[str, int]], overall: str = "OK") -> dict:
    stages = [_counted_stage(s, c) for s, c in counts]
    return {"base": base, "direction": "inbound", "overall": overall, "stages": stages}


def test_min_count_met_passes() -> None:
    report = {"peer": "b", "topics": [_counted_inbound("/tf", [("app_in", 100), ("native_in", 95)])]}
    assert evaluate_report(report, {"/tf": {"min_count": 50}}) == []


def test_min_count_short_fails() -> None:
    report = {"peer": "b", "topics": [_counted_inbound("/tf", [("app_in", 3), ("native_in", 3)])]}
    failures = evaluate_report(report, {"/tf": {"min_count": 50}})
    assert len(failures) == 1 and "min_count" in failures[0] and "3 msgs" in failures[0]


def test_completeness_ratio_met_passes() -> None:
    # 95/100 = 0.95 >= 0.9
    report = {"peer": "b", "topics": [_counted_inbound("/tf", [("app_in", 100), ("native_in", 95)])]}
    assert evaluate_report(report, {"/tf": {"completeness": {"min_ratio": 0.9}}}) == []


def test_completeness_ratio_lossy_pipeline_fails() -> None:
    # 40/100 = 0.4 < 0.9 -- a stage that is FLOWING but dropping 60%.
    report = {"peer": "b", "topics": [_counted_inbound("/tf", [("app_in", 100), ("native_in", 40)])]}
    failures = evaluate_report(report, {"/tf": {"completeness": {"min_ratio": 0.9}}})
    assert len(failures) == 1 and "completeness" in failures[0] and "app_in" in failures[0]


def test_completeness_uses_first_flowing_stage_only() -> None:
    # A leading non-delivered stage must not be the ratio denominator.
    stages = [
        {"stage": "ota_recv", "state": "ABSENT", "publishers": 0, "messages_total": 0, "hz": 0.0, "latency_ms": None},
        _counted_stage("com_in", 100),
        _counted_stage("native_in", 96),
    ]
    report = {"peer": "b", "topics": [{"base": "/tf", "direction": "inbound", "overall": "OK", "stages": stages}]}
    assert evaluate_report(report, {"/tf": {"completeness": {"min_ratio": 0.9}}}) == []


def test_completeness_not_asserted_on_outbound() -> None:
    # Like hz/latency, volume/loss is asserted on the receiving (inbound) side.
    stages = [_counted_stage("native", 3), _counted_stage("ota_sent", 3)]
    report = {"peer": "a", "topics": [{"base": "/tf", "direction": "outbound", "overall": "OK", "stages": stages}]}
    assert evaluate_report(report, {"/tf": {"min_count": 50}}) == []


# --- session-level link overhead -------------------------------------------

from rosotacom.status_eval import link_expect_from_cfg  # noqa: E402


def _report_with_link(link: dict | None) -> dict:
    rep = {"peer": "a", "topics": [_topic("/x", "inbound", "OK", [_stage("app_in", 10.0, 5.0)])]}
    rep["link"] = link
    return rep


def test_link_expect_from_cfg_reads_top_level() -> None:
    assert link_expect_from_cfg({"link": {"max_ratio": 4.0}}) == {"max_ratio": 4.0}
    assert link_expect_from_cfg({}) == {}


def test_link_ratio_within_bound_passes() -> None:
    rep = _report_with_link(
        {"overhead_ratio_out": 2.0, "overhead_ratio_in": 1.5, "link_tx_kbps": 100, "ros_payload_out_kbps": 50}
    )
    assert evaluate_report(rep, {}, {"max_ratio": 4.0}) == []


def test_link_ratio_exceeded_fails() -> None:
    rep = _report_with_link(
        {"overhead_ratio_out": 9.0, "overhead_ratio_in": 1.0, "link_tx_kbps": 900, "ros_payload_out_kbps": 100}
    )
    failures = evaluate_report(rep, {}, {"max_ratio": 4.0})
    assert len(failures) == 1 and "link overhead (outbound)" in failures[0] and "9.0" in failures[0]


def test_link_direction_specific_bounds() -> None:
    rep = _report_with_link({"overhead_ratio_out": 3.0, "overhead_ratio_in": 8.0})
    # out under its bound, in over its own tighter bound -> only the inbound fails.
    failures = evaluate_report(rep, {}, {"max_ratio_out": 5.0, "max_ratio_in": 6.0})
    assert len(failures) == 1 and "inbound" in failures[0]


def test_link_skipped_when_no_sample_or_no_expect() -> None:
    assert evaluate_report(_report_with_link(None), {}, {"max_ratio": 1.0}) == []
    assert evaluate_report(_report_with_link({"overhead_ratio_out": 99.0}), {}, {}) == []


def test_link_ratio_none_skipped() -> None:
    # No payload in a direction -> ratio None -> not asserted.
    rep = _report_with_link({"overhead_ratio_out": None, "overhead_ratio_in": None})
    assert evaluate_report(rep, {}, {"max_ratio": 1.0}) == []


# --- replay: completeness vs bag native rate + suggested expect ---------------

from rosotacom.status_eval import suggest_expectations  # noqa: E402


def _counted2(stage: str, hz: float, count: int, state: str = "FLOWING") -> dict:
    return {"stage": stage, "hz": hz, "latency_ms": None, "state": state, "publishers": 1, "messages_total": count}


def test_bag_completeness_below_ratio_fails() -> None:
    # /tf delivered 5 Hz vs bag native 100 Hz = 5% < vs_bag_ratio 0.1.
    topic = {"base": "/tf", "direction": "inbound", "overall": "OK", "stages": [_counted2("app_in", 5.0, 100)]}
    gt = {"/tf": {"native_hz": 100.0}}
    failures = evaluate_report(
        {"peer": "a", "topics": [topic]}, {"/tf": {"completeness": {"vs_bag_ratio": 0.1}}}, None, gt
    )
    assert len(failures) == 1 and "native" in failures[0] and "/tf" in failures[0]


def test_bag_completeness_above_ratio_passes() -> None:
    topic = {"base": "/tf", "direction": "inbound", "overall": "OK", "stages": [_counted2("app_in", 20.0, 100)]}
    gt = {"/tf": {"native_hz": 100.0}}
    assert (
        evaluate_report({"peer": "a", "topics": [topic]}, {"/tf": {"completeness": {"vs_bag_ratio": 0.1}}}, None, gt)
        == []
    )


def test_bag_completeness_skipped_without_ground_truth() -> None:
    # `evaluate_report` itself still skips -- it has no bag to compare against.
    # That silence is exactly why the CLI must refuse the run one level up; see
    # the `topics_requiring_bag` tests below (#214).
    topic = {"base": "/tf", "direction": "inbound", "overall": "OK", "stages": [_counted2("app_in", 1.0, 100)]}
    assert (
        evaluate_report({"peer": "a", "topics": [topic]}, {"/tf": {"completeness": {"vs_bag_ratio": 0.9}}}, None, None)
        == []
    )


def test_topics_requiring_bag_lists_only_replay_only_expectations() -> None:
    expect = {
        "/tf": {"completeness": {"vs_bag_ratio": 0.1}},
        "/costmap": {"completeness": {"vs_bag_ratio": 0.85}},
        "/plain": {"hz": {"min": 5}},
        "/within_peer_only": {"completeness": {"min_ratio": 0.7}},
    }
    assert topics_requiring_bag(expect) == ["/costmap", "/tf"]


def test_topics_requiring_bag_is_empty_without_any_declaration() -> None:
    assert topics_requiring_bag({"/plain": {"hz": {"min": 5}}}) == []
    assert topics_requiring_bag({}) == []


def test_topics_requiring_bag_sees_a_per_profile_declaration() -> None:
    # network profiles reference: the conditional block can come from a profile override, so a
    # profiled run must be inspected through the same resolution the evaluator
    # uses -- otherwise the requirement is invisible under `--profile`.
    expect = {"/tf": {"per_profile": {"lossy": {"completeness": {"vs_bag_ratio": 0.5}}}}}
    assert topics_requiring_bag(expect, profile="lossy") == ["/tf"]


def test_suggest_stream_topic_emits_hz_band() -> None:
    rep = {"peer": "a", "topics": [_topic("/tf", "inbound", "OK", [_stage("app_in", 10.0, 30.0)])]}
    s = suggest_expectations({"a": rep})
    assert "/tf" in s and s["/tf"]["hz"]["min"] == 6.0 and s["/tf"]["hz"]["max"] == 15.0
    assert s["/tf"]["latency_ms"]["max"] == 110  # 30*2+50


def test_suggest_latched_for_delivered_but_idle() -> None:
    rep = {"peer": "a", "topics": [_topic("/site", "inbound", "OK", [_stage("app_in", 0.0, None, "STALE")])]}
    assert suggest_expectations({"a": rep})["/site"] == {"mode": "latched"}


def test_suggest_optional_for_undelivered_and_skips_outbound() -> None:
    rep = {
        "peer": "a",
        "topics": [
            _topic("/cmd", "inbound", "STALLED", [_stage("app_in", 0.0, None, "ABSENT", publishers=0)]),
            _topic("/out", "outbound", "OK", [_stage("native", 10.0, None)]),
        ],
    }
    s = suggest_expectations({"a": rep})
    assert s["/cmd"] == {"presence": "optional"} and "/out" not in s


# --- latency_ms.stage (true OTA latency at the wrapper's send-stamp stage) -----


def _wrapped_topic() -> dict:
    # com_in carries the OtaStamped send-stamp (latency measurable); the final
    # unwrapped app_in is headerless (latency None).
    return {
        "base": "/wrapped",
        "direction": "inbound",
        "overall": "OK",
        "stages": [
            {"stage": "com_in", "state": "FLOWING", "hz": 5.0, "latency_ms": 30.0},
            {"stage": "app_in", "state": "FLOWING", "hz": 5.0, "latency_ms": None},
        ],
    }


def test_latency_stage_asserts_named_stage_passes() -> None:
    rep = {"peer": "a", "topics": [_wrapped_topic()]}
    assert evaluate_report(rep, {"/wrapped": {"latency_ms": {"max": 100, "stage": "com_in"}}}) == []


def test_latency_stage_exceeded_fails_at_named_stage() -> None:
    rep = {"peer": "a", "topics": [_wrapped_topic()]}
    f = evaluate_report(rep, {"/wrapped": {"latency_ms": {"max": 10, "stage": "com_in"}}})
    assert len(f) == 1 and "com_in" in f[0] and "30" in f[0]


def test_latency_default_stage_is_final_and_headerless_fails() -> None:
    # Without a stage, latency is checked on the final (headerless) stage -> None -> fail.
    rep = {"peer": "a", "topics": [_wrapped_topic()]}
    f = evaluate_report(rep, {"/wrapped": {"latency_ms": {"max": 100}}})
    assert len(f) == 1 and "latency None" in f[0]


def test_latency_unknown_stage_errors() -> None:
    rep = {"peer": "a", "topics": [_wrapped_topic()]}
    f = evaluate_report(rep, {"/wrapped": {"latency_ms": {"max": 100, "stage": "nope"}}})
    assert len(f) == 1 and "not in pipeline" in f[0]


# --- exact seq loss ---------------------------------------------------------


def test_loss_pct_uses_first_sequence_aware_stage() -> None:
    topic = _wrapped_topic()
    topic["stages"][0]["loss_pct"] = 2.5
    rep = {"peer": "a", "topics": [topic]}
    assert evaluate_report(rep, {"/wrapped": {"loss_pct": {"max": 3.0}}}) == []


def test_loss_pct_exceeded_fails() -> None:
    topic = _wrapped_topic()
    topic["stages"][0]["loss_pct"] = 7.5
    rep = {"peer": "a", "topics": [topic]}
    failures = evaluate_report(rep, {"/wrapped": {"loss_pct": {"max": 3.0}}})
    assert len(failures) == 1 and "loss 7.5%" in failures[0]


def test_loss_pct_without_sequence_metric_fails_honestly() -> None:
    rep = {"peer": "a", "topics": [_wrapped_topic()]}
    failures = evaluate_report(rep, {"/wrapped": {"loss_pct": {"max": 3.0}}})
    assert len(failures) == 1 and "loss None" in failures[0]


# --- network profiles reference: per-profile conditional overrides --------------------------- #


def test_resolve_keeps_invariant_overrides_conditional_and_strips_per_profile() -> None:
    expect = {
        "presence": "required",
        "mode": "stream",
        "hz": {"min": 9},
        "latency_ms": {"max": 200},
        "per_profile": {"cellular": {"hz": {"min": 6}, "loss_pct": {"max": 5}}},
    }
    default = resolve_expect_for_profile(expect, None)
    assert "per_profile" not in default
    assert default["hz"] == {"min": 9} and default["latency_ms"] == {"max": 200}

    cellular = resolve_expect_for_profile(expect, "cellular")
    assert cellular["presence"] == "required"  # invariant kept verbatim
    assert cellular["hz"] == {"min": 6}  # conditional overridden by the profile
    assert cellular["latency_ms"] == {"max": 200}  # not overridden -> default conditional
    assert cellular["loss_pct"] == {"max": 5}  # added by the profile
    assert "per_profile" not in cellular

    # A profile with no override block falls back entirely to the default conditional.
    assert resolve_expect_for_profile(expect, "unknown")["hz"] == {"min": 9}


def test_resolve_rejects_invariant_and_unknown_overrides() -> None:
    with pytest.raises(ValueError):
        resolve_expect_for_profile({"mode": "stream", "per_profile": {"c": {"mode": "latched"}}}, "c")
    with pytest.raises(ValueError):
        resolve_expect_for_profile({"per_profile": {"c": {"bogus": 1}}}, "c")


def test_per_profile_relaxes_the_conditional_bound_end_to_end() -> None:
    # /heartbeat_a is delivered inbound at ~6.9 ms; a 5 ms LAN bound fails, but the
    # cellular profile's laxer bound passes -- the same report, two verdicts.
    expect = {
        "/heartbeat_a": {
            "presence": "required",
            "latency_ms": {"max": 5},
            "per_profile": {"cellular": {"latency_ms": {"max": 600}}},
        }
    }
    assert evaluate_report(OK_REPORT, expect) != []  # unshaped: default conditional fails
    assert evaluate_report(OK_REPORT, expect, profile="cellular") == []  # cellular: relaxed, passes
    assert evaluate_report(OK_REPORT, expect, profile="other") != []  # no override -> default fails


def test_suggest_profile_band_nests_conditional_keys_under_per_profile() -> None:
    # A reference replay under 'cellular' -> the conditional band for that profile,
    # nested under per_profile so it merges into an authored invariant block.
    from rosotacom.status_eval import suggest_profile_band

    band = suggest_profile_band({"b": OK_REPORT}, "cellular")
    assert "/heartbeat_a" in band
    entry = band["/heartbeat_a"]["per_profile"]["cellular"]
    assert "hz" in entry and "latency_ms" in entry  # conditional keys only
    assert "presence" not in entry and "mode" not in entry  # invariant stays top-level


# A contract about a stage the session delays on purpose (#301).
#
# `status_overview` classifies such a stage on what is left after the pacer's
# own hold is taken back out. This evaluator has to measure the same thing, or a
# session passes `rosotacom test` while its own panel reads BAD -- one contract,
# two answers, which is worse than either answer alone.


def _paced_report(latency_ms: float, link_latency_ms: float | None) -> dict:
    stage = _stage("native_in", 10.0, latency_ms)
    if link_latency_ms is not None:
        stage["link_latency_ms"] = link_latency_ms
        stage["pacing_hold_ms"] = latency_ms - link_latency_ms
    return {"peer": "a", "topics": [_topic("/camera", "inbound", "OK", [stage])]}


def test_a_paced_stage_is_asserted_on_the_link_not_the_buffer() -> None:
    """286 ms on screen, 46 of them the link's: a 200 ms link contract holds."""
    report = _paced_report(latency_ms=286.0, link_latency_ms=46.0)

    assert evaluate_report(report, {"/camera": {"latency_ms": {"max": 200}}}) == []


def test_a_slow_link_under_a_pacer_still_fails() -> None:
    report = _paced_report(latency_ms=480.0, link_latency_ms=480.0)

    failures = evaluate_report(report, {"/camera": {"latency_ms": {"max": 200}}})

    assert len(failures) == 1
    assert "link latency 480.0ms" in failures[0]


def test_an_unpaced_stage_is_asserted_on_what_it_always_was() -> None:
    report = _paced_report(latency_ms=286.0, link_latency_ms=None)

    failures = evaluate_report(report, {"/camera": {"latency_ms": {"max": 200}}})

    assert len(failures) == 1
    assert "latency 286.0ms" in failures[0]
