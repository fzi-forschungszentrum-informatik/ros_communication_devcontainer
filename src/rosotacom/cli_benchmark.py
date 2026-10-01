# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""``rosotacom benchmark`` subcommand: live driver + CLI glue.

This wires :mod:`rosotacom.benchmark`'s pure, host-tested logic to real runs.
The pattern mirrors :func:`rosotacom.cli._start_noninteractive_ota_smoke`: a
probe starts a session under a profile, waits, collects ``events.jsonl``, and
feeds the transit-record pipeline. What lives here is the *benchmark-specific*
orchestration; the underlying arming, collection, and session lifecycle are all
reused from the existing OTA smoke path.

Real-link runs are *non-deterministic* (monitor-only, never gated); the
deterministic slice (emulated profiles, replay, loopback) gates against the
committed two-sided bands via ``benchmark compare`` / ``benchmark ratchet``
(performance bands reference). The glue below is host-tested via a **stubbed** ``run_point`` in
``tests/unit/test_cli_benchmark.py``.
"""

from __future__ import annotations

import argparse
import contextlib
import copy
import json
import math
import os
import re
import shlex
import shutil
import statistics
import subprocess
import sys
import time
from collections.abc import Callable, Iterator, Sequence
from dataclasses import asdict, dataclass, is_dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Any, cast

import yaml

DEFAULT_BENCHMARK_RMW = "cyclone"
DEFAULT_BENCHMARK_DRAIN_S = 2.0
DEFAULT_CYCLONE_SPDP_INTERVAL = "30s"
SPDP_EFFECT_DURATION_FRACTION = 2.0 / 3.0
SPDP_TIGHT_LINK_UTILIZATION = 0.70
BENCHMARK_RESULT_FILE = "result.json"
BENCHMARK_SESSIONS_BY_GENRE = {
    "probe": "bench_1_1_capacity",
    "capacity": "bench_1_1_capacity",
    "replay": "bench_1_1_capacity",
    "sweep": "bench_1_2_load_sweep",
    "ramp": "bench_1_3_ramp",
    "recovery": "bench_1_4_recovery",
    "sensitivity": "bench_1_1_capacity",
    "matrix": "bench_1_1_capacity",
    "requirements": "bench_1_1_capacity",
    "loss-boundaries": "bench_1_1_capacity",
}

# A/B experiments (#22) install every config under one internal session name and
# switch which one resolves by swapping ``session_configs_dir`` per run, so the
# same synthetic load (the a_to_b sized_publisher stream) drives each config.
AB_SESSION_NAME = "ab_experiment"


class TeeStream:
    def __init__(self, original_stream: Any, file_path: Path):
        self.original_stream: Any = original_stream
        self.file_path: Path = file_path
        self.file: Any = None

    def write(self, data: Any) -> None:
        self.original_stream.write(data)
        self.original_stream.flush()
        if self.file is None:
            try:
                self.file_path.parent.mkdir(parents=True, exist_ok=True)
                self.file = open(self.file_path, "a", encoding="utf-8")
            except Exception:
                pass
        if self.file is not None:
            try:
                self.file.write(data)
                self.file.flush()
            except Exception:
                pass

    def flush(self) -> None:
        self.original_stream.flush()
        if self.file is not None:
            try:
                self.file.flush()
            except Exception:
                pass

    def close(self) -> None:
        if self.file is not None:
            try:
                self.file.close()
            except Exception:
                pass
            self.file = None


@contextlib.contextmanager
def log_stdout_stderr_to_file(file_path: Path) -> Iterator[None]:
    import subprocess

    tee_stdout = TeeStream(sys.stdout, file_path)
    tee_stderr = TeeStream(sys.stderr, file_path)
    old_stdout = sys.stdout
    old_stderr = sys.stderr
    sys.stdout = tee_stdout
    sys.stderr = tee_stderr

    original_run = subprocess.run

    def hooked_run(*args, **kwargs):
        capture_stdout = kwargs.get("stdout") is None and not kwargs.get("capture_output")
        capture_stderr = kwargs.get("stderr") is None and not kwargs.get("capture_output")

        if capture_stdout:
            kwargs["stdout"] = subprocess.PIPE
        if capture_stderr:
            kwargs["stderr"] = subprocess.PIPE

        try:
            res = original_run(*args, **kwargs)
            if capture_stdout and res.stdout:
                val = res.stdout
                if isinstance(val, bytes):
                    val = val.decode("utf-8", errors="replace")
                sys.stdout.write(val)
                sys.stdout.flush()
            if capture_stderr and res.stderr:
                val = res.stderr
                if isinstance(val, bytes):
                    val = val.decode("utf-8", errors="replace")
                sys.stderr.write(val)
                sys.stderr.flush()
            return res
        except subprocess.CalledProcessError as exc:
            if capture_stdout and exc.stdout:
                val = exc.stdout
                if isinstance(val, bytes):
                    val = val.decode("utf-8", errors="replace")
                sys.stdout.write(val)
                sys.stdout.flush()
            if capture_stderr and exc.stderr:
                val = exc.stderr
                if isinstance(val, bytes):
                    val = val.decode("utf-8", errors="replace")
                sys.stderr.write(val)
                sys.stderr.flush()
            raise

    subprocess.run = hooked_run

    try:
        yield
    finally:
        subprocess.run = original_run
        sys.stdout = old_stdout
        sys.stderr = old_stderr
        tee_stdout.close()
        tee_stderr.close()


# --------------------------------------------------------------------------- #
# Transit-record collection
# --------------------------------------------------------------------------- #


def _find_events_files(instance_dir: Path) -> list[Path]:
    """Find all ``events.jsonl`` files under an instance's log tree.

    Confirmed path: ``<instance>/logs/<peer>/status/events.jsonl``, written
    next to ``status.json`` by :mod:`rosotacom.status_overview_core`.
    """
    return sorted(instance_dir.rglob("status/events.jsonl"))


def collect_transit_summary(
    instance_dir: Path,
    *,
    publish_window: tuple[float, float] | None = None,
) -> dict[str, Any]:
    """Load and summarize transit records from an instance's artifact tree."""
    from .transit import filter_transit_records_by_publish_window, load_transit_records, summarize_transit_records

    events_files = _find_events_files(instance_dir)
    if not events_files:
        raise FileNotFoundError(
            f"No status/events.jsonl files found under {instance_dir}. "
            "Did the benchmark point run long enough to produce transit records?"
        )
    records = load_transit_records(events_files)
    if publish_window is not None:
        records = filter_transit_records_by_publish_window(
            records,
            start_s=publish_window[0],
            end_s=publish_window[1],
        )
    return summarize_transit_records(records)


def collect_transit_records(instance_dir: Path) -> list[dict[str, Any]]:
    """Load raw joined transit records from an instance's artifact tree."""
    from .transit import join_transit_records, load_transit_records

    events_files = _find_events_files(instance_dir)
    if not events_files:
        raise FileNotFoundError(f"No status/events.jsonl files found under {instance_dir}.")
    return join_transit_records(load_transit_records(events_files))


def _benchmark_delivery_observed(status_paths: Sequence[Path], *, base: str | None) -> bool:
    """True once any peer's ``status.json`` reports a delivered message.

    ``base`` restricts the check to one topic (the single synthetic load topic);
    ``None`` accepts any contract topic — used for replay loads whose topics cross
    both directions, so the warm-up wait ends as soon as the replay is flowing.
    """
    for status_path in status_paths:
        if not status_path.is_file():
            continue
        try:
            status_data = json.loads(status_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        for topic_info in status_data.get("topics", []):
            if base is not None and topic_info.get("base") != base:
                continue
            if any(stage.get("messages_total", 0) > 0 for stage in topic_info.get("stages", [])):
                return True
    return False


# --------------------------------------------------------------------------- #
# Run-point primitive (the live probe, injected for testing)
# --------------------------------------------------------------------------- #

# The default probe callable is replaced in production by the CLI wiring;
# tests inject a stub.  ``RunPointFn`` is the signature.
RunPointFn = Callable[..., dict[str, Any]]


def _jsonable(value: Any) -> Any:
    if is_dataclass(value) and not isinstance(value, type):
        return _jsonable(asdict(value))
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_jsonable(item) for item in value]
    try:
        json.dumps(value)
    except TypeError:
        return str(value)
    return value


def _benchmark_rmw(args: argparse.Namespace) -> str:
    return str(getattr(args, "rmw", None) or DEFAULT_BENCHMARK_RMW)


def _benchmark_cyclone_spdp_interval(args: argparse.Namespace) -> str | None:
    raw = getattr(args, "cyclone_spdp_interval", None)
    if raw is None:
        return None
    value = str(raw).strip()
    if not value:
        return None
    from .network_profiles import parse_seconds

    parse_seconds(value, "--cyclone-spdp-interval")
    return value


def _normalize_benchmark_profile(profile: str | None) -> str | None:
    if profile is None:
        return None
    normalized = profile.strip()
    if not normalized or normalized.lower() in {"none", "unshaped"}:
        return None
    return normalized


def _benchmark_profile_label(profile: str | None) -> str:
    return _normalize_benchmark_profile(profile) or "none"


def _direction_requires_netem_seed(shaping: Any) -> bool:
    return bool(
        shaping is not None and getattr(shaping, "seed", None) is not None and getattr(shaping, "has_netem", False)
    )


def _profile_requires_netem_seed(profile: Any) -> bool:
    if profile is None:
        return False
    if getattr(profile, "is_timeline", False):
        return any(
            _direction_requires_netem_seed(getattr(segment, "uplink", None))
            or _direction_requires_netem_seed(getattr(segment, "downlink", None))
            for segment in getattr(profile, "timeline", ())
        )
    return _direction_requires_netem_seed(getattr(profile, "uplink", None)) or _direction_requires_netem_seed(
        getattr(profile, "downlink", None)
    )


def _profile_rate_limits_bps(profile: str | None, profiles_file: Path | None) -> list[float]:
    if not profile or profiles_file is None:
        return []
    try:
        from .network_profiles import load_profiles_file

        profile_obj = load_profiles_file(profiles_file).get(profile)
    except Exception:
        return []
    if profile_obj is None:
        return []

    def rate_of(shaping: Any) -> float | None:
        value = getattr(shaping, "rate_bps", None) if shaping is not None else None
        return float(value) if value is not None else None

    rates: list[float] = []
    if getattr(profile_obj, "is_timeline", False):
        for segment in getattr(profile_obj, "timeline", ()):
            for shaping in (getattr(segment, "uplink", None), getattr(segment, "downlink", None)):
                rate = rate_of(shaping)
                if rate is not None:
                    rates.append(rate)
    else:
        for shaping in (getattr(profile_obj, "uplink", None), getattr(profile_obj, "downlink", None)):
            rate = rate_of(shaping)
            if rate is not None:
                rates.append(rate)
    return rates


def _probe_spdp_diagnostics(
    *,
    args: argparse.Namespace,
    profile: str | None,
    duration_s: float,
    load_info: dict[str, Any],
) -> dict[str, Any] | None:
    rmw = _benchmark_rmw(args)
    if rmw != "cyclone":
        return None

    from .cli import _load_runtime_config
    from .network_profiles import parse_seconds

    runtime = _load_runtime_config(args)
    profiles_file = _benchmark_profiles_file(profile, runtime.profiles_file)
    configured_interval = _benchmark_cyclone_spdp_interval(args)
    interval_text = configured_interval or DEFAULT_CYCLONE_SPDP_INTERVAL
    interval_s = parse_seconds(interval_text, "CycloneDDS SPDP interval")
    offered_bps = load_info.get("offered_bandwidth_bps")
    rates = _profile_rate_limits_bps(profile, profiles_file)
    min_rate = min(rates) if rates else None
    utilization = (
        float(offered_bps) / float(min_rate)
        if offered_bps is not None and min_rate is not None and min_rate > 0.0
        else None
    )
    duration_ratio = float(duration_s) / interval_s if interval_s > 0.0 else None
    risk = "low"
    warnings: list[str] = []
    if (
        duration_ratio is not None
        and duration_ratio >= SPDP_EFFECT_DURATION_FRACTION
        and utilization is not None
        and utilization >= SPDP_TIGHT_LINK_UTILIZATION
    ):
        risk = "possible"
        warnings.append(
            "CycloneDDS SPDP discovery traffic may affect probe p99/max latency: "
            f"SPDPInterval={interval_text}, duration={duration_s:g}s, "
            f"offered/shaped-rate={utilization:.2f}. Keep this for end-to-end DDS behavior; "
            "raise --cyclone-spdp-interval only for payload-only characterization."
        )
    return {
        "rmw": rmw,
        "interval": interval_text,
        "interval_s": round(interval_s, 6),
        "override": configured_interval is not None,
        "duration_s": float(duration_s),
        "duration_to_interval_ratio": round(duration_ratio, 6) if duration_ratio is not None else None,
        "profile_rate_limits_bps": rates,
        "minimum_profile_rate_bps": min_rate,
        "offered_bandwidth_bps": offered_bps,
        "offered_to_minimum_profile_rate": round(utilization, 6) if utilization is not None else None,
        "risk": risk,
        "warnings": warnings,
    }


def _attach_benchmark_diagnostics(
    context: dict[str, Any],
    *,
    spdp: dict[str, Any] | None = None,
) -> None:
    if spdp is None:
        return
    diagnostics = context.setdefault("diagnostics", {})
    diagnostics["cyclonedds_spdp"] = spdp
    warnings = diagnostics.setdefault("warnings", [])
    warnings.extend(spdp.get("warnings", []))


def _print_benchmark_warnings(context: dict[str, Any]) -> None:
    for warning in context.get("diagnostics", {}).get("warnings", []):
        print(f"WARN: {warning}")


def _benchmark_drain_s(args: argparse.Namespace) -> float:
    drain_s = float(getattr(args, "drain_s", DEFAULT_BENCHMARK_DRAIN_S))
    if drain_s < 0.0:
        raise ValueError("drain-s must be >= 0.")
    return drain_s


def _apply_benchmark_qos_options(
    cfg: dict[str, Any],
    *,
    reliability: str | None = None,
    depth: int | None = None,
) -> dict[str, Any] | None:
    if reliability is None and depth is None:
        return None
    shared = cfg.setdefault("shared", {})
    if not isinstance(shared, dict):
        raise RuntimeError("Benchmark session config 'shared' must be a mapping.")
    qos = shared.setdefault("qos", {})
    if not isinstance(qos, dict):
        raise RuntimeError("Benchmark session config 'shared.qos' must be a mapping.")
    if depth is not None:
        if depth < 1:
            raise ValueError("QoS depth must be >= 1.")
        defaults = qos.setdefault("defaults", {})
        if not isinstance(defaults, dict):
            raise RuntimeError("Benchmark session config 'shared.qos.defaults' must be a mapping.")
        defaults["depth"] = depth
    if reliability is not None:
        if reliability not in {"best_effort", "reliable"}:
            raise ValueError("QoS reliability must be 'best_effort' or 'reliable'.")
        role_defaults = qos.setdefault("for_role", {})
        if not isinstance(role_defaults, dict):
            raise RuntimeError("Benchmark session config 'shared.qos.for_role' must be a mapping.")
        for role in ("ota_sub", "ota_pub"):
            role_cfg = role_defaults.setdefault(role, {})
            if not isinstance(role_cfg, dict):
                raise RuntimeError(f"Benchmark session config QoS role {role!r} must be a mapping.")
            role_cfg["reliability"] = reliability
    return {"reliability": reliability, "depth": depth}


def _rmw_runtime_implementation(rmw: str) -> str:
    from .cli import session_gen

    return str(session_gen.RMW_ALIASES.get(rmw, rmw))


def _prepare_benchmark_session_config(args: argparse.Namespace, session_name: str, run_dir: Path) -> dict[str, Any]:
    """Copy the benchmark session into the run artifacts and pin ``shared.rmw``."""
    from .cli import _load_runtime_config, _resolve_session

    rmw = _benchmark_rmw(args)
    runtime = _load_runtime_config(args)
    source_session = _resolve_session(session_name, runtime)
    dest_root = run_dir / "session-configs"
    dest_session = dest_root / session_name
    shutil.copytree(source_session.host_dir, dest_session)

    config_path = dest_session / "session-definition.yaml"
    if not config_path.is_file():
        config_path = dest_session / "session-parametrization.yaml"
    if not config_path.is_file():
        raise FileNotFoundError(f"Benchmark session {session_name!r} has no session YAML: {source_session.host_dir}")

    cfg = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(cfg, dict):
        raise RuntimeError(f"Benchmark session config must be a mapping: {config_path}")
    shared = cfg.setdefault("shared", {})
    if not isinstance(shared, dict):
        raise RuntimeError(f"Benchmark session config 'shared' must be a mapping: {config_path}")
    cyclone_spdp_interval = _benchmark_cyclone_spdp_interval(args)
    if cyclone_spdp_interval is not None:
        if rmw != "cyclone":
            raise ValueError("--cyclone-spdp-interval can only be used with --rmw cyclone.")
        shared["rmw"] = {"local": "cyclone", "ota": {"cyclone": {"spdp_interval": cyclone_spdp_interval}}}
    else:
        shared["rmw"] = rmw
    qos_options = _apply_benchmark_qos_options(
        cfg,
        reliability=getattr(args, "qos_reliability", None),
        depth=getattr(args, "qos_depth", None),
    )
    stream_options = None
    if session_name == "bench_1_1_capacity":
        stream_options = _expand_benchmark_capacity_stream_topics(
            cfg,
            int(getattr(args, "streams", 1) or 1),
        )
    config_path.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")

    existing_raw = getattr(args, "session_configs_dir", None)
    if existing_raw is None:
        existing: list[Any] = []
    elif isinstance(existing_raw, list | tuple):
        existing = list(existing_raw)
    else:
        existing = [existing_raw]
    args.session_configs_dir = [str(dest_root), *[str(path) for path in existing]]
    args._benchmark_session_dir = str(dest_session)
    args._benchmark_session_source = str(source_session.host_dir)
    args._benchmark_session_config = str(config_path)

    return {
        "name": session_name,
        "source_dir": source_session.host_dir,
        "artifact_dir": dest_session,
        "config_path": config_path,
        "rmw": rmw,
        "runtime_implementation": _rmw_runtime_implementation(rmw),
        "cyclone_spdp_interval": cyclone_spdp_interval,
        "qos": qos_options,
        "streams": stream_options,
    }


def _activate_benchmark_session_options(
    args: argparse.Namespace,
    session_name: str,
    out_dir: Path,
    options: dict[str, Any] | None,
) -> Any:
    if not options:
        return None

    source_session = Path(getattr(args, "_benchmark_session_dir", ""))
    if not source_session.is_dir():
        raise FileNotFoundError("Row-specific benchmark session options require a prepared session copy.")

    case_token = _safe_case_token(str(options.get("case_token") or "case"))
    dest_root = out_dir / "case-session-configs" / case_token
    dest_session = dest_root / session_name
    if dest_session.exists():
        shutil.rmtree(dest_session)
    shutil.copytree(source_session, dest_session)

    config_path = dest_session / "session-definition.yaml"
    if not config_path.is_file():
        config_path = dest_session / "session-parametrization.yaml"
    if not config_path.is_file():
        raise FileNotFoundError(f"Benchmark session {session_name!r} has no session YAML: {dest_session}")

    cfg = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(cfg, dict):
        raise RuntimeError(f"Benchmark session config must be a mapping: {config_path}")
    _apply_benchmark_qos_options(
        cfg,
        reliability=options.get("qos_reliability"),
        depth=options.get("qos_depth"),
    )
    config_path.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")

    previous = getattr(args, "session_configs_dir", None)
    if previous is None:
        existing: list[Any] = []
    elif isinstance(previous, list | tuple):
        existing = list(previous)
    else:
        existing = [previous]
    args.session_configs_dir = [str(dest_root), *[str(path) for path in existing]]
    return previous


def _restore_benchmark_session_options(args: argparse.Namespace, previous: Any) -> None:
    if previous is None:
        if hasattr(args, "session_configs_dir"):
            args.session_configs_dir = None
    else:
        args.session_configs_dir = previous


def _profile_context(profile: str | None, profiles_file: Path | None) -> dict[str, Any]:
    if not profile:
        return {"name": None, "kind": "none", "configured": None}
    context: dict[str, Any] = {"name": profile, "profiles_file": profiles_file}
    if profiles_file is None:
        context["configured"] = None
        return cast(dict[str, Any], _jsonable(context))
    try:
        from .network_profiles import load_profiles_file

        configured = load_profiles_file(profiles_file).get(profile)
    except Exception as exc:
        context["configured"] = None
        context["load_error"] = str(exc)
    else:
        context["configured"] = configured
    return cast(dict[str, Any], _jsonable(context))


def _benchmark_probe_load(args: argparse.Namespace) -> dict[str, Any]:
    """The load a probe drives, whether it is synthetic or a replay target."""
    if _uses_explicit_ota_target(args):
        return {
            "driven_by": {
                "target": str(args.target),
                "target_type": str(getattr(args, "target_type", None) or "auto"),
            }
        }
    return _probe_load(
        size=getattr(args, "size", 18_000),
        size_pattern=getattr(args, "size_pattern", None),
        rate_hz=getattr(args, "rate_hz", 20.0),
        streams=getattr(args, "streams", 1),
        interval_jitter_ms=getattr(args, "interval_jitter_ms", 0.0),
        interval_jitter_seed=getattr(args, "interval_jitter_seed", 42),
    )


def _benchmark_session_context(args: argparse.Namespace, session_name: str, run_dir: Path) -> dict[str, Any]:
    """The session a genre drives, and whether this run had to prepare it.

    A genre normally drives its own packaged session and pins `shared.rmw` into
    a copy of it. With an explicit OTA `--target` the load is somebody else's
    session, running from each peer's own project — nothing is copied and
    nothing is rewritten, so recording a prepared genre session here would
    describe a file the run never used.
    """
    if _uses_explicit_ota_target(args):
        return {
            "name": session_name,
            "prepared": False,
            "target_override": {
                "target": str(args.target),
                "target_type": str(getattr(args, "target_type", None) or "auto"),
            },
        }
    return _prepare_benchmark_session_config(args, session_name, run_dir)


def _benchmark_result_context(
    args: argparse.Namespace,
    *,
    genre: str,
    profile: str | None,
    run_dir: Path,
    session: dict[str, Any] | None,
) -> dict[str, Any]:
    from .cli import _load_runtime_config

    runtime = _load_runtime_config(args)
    rmw = _benchmark_rmw(args)
    profile = _normalize_benchmark_profile(profile)
    profiles_file = _benchmark_profiles_file(profile, runtime.profiles_file)
    return cast(
        dict[str, Any],
        _jsonable(
            {
                "command": " ".join(shlex.quote(arg) for arg in sys.argv),
                "genre": genre,
                "execution": {
                    "mode": "ota" if _is_ota_benchmark(args) else "local",
                    "dry_run": bool(getattr(args, "dry_run", False)),
                },
                "rmw": {
                    "requested": rmw,
                    "runtime_implementation": _rmw_runtime_implementation(rmw),
                },
                "profile": _profile_context(profile, profiles_file),
                "session": session,
                "paths": {
                    "run_dir": run_dir,
                    "rosotacom_config": runtime.rosotacom_config,
                    "ros2docker_config": runtime.ros2docker_config,
                    "profiles_file": profiles_file,
                    "benchmarks_dir": runtime.benchmarks_dir,
                },
            }
        ),
    )


def _mean_payload_bytes(load: dict[str, Any]) -> float | None:
    if "sizes" in load:
        sizes = load["sizes"]
        if not sizes:
            return 0.0
        return float(sum(sizes) / len(sizes))
    size_a_raw = load.get("size_a", load.get("size"))
    if size_a_raw is None:
        return None
    try:
        size_a = int(size_a_raw)
        size_b = int(load["size_b"]) if load.get("size_b") is not None else None
        pattern = str(load.get("pattern") or load.get("size_pattern") or "")
        if pattern:
            from .benchmark import pattern_mean_bytes

            return float(pattern_mean_bytes(pattern, size_a, size_b))
        return float(size_a)
    except Exception:
        return None


def _load_context(load: dict[str, Any]) -> dict[str, Any]:
    driven_by = load.get("driven_by")
    if driven_by is not None:
        # An explicit OTA `--target` brings its own publishers, so `--size` /
        # `--rate-hz` describe nothing that ran. Recording them anyway is worse
        # than recording nothing: a figure would carry an offered bandwidth the
        # link never saw.
        return cast(
            dict[str, Any],
            _jsonable(
                {
                    "driven_by": driven_by,
                    "parameters": None,
                    "rate_hz": None,
                    "streams": None,
                    "mean_payload_bytes": None,
                    "offered_bandwidth_bps": None,
                }
            ),
        )
    rate_hz = float(load.get("rate", load.get("rate_hz", 0.0)) or 0.0)
    streams = int(load.get("streams", 1) or 1)
    mean_payload_bytes = _mean_payload_bytes(load)
    offered_bandwidth_bps = None
    if mean_payload_bytes is not None:
        offered_bandwidth_bps = mean_payload_bytes * 8.0 * rate_hz * streams
    return cast(
        dict[str, Any],
        _jsonable(
            {
                "parameters": dict(load),
                "rate_hz": rate_hz,
                "streams": streams,
                "mean_payload_bytes": mean_payload_bytes,
                "offered_bandwidth_bps": offered_bandwidth_bps,
            }
        ),
    )


def _probe_load(
    *,
    size: int,
    size_pattern: str | None,
    rate_hz: float,
    streams: int,
    interval_jitter_ms: float = 0.0,
    interval_jitter_seed: int = 42,
) -> dict[str, Any]:
    if size_pattern is not None:
        from .benchmark import parse_size_pattern_load

        load = parse_size_pattern_load(size_pattern)
    else:
        if size < 0:
            raise ValueError("size must be >= 0.")
        load = {"size_a": size}

    load["rate"] = rate_hz
    if streams != 1:
        load["streams"] = streams
    if interval_jitter_ms > 0.0:
        load["interval_jitter_ms"] = interval_jitter_ms
        load["interval_jitter_seed"] = interval_jitter_seed
    return load


def _benchmark_stream_topic_name(topic_name: str, index: int) -> str:
    return f"{topic_name}_{index}"


def _expand_benchmark_capacity_stream_topics(cfg: dict[str, Any], streams: int) -> dict[str, Any] | None:
    if streams < 1:
        raise ValueError("streams must be >= 1.")
    if streams == 1:
        return None

    topics = cfg.setdefault("topics", {})
    if not isinstance(topics, dict):
        raise RuntimeError("Benchmark session config 'topics' must be a mapping.")
    a_to_b = topics.get("a_to_b")
    if not isinstance(a_to_b, list) or len(a_to_b) != 1:
        raise RuntimeError("Multi-stream capacity benchmark sessions must define exactly one a_to_b topic template.")

    template = a_to_b[0]
    if not isinstance(template, dict):
        raise RuntimeError("Benchmark session topic template must be a mapping.")
    topic_name = template.get("topic")
    if not isinstance(topic_name, str) or not topic_name:
        raise RuntimeError("Benchmark session topic template must define a non-empty topic.")

    expanded_topics: list[str] = []
    expanded_specs: list[dict[str, Any]] = []
    for index in range(streams):
        spec = copy.deepcopy(template)
        stream_topic = _benchmark_stream_topic_name(topic_name, index)
        spec["topic"] = stream_topic
        expanded_specs.append(spec)
        expanded_topics.append(stream_topic)
    topics["a_to_b"] = expanded_specs
    return {"count": streams, "base_topic": topic_name, "topics": expanded_topics}


def _publisher_streams_for_topic_specs(topic_specs: Sequence[Any], load: dict[str, Any]) -> int:
    streams = int(load.get("streams", 1) or 1)
    if streams > 1 and len(topic_specs) >= streams:
        return 1
    return streams


def _sized_publisher_param_args(
    topic_name: str,
    load: dict[str, Any],
    *,
    streams: int | None = None,
) -> list[str]:
    rate = load.get("rate", 20.0)
    publisher_streams = int(streams if streams is not None else load.get("streams", 1) or 1)
    params = [
        "-p",
        f"topic:={topic_name}",
        "-p",
        f"rate:={rate}",
        "-p",
        f"streams:={publisher_streams}",
    ]

    if "interval_jitter_ms" in load:
        params.extend(["-p", f"interval_jitter_ms:={load['interval_jitter_ms']}"])
    if "interval_jitter_seed" in load:
        params.extend(["-p", f"interval_jitter_seed:={load['interval_jitter_seed']}"])

    sizes = load.get("sizes")
    if sizes is not None and len(set(sizes)) >= 3:
        sizes_str = "[" + ",".join(str(s) for s in sizes) + "]"
        params.extend(["-p", f"sizes:={sizes_str}"])
        return params

    pattern = load.get("pattern")
    if pattern:
        size_a = load.get("size_a", load.get("size", 66000))
        params.extend(["-p", f"size_a:={size_a}", "-p", f"pattern:={pattern}"])
        if load.get("size_b") is not None:
            params.extend(["-p", f"size_b:={load['size_b']}"])
        return params

    size = load.get("size")
    if size is None:
        size = load.get("size_a")
    if size is None:
        size = 66000
    params.extend(["-p", f"size:={size}"])
    return params


def _probe_point_dirname(instance_id: str, load: dict[str, Any]) -> str:
    """Filesystem- and CI-artifact-safe name for one probe point's copied logs.

    Load values can carry pattern syntax (``a*1,b*1``, bracketed size lists)
    whose characters are hostile to shells and invalid in upload-artifact paths.
    """
    parts = ["probe", instance_id]
    parts += [_safe_case_token(f"{key}_{value}") for key, value in sorted(load.items())]
    return "_".join(parts)


def _benchmark_ota_target(args: argparse.Namespace, session_name: str) -> tuple[str, str]:
    target = getattr(args, "target", None)
    target_type = getattr(args, "target_type", None)
    if target:
        return str(target), str(target_type or "auto")
    if target_type not in (None, "session"):
        raise ValueError("--target-type requires --target unless --target-type=session.")
    return session_name, "session"


def _uses_explicit_ota_target(args: argparse.Namespace) -> bool:
    return _is_ota_benchmark(args) and bool(getattr(args, "target", None))


def _is_ota_benchmark(args: argparse.Namespace) -> bool:
    return bool(getattr(args, "ota_benchmark", False) or getattr(args, "deployment", None))


def _benchmark_profiles_file(profile: str | None, profiles_file: Path | None) -> Path | None:
    profile = _normalize_benchmark_profile(profile)
    if profiles_file is not None:
        return _find_profiles_file(profiles_file)
    try:
        return _find_profiles_file(None)
    except FileNotFoundError:
        if profile:
            raise
        return None


def _benchmark_artifacts_root(args: argparse.Namespace) -> Path:
    from .cli import _load_runtime_config

    runtime = _load_runtime_config(args)
    artifacts_dir = Path(args.artifacts_dir) if getattr(args, "artifacts_dir", None) else runtime.benchmarks_dir
    return artifacts_dir or Path.cwd() / "artifacts" / "benchmarks"


def _benchmark_tmux_session_name(args: argparse.Namespace, genre: str) -> str:
    from .cli import _safe_path_token

    profile = getattr(args, "profile", None) or getattr(args, "profile_grid", None) or "run"
    return _safe_path_token(f"benchmark-{genre}-{profile}")


def _strip_interactive_args(argv: Sequence[str]) -> list[str]:
    return [arg for arg in argv if arg not in {"--interactive", "--no-attach"}]


def _benchmark_child_command() -> list[str]:
    return [sys.executable, "-m", "rosotacom", *_strip_interactive_args(sys.argv[1:])]


def _initialize_interactive_log(full_log: Path, command: Sequence[str]) -> None:
    full_log.parent.mkdir(parents=True, exist_ok=True)
    full_log.write_text(
        "\n".join(
            [
                f"--- benchmark operator log initialized {datetime.now().isoformat(timespec='seconds')} ---",
                "command: " + " ".join(command),
                "",
            ]
        ),
        encoding="utf-8",
    )


def _benchmark_run_script(command: Sequence[str], full_log: Path, session_name: str) -> str:
    pattern = (
        r"probe\(|Probe time bins saved|Benchmark result saved|Capacity result|Capacity:|Ramp curve copied|"
        r"Recovery metrics copied|Sweep frontier copied|ERROR|Error|Warning|benchmark exited"
    )
    script = f"""
import pathlib
import re
import subprocess
import sys

cmd = {json.dumps(list(command))}
log_path = pathlib.Path({str(full_log)!r})
log_path.parent.mkdir(parents=True, exist_ok=True)
selected = re.compile({pattern!r})

print("[INFO] benchmark operator session: {session_name}")
print("[INFO] full orchestrator log:", log_path)
print("[INFO] running:", " ".join(cmd), flush=True)
with log_path.open("a", encoding="utf-8") as log:
    log.write("\\n--- benchmark run started ---\\n")
    log.write("command: " + " ".join(cmd) + "\\n")
    log.flush()
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    assert proc.stdout is not None
    for line in proc.stdout:
        log.write(line)
        log.flush()
        if selected.search(line):
            sys.stdout.write(line)
            sys.stdout.flush()
    rc = proc.wait()
    log.write(f"--- benchmark exited with status {{rc}} ---\\n")
print()
print("[INFO] benchmark exited with status", rc)
sys.exit(rc)
""".strip()
    return shlex.join([sys.executable, "-c", script])


def _grep_log_fragment(log_path: Path, pattern: str, *, lines: int = 40) -> str:
    return (
        f"log={shlex.quote(str(log_path))}; "
        'if [ -f "$log" ]; then '
        f'grep -E {shlex.quote(pattern)} "$log" 2>/dev/null | tail -{lines} || true; '
        "else "
        "echo '[INFO] waiting for orchestrator log:' \"$log\"; "
        "fi"
    )


def _peer_catmux_attach_script(identity: str, container_pattern: str, full_log: Path) -> str:
    # Container names are instance-scoped and allocated by the child benchmark
    # process, so the container is discovered by pattern on every iteration.
    launch_pattern = f"--identity {identity}|identity {identity}|rosotacom session started"
    return (
        "while true; do "
        "clear; date; "
        f"identity={shlex.quote(identity)}; "
        f"container=$(docker ps --filter status=running --format '{{{{.Names}}}}' "
        f"| grep -E {shlex.quote(container_pattern)} | head -n 1); "
        'if [ -z "$container" ]; then '
        f'echo "[INFO] waiting for benchmark peer $identity container matching:" {shlex.quote(container_pattern)}; '
        + _grep_log_fragment(full_log, launch_pattern)
        + "; "
        "sleep 1; continue; "
        "fi; "
        'session=$(docker exec "$container" tmux -L catmux list-sessions -F "#{session_name}" 2>/dev/null '
        "| head -1 || true); "
        'if [ -z "$session" ]; then '
        'echo "[INFO] container is running; waiting for catmux session on tmux socket catmux"; '
        'docker logs --tail 25 "$container" 2>&1 || true; '
        + _grep_log_fragment(full_log, launch_pattern, lines=25)
        + "; "
        "sleep 1; continue; "
        "fi; "
        'echo "[INFO] attaching to benchmark peer $identity catmux session: $session"; '
        'docker exec -it "$container" tmux -L catmux attach-session -t "$session"; '
        "rc=$?; echo; echo '[INFO] catmux attach exited with status' \"$rc\"; "
        "sleep 1; "
        "done"
    )


def _benchmark_peer_windows(args: argparse.Namespace, genre: str, full_log: Path) -> list[tuple[str, str, str]]:
    from .cli import (
        _effective_session_config,
        _load_runtime_config,
        _remote_peer_name,
        _resolve_session,
        _safe_path_token,
        _sanitize_docker_name,
        _workspace_container_prefix,
    )

    if getattr(args, "deployment", None):
        return []
    session_name = BENCHMARK_SESSIONS_BY_GENRE[genre]
    runtime = _load_runtime_config(args)
    session = _resolve_session(session_name, runtime)
    cfg = _effective_session_config(session.host_dir, runtime)
    peers = cfg.get("peers")
    if not isinstance(peers, dict):
        return []
    windows: list[tuple[str, str, str]] = []
    for identity in sorted(str(peer) for peer in peers):
        suffix = _sanitize_docker_name(f"com_to_{_remote_peer_name(cfg, identity)}")
        container_pattern = f"^{re.escape(_workspace_container_prefix(runtime))}[A-Za-z0-9.-]+_{re.escape(suffix)}$"
        window_name = _safe_path_token(f"{identity}_catmux")
        windows.append((window_name, identity, _peer_catmux_attach_script(identity, container_pattern, full_log)))
    return windows


def _result_once_script(full_log: Path) -> str:
    script = f"""
import json
import pathlib
import re
import time

log_path = pathlib.Path({str(full_log)!r})
result_re = re.compile(r"Benchmark result saved to (.+)")
exit_re = re.compile(r"benchmark exited with status (\\d+)")

def hold_open(message):
    print(message, flush=True)
    while True:
        time.sleep(3600)

print("[INFO] waiting for final benchmark result in", log_path, flush=True)
while True:
    if log_path.is_file():
        text = log_path.read_text(encoding="utf-8", errors="replace")
        matches = result_re.findall(text)
        if matches:
            result_path = pathlib.Path(matches[-1].strip())
            print("[INFO] final benchmark result:", result_path)
            if result_path.is_file():
                with result_path.open(encoding="utf-8") as handle:
                    print(json.dumps(json.load(handle), indent=2, sort_keys=True))
            else:
                print("[WARN] result path was reported but is not readable yet:", result_path)
            print()
            hold_open("[INFO] result printed once; pane stays alive for inspection")
        exit_match = exit_re.search(text)
        if exit_match:
            print("[WARN] benchmark exited with status", exit_match.group(1), "before reporting a result file")
            print("[INFO] inspect the run window or full orchestrator log:", log_path)
            hold_open("[INFO] pane stays alive for inspection")
    time.sleep(1)
""".strip()
    return shlex.join([sys.executable, "-c", script])


def _network_command_watch_script(full_log: Path, *, is_ota: bool) -> str:
    pattern = r"tc qdisc|netem|qdisc on|Timeline step|profile safety watchdog|OTA benchmark|tc/netem"
    label = "remote tc/netem commands" if is_ota else "local tc/netem commands"
    return (
        "while true; do "
        "clear; date; "
        f"echo '[INFO] {label} from orchestrator log'; " + _grep_log_fragment(full_log, pattern, lines=100) + "; "
        "sleep 2; "
        "done"
    )


def _network_watch_script(*, is_ota: bool) -> str:
    if is_ota:
        return (
            "while true; do "
            "clear; date; "
            "echo '[INFO] OTA benchmark: network shaping is applied on the remote peers.'; "
            "echo '[INFO] Watch the network:commands pane for tc/netem commands and qdisc diagnostics.'; "
            "sleep 5; "
            "done"
        )
    return (
        "while true; do "
        "clear; date; "
        "echo '[INFO] local benchmark containers'; "
        "docker ps --filter name=rosotacom --format 'table {{.Names}}\\t{{.Status}}' 2>/dev/null || true; "
        "for c in $(docker ps --filter name=rosotacom --format '{{.Names}}' 2>/dev/null); do "
        "echo; echo '[INFO]' \"$c\" 'eth0 qdisc'; "
        'docker exec -u root "$c" tc qdisc show dev eth0 2>/dev/null || true; '
        "done; "
        "sleep 2; "
        "done"
    )


def _start_interactive_benchmark(args: argparse.Namespace, genre: str) -> int:
    from .cli import (
        _attach_tmux_pipe,
        _create_tmux_split_below,
        _host_shell,
        _load_runtime_config,
        _require_tmux,
        _tmux_command,
        _tmux_session_exists,
    )

    runtime = _load_runtime_config(args)
    session_name = _benchmark_tmux_session_name(args, genre)
    command = _benchmark_child_command()
    artifacts_root = _benchmark_artifacts_root(args)
    interactive_logs = artifacts_root / "interactive" / session_name
    full_log = interactive_logs / "orchestrator.full.log"
    is_ota = _is_ota_benchmark(args)

    run_script = _benchmark_run_script(command, full_log, session_name)
    network_script = _network_watch_script(is_ota=is_ota)
    network_command_script = _network_command_watch_script(full_log, is_ota=is_ota)
    result_script = _result_once_script(full_log)
    peer_windows = _benchmark_peer_windows(args, genre, full_log) if not is_ota else []

    if getattr(args, "dry_run", False):
        print(f"Would create benchmark tmux session: {session_name}")
        print(f"Run window: high-level orchestrator (full log {full_log})")
        print("Child command: " + shlex.join(command))
        for window_name, identity, _attach_script in peer_windows:
            print(f"Peer window {identity}: {window_name} fullscreen catmux attach")
        print(
            "Network window: qdisc monitor + tc command log"
            if not is_ota
            else "Network window: OTA shaping monitor + remote tc command log"
        )
        print("Results window: final result printed once")
        return 0

    _require_tmux()
    if _tmux_session_exists(runtime, session_name):
        if not getattr(args, "no_attach", False):
            subprocess.run(_tmux_command(runtime, "attach-session", "-t", session_name), check=True)
        else:
            subcommand = sys.argv[1] if len(sys.argv) > 1 else ("ota-benchmark" if is_ota else "benchmark")
            print(f"Attach with: rosotacom {subcommand} {genre} ... --interactive")
        return 0

    _initialize_interactive_log(full_log, command)
    created = subprocess.run(
        _tmux_command(
            runtime,
            "new-session",
            "-d",
            "-P",
            "-F",
            "#{pane_id}",
            "-s",
            session_name,
            "-n",
            "run",
            _host_shell(run_script),
        ),
        text=True,
        capture_output=True,
        check=True,
    )
    run_pane = created.stdout.strip()
    _attach_tmux_pipe(runtime, run_pane, interactive_logs / "run.log")
    subprocess.run(
        _tmux_command(runtime, "set-window-option", "-g", "-t", session_name, "remain-on-exit", "on"),
        check=True,
    )
    subprocess.run(_tmux_command(runtime, "set-option", "-t", session_name, "prefix", "C-b"), check=True)
    subprocess.run(_tmux_command(runtime, "bind-key", "-T", "prefix", "C-b", "send-prefix"), check=True)
    subprocess.run(
        _tmux_command(runtime, "set-option", "-t", session_name, "status-right", " benchmark | C-b n/p "),
        check=True,
    )

    for window_name, identity, attach_script in peer_windows:
        created_window = subprocess.run(
            _tmux_command(
                runtime,
                "new-window",
                "-d",
                "-P",
                "-F",
                "#{pane_id}",
                "-t",
                session_name,
                "-n",
                window_name,
                _host_shell(attach_script),
            ),
            text=True,
            capture_output=True,
            check=True,
        )
        pane_id = created_window.stdout.strip()
        subprocess.run(_tmux_command(runtime, "select-pane", "-t", pane_id, "-T", f"{identity}:catmux"), check=True)
        _attach_tmux_pipe(runtime, pane_id, interactive_logs / f"{identity}_catmux.log")

    created_network = subprocess.run(
        _tmux_command(
            runtime,
            "new-window",
            "-d",
            "-P",
            "-F",
            "#{pane_id}",
            "-t",
            session_name,
            "-n",
            "network",
            _host_shell(network_script),
        ),
        text=True,
        capture_output=True,
        check=True,
    )
    network_pane = created_network.stdout.strip()
    subprocess.run(_tmux_command(runtime, "select-pane", "-t", network_pane, "-T", "network:status"), check=True)
    _attach_tmux_pipe(runtime, network_pane, interactive_logs / "network.log")
    _create_tmux_split_below(
        runtime,
        network_pane,
        "network:commands",
        _host_shell(network_command_script),
        log_path=interactive_logs / "network_commands.log",
    )
    subprocess.run(
        _tmux_command(runtime, "select-layout", "-t", f"{session_name}:network", "even-vertical"), check=True
    )

    created_results = subprocess.run(
        _tmux_command(
            runtime,
            "new-window",
            "-d",
            "-P",
            "-F",
            "#{pane_id}",
            "-t",
            session_name,
            "-n",
            "results",
            _host_shell(result_script),
        ),
        text=True,
        capture_output=True,
        check=True,
    )
    results_pane = created_results.stdout.strip()
    subprocess.run(_tmux_command(runtime, "select-pane", "-t", results_pane, "-T", "results:final"), check=True)
    _attach_tmux_pipe(runtime, results_pane, interactive_logs / "results.log")

    subprocess.run(_tmux_command(runtime, "select-window", "-t", f"{session_name}:run"), check=True)
    print(f"Benchmark tmux session: {session_name}")
    print("Local control tmux prefix: Ctrl-b.")
    if not getattr(args, "no_attach", False):
        subprocess.run(_tmux_command(runtime, "attach-session", "-t", session_name), check=True)
    else:
        print("Attach with: " + shlex.join(_tmux_command(runtime, "attach-session", "-t", session_name)))
    return 0


def _format_bps(value: float | None) -> str:
    if value is None:
        return "unknown"

    def decimal(scaled: float) -> str:
        return f"{scaled:.3f}".rstrip("0").rstrip(".")

    if value >= 1_000_000:
        return f"{decimal(value / 1_000_000)} Mbit/s"
    if value >= 1_000:
        return f"{decimal(value / 1_000)} kbit/s"
    return f"{value:.0f} bit/s"


def _topic_rows(summary: dict[str, Any], topic: str = "") -> list[dict[str, Any]]:
    topics = summary.get("topics", {})
    if not isinstance(topics, dict):
        return []
    selected = [(topic, topics.get(topic, {}))] if topic else list(topics.items())
    rows: list[dict[str, Any]] = []
    for topic_name, topic_data in selected:
        topic_data = topic_data if isinstance(topic_data, dict) else {}

        def stat_block(name: str, data: dict[str, Any] = topic_data) -> dict[str, Any]:
            value = data.get(name)
            return dict(value) if isinstance(value, dict) else {}

        rows.append(
            {
                "topic": str(topic_name),
                "expected": topic_data.get("expected"),
                "delivered": topic_data.get("delivered"),
                "lost": topic_data.get("lost"),
                "loss_pct": topic_data.get("loss_pct"),
                "reordered": topic_data.get("reordered"),
                "latency_ms": stat_block("ota_hop_ms"),
                "latency_trend_ms": stat_block("ota_hop_trend_ms"),
                "jitter_ms": stat_block("jitter_ms"),
                "inter_arrival_ms": stat_block("inter_arrival_ms"),
            }
        )
    return rows


def _format_topic_rows(rows: Sequence[dict[str, Any]]) -> str:
    if not rows:
        return "no topic metrics"
    parts = []
    for row in rows:
        loss = row.get("loss_pct")
        latency = row.get("latency_ms") or {}
        jitter_p95 = (row.get("jitter_ms") or {}).get("p95")
        parts.append(
            f"{row.get('topic')}: delivered={row.get('delivered')}/{row.get('expected')} "
            f"lost={row.get('lost')} loss={loss}% p50={latency.get('p50')}ms p95={latency.get('p95')}ms "
            f"jitter_p95={jitter_p95}ms"
        )
    return "; ".join(parts)


def _format_stats(stats: dict[str, Any], keys: Sequence[str]) -> str:
    parts = [f"{key}={stats[key]}" for key in keys if stats.get(key) is not None]
    return " ".join(parts) if parts else "no data"


def _format_topic_details(rows: Sequence[dict[str, Any]]) -> str:
    """Multi-line per-topic distribution block printed under the probe summary line."""
    from .transit import BUNCHED_GAP_FRACTION, STALLED_GAP_FRACTION

    lines: list[str] = []
    for row in rows:
        lines.append(f"    {row.get('topic')}:")
        lines.append(
            "      latency_ms: "
            + _format_stats(row.get("latency_ms") or {}, ("min", "p50", "mean", "p95", "p99", "max", "std"))
        )
        trend = row.get("latency_trend_ms") or {}
        if trend.get("delta") is not None:
            lines.append(
                f"      latency_p50_trend_ms: first_third={trend.get('first_third_p50')} "
                f"last_third={trend.get('last_third_p50')} delta={trend.get('delta'):+}"
            )
        lines.append("      jitter_ms: " + _format_stats(row.get("jitter_ms") or {}, ("p50", "mean", "p95", "max")))
        spacing = row.get("inter_arrival_ms") or {}
        lines.append(
            "      inter_arrival_ms: "
            + _format_stats(spacing, ("min", "p05", "p50", "mean", "p95", "p99", "max", "std"))
        )
        nominal = spacing.get("nominal_period_ms")
        if nominal:
            lines.append(
                f"      arrival_spacing vs nominal={nominal}ms: "
                f"bunched(<{round(BUNCHED_GAP_FRACTION * nominal, 3)}ms)={spacing.get('bunched_pct')}% "
                f"stalled(>{round(STALLED_GAP_FRACTION * nominal, 3)}ms)={spacing.get('stalled_pct')}% "
                f"stall_then_bunch={spacing.get('stall_then_bunch_pct')}%"
            )
    return "\n".join(lines)


def _write_benchmark_result(
    out_dir: Path,
    *,
    genre: str,
    configuration: dict[str, Any],
    result: dict[str, Any] | list[dict[str, Any]],
    measurements: dict[str, Any],
    verdict: dict[str, Any],
    context: dict[str, Any] | None = None,
    artifacts: dict[str, Any] | None = None,
) -> Path:
    doc = _jsonable(
        {
            "schema_version": 1,
            "created_at": datetime.now().isoformat(),
            "sha": _current_sha(),
            "runner": _runner_context(),
            "genre": genre,
            "context": context or {},
            "configuration": configuration,
            "measurements": measurements,
            "result": result,
            "verdict": verdict,
            "artifacts": artifacts or {},
        }
    )
    path = out_dir / BENCHMARK_RESULT_FILE
    path.write_text(json.dumps(doc, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Benchmark result saved to {path}")
    return path


def _default_run_point(
    *,
    profile: str | None,
    load: dict[str, Any],
    duration_s: float,
    out_dir: Path,
) -> dict[str, Any]:
    """Placeholder probe — the live implementation is wired by the CLI.

    In production, this is never called directly; the benchmark subcommands
    inject the real probe via ``_make_live_run_point``. For host-deterministic
    tests, a stubbed probe replaces this entirely.
    """
    raise NotImplementedError(
        "run_point requires a live rosotacom session. This should be called through the CLI benchmark subcommands."
    )


# --------------------------------------------------------------------------- #
# Benchmark driver: fixed probe
# --------------------------------------------------------------------------- #


def _run_probe_attempt(
    run_point: RunPointFn,
    *,
    profile: str | None,
    load: dict[str, Any],
    duration_s: float,
    out_dir: Path,
    topic: str = "",
    bin_s: float | None = None,
) -> dict[str, Any]:
    """Run one fixed-load benchmark point and collect the shared per-topic view."""
    summary = run_point(profile=profile, load=load, duration_s=duration_s, out_dir=out_dir)
    rows = _topic_rows(summary, topic)
    load_info = _load_context(load)
    result: dict[str, Any] = {
        "summary": summary,
        "topics": rows,
        "load": load_info,
    }
    if bin_s is not None:
        from .benchmark import characterize_probe_records, exclude_probe_warmup

        records = _load_raw_records_from_out(out_dir)
        rate_hz = float(load.get("rate", load.get("rate_hz", 0.0)) or 0.0)
        nominal_period_s = (1.0 / rate_hz) if rate_hz > 0.0 else None
        result["raw_record_count"] = len(records)

        # Drop the un-impaired warm-up plateau and the partial-impairment
        # transition packet so neither the time bins nor the per-topic p95/loss
        # summary reflect start-up artefacts (benchmark commands reference, "characterise the profile
        # under test, not the setup"). Detection is conservative: without a clear
        # step nothing is dropped and the full-run summary stands.
        settled, onset = exclude_probe_warmup(records, nominal_period_s=nominal_period_s)
        if onset is not None:
            from .transit import summarize_transit_records

            summary = summarize_transit_records(settled)
            rows = _topic_rows(summary, topic)
            result["summary"] = summary
            result["topics"] = rows
            result["warmup_excluded"] = len(records) - len(settled)
            result["settled_onset_s"] = round(onset, 6)

        result["time_bins"] = characterize_probe_records(
            settled,
            bin_s=bin_s,
            nominal_period_s=nominal_period_s,
            topic=topic,
            exclude_warmup=False,
        )
    return result


def drive_probe(
    run_point: RunPointFn,
    *,
    profile: str | None,
    size: int = 18_000,
    size_pattern: str | None = None,
    rate_hz: float = 20.0,
    streams: int = 1,
    topic: str = "",
    repeats: int = 1,
    duration_s: float = 60.0,
    bin_s: float = 1.0,
    render_plot: bool = True,
    out_dir: Path,
    result_context: dict[str, Any] | None = None,
    interval_jitter_ms: float = 0.0,
    interval_jitter_seed: int = 42,
    genre: str = "probe",
    replay: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Run a fixed probe and characterize time-dependent latency/loss behavior."""
    if repeats < 1:
        raise ValueError("repeats must be >= 1.")
    if rate_hz <= 0.0:
        raise ValueError("rate_hz must be > 0.")
    if streams < 1:
        raise ValueError("streams must be >= 1.")

    profile = _normalize_benchmark_profile(profile)
    profile_label = _benchmark_profile_label(profile)
    load = _probe_load(
        size=size,
        size_pattern=size_pattern,
        rate_hz=rate_hz,
        streams=streams,
        interval_jitter_ms=interval_jitter_ms,
        interval_jitter_seed=interval_jitter_seed,
    )
    load_info = _load_context(load)
    out_dir.mkdir(parents=True, exist_ok=True)

    attempts: list[dict[str, Any]] = []
    time_bins: list[dict[str, Any]] = []
    for attempt in range(1, repeats + 1):
        attempt_dir = out_dir / "attempts" / f"attempt_{attempt:02d}"
        attempt_dir.mkdir(parents=True, exist_ok=True)
        attempt_result = _run_probe_attempt(
            run_point,
            profile=profile,
            load=load,
            duration_s=duration_s,
            out_dir=attempt_dir,
            topic=topic,
            bin_s=bin_s,
        )
        attempt_bins = [
            {
                "attempt": attempt,
                "profile": profile_label,
                **bin_row,
            }
            for bin_row in attempt_result.get("time_bins", [])
        ]
        time_bins.extend(attempt_bins)
        warmup_excluded = int(attempt_result.get("warmup_excluded", 0) or 0)
        attempts.append(
            {
                "attempt": attempt,
                "artifact_dir": str(attempt_dir.relative_to(out_dir)),
                "raw_record_count": attempt_result.get("raw_record_count", 0),
                "warmup_excluded": warmup_excluded,
                "settled_onset_s": attempt_result.get("settled_onset_s"),
                "time_bin_count": len(attempt_bins),
                "topics": attempt_result["topics"],
                "summary": attempt_result["summary"],
            }
        )
        warmup_note = f" warmup_excluded={warmup_excluded}" if warmup_excluded else ""
        print(
            f"  probe(attempt={attempt}/{repeats}): "
            f"offered_bw={_format_bps(load_info.get('offered_bandwidth_bps'))} "
            f"bins={len(attempt_bins)}{warmup_note} {_format_topic_rows(attempt_result['topics'])}"
        )
        details = _format_topic_details(attempt_result["topics"])
        if details:
            print(details)

    bins_path = out_dir / "time-bins.jsonl"
    bins_path.write_text(
        "\n".join(json.dumps(row, sort_keys=True) for row in time_bins) + ("\n" if time_bins else ""),
        encoding="utf-8",
    )

    artifacts: dict[str, Any] = {"time_bins": bins_path.name, "stdout": "stdout.txt", "attempts_dir": "attempts"}
    plot_error: str | None = None
    if render_plot and time_bins:
        try:
            from .plots import plot_probe_raw, plot_probe_timeseries

            plot_path = plot_probe_timeseries(time_bins, out=out_dir / "probe-timeseries.png")
            artifacts["plot"] = plot_path.name
            print(f"Probe plot saved to {plot_path}")

            try:
                raw_records = _load_raw_records_from_out(out_dir)
                if raw_records:
                    raw_plot_path = plot_probe_raw(raw_records, out=out_dir / "probe-raw.png")
                    artifacts["plot_raw"] = raw_plot_path.name
                    print(f"Probe raw plot saved to {raw_plot_path}")
            except Exception as raw_exc:
                print(f"Probe raw plot skipped: {raw_exc}", file=sys.stderr)
        except Exception as exc:
            plot_error = str(exc)
            artifacts["plot_error"] = plot_error
            print(f"Probe plot skipped: {plot_error}", file=sys.stderr)

    result = {
        "profile": profile_label,
        "load": load_info,
        "attempts": [
            {
                "attempt": attempt["attempt"],
                "raw_record_count": attempt["raw_record_count"],
                "time_bin_count": attempt["time_bin_count"],
                "topics": attempt["topics"],
            }
            for attempt in attempts
        ],
        "time_bin_count": len(time_bins),
        "plot_error": plot_error,
    }
    configuration: dict[str, Any] = {
        "profile": profile_label,
        "load": load_info,
        "topic": topic or None,
        "repeats": repeats,
        "duration_s": duration_s,
        "bin_s": bin_s,
        "render_plot": render_plot,
    }
    if replay is not None:
        # The provenance travels with the measurement: a result.json that cannot
        # name the bag it reproduces cannot be judged against that bag later.
        configuration["replay"] = replay
        result["replay"] = replay
    result_path = _write_benchmark_result(
        out_dir,
        genre=genre,
        context=result_context,
        configuration=configuration,
        result=result,
        measurements={"attempts": attempts, "time_bins": time_bins},
        verdict={"passed": True, "status": "completed"},
        artifacts=artifacts,
    )
    result["result_file"] = str(result_path)
    print(f"Probe time bins saved to {bins_path}")
    return result


# --------------------------------------------------------------------------- #
# Benchmark driver: replay
# --------------------------------------------------------------------------- #


def drive_replay(
    run_point: RunPointFn,
    *,
    replay: dict[str, Any],
    out_dir: Path,
    **probe_kwargs: Any,
) -> dict[str, Any]:
    """Run a bag-derived load under an emulated profile.

    Mechanically a probe — same load driver, same window, same per-topic
    measurement — with one difference that is the whole point of the genre: the
    load parameters come from a recording, so the run carries that recording's
    provenance and is judged against its whole-bag ``expect`` fragment rather
    than only against a calibrated band (performance bands reference, "realistic loads").

    The public repository ships the bag's *measured shape* (cadence, payload
    distribution, interval jitter), not the recording: anonymized bags stay in
    the operator harness. A row is therefore faithful to the bag exactly as far
    as its committed provenance is, which is what
    ``benched_set._validate_replay_load_matches_bag`` keeps true.
    """
    return drive_probe(run_point, out_dir=out_dir, genre="replay", replay=replay, **probe_kwargs)


# --------------------------------------------------------------------------- #
# Benchmark driver: capacity
# --------------------------------------------------------------------------- #


def drive_capacity(
    run_point: RunPointFn,
    *,
    profile: str | None,
    knob: str,
    low: int,
    high: int,
    max_loss_pct: float,
    max_latency_ms: float,
    rate_hz: float = 20.0,
    topic: str = "",
    repeats: int = 1,
    duration_s: float = 60.0,
    out_dir: Path,
    result_context: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Drive a capacity binary search using real runs.

    Returns the capacity result dict (slice + capacity value + budget metrics).
    """
    from .benchmark import (
        CapacitySlice,
        OracleThresholds,
        find_capacity,
        oracle_passes_topic,
    )

    thresholds = OracleThresholds(max_loss_pct=max_loss_pct, max_latency_ms=max_latency_ms)
    profile = _normalize_benchmark_profile(profile)
    profile_label = _benchmark_profile_label(profile)
    slice_ = CapacitySlice(profile=profile_label, knob=knob, fixed={"rate": rate_hz})
    probe_results: list[dict[str, Any]] = []

    def probe(value: int) -> bool:
        """Run a live point at the given knob value and check the oracle."""
        load: dict[str, Any] = {"rate": rate_hz}
        if knob == "size":
            load["size_a"] = value
        elif knob == "rate":
            load["rate"] = float(value)
        elif knob == "bandwidth":
            load["size_a"] = int(value / (8.0 * rate_hz)) if rate_hz > 0 else value
        else:
            load[knob] = value

        results: list[bool] = []
        for attempt in range(repeats):
            attempt_result = _run_probe_attempt(
                run_point,
                profile=profile,
                load=load,
                duration_s=duration_s,
                out_dir=out_dir,
                topic=topic,
            )
            summary = attempt_result["summary"]
            topics = summary.get("topics", {})
            rows = attempt_result["topics"]
            if topic:
                passes = oracle_passes_topic(topics.get(topic, {}), thresholds)
            else:
                passes = all(oracle_passes_topic(t, thresholds) for t in topics.values()) if topics else False
            results.append(passes)
            load_info = attempt_result["load"]
            probe_results.append(
                {
                    "knob": knob,
                    "value": value,
                    "attempt": attempt + 1,
                    "passed": passes,
                    "load": load_info,
                    "topics": rows,
                    "summary": summary,
                }
            )
            print(
                f"  probe({knob}={value}, attempt={attempt + 1}/{repeats}): "
                f"{'PASS' if passes else 'FAIL'} "
                f"offered_bw={_format_bps(load_info.get('offered_bandwidth_bps'))} "
                f"{_format_topic_rows(rows)}"
            )

        # Median vote: passes if more than half the repeats pass.
        return sum(results) > len(results) // 2

    result = find_capacity(slice_, low, high, probe)
    capacity_load: dict[str, Any] | None = None
    if result.capacity is not None:
        capacity_load_raw: dict[str, Any] = {"rate": rate_hz}
        if knob == "size":
            capacity_load_raw["size_a"] = result.capacity
        elif knob == "rate":
            capacity_load_raw["rate"] = float(result.capacity)
        elif knob == "bandwidth":
            capacity_load_raw["size_a"] = int(result.capacity / (8.0 * rate_hz)) if rate_hz > 0 else result.capacity
        else:
            capacity_load_raw[knob] = result.capacity
        capacity_load = _load_context(capacity_load_raw)
    capacity_result: dict[str, Any] = {
        "slice": {"profile": result.slice.profile, "knob": result.slice.knob, "fixed": result.slice.fixed},
        "capacity": result.capacity,
        "capacity_load": capacity_load,
    }

    result_path = _write_benchmark_result(
        out_dir,
        genre="capacity",
        context=result_context,
        configuration={
            "profile": profile_label,
            "knob": knob,
            "bounds": {"low": low, "high": high},
            "load": _load_context({"rate": rate_hz}),
            "thresholds": {
                "max_loss_pct": max_loss_pct,
                "max_latency_ms": max_latency_ms,
                "latency_quantile": thresholds.latency_quantile,
            },
            "topic": topic or None,
            "repeats": repeats,
            "duration_s": duration_s,
        },
        result=capacity_result,
        measurements={"probes": probe_results},
        verdict={
            "passed": result.capacity is not None,
            "status": "capacity_found" if result.capacity is not None else "no_passing_probe",
        },
        artifacts={"stdout": "stdout.txt", "probes_dir": "probes"},
    )
    capacity_result["result_file"] = str(result_path)
    print(f"Capacity result: {knob}={result.capacity} (result {result_path})")
    return capacity_result


# --------------------------------------------------------------------------- #
# Benchmark driver: ramp
# --------------------------------------------------------------------------- #


def drive_ramp(
    run_point: RunPointFn,
    *,
    profile: str | None,
    values: Sequence[float],
    knob: str = "size",
    rate_hz: float = 20.0,
    topic: str = "",
    duration_s: float = 60.0,
    out_dir: Path,
    result_context: dict[str, Any] | None = None,
) -> list[dict[str, float]]:
    """Drive a linear ramp: measure latency at each load value.

    Returns the response curve as a list of ``{value, metric}`` dicts.
    """
    profile = _normalize_benchmark_profile(profile)
    profile_label = _benchmark_profile_label(profile)
    curve: list[dict[str, float]] = []
    for value in values:
        load: dict[str, Any] = {"rate": rate_hz}
        if knob == "size":
            load["size_a"] = int(value)
        elif knob == "rate":
            load["rate"] = value
        else:
            load[knob] = value

        summary = run_point(profile=profile, load=load, duration_s=duration_s, out_dir=out_dir)
        rows = _topic_rows(summary, topic)
        topics = summary.get("topics", {})
        # Use the first topic or the specified one.
        target_topic = topic if topic else next(iter(topics), "")
        topic_data = topics.get(target_topic, {})
        latency_p95 = (topic_data.get("ota_hop_ms") or {}).get("p95")
        loss_pct = topic_data.get("loss_pct", 100.0)
        load_info = _load_context(load)
        point = {
            "value": float(value),
            "metric": float(latency_p95 or 0.0),
            "loss_pct": float(loss_pct),
            "offered_bw_bps": float(load_info["offered_bandwidth_bps"] or 0.0),
        }
        curve.append(point)
        print(
            f"  ramp({knob}={value}): offered_bw={_format_bps(load_info.get('offered_bandwidth_bps'))} "
            f"{_format_topic_rows(rows)}"
        )

    curve_path = out_dir / "curve.jsonl"
    curve_path.write_text(
        "\n".join(json.dumps(point, sort_keys=True) for point in curve) + "\n",
        encoding="utf-8",
    )
    _write_benchmark_result(
        out_dir,
        genre="ramp",
        context=result_context,
        configuration={
            "profile": profile_label,
            "knob": knob,
            "values": list(values),
            "load": _load_context({"rate": rate_hz}),
            "topic": topic or None,
            "duration_s": duration_s,
        },
        result=curve,
        measurements={"points": curve},
        verdict={"passed": True, "status": "completed"},
        artifacts={"curve": curve_path.name, "stdout": "stdout.txt", "probes_dir": "probes"},
    )
    print(f"Ramp curve saved to {curve_path}")
    return curve


# --------------------------------------------------------------------------- #
# Benchmark driver: recovery
# --------------------------------------------------------------------------- #


def drive_recovery(
    run_point: RunPointFn,
    *,
    profile: str,
    duration_s: float = 90.0,
    nominal_period_s: float = 0.05,
    latched_topics: Sequence[str] = (),
    out_dir: Path,
    profiles_file: Path | None = None,
    result_context: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Drive a recovery test: run under a timeline profile, extract recovery metrics.

    The ``run_point`` for recovery must handle timeline profiles (stepping them
    live). The ``OutageWindow`` is discovered from the timeline.
    """
    from .benchmark import OutageWindow, recovery_metrics
    from .network_profiles import load_profiles_file

    # Load the timeline profile to find the outage window.
    profiles = load_profiles_file(_find_profiles_file(profiles_file))
    profile_obj = profiles.get(profile)
    if profile_obj is None:
        raise ValueError(f"Profile {profile!r} not found in profiles file.")
    if not profile_obj.is_timeline:
        raise ValueError(f"Profile {profile!r} is not a timeline profile; recovery requires a timeline.")

    outage_start: float | None = None
    outage_end: float | None = None
    clock = 0.0
    for segment in profile_obj.timeline:
        if segment.outage is not None:
            outage_start = clock
            outage_end = clock + segment.for_s
        clock += segment.for_s

    if outage_start is None or outage_end is None:
        raise ValueError(f"Profile {profile!r} has no outage segment; recovery needs one.")

    # Run the point (the live probe handles timeline stepping).
    summary = run_point(profile=profile, load={}, duration_s=duration_s, out_dir=out_dir)

    # Collect raw records for recovery analysis.
    records = _load_raw_records_from_out(out_dir)

    metrics = recovery_metrics(
        records,
        OutageWindow(start=outage_start, end=outage_end),
        nominal_period_s=nominal_period_s,
        latched_topics=latched_topics,
    )
    result = {
        "t_recover": metrics.t_recover,
        "t_steady": metrics.t_steady,
        "recovery_burst": metrics.recovery_burst,
        "lost_during_outage": metrics.lost_during_outage,
        "latched_rearrival": metrics.latched_rearrival,
        "outage": {"start": outage_start, "end": outage_end},
    }

    rec_path = out_dir / "recovery.json"
    rec_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _write_benchmark_result(
        out_dir,
        genre="recovery",
        context=result_context,
        configuration={
            "profile": profile,
            "duration_s": duration_s,
            "nominal_period_s": nominal_period_s,
            "latched_topics": list(latched_topics),
            "outage": {"start": outage_start, "end": outage_end},
        },
        result=result,
        measurements={"summary": summary, "raw_record_count": len(records)},
        verdict={"passed": True, "status": "completed"},
        artifacts={"recovery": rec_path.name, "stdout": "stdout.txt", "probes_dir": "probes"},
    )
    print(f"Recovery metrics saved to {rec_path}")
    return result


def _load_raw_records_from_out(out_dir: Path) -> list[dict[str, Any]]:
    """Load raw transit records from the most recent run's events files."""
    from .transit import join_transit_records, load_transit_records

    # Check if there are attempt subdirectories
    attempts_dirs = sorted(out_dir.glob("attempts/attempt_*"))
    if attempts_dirs:
        records: list[dict[str, Any]] = []
        for attempt_dir in attempts_dirs:
            try:
                attempt = int(attempt_dir.name.split("_")[-1])
            except ValueError:
                attempt = 1
            events = sorted(attempt_dir.rglob("status/events.jsonl"))
            if not events:
                events = sorted(attempt_dir.rglob("events.jsonl"))
            if events:
                joined = join_transit_records(load_transit_records(events))
                for r in joined:
                    r["attempt"] = attempt
                records.extend(joined)
        return records

    events = sorted(out_dir.rglob("status/events.jsonl"))
    if not events:
        events = sorted(out_dir.rglob("events.jsonl"))
    if not events:
        return []
    joined = join_transit_records(load_transit_records(events))
    for r in joined:
        if "attempt" not in r:
            r["attempt"] = 1
    return joined


# --------------------------------------------------------------------------- #
# Benchmark driver: sweep
# --------------------------------------------------------------------------- #


def drive_sweep(
    run_point: RunPointFn,
    *,
    profile_grid: Sequence[str],
    max_loss_pct: float = 5.0,
    max_latency_ms: float = 300.0,
    rate_hz: float = 20.0,
    size: int = 60000,
    topic: str = "",
    duration_s: float = 60.0,
    out_dir: Path,
    result_context: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    """Drive a profile-grid sweep (tests 1.1/2.1): run one point per profile,
    report oracle pass/fail for each.

    Returns the frontier as a list of per-profile result dicts.
    """
    from .benchmark import OracleThresholds, oracle_passes_topic

    thresholds = OracleThresholds(max_loss_pct=max_loss_pct, max_latency_ms=max_latency_ms)
    frontier: list[dict[str, Any]] = []
    point_measurements: list[dict[str, Any]] = []

    for raw_profile in profile_grid:
        profile = _normalize_benchmark_profile(raw_profile)
        profile_label = _benchmark_profile_label(profile)
        load = {"size_a": size, "rate": rate_hz}
        summary = run_point(profile=profile, load=load, duration_s=duration_s, out_dir=out_dir)
        rows = _topic_rows(summary, topic)
        topics = summary.get("topics", {})
        target_topic = topic if topic else next(iter(topics), "")
        topic_data = topics.get(target_topic, {})
        passes = oracle_passes_topic(topic_data, thresholds)
        loss_pct = topic_data.get("loss_pct", 100.0)
        latency_p95 = (topic_data.get("ota_hop_ms") or {}).get("p95")
        result = {
            "profile": profile_label,
            "passes": passes,
            "loss_pct": float(loss_pct),
            "latency_p95_ms": float(latency_p95) if latency_p95 is not None else None,
        }
        frontier.append(result)
        load_info = _load_context(load)
        point_measurements.append(
            {
                "profile": profile,
                "profile_label": profile_label,
                "passed": passes,
                "load": load_info,
                "topics": rows,
                "summary": summary,
            }
        )
        print(
            f"  sweep({profile_label}): {'PASS' if passes else 'FAIL'} "
            f"offered_bw={_format_bps(load_info.get('offered_bandwidth_bps'))} {_format_topic_rows(rows)}"
        )

    frontier_path = out_dir / "frontier.jsonl"
    frontier_path.write_text(
        "\n".join(json.dumps(row, sort_keys=True) for row in frontier) + "\n",
        encoding="utf-8",
    )
    _write_benchmark_result(
        out_dir,
        genre="sweep",
        context=result_context,
        configuration={
            "profile_grid": [_benchmark_profile_label(profile) for profile in profile_grid],
            "load": _load_context({"size_a": size, "rate": rate_hz}),
            "thresholds": {
                "max_loss_pct": max_loss_pct,
                "max_latency_ms": max_latency_ms,
                "latency_quantile": thresholds.latency_quantile,
            },
            "topic": topic or None,
            "duration_s": duration_s,
        },
        result=frontier,
        measurements={"points": point_measurements},
        verdict={
            "passed": all(row["passes"] for row in frontier),
            "status": "all_profiles_passed" if all(row["passes"] for row in frontier) else "profile_failures",
        },
        artifacts={"frontier": frontier_path.name, "stdout": "stdout.txt", "probes_dir": "probes"},
    )
    print(f"Sweep frontier saved to {frontier_path}")
    return frontier


# --------------------------------------------------------------------------- #
# Benchmark driver: sensitivity
# --------------------------------------------------------------------------- #


def _safe_case_token(value: str) -> str:
    from .cli import _safe_path_token

    return _safe_path_token(
        value.replace("%", "pct")
        .replace("/", "_")
        .replace(" ", "")
        .replace(".", "p")
        .replace("-", "m")
        .replace("+", "p")
    )


def _format_yaml_ms(value: float | None) -> str | None:
    return None if value is None else f"{value:g}ms"


def _format_yaml_pct(value: float | None) -> str | None:
    return None if value is None else f"{value:g}%"


def _format_yaml_rate(value: float | str | None) -> str | None:
    if value is None:
        return None
    if isinstance(value, str):
        return value

    def decimal(scaled: float) -> str:
        return f"{scaled:.6f}".rstrip("0").rstrip(".")

    if value >= 1_000_000_000:
        return f"{decimal(value / 1_000_000_000)}gbit"
    if value >= 1_000_000:
        return f"{decimal(value / 1_000_000)}mbit"
    if value >= 1_000:
        return f"{decimal(value / 1_000)}kbit"
    return f"{decimal(value)}bit"


def _parse_rate_values(raw: str) -> list[str]:
    return [value.strip() for value in raw.split(",") if value.strip()]


def _parse_sensitivity_axes(raw: str) -> set[str]:
    available = {"loss", "delay", "jitter", "rate", "loss_correlation"}
    aliases = {"loss-correlation": "loss_correlation"}
    axes = {aliases.get(value.strip(), value.strip()) for value in raw.split(",") if value.strip()}
    if not axes or "all" in axes:
        return available
    unknown = axes - available
    if unknown:
        raise ValueError(f"Unknown sensitivity axes: {', '.join(sorted(unknown))}")
    return axes


def _parse_matrix_axes(raw: str) -> set[str]:
    available = {"latency", "hz", "qos"}
    axes = {value.strip().lower() for value in raw.split(",") if value.strip()}
    if not axes or "all" in axes:
        return available
    unknown = axes - available
    if unknown:
        raise ValueError(f"Unknown matrix axes: {', '.join(sorted(unknown))}")
    return axes


def _parse_qos_cases(raw: str) -> list[dict[str, Any]]:
    cases: list[dict[str, Any]] = []
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        if ":" in item:
            reliability, depth = item.split(":", 1)
        else:
            reliability, depth = item, "1"
        reliability = reliability.strip()
        if reliability not in {"best_effort", "reliable"}:
            raise ValueError(f"Unknown QoS reliability in case {item!r}.")
        try:
            depth_int = int(depth.strip())
        except ValueError as exc:
            raise ValueError(f"QoS depth must be an integer in case {item!r}.") from exc
        if depth_int < 1:
            raise ValueError(f"QoS depth must be >= 1 in case {item!r}.")
        cases.append({"reliability": reliability, "depth": depth_int})
    return cases


def _duration_for_min_messages(rate_hz: float, *, min_duration_s: float, min_messages: int) -> float:
    if rate_hz <= 0:
        raise ValueError("rate_hz must be > 0.")
    return max(float(min_duration_s), math.ceil(float(min_messages) / rate_hz))


def _direction_to_yaml(
    *,
    rate: str | None = None,
    delay_ms: float | None = None,
    jitter_ms: float | None = None,
    distribution: str | None = None,
    loss_pct: float | None = None,
    loss_correlation_pct: float | None = None,
    seed: int | None = None,
) -> dict[str, Any]:
    spec: dict[str, Any] = {}
    if rate is not None:
        spec["rate"] = rate
    if delay_ms is not None:
        spec["delay"] = _format_yaml_ms(delay_ms)
    if jitter_ms is not None:
        spec["jitter"] = _format_yaml_ms(jitter_ms)
        if distribution and jitter_ms > 0.0:
            spec["distribution"] = distribution
    if loss_pct is not None:
        spec["loss"] = _format_yaml_pct(loss_pct)
        if loss_correlation_pct is not None and loss_pct > 0.0:
            spec["loss_correlation"] = _format_yaml_pct(loss_correlation_pct)
    if seed is not None:
        spec["seed"] = seed
    return spec


def _profile_to_yaml(profile: Any) -> dict[str, Any]:
    if getattr(profile, "is_timeline", False):
        raise ValueError("sensitivity requires a static base profile, not a timeline profile")

    def direction(direction_obj: Any) -> dict[str, Any]:
        if direction_obj is None:
            return {}
        return _direction_to_yaml(
            rate=_format_yaml_rate(direction_obj.rate_bps),
            delay_ms=direction_obj.delay_ms,
            jitter_ms=direction_obj.jitter_ms,
            distribution=direction_obj.distribution,
            loss_pct=direction_obj.loss_pct,
            loss_correlation_pct=direction_obj.loss_correlation_pct,
            seed=direction_obj.seed,
        )

    return {"uplink": direction(profile.uplink), "downlink": direction(profile.downlink)}


def _first_direction_attr(profile: Any, direction_name: str, attr: str, default: Any = None) -> Any:
    direction = getattr(profile, direction_name, None)
    return getattr(direction, attr, default) if direction is not None else default


def _build_sensitivity_profiles(
    *,
    base_profile: str,
    profiles_file: Path,
    ideal_rate: str,
    loss_values: Sequence[float],
    delay_values: Sequence[float],
    jitter_values: Sequence[float],
    rate_values: Sequence[str],
    correlation_values: Sequence[float],
    axes: Sequence[str] | set[str] | None = None,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    from .network_profiles import load_profiles_file, parse_rate_bps

    profiles = load_profiles_file(profiles_file)
    base_obj = profiles.get(base_profile)
    if base_obj is None:
        raise ValueError(f"Base profile {base_profile!r} not found in {profiles_file}.")
    if getattr(base_obj, "is_timeline", False):
        raise ValueError(f"Base profile {base_profile!r} is a timeline; sensitivity needs a static profile.")

    # Validate rate tokens early; keep the original strings in YAML for readability.
    parse_rate_bps(ideal_rate)
    for rate in rate_values:
        parse_rate_bps(rate)

    base_distribution = (
        _first_direction_attr(base_obj, "uplink", "distribution")
        or _first_direction_attr(base_obj, "downlink", "distribution")
        or "normal"
    )
    base_uplink_delay = float(_first_direction_attr(base_obj, "uplink", "delay_ms", 0.0) or 0.0)
    base_downlink_delay = float(_first_direction_attr(base_obj, "downlink", "delay_ms", base_uplink_delay) or 0.0)
    base_uplink_loss = float(_first_direction_attr(base_obj, "uplink", "loss_pct", 0.0) or 0.0)
    base_downlink_loss = float(_first_direction_attr(base_obj, "downlink", "loss_pct", base_uplink_loss) or 0.0)

    selected_axes = set(axes) if axes is not None else _parse_sensitivity_axes("all")
    generated: dict[str, Any] = {}
    cases: list[dict[str, Any]] = [{"axis": "baseline", "profile": None, "value": "unshaped"}]
    generated[base_profile] = _profile_to_yaml(base_obj)

    def add_case(name: str, axis: str, value: str, profile_spec: dict[str, Any], description: str) -> None:
        generated[name] = profile_spec
        cases.append({"axis": axis, "profile": name, "value": value, "description": description})

    def static_profile(
        *,
        rate: str = ideal_rate,
        delay_uplink_ms: float | None = None,
        delay_downlink_ms: float | None = None,
        jitter_uplink_ms: float | None = None,
        jitter_downlink_ms: float | None = None,
        loss_uplink_pct: float | None = None,
        loss_downlink_pct: float | None = None,
        loss_correlation_pct: float | None = None,
    ) -> dict[str, Any]:
        return {
            "uplink": _direction_to_yaml(
                rate=rate,
                delay_ms=delay_uplink_ms,
                jitter_ms=jitter_uplink_ms,
                distribution=base_distribution,
                loss_pct=loss_uplink_pct,
                loss_correlation_pct=loss_correlation_pct,
            ),
            "downlink": _direction_to_yaml(
                rate=rate,
                delay_ms=delay_downlink_ms,
                jitter_ms=jitter_downlink_ms,
                distribution=base_distribution,
                loss_pct=loss_downlink_pct,
                loss_correlation_pct=loss_correlation_pct,
            ),
        }

    add_case(
        "lab_ideal_rate_only",
        "ideal",
        ideal_rate,
        static_profile(rate=ideal_rate),
        "High-rate tbf only; no netem delay/jitter/loss.",
    )
    add_case(
        f"lab_base_{_safe_case_token(base_profile)}",
        "base",
        base_profile,
        _profile_to_yaml(base_obj),
        "Original base profile copied into the generated lab file.",
    )

    if "loss" in selected_axes:
        for loss in loss_values:
            add_case(
                f"lab_loss_{_safe_case_token(_format_yaml_pct(loss) or str(loss))}",
                "loss",
                _format_yaml_pct(loss) or str(loss),
                static_profile(rate=ideal_rate, loss_uplink_pct=loss, loss_downlink_pct=loss),
                "Only random packet loss is varied; rate is high and delay/jitter are absent.",
            )

    if "delay" in selected_axes:
        for delay in delay_values:
            add_case(
                f"lab_delay_{_safe_case_token(_format_yaml_ms(delay) or str(delay))}",
                "delay",
                _format_yaml_ms(delay) or str(delay),
                static_profile(rate=ideal_rate, delay_uplink_ms=delay, delay_downlink_ms=delay),
                "Only fixed delay is varied; rate is high and loss/jitter are absent.",
            )

    if "jitter" in selected_axes:
        for jitter in jitter_values:
            add_case(
                f"lab_jitter_{_safe_case_token(_format_yaml_ms(jitter) or str(jitter))}",
                "jitter",
                _format_yaml_ms(jitter) or str(jitter),
                static_profile(
                    rate=ideal_rate,
                    delay_uplink_ms=base_uplink_delay,
                    delay_downlink_ms=base_downlink_delay,
                    jitter_uplink_ms=jitter,
                    jitter_downlink_ms=jitter,
                ),
                "Jitter is varied around the base profile's mean delay; rate is high and loss is absent.",
            )

    if "rate" in selected_axes:
        for rate in rate_values:
            add_case(
                f"lab_rate_{_safe_case_token(rate)}",
                "rate",
                rate,
                static_profile(rate=rate),
                "Only rate limiting is varied; delay/jitter/loss are absent.",
            )

    if "loss_correlation" in selected_axes:
        for correlation in correlation_values:
            add_case(
                f"lab_loss_correlation_{_safe_case_token(_format_yaml_pct(correlation) or str(correlation))}",
                "loss_correlation",
                _format_yaml_pct(correlation) or str(correlation),
                static_profile(
                    rate=ideal_rate,
                    loss_uplink_pct=base_uplink_loss,
                    loss_downlink_pct=base_downlink_loss,
                    loss_correlation_pct=correlation,
                ),
                "Loss correlation is varied at the base profile's loss rate; rate is high and delay/jitter are absent.",
            )

    doc = {
        "profiles": generated,
        "metadata": {
            "kind": "benchmark-sensitivity-generated",
            "source_profiles_file": str(profiles_file),
            "base_profile": base_profile,
            "ideal_rate": ideal_rate,
            "axes": sorted(selected_axes),
        },
    }
    return doc, cases


def _build_matrix_profiles(
    *,
    base_profile: str,
    profiles_file: Path,
    ideal_rate: str,
    jitter_ms: float,
    latency_values: Sequence[float],
    rate_hz_values: Sequence[float],
    qos_cases: Sequence[dict[str, Any]],
    axes: Sequence[str] | set[str] | None,
    size: int,
    fixed_rate_hz: float,
    min_duration_s: float,
    min_messages: int,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    from .network_profiles import load_profiles_file, parse_rate_bps

    profiles = load_profiles_file(profiles_file)
    base_obj = profiles.get(base_profile)
    if base_obj is None:
        raise ValueError(f"Base profile {base_profile!r} not found in {profiles_file}.")
    if getattr(base_obj, "is_timeline", False):
        raise ValueError(f"Base profile {base_profile!r} is a timeline; matrix needs a static profile.")

    parse_rate_bps(ideal_rate)
    selected_axes = set(axes) if axes is not None else _parse_matrix_axes("all")
    base_distribution = (
        _first_direction_attr(base_obj, "uplink", "distribution")
        or _first_direction_attr(base_obj, "downlink", "distribution")
        or "normal"
    )
    base_uplink_delay = float(_first_direction_attr(base_obj, "uplink", "delay_ms", 0.0) or 0.0)
    base_downlink_delay = float(_first_direction_attr(base_obj, "downlink", "delay_ms", base_uplink_delay) or 0.0)
    downlink_ratio = base_downlink_delay / base_uplink_delay if base_uplink_delay > 0.0 else 1.0

    generated: dict[str, Any] = {}
    cases: list[dict[str, Any]] = []

    def jitter_profile(name: str, uplink_delay_ms: float) -> str:
        downlink_delay_ms = round(uplink_delay_ms * downlink_ratio, 6)
        generated[name] = {
            "uplink": _direction_to_yaml(
                rate=ideal_rate,
                delay_ms=uplink_delay_ms,
                jitter_ms=jitter_ms,
                distribution=base_distribution,
            ),
            "downlink": _direction_to_yaml(
                rate=ideal_rate,
                delay_ms=downlink_delay_ms,
                jitter_ms=jitter_ms,
                distribution=base_distribution,
            ),
        }
        return name

    fixed_latency_label = _format_yaml_ms(base_uplink_delay) or str(base_uplink_delay)
    fixed_profile = jitter_profile(
        f"matrix_jitter_{_safe_case_token(_format_yaml_ms(jitter_ms) or str(jitter_ms))}"
        f"_latency_{_safe_case_token(fixed_latency_label)}",
        base_uplink_delay,
    )

    def add_case(
        *,
        axis: str,
        value: str,
        profile: str,
        rate_hz: float,
        qos_reliability: str = "best_effort",
        qos_depth: int = 1,
        description: str,
    ) -> None:
        duration_s = _duration_for_min_messages(
            rate_hz,
            min_duration_s=min_duration_s,
            min_messages=min_messages,
        )
        case_token = _safe_case_token(f"{axis}_{value}_{qos_reliability}_{qos_depth}_{rate_hz:g}")
        load = {"size_a": size, "rate": rate_hz}
        cases.append(
            {
                "axis": axis,
                "value": value,
                "profile": profile,
                "description": description,
                "load": load,
                "duration_s": duration_s,
                "qos": {"reliability": qos_reliability, "depth": qos_depth},
                "session_options": {
                    "case_token": case_token,
                    "qos_reliability": qos_reliability,
                    "qos_depth": qos_depth,
                },
            }
        )

    add_case(
        axis="reference",
        value=f"jitter={_format_yaml_ms(jitter_ms)},latency={fixed_latency_label},hz={fixed_rate_hz:g}",
        profile=fixed_profile,
        rate_hz=fixed_rate_hz,
        description="Fixed 30ms-jitter reference using base-profile latency ratio and benchmark default QoS.",
    )

    if "latency" in selected_axes:
        for latency in latency_values:
            latency_label = _format_yaml_ms(latency) or str(latency)
            profile = jitter_profile(
                f"matrix_latency_{_safe_case_token(latency_label)}"
                f"_jitter_{_safe_case_token(_format_yaml_ms(jitter_ms) or str(jitter_ms))}",
                latency,
            )
            add_case(
                axis="latency",
                value=latency_label,
                profile=profile,
                rate_hz=fixed_rate_hz,
                description=(
                    "Vary mean uplink delay while preserving the base profile's uplink/downlink latency ratio; "
                    "jitter is fixed and random loss is absent."
                ),
            )

    if "hz" in selected_axes:
        for rate_hz in rate_hz_values:
            add_case(
                axis="hz",
                value=f"{rate_hz:g}hz",
                profile=fixed_profile,
                rate_hz=rate_hz,
                description=(
                    "Vary publisher frequency under the fixed 30ms-jitter profile; duration is extended to keep "
                    "at least the configured minimum message count."
                ),
            )

    if "qos" in selected_axes:
        for qos in qos_cases:
            reliability = str(qos["reliability"])
            depth = int(qos["depth"])
            add_case(
                axis="qos",
                value=f"{reliability}:depth{depth}",
                profile=fixed_profile,
                rate_hz=fixed_rate_hz,
                qos_reliability=reliability,
                qos_depth=depth,
                description="Vary OTA QoS role defaults under the fixed 30ms-jitter profile.",
            )

    doc = {
        "profiles": generated,
        "metadata": {
            "kind": "benchmark-matrix-generated",
            "source_profiles_file": str(profiles_file),
            "base_profile": base_profile,
            "ideal_rate": ideal_rate,
            "jitter_ms": jitter_ms,
            "fixed_rate_hz": fixed_rate_hz,
            "min_duration_s": min_duration_s,
            "min_messages": min_messages,
            "axes": sorted(selected_axes),
            "latency_model": {
                "mode": "base-ratio",
                "base_uplink_delay_ms": base_uplink_delay,
                "base_downlink_delay_ms": base_downlink_delay,
                "downlink_ratio": downlink_ratio,
            },
        },
    }
    return doc, cases


def _selected_topic(summary: dict[str, Any], topic: str = "") -> tuple[str, dict[str, Any]]:
    topics = summary.get("topics", {})
    if not isinstance(topics, dict) or not topics:
        return "", {}
    if topic:
        return topic, topics.get(topic, {}) if isinstance(topics.get(topic), dict) else {}
    first_topic = next(iter(topics))
    topic_data = topics.get(first_topic)
    return first_topic, topic_data if isinstance(topic_data, dict) else {}


def _probe_sample_verdict(
    summary: dict[str, Any],
    *,
    thresholds: Any,
    topic: str,
    bag_mode: bool,
    ground_truth: dict[str, dict[str, Any]] | None = None,
    min_completeness: float = 0.95,
    required_topics: Sequence[str] | None = None,
) -> dict[str, Any]:
    """One probe run's verdict fields, shared by the requirements/boundary drivers.

    In synthetic-load mode (``bag_mode`` false) this reproduces the single-topic
    oracle the drivers have always used: the selected (or first) topic's loss,
    latency ``p95`` and jitter ``p95``. In bag-as-load mode it aggregates the whole
    replay contract with :func:`rosotacom.benchmark.evaluate_bag_run` — every
    required topic must clear the loss/latency oracle and (when a bag supplies
    ground truth) the completeness floor — and attaches the per-topic verdicts and
    the failing-topic list. Both modes return the same keys, so the search loops
    consume replay verdicts without any change to the boundary/requirements logic.
    """
    from .benchmark import oracle_passes_topic

    if not bag_mode:
        sample_topic, topic_data = _selected_topic(summary, topic)
        loss_pct = float(topic_data.get("loss_pct", 100.0)) if topic_data else 100.0
        lost = topic_data.get("lost") if topic_data else None
        latency_p95 = (topic_data.get("ota_hop_ms") or {}).get("p95") if topic_data else None
        jitter_p95 = (topic_data.get("jitter_ms") or {}).get("p95") if topic_data else None
        loss_free = bool(loss_pct <= 0.0 and (lost is None or int(lost) == 0))
        latency_ok = bool(latency_p95 is not None and float(latency_p95) <= thresholds.max_latency_ms)
        return {
            "topic": sample_topic,
            "expected": topic_data.get("expected") if topic_data else None,
            "delivered": topic_data.get("delivered") if topic_data else None,
            "lost": int(lost) if lost is not None else None,
            "reordered": topic_data.get("reordered") if topic_data else None,
            "loss_pct": loss_pct,
            "latency_p95_ms": float(latency_p95) if latency_p95 is not None else None,
            "jitter_p95_ms": float(jitter_p95) if jitter_p95 is not None else None,
            "loss_free": loss_free,
            "latency_ok": latency_ok,
            "passes": oracle_passes_topic(topic_data, thresholds) if topic_data else False,
            "per_topic": None,
            "failing_topics": None,
        }

    from .benchmark import evaluate_bag_run

    verdict = evaluate_bag_run(
        summary,
        thresholds=thresholds,
        ground_truth=ground_truth,
        min_completeness=min_completeness,
        required_topics=required_topics,
    )
    return {
        "topic": verdict.representative_topic,
        "expected": verdict.expected,
        "delivered": verdict.delivered,
        "lost": verdict.lost,
        "reordered": verdict.reordered,
        "loss_pct": verdict.loss_pct,
        "latency_p95_ms": verdict.latency_p95_ms,
        "jitter_p95_ms": verdict.jitter_p95_ms,
        "loss_free": verdict.loss_free,
        "latency_ok": verdict.latency_ok,
        "passes": verdict.passes,
        "per_topic": [_jsonable(dataclass_verdict) for dataclass_verdict in verdict.topics],
        "failing_topics": list(verdict.failing_topics),
    }


def _sensitivity_analysis(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    baseline = next((row for row in rows if row.get("axis") == "baseline"), None)
    explicit_reference = next((row for row in rows if row.get("axis") == "reference"), None)
    ideal = next((row for row in rows if row.get("axis") == "ideal"), None)
    reference = baseline or explicit_reference or ideal
    reference_loss = float(reference.get("loss_pct") or 0.0) if reference else 0.0
    reference_latency = float(reference.get("latency_p95_ms") or 0.0) if reference else 0.0

    axes: dict[str, Any] = {}
    for axis in sorted({str(row.get("axis")) for row in rows} - {"baseline", "reference"}):
        axis_rows = [row for row in rows if row.get("axis") == axis]
        if not axis_rows:
            continue
        worst_loss = max(axis_rows, key=lambda row: float(row.get("loss_pct") or 0.0))
        worst_latency = max(axis_rows, key=lambda row: float(row.get("latency_p95_ms") or 0.0))
        axes[axis] = {
            "points": len(axis_rows),
            "worst_loss": {
                "profile": worst_loss.get("profile"),
                "value": worst_loss.get("value"),
                "loss_pct": worst_loss.get("loss_pct"),
                "delta_vs_reference_pct": round(float(worst_loss.get("loss_pct") or 0.0) - reference_loss, 3),
            },
            "worst_latency": {
                "profile": worst_latency.get("profile"),
                "value": worst_latency.get("value"),
                "latency_p95_ms": worst_latency.get("latency_p95_ms"),
                "delta_vs_reference_ms": round(
                    float(worst_latency.get("latency_p95_ms") or 0.0) - reference_latency,
                    3,
                ),
            },
        }
    return {"reference_profile": reference.get("profile") if reference else None, "axes": axes}


def drive_sensitivity(
    run_point: RunPointFn,
    *,
    cases: Sequence[dict[str, Any]],
    max_loss_pct: float,
    max_latency_ms: float,
    rate_hz: float,
    size: int,
    topic: str,
    duration_s: float,
    out_dir: Path,
    result_context: dict[str, Any] | None = None,
    generated_profiles_file: Path | None = None,
) -> list[dict[str, Any]]:
    from .benchmark import OracleThresholds, oracle_passes_topic

    thresholds = OracleThresholds(max_loss_pct=max_loss_pct, max_latency_ms=max_latency_ms)
    rows: list[dict[str, Any]] = []
    measurements: list[dict[str, Any]] = []
    load = {"size_a": size, "rate": rate_hz}
    load_info = _load_context(load)

    for case in cases:
        profile = _normalize_benchmark_profile(cast(str | None, case.get("profile")))
        profile_label = _benchmark_profile_label(profile)
        summary = run_point(profile=profile, load=load, duration_s=duration_s, out_dir=out_dir)
        selected_topic, topic_data = _selected_topic(summary, topic)
        rows_for_print = _topic_rows(summary, topic)
        passes = oracle_passes_topic(topic_data, thresholds) if topic_data else False
        loss_pct = float(topic_data.get("loss_pct", 100.0)) if topic_data else 100.0
        latency_p95 = (topic_data.get("ota_hop_ms") or {}).get("p95") if topic_data else None
        jitter_p95 = (topic_data.get("jitter_ms") or {}).get("p95") if topic_data else None
        row = {
            "profile": profile_label,
            "axis": case.get("axis"),
            "value": case.get("value"),
            "description": case.get("description"),
            "passes": passes,
            "topic": selected_topic,
            "expected": topic_data.get("expected") if topic_data else None,
            "delivered": topic_data.get("delivered") if topic_data else None,
            "lost": topic_data.get("lost") if topic_data else None,
            "reordered": topic_data.get("reordered") if topic_data else None,
            "loss_pct": loss_pct,
            "latency_p95_ms": float(latency_p95) if latency_p95 is not None else None,
            "jitter_p95_ms": float(jitter_p95) if jitter_p95 is not None else None,
            "offered_bw_bps": float(load_info["offered_bandwidth_bps"] or 0.0),
        }
        rows.append(row)
        measurements.append({"case": case, "load": load_info, "topics": rows_for_print, "summary": summary})
        print(
            f"  sensitivity({profile_label}, axis={case.get('axis')}, value={case.get('value')}): "
            f"{'PASS' if passes else 'FAIL'} offered_bw={_format_bps(load_info.get('offered_bandwidth_bps'))} "
            f"{_format_topic_rows(rows_for_print)}"
        )

    sensitivity_path = out_dir / "sensitivity.jsonl"
    sensitivity_path.write_text(
        "\n".join(json.dumps(row, sort_keys=True) for row in rows) + "\n",
        encoding="utf-8",
    )
    analysis = _sensitivity_analysis(rows)
    _write_benchmark_result(
        out_dir,
        genre="sensitivity",
        context=result_context,
        configuration={
            "cases": list(cases),
            "load": load_info,
            "thresholds": {
                "max_loss_pct": max_loss_pct,
                "max_latency_ms": max_latency_ms,
                "latency_quantile": thresholds.latency_quantile,
            },
            "topic": topic or None,
            "duration_s": duration_s,
            "generated_profiles_file": generated_profiles_file.name if generated_profiles_file else None,
        },
        result={"rows": rows, "analysis": analysis},
        measurements={"points": measurements},
        verdict={
            "passed": all(row["passes"] for row in rows),
            "status": "all_cases_passed" if all(row["passes"] for row in rows) else "case_failures",
        },
        artifacts={
            "sensitivity": sensitivity_path.name,
            "generated_profiles": generated_profiles_file.name if generated_profiles_file else None,
            "stdout": "stdout.txt",
            "probes_dir": "probes",
        },
    )
    print(f"Sensitivity rows saved to {sensitivity_path}")
    return rows


def drive_ab(
    run_point: RunPointFn,
    *,
    configs: Sequence[dict[str, Any]],
    baseline_label: str,
    profile: str | None,
    load: dict[str, Any],
    duration_s: float,
    repeats: int,
    metrics: Sequence[str],
    tolerances: dict[str, Any],
    topic: str,
    out_dir: Path,
    result_context: dict[str, Any] | None = None,
    config_diffs: dict[str, str] | None = None,
) -> dict[str, Any]:
    """Run baseline + candidates interleaved and classify each candidate.

    ``configs`` are ``{"label", "is_baseline", "source", "configs_root"}`` dicts;
    ``run_point`` receives a ``config`` kwarg naming the active one so the CLI
    wrapper can point ``session_configs_dir`` at it (a stub in tests reads it to
    return config-specific summaries). The load and profile are held identical
    across every run — only the config changes.
    """
    from .benchmark import AbRun, ab_schedule, ab_verdict, render_ab_markdown

    labels = [str(config["label"]) for config in configs]
    config_by_label = {str(config["label"]): config for config in configs}
    schedule = ab_schedule(labels, repeats)

    profile = _normalize_benchmark_profile(profile)
    profile_label = _benchmark_profile_label(profile)
    load_info = _load_context(load)

    runs: list[AbRun] = []
    run_rows: list[dict[str, Any]] = []
    for order, (label, repeat) in enumerate(schedule, start=1):
        config = config_by_label[label]
        run_dir = out_dir / "runs" / f"{order:03d}_{_safe_case_token(label)}_r{repeat}"
        run_dir.mkdir(parents=True, exist_ok=True)
        summary = run_point(profile=profile, load=load, duration_s=duration_s, out_dir=run_dir, config=config)
        runs.append(AbRun(config=label, repeat=repeat, summary=summary))
        rows = _topic_rows(summary, topic)
        run_rows.append(
            {
                "order": order,
                "config": label,
                "repeat": repeat,
                "is_baseline": bool(config.get("is_baseline")),
                "profile": profile_label,
                "artifact_dir": str(run_dir.relative_to(out_dir)),
                "topics": rows,
            }
        )
        print(
            f"  ab[{order}/{len(schedule)}] config={label} repeat={repeat}: "
            f"offered_bw={_format_bps(load_info.get('offered_bandwidth_bps'))} {_format_topic_rows(rows)}"
        )

    report = ab_verdict(runs, baseline=baseline_label, metrics=metrics, tolerances=tolerances, topic=topic)
    report_doc = cast(dict[str, Any], _jsonable(report))

    ab_rows_path = out_dir / "ab.jsonl"
    ab_rows_path.write_text(
        "\n".join(json.dumps(row, sort_keys=True) for row in run_rows) + "\n",
        encoding="utf-8",
    )
    markdown = render_ab_markdown(report)
    ab_md_path = out_dir / "ab.md"
    ab_md_path.write_text(markdown + "\n", encoding="utf-8")

    _write_benchmark_result(
        out_dir,
        genre="ab",
        context=result_context,
        configuration={
            "baseline": baseline_label,
            "configs": [
                {
                    "label": config["label"],
                    "is_baseline": bool(config.get("is_baseline")),
                    "source": str(config["source"]),
                }
                for config in configs
            ],
            "profile": profile_label,
            "load": load_info,
            "duration_s": duration_s,
            "repeats": repeats,
            "metrics": list(metrics),
            "tolerances": {metric: {"rel": tol.rel, "abs": tol.abs} for metric, tol in tolerances.items()},
            "topic": topic or None,
            "schedule": [{"order": i, "config": lbl, "repeat": rp} for i, (lbl, rp) in enumerate(schedule, start=1)],
        },
        result=report_doc,
        measurements={"runs": run_rows},
        verdict={
            "passed": report.passed,
            "status": "no_regression" if report.passed else "candidate_regressed",
            "regressed": {
                candidate.config: candidate.regressed
                for candidate in report.candidates
                if candidate.regressed or candidate.dropped_topics
            },
            "improved": {candidate.config: candidate.improved for candidate in report.candidates if candidate.improved},
        },
        artifacts={
            "ab_rows": ab_rows_path.name,
            "ab_markdown": ab_md_path.name,
            "configs_dir": "configs",
            "config_diffs": dict(config_diffs or {}),
            "stdout": "stdout.txt",
        },
    )
    print()
    print(markdown)
    print()
    print(f"A/B verdict: {'PASS' if report.passed else 'FAIL'} — rows saved to {ab_rows_path}")
    return report_doc


def drive_matrix(
    run_point: RunPointFn,
    *,
    cases: Sequence[dict[str, Any]],
    max_loss_pct: float,
    max_latency_ms: float,
    topic: str,
    out_dir: Path,
    result_context: dict[str, Any] | None = None,
    generated_profiles_file: Path | None = None,
) -> list[dict[str, Any]]:
    from .benchmark import OracleThresholds, oracle_passes_topic

    thresholds = OracleThresholds(max_loss_pct=max_loss_pct, max_latency_ms=max_latency_ms)
    rows: list[dict[str, Any]] = []
    measurements: list[dict[str, Any]] = []

    for case in cases:
        profile = _normalize_benchmark_profile(cast(str | None, case.get("profile")))
        profile_label = _benchmark_profile_label(profile)
        load = dict(cast(dict[str, Any], case.get("load", {})))
        load_info = _load_context(load)
        duration_s = float(case.get("duration_s", 0.0) or 0.0)
        summary = run_point(
            profile=profile,
            load=load,
            duration_s=duration_s,
            out_dir=out_dir,
            session_options=case.get("session_options"),
        )
        selected_topic, topic_data = _selected_topic(summary, topic)
        rows_for_print = _topic_rows(summary, topic)
        passes = oracle_passes_topic(topic_data, thresholds) if topic_data else False
        loss_pct = float(topic_data.get("loss_pct", 100.0)) if topic_data else 100.0
        latency_p95 = (topic_data.get("ota_hop_ms") or {}).get("p95") if topic_data else None
        jitter_p95 = (topic_data.get("jitter_ms") or {}).get("p95") if topic_data else None
        row = {
            "profile": profile_label,
            "axis": case.get("axis"),
            "value": case.get("value"),
            "description": case.get("description"),
            "qos": case.get("qos"),
            "duration_s": duration_s,
            "load": load_info,
            "passes": passes,
            "topic": selected_topic,
            "expected": topic_data.get("expected") if topic_data else None,
            "delivered": topic_data.get("delivered") if topic_data else None,
            "lost": topic_data.get("lost") if topic_data else None,
            "reordered": topic_data.get("reordered") if topic_data else None,
            "loss_pct": loss_pct,
            "latency_p95_ms": float(latency_p95) if latency_p95 is not None else None,
            "jitter_p95_ms": float(jitter_p95) if jitter_p95 is not None else None,
            "offered_bw_bps": float(load_info["offered_bandwidth_bps"] or 0.0),
        }
        rows.append(row)
        measurements.append({"case": case, "load": load_info, "topics": rows_for_print, "summary": summary})
        print(
            f"  matrix({case.get('axis')}={case.get('value')}, qos={case.get('qos')}, "
            f"duration={duration_s:g}s): {'PASS' if passes else 'FAIL'} "
            f"offered_bw={_format_bps(load_info.get('offered_bandwidth_bps'))} {_format_topic_rows(rows_for_print)}"
        )

    matrix_path = out_dir / "matrix.jsonl"
    matrix_path.write_text(
        "\n".join(json.dumps(row, sort_keys=True) for row in rows) + "\n",
        encoding="utf-8",
    )
    analysis = _sensitivity_analysis(rows)
    _write_benchmark_result(
        out_dir,
        genre="matrix",
        context=result_context,
        configuration={
            "cases": list(cases),
            "thresholds": {
                "max_loss_pct": max_loss_pct,
                "max_latency_ms": max_latency_ms,
                "latency_quantile": thresholds.latency_quantile,
            },
            "topic": topic or None,
            "generated_profiles_file": generated_profiles_file.name if generated_profiles_file else None,
        },
        result={"rows": rows, "analysis": analysis},
        measurements={"points": measurements},
        verdict={
            "passed": all(row["passes"] for row in rows),
            "status": "all_cases_passed" if all(row["passes"] for row in rows) else "case_failures",
        },
        artifacts={
            "matrix": matrix_path.name,
            "generated_profiles": generated_profiles_file.name if generated_profiles_file else None,
            "stdout": "stdout.txt",
            "probes_dir": "probes",
        },
    )
    print(f"Matrix rows saved to {matrix_path}")
    return rows


# --------------------------------------------------------------------------- #
# Benchmark driver: requirements
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class _RequirementCandidate:
    bandwidth_bps: float
    latency_ms: float = 0.0
    jitter_ms: float = 0.0
    loss_pct: float = 0.0


def _parse_requirements_axes(raw: str) -> list[str]:
    available = ("bandwidth", "latency", "jitter", "loss")
    aliases = {"rate": "bandwidth", "bw": "bandwidth", "delay": "latency"}
    axes = [aliases.get(value.strip().lower(), value.strip().lower()) for value in raw.split(",") if value.strip()]
    if not axes or "all" in axes:
        return list(available)
    unknown = sorted(set(axes) - set(available))
    if unknown:
        raise ValueError(f"Unknown requirements axes: {', '.join(unknown)}")
    return [axis for axis in available if axis in axes]


def _requirements_axis_value(candidate: _RequirementCandidate, axis: str) -> float:
    return {
        "bandwidth": candidate.bandwidth_bps,
        "latency": candidate.latency_ms,
        "jitter": candidate.jitter_ms,
        "jitter_loss": candidate.jitter_ms,
        "loss": candidate.loss_pct,
    }[axis]


def _requirements_with_axis(candidate: _RequirementCandidate, axis: str, value: float) -> _RequirementCandidate:
    updates = {
        "bandwidth_bps": candidate.bandwidth_bps,
        "latency_ms": candidate.latency_ms,
        "jitter_ms": candidate.jitter_ms,
        "loss_pct": candidate.loss_pct,
    }
    if axis == "bandwidth":
        updates["bandwidth_bps"] = value
    elif axis == "latency":
        updates["latency_ms"] = value
    elif axis == "jitter":
        updates["jitter_ms"] = value
    elif axis == "loss":
        updates["loss_pct"] = value
    else:
        raise ValueError(f"Unknown requirements axis {axis!r}.")
    return _RequirementCandidate(**updates)


def _requirements_candidate_context(
    candidate: _RequirementCandidate,
    *,
    downlink_ratio: float = 1.0,
    downlink_mode: str = "mirror",
) -> dict[str, Any]:
    downlink_latency_ms = 0.0 if downlink_mode == "lan" else candidate.latency_ms * downlink_ratio
    downlink_bandwidth_bps = None if downlink_mode == "lan" else candidate.bandwidth_bps
    downlink_jitter_ms = 0.0 if downlink_mode == "lan" else candidate.jitter_ms
    downlink_loss_pct = 0.0 if downlink_mode == "lan" else candidate.loss_pct
    return {
        "bandwidth_bps": candidate.bandwidth_bps,
        "bandwidth": _format_yaml_rate(candidate.bandwidth_bps),
        "uplink_latency_ms": candidate.latency_ms,
        "downlink_mode": downlink_mode,
        "downlink_bandwidth_bps": downlink_bandwidth_bps,
        "downlink_bandwidth": _format_yaml_rate(downlink_bandwidth_bps) if downlink_bandwidth_bps else None,
        "downlink_latency_ms": downlink_latency_ms,
        "downlink_jitter_ms": downlink_jitter_ms,
        "downlink_loss_pct": downlink_loss_pct,
        "jitter_ms": candidate.jitter_ms,
        "loss_pct": candidate.loss_pct,
    }


def _requirements_profile_spec(
    candidate: _RequirementCandidate,
    *,
    distribution: str,
    downlink_ratio: float,
    downlink_mode: str,
    netem_seed: int | None = None,
) -> dict[str, Any]:
    latency_downlink_ms = candidate.latency_ms * downlink_ratio
    delay_uplink = candidate.latency_ms if candidate.latency_ms > 0.0 or candidate.jitter_ms > 0.0 else None
    delay_downlink = latency_downlink_ms if latency_downlink_ms > 0.0 or candidate.jitter_ms > 0.0 else None
    loss = candidate.loss_pct if candidate.loss_pct > 0.0 else None
    downlink = (
        {}
        if downlink_mode == "lan"
        else _direction_to_yaml(
            rate=_format_yaml_rate(candidate.bandwidth_bps),
            delay_ms=delay_downlink,
            jitter_ms=candidate.jitter_ms if candidate.jitter_ms > 0.0 else None,
            distribution=distribution,
            loss_pct=loss,
            seed=netem_seed if delay_downlink is not None or loss is not None else None,
        )
    )
    return {
        "uplink": _direction_to_yaml(
            rate=_format_yaml_rate(candidate.bandwidth_bps),
            delay_ms=delay_uplink,
            jitter_ms=candidate.jitter_ms if candidate.jitter_ms > 0.0 else None,
            distribution=distribution,
            loss_pct=loss,
            seed=netem_seed if delay_uplink is not None or loss is not None else None,
        ),
        "downlink": downlink,
    }


def _requirements_profile_name(prefix: str, index: int, axis: str, candidate: _RequirementCandidate) -> str:
    parts = [
        prefix,
        f"{index:03d}",
        axis,
        _safe_case_token(_format_yaml_rate(candidate.bandwidth_bps) or str(candidate.bandwidth_bps)),
        f"lat{candidate.latency_ms:g}ms",
        f"jit{candidate.jitter_ms:g}ms",
        f"loss{candidate.loss_pct:g}pct",
    ]
    return _safe_case_token("_".join(parts))


def _requirements_row_summary(row: dict[str, Any]) -> str:
    return (
        f"{row['axis']}[{row['phase']}] "
        f"bw={_format_bps(row['candidate']['bandwidth_bps'])} "
        f"lat={row['candidate']['uplink_latency_ms']:g}ms "
        f"jitter={row['candidate']['jitter_ms']:g}ms "
        f"loss={row['candidate']['loss_pct']:g}%"
    )


def _requirements_jitter_loss_pct(jitter_ms: float) -> float:
    """Practical jitter/loss coupling curve for mobile-ish shaped links."""
    points = (
        (0.0, 0.0),
        (10.0, 0.1),
        (15.0, 0.2),
        (20.0, 0.5),
        (30.0, 1.0),
        (50.0, 3.0),
    )
    if jitter_ms <= points[0][0]:
        return points[0][1]
    for (left_jitter, left_loss), (right_jitter, right_loss) in zip(points, points[1:], strict=False):
        if jitter_ms <= right_jitter:
            span = right_jitter - left_jitter
            fraction = (jitter_ms - left_jitter) / span if span else 0.0
            return left_loss + (right_loss - left_loss) * fraction
    last_jitter, last_loss = points[-1]
    prev_jitter, prev_loss = points[-2]
    slope = (last_loss - prev_loss) / (last_jitter - prev_jitter)
    return last_loss + (jitter_ms - last_jitter) * slope


def _requirements_target_quality(row: dict[str, Any], *, max_loss_pct: float, max_latency_ms: float) -> dict[str, Any]:
    loss_pct = row.get("loss_pct")
    latency_p95_ms = row.get("latency_p95_ms")
    loss_ratio: float | None
    if loss_pct is None:
        loss_ratio = None
    elif max_loss_pct > 0:
        loss_ratio = float(loss_pct) / max_loss_pct
    elif float(loss_pct) <= 0.0:
        loss_ratio = 0.0
    else:
        loss_ratio = math.inf
    latency_ratio = (
        float(latency_p95_ms) / max_latency_ms if latency_p95_ms is not None and max_latency_ms > 0 else None
    )
    known_ratios: dict[str, float] = {}
    if loss_ratio is not None:
        known_ratios["loss"] = loss_ratio
    if latency_ratio is not None:
        known_ratios["latency"] = latency_ratio
    limiting_metric = max(known_ratios, key=lambda key: known_ratios[key]) if known_ratios else None
    utilization = known_ratios[limiting_metric] if limiting_metric else None
    return {
        "loss_margin_pct": max_loss_pct - float(loss_pct) if loss_pct is not None else None,
        "latency_margin_ms": max_latency_ms - float(latency_p95_ms) if latency_p95_ms is not None else None,
        "loss_ratio": loss_ratio,
        "latency_ratio": latency_ratio,
        "utilization": utilization,
        "limiting_metric": limiting_metric,
    }


def _format_requirements_target_quality(quality: dict[str, Any]) -> str:
    sample_count = int(quality.get("repeat_sample_count") or 1)
    if sample_count > 1:
        return (
            f"target={quality.get('repeat_pass_count')}/{quality.get('repeat_required_passes')}pass"
            f" lossfree={quality.get('repeat_loss_free_count')}/{sample_count}"
        )
    utilization = quality.get("utilization")
    limiting = quality.get("limiting_metric") or "unknown"
    if utilization is None:
        return "target=unknown"
    if math.isinf(float(utilization)):
        return f"target=over-limit({limiting})"
    return f"target={utilization * 100:.1f}%({limiting})"


def _format_requirements_profile(
    candidate: _RequirementCandidate,
    *,
    downlink_ratio: float,
    downlink_mode: str,
) -> str:
    downlink_latency_ms = candidate.latency_ms * downlink_ratio
    downlink = (
        "downlink(mode=lan/unshaped)"
        if downlink_mode == "lan"
        else (
            f"downlink(rate={_format_bps(candidate.bandwidth_bps)}, delay={downlink_latency_ms:g}ms, "
            f"jitter={candidate.jitter_ms:g}ms, loss={candidate.loss_pct:g}%)"
        )
    )
    return (
        f"uplink(rate={_format_bps(candidate.bandwidth_bps)}, delay={candidate.latency_ms:g}ms, "
        f"jitter={candidate.jitter_ms:g}ms, loss={candidate.loss_pct:g}%), "
        f"{downlink}"
    )


def _requirements_default_min_passes(
    *,
    probe_repeats: int,
    max_loss_pct: float,
    explicit_min_passes: int | None,
) -> int:
    if probe_repeats < 1:
        raise ValueError("probe_repeats must be >= 1.")
    if explicit_min_passes is not None:
        if not 1 <= explicit_min_passes <= probe_repeats:
            raise ValueError("probe_min_passes must be within [1, probe_repeats].")
        return explicit_min_passes
    if max_loss_pct <= 0.0 and probe_repeats > 1:
        return max(1, math.ceil(probe_repeats * 0.9))
    return probe_repeats


def _requirements_order_axes(axes: Sequence[str], *, strict_zero_loss_target: bool, search_order: str) -> list[str]:
    if search_order not in {"auto", "input"}:
        raise ValueError("search_order must be 'auto' or 'input'.")
    ordered = list(axes)
    if search_order == "auto" and strict_zero_loss_target:
        priority = {"jitter": 0, "jitter_loss": 0, "bandwidth": 1, "latency": 2, "loss": 3}
        ordered = sorted(ordered, key=lambda axis: priority.get(axis, 99))
    return ordered


def _format_requirements_repeat(row: dict[str, Any]) -> str:
    repeat = row.get("repeat") or {}
    sample_count = int(repeat.get("sample_count") or 1)
    if sample_count <= 1:
        return ""
    return (
        f" repeat={repeat.get('pass_count')}/{repeat.get('required_passes')}pass"
        f" lossfree={repeat.get('loss_free_count')}/{sample_count}"
        f" lossy={repeat.get('lossy_count')}/{sample_count}"
    )


def drive_requirements(
    run_point: RunPointFn,
    *,
    max_loss_pct: float,
    max_latency_ms: float,
    rate_hz: float,
    size: int,
    streams: int,
    qos_reliability: str,
    qos_depth: int,
    topic: str,
    out_dir: Path,
    bandwidth_high_bps: float,
    bandwidth_low_bps: float,
    latency_base_ms: float,
    latency_high_ms: float,
    jitter_high_ms: float,
    loss_high_pct: float,
    axes: Sequence[str],
    min_duration_s: float,
    min_messages: int,
    search_iterations: int,
    search_rounds: int,
    distribution: str,
    final_refine_iterations: int = 2,
    loss_coupling: str = "jitter",
    downlink_ratio: float = 1.0,
    downlink_mode: str = "mirror",
    probe_repeats: int = 1,
    probe_min_passes: int | None = None,
    bad_lossy_count: int | None = None,
    bandwidth_probe_repeats: int | None = None,
    search_order: str = "auto",
    netem_seed: int | None = None,
    jitter_guard_ratio: float = 0.0,
    bandwidth_guard_ratio: float = 0.0,
    result_context: dict[str, Any] | None = None,
    generated_profiles_file: Path | None = None,
    profile_prefix: str = "requirements",
    ground_truth: dict[str, dict[str, Any]] | None = None,
    oracle_min_completeness: float = 0.95,
    required_topics: Sequence[str] | None = None,
    target: dict[str, Any] | None = None,
    bag: dict[str, Any] | None = None,
) -> dict[str, Any]:
    from .benchmark import OracleThresholds

    bag_mode = target is not None or ground_truth is not None or required_topics is not None
    if bandwidth_low_bps <= 0.0 or bandwidth_high_bps <= 0.0:
        raise ValueError("Bandwidth bounds must be > 0.")
    if bandwidth_low_bps >= bandwidth_high_bps:
        raise ValueError("bandwidth-low must be smaller than bandwidth-high.")
    if latency_base_ms < 0.0:
        raise ValueError("latency-base-ms must be >= 0.")
    if latency_base_ms > latency_high_ms:
        raise ValueError("latency-base-ms must be <= latency-high-ms.")
    if search_iterations < 1:
        raise ValueError("search_iterations must be >= 1.")
    if search_rounds < 1:
        raise ValueError("search_rounds must be >= 1.")
    if final_refine_iterations < 0:
        raise ValueError("final_refine_iterations must be >= 0.")
    if loss_coupling not in {"independent", "jitter"}:
        raise ValueError("loss_coupling must be 'independent' or 'jitter'.")
    if downlink_mode not in {"mirror", "lan"}:
        raise ValueError("downlink_mode must be 'mirror' or 'lan'.")
    if not 0.0 <= jitter_guard_ratio < 1.0:
        raise ValueError("jitter-guard-ratio must be within [0, 1).")
    if bandwidth_guard_ratio < 0.0:
        raise ValueError("bandwidth-guard-ratio must be >= 0.")

    thresholds = OracleThresholds(max_loss_pct=max_loss_pct, max_latency_ms=max_latency_ms)
    required_passes = _requirements_default_min_passes(
        probe_repeats=probe_repeats,
        max_loss_pct=max_loss_pct,
        explicit_min_passes=probe_min_passes,
    )
    bad_lossy_threshold = bad_lossy_count if bad_lossy_count is not None else probe_repeats
    if not 1 <= bad_lossy_threshold <= probe_repeats:
        raise ValueError("bad_lossy_count must be within [1, probe_repeats].")
    bandwidth_repeats = bandwidth_probe_repeats if bandwidth_probe_repeats is not None else probe_repeats
    if bandwidth_repeats < 1:
        raise ValueError("bandwidth_probe_repeats must be >= 1.")
    bandwidth_required_passes = _requirements_default_min_passes(
        probe_repeats=bandwidth_repeats,
        max_loss_pct=max_loss_pct,
        explicit_min_passes=min(probe_min_passes, bandwidth_repeats) if probe_min_passes is not None else None,
    )
    bandwidth_bad_lossy_threshold = min(bad_lossy_threshold, bandwidth_repeats)
    requested_axes = list(axes)
    selected_axes = list(requested_axes)
    strict_zero_loss_target = max_loss_pct <= 0.0
    effective_loss_coupling = "zero_loss" if strict_zero_loss_target else loss_coupling
    skipped_axes: list[str] = []
    if strict_zero_loss_target:
        skipped_axes = [axis for axis in selected_axes if axis == "loss"]
        selected_axes = [axis for axis in selected_axes if axis != "loss"]
    elif loss_coupling == "jitter" and "jitter" in selected_axes and "loss" in selected_axes:
        selected_axes = ["jitter_loss" if axis == "jitter" else axis for axis in selected_axes if axis != "loss"]
    selected_axes = _requirements_order_axes(
        selected_axes,
        strict_zero_loss_target=strict_zero_loss_target,
        search_order=search_order,
    )
    load = {"size_a": size, "rate": rate_hz, "streams": streams}
    load_info = _load_context(load)
    duration_s = _duration_for_min_messages(rate_hz, min_duration_s=min_duration_s, min_messages=min_messages)
    ideal = _RequirementCandidate(bandwidth_bps=bandwidth_high_bps, latency_ms=latency_base_ms)
    candidate = ideal
    rows: list[dict[str, Any]] = []
    measurements: list[dict[str, Any]] = []
    bounds: dict[str, Any] = {}
    generated_doc: dict[str, Any] = {
        "profiles": {},
        "metadata": {
            "kind": "benchmark-requirements-generated",
            "stream": {
                "rate_hz": rate_hz,
                "size_bytes": size,
                "qos": {"reliability": qos_reliability, "depth": qos_depth},
                "offered_bandwidth_bps": load_info.get("offered_bandwidth_bps"),
            },
            "thresholds": {
                "max_loss_pct": max_loss_pct,
                "max_latency_ms": max_latency_ms,
                "latency_quantile": thresholds.latency_quantile,
            },
            "search": {
                "requested_axes": requested_axes,
                "axes": selected_axes,
                "skipped_axes": skipped_axes,
                "bandwidth_low_bps": bandwidth_low_bps,
                "bandwidth_high_bps": bandwidth_high_bps,
                "latency_base_ms": latency_base_ms,
                "latency_high_ms": latency_high_ms,
                "jitter_high_ms": jitter_high_ms,
                "loss_high_pct": loss_high_pct,
                "iterations": search_iterations,
                "rounds": search_rounds,
                "final_refine_iterations": final_refine_iterations,
                "distribution": distribution,
                "loss_coupling": loss_coupling,
                "effective_loss_coupling": effective_loss_coupling,
                "strict_zero_loss_target": strict_zero_loss_target,
                "downlink_ratio": downlink_ratio,
                "downlink_mode": downlink_mode,
                "probe_repeats": probe_repeats,
                "probe_min_passes": required_passes,
                "bad_lossy_count": bad_lossy_threshold,
                "bandwidth_probe_repeats": bandwidth_repeats,
                "bandwidth_probe_min_passes": bandwidth_required_passes,
                "bandwidth_bad_lossy_count": bandwidth_bad_lossy_threshold,
                "search_order": search_order,
                "netem_seed": netem_seed,
                "jitter_guard_ratio": jitter_guard_ratio,
                "bandwidth_guard_ratio": bandwidth_guard_ratio,
            },
        },
    }
    profile_counter = 0

    def write_profiles() -> None:
        if generated_profiles_file is not None:
            generated_profiles_file.write_text(yaml.safe_dump(generated_doc, sort_keys=False), encoding="utf-8")

    def probe(candidate_to_probe: _RequirementCandidate, *, axis: str, phase: str) -> dict[str, Any]:
        nonlocal profile_counter
        local_probe_repeats = bandwidth_repeats if axis == "bandwidth" else probe_repeats
        local_required_passes = bandwidth_required_passes if axis == "bandwidth" else required_passes
        local_bad_lossy_threshold = bandwidth_bad_lossy_threshold if axis == "bandwidth" else bad_lossy_threshold
        profile_counter += 1
        profile_name = _requirements_profile_name(profile_prefix, profile_counter, axis, candidate_to_probe)
        generated_doc["profiles"][profile_name] = _requirements_profile_spec(
            candidate_to_probe,
            distribution=distribution,
            downlink_ratio=downlink_ratio,
            downlink_mode=downlink_mode,
            netem_seed=netem_seed,
        )
        write_profiles()
        samples: list[dict[str, Any]] = []
        sample_measurements: list[dict[str, Any]] = []
        selected_topic = ""
        for repeat_index in range(1, local_probe_repeats + 1):
            summary = run_point(profile=profile_name, load=load, duration_s=duration_s, out_dir=out_dir)
            verdict = _probe_sample_verdict(
                summary,
                thresholds=thresholds,
                topic=topic,
                bag_mode=bag_mode,
                ground_truth=ground_truth,
                min_completeness=oracle_min_completeness,
                required_topics=required_topics,
            )
            rows_for_print = _topic_rows(summary, "" if bag_mode else topic)
            if verdict["topic"] and not selected_topic:
                selected_topic = verdict["topic"]
            sample = {
                "repeat": repeat_index,
                "passes": verdict["passes"],
                "topic": verdict["topic"],
                "expected": verdict["expected"],
                "delivered": verdict["delivered"],
                "lost": verdict["lost"],
                "reordered": verdict["reordered"],
                "loss_pct": verdict["loss_pct"],
                "latency_p95_ms": verdict["latency_p95_ms"],
                "jitter_p95_ms": verdict["jitter_p95_ms"],
                "loss_free": verdict["loss_free"],
            }
            if verdict["failing_topics"] is not None:
                sample["failing_topics"] = verdict["failing_topics"]
                sample["per_topic"] = verdict["per_topic"]
            sample["target_quality"] = _requirements_target_quality(
                sample,
                max_loss_pct=max_loss_pct,
                max_latency_ms=max_latency_ms,
            )
            samples.append(sample)
            sample_measurements.append(
                {
                    "repeat": repeat_index,
                    "topics": rows_for_print,
                    "summary": summary,
                    "sample": sample,
                }
            )
            running_pass_count = sum(1 for item in samples if item["passes"])
            running_lossy_count = sum(1 for item in samples if not item["loss_free"])
            remaining = local_probe_repeats - repeat_index
            pass_impossible = running_pass_count + remaining < local_required_passes
            bad_achieved = running_lossy_count >= local_bad_lossy_threshold
            bad_still_possible = running_lossy_count + remaining >= local_bad_lossy_threshold
            if pass_impossible and (bad_achieved or not bad_still_possible):
                break

        pass_count = sum(1 for sample in samples if sample["passes"])
        loss_free_count = sum(1 for sample in samples if sample["loss_free"])
        lossy_count = len(samples) - loss_free_count
        expected_total = sum(int(sample["expected"]) for sample in samples if sample.get("expected") is not None)
        delivered_total = sum(int(sample["delivered"]) for sample in samples if sample.get("delivered") is not None)
        lost_total = sum(int(sample["lost"]) for sample in samples if sample.get("lost") is not None)
        reordered_total = sum(int(sample["reordered"]) for sample in samples if sample.get("reordered") is not None)
        loss_values = [float(sample["loss_pct"]) for sample in samples if sample.get("loss_pct") is not None]
        latency_values = [
            float(sample["latency_p95_ms"]) for sample in samples if sample.get("latency_p95_ms") is not None
        ]
        jitter_values = [
            float(sample["jitter_p95_ms"]) for sample in samples if sample.get("jitter_p95_ms") is not None
        ]
        if len(samples) == 1 and loss_values:
            loss_pct = loss_values[0]
        elif expected_total > 0:
            loss_pct = lost_total / expected_total * 100.0
        else:
            loss_pct = max(loss_values) if loss_values else 100.0
        latency_p95 = max(latency_values) if latency_values else None
        jitter_p95 = max(jitter_values) if jitter_values else None
        passes = pass_count >= local_required_passes
        repeat_summary = {
            "sample_count": len(samples),
            "configured_repeats": local_probe_repeats,
            "required_passes": local_required_passes,
            "pass_count": pass_count,
            "fail_count": len(samples) - pass_count,
            "pass_ratio": pass_count / len(samples) if samples else 0.0,
            "loss_free_count": loss_free_count,
            "loss_free_ratio": loss_free_count / len(samples) if samples else 0.0,
            "lossy_count": lossy_count,
            "bad_lossy_count": local_bad_lossy_threshold,
            "good_case": pass_count >= local_required_passes,
            "bad_case": lossy_count >= local_bad_lossy_threshold,
        }
        row = {
            "profile": profile_name,
            "axis": axis,
            "phase": phase,
            "candidate": _requirements_candidate_context(
                candidate_to_probe,
                downlink_ratio=downlink_ratio,
                downlink_mode=downlink_mode,
            ),
            "passes": passes,
            "topic": selected_topic,
            "expected": expected_total,
            "delivered": delivered_total,
            "lost": lost_total,
            "reordered": reordered_total,
            "loss_pct": loss_pct,
            "latency_p95_ms": float(latency_p95) if latency_p95 is not None else None,
            "jitter_p95_ms": float(jitter_p95) if jitter_p95 is not None else None,
            "load": load_info,
            "duration_s": duration_s,
            "qos": {"reliability": qos_reliability, "depth": qos_depth},
            "repeat": repeat_summary,
            "samples": samples,
        }
        if bag_mode:
            row["failing_topics"] = sorted({t for s in samples for t in (s.get("failing_topics") or [])})
        target_quality = _requirements_target_quality(
            row,
            max_loss_pct=max_loss_pct,
            max_latency_ms=max_latency_ms,
        )
        if local_probe_repeats > 1:
            target_quality.update(
                {
                    "repeat_sample_count": len(samples),
                    "repeat_required_passes": local_required_passes,
                    "repeat_pass_count": pass_count,
                    "repeat_loss_free_count": loss_free_count,
                    "repeat_bad_lossy_count": local_bad_lossy_threshold,
                }
            )
            if not passes or strict_zero_loss_target:
                target_quality["limiting_metric"] = "repeat"
            target_quality["utilization"] = local_required_passes / pass_count if pass_count > 0 else math.inf
        row["target_quality"] = target_quality
        rows.append(row)
        measurements.append({"row": row, "samples": sample_measurements})
        print(
            f"  requirements({_requirements_row_summary(row)}): {'PASS' if passes else 'FAIL'} "
            f"offered_bw={_format_bps(load_info.get('offered_bandwidth_bps'))} "
            f"{_format_requirements_target_quality(target_quality)}{_format_requirements_repeat(row)} "
            f"{selected_topic or 'topic'}: delivered={delivered_total}/{expected_total} "
            f"lost={lost_total} loss={round(loss_pct, 3)}% p95={row['latency_p95_ms']}ms "
            f"jitter_p95={row['jitter_p95_ms']}ms"
        )
        return row

    def last_row_ref(row: dict[str, Any] | None) -> dict[str, Any] | None:
        if row is None:
            return None
        return {
            "profile": row.get("profile"),
            "candidate": row.get("candidate"),
            "passes": row.get("passes"),
            "loss_pct": row.get("loss_pct"),
            "latency_p95_ms": row.get("latency_p95_ms"),
            "repeat": row.get("repeat"),
        }

    def is_bad_case(row: dict[str, Any] | None) -> bool:
        repeat = row.get("repeat") if row else None
        return bool(isinstance(repeat, dict) and repeat.get("bad_case"))

    def candidate_at_axis(base: _RequirementCandidate, axis: str, value: float) -> _RequirementCandidate:
        effective_axis = "jitter" if axis == "jitter_loss" else axis
        candidate_at_value = _requirements_with_axis(base, effective_axis, value)
        if effective_loss_coupling == "jitter" and axis in {"jitter", "jitter_loss"}:
            modeled_loss = min(loss_high_pct, max(0.0, _requirements_jitter_loss_pct(value)))
            candidate_at_value = _requirements_with_axis(candidate_at_value, "loss", modeled_loss)
        return candidate_at_value

    def axis_ceiling(axis: str) -> float:
        return {
            "latency": latency_high_ms,
            "jitter": jitter_high_ms,
            "jitter_loss": jitter_high_ms,
            "loss": loss_high_pct,
        }[axis]

    def bound_axis_value(axis: str, bound: dict[str, Any] | None, ref_name: str, default: float) -> float:
        if not bound:
            return default
        ref = bound.get(ref_name)
        if not isinstance(ref, dict):
            return default
        candidate_ref = ref.get("candidate")
        if not isinstance(candidate_ref, dict):
            return default
        key = {
            "bandwidth": "bandwidth_bps",
            "latency": "uplink_latency_ms",
            "jitter": "jitter_ms",
            "jitter_loss": "jitter_ms",
            "loss": "loss_pct",
        }[axis]
        value = candidate_ref.get(key)
        return float(value) if value is not None else default

    def search_axis(
        axis: str,
        current: _RequirementCandidate,
        *,
        phase_label: str,
        iterations: int,
        low_value: float | None = None,
        high_value: float | None = None,
    ) -> tuple[_RequirementCandidate, dict[str, Any]]:
        if axis == "bandwidth":
            floor_value = low_value if low_value is not None else bandwidth_low_bps
            low_row = probe(
                candidate_at_axis(current, axis, floor_value),
                axis=axis,
                phase=f"{phase_label}:floor",
            )
            if low_row["passes"]:
                selected = candidate_at_axis(current, axis, floor_value)
                if floor_value > bandwidth_low_bps:
                    reset_row = probe(
                        candidate_at_axis(current, axis, bandwidth_low_bps),
                        axis=axis,
                        phase=f"{phase_label}:floor-reset",
                    )
                    if not reset_row["passes"]:
                        pass_candidate = selected
                        fail_candidate = candidate_at_axis(current, axis, bandwidth_low_bps)
                        rebound_pass_row: dict[str, Any] | None = low_row
                        rebound_fail_row: dict[str, Any] | None = reset_row
                        rebound_bad_row: dict[str, Any] | None = reset_row if is_bad_case(reset_row) else None
                        for iteration in range(iterations):
                            mid = math.sqrt(pass_candidate.bandwidth_bps * fail_candidate.bandwidth_bps)
                            mid_candidate = candidate_at_axis(current, axis, mid)
                            mid_row = probe(mid_candidate, axis=axis, phase=f"{phase_label}:rebound{iteration + 1}")
                            if mid_row["passes"]:
                                pass_candidate = mid_candidate
                                rebound_pass_row = mid_row
                            else:
                                fail_candidate = mid_candidate
                                rebound_fail_row = mid_row
                                if is_bad_case(mid_row):
                                    rebound_bad_row = mid_row
                        return pass_candidate, {
                            "axis": axis,
                            "selected": _requirements_candidate_context(
                                pass_candidate,
                                downlink_ratio=downlink_ratio,
                                downlink_mode=downlink_mode,
                            ),
                            "tight": True,
                            "status": "bounded_after_floor_reset",
                            "search": "geometric",
                            "last_pass": last_row_ref(rebound_pass_row),
                            "last_fail": last_row_ref(rebound_fail_row),
                            "last_bad": last_row_ref(rebound_bad_row),
                        }
                    selected = candidate_at_axis(current, axis, bandwidth_low_bps)
                    low_row = reset_row
                return selected, {
                    "axis": axis,
                    "selected": _requirements_candidate_context(
                        selected,
                        downlink_ratio=downlink_ratio,
                        downlink_mode=downlink_mode,
                    ),
                    "tight": False,
                    "status": "floor_still_passes",
                    "search": "geometric",
                    "last_pass": last_row_ref(low_row),
                    "last_fail": None,
                    "last_bad": None,
                }
            pass_candidate = current
            fail_candidate = candidate_at_axis(current, axis, floor_value)
            pass_row: dict[str, Any] | None = None
            fail_row: dict[str, Any] | None = low_row
            bad_row: dict[str, Any] | None = low_row if is_bad_case(low_row) else None
            for iteration in range(iterations):
                mid = math.sqrt(pass_candidate.bandwidth_bps * fail_candidate.bandwidth_bps)
                mid_candidate = candidate_at_axis(current, axis, mid)
                mid_row = probe(mid_candidate, axis=axis, phase=f"{phase_label}:bisect{iteration + 1}")
                if mid_row["passes"]:
                    pass_candidate = mid_candidate
                    pass_row = mid_row
                else:
                    fail_candidate = mid_candidate
                    fail_row = mid_row
                    if is_bad_case(mid_row):
                        bad_row = mid_row
            return pass_candidate, {
                "axis": axis,
                "selected": _requirements_candidate_context(
                    pass_candidate,
                    downlink_ratio=downlink_ratio,
                    downlink_mode=downlink_mode,
                ),
                "tight": True,
                "status": "bounded",
                "search": "geometric",
                "last_pass": last_row_ref(pass_row),
                "last_fail": last_row_ref(fail_row),
                "last_bad": last_row_ref(bad_row),
            }

        ceiling_value = high_value if high_value is not None else axis_ceiling(axis)
        high_candidate = candidate_at_axis(current, axis, ceiling_value)
        high_row = probe(high_candidate, axis=axis, phase=f"{phase_label}:ceiling")
        if high_row["passes"]:
            return high_candidate, {
                "axis": axis,
                "selected": _requirements_candidate_context(
                    high_candidate,
                    downlink_ratio=downlink_ratio,
                    downlink_mode=downlink_mode,
                ),
                "tight": False,
                "status": "ceiling_still_passes",
                "search": "linear",
                "last_pass": last_row_ref(high_row),
                "last_fail": None,
                "last_bad": None,
            }
        pass_candidate = current
        fail_candidate = high_candidate
        pass_row = None
        fail_row = high_row
        bad_row = high_row if is_bad_case(high_row) else None
        for iteration in range(iterations):
            mid = (
                _requirements_axis_value(pass_candidate, axis) + _requirements_axis_value(fail_candidate, axis)
            ) / 2.0
            mid_candidate = candidate_at_axis(current, axis, mid)
            mid_row = probe(mid_candidate, axis=axis, phase=f"{phase_label}:bisect{iteration + 1}")
            if mid_row["passes"]:
                pass_candidate = mid_candidate
                pass_row = mid_row
            else:
                fail_candidate = mid_candidate
                fail_row = mid_row
                if is_bad_case(mid_row):
                    bad_row = mid_row
        return pass_candidate, {
            "axis": axis,
            "selected": _requirements_candidate_context(
                pass_candidate,
                downlink_ratio=downlink_ratio,
                downlink_mode=downlink_mode,
            ),
            "tight": True,
            "status": "bounded",
            "search": "linear",
            "last_pass": last_row_ref(pass_row),
            "last_fail": last_row_ref(fail_row),
            "last_bad": last_row_ref(bad_row),
        }

    guarded_axes: dict[str, Any] = {}

    def apply_axis_guard(axis: str, selected: _RequirementCandidate) -> _RequirementCandidate:
        if axis == "bandwidth" and bandwidth_guard_ratio > 0.0:
            guarded_bandwidth = min(bandwidth_high_bps, selected.bandwidth_bps * (1.0 + bandwidth_guard_ratio))
            guarded = _requirements_with_axis(selected, "bandwidth", guarded_bandwidth)
            guard_ratio = bandwidth_guard_ratio
        elif axis in {"jitter", "jitter_loss"} and jitter_guard_ratio > 0.0:
            guarded_jitter = max(0.0, selected.jitter_ms * (1.0 - jitter_guard_ratio))
            guarded = _requirements_with_axis(selected, "jitter", guarded_jitter)
            guard_ratio = jitter_guard_ratio
        else:
            return selected
        guarded_axes[axis] = {
            "guard_ratio": guard_ratio,
            "selected": _requirements_candidate_context(
                selected,
                downlink_ratio=downlink_ratio,
                downlink_mode=downlink_mode,
            ),
            "guarded": _requirements_candidate_context(
                guarded,
                downlink_ratio=downlink_ratio,
                downlink_mode=downlink_mode,
            ),
        }
        return guarded

    baseline = probe(candidate, axis="baseline", phase="ideal")
    if not baseline["passes"]:
        requirements_path = out_dir / "requirements.jsonl"
        requirements_path.write_text(
            "\n".join(json.dumps(row, sort_keys=True) for row in rows) + "\n",
            encoding="utf-8",
        )
        failure_analysis = {
            "status": "ideal_failed",
            "tight": False,
            "loss_coupling": loss_coupling,
            "effective_loss_coupling": effective_loss_coupling,
            "strict_zero_loss_target": strict_zero_loss_target,
            "requested_axes": requested_axes,
            "searched_axes": selected_axes,
            "skipped_axes": skipped_axes,
            "downlink_mode": downlink_mode,
            "probe_repeats": probe_repeats,
            "probe_min_passes": required_passes,
            "bad_lossy_count": bad_lossy_threshold,
            "bandwidth_probe_repeats": bandwidth_repeats,
            "bandwidth_probe_min_passes": bandwidth_required_passes,
            "bandwidth_bad_lossy_count": bandwidth_bad_lossy_threshold,
            "search_order": search_order,
            "jitter_guard_ratio": jitter_guard_ratio,
            "bandwidth_guard_ratio": bandwidth_guard_ratio,
        }
        _write_benchmark_result(
            out_dir,
            genre="requirements",
            context=result_context,
            configuration={
                "load": load_info,
                "duration_s": duration_s,
                "thresholds": {
                    "max_loss_pct": max_loss_pct,
                    "max_latency_ms": max_latency_ms,
                    "latency_quantile": thresholds.latency_quantile,
                },
                "topic": topic or None,
                "requested_axes": requested_axes,
                "axes": selected_axes,
                "skipped_axes": skipped_axes,
                "bounds": {
                    "bandwidth_low_bps": bandwidth_low_bps,
                    "bandwidth_high_bps": bandwidth_high_bps,
                    "latency_base_ms": latency_base_ms,
                    "latency_high_ms": latency_high_ms,
                    "jitter_high_ms": jitter_high_ms,
                    "loss_high_pct": loss_high_pct,
                },
                "final_refine_iterations": final_refine_iterations,
                "loss_coupling": loss_coupling,
                "effective_loss_coupling": effective_loss_coupling,
                "strict_zero_loss_target": strict_zero_loss_target,
                "downlink_mode": downlink_mode,
                "downlink_ratio": downlink_ratio,
                "probe_repeats": probe_repeats,
                "probe_min_passes": required_passes,
                "bad_lossy_count": bad_lossy_threshold,
                "bandwidth_probe_repeats": bandwidth_repeats,
                "bandwidth_probe_min_passes": bandwidth_required_passes,
                "bandwidth_bad_lossy_count": bandwidth_bad_lossy_threshold,
                "search_order": search_order,
                "netem_seed": netem_seed,
                "jitter_guard_ratio": jitter_guard_ratio,
                "bandwidth_guard_ratio": bandwidth_guard_ratio,
                "generated_profiles_file": generated_profiles_file.name if generated_profiles_file else None,
            },
            result={
                "stream": {"load": load_info, "qos": {"reliability": qos_reliability, "depth": qos_depth}},
                "profile": None,
                "bounds": {},
                "rows": rows,
                "analysis": failure_analysis,
            },
            measurements={"points": measurements},
            verdict={"passed": False, "status": "ideal_failed"},
            artifacts={
                "requirements": requirements_path.name,
                "generated_profiles": generated_profiles_file.name if generated_profiles_file else None,
                "stdout": "stdout.txt",
                "probes_dir": "probes",
            },
        )
        return {"profile": None, "bounds": {}, "rows": rows, "analysis": failure_analysis}

    for round_index in range(1, search_rounds + 1):
        for axis in selected_axes:
            candidate, bounds[axis] = search_axis(
                axis,
                candidate,
                phase_label=f"round{round_index}",
                iterations=search_iterations,
            )
            candidate = apply_axis_guard(axis, candidate)

    if final_refine_iterations:
        for axis in selected_axes:
            previous_bound = bounds.get(axis)
            candidate, bounds[axis] = search_axis(
                axis,
                candidate,
                phase_label="refine",
                iterations=final_refine_iterations,
                low_value=bound_axis_value(axis, previous_bound, "last_fail", bandwidth_low_bps)
                if axis == "bandwidth"
                else None,
                high_value=bound_axis_value(axis, previous_bound, "last_fail", axis_ceiling(axis))
                if axis != "bandwidth"
                else None,
            )
            candidate = apply_axis_guard(axis, candidate)

    final_profile_name = _safe_case_token(f"{profile_prefix}_final")
    generated_doc["profiles"][final_profile_name] = _requirements_profile_spec(
        candidate,
        distribution=distribution,
        downlink_ratio=downlink_ratio,
        downlink_mode=downlink_mode,
        netem_seed=netem_seed,
    )
    write_profiles()
    final_row = probe(candidate, axis="combined", phase="final")
    tight = bool(final_row["passes"]) and all(bounds.get(axis, {}).get("tight") for axis in selected_axes)
    unresolved = [axis for axis in selected_axes if not bounds.get(axis, {}).get("tight")]
    offered_bandwidth_bps = load_info.get("offered_bandwidth_bps")
    bandwidth_overhead_ratio = candidate.bandwidth_bps / float(offered_bandwidth_bps) if offered_bandwidth_bps else None
    final_profile = {
        "name": final_profile_name,
        "candidate": _requirements_candidate_context(
            candidate,
            downlink_ratio=downlink_ratio,
            downlink_mode=downlink_mode,
        ),
        "yaml": generated_doc["profiles"][final_profile_name],
    }
    analysis: dict[str, Any] = {
        "status": "tight" if tight else "not_tight",
        "tight": tight,
        "unresolved_axes": unresolved,
        "final_passes": final_row["passes"],
        "final_target_quality": final_row["target_quality"],
        "bandwidth_overhead_ratio": bandwidth_overhead_ratio,
        "search_rounds": search_rounds,
        "search_iterations": search_iterations,
        "final_refine_iterations": final_refine_iterations,
        "loss_coupling": loss_coupling,
        "effective_loss_coupling": effective_loss_coupling,
        "strict_zero_loss_target": strict_zero_loss_target,
        "requested_axes": requested_axes,
        "searched_axes": selected_axes,
        "skipped_axes": skipped_axes,
        "downlink_mode": downlink_mode,
        "probe_repeats": probe_repeats,
        "probe_min_passes": required_passes,
        "bad_lossy_count": bad_lossy_threshold,
        "bandwidth_probe_repeats": bandwidth_repeats,
        "bandwidth_probe_min_passes": bandwidth_required_passes,
        "bandwidth_bad_lossy_count": bandwidth_bad_lossy_threshold,
        "search_order": search_order,
        "netem_seed": netem_seed,
        "jitter_guard_ratio": jitter_guard_ratio,
        "bandwidth_guard_ratio": bandwidth_guard_ratio,
        "guarded_axes": guarded_axes,
        "bad_cases_observed": {
            axis: bound.get("last_bad")
            for axis, bound in bounds.items()
            if isinstance(bound, dict) and bound.get("last_bad")
        },
    }
    result: dict[str, Any] = {
        "stream": {
            "load": load_info,
            "qos": {"reliability": qos_reliability, "depth": qos_depth},
        },
        "target": {
            "max_loss_pct": max_loss_pct,
            "max_latency_ms": max_latency_ms,
            "latency_quantile": thresholds.latency_quantile,
        },
        "profile": final_profile,
        "bounds": bounds,
        "rows": rows,
        "analysis": analysis,
    }
    if bag_mode:
        result["replay"] = {
            "target": target,
            "bag": bag,
            "required_topics": list(required_topics) if required_topics is not None else None,
            "oracle_min_completeness": oracle_min_completeness,
        }
    requirements_path = out_dir / "requirements.jsonl"
    requirements_path.write_text(
        "\n".join(json.dumps(row, sort_keys=True) for row in rows) + "\n",
        encoding="utf-8",
    )
    _write_benchmark_result(
        out_dir,
        genre="requirements",
        context=result_context,
        configuration={
            "load": load_info,
            "duration_s": duration_s,
            "thresholds": {
                "max_loss_pct": max_loss_pct,
                "max_latency_ms": max_latency_ms,
                "latency_quantile": thresholds.latency_quantile,
            },
            "topic": topic or None,
            "requested_axes": requested_axes,
            "axes": selected_axes,
            "skipped_axes": skipped_axes,
            "bounds": {
                "bandwidth_low_bps": bandwidth_low_bps,
                "bandwidth_high_bps": bandwidth_high_bps,
                "latency_base_ms": latency_base_ms,
                "latency_high_ms": latency_high_ms,
                "jitter_high_ms": jitter_high_ms,
                "loss_high_pct": loss_high_pct,
            },
            "final_refine_iterations": final_refine_iterations,
            "loss_coupling": loss_coupling,
            "effective_loss_coupling": effective_loss_coupling,
            "strict_zero_loss_target": strict_zero_loss_target,
            "downlink_mode": downlink_mode,
            "downlink_ratio": downlink_ratio,
            "probe_repeats": probe_repeats,
            "probe_min_passes": required_passes,
            "bad_lossy_count": bad_lossy_threshold,
            "bandwidth_probe_repeats": bandwidth_repeats,
            "bandwidth_probe_min_passes": bandwidth_required_passes,
            "bandwidth_bad_lossy_count": bandwidth_bad_lossy_threshold,
            "search_order": search_order,
            "netem_seed": netem_seed,
            "jitter_guard_ratio": jitter_guard_ratio,
            "bandwidth_guard_ratio": bandwidth_guard_ratio,
            "generated_profiles_file": generated_profiles_file.name if generated_profiles_file else None,
        },
        result=result,
        measurements={"points": measurements},
        verdict={"passed": bool(final_row["passes"]), "status": str(analysis["status"])},
        artifacts={
            "requirements": requirements_path.name,
            "generated_profiles": generated_profiles_file.name if generated_profiles_file else None,
            "stdout": "stdout.txt",
            "probes_dir": "probes",
        },
    )
    print(f"Requirements rows saved to {requirements_path}")
    print(
        "Required network profile: "
        f"{_format_requirements_profile(candidate, downlink_ratio=downlink_ratio, downlink_mode=downlink_mode)} "
        f"{_format_requirements_target_quality(final_row['target_quality'])} "
        f"({'tight' if tight else 'not tight within configured bounds'})"
    )
    bad_refs: list[str] = []
    for axis, bound in bounds.items():
        ref = bound.get("last_bad") if isinstance(bound, dict) else None
        candidate_ref = ref.get("candidate") if isinstance(ref, dict) else None
        if not isinstance(candidate_ref, dict):
            continue
        if axis == "bandwidth":
            value = _format_bps(candidate_ref.get("bandwidth_bps"))
        elif axis == "latency":
            value = f"{candidate_ref.get('uplink_latency_ms')}ms"
        else:
            value = f"{candidate_ref.get('jitter_ms')}ms"
        bad_refs.append(f"{axis}={value}")
    if bad_refs:
        print("Observed bad-case candidates: " + ", ".join(str(ref) for ref in bad_refs))
    return result


# --------------------------------------------------------------------------- #
# Benchmark driver: loss boundaries
# --------------------------------------------------------------------------- #


def _parse_loss_boundary_axes(raw: str) -> list[str]:
    available = ("bandwidth", "jitter")
    aliases = {"bw": "bandwidth", "rate": "bandwidth"}
    axes = [aliases.get(value.strip().lower(), value.strip().lower()) for value in raw.split(",") if value.strip()]
    if not axes or "all" in axes:
        return list(available)
    unknown = sorted(set(axes) - set(available))
    if unknown:
        raise ValueError(f"Unknown loss-boundary axes: {', '.join(unknown)}")
    return [axis for axis in available if axis in axes]


def _parse_netem_seed_values(raw: str | None) -> list[int]:
    if raw is None or not raw.strip():
        return []
    from .network_profiles import parse_seed

    seeds: list[int] = []
    for token in raw.split(","):
        value = token.strip()
        if not value:
            continue
        seeds.append(parse_seed(value, "netem-seeds"))
    return seeds


def _loss_boundary_step_units(low: float, high: float, step: float, *, label: str) -> tuple[int, int]:
    if step <= 0.0:
        raise ValueError(f"{label}-step must be > 0.")
    if low < 0.0 or high < 0.0:
        raise ValueError(f"{label} bounds must be >= 0.")
    if low >= high:
        raise ValueError(f"{label}-low must be smaller than {label}-high.")
    low_units = int(math.floor(low / step + 1e-9))
    high_units = int(math.ceil(high / step - 1e-9))
    if low_units >= high_units:
        raise ValueError(f"{label} bounds collapse at the configured step.")
    return low_units, high_units


def drive_loss_boundaries(
    run_point: RunPointFn,
    *,
    max_latency_ms: float,
    rate_hz: float,
    size: int,
    streams: int,
    qos_reliability: str,
    qos_depth: int,
    topic: str,
    out_dir: Path,
    axes: Sequence[str],
    bandwidth_low_bps: float,
    bandwidth_high_bps: float,
    bandwidth_step_bps: float,
    latency_base_ms: float,
    jitter_low_ms: float,
    jitter_high_ms: float,
    jitter_step_ms: float,
    min_duration_s: float,
    min_messages: int,
    distribution: str,
    downlink_ratio: float = 1.0,
    downlink_mode: str = "lan",
    probe_repeats: int = 10,
    good_clean_count: int | None = None,
    bad_lossy_count: int | None = None,
    netem_seed: int | None = None,
    netem_seeds: Sequence[int] = (),
    result_context: dict[str, Any] | None = None,
    generated_profiles_file: Path | None = None,
    profile_prefix: str = "loss_boundary",
    ground_truth: dict[str, dict[str, Any]] | None = None,
    oracle_min_completeness: float = 0.95,
    required_topics: Sequence[str] | None = None,
    target: dict[str, Any] | None = None,
    bag: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Find discrete zero-loss good/bad boundaries for bandwidth and jitter."""

    from .benchmark import OracleThresholds

    bag_mode = target is not None or ground_truth is not None or required_topics is not None
    # The boundary oracle is zero network loss plus the latency ceiling; in bag
    # mode the completeness floor rides along inside ``loss_free`` (an incomplete
    # replay is a lossy run).
    oracle_thresholds = OracleThresholds(max_loss_pct=0.0, max_latency_ms=max_latency_ms)
    if downlink_mode not in {"mirror", "lan"}:
        raise ValueError("downlink_mode must be 'mirror' or 'lan'.")
    if latency_base_ms < 0.0:
        raise ValueError("latency-base-ms must be >= 0.")
    if probe_repeats < 1:
        raise ValueError("probe-repeats must be >= 1.")
    if netem_seed is not None and netem_seeds:
        raise ValueError("Use either netem_seed or netem_seeds, not both.")

    selected_axes = _parse_loss_boundary_axes(",".join(axes))
    bandwidth_low_units, bandwidth_high_units = _loss_boundary_step_units(
        bandwidth_low_bps,
        bandwidth_high_bps,
        bandwidth_step_bps,
        label="bandwidth",
    )
    jitter_low_units, jitter_high_units = _loss_boundary_step_units(
        jitter_low_ms,
        jitter_high_ms,
        jitter_step_ms,
        label="jitter",
    )

    if netem_seeds:
        sample_seeds: tuple[int | None, ...] = tuple(seed for seed in netem_seeds for _ in range(probe_repeats))
        seed_policy = "seed_set"
    elif netem_seed is not None:
        sample_seeds = tuple(netem_seed for _ in range(probe_repeats))
        seed_policy = "fixed_seed"
    else:
        sample_seeds = tuple(None for _ in range(probe_repeats))
        seed_policy = "seedless"
    sample_count = len(sample_seeds)
    required_good = good_clean_count if good_clean_count is not None else sample_count
    required_bad = bad_lossy_count if bad_lossy_count is not None else sample_count
    if not 1 <= required_good <= sample_count:
        raise ValueError("good-clean-count must be within [1, total samples].")
    if not 1 <= required_bad <= sample_count:
        raise ValueError("bad-lossy-count must be within [1, total samples].")

    load = {"size_a": size, "rate": rate_hz, "streams": streams}
    load_info = _load_context(load)
    duration_s = _duration_for_min_messages(rate_hz, min_duration_s=min_duration_s, min_messages=min_messages)
    rows: list[dict[str, Any]] = []
    measurements: list[dict[str, Any]] = []
    boundaries: dict[str, Any] = {}
    row_cache: dict[tuple[str, int, str], dict[str, Any]] = {}
    generated_doc: dict[str, Any] = {
        "profiles": {},
        "metadata": {
            "kind": "benchmark-loss-boundaries-generated",
            "stream": {
                "rate_hz": rate_hz,
                "size_bytes": size,
                "streams": streams,
                "qos": {"reliability": qos_reliability, "depth": qos_depth},
                "offered_bandwidth_bps": load_info.get("offered_bandwidth_bps"),
            },
            "search": {
                "axes": selected_axes,
                "bandwidth_low_bps": bandwidth_low_bps,
                "bandwidth_high_bps": bandwidth_high_bps,
                "bandwidth_step_bps": bandwidth_step_bps,
                "latency_base_ms": latency_base_ms,
                "jitter_low_ms": jitter_low_ms,
                "jitter_high_ms": jitter_high_ms,
                "jitter_step_ms": jitter_step_ms,
                "distribution": distribution,
                "downlink_ratio": downlink_ratio,
                "downlink_mode": downlink_mode,
                "probe_repeats": probe_repeats,
                "sample_count": sample_count,
                "good_clean_count": required_good,
                "bad_lossy_count": required_bad,
                "seed_policy": seed_policy,
                "netem_seed": netem_seed,
                "netem_seeds": list(netem_seeds),
            },
            "thresholds": {
                "max_ros2_loss_pct": 0.0,
                "max_latency_ms": max_latency_ms,
            },
        },
    }
    profile_counter = 0

    def write_profiles() -> None:
        if generated_profiles_file is not None:
            generated_profiles_file.write_text(yaml.safe_dump(generated_doc, sort_keys=False), encoding="utf-8")

    def axis_value(axis: str, unit: int) -> float:
        if axis == "bandwidth":
            return unit * bandwidth_step_bps
        if axis == "jitter":
            return unit * jitter_step_ms
        raise ValueError(f"Unknown axis {axis!r}.")

    def candidate_for_axis(axis: str, unit: int) -> _RequirementCandidate:
        if axis == "bandwidth":
            return _RequirementCandidate(
                bandwidth_bps=axis_value(axis, unit),
                latency_ms=latency_base_ms,
                jitter_ms=0.0,
                loss_pct=0.0,
            )
        if axis == "jitter":
            return _RequirementCandidate(
                bandwidth_bps=bandwidth_high_bps,
                latency_ms=latency_base_ms,
                jitter_ms=axis_value(axis, unit),
                loss_pct=0.0,
            )
        raise ValueError(f"Unknown axis {axis!r}.")

    def candidate_for_combined() -> _RequirementCandidate:
        bandwidth_candidate = boundaries.get("bandwidth", {}).get("good_boundary", {}).get("candidate", {})
        jitter_candidate = boundaries.get("jitter", {}).get("good_boundary", {}).get("candidate", {})
        bandwidth = float(bandwidth_candidate.get("bandwidth_bps") or bandwidth_high_bps)
        jitter = float(jitter_candidate.get("jitter_ms") or 0.0)
        return _RequirementCandidate(
            bandwidth_bps=bandwidth,
            latency_ms=latency_base_ms,
            jitter_ms=jitter,
            loss_pct=0.0,
        )

    def row_ref(row: dict[str, Any] | None) -> dict[str, Any] | None:
        if row is None:
            return None
        return {
            "candidate": row.get("candidate"),
            "classification": row.get("classification"),
            "loss_pct": row.get("loss_pct"),
            "latency_p95_ms": row.get("latency_p95_ms"),
            "repeat": row.get("repeat"),
        }

    def classify(
        candidate: _RequirementCandidate,
        *,
        axis: str,
        unit: int,
        mode: str,
        phase: str,
        use_cache: bool = True,
    ) -> dict[str, Any]:
        nonlocal profile_counter
        cache_key = (axis, unit, mode)
        if use_cache and cache_key in row_cache:
            return row_cache[cache_key]

        samples: list[dict[str, Any]] = []
        sample_measurements: list[dict[str, Any]] = []
        selected_topic = ""
        for sample_index, sample_seed in enumerate(sample_seeds, start=1):
            profile_counter += 1
            seed_suffix = "seedless" if sample_seed is None else f"seed{sample_seed}"
            profile_name = _safe_case_token(
                f"{profile_prefix}_{profile_counter:03d}_{axis}_{phase}_{mode}_sample{sample_index}_{seed_suffix}"
            )
            generated_doc["profiles"][profile_name] = _requirements_profile_spec(
                candidate,
                distribution=distribution,
                downlink_ratio=downlink_ratio,
                downlink_mode=downlink_mode,
                netem_seed=sample_seed,
            )
            write_profiles()
            summary = run_point(profile=profile_name, load=load, duration_s=duration_s, out_dir=out_dir)
            verdict = _probe_sample_verdict(
                summary,
                thresholds=oracle_thresholds,
                topic=topic,
                bag_mode=bag_mode,
                ground_truth=ground_truth,
                min_completeness=oracle_min_completeness,
                required_topics=required_topics,
            )
            rows_for_print = _topic_rows(summary, "" if bag_mode else topic)
            if verdict["topic"] and not selected_topic:
                selected_topic = verdict["topic"]
            loss_free = verdict["loss_free"]
            latency_ok = verdict["latency_ok"]
            good = loss_free and latency_ok
            lossy = not loss_free
            sample = {
                "sample": sample_index,
                "seed": sample_seed,
                "topic": verdict["topic"],
                "expected": verdict["expected"],
                "delivered": verdict["delivered"],
                "lost": verdict["lost"],
                "reordered": verdict["reordered"],
                "loss_pct": verdict["loss_pct"],
                "latency_p95_ms": verdict["latency_p95_ms"],
                "jitter_p95_ms": verdict["jitter_p95_ms"],
                "loss_free": loss_free,
                "latency_ok": latency_ok,
                "good": good,
                "lossy": lossy,
            }
            if verdict["failing_topics"] is not None:
                sample["failing_topics"] = verdict["failing_topics"]
                sample["per_topic"] = verdict["per_topic"]
            samples.append(sample)
            sample_measurements.append(
                {
                    "sample": sample_index,
                    "seed": sample_seed,
                    "topics": rows_for_print,
                    "summary": summary,
                    "sample_result": sample,
                }
            )

            good_count = sum(1 for item in samples if item["good"])
            lossy_count = sum(1 for item in samples if item["lossy"])
            remaining = sample_count - sample_index
            if mode == "good" and good_count + remaining < required_good:
                break
            if mode == "bad" and lossy_count + remaining < required_bad:
                break

        good_count = sum(1 for sample in samples if sample["good"])
        loss_free_count = sum(1 for sample in samples if sample["loss_free"])
        lossy_count = sum(1 for sample in samples if sample["lossy"])
        expected_total = sum(int(sample["expected"]) for sample in samples if sample.get("expected") is not None)
        delivered_total = sum(int(sample["delivered"]) for sample in samples if sample.get("delivered") is not None)
        lost_total = sum(int(sample["lost"]) for sample in samples if sample.get("lost") is not None)
        reordered_total = sum(int(sample["reordered"]) for sample in samples if sample.get("reordered") is not None)
        latency_values = [
            float(sample["latency_p95_ms"]) for sample in samples if sample.get("latency_p95_ms") is not None
        ]
        jitter_values = [
            float(sample["jitter_p95_ms"]) for sample in samples if sample.get("jitter_p95_ms") is not None
        ]
        loss_pct = lost_total / expected_total * 100.0 if expected_total > 0 else 100.0
        good_case = good_count >= required_good
        bad_case = lossy_count >= required_bad
        if good_case:
            classification = "good"
        elif bad_case:
            classification = "bad"
        elif loss_free_count > 0 and lossy_count > 0:
            classification = "mixed"
        elif lossy_count > 0:
            classification = "lossy"
        else:
            classification = "quality_fail"
        repeat_summary = {
            "sample_count": len(samples),
            "configured_samples": sample_count,
            "probe_repeats": probe_repeats,
            "good_clean_count": required_good,
            "bad_lossy_count": required_bad,
            "good_count": good_count,
            "loss_free_count": loss_free_count,
            "lossy_count": lossy_count,
            "good_case": good_case,
            "bad_case": bad_case,
            "seed_policy": seed_policy,
        }
        row = {
            "axis": axis,
            "phase": phase,
            "mode": mode,
            "candidate": _requirements_candidate_context(
                candidate,
                downlink_ratio=downlink_ratio,
                downlink_mode=downlink_mode,
            ),
            "classification": classification,
            "good_case": good_case,
            "bad_case": bad_case,
            "topic": selected_topic,
            "expected": expected_total,
            "delivered": delivered_total,
            "lost": lost_total,
            "reordered": reordered_total,
            "loss_pct": loss_pct,
            "latency_p95_ms": max(latency_values) if latency_values else None,
            "jitter_p95_ms": max(jitter_values) if jitter_values else None,
            "load": load_info,
            "duration_s": duration_s,
            "qos": {"reliability": qos_reliability, "depth": qos_depth},
            "repeat": repeat_summary,
            "samples": samples,
        }
        if bag_mode:
            row["failing_topics"] = sorted({t for s in samples for t in (s.get("failing_topics") or [])})
        rows.append(row)
        measurements.append({"row": row, "samples": sample_measurements})
        row_cache[cache_key] = row
        print(
            f"  loss-boundary({axis}[{phase}:{mode}] "
            f"{_format_requirements_profile(candidate, downlink_ratio=downlink_ratio, downlink_mode=downlink_mode)}): "
            f"{classification.upper()} clean={good_count}/{sample_count} lossy={lossy_count}/{sample_count} "
            f"loss={round(loss_pct, 3)}% p95={row['latency_p95_ms']}ms"
        )
        return row

    def classify_axis_unit(axis: str, unit: int, *, mode: str, phase: str) -> dict[str, Any]:
        return classify(candidate_for_axis(axis, unit), axis=axis, unit=unit, mode=mode, phase=phase)

    def search_bandwidth() -> dict[str, Any]:
        high_row = classify_axis_unit("bandwidth", bandwidth_high_units, mode="good", phase="ceiling")
        low_row = classify_axis_unit("bandwidth", bandwidth_low_units, mode="good", phase="floor")
        if not high_row["good_case"]:
            return {
                "status": "ceiling_not_good",
                "tight": False,
                "resolution_bps": bandwidth_step_bps,
                "ceiling": row_ref(high_row),
                "floor": row_ref(low_row),
            }
        if low_row["good_case"]:
            return {
                "status": "floor_still_good",
                "tight": False,
                "resolution_bps": bandwidth_step_bps,
                "good_boundary": row_ref(low_row),
                "bad_boundary": None,
                "not_good_neighbor": None,
            }

        fail_unit = bandwidth_low_units
        pass_unit = bandwidth_high_units
        while pass_unit - fail_unit > 1:
            mid_unit = (pass_unit + fail_unit) // 2
            mid_row = classify_axis_unit("bandwidth", mid_unit, mode="good", phase="bisect")
            if mid_row["good_case"]:
                pass_unit = mid_unit
            else:
                fail_unit = mid_unit

        good_row = classify_axis_unit("bandwidth", pass_unit, mode="good", phase="confirm-good")
        not_good_row = classify_axis_unit("bandwidth", fail_unit, mode="good", phase="confirm-not-good")
        bad_row = classify_axis_unit("bandwidth", fail_unit, mode="bad", phase="confirm-bad")
        bad_unit = fail_unit if bad_row["bad_case"] else None
        scan_unit = fail_unit - 1
        while bad_unit is None and scan_unit >= bandwidth_low_units:
            scan_row = classify_axis_unit("bandwidth", scan_unit, mode="bad", phase="bad-scan")
            if scan_row["bad_case"]:
                bad_unit = scan_unit
                bad_row = scan_row
                break
            scan_unit -= 1
        return {
            "status": "bounded" if bad_unit is not None else "bounded_without_bad_neighbor",
            "tight": bad_unit == pass_unit - 1,
            "resolution_bps": bandwidth_step_bps,
            "good_boundary": row_ref(good_row),
            "not_good_neighbor": row_ref(not_good_row),
            "bad_boundary": row_ref(bad_row) if bad_unit is not None else None,
        }

    def search_jitter() -> dict[str, Any]:
        low_good_row = classify_axis_unit("jitter", jitter_low_units, mode="good", phase="floor-good")
        high_good_row = classify_axis_unit("jitter", jitter_high_units, mode="good", phase="ceiling-good")
        high_bad_row = classify_axis_unit("jitter", jitter_high_units, mode="bad", phase="ceiling-bad")
        if not low_good_row["good_case"]:
            return {
                "status": "floor_not_good",
                "tight": False,
                "resolution_ms": jitter_step_ms,
                "floor": row_ref(low_good_row),
                "ceiling": row_ref(high_good_row),
            }
        if high_good_row["good_case"]:
            return {
                "status": "ceiling_still_good",
                "tight": False,
                "resolution_ms": jitter_step_ms,
                "good_boundary": row_ref(high_good_row),
                "first_not_good": None,
                "bad_boundary": None,
            }

        good_unit = jitter_low_units
        not_good_unit = jitter_high_units
        while not_good_unit - good_unit > 1:
            mid_unit = (not_good_unit + good_unit) // 2
            mid_row = classify_axis_unit("jitter", mid_unit, mode="good", phase="good-bisect")
            if mid_row["good_case"]:
                good_unit = mid_unit
            else:
                not_good_unit = mid_unit
        good_row = classify_axis_unit("jitter", good_unit, mode="good", phase="confirm-good")
        first_not_good_row = classify_axis_unit("jitter", not_good_unit, mode="good", phase="confirm-not-good")

        bad_boundary_row: dict[str, Any] | None = None
        last_not_bad_row: dict[str, Any] | None = None
        if high_bad_row["bad_case"]:
            not_bad_unit = good_unit
            bad_unit = jitter_high_units
            while bad_unit - not_bad_unit > 1:
                mid_unit = (bad_unit + not_bad_unit) // 2
                mid_row = classify_axis_unit("jitter", mid_unit, mode="bad", phase="bad-bisect")
                if mid_row["bad_case"]:
                    bad_unit = mid_unit
                else:
                    not_bad_unit = mid_unit
            bad_boundary_row = classify_axis_unit("jitter", bad_unit, mode="bad", phase="confirm-bad")
            last_not_bad_row = classify_axis_unit("jitter", not_bad_unit, mode="bad", phase="confirm-not-bad")

        mixed_low = axis_value("jitter", good_unit + 1) if good_unit + 1 < jitter_high_units else None
        bad_candidate = bad_boundary_row.get("candidate") if bad_boundary_row else None
        bad_jitter = bad_candidate.get("jitter_ms") if isinstance(bad_candidate, dict) else None
        mixed_high = float(bad_jitter) - jitter_step_ms if bad_jitter is not None else None
        if mixed_low is not None and mixed_high is not None and mixed_low > mixed_high:
            mixed_low = None
            mixed_high = None
        return {
            "status": "bounded" if bad_boundary_row is not None else "good_bounded_bad_unbounded",
            "tight": bad_boundary_row is not None,
            "resolution_ms": jitter_step_ms,
            "good_boundary": row_ref(good_row),
            "first_not_good": row_ref(first_not_good_row),
            "bad_boundary": row_ref(bad_boundary_row),
            "last_not_bad": row_ref(last_not_bad_row),
            "mixed_zone": {"low_ms": mixed_low, "high_ms": mixed_high},
        }

    for axis in selected_axes:
        if axis == "bandwidth":
            boundaries["bandwidth"] = search_bandwidth()
        elif axis == "jitter":
            boundaries["jitter"] = search_jitter()

    combined_row: dict[str, Any] | None = None
    if set(selected_axes) == {"bandwidth", "jitter"}:
        combined = candidate_for_combined()
        combined_row = classify(
            combined,
            axis="combined",
            unit=0,
            mode="good",
            phase="combined-good",
            use_cache=False,
        )

    result = {
        "stream": {
            "load": load_info,
            "qos": {"reliability": qos_reliability, "depth": qos_depth},
        },
        "target": {
            "max_ros2_loss_pct": 0.0,
            "max_latency_ms": max_latency_ms,
        },
        "search": generated_doc["metadata"]["search"],
        "boundaries": boundaries,
        "combined_good": row_ref(combined_row),
        "rows": rows,
        "analysis": {
            "status": (
                "bounded" if all(boundaries.get(axis, {}).get("tight") for axis in selected_axes) else "not_tight"
            ),
            "seed_policy": seed_policy,
            "sample_count": sample_count,
            "interpretation": {
                "seedless": "best empirical estimate for this host/run; repeat the command to check drift.",
                "fixed_seed": "replays one netem random sequence; useful for debugging, but can overfit.",
                "seed_set": "replays several netem random sequences; best reproducible compromise.",
            }[seed_policy],
        },
    }
    if bag_mode:
        result["replay"] = {
            "target": target,
            "bag": bag,
            "required_topics": list(required_topics) if required_topics is not None else None,
            "oracle_min_completeness": oracle_min_completeness,
        }
    boundaries_path = out_dir / "loss-boundaries.json"
    boundaries_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _write_benchmark_result(
        out_dir,
        genre="loss-boundaries",
        context=result_context,
        configuration={
            "load": load_info,
            "duration_s": duration_s,
            "thresholds": result["target"],
            "topic": topic or None,
            "search": generated_doc["metadata"]["search"],
            "generated_profiles_file": generated_profiles_file.name if generated_profiles_file else None,
        },
        result=result,
        measurements={"points": measurements},
        verdict={
            "passed": result["analysis"]["status"] == "bounded",
            "status": str(result["analysis"]["status"]),
        },
        artifacts={
            "loss_boundaries": boundaries_path.name,
            "generated_profiles": generated_profiles_file.name if generated_profiles_file else None,
            "stdout": "stdout.txt",
            "probes_dir": "probes",
        },
    )
    write_profiles()
    print(f"Loss boundaries saved to {boundaries_path}")
    for axis, boundary in boundaries.items():
        good = boundary.get("good_boundary", {}).get("candidate") if isinstance(boundary, dict) else None
        bad = boundary.get("bad_boundary", {}).get("candidate") if isinstance(boundary, dict) else None
        if axis == "bandwidth" and isinstance(good, dict):
            good_label = _format_bps(good.get("bandwidth_bps"))
            bad_label = _format_bps(bad.get("bandwidth_bps")) if isinstance(bad, dict) else "unbounded"
            print(f"Bandwidth boundary: good={good_label}, bad={bad_label}")
        elif axis == "jitter" and isinstance(good, dict):
            good_label = f"{good.get('jitter_ms')}ms"
            bad_label = f"{bad.get('jitter_ms')}ms" if isinstance(bad, dict) else "unbounded"
            print(f"Jitter boundary: good={good_label}, bad={bad_label}")
    return result


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #


def _current_sha() -> str:
    """Best-effort current git SHA for result/band provenance."""
    import subprocess

    try:
        result = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
            timeout=5,
        )
        return result.stdout.strip()
    except Exception:
        return "unknown"


def _runner_context() -> dict[str, Any]:
    """Runner identity embedded in every result so bands can refuse cross-class runs."""
    import os
    import platform
    import socket

    from .benchmark import runner_fingerprint

    return {
        "fingerprint": runner_fingerprint(),
        "os": platform.system().lower(),
        "machine": platform.machine().lower(),
        "hostname": socket.gethostname(),
        "cpu_count": os.cpu_count(),
    }


def _setup_benchmark_run_dir(args: argparse.Namespace, genre: str, profile: str | None) -> Path:
    """Setup a unique run directory under benchmarks_dir and write command/metadata/configs."""
    from datetime import datetime

    from . import __version__
    from .cli import _load_runtime_config, _resolve_project_config_source

    runtime = _load_runtime_config(args)
    artifacts_dir = Path(args.artifacts_dir) if getattr(args, "artifacts_dir", None) else runtime.benchmarks_dir
    if not artifacts_dir:
        artifacts_dir = Path.cwd() / "artifacts" / "benchmarks"

    now_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    profile_part = f"_{profile}" if profile else ""
    # Absolute: the run dir becomes a session-configs source, and relative
    # entries there would later be re-resolved against the project config's
    # directory instead of the invocation cwd.
    run_dir = (Path.cwd() / artifacts_dir / f"{genre}{profile_part}_{now_str}").resolve()
    run_dir.mkdir(parents=True, exist_ok=True)

    # 1. Write command.txt
    command_line = " ".join(shlex.quote(arg) for arg in sys.argv)
    (run_dir / "command.txt").write_text(command_line + "\n", encoding="utf-8")

    # 2. Write metadata.json
    safe_args = {}
    for k, v in vars(args).items():
        if k.startswith("_"):
            continue
        try:
            json.dumps(v)
            safe_args[k] = v
        except Exception:
            safe_args[k] = str(v)

    metadata = {
        "timestamp": datetime.now().isoformat(),
        "rosotacom_version": __version__,
        "genre": genre,
        "profile": profile,
        "args": safe_args,
        "rosotacom_sha": _current_sha(),
    }
    (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    # 3. Copy config files if present
    # rosotacom.yaml
    rosotacom_config_raw, _ = _resolve_project_config_source(args)
    if rosotacom_config_raw:
        config_path = Path(rosotacom_config_raw).resolve()
        if config_path.is_file():
            shutil.copy2(config_path, run_dir / "rosotacom.yaml")

    # profiles file
    profiles_file = _benchmark_profiles_file(profile, runtime.profiles_file)
    if profiles_file:
        profiles_path = Path(profiles_file).resolve()
        if profiles_path.is_file():
            shutil.copy2(profiles_path, run_dir / "profiles.yaml")

    # ros2docker.json
    if runtime.ros2docker_config:
        ros2docker_path = Path(runtime.ros2docker_config).resolve()
        if ros2docker_path.is_file():
            shutil.copy2(ros2docker_path, run_dir / "ros2docker.json")

    return run_dir


def _find_profiles_file(profiles_file: Path | None = None) -> Path:
    """Locate the profiles file from the project config."""
    if profiles_file is not None and profiles_file.is_file():
        return profiles_file
    # Walk up from the package to find profiles.yaml or rosotacom.yaml.
    for candidate in (
        Path.cwd() / "profiles.yaml",
        Path.cwd() / "profiles" / "benchmark-profiles.yaml",
        Path(__file__).parent / "resources" / "examples" / "profiles.yaml",
    ):
        if candidate.is_file():
            return candidate
    raise FileNotFoundError("No profiles file found. Provide one via --profiles-file or 'profiles:' in rosotacom.yaml.")


def _parse_values(raw: str) -> list[float]:
    """Parse a comma-separated or range-based value list.

    Supports: ``"1000,2000,4000"`` or ``"1000:10000:1000"`` (start:stop:step)
    or ``"1000..10000/10"`` (start..stop/count, linearly spaced).
    """
    raw = raw.strip()
    if ".." in raw and "/" in raw:
        range_part, count_str = raw.split("/", 1)
        start_str, end_str = range_part.split("..", 1)
        start, end, count = float(start_str), float(end_str), int(count_str)
        if count <= 1:
            return [start]
        step = (end - start) / (count - 1)
        return [start + i * step for i in range(count)]
    if ":" in raw:
        parts = raw.split(":")
        start, stop = float(parts[0]), float(parts[1])
        step = float(parts[2]) if len(parts) > 2 else 1.0
        values: list[float] = []
        v = start
        while v <= stop + 1e-9:
            values.append(v)
            v += step
        return values
    return [float(v.strip()) for v in raw.split(",") if v.strip()]


# --------------------------------------------------------------------------- #
# CLI subcommand handlers
# --------------------------------------------------------------------------- #


def benchmark_delta(args: argparse.Namespace) -> int:
    """`rosotacom benchmark delta` — the cross-environment comparison.

    Exits 0 whenever the comparison itself succeeded, including when rows are
    unmatched or every metric moved. This is monitor-only by construction
    (`benchmark_delta.compare` puts `monitor_only: true` first in the report):
    the number is the deliverable, and an exit code would turn "explain this"
    into "re-run until it is quiet".

    It exits non-zero for the one thing that is a failure — no rows in common,
    which means the two sides were not the same rows and the report would be
    an empty table with a confident heading.
    """
    from . import benchmark_delta as delta_module

    reference = delta_module.collect([Path(args.reference)])
    measured = delta_module.collect([Path(args.measured)])
    report = delta_module.compare(
        reference,
        measured,
        label_reference=args.label_reference,
        label_measured=args.label_measured,
    )

    if args.out:
        Path(args.out).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    markdown = delta_module.render_markdown(report)
    if args.markdown:
        Path(args.markdown).write_text(markdown, encoding="utf-8")
    else:
        print(markdown)

    counts = report["counts"]
    print(
        f"BENCHMARK DELTA: {counts['matched']} row(s) matched, "
        f"{counts[f'only_{args.label_reference}']} only in {args.label_reference}, "
        f"{counts[f'only_{args.label_measured}']} only in {args.label_measured} "
        "(monitor-only)"
    )
    if not report["rows"]:
        print(
            "rosotacom: error: the two sets share no rows, so there is nothing to "
            "compare. Check that both sides ran the same registry rows under the "
            "same profile and RMW.",
            file=sys.stderr,
        )
        return 1
    return 0


def _add_benchmark_common_args(parser: argparse.ArgumentParser, *, ota_benchmark: bool = False) -> None:
    from .cli import (
        OTA_DEFAULT_SUDO_MODE,
        OTA_SUDO_MODES,
        _add_common_config_args,
        _add_ota_install_args,
        _add_peer_address_arg,
        _add_peer_arg,
        _add_peer_ssh_arg,
    )

    _add_common_config_args(parser)
    _add_peer_arg(parser)
    _add_peer_ssh_arg(parser)
    _add_peer_address_arg(parser)
    if ota_benchmark:
        # Same four options as `ota-smoke`: a measurement run has to be able to
        # reach a peer the same ways a smoke run can, including a transport that
        # is narrower than a shell.
        _add_ota_install_args(parser)
    parser.add_argument("--skip-preflight", action="store_true", help="Skip SSH/Docker readiness checks.")
    parser.add_argument("--keep-workdir", action="store_true", help="Keep temporary checkout directories.")
    parser.add_argument("--dry-run", action="store_true", help="Show commands but do not run them.")
    parser.add_argument("--artifacts-dir", help="Output directory for all benchmark artifacts.")
    parser.add_argument(
        "--drain-s",
        type=float,
        default=DEFAULT_BENCHMARK_DRAIN_S,
        help=(
            "Seconds to keep benchmark peers and shaping alive after stopping synthetic publishers, "
            f"so delayed in-flight messages can arrive before teardown (default: {DEFAULT_BENCHMARK_DRAIN_S:g})."
        ),
    )
    parser.add_argument(
        "--target",
        help="OTA target session or scenario (default for OTA: the benchmark genre session).",
    )
    parser.add_argument(
        "--target-type",
        choices=["auto", "session", "scenario"],
        help=(
            "How to resolve --target for OTA benchmark runs "
            "(default: session when --target is omitted, auto otherwise)."
        ),
    )
    parser.add_argument("--workdir", help="Remote OTA workdir (default: /tmp/rosotacom_ota).")
    parser.add_argument(
        "--reuse",
        action="store_true",
        help="Reuse already staged OTA source/project on remote hosts.",
    )
    parser.add_argument(
        "--sudo-mode",
        choices=OTA_SUDO_MODES,
        default=OTA_DEFAULT_SUDO_MODE,
        help=(
            "How OTA benchmark profile shaping obtains tc/ip privileges "
            "(passwordless: require sudo -n; askpass: prompt locally per peer and feed sudo -S; "
            "container: no sudo -- run each tc/ip in a short-lived NET_ADMIN host-netns container, "
            "for peers with Docker but no passwordless sudo)."
        ),
    )
    parser.add_argument(
        "--shaping-image",
        help=(
            "Image for --sudo-mode container (must contain tc/ip). Default: resolve on the peer -- "
            "the image of a running rosotacom_ container, else the newest local ros-communication* image."
        ),
    )
    parser.add_argument("--interactive", action="store_true", help="Open a tmux operator view for this benchmark run.")
    parser.add_argument(
        "--no-attach",
        action="store_true",
        help="Create the interactive tmux session without attaching.",
    )
    parser.add_argument(
        "--skip-conflict-check",
        action="store_true",
        help="Run even though another rosotacom run is active (metrics may be distorted).",
    )
    parser.add_argument(
        "--rmw",
        default=DEFAULT_BENCHMARK_RMW,
        help=f"RMW implementation or rosotacom RMW alias for benchmark sessions (default: {DEFAULT_BENCHMARK_RMW}).",
    )
    parser.add_argument(
        "--cyclone-spdp-interval",
        help=(
            "Override CycloneDDS SPDPInterval for benchmark session OTA XML, e.g. 150s. "
            f"Default remains {DEFAULT_CYCLONE_SPDP_INTERVAL}; use longer values only for quiet-discovery "
            "payload characterization, not for normal end-to-end DDS behavior."
        ),
    )
    parser.add_argument(
        "--qos-reliability",
        choices=["best_effort", "reliable"],
        help="Override benchmark session OTA pub/sub reliability for the whole run.",
    )
    parser.add_argument(
        "--qos-depth",
        type=int,
        help="Override benchmark session QoS depth for the whole run.",
    )
    parser.set_defaults(ota_benchmark=ota_benchmark)


def _add_benchmark_replay_args(parser: argparse.ArgumentParser) -> None:
    """Bag-as-load options: a replay ``--target`` (session/scenario, via the shared
    ``--target``/``--target-type``) plus its per-topic completeness ground truth."""
    parser.add_argument(
        "--bag",
        help=(
            "rosbag2 metadata.yaml (or bag dir) giving per-topic message-count ground "
            "truth for the replay --target. Adds the completeness gate (delivered / "
            "bag-count) on top of the loss/latency oracle."
        ),
    )
    parser.add_argument(
        "--oracle-min-completeness",
        type=float,
        default=0.95,
        help="Bag-as-load: min delivered/bag-count per topic to pass (default: 0.95).",
    )


def _abort_on_local_benchmark_conflicts(args: argparse.Namespace) -> None:
    """Local benchmarks need a quiet host: concurrent runs distort latency/loss metrics."""
    if getattr(args, "dry_run", False) or getattr(args, "skip_conflict_check", False):
        return
    from .cli import _conflict_error, _list_docker_containers

    active = sorted(name for name, _networks in _list_docker_containers() if name.startswith("rosotacom_"))
    if active:
        raise _conflict_error(
            "A rosotacom run is already active on this host; local benchmarks need exclusive resources"
            " because concurrent load injects scheduling jitter into latency/loss metrics.",
            active,
            "Wait for it to finish or stop it first (`rosotacom ps` lists this workspace's runs;"
            " `rosotacom stop <session>` / `rosotacom smoke <target> --stop` clean up),"
            " or pass --skip-conflict-check.",
        )


def _make_live_run_point(args: argparse.Namespace, session_name: str) -> RunPointFn:
    """Create a live run_point probe from the parsed CLI context."""
    import shlex
    import subprocess
    from dataclasses import replace

    from .cli import (
        _PROFILE_SAFETY_MAX_S,
        _effective_session_config,
        _ensure_smoke_network,
        _load_runtime_config,
        _new_instance_id,
        _noninteractive_smoke_network_config,
        _ota_arm_profile,
        _ota_cleanup_hosts,
        _ota_collect_logs,
        _ota_network_sudo_passwords,
        _ota_preflight,
        _ota_prepare_hosts,
        _ota_resolve_interfaces,
        _ota_start_peers,
        _ota_start_session_publishers,
        _ota_stop_peers,
        _ota_stop_session_publishers,
        _ota_teardown_profile,
        _ota_write_manifest,
        _ota_write_state,
        _peer_command_runner,
        _peer_watchdog_launcher,
        _profile_directions,
        _profile_directions_for,
        _remove_smoke_network,
        _resolve_ota_profile,
        _resolve_ota_smoke_context,
        _resolve_session,
        _resolve_session_instance,
        _smoke_peer_address_args,
        _smoke_ros_setup,
        _stop_container_name,
        _write_docker_log,
        start_session,
    )
    from .network_profiles import expand_timeline
    from .network_shaper import ProfileShaper

    is_ota = _is_ota_benchmark(args)
    if not is_ota:
        # OTA runs check the peers instead (see _ota_preflight conflict checks).
        _abort_on_local_benchmark_conflicts(args)

    def run_point(
        *,
        profile: str | None,
        load: dict[str, Any],
        duration_s: float,
        out_dir: Path,
    ) -> dict[str, Any]:
        if is_ota:
            # --- Real-near (ota-smoke) path ---
            # Default project to fzi_projects/remote_assist/rosotacom.yaml if not set.
            rosotacom_config = getattr(args, "rosotacom_config", None)
            if not rosotacom_config:
                candidate = Path.cwd() / "fzi_projects" / "remote_assist" / "rosotacom.yaml"
                if candidate.is_file():
                    args.rosotacom_config = str(candidate)
            target_name, target_type = _benchmark_ota_target(args, session_name)
            profiles_file = _benchmark_profiles_file(profile, _load_runtime_config(args).profiles_file)

            smoke_args = argparse.Namespace(
                rosotacom_config=getattr(args, "rosotacom_config", None),
                ros2docker_config=getattr(args, "ros2docker_config", None),
                session_configs_dir=getattr(args, "session_configs_dir", None),
                scenario_configs_dir=getattr(args, "scenario_configs_dir", None),
                session_instances_dir=getattr(args, "session_instances_dir", None),
                deployment=getattr(args, "deployment", None),
                profiles_file=str(profiles_file) if profiles_file else None,
                target=target_name,
                target_type=target_type,
                peer=getattr(args, "peer", None),
                peer_address=getattr(args, "peer_address", None),
                peer_ssh=getattr(args, "peer_ssh", None),
                peer_checkout=getattr(args, "peer_checkout", None),
                peer_cpuset=getattr(args, "peer_cpuset", None),
                peer_exec=getattr(args, "peer_exec", None),
                install_mode=getattr(args, "install_mode", None),
                install_pin=getattr(args, "install_pin", None),
                workdir=getattr(args, "workdir", None),
                reuse=getattr(args, "reuse", False),
                skip_preflight=getattr(args, "skip_preflight", False),
                keep_workdir=getattr(args, "keep_workdir", False),
                dry_run=getattr(args, "dry_run", False),
                instance_id=getattr(args, "instance_id", None),
                profile=profile,
                benchmark_stepping=True,
                sudo_mode=getattr(args, "sudo_mode", "passwordless"),
                shaping_image=getattr(args, "shaping_image", None),
            )

            runtime, plan, target = _resolve_ota_smoke_context(smoke_args)
            dry_run = smoke_args.dry_run
            drain_s = _benchmark_drain_s(args)
            _profile_name, profile_obj = _resolve_ota_profile(runtime, target, smoke_args)
            sudo_passwords = _ota_network_sudo_passwords(
                plan,
                sudo_mode=smoke_args.sudo_mode,
                require_network_shaping_sudo=profile_obj is not None,
                dry_run=dry_run,
            )

            if not smoke_args.skip_preflight:
                _ota_preflight(
                    plan,
                    require_tmux=target.target_type == "scenario",
                    check_peer_reachability=False,
                    dry_run=dry_run,
                    require_network_shaping_sudo=profile_obj is not None,
                    sudo_mode=smoke_args.sudo_mode,
                    sudo_passwords=sudo_passwords,
                    shaping_image=smoke_args.shaping_image,
                    check_conflicts=not getattr(args, "skip_conflict_check", False),
                )
            _ota_prepare_hosts(smoke_args, runtime, plan)
            instance = _resolve_session_instance(
                runtime,
                target.session,
                smoke_args.instance_id or _new_instance_id(),
            )
            if dry_run:
                plan = replace(plan, state_path=instance.host_dir / "ota-deployment.yaml")
            else:
                plan = _ota_write_state(instance, plan)
            _ota_write_manifest(
                instance,
                target,
                runtime,
                plan,
                tmux_session=None,
                interactive=False,
                phase="running",
            )
            directions = _profile_directions(plan, target.cfg) if profile_obj is not None else {}
            shapers = []
            try:
                # Arm the profile (or start stepping if timeline)
                if profile_obj is not None:
                    if profile_obj.is_timeline:
                        peer_names = sorted(plan.peers)
                        peer_steps = {}
                        for peer_name in peer_names:
                            peer = plan.peers[peer_name]
                            direction = directions.get(peer_name, "uplink")
                            other_addr = next(plan.peers[name].address for name in peer_names if name != peer_name)
                            if dry_run:
                                ota_iface, control_iface = "<ota-if>", None
                            else:
                                ota_iface, control_iface = _ota_resolve_interfaces(peer, other_addr, dry_run=False)
                            shaper = ProfileShaper(
                                ota_iface,
                                _peer_command_runner(
                                    peer,
                                    dry_run=dry_run,
                                    sudo_password=sudo_passwords.get(peer_name),
                                    sudo_mode=smoke_args.sudo_mode,
                                    shaping_image=smoke_args.shaping_image,
                                ),
                                control_interface=control_iface,
                                safety_max_duration_s=_PROFILE_SAFETY_MAX_S,
                                watchdog_launcher=_peer_watchdog_launcher(
                                    peer,
                                    dry_run=dry_run,
                                    sudo_password=sudo_passwords.get(peer_name),
                                    sudo_mode=smoke_args.sudo_mode,
                                    shaping_image=smoke_args.shaping_image,
                                ),
                            )
                            shapers.append(shaper)
                            steps = expand_timeline(profile_obj, ota_iface, direction=direction)
                            peer_steps[peer_name] = (shaper, steps)

                        for shaper in shapers:
                            shaper.arm([])
                    else:
                        shapers = _ota_arm_profile(
                            plan,
                            profile_obj,
                            directions,
                            dry_run=dry_run,
                            sudo_passwords=sudo_passwords,
                            sudo_mode=smoke_args.sudo_mode,
                            shaping_image=smoke_args.shaping_image,
                        )

                _ota_start_peers(target, plan, instance.instance_id, dry_run=dry_run)
                if not dry_run:
                    time.sleep(12)
                # The load is the benchmark's, not the session's expectation:
                # without this the peer published what its own `expect` declares
                # and the run reported the load that was asked for.
                _ota_start_session_publishers(target, plan, dry_run=dry_run, load=load)

                if not dry_run:
                    if profile_obj is not None and profile_obj.is_timeline:
                        num_steps = len(profile_obj.timeline)
                        for i in range(num_steps):
                            step_duration = profile_obj.timeline[i].for_s
                            for peer_name in peer_names:
                                shaper, steps = peer_steps[peer_name]
                                step = steps[i]
                                print(f"Timeline step {i}: arming commands for {peer_name}:{shaper.interface}")
                                shaper.apply(step.commands)
                            time.sleep(step_duration)
                    else:
                        time.sleep(duration_s)
                    _ota_stop_session_publishers(target, plan, dry_run=dry_run)
                    if drain_s > 0.0:
                        time.sleep(drain_s)
            finally:
                _ota_teardown_profile(shapers)
                _ota_collect_logs(instance, plan, dry_run=dry_run)
                if not getattr(smoke_args, "keep_running", False):
                    _ota_stop_peers(target, plan, instance.instance_id, dry_run=dry_run)
                    _ota_write_manifest(
                        instance,
                        target,
                        runtime,
                        plan,
                        tmux_session=None,
                        interactive=False,
                        phase="stopped",
                    )
                    if not getattr(smoke_args, "keep_workdir", False):
                        _ota_cleanup_hosts(plan, dry_run=dry_run)
            if dry_run:
                return {
                    "topics": {
                        "/bench_capacity": {
                            "expected": 100,
                            "delivered": 100,
                            "lost": 0,
                            "loss_pct": 0.0,
                            "reordered": 0,
                            "ota_hop_ms": {"p50": 50.0, "p95": 100.0},
                            "jitter_ms": {"p50": 1.0, "p95": 3.0},
                        }
                    }
                }
            if not dry_run:
                dest = out_dir / "probes" / _probe_point_dirname(instance.instance_id, load)
                shutil.copytree(instance.host_dir, dest, dirs_exist_ok=True)
            return collect_transit_summary(instance.host_dir)

        else:
            # --- Lab (smoke) path ---
            if getattr(args, "dry_run", False):
                return {
                    "topics": {
                        "/bench_capacity": {
                            "expected": 100,
                            "delivered": 100,
                            "lost": 0,
                            "loss_pct": 0.0,
                            "reordered": 0,
                            "ota_hop_ms": {"p50": 50.0, "p95": 100.0},
                            "jitter_ms": {"p50": 1.0, "p95": 3.0},
                        }
                    }
                }

            runtime = _load_runtime_config(args)
            session = _resolve_session(session_name, runtime)
            instance_id = getattr(args, "instance_id", None) or _new_instance_id()
            smoke_instance = _resolve_session_instance(runtime, session, instance_id)
            smoke_network = _noninteractive_smoke_network_config(runtime, session, smoke_instance.instance_id)
            drain_s = _benchmark_drain_s(args)

            peer_address_args = _smoke_peer_address_args(smoke_network.peer_ips)
            cfg = _effective_session_config(session.host_dir, runtime)
            common = {
                "rosotacom_config": args.rosotacom_config,
                "ros2docker_config": args.ros2docker_config,
                "session_configs_dir": args.session_configs_dir,
                "session_instances_dir": getattr(args, "session_instances_dir", None),
                "deployment": args.deployment,
                "session_dir": session_name,
                "mode": "detached",
                # Names are instance-scoped; nothing to force-replace, and a
                # parallel run's containers must never be stopped from here.
                "force": False,
                "rewrite_formatting": False,
                "peer": [],
                "peer_address": peer_address_args,
                "instance_id": smoke_instance.instance_id,
                "network_name": smoke_network.name,
            }

            from .network_profiles import load_profiles_file, shaping_commands

            profile_name = profile
            profile_obj = None
            profiles_file = _benchmark_profiles_file(profile_name, runtime.profiles_file)
            if profile_name and profiles_file:
                profiles = load_profiles_file(profiles_file)
                profile_obj = profiles.get(profile_name)
                if profile_obj is None:
                    raise ValueError(f"Profile {profile_name!r} not found in profiles file {profiles_file}.")

            container_tc_overrides: dict[str, str] = {}

            def install_seeded_tc(container_name: str) -> str:
                host_tc = shutil.which("tc")
                if host_tc is None:
                    raise RuntimeError(
                        "netem seed was requested, but no host tc binary was found to copy into the container."
                    )
                target = "/tmp/rosotacom-host-tc"
                commands = [
                    ["docker", "cp", host_tc, f"{container_name}:{target}"],
                    ["docker", "exec", "-u", "root", container_name, "chmod", "0755", target],
                    ["docker", "exec", "-u", "root", container_name, target, "-V"],
                ]
                for cmd in commands:
                    res = subprocess.run(cmd, capture_output=True, text=True, check=False)
                    if res.returncode != 0:
                        raise RuntimeError(
                            f"failed to install seed-capable tc in {container_name}: "
                            f"{' '.join(cmd)} -> {res.stderr or res.stdout}"
                        )
                print(f"  netem seed support in {container_name}: using host tc {host_tc} as {target}")
                return target

            def make_container_runner(container_name: str) -> Callable[[Sequence[str]], None]:
                def run(argv: Sequence[str]) -> None:
                    # Execute tc command inside container namespace as root.
                    adjusted_argv = list(argv)
                    if adjusted_argv and adjusted_argv[0] == "tc":
                        adjusted_argv[0] = container_tc_overrides.get(container_name, "tc")
                    cmd = ["docker", "exec", "-u", "root", container_name] + adjusted_argv
                    res = subprocess.run(cmd, capture_output=True, text=True, check=False)
                    if res.returncode != 0:
                        raise RuntimeError(
                            f"network shaping command failed in {container_name}: "
                            f"{' '.join(cmd)} -> {res.stderr or res.stdout}"
                        )

                return run

            # Replay-as-load: the target session's own contract publishers drive
            # the link (bag-shaped synthetic payloads via `_start_smoke_topic_publishers`,
            # both directions), instead of a single `sized_publisher` stream. The
            # container lifecycle, profile shaping, measurement window and transit
            # collection below are identical to the synthetic path.
            replay = bool(getattr(args, "_benchmark_replay_target", None))
            ros_setup_a = _smoke_ros_setup(smoke_instance.config_container_dir, cfg, "a")
            ros_setup_b = _smoke_ros_setup(smoke_instance.config_container_dir, cfg, "b")
            topics_a = cfg.get("topics", {}).get("a_to_b", [])
            replay_publisher_duration = int(duration_s + 120.0)
            pub_cmds = []
            if not replay:
                publisher_streams = _publisher_streams_for_topic_specs(topics_a, load)
                for topic_index, topic_spec in enumerate(topics_a):
                    topic_name = topic_spec.get("topic")
                    publisher_args = " ".join(
                        shlex.quote(str(part))
                        for part in _sized_publisher_param_args(topic_name, load, streams=publisher_streams)
                    )
                    log_path = (
                        "${ROSOTACOM_LOGS_DIR}/a/sized_publisher.log"
                        if len(topics_a) == 1
                        else f"${{ROSOTACOM_LOGS_DIR}}/a/sized_publisher_{topic_index}.log"
                    )
                    cmd = (
                        f"{ros_setup_a} && timeout 300 ros2 run com_py sized_publisher --ros-args "
                        f"{publisher_args} "
                        f'> "{log_path}" 2>&1'
                    )
                    pub_cmds.append(cmd)

            a_container: str | None = None
            b_container: str | None = None
            lab_shapers: list[ProfileShaper] = []
            lab_peer_steps: dict[str, tuple[ProfileShaper, Any]] = {}
            measurement_window: tuple[float, float] | None = None

            def stop_local_publishers() -> None:
                for container in (a_container, b_container) if replay else (a_container,):
                    if not container:
                        continue
                    for pattern in ("sized_publisher", "ros2 topic pub"):
                        subprocess.run(
                            ["docker", "exec", container, "pkill", "-f", pattern],
                            capture_output=True,
                            check=False,
                        )

            try:
                from .cli import _smoke_network_labels, _smoke_target_key

                _ensure_smoke_network(
                    smoke_network.name,
                    smoke_network.subnet,
                    labels=_smoke_network_labels(runtime, _smoke_target_key("session", session.host_dir.name)),
                )
                a_container = start_session(
                    argparse.Namespace(
                        **common,
                        identity="a",
                        auto_identity=True,
                        network_ip=smoke_network.peer_ips["a"],
                    )
                )
                b_container = start_session(
                    argparse.Namespace(
                        **common,
                        identity="b",
                        auto_identity=True,
                        network_ip=smoke_network.peer_ips["b"],
                    )
                )
                if not getattr(args, "dry_run", False):
                    time.sleep(12)

                if profile_obj is not None and _profile_requires_netem_seed(profile_obj):
                    container_tc_overrides[a_container] = install_seeded_tc(a_container)
                    container_tc_overrides[b_container] = install_seeded_tc(b_container)

                # 1. Start the publishers
                if replay:
                    from .cli import _start_smoke_topic_publishers

                    _start_smoke_topic_publishers(
                        {"a": a_container, "b": b_container},
                        {"a": ros_setup_a, "b": ros_setup_b},
                        cfg,
                        log_line=print,
                        duration=float(replay_publisher_duration),
                    )
                else:
                    for cmd in pub_cmds:
                        subprocess.run(
                            ["docker", "exec", "-d", a_container, "bash", "-c", cmd],
                            capture_output=True,
                            text=True,
                            check=False,
                        )

                # 2. Wait for discovery to complete on unshaped network
                if not getattr(args, "dry_run", False):
                    default_topic = "/bench_capacity"
                    topic_name = (topics_a[0].get("topic") if topics_a else default_topic) or default_topic
                    if not replay:
                        # A. Wait for local subscription discovery of the single load topic.
                        info_cmd = f"{ros_setup_a} && ros2 topic info {shlex.quote(topic_name)}"
                        for _ in range(30):
                            res = subprocess.run(
                                ["docker", "exec", a_container, "bash", "-c", info_cmd],
                                capture_output=True,
                                text=True,
                                check=False,
                            )
                            output = res.stdout or ""
                            if "Subscription count: 0" not in output and "Subscription count:" in output:
                                break
                            time.sleep(0.5)

                    # B. Wait for end-to-end delivery: the load topic to Peer B in
                    # synthetic mode, or any contract topic reaching either peer in
                    # replay mode (topics cross both directions).
                    status_paths = (
                        [smoke_instance.host_dir / "logs" / peer / "status" / "status.json" for peer in ("a", "b")]
                        if replay
                        else [smoke_instance.host_dir / "logs" / "b" / "status" / "status.json"]
                    )
                    for _ in range(60):
                        if _benchmark_delivery_observed(status_paths, base=None if replay else topic_name):
                            break
                        time.sleep(0.5)
                    time.sleep(3.0)

                # 3. Arm the network shapers (after discovery is complete)
                if profile_obj is not None:
                    from .network_profiles import required_dist_installs

                    dist_installs = required_dist_installs(profile_obj)
                    if dist_installs:
                        # Custom delay-distribution tables must exist inside the
                        # shaping netns before netem references them by name, and
                        # in the directory this image's `tc` was compiled to read
                        # -- see tc_lib_dir_probe for why that is not a constant.
                        from .network_profiles import tc_lib_dir_probe

                        for dist_container in (a_container, b_container):
                            if not dist_container:
                                continue
                            probe = subprocess.run(
                                ["docker", "exec", "-u", "root", dist_container, "sh", "-c", tc_lib_dir_probe()],
                                capture_output=True,
                                text=True,
                                check=False,
                            )
                            tc_lib_dir = probe.stdout.strip()
                            if probe.returncode != 0 or not tc_lib_dir.startswith("/"):
                                raise RuntimeError(
                                    f"could not locate the tc library directory in {dist_container}: "
                                    f"{probe.stderr or probe.stdout}"
                                )
                            for table_name, file_path in dist_installs:
                                dist_target = f"{tc_lib_dir}/{table_name}.dist"
                                res = subprocess.run(
                                    ["docker", "cp", file_path, f"{dist_container}:{dist_target}"],
                                    capture_output=True,
                                    text=True,
                                    check=False,
                                )
                                if res.returncode != 0:
                                    raise RuntimeError(
                                        f"failed to install netem distribution table {table_name!r} in "
                                        f"{dist_container}: {file_path} -> {dist_target}: "
                                        f"{res.stderr or res.stdout}"
                                    )
                                print(
                                    f"  netem distribution {table_name!r} installed in {dist_container}: {dist_target}"
                                )
                    # Which peer's egress carries `uplink` is the session's
                    # answer, not the runner's: the same session under the same
                    # profile must be shaped identically here and across two
                    # machines, and hard-coding a=uplink here while the OTA path
                    # hard-coded b=uplink meant it was not.
                    lab_directions = _profile_directions_for(["a", "b"], cfg)
                    lab_containers = {"a": a_container, "b": b_container}
                    if profile_obj.is_timeline:
                        for peer_name in ("a", "b"):
                            shaper = ProfileShaper("eth0", make_container_runner(lab_containers[peer_name]))
                            lab_shapers.append(shaper)
                            steps = expand_timeline(profile_obj, "eth0", direction=lab_directions[peer_name])
                            lab_peer_steps[peer_name] = (shaper, steps)

                        for shaper in lab_shapers:
                            shaper.arm([])
                    else:
                        for peer_name in ("a", "b"):
                            leg = profile_obj.uplink if lab_directions[peer_name] == "uplink" else profile_obj.downlink
                            if leg is None or leg.is_empty:
                                continue
                            shaper = ProfileShaper("eth0", make_container_runner(lab_containers[peer_name]))
                            lab_shapers.append(shaper)
                            shaper.arm(shaping_commands("eth0", leg))

                # 4. Verify shaping was applied (diagnostic)
                if profile_obj is not None and not getattr(args, "dry_run", False):
                    for peer_name in ("a", "b"):
                        label = f"{peer_name.upper()} ({lab_directions[peer_name]})"
                        container = lab_containers[peer_name]
                        tc_binary = container_tc_overrides.get(container, "tc")
                        res = subprocess.run(
                            ["docker", "exec", "-u", "root", container, tc_binary, "qdisc", "show", "dev", "eth0"],
                            capture_output=True,
                            text=True,
                            check=False,
                        )
                        print(f"  qdisc on {label} ({container}): {(res.stdout or '').strip()}")

                # 5. Measure traffic under shaped conditions
                measurement_start_s = time.time()
                if profile_obj is not None and profile_obj.is_timeline:
                    num_steps = len(profile_obj.timeline)
                    for i in range(num_steps):
                        step_duration = profile_obj.timeline[i].for_s
                        for shaper, steps in lab_peer_steps.values():
                            step = steps[i]
                            shaper.apply(step.commands)
                        time.sleep(step_duration)
                else:
                    time.sleep(duration_s)
                measurement_end_s = time.time()
                stop_local_publishers()
                if drain_s > 0.0:
                    time.sleep(drain_s)
                measurement_window = (measurement_start_s, measurement_end_s)
            finally:
                for shaper in lab_shapers:
                    shaper.teardown()

                for cleanup_container in [a_container, b_container]:
                    if cleanup_container:
                        subprocess.run(
                            ["docker", "exec", cleanup_container, "pkill", "-f", "sized_publisher"],
                            capture_output=True,
                            check=False,
                        )
                        _write_docker_log(
                            cleanup_container,
                            smoke_instance,
                            "a" if cleanup_container == a_container else "b",
                        )
                        _stop_container_name(cleanup_container, runtime)
                _remove_smoke_network(smoke_network.name)

            dest = out_dir / "probes" / _probe_point_dirname(smoke_instance.instance_id, load)
            shutil.copytree(smoke_instance.host_dir, dest, dirs_exist_ok=True)

            return collect_transit_summary(smoke_instance.host_dir, publish_window=measurement_window)

    return run_point


def benchmark_probe(args: argparse.Namespace) -> int:
    """Handler for ``rosotacom benchmark probe``."""
    from .cli import _load_runtime_config

    if getattr(args, "interactive", False):
        return _start_interactive_benchmark(args, "probe")

    runtime = _load_runtime_config(args)
    artifacts_dir = Path(args.artifacts_dir) if getattr(args, "artifacts_dir", None) else runtime.benchmarks_dir

    run_dir = _setup_benchmark_run_dir(args, "probe", args.profile)
    session_context = _benchmark_session_context(args, "bench_1_1_capacity", run_dir)
    result_context = _benchmark_result_context(
        args,
        genre="probe",
        profile=args.profile,
        run_dir=run_dir,
        session=session_context,
    )
    probe_load = _benchmark_probe_load(args)
    _attach_benchmark_diagnostics(
        result_context,
        spdp=_probe_spdp_diagnostics(
            args=args,
            profile=_normalize_benchmark_profile(args.profile),
            duration_s=float(getattr(args, "duration", 60.0)),
            load_info=_load_context(probe_load),
        ),
    )

    run_point = getattr(args, "_test_run_point", None) or _make_live_run_point(args, "bench_1_1_capacity")

    with log_stdout_stderr_to_file(run_dir / "stdout.txt"):
        _print_benchmark_warnings(result_context)
        result = drive_probe(
            run_point,
            profile=args.profile,
            size=getattr(args, "size", 18_000),
            size_pattern=getattr(args, "size_pattern", None),
            rate_hz=getattr(args, "rate_hz", 20.0),
            streams=getattr(args, "streams", 1),
            topic=getattr(args, "topic", ""),
            repeats=getattr(args, "repeats", 1),
            duration_s=getattr(args, "duration", 60.0),
            bin_s=getattr(args, "bin_s", 1.0),
            render_plot=getattr(args, "plot", True),
            out_dir=run_dir,
            result_context=result_context,
            interval_jitter_ms=getattr(args, "interval_jitter_ms", 0.0),
            interval_jitter_seed=getattr(args, "interval_jitter_seed", 42),
        )

    out_file_name = getattr(args, "out", "time-bins.jsonl")
    if artifacts_dir:
        out_path = artifacts_dir / Path(out_file_name).name
    else:
        out_path = Path(out_file_name)

    run_bins_file = run_dir / "time-bins.jsonl"
    if run_bins_file.is_file():
        out_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(run_bins_file, out_path)
        print(f"Probe time bins copied to {out_path}")

    plot_file = run_dir / "probe-timeseries.png"
    if plot_file.is_file() and artifacts_dir:
        plot_out = artifacts_dir / plot_file.name
        shutil.copy2(plot_file, plot_out)
        print(f"Probe plot copied to {plot_out}")

    print(f"Probe: {result['time_bin_count']} bins → {run_dir / BENCHMARK_RESULT_FILE}")
    return 0


def benchmark_capacity(args: argparse.Namespace) -> int:
    """Handler for ``rosotacom benchmark capacity``."""
    if getattr(args, "interactive", False):
        return _start_interactive_benchmark(args, "capacity")

    run_dir = _setup_benchmark_run_dir(args, "capacity", args.profile)
    session_context = _benchmark_session_context(args, "bench_1_1_capacity", run_dir)
    result_context = _benchmark_result_context(
        args,
        genre="capacity",
        profile=args.profile,
        run_dir=run_dir,
        session=session_context,
    )

    # Use stub probe under test, or live probe in production.
    run_point = getattr(args, "_test_run_point", None) or _make_live_run_point(args, "bench_1_1_capacity")

    with log_stdout_stderr_to_file(run_dir / "stdout.txt"):
        result = drive_capacity(
            run_point,
            profile=args.profile,
            knob=args.knob,
            low=args.low,
            high=args.high,
            max_loss_pct=args.max_loss,
            max_latency_ms=args.max_latency_ms,
            rate_hz=getattr(args, "rate_hz", 20.0),
            topic=getattr(args, "topic", ""),
            repeats=getattr(args, "repeats", 1),
            duration_s=getattr(args, "duration", 60.0),
            out_dir=run_dir,
            result_context=result_context,
        )

    print(f"Capacity: {result['slice']['knob']}={result['capacity']} → {run_dir / BENCHMARK_RESULT_FILE}")
    return 0


# --------------------------------------------------------------------------- #
# Band gate: compare + ratchet (performance bands reference)
# --------------------------------------------------------------------------- #


def _load_result_docs(paths: Sequence[str]) -> list[dict[str, Any]]:
    """Load result.json documents; a run directory stands for its result.json."""
    docs: list[dict[str, Any]] = []
    for raw in paths:
        path = Path(raw)
        if path.is_dir():
            path = path / BENCHMARK_RESULT_FILE
        docs.append(json.loads(path.read_text(encoding="utf-8")))
    return docs


def _resolve_run_identity(
    docs: Sequence[dict[str, Any]],
    *,
    row: str | None = None,
    profile: str | None = None,
) -> tuple[str, str, str]:
    """One (row, profile, fingerprint) for the given runs — repeats, not a mixture."""
    from .benchmark import BandError, result_fingerprint, result_profile, result_row_id

    rows = {row or result_row_id(doc) for doc in docs}
    profiles = {profile or result_profile(doc) for doc in docs}
    fingerprints = {result_fingerprint(doc) for doc in docs}
    if len(rows) > 1 or len(profiles) > 1:
        raise BandError(
            f"the given runs span rows {sorted(rows)} and profiles {sorted(profiles)}; "
            "compare/ratchet takes repeats of one (row, profile) per invocation"
        )
    if len(fingerprints) > 1:
        raise BandError(
            f"the given runs come from different runner classes {sorted(fingerprints)}; "
            "use repeats from a single runner class"
        )
    return rows.pop(), profiles.pop(), fingerprints.pop()


def _median_run_metrics(docs: Sequence[dict[str, Any]]) -> dict[str, float]:
    """Per-metric median across the given runs (metrics all runs share)."""
    from .benchmark import BandError, metrics_from_result

    per_run = [metrics_from_result(doc) for doc in docs]
    shared = set(per_run[0])
    for metrics in per_run[1:]:
        shared &= set(metrics)
    if not shared:
        raise BandError("the given runs share no metrics — they are not repeats of one row")
    return {name: statistics.median([metrics[name] for metrics in per_run]) for name in sorted(shared)}


def _ratchet_command(
    results: Sequence[Any],
    budgets: Any,
    *,
    row: str | None = None,
    profile: str | None = None,
) -> str:
    """The exact ratchet invocation for the runs/bands `compare` was called with."""
    parts = ["rosotacom", "benchmark", "ratchet", *[str(p) for p in results], "--budgets", str(budgets)]
    if row:
        parts += ["--row", str(row)]
    if profile:
        parts += ["--profile", str(profile)]
    return " ".join(shlex.quote(part) for part in parts)


def _compare_docs_to_bands(
    docs: Sequence[dict[str, Any]],
    budgets_path: Path,
    *,
    row: str | None = None,
    profile: str | None = None,
    required_metrics: Sequence[str] = (),
    ratchet_hint: str,
) -> list[Any]:
    """Per-metric band comparisons for repeats of one row, or a ``BandError`` refusal.

    ``required_metrics`` closes the silent-no-op hole for registry rows: a gated
    metric whose band vanished from the store refuses instead of not gating.
    """
    from .benchmark import BandError, bands_for, compare_to_band, load_bands

    bands = load_bands(budgets_path)
    row_id, profile_name, fingerprint = _resolve_run_identity(docs, row=row, profile=profile)
    selected = bands_for(bands, row=row_id, profile=profile_name)
    if not selected:
        raise BandError(
            f"no committed bands for ({row_id}, {profile_name}) in {budgets_path} — calibrate them first: "
            f"{ratchet_hint} --recalibrate"
        )
    missing_bands = sorted(set(required_metrics) - {band.metric for band in selected})
    if missing_bands:
        raise BandError(
            f"({row_id}, {profile_name}) gates {missing_bands} but {budgets_path} has no band for them — "
            f"calibrate them first: {ratchet_hint} --recalibrate"
        )
    run_metrics = _median_run_metrics(docs)

    comparisons = []
    for band in selected:
        if band.metric not in run_metrics:
            raise BandError(
                f"the run(s) carry no {band.metric!r}, but ({row_id}, {profile_name}) bands it — "
                "the gate cannot assert a missing metric"
            )
        comparisons.append(compare_to_band(band, run_metrics[band.metric], fingerprint=fingerprint))
    return comparisons


def _print_band_comparisons(comparisons: Sequence[Any]) -> None:
    for comparison in comparisons:
        band = comparison.band
        print(
            f"{comparison.verdict.value:9s} {band.metric}: {comparison.value:g} vs [{band.lo:g}, {band.hi:g}] "
            f"(better={band.better.value}, runner={band.provenance.fingerprint})"
        )


def _gate_exit_code(comparisons: Sequence[Any], *, budgets: Any, ratchet_hint: str) -> int:
    """Print the two-sided gate verdict and map it to the CI exit code."""
    from .benchmark import Verdict

    regressed = [c for c in comparisons if c.verdict is Verdict.REGRESSED]
    improved = [c for c in comparisons if c.verdict is Verdict.IMPROVED]
    if regressed:
        names = ", ".join(c.band.metric for c in regressed)
        print(f"REGRESSED: {names} left the band on the worse side — fix the regression;")
        print("a deliberate trade-off is a recalibration with a cause note:")
        print(f"  {ratchet_hint} --recalibrate --note '<why this level is accepted>'")
        return 1
    if improved:
        names = ", ".join(c.band.metric for c in improved)
        print(f"IMPROVED: {names} left the band on the better side — nice. Bank it in this same change:")
        print(f"  {ratchet_hint} --note '<one-line cause>'")
        print(f"then commit the tightened {budgets} (the band diff is part of the review).")
        return 2
    print(f"WITHIN: all {len(comparisons)} banded metric(s) inside their bands.")
    return 0


def _threshold_failures(metrics: dict[str, float], rules: dict[str, dict[str, float]]) -> list[str]:
    failures: list[str] = []
    for metric, rule in rules.items():
        if metric not in metrics:
            failures.append(f"{metric}: missing")
            continue
        value = float(metrics[metric])
        if "min" in rule and value < float(rule["min"]):
            failures.append(f"{metric}: {value:g} < min {float(rule['min']):g}")
        if "max" in rule and value > float(rule["max"]):
            failures.append(f"{metric}: {value:g} > max {float(rule['max']):g}")
    return failures


def _format_thresholds(rules: dict[str, dict[str, float]]) -> str:
    parts: list[str] = []
    for metric, rule in sorted(rules.items()):
        limits = []
        if "min" in rule:
            limits.append(f">= {float(rule['min']):g}")
        if "max" in rule:
            limits.append(f"<= {float(rule['max']):g}")
        parts.append(f"{metric} {' and '.join(limits)}")
    return ", ".join(parts)


def _boundary_side_row(row: Any, *, side: str, profile: str) -> Any:
    return replace(row, id=f"{row.id}-{side}", profile=profile)


def _drive_boundary_row(args: argparse.Namespace, row: Any, run_dir: Path) -> dict[str, dict[str, Any]]:
    """Run a boundary row's good and bad sides under their committed profiles."""
    sides = {
        "good": row.profile,
        "bad": str(row.boundary["bad_profile"]),
    }
    docs: dict[str, dict[str, Any]] = {}
    for side, profile in sides.items():
        side_dir = run_dir / side
        _drive_gate_row(args, _boundary_side_row(row, side=side, profile=profile), side_dir)
        docs[side] = json.loads((side_dir / BENCHMARK_RESULT_FILE).read_text(encoding="utf-8"))
    return docs


def _boundary_verdict(
    row: Any,
    *,
    comparisons: Sequence[Any],
    side_metrics: dict[str, dict[str, float]],
    budgets: Any,
    ratchet_hint: str,
) -> tuple[str, int, str | None]:
    """Map a good/bad boundary pair to the gate verdict.

    Normal rows assert success. Boundary rows assert both success (good side)
    and a documented failure signature (bad side). If the bad side now satisfies
    the good oracle, the system improved past the documented envelope: the row is
    happy-red with a concrete boundary-moving/ratchet message.
    """
    from .benchmark import Verdict

    finding = str(row.boundary["finding"])
    bad_profile = str(row.boundary["bad_profile"])
    good_oracle = row.boundary["good_oracle"]
    failure_signature = row.boundary["failure_signature"]
    next_steps = str(row.boundary["next_steps"])

    good_verdicts = {comparison.verdict for comparison in comparisons}
    if Verdict.REGRESSED in good_verdicts:
        print(
            f"REGRESSED: boundary good side {row.profile} no longer satisfies {finding}; "
            "fix the regression or recalibrate the documented good boundary."
        )
        print(f"  good oracle: {_format_thresholds(good_oracle)}")
        return Verdict.REGRESSED.value, 1, None
    if Verdict.IMPROVED in good_verdicts:
        print(f"IMPROVED: boundary good side {row.profile} beat its band. Bank it before judging the pair:")
        print(f"  {ratchet_hint} --note '<one-line cause>'")
        return Verdict.IMPROVED.value, 2, ratchet_hint

    good_oracle_failures = _threshold_failures(side_metrics["good"], good_oracle)
    if good_oracle_failures:
        print(
            f"REGRESSED: boundary good side {row.profile} stayed within its band but failed "
            f"the documented oracle for {finding}: {'; '.join(good_oracle_failures)}"
        )
        return Verdict.REGRESSED.value, 1, None

    failure_signature_failures = _threshold_failures(side_metrics["bad"], failure_signature)
    if not failure_signature_failures:
        print(
            f"WITHIN: boundary good side {row.profile} passed and bad side {bad_profile} still matches "
            f"the documented failure signature ({_format_thresholds(failure_signature)})."
        )
        return Verdict.WITHIN.value, 0, None

    bad_good_failures = _threshold_failures(side_metrics["bad"], good_oracle)
    if not bad_good_failures:
        print(
            f"BOUNDARY_WIDENED: bad side {bad_profile} now satisfies the good oracle for {finding} "
            f"({_format_thresholds(good_oracle)})."
        )
        print(f"  profiles: good={row.profile}, bad={bad_profile}")
        print(f"  next: {next_steps}")
        print("  after moving the bad-side profile and updating the finding, bank the good-side band:")
        print(f"  {ratchet_hint} --note '<one-line cause>'")
        print(f"then commit the tightened {budgets} and the finding/profile update together.")
        return "BOUNDARY_WIDENED", 2, ratchet_hint

    print(
        f"REGRESSED: bad side {bad_profile} no longer matches the documented failure signature for {finding}, "
        "but it also did not satisfy the good oracle."
    )
    print(f"  signature misses: {'; '.join(failure_signature_failures)}")
    print(f"  good-oracle misses: {'; '.join(bad_good_failures)}")
    print(f"  next: {next_steps}")
    return Verdict.REGRESSED.value, 1, None


def benchmark_compare(args: argparse.Namespace) -> int:
    """Handler for ``rosotacom benchmark compare`` — the two-sided gate verdict.

    Exit codes: 0 all WITHIN, 1 REGRESSED or any refusal (fingerprint mismatch,
    missing band/metric), 2 IMPROVED beyond the band — red too, but the fix is
    the printed ratchet command, not a revert. ``--monitor`` reports the same
    verdicts (refusals included) and always exits 0.
    """
    from .benchmark import BandError

    docs = _load_result_docs(args.results)
    ratchet_hint = _ratchet_command(args.results, args.budgets, row=args.row, profile=args.profile)
    try:
        comparisons = _compare_docs_to_bands(
            docs, Path(args.budgets), row=args.row, profile=args.profile, ratchet_hint=ratchet_hint
        )
    except BandError as exc:
        print(f"REFUSED: {exc}")
        return 0 if args.monitor else 1
    _print_band_comparisons(comparisons)
    exit_code = _gate_exit_code(comparisons, budgets=args.budgets, ratchet_hint=ratchet_hint)
    return 0 if args.monitor else exit_code


def benchmark_ratchet(args: argparse.Namespace) -> int:
    """Handler for ``rosotacom benchmark ratchet`` — the only way bands change."""
    from .benchmark import (
        BandError,
        Better,
        default_better,
        find_band,
        load_bands,
        metrics_from_result,
        ratchet_band,
        result_sha,
        result_window_s,
        save_bands,
    )

    docs = _load_result_docs(args.results)
    budgets_path = Path(args.budgets)
    bands = load_bands(budgets_path) if budgets_path.is_file() else []
    row, profile, fingerprint = _resolve_run_identity(docs, row=args.row, profile=args.profile)
    run_metrics = _median_run_metrics(docs)
    per_run = [metrics_from_result(doc) for doc in docs]

    selected_metrics = sorted(run_metrics)
    if args.metric:
        missing = sorted(set(args.metric) - set(run_metrics))
        if missing:
            raise BandError(f"--metric {missing} not among the run metrics {sorted(run_metrics)}")
        selected_metrics = sorted(set(args.metric))

    windows = {result_window_s(doc) for doc in docs}
    if len(windows) > 1:
        raise BandError(f"the given runs disagree on window length {sorted(windows)} — calibrate from uniform repeats")
    window_s = windows.pop()
    shas = {result_sha(doc) for doc in docs}
    source_sha = shas.pop() if len(shas) == 1 else "mixed"
    ratcheted_at = datetime.now().isoformat(timespec="seconds")
    better_override = Better(args.better) if getattr(args, "better", None) else None

    updated: dict[tuple[str, str, str], Any] = {}
    skipped: list[str] = []
    for metric in selected_metrics:
        values = [metrics[metric] for metrics in per_run]
        existing = find_band(bands, row=row, profile=profile, metric=metric)
        if existing is None and not args.recalibrate:
            skipped.append(metric)
            continue
        better = better_override or (existing.better if existing is not None else default_better(metric))
        new_band = ratchet_band(
            existing,
            values,
            row=row,
            profile=profile,
            metric=metric,
            better=better,
            fingerprint=fingerprint,
            window_s=window_s,
            source_sha=source_sha,
            ratcheted_at=ratcheted_at,
            note=args.note,
            recalibrate=args.recalibrate,
            k=args.k,
            floor=args.floor,
            floor_frac=args.floor_frac,
        )
        if existing is not None:
            print(
                f"ratchet {row}/{profile} {metric}: [{existing.lo:g}, {existing.hi:g}] → "
                f"[{new_band.lo:g}, {new_band.hi:g}]"
            )
        else:
            print(
                f"new band {row}/{profile} {metric}: [{new_band.lo:g}, {new_band.hi:g}] "
                f"(calibrated from {len(values)} run(s), runner={fingerprint})"
            )
        updated[new_band.key] = new_band

    if skipped:
        print(f"skipped (no committed band; create with --recalibrate): {', '.join(skipped)}")
    if not updated:
        raise BandError(
            f"no committed bands for ({row}, {profile}) match the run metrics {sorted(run_metrics)} — "
            "create them with --recalibrate"
        )
    kept = [band for band in bands if band.key not in updated]
    save_bands(budgets_path, kept + list(updated.values()))
    print(f"{len(updated)} band(s) written to {budgets_path}")
    return 0


# --------------------------------------------------------------------------- #
# Benched set: rows / row / calibrate / gate-summary (performance bands reference)
# --------------------------------------------------------------------------- #


def _load_gate_registry(args: argparse.Namespace) -> list[Any]:
    from .benched_set import load_registry

    registry_raw = getattr(args, "registry", None)
    return load_registry(Path(registry_raw) if registry_raw else None)


def benchmark_rows(args: argparse.Namespace) -> int:
    """Handler for ``rosotacom benchmark rows`` — list the benched set."""
    from .benched_set import rows_for_calibration, rows_for_lane

    rows = rows_for_lane(_load_gate_registry(args), args.lane)
    if getattr(args, "calibratable", False):
        rows = rows_for_calibration(rows)
    if args.format == "ids":
        print(json.dumps([row.id for row in rows]))
    elif args.format == "json":
        print(json.dumps([asdict(row) for row in rows], indent=2, sort_keys=True))
    else:
        for row in rows:
            print(f"{row.id:32s} {row.lane:10s} {row.rmw:8s} {row.genre:9s} {row.profile}")
            print(f"{'':32s} {row.reason}")
    return 0


def _drive_gate_row(args: argparse.Namespace, row: Any, run_dir: Path) -> None:
    """Run one registry row's genre driver with the row's committed parameters."""
    session_name = BENCHMARK_SESSIONS_BY_GENRE[row.genre]
    session_context = _benchmark_session_context(args, session_name, run_dir)
    result_context = _benchmark_result_context(
        args, genre=row.genre, profile=row.profile, run_dir=run_dir, session=session_context
    )
    result_context["gate_row"] = {"id": row.id, "lane": row.lane, "reason": row.reason}
    run_point = getattr(args, "_test_run_point", None) or _make_live_run_point(args, session_name)

    with log_stdout_stderr_to_file(run_dir / "stdout.txt"):
        _print_benchmark_warnings(result_context)
        if row.genre in ("probe", "replay"):
            load_kwargs = dict(
                profile=row.profile,
                size=int(row.load.get("size") or 0),
                size_pattern=row.load.get("size_pattern"),
                rate_hz=float(row.load["rate_hz"]),
                streams=int(row.load.get("streams") or 1),
                repeats=row.repeats,
                duration_s=row.duration_s,
                bin_s=1.0,
                render_plot=False,
                result_context=result_context,
                interval_jitter_ms=float(row.load.get("interval_jitter_ms") or 0.0),
                interval_jitter_seed=int(row.load.get("interval_jitter_seed") or 42),
            )
            if row.genre == "replay":
                drive_replay(run_point, replay=row.replay, out_dir=run_dir, **load_kwargs)
            else:
                drive_probe(run_point, out_dir=run_dir, **load_kwargs)
        else:
            drive_capacity(
                run_point,
                profile=row.profile,
                knob=str(row.search["knob"]),
                low=int(row.search["low"]),
                high=int(row.search["high"]),
                max_loss_pct=float(row.oracle["max_loss_pct"]),
                max_latency_ms=float(row.oracle["max_latency_ms"]),
                rate_hz=float(row.search["rate_hz"]),
                repeats=row.repeats,
                duration_s=row.duration_s,
                out_dir=run_dir,
                result_context=result_context,
            )


def _replay_expect_result(row: Any, doc: dict[str, Any]) -> dict[str, Any]:
    """Judge one replay run against the bag's whole-bag ``expect`` fragment."""
    from .benched_set import replay_expect_failures
    from .benchmark import probe_delivery_totals, result_window_s

    expected, delivered, attempts = probe_delivery_totals(doc)
    window_s = result_window_s(doc)
    per_attempt = delivered / attempts if attempts else 0
    failures = replay_expect_failures(
        row,
        expected=expected // attempts if attempts else 0,
        delivered=int(per_attempt),
        duration_s=window_s,
    )
    return {
        "expect": dict(row.replay["expect"]),
        "bag": {
            "source": row.replay["source"],
            "public_example": row.replay["public_example"],
            "native_count": row.replay["native_count"],
            "native_hz": row.replay["native_hz"],
        },
        "measured": {
            "expected": expected // attempts if attempts else 0,
            "delivered": int(per_attempt),
            "attempts": attempts,
            "window_s": window_s,
        },
        "failures": failures,
        "passed": not failures,
    }


def _print_replay_expect(row: Any, result: dict[str, Any]) -> None:
    measured = result["measured"]
    status = "OK" if result["passed"] else "FAILED"
    print(
        f"Replay expect {status} ({row.id}): delivered {measured['delivered']} of "
        f"{measured['expected']} over {measured['window_s']:g}s "
        f"vs {result['bag']['public_example']} ({result['bag']['native_count']} msgs "
        f"@ {result['bag']['native_hz']:g} Hz)"
    )
    for failure in result["failures"]:
        print(f"  - {failure}")


def benchmark_row(args: argparse.Namespace) -> int:
    """Handler for ``rosotacom benchmark row`` — run one benched row and gate it.

    Failure semantics (performance bands reference): a run/setup failure raises and is red;
    ``REGRESSED`` exits 1; ``IMPROVED`` exits 2 with the exact ratchet command
    (bank it, don't revert); a replay row whose whole-bag ``expect`` fragment did
    not hold exits 1 as ``EXPECT_FAILED``; a refusal (uncalibrated or
    foreign-runner bands, missing band) exits 1. ``--monitor`` reports the same
    verdict but exits 0; ``--no-compare`` skips gating entirely (calibration
    runs). The verdict is also written as machine-readable JSON for downstream
    gates.

    A row with no banded metrics is not gate-less: replay rows assert their
    bag-derived expect fragment from the day they land, and their bands follow
    from a calibration run on the runner class.
    """
    from .benched_set import find_row, verdict_document, write_verdict
    from .benchmark import BandError, Verdict, metrics_from_result, result_fingerprint, result_sha

    row = find_row(_load_gate_registry(args), args.row_id)
    args.rmw = row.rmw
    args.profile = row.profile

    run_dir = _setup_benchmark_run_dir(args, f"row-{row.id}", row.profile)
    if row.kind == "boundary":
        side_docs = _drive_boundary_row(args, row, run_dir)
        side_result_paths = {side: str(run_dir / side / BENCHMARK_RESULT_FILE) for side in ("good", "bad")}
        boundary_summary_path = run_dir / "boundary-result.json"

        boundary_refusal: str | None = None
        side_metrics: dict[str, dict[str, float]] = {}
        try:
            for side, doc in side_docs.items():
                side_metrics[side] = metrics_from_result(doc)
        except BandError as exc:
            boundary_refusal = str(exc)

        fingerprints = {result_fingerprint(doc) for doc in side_docs.values()}
        fingerprint = fingerprints.pop() if len(fingerprints) == 1 else "mixed"
        shas = {result_sha(doc) for doc in side_docs.values()}
        sha = shas.pop() if len(shas) == 1 else "mixed"
        if fingerprint == "mixed":
            side_fingerprints = sorted(result_fingerprint(doc) for doc in side_docs.values())
            boundary_refusal = f"boundary sides came from different runner classes: {side_fingerprints}"

        good_run_dir = run_dir / "good"
        ratchet_hint = _ratchet_command([good_run_dir], args.budgets, row=row.id, profile=row.profile)
        boundary_comparisons: list[Any] = []
        exit_code = 0
        verdict = "RAN"
        gate = not (args.no_compare or args.monitor)
        ratchet_command: str | None = None

        if not args.no_compare and boundary_refusal is None:
            try:
                boundary_comparisons = _compare_docs_to_bands(
                    [side_docs["good"]],
                    Path(args.budgets),
                    row=row.id,
                    profile=row.profile,
                    required_metrics=row.metrics,
                    ratchet_hint=ratchet_hint,
                )
            except BandError as exc:
                boundary_refusal = str(exc)

        if boundary_refusal is not None:
            print(f"REFUSED: {boundary_refusal}")
            verdict = "REFUSED"
            exit_code = 1
        elif not args.no_compare:
            print(f"Boundary row {row.id}: good={row.profile}, bad={row.boundary['bad_profile']}")
            _print_band_comparisons(boundary_comparisons)
            verdict, exit_code, ratchet_command = _boundary_verdict(
                row,
                comparisons=boundary_comparisons,
                side_metrics=side_metrics,
                budgets=args.budgets,
                ratchet_hint=ratchet_hint,
            )

        boundary_payload = {
            "finding": row.boundary["finding"],
            "good_profile": row.profile,
            "bad_profile": row.boundary["bad_profile"],
            "good_oracle": row.boundary["good_oracle"],
            "failure_signature": row.boundary["failure_signature"],
            "next_steps": row.boundary["next_steps"],
            "sides": {
                side: {
                    "profile": row.profile if side == "good" else row.boundary["bad_profile"],
                    "metrics": side_metrics.get(side, {}),
                    "result": side_result_paths[side],
                }
                for side in ("good", "bad")
            },
        }
        boundary_summary_path.write_text(
            json.dumps(boundary_payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        verdict_doc = verdict_document(
            row,
            verdict=verdict,
            exit_code=exit_code,
            gate=gate,
            sha=sha,
            fingerprint=fingerprint,
            created_at=datetime.now().isoformat(timespec="seconds"),
            metrics={
                name: side_metrics.get("good", {})[name] for name in row.metrics if name in side_metrics.get("good", {})
            },
            monitor_metrics={
                name: side_metrics.get("good", {})[name] for name in row.monitor if name in side_metrics.get("good", {})
            },
            bands={
                c.band.metric: {"lo": c.band.lo, "hi": c.band.hi, "better": c.band.better.value}
                for c in boundary_comparisons
            },
            result_path=str(boundary_summary_path),
            refusal=boundary_refusal,
            ratchet_command=ratchet_command,
            boundary=boundary_payload,
        )
        verdict_path = Path(args.verdict_file) if args.verdict_file else run_dir / "verdict.json"
        write_verdict(verdict_path, verdict_doc)
        print(f"Gate verdict {verdict} ({row.id}) written to {verdict_path}")
        return 0 if args.monitor else exit_code

    _drive_gate_row(args, row, run_dir)

    doc = json.loads((run_dir / BENCHMARK_RESULT_FILE).read_text(encoding="utf-8"))
    sha = result_sha(doc)
    fingerprint = result_fingerprint(doc)

    refusal: str | None = None
    ratchet_hint = _ratchet_command([run_dir], args.budgets, row=row.id, profile=row.profile)
    try:
        run_metrics = metrics_from_result(doc)
    except BandError as exc:
        run_metrics = {}
        refusal = str(exc)

    comparisons: list[Any] = []
    exit_code = 0
    verdict = "RAN"
    gate = not (args.no_compare or args.monitor)
    # An unbanded row has nothing to compare — asking the band store for bands it
    # was never meant to have would refuse instead of running.
    compare_bands = bool(row.metrics) and not args.no_compare
    if compare_bands and refusal is None:
        try:
            comparisons = _compare_docs_to_bands(
                [doc],
                Path(args.budgets),
                row=row.id,
                profile=row.profile,
                required_metrics=row.metrics,
                ratchet_hint=ratchet_hint,
            )
        except BandError as exc:
            refusal = str(exc)

    # A refused run has no trustworthy counts, so there is nothing to judge the
    # bag against either — the refusal is the finding.
    replay_expect = _replay_expect_result(row, doc) if row.genre == "replay" and refusal is None else None

    if refusal is not None:
        print(f"REFUSED: {refusal}")
        verdict = "REFUSED"
        exit_code = 1
    elif compare_bands:
        _print_band_comparisons(comparisons)
        exit_code = _gate_exit_code(comparisons, budgets=args.budgets, ratchet_hint=ratchet_hint)
        verdicts = {c.verdict for c in comparisons}
        if Verdict.REGRESSED in verdicts:
            verdict = Verdict.REGRESSED.value
        elif Verdict.IMPROVED in verdicts:
            verdict = Verdict.IMPROVED.value
        else:
            verdict = Verdict.WITHIN.value

    # The whole-bag expect fragment is bag-derived, not runner-derived, so it
    # asserts on every gating run — including one that has no bands yet. A band
    # verdict already red stays the reported one; the failures are printed either
    # way and recorded in the verdict document.
    if replay_expect is not None and not args.no_compare:
        _print_replay_expect(row, replay_expect)
        if not replay_expect["passed"]:
            if exit_code == 0:
                verdict = "EXPECT_FAILED"
            exit_code = exit_code or 1
        elif verdict == "RAN":
            verdict = Verdict.WITHIN.value

    verdict_doc = verdict_document(
        row,
        verdict=verdict,
        exit_code=exit_code,
        gate=gate,
        sha=sha,
        fingerprint=fingerprint,
        created_at=datetime.now().isoformat(timespec="seconds"),
        metrics={name: run_metrics[name] for name in row.metrics if name in run_metrics},
        monitor_metrics={name: run_metrics[name] for name in row.monitor if name in run_metrics},
        bands={c.band.metric: {"lo": c.band.lo, "hi": c.band.hi, "better": c.band.better.value} for c in comparisons},
        result_path=str(run_dir / BENCHMARK_RESULT_FILE),
        refusal=refusal,
        ratchet_command=ratchet_hint if verdict == Verdict.IMPROVED.value else None,
        replay_expect=replay_expect,
    )
    verdict_path = Path(args.verdict_file) if args.verdict_file else run_dir / "verdict.json"
    write_verdict(verdict_path, verdict_doc)
    print(f"Gate verdict {verdict} ({row.id}) written to {verdict_path}")
    return 0 if args.monitor else exit_code


def benchmark_calibrate(args: argparse.Namespace) -> int:
    """Handler for ``rosotacom benchmark calibrate`` — mint one row's bands.

    A row-aware ``ratchet --recalibrate``: bands only for the row's *gated*
    metrics, keyed by the row id, plus a calibration report carrying the
    measured per-metric spread — monitor metrics included, so the tighten-or-
    monitor decision (performance bands reference) is made from committed evidence.
    """
    from .benched_set import find_row
    from .benchmark import _sample_stdev, metrics_from_result, result_fingerprint, result_window_s

    row = find_row(_load_gate_registry(args), args.row_id)
    if row.kind != "performance":
        raise ValueError(f"row {row.id!r} is a {row.kind} row; boundary rows are not calibrated by budgets.jsonl")
    note = args.note or f"runner-class calibration of benched row {row.id}"
    # One ratchet per gated metric so each takes its committed floor (the
    # registry's `floors` are the reviewed per-metric width parameters).
    for metric in row.metrics:
        ratchet_args = argparse.Namespace(
            results=list(args.results),
            budgets=args.budgets,
            row=row.id,
            profile=row.profile,
            metric=[metric],
            note=note,
            recalibrate=True,
            better=None,
            k=args.k,
            floor=row.floors.get(metric, args.floor),
            floor_frac=args.floor_frac,
        )
        exit_code = benchmark_ratchet(ratchet_args)
        if exit_code != 0:
            return exit_code

    docs = _load_result_docs(args.results)
    per_run = [metrics_from_result(doc) for doc in docs]
    shared = sorted(set.intersection(*[set(metrics) for metrics in per_run]))
    spread = {}
    for metric in shared:
        values = [metrics[metric] for metrics in per_run]
        spread[metric] = {
            "values": values,
            "median": statistics.median(values),
            "sigma": _sample_stdev(values),
            "banded": metric in row.metrics,
        }
    report = {
        "schema": 1,
        "kind": "benchmark-calibration-report",
        "row": row.id,
        "profile": row.profile,
        "rmw": row.rmw,
        "fingerprint": result_fingerprint(docs[0]),
        "window_s": result_window_s(docs[0]),
        "repeats": len(docs),
        "k": args.k,
        "floor": args.floor,
        "floor_frac": args.floor_frac,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "metrics": spread,
    }
    if args.report:
        report_path = Path(args.report)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"Calibration report written to {report_path}")
    for metric, stats in spread.items():
        role = "banded" if stats["banded"] else "monitor"
        print(f"  {metric}: median={stats['median']:g} sigma={stats['sigma']:g} ({role}, {len(docs)} repeats)")
    return 0


def benchmark_gate_summary(args: argparse.Namespace) -> int:
    """Handler for ``rosotacom benchmark gate-summary`` — aggregate row verdicts.

    Reads every per-row verdict JSON in ``--verdicts``, checks it against the
    registry's rows for the lane, and writes the one summary document the
    operator harness promotion gate consumes. A missing or non-``WITHIN`` gated
    row makes the summary red and the exit code 1.
    """
    from .benched_set import rows_for_lane, summarize_verdicts
    from .benchmark import runner_fingerprint

    rows = rows_for_lane(_load_gate_registry(args), args.lane)
    verdicts: dict[str, dict[str, Any]] = {}
    verdicts_dir = Path(args.verdicts)
    for path in sorted(verdicts_dir.rglob("*.json")):
        try:
            doc = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            continue
        if isinstance(doc, dict) and doc.get("kind") == "benchmark-gate-verdict":
            verdicts[str(doc.get("row"))] = doc

    summary = summarize_verdicts(
        rows,
        verdicts,
        run={
            "sha": os.environ.get("GITHUB_SHA") or _current_sha(),
            "run_id": os.environ.get("GITHUB_RUN_ID"),
            "run_attempt": os.environ.get("GITHUB_RUN_ATTEMPT"),
            "fingerprint": runner_fingerprint(),
        },
    )
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    for row_doc in summary["rows"]:
        print(f"  {row_doc.get('row')}: {row_doc.get('verdict')}")
    print(f"Gate summary ({summary['overall']}) written to {out_path}")
    if summary["overall"] != "green":
        red = ", ".join(summary["red_rows"])
        print(f"RED: {red} — a red nightly gate outranks feature work (performance bands reference)")
        return 1
    return 0


def benchmark_ramp(args: argparse.Namespace) -> int:
    """Handler for ``rosotacom benchmark ramp``."""
    from .cli import _load_runtime_config

    if getattr(args, "interactive", False):
        return _start_interactive_benchmark(args, "ramp")

    runtime = _load_runtime_config(args)
    artifacts_dir = Path(args.artifacts_dir) if getattr(args, "artifacts_dir", None) else runtime.benchmarks_dir

    run_dir = _setup_benchmark_run_dir(args, "ramp", args.profile)
    session_context = _benchmark_session_context(args, "bench_1_3_ramp", run_dir)
    result_context = _benchmark_result_context(
        args,
        genre="ramp",
        profile=args.profile,
        run_dir=run_dir,
        session=session_context,
    )

    run_point = getattr(args, "_test_run_point", None) or _make_live_run_point(args, "bench_1_3_ramp")

    values = _parse_values(args.values)
    with log_stdout_stderr_to_file(run_dir / "stdout.txt"):
        drive_ramp(
            run_point,
            profile=args.profile,
            values=values,
            knob=getattr(args, "knob", "size"),
            rate_hz=getattr(args, "rate_hz", 20.0),
            topic=getattr(args, "topic", ""),
            duration_s=getattr(args, "duration", 60.0),
            out_dir=run_dir,
            result_context=result_context,
        )

    # Resolve aggregated file output path
    out_file_name = getattr(args, "out", "curve.jsonl")
    if artifacts_dir:
        out_path = artifacts_dir / Path(out_file_name).name
    else:
        out_path = Path(out_file_name)

    run_curve_file = run_dir / "curve.jsonl"
    if run_curve_file.is_file():
        out_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(run_curve_file, out_path)
        print(f"Ramp curve copied to {out_path}")

    return 0


def benchmark_recovery(args: argparse.Namespace) -> int:
    """Handler for ``rosotacom benchmark recovery``."""
    from .cli import _load_runtime_config

    if getattr(args, "interactive", False):
        return _start_interactive_benchmark(args, "recovery")

    runtime = _load_runtime_config(args)
    artifacts_dir = Path(args.artifacts_dir) if getattr(args, "artifacts_dir", None) else runtime.benchmarks_dir

    run_dir = _setup_benchmark_run_dir(args, "recovery", args.profile)
    session_context = _benchmark_session_context(args, "bench_1_4_recovery", run_dir)
    profiles_file = _benchmark_profiles_file(args.profile, runtime.profiles_file)
    result_context = _benchmark_result_context(
        args,
        genre="recovery",
        profile=args.profile,
        run_dir=run_dir,
        session=session_context,
    )

    run_point = getattr(args, "_test_run_point", None) or _make_live_run_point(args, "bench_1_4_recovery")

    with log_stdout_stderr_to_file(run_dir / "stdout.txt"):
        drive_recovery(
            run_point,
            profile=args.profile,
            duration_s=getattr(args, "duration", 90.0),
            nominal_period_s=getattr(args, "nominal_period", 0.05),
            latched_topics=getattr(args, "latched_topics", "").split(",")
            if getattr(args, "latched_topics", "")
            else (),
            out_dir=run_dir,
            profiles_file=profiles_file,
            result_context=result_context,
        )

    # Resolve aggregated file output path
    out_file_name = getattr(args, "out", "recovery.json")
    if artifacts_dir:
        out_path = artifacts_dir / Path(out_file_name).name
    else:
        out_path = Path(out_file_name)

    run_rec_file = run_dir / "recovery.json"
    if run_rec_file.is_file():
        out_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(run_rec_file, out_path)
        print(f"Recovery metrics copied to {out_path}")

    return 0


def benchmark_sweep(args: argparse.Namespace) -> int:
    """Handler for ``rosotacom benchmark sweep``."""
    from .cli import _load_runtime_config

    if getattr(args, "interactive", False):
        return _start_interactive_benchmark(args, "sweep")

    runtime = _load_runtime_config(args)
    artifacts_dir = Path(args.artifacts_dir) if getattr(args, "artifacts_dir", None) else runtime.benchmarks_dir

    run_dir = _setup_benchmark_run_dir(args, "sweep", None)
    session_context = _benchmark_session_context(args, "bench_1_2_load_sweep", run_dir)
    result_context = _benchmark_result_context(
        args,
        genre="sweep",
        profile=None,
        run_dir=run_dir,
        session=session_context,
    )

    run_point = getattr(args, "_test_run_point", None) or _make_live_run_point(args, "bench_1_2_load_sweep")

    profile_grid = [p.strip() for p in args.profile_grid.split(",") if p.strip()]
    with log_stdout_stderr_to_file(run_dir / "stdout.txt"):
        drive_sweep(
            run_point,
            profile_grid=profile_grid,
            max_loss_pct=getattr(args, "max_loss", 5.0),
            max_latency_ms=getattr(args, "max_latency_ms", 300.0),
            rate_hz=getattr(args, "rate_hz", 20.0),
            size=getattr(args, "size", 60000),
            topic=getattr(args, "topic", ""),
            duration_s=getattr(args, "duration", 60.0),
            out_dir=run_dir,
            result_context=result_context,
        )

    # Resolve aggregated file output path
    out_file_name = getattr(args, "out", "frontier.jsonl")
    if artifacts_dir:
        out_path = artifacts_dir / Path(out_file_name).name
    else:
        out_path = Path(out_file_name)

    run_frontier_file = run_dir / "frontier.jsonl"
    if run_frontier_file.is_file():
        out_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(run_frontier_file, out_path)
        print(f"Sweep frontier copied to {out_path}")

    return 0


def _parse_ab_candidates(raw: Sequence[str]) -> list[tuple[str, str]]:
    """Parse ``--candidate label=path`` values into ``(label, path)`` pairs."""
    parsed: list[tuple[str, str]] = []
    for item in raw:
        if "=" not in item:
            raise ValueError(f"--candidate must be label=config, got {item!r}.")
        label, path = item.split("=", 1)
        label = label.strip()
        if not label or not path.strip():
            raise ValueError(f"--candidate must be label=config, got {item!r}.")
        parsed.append((label, path.strip()))
    return parsed


def _resolve_ab_config_path(raw: str) -> Path:
    """A config is a session directory or a session-definition.yaml; return the YAML."""
    path = Path(raw).expanduser()
    if path.is_dir():
        candidate = path / "session-definition.yaml"
        if not candidate.is_file():
            raise FileNotFoundError(f"A/B config directory {path} has no session-definition.yaml.")
        return candidate
    if path.is_file():
        return path
    raise FileNotFoundError(f"A/B config {raw!r} is neither a session directory nor a session YAML.")


def _prepare_ab_config(
    args: argparse.Namespace,
    *,
    label: str,
    source_yaml: Path,
    is_baseline: bool,
    run_dir: Path,
) -> dict[str, Any]:
    """Materialize one A/B config as an ``AB_SESSION_NAME`` session under ``run_dir``.

    Every config is a whole session-definition.yaml (no patch format). The
    run-wide knobs (``shared.rmw`` and any ``--qos-*`` overrides) are pinned
    identically on all configs so the only thing that varies between runs is what
    the user actually changed in the file.
    """
    cfg = yaml.safe_load(source_yaml.read_text(encoding="utf-8")) or {}
    if not isinstance(cfg, dict):
        raise RuntimeError(f"A/B config {source_yaml} must be a YAML mapping.")
    topics = cfg.get("topics")
    a_to_b = topics.get("a_to_b") if isinstance(topics, dict) else None
    if not a_to_b:
        raise ValueError(
            f"A/B config {source_yaml} defines no topics.a_to_b stream. The synthetic load "
            "publishes on a_to_b, so every config must carry the same a_to_b load topic(s) and "
            "differ only in the pipeline knobs under test (throttle_hz, drop, QoS, compression, ...)."
        )
    shared = cfg.setdefault("shared", {})
    if not isinstance(shared, dict):
        raise RuntimeError(f"A/B config {source_yaml} 'shared' must be a mapping.")
    rmw = _benchmark_rmw(args)
    cyclone_spdp_interval = _benchmark_cyclone_spdp_interval(args)
    if cyclone_spdp_interval is not None:
        if rmw != "cyclone":
            raise ValueError("--cyclone-spdp-interval can only be used with --rmw cyclone.")
        shared["rmw"] = {"local": "cyclone", "ota": {"cyclone": {"spdp_interval": cyclone_spdp_interval}}}
    else:
        shared["rmw"] = rmw
    _apply_benchmark_qos_options(
        cfg,
        reliability=getattr(args, "qos_reliability", None),
        depth=getattr(args, "qos_depth", None),
    )

    configs_root = run_dir / "configs" / _safe_case_token(label)
    session_dir = configs_root / AB_SESSION_NAME
    session_dir.mkdir(parents=True, exist_ok=True)
    config_path = session_dir / "session-definition.yaml"
    config_path.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
    return {
        "label": label,
        "is_baseline": is_baseline,
        "source": source_yaml,
        "configs_root": configs_root,
        "config_path": config_path,
        "cfg": cfg,
    }


def _write_ab_config_diffs(configs: Sequence[dict[str, Any]], run_dir: Path) -> dict[str, str]:
    """Unified diff of each candidate config against the baseline (self-describing)."""
    import difflib

    baseline = next(config for config in configs if config["is_baseline"])
    baseline_text = yaml.safe_dump(baseline["cfg"], sort_keys=True).splitlines(keepends=True)
    diffs: dict[str, str] = {}
    for config in configs:
        if config["is_baseline"]:
            continue
        candidate_text = yaml.safe_dump(config["cfg"], sort_keys=True).splitlines(keepends=True)
        diff = "".join(
            difflib.unified_diff(
                baseline_text,
                candidate_text,
                fromfile=f"{baseline['label']}/session-definition.yaml",
                tofile=f"{config['label']}/session-definition.yaml",
            )
        )
        diff_name = f"{_safe_case_token(str(config['label']))}.diff"
        (run_dir / "configs" / diff_name).write_text(diff, encoding="utf-8")
        diffs[str(config["label"])] = f"configs/{diff_name}"
    return diffs


def benchmark_ab(args: argparse.Namespace) -> int:
    """Handler for ``rosotacom benchmark ab`` — candidate configs vs a baseline."""
    from .benchmark import AB_METRICS, DEFAULT_AB_METRICS, AbTolerance, default_ab_tolerance
    from .cli import _load_runtime_config

    runtime = _load_runtime_config(args)
    artifacts_dir = Path(args.artifacts_dir) if getattr(args, "artifacts_dir", None) else runtime.benchmarks_dir
    profile = str(args.profile)
    repeats = int(getattr(args, "repeats", 3))
    if repeats < 1:
        raise ValueError("--repeats must be >= 1.")

    baseline_label = "baseline"
    candidate_specs = _parse_ab_candidates(getattr(args, "candidate", None) or [])
    if not candidate_specs:
        raise ValueError("benchmark ab needs at least one --candidate label=config.")
    labels = [baseline_label, *(label for label, _ in candidate_specs)]
    if len(set(labels)) != len(labels):
        raise ValueError("A/B config labels must be unique and none may be 'baseline'.")

    metrics_arg = getattr(args, "metrics", None)
    metrics = [m.strip() for m in metrics_arg.split(",") if m.strip()] if metrics_arg else list(DEFAULT_AB_METRICS)
    unknown = [metric for metric in metrics if metric not in AB_METRICS]
    if unknown:
        raise ValueError(f"unknown A/B metric(s) {unknown}; known: {sorted(AB_METRICS)}.")
    rel = getattr(args, "rel_tolerance", None)
    abs_override = getattr(args, "abs_tolerance", None)
    tolerances: dict[str, AbTolerance] = {}
    for metric in metrics:
        base = default_ab_tolerance(metric) if rel is None else default_ab_tolerance(metric, rel=rel)
        tolerances[metric] = base if abs_override is None else AbTolerance(rel=base.rel, abs=float(abs_override))

    run_dir = _setup_benchmark_run_dir(args, "ab", profile)
    configs: list[dict[str, Any]] = [
        _prepare_ab_config(
            args,
            label=baseline_label,
            source_yaml=_resolve_ab_config_path(args.baseline),
            is_baseline=True,
            run_dir=run_dir,
        )
    ]
    for label, raw in candidate_specs:
        configs.append(
            _prepare_ab_config(
                args,
                label=label,
                source_yaml=_resolve_ab_config_path(raw),
                is_baseline=False,
                run_dir=run_dir,
            )
        )
    config_diffs = _write_ab_config_diffs(configs, run_dir)

    result_context = _benchmark_result_context(args, genre="ab", profile=profile, run_dir=run_dir, session=None)
    result_context["ab"] = {
        "baseline": baseline_label,
        "configs": [{"label": config["label"], "source": str(config["source"])} for config in configs],
    }

    load = _probe_load(
        size=int(getattr(args, "size", 18_000)),
        size_pattern=getattr(args, "size_pattern", None),
        rate_hz=float(getattr(args, "rate_hz", 20.0)),
        streams=int(getattr(args, "streams", 1)),
        interval_jitter_ms=float(getattr(args, "interval_jitter_ms", 0.0)),
        interval_jitter_seed=int(getattr(args, "interval_jitter_seed", 42)),
    )

    raw_run_point = getattr(args, "_test_run_point", None) or _make_live_run_point(args, AB_SESSION_NAME)

    def run_point_with_config(
        *,
        profile: str | None,
        load: dict[str, Any],
        duration_s: float,
        out_dir: Path,
        config: dict[str, Any],
    ) -> dict[str, Any]:
        previous = getattr(args, "session_configs_dir", None)
        if previous is None:
            existing: list[Any] = []
        elif isinstance(previous, list | tuple):
            existing = list(previous)
        else:
            existing = [previous]
        args.session_configs_dir = [str(config["configs_root"]), *[str(path) for path in existing]]
        try:
            return raw_run_point(profile=profile, load=load, duration_s=duration_s, out_dir=out_dir)
        finally:
            args.session_configs_dir = previous

    with log_stdout_stderr_to_file(run_dir / "stdout.txt"):
        drive_ab(
            run_point_with_config,
            configs=configs,
            baseline_label=baseline_label,
            profile=profile,
            load=load,
            duration_s=float(getattr(args, "duration", 20.0)),
            repeats=repeats,
            metrics=metrics,
            tolerances=tolerances,
            topic=getattr(args, "topic", ""),
            out_dir=run_dir,
            result_context=result_context,
            config_diffs=config_diffs,
        )

    out_file_name = getattr(args, "out", "ab.jsonl")
    out_path = artifacts_dir / Path(out_file_name).name if artifacts_dir else Path(out_file_name)
    run_ab_file = run_dir / "ab.jsonl"
    if run_ab_file.is_file():
        out_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(run_ab_file, out_path)
        print(f"A/B rows copied to {out_path}")

    passed = json.loads((run_dir / BENCHMARK_RESULT_FILE).read_text(encoding="utf-8")).get("verdict", {}).get("passed")
    if not passed and getattr(args, "fail_on_regression", False):
        return 1
    return 0


def benchmark_sensitivity(args: argparse.Namespace) -> int:
    """Handler for ``rosotacom benchmark sensitivity``."""
    from .cli import _load_runtime_config

    if getattr(args, "interactive", False):
        return _start_interactive_benchmark(args, "sensitivity")

    runtime = _load_runtime_config(args)
    artifacts_dir = Path(args.artifacts_dir) if getattr(args, "artifacts_dir", None) else runtime.benchmarks_dir
    base_profile = str(args.profile)

    run_dir = _setup_benchmark_run_dir(args, "sensitivity", base_profile)
    source_profiles_file = _benchmark_profiles_file(base_profile, runtime.profiles_file)
    if source_profiles_file is None:
        raise FileNotFoundError("sensitivity requires a profiles file containing the base profile.")

    generated_doc, cases = _build_sensitivity_profiles(
        base_profile=base_profile,
        profiles_file=source_profiles_file,
        ideal_rate=getattr(args, "ideal_rate", "1gbit"),
        loss_values=_parse_values(getattr(args, "loss_values", "0,0.5,1,3,5")),
        delay_values=_parse_values(getattr(args, "delay_values", "0,60,120,180")),
        jitter_values=_parse_values(getattr(args, "jitter_values", "0,10,30,50,80")),
        rate_values=_parse_rate_values(getattr(args, "rate_values", "1gbit,100mbit,10mbit,1mbit")),
        correlation_values=_parse_values(getattr(args, "correlation_values", "0,25,50,75")),
        axes=_parse_sensitivity_axes(getattr(args, "axes", "all")),
    )
    generated_profiles_file = run_dir / "generated-profiles.yaml"
    generated_profiles_file.write_text(yaml.safe_dump(generated_doc, sort_keys=False), encoding="utf-8")
    args.profiles_file = str(generated_profiles_file)

    session_name = BENCHMARK_SESSIONS_BY_GENRE["sensitivity"]
    session_context = _benchmark_session_context(args, session_name, run_dir)
    result_context = _benchmark_result_context(
        args,
        genre="sensitivity",
        profile=base_profile,
        run_dir=run_dir,
        session=session_context,
    )
    result_context["sensitivity"] = {
        "source_profiles_file": str(source_profiles_file),
        "generated_profiles_file": str(generated_profiles_file),
        "base_profile": base_profile,
    }

    run_point = getattr(args, "_test_run_point", None) or _make_live_run_point(args, session_name)
    with log_stdout_stderr_to_file(run_dir / "stdout.txt"):
        drive_sensitivity(
            run_point,
            cases=cases,
            max_loss_pct=getattr(args, "max_loss", 5.0),
            max_latency_ms=getattr(args, "max_latency_ms", 300.0),
            rate_hz=getattr(args, "rate_hz", 20.0),
            size=getattr(args, "size", 1),
            topic=getattr(args, "topic", ""),
            duration_s=getattr(args, "duration", 60.0),
            out_dir=run_dir,
            result_context=result_context,
            generated_profiles_file=generated_profiles_file,
        )

    out_file_name = getattr(args, "out", "sensitivity.jsonl")
    if artifacts_dir:
        out_path = artifacts_dir / Path(out_file_name).name
    else:
        out_path = Path(out_file_name)

    run_sensitivity_file = run_dir / "sensitivity.jsonl"
    if run_sensitivity_file.is_file():
        out_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(run_sensitivity_file, out_path)
        print(f"Sensitivity rows copied to {out_path}")

    return 0


def benchmark_matrix(args: argparse.Namespace) -> int:
    """Handler for ``rosotacom benchmark matrix``."""
    from .cli import _load_runtime_config

    if getattr(args, "interactive", False):
        return _start_interactive_benchmark(args, "matrix")

    runtime = _load_runtime_config(args)
    artifacts_dir = Path(args.artifacts_dir) if getattr(args, "artifacts_dir", None) else runtime.benchmarks_dir
    base_profile = str(args.profile)

    run_dir = _setup_benchmark_run_dir(args, "matrix", base_profile)
    source_profiles_file = _benchmark_profiles_file(base_profile, runtime.profiles_file)
    if source_profiles_file is None:
        raise FileNotFoundError("matrix requires a profiles file containing the base profile.")

    generated_doc, cases = _build_matrix_profiles(
        base_profile=base_profile,
        profiles_file=source_profiles_file,
        ideal_rate=getattr(args, "ideal_rate", "1gbit"),
        jitter_ms=float(getattr(args, "jitter_ms", 30.0)),
        latency_values=_parse_values(getattr(args, "latency_values", "30,60,100,140,180,220")),
        rate_hz_values=_parse_values(getattr(args, "rate_hz_values", "20,15,10,5,1")),
        qos_cases=_parse_qos_cases(getattr(args, "qos_cases", "best_effort:1,reliable:1,best_effort:10,reliable:10")),
        axes=_parse_matrix_axes(getattr(args, "axes", "all")),
        size=int(getattr(args, "size", 1)),
        fixed_rate_hz=float(getattr(args, "rate_hz", 20.0)),
        min_duration_s=float(getattr(args, "min_duration", 20.0)),
        min_messages=int(getattr(args, "min_messages", 100)),
    )
    generated_profiles_file = run_dir / "generated-profiles.yaml"
    generated_profiles_file.write_text(yaml.safe_dump(generated_doc, sort_keys=False), encoding="utf-8")
    args.profiles_file = str(generated_profiles_file)

    session_name = BENCHMARK_SESSIONS_BY_GENRE["matrix"]
    session_context = _benchmark_session_context(args, session_name, run_dir)
    result_context = _benchmark_result_context(
        args,
        genre="matrix",
        profile=base_profile,
        run_dir=run_dir,
        session=session_context,
    )
    result_context["matrix"] = {
        "source_profiles_file": str(source_profiles_file),
        "generated_profiles_file": str(generated_profiles_file),
        "base_profile": base_profile,
    }

    raw_run_point = getattr(args, "_test_run_point", None) or _make_live_run_point(args, session_name)

    def run_point_with_session_options(
        *,
        profile: str | None,
        load: dict[str, Any],
        duration_s: float,
        out_dir: Path,
        session_options: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        previous = _activate_benchmark_session_options(args, session_name, out_dir, session_options)
        try:
            return raw_run_point(profile=profile, load=load, duration_s=duration_s, out_dir=out_dir)
        finally:
            if session_options:
                _restore_benchmark_session_options(args, previous)

    with log_stdout_stderr_to_file(run_dir / "stdout.txt"):
        drive_matrix(
            run_point_with_session_options,
            cases=cases,
            max_loss_pct=getattr(args, "max_loss", 5.0),
            max_latency_ms=getattr(args, "max_latency_ms", 300.0),
            topic=getattr(args, "topic", ""),
            out_dir=run_dir,
            result_context=result_context,
            generated_profiles_file=generated_profiles_file,
        )

    out_file_name = getattr(args, "out", "matrix.jsonl")
    if artifacts_dir:
        out_path = artifacts_dir / Path(out_file_name).name
    else:
        out_path = Path(out_file_name)

    run_matrix_file = run_dir / "matrix.jsonl"
    if run_matrix_file.is_file():
        out_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(run_matrix_file, out_path)
        print(f"Matrix rows copied to {out_path}")

    return 0


def _session_contract_topics(cfg: dict[str, Any]) -> list[str]:
    """Every topic a session carries, across all directions — the required set."""
    names: list[str] = []
    topics = cfg.get("topics")
    if isinstance(topics, dict):
        for entries in topics.values():
            if not isinstance(entries, list):
                continue
            for item in entries:
                name = item.get("topic") if isinstance(item, dict) else None
                if isinstance(name, str) and name not in names:
                    names.append(name)
    return names


@dataclass
class _BenchmarkReplay:
    """Bag-as-load inputs: which session runs, the required topics, ground truth."""

    session_name: str
    required_topics: list[str] | None
    ground_truth: dict[str, dict[str, Any]] | None
    target: dict[str, Any] | None
    bag: dict[str, Any] | None
    lab_replay: bool


def _benchmark_replay_inputs(
    args: argparse.Namespace, runtime: Any, *, genre_session_name: str
) -> _BenchmarkReplay | None:
    """Resolve a ``--target``/``--bag`` replay load, or ``None`` for synthetic load.

    A session ``--target`` supplies the required contract topics (both directions)
    and, when not an OTA run, switches the live probe to run that session's own
    publishers in the lab (Docker) — a probe point is then one full replay under
    the candidate profile. ``--bag`` supplies per-topic completeness ground truth.
    """
    target = getattr(args, "target", None)
    target_type = getattr(args, "target_type", None) or "auto"
    bag_arg = getattr(args, "bag", None)
    if not target and not bag_arg:
        return None

    from .cli import _effective_session_config, _resolve_session

    is_ota = _is_ota_benchmark(args)
    ground_truth: dict[str, dict[str, Any]] | None = None
    bag_identity: dict[str, Any] | None = None
    if bag_arg:
        from .bag_ground_truth import bag_ground_truth

        ground_truth = bag_ground_truth(bag_arg)
        bag_identity = {"metadata": str(Path(bag_arg)), "topics": sorted(ground_truth)}

    required_topics: list[str] | None = None
    target_identity: dict[str, Any] | None = None
    lab_replay = False
    session_name = genre_session_name
    if target:
        target_identity = {"name": str(target), "type": target_type}
        session_cfg: dict[str, Any] | None = None
        if target_type in ("session", "auto"):
            try:
                session = _resolve_session(str(target), runtime)
                session_cfg = _effective_session_config(session.host_dir, runtime)
            except Exception:
                session_cfg = None
        if session_cfg is not None:
            required_topics = _session_contract_topics(session_cfg) or None
            target_identity["type"] = "session"
            if not is_ota:
                lab_replay = True
                session_name = str(target)
                args._benchmark_replay_target = str(target)
        elif not is_ota:
            raise ValueError(
                f"--target {target!r} did not resolve to a session for a local replay benchmark. "
                "Local (Docker) replay supports two-peer session targets; use ota-benchmark "
                "for scenarios or real hosts."
            )
    elif ground_truth is not None:
        required_topics = sorted(ground_truth)

    return _BenchmarkReplay(
        session_name=session_name,
        required_topics=required_topics,
        ground_truth=ground_truth,
        target=target_identity,
        bag=bag_identity,
        lab_replay=lab_replay,
    )


def benchmark_requirements(args: argparse.Namespace) -> int:
    """Handler for ``rosotacom benchmark requirements``."""
    from .cli import _load_runtime_config
    from .network_profiles import parse_rate_bps, parse_seed

    if getattr(args, "interactive", False):
        return _start_interactive_benchmark(args, "requirements")

    runtime = _load_runtime_config(args)
    artifacts_dir = Path(args.artifacts_dir) if getattr(args, "artifacts_dir", None) else runtime.benchmarks_dir
    run_dir = _setup_benchmark_run_dir(args, "requirements", None)
    generated_profiles_file = run_dir / "generated-profiles.yaml"
    generated_profiles_file.write_text(
        yaml.safe_dump(
            {
                "profiles": {},
                "metadata": {
                    "kind": "benchmark-requirements-generated",
                    "status": "initializing",
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    args.profiles_file = str(generated_profiles_file)
    replay = _benchmark_replay_inputs(args, runtime, genre_session_name=BENCHMARK_SESSIONS_BY_GENRE["requirements"])
    if replay is None:
        # Synthetic load pins the benchmark QoS; a replay keeps its session's own.
        args.qos_reliability = getattr(args, "qos_reliability", None) or "best_effort"
        args.qos_depth = int(getattr(args, "qos_depth", None) or 1)
        session_name = BENCHMARK_SESSIONS_BY_GENRE["requirements"]
    else:
        session_name = replay.session_name

    session_context = _benchmark_session_context(args, session_name, run_dir)
    result_context = _benchmark_result_context(
        args,
        genre="requirements",
        profile=None,
        run_dir=run_dir,
        session=session_context,
    )
    result_context["requirements"] = {
        "generated_profiles_file": str(generated_profiles_file),
    }
    if replay is not None:
        result_context["requirements"]["replay_target"] = replay.target
        result_context["requirements"]["bag"] = replay.bag

    rate_hz = float(getattr(args, "rate_hz", 20.0))
    size = int(getattr(args, "size", 18_000))
    streams = int(getattr(args, "streams", 1))
    offered_bw = (size * 8.0 * rate_hz * streams) if size > 0 and rate_hz > 0 else 1.0
    bandwidth_high_raw = str(getattr(args, "bandwidth_high", "auto"))
    if bandwidth_high_raw.lower() == "auto":
        bandwidth_high_bps = max(1.0, offered_bw * float(getattr(args, "bandwidth_high_factor", 8.0)))
    else:
        bandwidth_high_bps = parse_rate_bps(bandwidth_high_raw)
    bandwidth_low_raw = getattr(args, "bandwidth_low", None)
    if bandwidth_low_raw:
        bandwidth_low_bps = parse_rate_bps(bandwidth_low_raw)
    else:
        bandwidth_low_bps = max(1.0, offered_bw * float(getattr(args, "bandwidth_low_factor", 1.0)))
    max_latency_ms = float(getattr(args, "max_latency_ms", 250.0))
    latency_base_ms = float(getattr(args, "latency_base_ms", 0.0))
    latency_high_raw = getattr(args, "latency_high_ms", None)
    latency_high_ms = min(float(latency_high_raw), max_latency_ms) if latency_high_raw is not None else max_latency_ms
    netem_seed_raw = getattr(args, "netem_seed", None)
    netem_seed = parse_seed(netem_seed_raw, "netem_seed") if netem_seed_raw is not None else None
    jitter_guard_ratio = float(getattr(args, "jitter_guard_ratio", 0.0))
    bandwidth_guard_ratio = float(getattr(args, "bandwidth_guard_ratio", 0.0))

    run_point = getattr(args, "_test_run_point", None) or _make_live_run_point(args, session_name)
    with log_stdout_stderr_to_file(run_dir / "stdout.txt"):
        drive_requirements(
            run_point,
            max_loss_pct=float(getattr(args, "max_loss", 5.0)),
            max_latency_ms=max_latency_ms,
            rate_hz=rate_hz,
            size=size,
            streams=streams,
            qos_reliability=str(getattr(args, "qos_reliability", None) or "best_effort"),
            qos_depth=int(getattr(args, "qos_depth", None) or 1),
            topic=getattr(args, "topic", ""),
            out_dir=run_dir,
            bandwidth_high_bps=bandwidth_high_bps,
            bandwidth_low_bps=bandwidth_low_bps,
            latency_base_ms=latency_base_ms,
            latency_high_ms=latency_high_ms,
            jitter_high_ms=float(getattr(args, "jitter_high_ms", 120.0)),
            loss_high_pct=float(getattr(args, "loss_high", 20.0)),
            axes=_parse_requirements_axes(getattr(args, "axes", "all")),
            min_duration_s=float(getattr(args, "min_duration", 20.0)),
            min_messages=int(getattr(args, "min_messages", 100)),
            search_iterations=int(getattr(args, "search_iterations", 6)),
            search_rounds=int(getattr(args, "search_rounds", 1)),
            distribution=str(getattr(args, "distribution", "normal")),
            final_refine_iterations=int(getattr(args, "final_refine_iterations", 3)),
            loss_coupling=str(getattr(args, "loss_coupling", "jitter")),
            downlink_ratio=float(getattr(args, "downlink_ratio", 1.0)),
            downlink_mode=str(getattr(args, "downlink_mode", "mirror")),
            probe_repeats=int(getattr(args, "probe_repeats", 1)),
            probe_min_passes=getattr(args, "probe_min_passes", None),
            bad_lossy_count=getattr(args, "bad_lossy_count", None),
            bandwidth_probe_repeats=getattr(args, "bandwidth_probe_repeats", None),
            search_order=str(getattr(args, "search_order", "auto")),
            netem_seed=netem_seed,
            jitter_guard_ratio=jitter_guard_ratio,
            bandwidth_guard_ratio=bandwidth_guard_ratio,
            result_context=result_context,
            generated_profiles_file=generated_profiles_file,
            profile_prefix=str(getattr(args, "profile_prefix", "requirements")),
            ground_truth=replay.ground_truth if replay else None,
            required_topics=replay.required_topics if replay else None,
            target=replay.target if replay else None,
            bag=replay.bag if replay else None,
            oracle_min_completeness=float(getattr(args, "oracle_min_completeness", 0.95)),
        )

    out_file_name = getattr(args, "out", "requirements.jsonl")
    if artifacts_dir:
        out_path = artifacts_dir / Path(out_file_name).name
    else:
        out_path = Path(out_file_name)

    run_requirements_file = run_dir / "requirements.jsonl"
    if run_requirements_file.is_file():
        out_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(run_requirements_file, out_path)
        print(f"Requirements rows copied to {out_path}")

    return 0


def benchmark_loss_boundaries(args: argparse.Namespace) -> int:
    """Handler for ``rosotacom benchmark loss-boundaries``."""
    from .cli import _load_runtime_config
    from .network_profiles import parse_rate_bps, parse_seed

    if getattr(args, "interactive", False):
        return _start_interactive_benchmark(args, "loss-boundaries")

    runtime = _load_runtime_config(args)
    artifacts_dir = Path(args.artifacts_dir) if getattr(args, "artifacts_dir", None) else runtime.benchmarks_dir
    run_dir = _setup_benchmark_run_dir(args, "loss-boundaries", None)
    generated_profiles_file = run_dir / "generated-profiles.yaml"
    generated_profiles_file.write_text(
        yaml.safe_dump(
            {
                "profiles": {},
                "metadata": {
                    "kind": "benchmark-loss-boundaries-generated",
                    "status": "initializing",
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    args.profiles_file = str(generated_profiles_file)
    replay = _benchmark_replay_inputs(args, runtime, genre_session_name=BENCHMARK_SESSIONS_BY_GENRE["requirements"])
    if replay is None:
        args.qos_reliability = getattr(args, "qos_reliability", None) or "best_effort"
        args.qos_depth = int(getattr(args, "qos_depth", None) or 1)
        session_name = BENCHMARK_SESSIONS_BY_GENRE["requirements"]
    else:
        session_name = replay.session_name

    session_context = _benchmark_session_context(args, session_name, run_dir)
    result_context = _benchmark_result_context(
        args,
        genre="loss-boundaries",
        profile=None,
        run_dir=run_dir,
        session=session_context,
    )
    result_context["loss_boundaries"] = {
        "generated_profiles_file": str(generated_profiles_file),
    }
    if replay is not None:
        result_context["loss_boundaries"]["replay_target"] = replay.target
        result_context["loss_boundaries"]["bag"] = replay.bag

    rate_hz = float(getattr(args, "rate_hz", 20.0))
    size = int(getattr(args, "size", 18_000))
    streams = int(getattr(args, "streams", 1))
    offered_bw = (size * 8.0 * rate_hz * streams) if size > 0 and rate_hz > 0 else 1.0
    bandwidth_high_raw = str(getattr(args, "bandwidth_high", "auto"))
    if bandwidth_high_raw.lower() == "auto":
        bandwidth_high_bps = max(1.0, offered_bw * float(getattr(args, "bandwidth_high_factor", 8.0)))
    else:
        bandwidth_high_bps = parse_rate_bps(bandwidth_high_raw)
    bandwidth_low_raw = getattr(args, "bandwidth_low", None)
    if bandwidth_low_raw:
        bandwidth_low_bps = parse_rate_bps(bandwidth_low_raw)
    else:
        bandwidth_low_bps = max(1.0, offered_bw * float(getattr(args, "bandwidth_low_factor", 1.0)))
    bandwidth_step_bps = parse_rate_bps(str(getattr(args, "bandwidth_step", "0.1mbit")))
    netem_seed_raw = getattr(args, "netem_seed", None)
    netem_seed = parse_seed(netem_seed_raw, "netem_seed") if netem_seed_raw is not None else None
    netem_seeds = _parse_netem_seed_values(getattr(args, "netem_seeds", None))

    run_point = getattr(args, "_test_run_point", None) or _make_live_run_point(args, session_name)
    with log_stdout_stderr_to_file(run_dir / "stdout.txt"):
        drive_loss_boundaries(
            run_point,
            max_latency_ms=float(getattr(args, "max_latency_ms", 250.0)),
            rate_hz=rate_hz,
            size=size,
            streams=streams,
            qos_reliability=str(getattr(args, "qos_reliability", None) or "best_effort"),
            qos_depth=int(getattr(args, "qos_depth", None) or 1),
            topic=getattr(args, "topic", ""),
            out_dir=run_dir,
            axes=_parse_loss_boundary_axes(getattr(args, "axes", "all")),
            bandwidth_low_bps=bandwidth_low_bps,
            bandwidth_high_bps=bandwidth_high_bps,
            bandwidth_step_bps=bandwidth_step_bps,
            latency_base_ms=float(getattr(args, "latency_base_ms", 30.0)),
            jitter_low_ms=float(getattr(args, "jitter_low_ms", 0.0)),
            jitter_high_ms=float(getattr(args, "jitter_high_ms", 40.0)),
            jitter_step_ms=float(getattr(args, "jitter_step_ms", 1.0)),
            min_duration_s=float(getattr(args, "min_duration", 20.0)),
            min_messages=int(getattr(args, "min_messages", 100)),
            distribution=str(getattr(args, "distribution", "normal")),
            downlink_ratio=float(getattr(args, "downlink_ratio", 1.0)),
            downlink_mode=str(getattr(args, "downlink_mode", "lan")),
            probe_repeats=int(getattr(args, "probe_repeats", 10)),
            good_clean_count=getattr(args, "good_clean_count", None),
            bad_lossy_count=getattr(args, "bad_lossy_count", None),
            netem_seed=netem_seed,
            netem_seeds=netem_seeds,
            result_context=result_context,
            generated_profiles_file=generated_profiles_file,
            profile_prefix=str(getattr(args, "profile_prefix", "loss_boundary")),
            ground_truth=replay.ground_truth if replay else None,
            required_topics=replay.required_topics if replay else None,
            target=replay.target if replay else None,
            bag=replay.bag if replay else None,
            oracle_min_completeness=float(getattr(args, "oracle_min_completeness", 0.95)),
        )

    out_file_name = getattr(args, "out", "loss-boundaries.json")
    if artifacts_dir:
        out_path = artifacts_dir / Path(out_file_name).name
    else:
        out_path = Path(out_file_name)
    run_boundaries_file = run_dir / "loss-boundaries.json"
    if run_boundaries_file.is_file():
        out_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(run_boundaries_file, out_path)
        print(f"Loss boundaries copied to {out_path}")

    return 0


def benchmark_plot(args: argparse.Namespace) -> int:
    """Handler for ``rosotacom benchmark plot``."""
    from .cli import _load_runtime_config
    from .plots import (
        plot_capacity_frontier,
        plot_offered_bw,
        plot_probe_raw,
        plot_probe_timeseries,
        plot_ramp,
        plot_recovery_timeline,
        plot_topic_heatmap,
    )

    runtime = _load_runtime_config(args)
    artifacts_dir = Path(args.artifacts_dir) if getattr(args, "artifacts_dir", None) else runtime.benchmarks_dir
    input_path_raw = getattr(args, "input", None)

    if not input_path_raw and not artifacts_dir:
        print("Error: either 'input' or '--artifacts-dir' must be specified.", file=sys.stderr)
        return 1

    inputs_to_plot: list[Path] = []
    if input_path_raw:
        inputs_to_plot.append(Path(input_path_raw))
    elif artifacts_dir:
        # Search for standard files. Exclude budgets.jsonl since it has no plot format
        for name in ("time-bins.jsonl", "curve.jsonl", "recovery.json", "frontier.jsonl"):
            candidate = artifacts_dir / name
            if candidate.is_file():
                inputs_to_plot.append(candidate)

    if not inputs_to_plot:
        print("No input files to plot found.", file=sys.stderr)
        return 0

    for input_path in inputs_to_plot:
        if not input_path.exists():
            print(f"Input file not found: {input_path}", file=sys.stderr)
            continue

        out = Path(args.out) if getattr(args, "out", None) else input_path.with_suffix(".png")
        plot_type = getattr(args, "type", "auto")

        try:
            data = _load_jsonl(input_path)
            if plot_type == "auto":
                plot_type = _infer_plot_type(data, input_path.name)

            if plot_type == "frontier":
                plot_capacity_frontier(data, out=out)
            elif plot_type == "probe":
                plot_probe_timeseries(data, out=out)
                try:
                    raw_records = _load_raw_records_from_out(input_path.parent)
                    if raw_records:
                        raw_out = (
                            Path(args.out).parent / "probe-raw.png"
                            if getattr(args, "out", None)
                            else input_path.parent / "probe-raw.png"
                        )
                        if getattr(args, "out", None) and Path(args.out) == raw_out:
                            raw_out = Path(args.out).parent / "probe-raw.png"
                        plot_probe_raw(raw_records, out=raw_out)
                        print(f"Probe raw plot saved to {raw_out}")
                except Exception:
                    pass
            elif plot_type == "probe-raw":
                raw_out = Path(args.out) if getattr(args, "out", None) else input_path.parent / "probe-raw.png"
                raw_records = _load_raw_records_from_out(input_path.parent)
                plot_probe_raw(raw_records, out=raw_out)
                print(f"Probe raw plot saved to {raw_out}")

            elif plot_type == "offered_bw":
                plot_offered_bw(data, out=out)
            elif plot_type == "ramp":
                plot_ramp(data, out=out)
            elif plot_type == "recovery":
                outage_start = float(data[0].get("outage", {}).get("start", 0)) if data else 0.0
                outage_end = float(data[0].get("outage", {}).get("end", 0)) if data else 0.0
                records = _load_raw_records_from_out(input_path.parent)
                plot_recovery_timeline(records, outage_start, outage_end, out=out)
            elif plot_type == "heatmap":
                per_topic = {row.get("topic", ""): row for row in data if "topic" in row}
                plot_topic_heatmap(per_topic, out=out)
            else:
                print(
                    f"Unknown plot type: {plot_type!r}. Use: frontier, probe, "
                    "probe-raw, offered_bw, ramp, recovery, heatmap.",
                    file=sys.stderr,
                )
                return 1

            print(f"Plot written to {out}")
        except Exception as exc:
            print(f"Failed to plot {input_path.name}: {exc}", file=sys.stderr)

    return 0


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    """Load a JSON-lines file into a list of dicts."""
    text_stripped = path.read_text(encoding="utf-8").strip()
    if text_stripped.startswith("["):
        return json.loads(text_stripped)  # type: ignore[no-any-return]
    try:
        return [json.loads(text_stripped)]
    except json.JSONDecodeError:
        data: list[dict[str, Any]] = []
        for line in text_stripped.splitlines():
            if line.strip():
                data.append(json.loads(line))
        return data


def _infer_plot_type(data: list[dict[str, Any]], filename: str) -> str:
    """Best-effort inference of the plot type from the data shape and filename."""
    if "frontier" in filename:
        return "frontier"
    if "time-bins" in filename or "probe" in filename:
        return "probe"
    if "curve" in filename:
        return "ramp"
    if "recovery" in filename:
        return "recovery"
    if not data:
        return "frontier"
    first = data[0]
    if "genre" in first and "metrics" in first:
        raise ValueError("Budget files (e.g. budgets.jsonl) contain target thresholds and cannot be plotted directly.")
    if "capacity" in first or "bandwidth_bps" in first:
        return "frontier"
    if "bin_start_s" in first and "delivered_hz" in first:
        return "probe"
    if "value" in first and "metric" in first:
        return "ramp"
    if "t_recover" in first:
        return "recovery"
    if "offered_bw_bps" in first:
        return "offered_bw"
    return "frontier"


# --------------------------------------------------------------------------- #
# Argparse registration (called from cli.py)
# --------------------------------------------------------------------------- #


def _register_benchmark_driver_parsers(benchmark_subparsers: Any, *, ota_benchmark: bool) -> None:
    # --- probe ---
    probe_parser = benchmark_subparsers.add_parser(
        "probe",
        help="Run one fixed load under a profile and write time-binned latency/loss/Hz/bandwidth metrics.",
    )
    _add_benchmark_common_args(probe_parser, ota_benchmark=ota_benchmark)
    probe_parser.add_argument("--profile", required=True, help="Network profile name.")
    probe_parser.add_argument("--size", type=int, default=18_000, help="Payload size (bytes).")
    probe_parser.add_argument(
        "--size-pattern",
        default=None,
        help="Cyclic payload sizes such as 1x20KB+1x0KB; overrides --size when set.",
    )
    probe_parser.add_argument("--rate-hz", type=float, default=20.0, help="Publish rate (Hz).")
    probe_parser.add_argument("--streams", type=int, default=1, help="Parallel stream count.")
    probe_parser.add_argument("--topic", default="", help="Topic to characterize (default: all).")
    probe_parser.add_argument(
        "--interval-jitter-ms",
        type=float,
        default=0.0,
        help="Standard deviation of interval jitter in ms.",
    )
    probe_parser.add_argument(
        "--interval-jitter-seed",
        type=int,
        default=42,
        help="Random seed for interval jitter.",
    )
    probe_parser.add_argument("--duration", type=float, default=60.0, help="Seconds per probe attempt.")
    probe_parser.add_argument("--repeats", type=int, default=1, help="Repeat the same fixed probe this many times.")
    probe_parser.add_argument("--bin-s", type=float, default=1.0, help="Time-bin width in seconds.")
    probe_parser.add_argument("--out", default="time-bins.jsonl", help="Output time-bin JSONL file.")
    probe_plot = probe_parser.add_mutually_exclusive_group()
    probe_plot.add_argument("--plot", dest="plot", action="store_true", help="Render probe-timeseries.png.")
    probe_plot.add_argument("--no-plot", dest="plot", action="store_false", help="Skip plot rendering.")
    probe_parser.set_defaults(func=benchmark_probe, plot=True)

    # --- capacity ---
    cap_parser = benchmark_subparsers.add_parser(
        "capacity",
        help="Binary-search for the capacity breakpoint under a profile.",
    )
    _add_benchmark_common_args(cap_parser, ota_benchmark=ota_benchmark)
    cap_parser.add_argument("--profile", required=True, help="Network profile name.")
    cap_parser.add_argument("--knob", required=True, choices=["size", "rate", "bandwidth"], help="Parameter to sweep.")
    cap_parser.add_argument("--low", type=int, required=True, help="Lower bound for the sweep.")
    cap_parser.add_argument("--high", type=int, required=True, help="Upper bound for the sweep.")
    cap_parser.add_argument("--max-loss", type=float, required=True, help="Oracle: max acceptable loss %%.")
    cap_parser.add_argument(
        "--max-latency-ms", type=float, required=True, help="Oracle: max acceptable p95 latency (ms)."
    )
    cap_parser.add_argument("--repeats", type=int, default=1, help="Repeats per probe point (median vote).")
    cap_parser.add_argument("--rate-hz", type=float, default=20.0, help="Publish rate (Hz).")
    cap_parser.add_argument("--topic", default="", help="Topic to evaluate (default: all).")
    cap_parser.add_argument("--duration", type=float, default=60.0, help="Seconds per probe point.")
    cap_parser.set_defaults(func=benchmark_capacity)

    # --- ramp ---
    ramp_parser = benchmark_subparsers.add_parser(
        "ramp",
        help="Measure latency over a linear load ramp (monitor-only trend).",
    )
    _add_benchmark_common_args(ramp_parser, ota_benchmark=ota_benchmark)
    ramp_parser.add_argument("--profile", required=True, help="Network profile name.")
    ramp_parser.add_argument(
        "--values", required=True, help="Comma-separated values, start:stop:step, or start..stop/count."
    )
    ramp_parser.add_argument("--knob", default="size", help="Parameter to ramp (default: size).")
    ramp_parser.add_argument("--rate-hz", type=float, default=20.0, help="Publish rate (Hz).")
    ramp_parser.add_argument("--topic", default="", help="Topic to evaluate.")
    ramp_parser.add_argument("--duration", type=float, default=60.0, help="Seconds per ramp point.")
    ramp_parser.add_argument("--out", default="curve.jsonl", help="Output curve JSONL file.")
    ramp_parser.set_defaults(func=benchmark_ramp)

    # --- recovery ---
    rec_parser = benchmark_subparsers.add_parser(
        "recovery",
        help="Run a recovery test under a timeline profile.",
    )
    _add_benchmark_common_args(rec_parser, ota_benchmark=ota_benchmark)
    rec_parser.add_argument("--profile", required=True, help="Timeline profile name (must have an outage segment).")
    rec_parser.add_argument("--duration", type=float, default=90.0, help="Total run duration (s).")
    rec_parser.add_argument("--nominal-period", type=float, default=0.05, help="Nominal publish period (s).")
    rec_parser.add_argument("--latched-topics", default="", help="Comma-separated latched topic names.")
    rec_parser.add_argument("--out", default="recovery.json", help="Output recovery metrics file.")
    rec_parser.set_defaults(func=benchmark_recovery)

    # --- sweep ---
    sweep_parser = benchmark_subparsers.add_parser(
        "sweep",
        help="Grid sweep over profiles; oracle pass/fail for each (frontier).",
    )
    _add_benchmark_common_args(sweep_parser, ota_benchmark=ota_benchmark)
    sweep_parser.add_argument("--profile-grid", required=True, help="Comma-separated list of profile names to sweep.")
    sweep_parser.add_argument("--max-loss", type=float, default=5.0, help="Oracle: max loss %%.")
    sweep_parser.add_argument("--max-latency-ms", type=float, default=300.0, help="Oracle: max p95 latency (ms).")
    sweep_parser.add_argument("--rate-hz", type=float, default=20.0, help="Publish rate (Hz).")
    sweep_parser.add_argument("--size", type=int, default=60000, help="Payload size (bytes).")
    sweep_parser.add_argument("--topic", default="", help="Topic to evaluate.")
    sweep_parser.add_argument("--duration", type=float, default=60.0, help="Seconds per sweep point.")
    sweep_parser.add_argument("--out", default="frontier.jsonl", help="Output frontier JSONL file.")
    sweep_parser.set_defaults(func=benchmark_sweep)

    # --- sensitivity ---
    sensitivity_parser = benchmark_subparsers.add_parser(
        "sensitivity",
        help="Run a generated profile lab to isolate loss, delay, jitter, rate, and loss-correlation effects.",
    )
    _add_benchmark_common_args(sensitivity_parser, ota_benchmark=ota_benchmark)
    sensitivity_parser.add_argument("--profile", required=True, help="Static base profile to investigate.")
    sensitivity_parser.add_argument("--max-loss", type=float, default=5.0, help="Oracle: max loss %%.")
    sensitivity_parser.add_argument("--max-latency-ms", type=float, default=300.0, help="Oracle: max p95 latency (ms).")
    sensitivity_parser.add_argument("--rate-hz", type=float, default=20.0, help="Publish rate (Hz).")
    sensitivity_parser.add_argument("--size", type=int, default=1, help="Payload size (bytes).")
    sensitivity_parser.add_argument("--topic", default="", help="Topic to evaluate.")
    sensitivity_parser.add_argument("--duration", type=float, default=60.0, help="Seconds per sensitivity point.")
    sensitivity_parser.add_argument("--ideal-rate", default="1gbit", help="High-quality shaped-link rate.")
    sensitivity_parser.add_argument(
        "--loss-values",
        default="0,0.5,1,3,5",
        help="Comma/range list of loss percentages for the loss-only axis.",
    )
    sensitivity_parser.add_argument(
        "--delay-values",
        default="0,60,120,180",
        help="Comma/range list of fixed delays in milliseconds for the delay-only axis.",
    )
    sensitivity_parser.add_argument(
        "--jitter-values",
        default="0,10,30,50,80",
        help="Comma/range list of jitter values in milliseconds around the base mean delay.",
    )
    sensitivity_parser.add_argument(
        "--rate-values",
        default="1gbit,100mbit,10mbit,1mbit",
        help="Comma list of rate tokens for the rate-only axis.",
    )
    sensitivity_parser.add_argument(
        "--correlation-values",
        default="0,25,50,75",
        help="Comma/range list of loss-correlation percentages at the base loss rate.",
    )
    sensitivity_parser.add_argument(
        "--axes",
        default="all",
        help="Comma list of sensitivity axes to run: all, loss, delay, jitter, rate, loss-correlation.",
    )
    sensitivity_parser.add_argument("--out", default="sensitivity.jsonl", help="Output sensitivity JSONL file.")
    sensitivity_parser.set_defaults(func=benchmark_sensitivity)

    # --- ab ---
    ab_parser = benchmark_subparsers.add_parser(
        "ab",
        help="A/B tuning: compare candidate session configs against a baseline on the same load and profile.",
    )
    _add_benchmark_common_args(ab_parser, ota_benchmark=ota_benchmark)
    ab_parser.add_argument("--profile", required=True, help="Network profile, held identical across every config.")
    ab_parser.add_argument(
        "--baseline",
        required=True,
        help="Baseline session config: a session directory or a session-definition.yaml.",
    )
    ab_parser.add_argument(
        "--candidate",
        action="append",
        metavar="LABEL=CONFIG",
        help="Candidate config as label=path (session dir or session-definition.yaml). Repeatable.",
    )
    ab_parser.add_argument("--repeats", type=int, default=3, help="Repeats per config (spread + median vote).")
    ab_parser.add_argument("--size", type=int, default=18_000, help="Synthetic load payload size (bytes).")
    ab_parser.add_argument(
        "--size-pattern",
        default=None,
        help="Cyclic payload sizes such as 1x20KB+1x0KB; overrides --size when set.",
    )
    ab_parser.add_argument("--rate-hz", type=float, default=20.0, help="Synthetic load publish rate (Hz).")
    ab_parser.add_argument("--streams", type=int, default=1, help="Parallel stream count.")
    ab_parser.add_argument("--interval-jitter-ms", type=float, default=0.0, help="Std dev of interval jitter (ms).")
    ab_parser.add_argument("--interval-jitter-seed", type=int, default=42, help="Seed for interval jitter.")
    ab_parser.add_argument("--duration", type=float, default=20.0, help="Seconds per run.")
    ab_parser.add_argument("--topic", default="", help="Restrict the verdict to one topic (default: all).")
    ab_parser.add_argument(
        "--metrics",
        default=None,
        help="Comma list of watched metrics (default: completeness_pct,loss_pct,latency_p95_ms,jitter_p95_ms).",
    )
    ab_parser.add_argument(
        "--rel-tolerance",
        type=float,
        default=None,
        help="Relative half-width of the unchanged band around the baseline median (default: 0.10).",
    )
    ab_parser.add_argument(
        "--abs-tolerance",
        type=float,
        default=None,
        help="Override the absolute half-width floor (metric units) for every watched metric.",
    )
    ab_parser.add_argument(
        "--fail-on-regression",
        action="store_true",
        help="Exit non-zero when any candidate regressed (default: verdict in result.json, exit 0).",
    )
    ab_parser.add_argument("--out", default="ab.jsonl", help="Output per-run JSONL file.")
    ab_parser.set_defaults(func=benchmark_ab)

    # --- matrix ---
    matrix_parser = benchmark_subparsers.add_parser(
        "matrix",
        help="Run a fixed-jitter matrix over latency, publish frequency, and QoS settings.",
    )
    _add_benchmark_common_args(matrix_parser, ota_benchmark=ota_benchmark)
    matrix_parser.add_argument("--profile", required=True, help="Static base profile whose latency ratio is reused.")
    matrix_parser.add_argument("--max-loss", type=float, default=5.0, help="Oracle: max loss %%.")
    matrix_parser.add_argument("--max-latency-ms", type=float, default=300.0, help="Oracle: max p95 latency (ms).")
    matrix_parser.add_argument("--rate-hz", type=float, default=20.0, help="Fixed publish rate for non-Hz axes.")
    matrix_parser.add_argument("--size", type=int, default=1, help="Payload size (bytes).")
    matrix_parser.add_argument("--topic", default="", help="Topic to evaluate.")
    matrix_parser.add_argument("--ideal-rate", default="1gbit", help="High-quality shaped-link rate.")
    matrix_parser.add_argument("--jitter-ms", type=float, default=30.0, help="Fixed jitter in milliseconds.")
    matrix_parser.add_argument(
        "--latency-values",
        default="30,60,100,140,180,220",
        help="Comma/range list of uplink mean delays in milliseconds; downlink preserves the base profile ratio.",
    )
    matrix_parser.add_argument(
        "--rate-hz-values",
        default="20,15,10,5,1",
        help="Comma/range list of publish frequencies for the Hz axis.",
    )
    matrix_parser.add_argument(
        "--qos-cases",
        default="best_effort:1,reliable:1,best_effort:10,reliable:10",
        help="Comma list of QoS cases as reliability:depth.",
    )
    matrix_parser.add_argument(
        "--axes",
        default="all",
        help="Comma list of matrix axes to run: all, latency, hz, qos.",
    )
    matrix_parser.add_argument(
        "--min-duration",
        type=float,
        default=20.0,
        help="Minimum seconds per matrix point.",
    )
    matrix_parser.add_argument(
        "--min-messages",
        type=int,
        default=100,
        help="Minimum intended published messages per matrix point.",
    )
    matrix_parser.add_argument("--out", default="matrix.jsonl", help="Output matrix JSONL file.")
    matrix_parser.set_defaults(func=benchmark_matrix)

    # --- requirements ---
    requirements_parser = benchmark_subparsers.add_parser(
        "requirements",
        help="Search for a tight network profile that satisfies a ROS 2 stream quality target.",
    )
    _add_benchmark_common_args(requirements_parser, ota_benchmark=ota_benchmark)
    _add_benchmark_replay_args(requirements_parser)
    requirements_parser.add_argument("--max-loss", type=float, default=5.0, help="Oracle: max ROS 2 loss %%.")
    requirements_parser.add_argument(
        "--max-latency-ms", type=float, default=250.0, help="Oracle: max acceptable p95 latency (ms)."
    )
    requirements_parser.add_argument("--rate-hz", type=float, default=20.0, help="ROS 2 publish rate (Hz).")
    requirements_parser.add_argument("--size", type=int, default=18_000, help="Payload size (bytes).")
    requirements_parser.add_argument("--streams", type=int, default=1, help="Parallel stream count.")
    requirements_parser.add_argument("--topic", default="", help="Topic to evaluate.")
    requirements_parser.add_argument(
        "--bandwidth-high",
        default="auto",
        help="Known-good search bandwidth ceiling; auto uses offered bandwidth times --bandwidth-high-factor.",
    )
    requirements_parser.add_argument(
        "--bandwidth-high-factor",
        type=float,
        default=8.0,
        help="Auto bandwidth ceiling as a factor of offered stream bandwidth.",
    )
    requirements_parser.add_argument(
        "--bandwidth-low",
        help="Search bandwidth floor; default is offered bandwidth times --bandwidth-low-factor.",
    )
    requirements_parser.add_argument(
        "--bandwidth-low-factor",
        type=float,
        default=1.0,
        help="Default bandwidth floor as a factor of offered stream bandwidth.",
    )
    requirements_parser.add_argument(
        "--latency-base-ms",
        type=float,
        default=0.0,
        help="Initial/fixed uplink delay before searching worse latency values.",
    )
    requirements_parser.add_argument(
        "--latency-high-ms",
        type=float,
        help="Worst latency ceiling to try while searching; default is --max-latency-ms.",
    )
    requirements_parser.add_argument(
        "--jitter-high-ms",
        type=float,
        default=120.0,
        help="Worst jitter ceiling to try while searching.",
    )
    requirements_parser.add_argument(
        "--loss-high",
        type=float,
        default=20.0,
        help="Worst network loss percentage to try while searching.",
    )
    requirements_parser.add_argument(
        "--axes",
        default="all",
        help="Comma list of network axes to tighten: all, bandwidth, latency, jitter, loss.",
    )
    requirements_parser.add_argument(
        "--search-iterations",
        type=int,
        default=6,
        help="Binary-search probes per axis and round.",
    )
    requirements_parser.add_argument(
        "--search-rounds",
        type=int,
        default=1,
        help="Coordinate-search rounds; more rounds account for knob interactions better.",
    )
    requirements_parser.add_argument(
        "--final-refine-iterations",
        type=int,
        default=3,
        help="Extra bounded refinement probes per axis after the coordinate search.",
    )
    requirements_parser.add_argument(
        "--loss-coupling",
        choices=["jitter", "independent"],
        default="jitter",
        help=(
            "Use a practical jitter/loss tradeoff curve or search network loss as its own independent axis. "
            "For --max-loss 0, generated network loss is clamped to 0 and jitter is searched separately."
        ),
    )
    requirements_parser.add_argument(
        "--min-duration",
        type=float,
        default=20.0,
        help="Minimum seconds per probe.",
    )
    requirements_parser.add_argument(
        "--min-messages",
        type=int,
        default=100,
        help="Minimum intended published messages per probe.",
    )
    requirements_parser.add_argument(
        "--distribution",
        default="normal",
        choices=["normal", "pareto", "paretonormal"],
        help="netem jitter distribution.",
    )
    requirements_parser.add_argument(
        "--downlink-ratio",
        type=float,
        default=1.0,
        help="Downlink latency as a ratio of uplink latency when --downlink-mode=mirror.",
    )
    requirements_parser.add_argument(
        "--downlink-mode",
        choices=["mirror", "lan"],
        default="mirror",
        help=(
            "mirror shapes uplink and downlink alike; "
            "lan leaves downlink unshaped for mobile-upload/LAN-download tests."
        ),
    )
    requirements_parser.add_argument(
        "--probe-repeats",
        type=int,
        default=1,
        help="Repeat each candidate this many times before deciding PASS/FAIL.",
    )
    requirements_parser.add_argument(
        "--probe-min-passes",
        type=int,
        help="Required passing repeats; default is all repeats, or ceil(0.9 * repeats) for --max-loss 0.",
    )
    requirements_parser.add_argument(
        "--bad-lossy-count",
        type=int,
        help="Mark a candidate as a bad case when at least this many repeats lose messages; default is all repeats.",
    )
    requirements_parser.add_argument(
        "--bandwidth-probe-repeats",
        type=int,
        help=(
            "Override --probe-repeats for bandwidth bisection probes. "
            "Use this when bandwidth is latency/backlog limited and jitter/final probes still need strict repeats."
        ),
    )
    requirements_parser.add_argument(
        "--search-order",
        choices=["auto", "input"],
        default="auto",
        help="auto searches jitter before bandwidth/latency for exact zero-loss targets; input preserves axis order.",
    )
    requirements_parser.add_argument(
        "--netem-seed",
        type=int,
        help="Seed for generated netem qdiscs so jitter/loss draws can be replayed on hosts whose tc supports it.",
    )
    requirements_parser.add_argument(
        "--jitter-guard-ratio",
        type=float,
        default=0.0,
        help=(
            "After finding a zero-loss jitter boundary, use this fractional margin for later axes and the final "
            "good-case profile; e.g. 0.10 uses 10%% less jitter than the boundary."
        ),
    )
    requirements_parser.add_argument(
        "--bandwidth-guard-ratio",
        type=float,
        default=0.0,
        help=(
            "After finding a bandwidth boundary, use this fractional margin for the final good-case profile; "
            "e.g. 0.10 uses 10%% more bandwidth than the boundary."
        ),
    )
    requirements_parser.add_argument(
        "--profile-prefix",
        default="requirements",
        help="Prefix for generated profile names.",
    )
    requirements_parser.add_argument("--out", default="requirements.jsonl", help="Output requirements JSONL file.")
    requirements_parser.set_defaults(func=benchmark_requirements)

    # --- loss-boundaries ---
    loss_boundaries_parser = benchmark_subparsers.add_parser(
        "loss-boundaries",
        help="Find discrete zero-loss good/bad boundaries for bandwidth and jitter.",
    )
    _add_benchmark_common_args(loss_boundaries_parser, ota_benchmark=ota_benchmark)
    _add_benchmark_replay_args(loss_boundaries_parser)
    loss_boundaries_parser.add_argument(
        "--max-latency-ms",
        type=float,
        default=250.0,
        help="Maximum acceptable p95 latency while classifying a zero-loss good case.",
    )
    loss_boundaries_parser.add_argument("--rate-hz", type=float, default=20.0, help="ROS 2 publish rate (Hz).")
    loss_boundaries_parser.add_argument("--size", type=int, default=18_000, help="Payload size (bytes).")
    loss_boundaries_parser.add_argument("--streams", type=int, default=1, help="Parallel stream count.")
    loss_boundaries_parser.add_argument("--topic", default="", help="Topic to evaluate.")
    loss_boundaries_parser.add_argument(
        "--axes",
        default="all",
        help="Comma list of loss boundary axes to search: all, bandwidth, jitter.",
    )
    loss_boundaries_parser.add_argument(
        "--bandwidth-high",
        default="auto",
        help="Known-good bandwidth ceiling; auto uses offered bandwidth times --bandwidth-high-factor.",
    )
    loss_boundaries_parser.add_argument(
        "--bandwidth-high-factor",
        type=float,
        default=8.0,
        help="Auto bandwidth ceiling as a factor of offered stream bandwidth.",
    )
    loss_boundaries_parser.add_argument(
        "--bandwidth-low",
        help="Bandwidth floor; default is offered bandwidth times --bandwidth-low-factor.",
    )
    loss_boundaries_parser.add_argument(
        "--bandwidth-low-factor",
        type=float,
        default=1.0,
        help="Default bandwidth floor as a factor of offered stream bandwidth.",
    )
    loss_boundaries_parser.add_argument(
        "--bandwidth-step",
        default="0.1mbit",
        help="Discrete bandwidth tolerance/resolution.",
    )
    loss_boundaries_parser.add_argument(
        "--latency-base-ms",
        type=float,
        default=30.0,
        help="Fixed uplink delay used while searching bandwidth and jitter boundaries.",
    )
    loss_boundaries_parser.add_argument("--jitter-low-ms", type=float, default=0.0, help="Jitter search floor.")
    loss_boundaries_parser.add_argument("--jitter-high-ms", type=float, default=40.0, help="Jitter search ceiling.")
    loss_boundaries_parser.add_argument(
        "--jitter-step-ms",
        type=float,
        default=1.0,
        help="Discrete jitter tolerance/resolution.",
    )
    loss_boundaries_parser.add_argument(
        "--min-duration",
        type=float,
        default=20.0,
        help="Minimum seconds per probe.",
    )
    loss_boundaries_parser.add_argument(
        "--min-messages",
        type=int,
        default=100,
        help="Minimum intended published messages per probe.",
    )
    loss_boundaries_parser.add_argument(
        "--distribution",
        default="normal",
        choices=["normal", "pareto", "paretonormal"],
        help="netem jitter distribution.",
    )
    loss_boundaries_parser.add_argument(
        "--downlink-ratio",
        type=float,
        default=1.0,
        help="Downlink latency as a ratio of uplink latency when --downlink-mode=mirror.",
    )
    loss_boundaries_parser.add_argument(
        "--downlink-mode",
        choices=["mirror", "lan"],
        default="lan",
        help="lan leaves downlink unshaped for mobile-upload/LAN-download tests.",
    )
    loss_boundaries_parser.add_argument(
        "--probe-repeats",
        type=int,
        default=10,
        help="Seedless/fixed-seed samples per candidate, or samples per seed when --netem-seeds is used.",
    )
    loss_boundaries_parser.add_argument(
        "--good-clean-count",
        type=int,
        help="Samples that must be loss-free and below latency limit to classify a good case; default is all samples.",
    )
    loss_boundaries_parser.add_argument(
        "--bad-lossy-count",
        type=int,
        help="Samples that must lose messages to classify a bad case; default is all samples.",
    )
    loss_boundaries_parser.add_argument(
        "--netem-seed",
        type=int,
        help="Single netem seed for exact replay/debugging.",
    )
    loss_boundaries_parser.add_argument(
        "--netem-seeds",
        help="Comma list of netem seeds; each seed is probed for reproducible multi-seed boundaries.",
    )
    loss_boundaries_parser.add_argument(
        "--profile-prefix",
        default="loss_boundary",
        help="Prefix for generated profile names.",
    )
    loss_boundaries_parser.add_argument(
        "--out",
        default="loss-boundaries.json",
        help="Output loss boundary JSON file.",
    )
    loss_boundaries_parser.set_defaults(func=benchmark_loss_boundaries)


def _register_benchmark_plot_parser(benchmark_subparsers: Any) -> None:
    # --- plot ---
    plot_parser = benchmark_subparsers.add_parser(
        "plot",
        help="Render a benchmark figure from a results file.",
    )
    plot_parser.add_argument(
        "input",
        nargs="?",
        help="Input file (time-bins.jsonl, curve.jsonl, recovery.json, frontier.jsonl, etc.).",
    )
    plot_parser.add_argument("--out", help="Output figure path (default: <input>.png).")
    plot_parser.add_argument("--artifacts-dir", help="Output directory for all benchmark artifacts.")
    plot_parser.add_argument(
        "--type",
        choices=["auto", "frontier", "probe", "probe-raw", "offered_bw", "ramp", "recovery", "heatmap"],
        default="auto",
        help="Plot type (default: auto-detect from data).",
    )
    plot_parser.set_defaults(func=benchmark_plot)


def _register_benchmark_band_parsers(benchmark_subparsers: Any) -> None:
    """Register ``compare`` and ``ratchet`` — offline band operations (performance bands reference)."""
    from .benchmark import DEFAULT_FLOOR_FRAC, DEFAULT_WIDTH_K

    compare_parser = benchmark_subparsers.add_parser(
        "compare",
        help="Gate result.json run(s) against the committed two-sided bands.",
    )
    compare_parser.add_argument("results", nargs="+", help="result.json file(s) or benchmark run director(y/ies).")
    compare_parser.add_argument("--budgets", default="budgets.jsonl", help="Committed band store (JSONL).")
    compare_parser.add_argument("--row", default=None, help="Band row id (default: derived from the run).")
    compare_parser.add_argument("--profile", default=None, help="Band profile (default: from the run).")
    compare_parser.add_argument(
        "--monitor",
        action="store_true",
        help="Report verdicts without blocking: exit 0 even when out of band.",
    )
    compare_parser.set_defaults(func=benchmark_compare)

    delta_parser = benchmark_subparsers.add_parser(
        "delta",
        help="Compare two result.json sets taken in different environments (monitor-only).",
    )
    delta_parser.add_argument("reference", help="Directory or result.json for the reference environment.")
    delta_parser.add_argument("measured", help="Directory or result.json for the measured environment.")
    delta_parser.add_argument("--label-reference", default="emulated", help="Name for the reference side.")
    delta_parser.add_argument("--label-measured", default="ota", help="Name for the measured side.")
    delta_parser.add_argument("--out", default=None, help="Write the JSON report here.")
    delta_parser.add_argument("--markdown", default=None, help="Write the Markdown summary here.")
    delta_parser.set_defaults(func=benchmark_delta)

    ratchet_parser = benchmark_subparsers.add_parser(
        "ratchet",
        help="Rewrite committed bands from result.json run(s) — bands are never hand-edited.",
    )
    ratchet_parser.add_argument("results", nargs="+", help="result.json file(s) or benchmark run director(y/ies).")
    ratchet_parser.add_argument("--budgets", default="budgets.jsonl", help="Committed band store (JSONL).")
    ratchet_parser.add_argument("--row", default=None, help="Band row id (default: derived from the run).")
    ratchet_parser.add_argument("--profile", default=None, help="Band profile (default: from the run).")
    ratchet_parser.add_argument("--metric", action="append", help="Limit to these metrics (repeatable).")
    ratchet_parser.add_argument("--note", default="", help="One-line cause note recorded in the band.")
    ratchet_parser.add_argument(
        "--recalibrate",
        action="store_true",
        help="Recompute the width from these runs (K fresh repeats) — the only path that may widen a "
        "band, move it toward worse, or change its runner class.",
    )
    ratchet_parser.add_argument(
        "--better",
        choices=["higher", "lower"],
        default=None,
        help="Better-direction for newly calibrated metrics without a default.",
    )
    ratchet_parser.add_argument(
        "--k", type=float, default=DEFAULT_WIDTH_K, help="Width multiplier: half-width = max(k*sigma, floor)."
    )
    ratchet_parser.add_argument("--floor", type=float, default=0.0, help="Absolute minimum half-width.")
    ratchet_parser.add_argument(
        "--floor-frac",
        type=float,
        default=DEFAULT_FLOOR_FRAC,
        help="Minimum half-width as a fraction of the band center.",
    )
    ratchet_parser.set_defaults(func=benchmark_ratchet)


def _register_benchmark_gate_parsers(benchmark_subparsers: Any, *, ota_benchmark: bool) -> None:
    """Register the benched-set commands (performance bands reference): rows/row/calibrate/gate-summary."""
    from .benched_set import LANES
    from .benchmark import DEFAULT_FLOOR_FRAC, DEFAULT_WIDTH_K

    rows_parser = benchmark_subparsers.add_parser(
        "rows",
        help="List the benched-set registry — the curated rows the gate lanes run.",
    )
    rows_parser.add_argument("--registry", default=None, help="Registry file (default: the packaged benched set).")
    rows_parser.add_argument("--lane", choices=list(LANES), default=None, help="Only rows of this lane.")
    rows_parser.add_argument(
        "--calibratable",
        action="store_true",
        help="Only rows whose metrics mint committed bands through benchmark calibrate.",
    )
    rows_parser.add_argument(
        "--format",
        choices=["text", "ids", "json"],
        default="text",
        help="Output format; 'ids' is a JSON array for a workflow matrix.",
    )
    rows_parser.set_defaults(func=benchmark_rows)

    row_parser = benchmark_subparsers.add_parser(
        "row",
        help="Run one benched row end-to-end and gate it against the committed bands.",
    )
    _add_benchmark_common_args(row_parser, ota_benchmark=ota_benchmark)
    row_parser.add_argument("row_id", help="Registry row id (see 'benchmark rows'); pins rmw/profile/load/duration.")
    row_parser.add_argument("--registry", default=None, help="Registry file (default: the packaged benched set).")
    row_parser.add_argument("--budgets", default="budgets.jsonl", help="Committed band store (JSONL).")
    row_parser.add_argument(
        "--verdict-file",
        default=None,
        help="Where to write the machine-readable verdict JSON (default: <run-dir>/verdict.json).",
    )
    row_gate_mode = row_parser.add_mutually_exclusive_group()
    row_gate_mode.add_argument(
        "--monitor",
        action="store_true",
        help="Report the verdict (refusals included) without blocking: always exit 0.",
    )
    row_gate_mode.add_argument(
        "--no-compare",
        action="store_true",
        help="Run the row without gating it — calibration repeats use this.",
    )
    row_parser.set_defaults(func=benchmark_row)

    calibrate_parser = benchmark_subparsers.add_parser(
        "calibrate",
        help="Mint one benched row's bands from K fresh repeats on this runner class.",
    )
    calibrate_parser.add_argument("row_id", help="Registry row id whose gated metrics get bands.")
    calibrate_parser.add_argument("results", nargs="+", help="The K result.json file(s) or run director(y/ies).")
    calibrate_parser.add_argument("--registry", default=None, help="Registry file (default: the packaged benched set).")
    calibrate_parser.add_argument("--budgets", default="budgets.jsonl", help="Committed band store (JSONL).")
    calibrate_parser.add_argument(
        "--report",
        default=None,
        help="Also write a calibration report (per-metric values/median/sigma, monitor metrics included).",
    )
    calibrate_parser.add_argument("--note", default="", help="One-line cause note recorded in the bands.")
    calibrate_parser.add_argument(
        "--k", type=float, default=DEFAULT_WIDTH_K, help="Width multiplier: half-width = max(k*sigma, floor)."
    )
    calibrate_parser.add_argument("--floor", type=float, default=0.0, help="Absolute minimum half-width.")
    calibrate_parser.add_argument(
        "--floor-frac",
        type=float,
        default=DEFAULT_FLOOR_FRAC,
        help="Minimum half-width as a fraction of the band center.",
    )
    calibrate_parser.set_defaults(func=benchmark_calibrate)

    summary_parser = benchmark_subparsers.add_parser(
        "gate-summary",
        help="Aggregate per-row verdict JSONs into the one summary a promotion gate reads.",
    )
    summary_parser.add_argument("--verdicts", required=True, help="Directory holding the per-row verdict JSON files.")
    summary_parser.add_argument("--registry", default=None, help="Registry file (default: the packaged benched set).")
    summary_parser.add_argument("--lane", choices=list(LANES), default="nightly", help="Lane the summary covers.")
    summary_parser.add_argument("--out", required=True, help="Summary JSON output path.")
    summary_parser.set_defaults(func=benchmark_gate_summary)


def register_benchmark_parser(subparsers: Any) -> None:
    """Register the ``benchmark`` and ``ota-benchmark`` commands."""
    benchmark_parser = subparsers.add_parser(
        "benchmark",
        help="Run local benchmark tests and render plots.",
    )
    benchmark_subparsers = benchmark_parser.add_subparsers(dest="benchmark_command", required=True)
    _register_benchmark_driver_parsers(benchmark_subparsers, ota_benchmark=False)
    _register_benchmark_band_parsers(benchmark_subparsers)
    _register_benchmark_gate_parsers(benchmark_subparsers, ota_benchmark=False)
    _register_benchmark_plot_parser(benchmark_subparsers)

    ota_parser = subparsers.add_parser(
        "ota-benchmark",
        help="Run OTA benchmark tests against deployment peers.",
    )
    ota_subparsers = ota_parser.add_subparsers(dest="benchmark_command", required=True)
    _register_benchmark_driver_parsers(ota_subparsers, ota_benchmark=True)
    _register_benchmark_gate_parsers(ota_subparsers, ota_benchmark=True)
