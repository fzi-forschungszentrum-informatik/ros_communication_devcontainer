# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Geo-reference recorded link quality against GPS or pose samples.

The join is deliberately offline and explicit about clocks: metric samples keep
their source timestamp, the caller provides the seconds offset that converts that
source timestamp into the GPS/bag timestamp domain, and samples outside the
nearest-neighbor tolerance are dropped.
"""

from __future__ import annotations

import bisect
import csv
import html
import importlib
import json
import math
import statistics
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Final

import yaml
from PIL import Image, ImageDraw

from .anonymize import bag_storage_id, bag_topics_info, load_bag_metadata
from .transit import join_transit_records, load_transit_records

CSV_COLUMNS: Final[tuple[str, ...]] = (
    "schema_version",
    "source",
    "metric",
    "gps_time_s",
    "source_time_s",
    "aligned_time_s",
    "time_delta_s",
    "latitude",
    "longitude",
    "altitude_m",
    "metric_value",
    "observed_tx_kbps",
    "observed_rx_kbps",
    "rtt_ms",
    "loss_pct",
    "delivery_pct",
    "event_loss_pct",
    "event_expected",
    "event_delivered",
    "event_lost",
    "event_topic",
)
DEFAULT_METRIC = "observed_tx_kbps"
KNOWN_METRICS: Final[tuple[str, ...]] = (
    "observed_tx_kbps",
    "observed_rx_kbps",
    "rtt_ms",
    "loss_pct",
    "delivery_pct",
    "event_loss_pct",
    "ota_hop_ms",
)
HIGHER_IS_BETTER: Final[frozenset[str]] = frozenset({"observed_tx_kbps", "observed_rx_kbps", "delivery_pct"})
SCHEMA_VERSION = 1
EARTH_RADIUS_M = 6_378_137.0
MAP_WIDTH = 960
MAP_HEIGHT = 560
MAP_PAD = 40


@dataclass(frozen=True)
class GeoSample:
    """One GPS or projected local-pose sample."""

    time_s: float
    latitude: float
    longitude: float
    altitude_m: float | None = None


@dataclass(frozen=True)
class MetricSample:
    """One link-quality sample in its source time domain."""

    time_s: float
    source: str
    values: dict[str, float]
    context: dict[str, object]


@dataclass(frozen=True)
class GeoreferencedSample:
    """A metric sample joined to the nearest GPS point."""

    source: str
    metric: str
    metric_value: float
    gps: GeoSample
    source_time_s: float
    aligned_time_s: float
    time_delta_s: float
    values: dict[str, float]
    context: dict[str, object]


def load_gps_csv(path: str | Path) -> list[GeoSample]:
    """Load fixture/legacy GPS samples from CSV.

    Accepted column aliases:
    ``time_s``/``stamp_s``/``timestamp_s``/``bag_time_s``/``t``,
    ``latitude``/``lat``, ``longitude``/``lon``/``lng``, and optional
    ``altitude_m``/``altitude``/``alt``.
    """

    csv_path = Path(path)
    with csv_path.open(newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        if reader.fieldnames is None:
            raise ValueError(f"{csv_path}: missing CSV header")
        samples = [
            _gps_sample_from_csv_row(row, csv_path=csv_path, line_number=index) for index, row in enumerate(reader, 2)
        ]
    return _sorted_gps(samples, source=str(csv_path))


def load_gps_from_bag(
    bag: str | Path,
    *,
    topic: str,
    storage_id: str | None = None,
    time_source: str = "bag",
    origin_latitude: float | None = None,
    origin_longitude: float | None = None,
) -> list[GeoSample]:
    """Read GPS or pose samples from a rosbag2 bag.

    ``time_source`` is ``bag`` for the rosbag record timestamp or ``header`` for
    message ``header.stamp``. Local pose/odometry topics are projected to
    latitude/longitude from an explicit WGS84 origin, interpreting x=east and
    y=north meters.
    """

    if time_source not in {"bag", "header"}:
        raise ValueError("time_source must be 'bag' or 'header'")

    try:
        rosbag2_py = importlib.import_module("rosbag2_py")
        deserialize_message = importlib.import_module("rclpy.serialization").deserialize_message
        get_message = importlib.import_module("rosidl_runtime_py.utilities").get_message
    except ImportError as exc:
        raise RuntimeError(
            "reading GPS samples from a bag requires a ROS 2 Python environment with "
            "rosbag2_py, rclpy, and rosidl_runtime_py available; use --gps-csv for host-only fixtures"
        ) from exc

    bag_dir = _bag_directory(Path(bag))
    metadata = load_bag_metadata(bag_dir)
    topic_info = bag_topics_info(metadata)
    if topic not in topic_info:
        available = ", ".join(sorted(topic_info)) or "<none>"
        raise RuntimeError(f"topic {topic!r} not found in {bag_dir}; available topics: {available}")

    msg_type = str(topic_info[topic].get("type") or "")
    msg_cls = get_message(msg_type)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id or bag_storage_id(metadata)),
        rosbag2_py.ConverterOptions("", ""),
    )

    samples: list[GeoSample] = []
    origin = _origin_tuple(origin_latitude, origin_longitude)
    while reader.has_next():
        topic_name, payload, timestamp_ns = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(payload, msg_cls)
        time_s = float(timestamp_ns) / 1e9 if time_source == "bag" else _message_header_time_s(msg)
        if time_s is None:
            raise RuntimeError(f"{topic}: message has no usable header.stamp for --gps-time-source header")
        samples.append(_geo_from_message(msg, msg_type=msg_type, time_s=time_s, origin=origin))
    return _sorted_gps(samples, source=f"{bag_dir}:{topic}")


def load_link_trace_metrics(paths: Iterable[str | Path]) -> list[MetricSample]:
    """Load ``link_trace.jsonl`` rows as metric samples."""

    samples: list[MetricSample] = []
    for raw_path in paths:
        path = Path(raw_path)
        for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number}: invalid JSON: {exc}") from exc
            if not isinstance(row, Mapping) or row.get("kind") not in (None, "link_trace"):
                continue
            sample = _metric_from_link_trace_row(row, path=path, line_number=line_number)
            if sample is not None:
                samples.append(sample)
    return sorted(samples, key=lambda sample: sample.time_s)


def load_event_metrics(
    paths: Iterable[str | Path],
    *,
    bin_s: float = 1.0,
    time_field: str = "t_wrap",
    topic: str | None = None,
) -> list[MetricSample]:
    """Load binned per-topic delivery metrics from ``events.jsonl`` transit rows."""

    if bin_s <= 0:
        raise ValueError("bin_s must be > 0")
    if time_field not in {"t_wrap", "t_com_in"}:
        raise ValueError("time_field must be 't_wrap' or 't_com_in'")
    path_list = [Path(path) for path in paths]
    records = join_transit_records(load_transit_records(path_list))
    grouped: dict[str, list[dict[str, Any]]] = {}
    for record in records:
        label = _event_label(record)
        if topic is not None and topic not in {label, str(record.get("topic") or "")}:
            continue
        grouped.setdefault(label, []).append(record)

    samples: list[MetricSample] = []
    source = "events:" + ",".join(path.name for path in path_list)
    for label, stream in sorted(grouped.items()):
        timed = _stream_records_with_times(stream, time_field=time_field)
        bins: dict[float, list[tuple[dict[str, Any], float]]] = {}
        for record, stamp in timed:
            start = math.floor(stamp / bin_s) * bin_s
            bins.setdefault(start, []).append((record, stamp))
        for start, entries in sorted(bins.items()):
            expected = len(entries)
            lost = sum(record.get("status") == "lost" for record, _stamp in entries)
            delivered = expected - lost
            ota_values = [
                latency
                for record, _stamp in entries
                if (latency := _record_latency_ms(record)) is not None and record.get("status") != "lost"
            ]
            values = {
                "delivery_pct": round(100.0 * delivered / expected, 6) if expected else 0.0,
                "event_loss_pct": round(100.0 * lost / expected, 6) if expected else 0.0,
            }
            if ota_values:
                values["ota_hop_ms"] = round(statistics.median(ota_values), 6)
            samples.append(
                MetricSample(
                    time_s=start + bin_s / 2.0,
                    source=f"{source}:{label}",
                    values=values,
                    context={
                        "event_topic": label,
                        "event_expected": expected,
                        "event_delivered": delivered,
                        "event_lost": lost,
                    },
                )
            )
    return sorted(samples, key=lambda sample: sample.time_s)


def join_metric_samples(
    gps_samples: Sequence[GeoSample],
    metric_samples: Sequence[MetricSample],
    *,
    metric: str = DEFAULT_METRIC,
    trace_to_gps_offset_s: float = 0.0,
    max_gap_s: float = 1.0,
) -> list[GeoreferencedSample]:
    """Join metric samples to nearest GPS sample after applying the time offset."""

    if max_gap_s < 0:
        raise ValueError("max_gap_s must be >= 0")
    gps = _sorted_gps(gps_samples, source="gps samples")
    times = [sample.time_s for sample in gps]
    joined: list[GeoreferencedSample] = []
    for sample in sorted(metric_samples, key=lambda item: item.time_s):
        metric_value = sample.values.get(metric)
        if metric_value is None or not math.isfinite(metric_value):
            continue
        aligned = sample.time_s + trace_to_gps_offset_s
        nearest = _nearest_gps(gps, times, aligned)
        if nearest is None:
            continue
        delta = nearest.time_s - aligned
        if abs(delta) > max_gap_s:
            continue
        joined.append(
            GeoreferencedSample(
                source=sample.source,
                metric=metric,
                metric_value=metric_value,
                gps=nearest,
                source_time_s=sample.time_s,
                aligned_time_s=aligned,
                time_delta_s=delta,
                values=dict(sample.values),
                context=dict(sample.context),
            )
        )
    return joined


def write_geo_csv(samples: Sequence[GeoreferencedSample], path: str | Path) -> None:
    """Write the stable georeferenced sample CSV."""

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(CSV_COLUMNS))
        writer.writeheader()
        for sample in samples:
            writer.writerow(_csv_row(sample))


def render_geomap_html(*, route_image_src: str, metric: str, title: str | None = None) -> str:
    """Render the HTML report around a sibling route image."""

    metric_title = title or f"rosotacom geo link-quality map: {metric}"

    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{html.escape(metric_title)}</title>
  <style>
    :root {{
      color-scheme: light;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #f5f7f9;
      color: #16202a;
    }}
    body {{
      margin: 0;
      padding: 24px;
    }}
    main {{
      max-width: 1120px;
      margin: 0 auto;
    }}
    h1 {{
      font-size: 1.5rem;
      margin: 0 0 8px;
      letter-spacing: 0;
    }}
    .meta {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 8px;
      margin: 12px 0 18px;
      color: #425466;
      font-size: 0.92rem;
    }}
    .map {{
      background: #ffffff;
      border: 1px solid #d8e0e8;
      border-radius: 8px;
      overflow: hidden;
    }}
    .map img {{
      display: block;
      width: 100%;
      height: auto;
    }}
    .legend {{
      display: flex;
      align-items: center;
      gap: 10px;
      margin: 14px 0 22px;
      font-size: 0.92rem;
      color: #425466;
    }}
    .ramp {{
      width: 180px;
      height: 12px;
      border-radius: 999px;
      background: linear-gradient(90deg, hsl(0 72% 45%), hsl(60 72% 45%), hsl(120 72% 38%));
      border: 1px solid #c8d1da;
    }}
  </style>
</head>
<body>
<main>
  <h1>{html.escape(metric_title)}</h1>
  <div class="meta">
    <div><strong>route image</strong> {html.escape(route_image_src)}</div>
    <div><strong>coordinates</strong> written to CSV only</div>
  </div>
  <div class="map">
    <img src="{html.escape(route_image_src)}" alt="Route colored by {html.escape(metric)}">
  </div>
  <div class="legend">
    <span>lower quality</span>
    <span class="ramp" aria-hidden="true"></span>
    <span>higher quality</span>
    <span>{html.escape(metric)}</span>
  </div>
</main>
</body>
</html>
"""


def write_geomap_html(
    samples: Sequence[GeoreferencedSample],
    path: str | Path,
    *,
    metric: str,
    title: str | None = None,
) -> None:
    """Write an HTML route map and its sibling PNG route image."""

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    route_image = _route_image_path(output)
    _write_route_png(samples, route_image, metric=metric)
    rendered = render_geomap_html(
        route_image_src=route_image.name,
        metric=metric,
        title=title,
    )
    output.write_text(rendered, encoding="utf-8")


def load_metrics_for_inputs(
    *,
    traces: Sequence[str | Path] = (),
    events: Sequence[str | Path] = (),
    event_bin_s: float = 1.0,
    event_time_field: str = "t_wrap",
    event_topic: str | None = None,
) -> list[MetricSample]:
    """Load all configured metric inputs."""

    samples: list[MetricSample] = []
    if traces:
        samples.extend(load_link_trace_metrics(traces))
    if events:
        samples.extend(load_event_metrics(events, bin_s=event_bin_s, time_field=event_time_field, topic=event_topic))
    return sorted(samples, key=lambda sample: sample.time_s)


def _gps_sample_from_csv_row(row: Mapping[str, str | None], *, csv_path: Path, line_number: int) -> GeoSample:
    return GeoSample(
        time_s=_required_csv_float(row, ("time_s", "stamp_s", "timestamp_s", "bag_time_s", "t"), csv_path, line_number),
        latitude=_required_csv_float(row, ("latitude", "lat"), csv_path, line_number),
        longitude=_required_csv_float(row, ("longitude", "lon", "lng"), csv_path, line_number),
        altitude_m=_optional_csv_float(row, ("altitude_m", "altitude", "alt")),
    )


def _required_csv_float(
    row: Mapping[str, str | None], names: tuple[str, ...], csv_path: Path, line_number: int
) -> float:
    value = _optional_csv_float(row, names)
    if value is None:
        raise ValueError(f"{csv_path}:{line_number}: missing numeric column from {names}")
    return value


def _optional_csv_float(row: Mapping[str, str | None], names: tuple[str, ...]) -> float | None:
    for name in names:
        if name not in row:
            continue
        value = row[name]
        if value is None or value == "":
            return None
        return _required_number(value, name)
    return None


def _sorted_gps(samples: Sequence[GeoSample], *, source: str) -> list[GeoSample]:
    out = sorted(samples, key=lambda sample: sample.time_s)
    if not out:
        raise ValueError(f"{source}: no GPS samples found")
    for sample in out:
        if not (-90.0 <= sample.latitude <= 90.0 and -180.0 <= sample.longitude <= 180.0):
            raise ValueError(f"{source}: invalid latitude/longitude at {sample.time_s:g}s")
    return out


def _bag_directory(path: Path) -> Path:
    if path.name == "metadata.yaml":
        return path.parent
    if path.is_file():
        return path.parent
    return path


def _origin_tuple(latitude: float | None, longitude: float | None) -> tuple[float, float] | None:
    if latitude is None and longitude is None:
        return None
    if latitude is None or longitude is None:
        raise ValueError("origin latitude and longitude must be provided together")
    if not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
        raise ValueError("origin latitude/longitude are out of range")
    return latitude, longitude


def _message_header_time_s(msg: Any) -> float | None:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", getattr(stamp, "nsec", None))
    if sec is None or nanosec is None:
        return None
    return float(sec) + float(nanosec) / 1e9


def _geo_from_message(
    msg: Any,
    *,
    msg_type: str,
    time_s: float,
    origin: tuple[float, float] | None,
) -> GeoSample:
    latitude = _optional_number(getattr(msg, "latitude", None))
    longitude = _optional_number(getattr(msg, "longitude", None))
    altitude = _optional_number(getattr(msg, "altitude", None))
    if latitude is not None and longitude is not None:
        return GeoSample(time_s=time_s, latitude=latitude, longitude=longitude, altitude_m=altitude)

    position = _message_position(msg)
    if position is None:
        raise RuntimeError(f"{msg_type}: unsupported GPS/pose message shape")
    if origin is None:
        raise RuntimeError(
            f"{msg_type}: local pose/odometry topics require --origin-lat and --origin-lon for geo projection"
        )
    east_m = _required_number(getattr(position, "x", None), "position.x")
    north_m = _required_number(getattr(position, "y", None), "position.y")
    up_m = _optional_number(getattr(position, "z", None))
    latitude, longitude = _offset_meters_to_latlon(east_m, north_m, origin_lat=origin[0], origin_lon=origin[1])
    return GeoSample(time_s=time_s, latitude=latitude, longitude=longitude, altitude_m=up_m)


def _message_position(msg: Any) -> Any | None:
    pose = getattr(msg, "pose", None)
    if pose is not None:
        nested = getattr(pose, "pose", pose)
        position = getattr(nested, "position", None)
        if position is not None:
            return position
    return getattr(msg, "position", None)


def _offset_meters_to_latlon(
    east_m: float, north_m: float, *, origin_lat: float, origin_lon: float
) -> tuple[float, float]:
    lat = origin_lat + math.degrees(north_m / EARTH_RADIUS_M)
    cos_lat = math.cos(math.radians(origin_lat))
    if abs(cos_lat) < 1e-12:
        raise ValueError("origin latitude is too close to a pole for local x/y projection")
    lon = origin_lon + math.degrees(east_m / (EARTH_RADIUS_M * cos_lat))
    return lat, lon


def _metric_from_link_trace_row(row: Mapping[str, Any], *, path: Path, line_number: int) -> MetricSample | None:
    time_s = _row_time_s(row, path=path, line_number=line_number)
    passive = _mapping_or_none(row.get("passive_counter_delta"))
    tx = _mapping_or_none(passive.get("tx") if passive else None)
    rx = _mapping_or_none(passive.get("rx") if passive else None)
    probe = _mapping_or_none(row.get("peer_probe"))
    values: dict[str, float] = {}
    _add_numeric(values, "observed_tx_kbps", tx.get("observed_kbps") if tx else None)
    _add_numeric(values, "observed_rx_kbps", rx.get("observed_kbps") if rx else None)
    _add_numeric(values, "rtt_ms", probe.get("rtt_ms") if probe else None)
    _add_numeric(values, "loss_pct", probe.get("loss_pct") if probe else None)
    if not values:
        return None
    return MetricSample(
        time_s=time_s,
        source=f"link_trace:{path.name}:{line_number}",
        values=values,
        context={"peer": str(row.get("peer") or ""), "remote": str(row.get("remote") or "")},
    )


def _row_time_s(row: Mapping[str, Any], *, path: Path, line_number: int) -> float:
    generated_at = row.get("generated_at")
    if isinstance(generated_at, str) and generated_at:
        parsed = _parse_wall_iso(generated_at)
        if parsed is not None:
            return parsed
        raise ValueError(f"{path}:{line_number}: generated_at is not a valid ISO timestamp")
    return _required_number(row.get("monotonic_s"), f"{path}:{line_number}: monotonic_s")


def _parse_wall_iso(value: str) -> float | None:
    try:
        return datetime.fromisoformat(value.replace("Z", "+00:00")).timestamp()
    except ValueError:
        return None


def _mapping_or_none(value: Any) -> Mapping[str, Any] | None:
    return value if isinstance(value, Mapping) else None


def _add_numeric(values: dict[str, float], name: str, raw: Any) -> None:
    value = _optional_number(raw)
    if value is not None:
        values[name] = value


def _required_number(value: Any, label: str) -> float:
    number = _optional_number(value)
    if number is None:
        raise ValueError(f"{label} must be a finite number")
    return number


def _optional_number(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _event_label(record: Mapping[str, Any]) -> str:
    source = str(record.get("source") or "")
    target = str(record.get("target") or "")
    topic = str(record.get("topic") or "")
    return f"{source}->{target}:{topic}" if source or target else topic


def _record_latency_ms(record: Mapping[str, Any]) -> float | None:
    sections = record.get("sections")
    if not isinstance(sections, Mapping):
        return None
    value = _optional_number(sections.get("ota_hop_ms"))
    if value is None:
        value = _optional_number(sections.get("ota_hop_uncorrected_ms"))
    return value


def _stream_records_with_times(
    stream: Sequence[dict[str, Any]], *, time_field: str
) -> list[tuple[dict[str, Any], float]]:
    known = sorted(
        (int(record["seq"]), float(stamp))
        for record in stream
        if (stamp := _optional_number(record.get(time_field))) is not None and record.get("seq") is not None
    )
    if not known:
        return []
    period_s = _median_sequence_period(known)
    known_seq = [seq for seq, _stamp in known]
    known_time = [stamp for _seq, stamp in known]
    timed: list[tuple[dict[str, Any], float]] = []
    for record in stream:
        stamp = _optional_number(record.get(time_field))
        if stamp is not None:
            timed.append((record, stamp))
            continue
        if record.get("seq") is None:
            continue
        inferred = _infer_record_time(int(record["seq"]), known_seq, known_time, period_s=period_s)
        if inferred is not None:
            timed.append((record, inferred))
    return sorted(timed, key=lambda item: item[1])


def _median_sequence_period(known: Sequence[tuple[int, float]]) -> float | None:
    periods = [
        (t1 - t0) / (seq1 - seq0)
        for (seq0, t0), (seq1, t1) in zip(known, known[1:], strict=False)
        if seq1 > seq0 and t1 >= t0
    ]
    return statistics.median(periods) if periods else None


def _infer_record_time(
    seq: int, known_seq: Sequence[int], known_time: Sequence[float], *, period_s: float | None
) -> float | None:
    index = bisect.bisect_left(known_seq, seq)
    if 0 < index < len(known_seq):
        left_seq = known_seq[index - 1]
        right_seq = known_seq[index]
        if right_seq > left_seq:
            share = (seq - left_seq) / (right_seq - left_seq)
            return known_time[index - 1] + share * (known_time[index] - known_time[index - 1])
    if period_s is not None and index > 0:
        return known_time[index - 1] + (seq - known_seq[index - 1]) * period_s
    if period_s is not None and index < len(known_seq):
        return known_time[index] - (known_seq[index] - seq) * period_s
    return None


def _nearest_gps(gps: Sequence[GeoSample], times: Sequence[float], stamp: float) -> GeoSample | None:
    if not gps:
        return None
    index = bisect.bisect_left(times, stamp)
    candidates: list[GeoSample] = []
    if index > 0:
        candidates.append(gps[index - 1])
    if index < len(gps):
        candidates.append(gps[index])
    return min(candidates, key=lambda sample: abs(sample.time_s - stamp)) if candidates else None


def _csv_row(sample: GeoreferencedSample) -> dict[str, object]:
    return {
        "schema_version": SCHEMA_VERSION,
        "source": sample.source,
        "metric": sample.metric,
        "gps_time_s": _format_number(sample.gps.time_s),
        "source_time_s": _format_number(sample.source_time_s),
        "aligned_time_s": _format_number(sample.aligned_time_s),
        "time_delta_s": _format_number(sample.time_delta_s),
        "latitude": _format_number(sample.gps.latitude, digits=8),
        "longitude": _format_number(sample.gps.longitude, digits=8),
        "altitude_m": _format_optional(sample.gps.altitude_m),
        "metric_value": _format_number(sample.metric_value),
        "observed_tx_kbps": _format_optional(sample.values.get("observed_tx_kbps")),
        "observed_rx_kbps": _format_optional(sample.values.get("observed_rx_kbps")),
        "rtt_ms": _format_optional(sample.values.get("rtt_ms")),
        "loss_pct": _format_optional(sample.values.get("loss_pct")),
        "delivery_pct": _format_optional(sample.values.get("delivery_pct")),
        "event_loss_pct": _format_optional(sample.values.get("event_loss_pct")),
        "event_expected": sample.context.get("event_expected", ""),
        "event_delivered": sample.context.get("event_delivered", ""),
        "event_lost": sample.context.get("event_lost", ""),
        "event_topic": sample.context.get("event_topic", ""),
    }


def _format_optional(value: object) -> str:
    return "" if value is None else _format_number(float(value)) if isinstance(value, int | float) else str(value)


def _format_number(value: float, digits: int = 6) -> str:
    text = f"{value:.{digits}f}".rstrip("0").rstrip(".")
    return "0" if text == "-0" else text


def _project_map_points(samples: Sequence[GeoSample]) -> list[tuple[float, float]]:
    mean_lat = sum(sample.latitude for sample in samples) / len(samples)
    cos_lat = max(1e-9, math.cos(math.radians(mean_lat)))
    coords = [(sample.longitude * cos_lat, sample.latitude) for sample in samples]
    xs = [coord[0] for coord in coords]
    ys = [coord[1] for coord in coords]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max(max_x - min_x, 1e-9)
    span_y = max(max_y - min_y, 1e-9)
    return [
        (
            MAP_PAD + (x - min_x) / span_x * (MAP_WIDTH - 2 * MAP_PAD),
            MAP_HEIGHT - MAP_PAD - (y - min_y) / span_y * (MAP_HEIGHT - 2 * MAP_PAD),
        )
        for x, y in coords
    ]


def _route_image_path(html_path: Path) -> Path:
    return html_path.with_name(f"{html_path.stem}.route.png")


def _write_route_png(
    rows: Sequence[GeoreferencedSample],
    path: str | Path,
    *,
    metric: str,
) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    sorted_rows = sorted(rows, key=lambda sample: sample.gps.time_s)
    if not sorted_rows:
        raise ValueError("no georeferenced samples to render")

    values = [row.metric_value for row in sorted_rows]
    value_min = min(values)
    value_max = max(values)
    points = _project_map_points([row.gps for row in sorted_rows])
    image = Image.new("RGB", (MAP_WIDTH, MAP_HEIGHT), (251, 252, 253))
    draw = ImageDraw.Draw(image)

    for index in range(1, len(sorted_rows)):
        x0, y0 = points[index - 1]
        x1, y1 = points[index]
        color = _metric_color_rgb(
            sorted_rows[index].metric_value,
            value_min=value_min,
            value_max=value_max,
            metric=metric,
        )
        draw.line([(x0, y0), (x1, y1)], fill=color, width=5)

    for row, (x, y) in zip(sorted_rows, points, strict=True):
        radius = 5.0
        color = _metric_color_rgb(row.metric_value, value_min=value_min, value_max=value_max, metric=metric)
        draw.ellipse(
            (x - radius, y - radius, x + radius, y + radius),
            fill=color,
            outline=(21, 32, 43),
            width=1,
        )

    image.save(output, format="PNG", optimize=True)


def _metric_quality(value: float, *, value_min: float, value_max: float, metric: str) -> float:
    normalized = 0.5 if value_max == value_min else (value - value_min) / (value_max - value_min)
    normalized = max(0.0, min(1.0, normalized))
    return normalized if metric in HIGHER_IS_BETTER else 1.0 - normalized


def _metric_color_rgb(value: float, *, value_min: float, value_max: float, metric: str) -> tuple[int, int, int]:
    quality = _metric_quality(value, value_min=value_min, value_max=value_max, metric=metric)
    stops = ((196, 32, 39), (188, 170, 32), (45, 145, 63))
    if quality <= 0.5:
        start, end = stops[0], stops[1]
        fraction = quality * 2.0
    else:
        start, end = stops[1], stops[2]
        fraction = (quality - 0.5) * 2.0
    red = round(start[0] + (end[0] - start[0]) * fraction)
    green = round(start[1] + (end[1] - start[1]) * fraction)
    blue = round(start[2] + (end[2] - start[2]) * fraction)
    return red, green, blue


def write_manifest(path: str | Path, *, csv_path: str | Path, html_path: str | Path, sample_count: int) -> None:
    """Write a tiny sidecar manifest for scripted smoke checks."""

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": SCHEMA_VERSION,
        "kind": "geo_link_quality_map",
        "csv": str(csv_path),
        "html": str(html_path),
        "route_image": str(_route_image_path(Path(html_path))),
        "samples": sample_count,
    }
    output.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
