# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Offline image-quality metrics for decoded camera frames.

The command layer accepts rosbag2 stage bags (including a bare or unfinalized
`.mcap` a killed recorder leaves behind) and simple JSON frame manifests. Metric
math is intentionally independent of ROS so unit tests can cover the alignment
and loss semantics without a runtime container.
"""

from __future__ import annotations

import base64
import json
import math
import sqlite3
import struct
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from io import BytesIO
from pathlib import Path
from statistics import median
from typing import Any, Literal

import yaml

Alignment = Literal["auto", "pts", "index"]
ResolvedAlignment = Literal["pts", "index"]

SENSOR_IMAGE_TYPE = "sensor_msgs/msg/Image"
SENSOR_COMPRESSED_IMAGE_TYPE = "sensor_msgs/msg/CompressedImage"
SUPPORTED_IMAGE_TOPIC_TYPES = {SENSOR_IMAGE_TYPE, SENSOR_COMPRESSED_IMAGE_TYPE}
SUPPORTED_ENCODINGS = {"mono8", "rgb8", "bgr8", "rgba8", "bgra8"}


@dataclass(frozen=True)
class VideoFrame:
    index: int
    pts: int | None
    width: int
    height: int
    encoding: str
    channels: int
    data: bytes
    recorded_time_ns: int | None = None
    source: str | None = None


def _align(offset: int, alignment: int) -> int:
    return (offset + alignment - 1) // alignment * alignment


def _read_cdr_string(body: bytes, offset: int) -> tuple[str, int]:
    offset = _align(offset, 4)
    if offset + 4 > len(body):
        raise ValueError("truncated CDR string length")
    (length,) = struct.unpack_from("<I", body, offset)
    offset += 4
    end = offset + length
    if end > len(body) or length == 0:
        raise ValueError("truncated CDR string data")
    value = body[offset : end - 1].decode("utf-8", errors="replace")
    return value, end


def _canonicalize_image_data(width: int, height: int, encoding: str, step: int, raw: bytes) -> tuple[str, int, bytes]:
    normalized = encoding.strip().lower()
    if normalized not in SUPPORTED_ENCODINGS:
        raise ValueError(
            f"unsupported image encoding {encoding!r}; use a decoded sensor_msgs/Image stage with one of "
            f"{sorted(SUPPORTED_ENCODINGS)}"
        )

    if normalized == "mono8":
        channels = 1
        canonical_encoding = "mono8"
    elif normalized in {"rgb8", "bgr8"}:
        channels = 3
        canonical_encoding = "rgb8"
    else:
        channels = 4
        canonical_encoding = "rgba8"

    row_bytes = width * channels
    if step < row_bytes:
        raise ValueError(f"image step {step} is smaller than width*channels {row_bytes}")
    if len(raw) < step * height:
        raise ValueError(f"image data has {len(raw)} bytes, expected at least {step * height}")

    rows: list[bytes] = []
    for y in range(height):
        row = raw[y * step : y * step + row_bytes]
        if normalized == "bgr8":
            converted = bytearray(row_bytes)
            for x in range(0, row_bytes, 3):
                converted[x : x + 3] = bytes((row[x + 2], row[x + 1], row[x]))
            rows.append(bytes(converted))
        elif normalized == "bgra8":
            converted = bytearray(row_bytes)
            for x in range(0, row_bytes, 4):
                converted[x : x + 4] = bytes((row[x + 2], row[x + 1], row[x], row[x + 3]))
            rows.append(bytes(converted))
        else:
            rows.append(bytes(row))
    return canonical_encoding, channels, b"".join(rows)


def parse_sensor_image_cdr(
    cdr: bytes,
    *,
    index: int = 0,
    recorded_time_ns: int | None = None,
    source: str | None = None,
) -> VideoFrame:
    """Parse serialized little-endian XCDR1 ``sensor_msgs/msg/Image`` bytes."""
    if len(cdr) < 4 or cdr[1] != 0x01:
        raise ValueError("expected little-endian CDR encapsulation (0x00 0x01 ...)")
    body = cdr[4:]
    offset = 0
    if len(body) < 8:
        raise ValueError("truncated sensor_msgs/Image header")
    stamp_sec, stamp_nanosec = struct.unpack_from("<iI", body, offset)
    offset += 8
    _, offset = _read_cdr_string(body, offset)  # header.frame_id

    offset = _align(offset, 4)
    if offset + 8 > len(body):
        raise ValueError("truncated sensor_msgs/Image dimensions")
    height, width = struct.unpack_from("<II", body, offset)
    offset += 8
    encoding, offset = _read_cdr_string(body, offset)

    if offset >= len(body):
        raise ValueError("truncated sensor_msgs/Image is_bigendian")
    offset += 1  # is_bigendian, irrelevant for uint8 encodings

    offset = _align(offset, 4)
    if offset + 8 > len(body):
        raise ValueError("truncated sensor_msgs/Image step/data length")
    (step,) = struct.unpack_from("<I", body, offset)
    offset += 4
    (data_len,) = struct.unpack_from("<I", body, offset)
    offset += 4
    data = body[offset : offset + data_len]
    if len(data) != data_len:
        raise ValueError("truncated sensor_msgs/Image data")

    canonical_encoding, channels, canonical_data = _canonicalize_image_data(width, height, encoding, step, data)
    pts = stamp_sec * 1_000_000_000 + stamp_nanosec if stamp_sec or stamp_nanosec else None
    return VideoFrame(
        index=index,
        pts=pts,
        width=width,
        height=height,
        encoding=canonical_encoding,
        channels=channels,
        data=canonical_data,
        recorded_time_ns=recorded_time_ns,
        source=source,
    )


def _decode_compressed_image_payload(payload: bytes, *, source: str | None = None) -> tuple[int, int, str, int, bytes]:
    try:
        from PIL import Image, UnidentifiedImageError
    except ImportError:
        raise RuntimeError(
            "CompressedImage support requires the Python package `Pillow`; reinstall rosotacom or install Pillow"
        ) from None

    try:
        with Image.open(BytesIO(payload)) as image:
            if image.mode in {"1", "L", "I;16", "I"}:
                decoded = image.convert("L")
                return decoded.width, decoded.height, "mono8", 1, decoded.tobytes()
            decoded = image.convert("RGB")
            return decoded.width, decoded.height, "rgb8", 3, decoded.tobytes()
    except UnidentifiedImageError as exc:
        where = f" from {source}" if source else ""
        raise ValueError(f"unsupported CompressedImage payload{where}; expected JPEG or PNG bytes") from exc


def parse_sensor_compressed_image_cdr(
    cdr: bytes,
    *,
    index: int = 0,
    recorded_time_ns: int | None = None,
    source: str | None = None,
) -> VideoFrame:
    """Parse serialized little-endian XCDR1 ``sensor_msgs/msg/CompressedImage`` bytes."""
    if len(cdr) < 4 or cdr[1] != 0x01:
        raise ValueError("expected little-endian CDR encapsulation (0x00 0x01 ...)")
    body = cdr[4:]
    offset = 0
    if len(body) < 8:
        raise ValueError("truncated sensor_msgs/CompressedImage header")
    stamp_sec, stamp_nanosec = struct.unpack_from("<iI", body, offset)
    offset += 8
    _, offset = _read_cdr_string(body, offset)  # header.frame_id
    _, offset = _read_cdr_string(body, offset)  # format, e.g. "jpeg" or "png"

    offset = _align(offset, 4)
    if offset + 4 > len(body):
        raise ValueError("truncated sensor_msgs/CompressedImage data length")
    (data_len,) = struct.unpack_from("<I", body, offset)
    offset += 4
    payload = body[offset : offset + data_len]
    if len(payload) != data_len:
        raise ValueError("truncated sensor_msgs/CompressedImage data")

    width, height, encoding, channels, data = _decode_compressed_image_payload(payload, source=source)
    pts = stamp_sec * 1_000_000_000 + stamp_nanosec if stamp_sec or stamp_nanosec else None
    return VideoFrame(
        index=index,
        pts=pts,
        width=width,
        height=height,
        encoding=encoding,
        channels=channels,
        data=data,
        recorded_time_ns=recorded_time_ns,
        source=source,
    )


def _metadata_path(path: Path) -> Path | None:
    if path.is_dir() and (path / "metadata.yaml").is_file():
        return path / "metadata.yaml"
    if path.is_file() and path.name == "metadata.yaml":
        return path
    return None


def _load_bag_metadata(path: Path) -> dict[str, Any]:
    metadata = _metadata_path(path)
    if metadata is None:
        raise ValueError(f"not a rosbag2 directory or metadata.yaml: {path}")
    loaded = yaml.safe_load(metadata.read_text(encoding="utf-8")) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"bag metadata must be a mapping: {metadata}")
    info = loaded.get("rosbag2_bagfile_information") or {}
    if not isinstance(info, dict):
        raise ValueError(f"bag metadata missing rosbag2_bagfile_information: {metadata}")
    return loaded


def _bag_base_dir(path: Path) -> Path:
    metadata = _metadata_path(path)
    if metadata is None:
        raise ValueError(f"not a rosbag2 directory or metadata.yaml: {path}")
    return metadata.parent


def _bag_info(metadata: dict[str, Any]) -> dict[str, Any]:
    info = metadata.get("rosbag2_bagfile_information") or {}
    if not isinstance(info, dict):
        raise ValueError("bag metadata missing rosbag2_bagfile_information")
    return info


def _bag_file_paths(path: Path, metadata: dict[str, Any], suffix: str) -> list[Path]:
    base = _bag_base_dir(path)
    info = _bag_info(metadata)
    raw_paths = info.get("relative_file_paths") or []
    if isinstance(raw_paths, list) and raw_paths:
        files = [base / str(raw) for raw in raw_paths]
    else:
        files = sorted(base.glob(f"*{suffix}"))
    if not files:
        raise ValueError(f"bag has no {suffix} storage files: {base}")
    return files


def _topic_type(metadata: dict[str, Any], topic: str) -> str | None:
    topics = _bag_info(metadata).get("topics_with_message_count") or []
    if not isinstance(topics, list):
        return None
    for entry in topics:
        if not isinstance(entry, dict):
            continue
        topic_metadata = entry.get("topic_metadata") or {}
        if not isinstance(topic_metadata, dict):
            continue
        if topic_metadata.get("name") == topic:
            msg_type = topic_metadata.get("type")
            return str(msg_type) if msg_type is not None else None
    return None


def _require_supported_image_type(msg_type: str | None, topic: str, *, absent: str) -> str:
    if msg_type is None:
        raise ValueError(absent)
    if msg_type not in SUPPORTED_IMAGE_TOPIC_TYPES:
        raise ValueError(
            f"topic {topic!r} has type {msg_type!r}; PSNR/SSIM requires {SENSOR_IMAGE_TYPE} "
            f"or JPEG/PNG {SENSOR_COMPRESSED_IMAGE_TYPE} frames"
        )
    return msg_type


def _require_supported_image_topic(metadata: dict[str, Any], topic: str) -> str:
    return _require_supported_image_type(
        _topic_type(metadata, topic),
        topic,
        absent=f"topic {topic!r} is not present in bag metadata",
    )


def _parse_image_frame_cdr(
    msg_type: str,
    cdr: bytes,
    *,
    index: int,
    recorded_time_ns: int,
    source: str,
) -> VideoFrame:
    if msg_type == SENSOR_IMAGE_TYPE:
        return parse_sensor_image_cdr(cdr, index=index, recorded_time_ns=recorded_time_ns, source=source)
    if msg_type == SENSOR_COMPRESSED_IMAGE_TYPE:
        return parse_sensor_compressed_image_cdr(cdr, index=index, recorded_time_ns=recorded_time_ns, source=source)
    raise ValueError(f"unsupported image message type {msg_type!r}")


def _read_sqlite_bag(path: Path, metadata: dict[str, Any], topic: str) -> list[VideoFrame]:
    msg_type = _require_supported_image_topic(metadata, topic)
    rows: list[tuple[int, bytes]] = []
    for db_path in _bag_file_paths(path, metadata, ".db3"):
        conn = sqlite3.connect(db_path)
        try:
            topic_row = conn.execute("select id from topics where name = ?", (topic,)).fetchone()
            if topic_row is None:
                continue
            topic_id = int(topic_row[0])
            rows.extend(
                (int(timestamp), bytes(data))
                for timestamp, data in conn.execute(
                    "select timestamp, data from messages where topic_id = ? order by timestamp",
                    (topic_id,),
                )
            )
        finally:
            conn.close()
    rows.sort(key=lambda row: row[0])
    return [
        _parse_image_frame_cdr(msg_type, data, index=index, recorded_time_ns=timestamp, source=f"{path}:{topic}")
        for index, (timestamp, data) in enumerate(rows)
    ]


def _read_mcap_bag(path: Path, metadata: dict[str, Any], topic: str) -> list[VideoFrame]:
    msg_type = _require_supported_image_topic(metadata, topic)
    try:
        from mcap.reader import make_reader
    except ImportError:
        raise RuntimeError(
            "mcap support requires the Python package `mcap`; reinstall rosotacom or install mcap"
        ) from None

    rows: list[tuple[int, bytes]] = []
    for mcap_path in _bag_file_paths(path, metadata, ".mcap"):
        with mcap_path.open("rb") as fp:
            reader = make_reader(fp)
            for _schema, channel, message in reader.iter_messages(topics=[topic]):
                if channel.topic == topic:
                    rows.append((int(message.log_time), bytes(message.data)))
    rows.sort(key=lambda row: row[0])
    return [
        _parse_image_frame_cdr(msg_type, data, index=index, recorded_time_ns=timestamp, source=f"{path}:{topic}")
        for index, (timestamp, data) in enumerate(rows)
    ]


def read_rosbag_frames(path: str | Path, topic: str) -> list[VideoFrame]:
    bag_path = Path(path)
    metadata = _load_bag_metadata(bag_path)
    storage_id = str(_bag_info(metadata).get("storage_identifier") or "").strip()
    if storage_id == "sqlite3":
        return _read_sqlite_bag(bag_path, metadata, topic)
    if storage_id == "mcap":
        return _read_mcap_bag(bag_path, metadata, topic)
    raise ValueError(f"unsupported rosbag2 storage_identifier {storage_id!r}; expected sqlite3 or mcap")


def _stage_mcap_files(path: Path) -> list[Path]:
    """The `.mcap` split(s) of a metric stage bag, addressed as a bare file or a
    directory that lacks a finalized `metadata.yaml` (e.g. a recorder killed at
    session teardown)."""
    if path.is_file() and path.suffix == ".mcap":
        return [path]
    if path.is_dir():
        return sorted(p for p in path.glob("*.mcap") if p.is_file())
    return []


def read_mcap_stage_frames(path: str | Path, topic: str) -> list[VideoFrame]:
    """Read image frames from a metric stage `.mcap` that has no rosbag2
    `metadata.yaml`. rosbag2 writes `metadata.yaml` and the mcap summary/footer
    only on clean shutdown; the metric recorder runs inside a container that is
    torn down abruptly, so stage bags routinely arrive truncated. mcap is
    self-describing, so the message type comes from the file's own Schema records
    and a truncated tail is tolerated (the recorded prefix is kept)."""
    try:
        from mcap.exceptions import McapError
        from mcap.records import Channel, Message, Schema
        from mcap.stream_reader import StreamReader
    except ImportError:
        raise RuntimeError(
            "mcap support requires the Python package `mcap`; reinstall rosotacom or install mcap"
        ) from None

    stage_path = Path(path)
    schemas: dict[int, str] = {}
    channels: dict[int, tuple[str, int]] = {}
    rows: list[tuple[int, int, bytes]] = []
    for mcap_path in _stage_mcap_files(stage_path):
        with mcap_path.open("rb") as fp:
            records = iter(StreamReader(fp).records)
            while True:
                try:
                    record = next(records)
                except StopIteration:
                    break
                except (struct.error, EOFError, McapError):
                    # Unfinalized tail: keep what was decoded before the cut.
                    break
                if isinstance(record, Schema):
                    schemas[record.id] = record.name
                elif isinstance(record, Channel):
                    channels[record.id] = (record.topic, record.schema_id)
                elif isinstance(record, Message):
                    channel = channels.get(record.channel_id)
                    if channel is not None and channel[0] == topic:
                        rows.append((int(record.log_time), channel[1], bytes(record.data)))
    if not rows:
        raise ValueError(f"topic {topic!r} has no messages in mcap stage bag {stage_path}")
    msg_type = _require_supported_image_type(
        schemas.get(rows[0][1]),
        topic,
        absent=f"topic {topic!r} has no schema in mcap stage bag {stage_path}",
    )
    rows.sort(key=lambda row: row[0])
    return [
        _parse_image_frame_cdr(msg_type, data, index=index, recorded_time_ns=timestamp, source=f"{stage_path}:{topic}")
        for index, (timestamp, _schema_id, data) in enumerate(rows)
    ]


def _read_pnm(path: Path) -> tuple[int, int, str, int, bytes]:
    raw = path.read_bytes()
    offset = 0

    def token() -> bytes:
        nonlocal offset
        while offset < len(raw) and raw[offset] in b" \t\r\n":
            offset += 1
        if offset < len(raw) and raw[offset] == ord("#"):
            while offset < len(raw) and raw[offset] not in b"\r\n":
                offset += 1
            return token()
        start = offset
        while offset < len(raw) and raw[offset] not in b" \t\r\n":
            offset += 1
        if start == offset:
            raise ValueError(f"invalid PNM header in {path}")
        return raw[start:offset]

    magic = token()
    if magic not in {b"P5", b"P6"}:
        raise ValueError(f"unsupported PNM magic {magic!r}; expected P5 or P6")
    width = int(token())
    height = int(token())
    max_value = int(token())
    if max_value != 255:
        raise ValueError(f"unsupported PNM max value {max_value}; expected 255")
    while offset < len(raw) and raw[offset] in b" \t\r\n":
        offset += 1
    channels = 1 if magic == b"P5" else 3
    data = raw[offset:]
    expected = width * height * channels
    if len(data) != expected:
        raise ValueError(f"PNM payload has {len(data)} bytes, expected {expected}: {path}")
    return width, height, "mono8" if channels == 1 else "rgb8", channels, data


def read_frame_manifest(path: str | Path) -> list[VideoFrame]:
    manifest_path = Path(path)
    loaded = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict) or loaded.get("schema_version") != 1:
        raise ValueError(f"frame manifest must be a schema_version: 1 object: {manifest_path}")
    frames_raw = loaded.get("frames")
    if not isinstance(frames_raw, list):
        raise ValueError(f"frame manifest must contain frames list: {manifest_path}")

    frames: list[VideoFrame] = []
    for index, raw in enumerate(frames_raw):
        if not isinstance(raw, dict):
            raise ValueError(f"frame manifest entry {index} must be a mapping")
        pts = raw.get("pts")
        pts_value = int(pts) if pts is not None else None
        if raw.get("path") is not None:
            frame_path = manifest_path.parent / str(raw["path"])
            width, height, encoding, channels, data = _read_pnm(frame_path)
        else:
            width = int(raw["width"])
            height = int(raw["height"])
            encoding = str(raw.get("encoding") or "mono8")
            channels = int(raw.get("channels") or (1 if encoding == "mono8" else 3))
            data_b64 = raw.get("data_b64")
            if not isinstance(data_b64, str):
                raise ValueError(f"frame manifest entry {index} needs data_b64 or path")
            data = base64.b64decode(data_b64)
        canonical_encoding, canonical_channels, canonical_data = _canonicalize_image_data(
            width,
            height,
            encoding,
            width * channels,
            data,
        )
        frames.append(
            VideoFrame(
                index=index,
                pts=pts_value,
                width=width,
                height=height,
                encoding=canonical_encoding,
                channels=canonical_channels,
                data=canonical_data,
                source=str(manifest_path),
            )
        )
    return frames


def load_frames(path: str | Path, *, topic: str | None = None) -> list[VideoFrame]:
    frame_path = Path(path)
    if _metadata_path(frame_path) is not None:
        if not topic:
            raise ValueError("--topic is required when reading rosbag2 inputs")
        return read_rosbag_frames(frame_path, topic)
    if _stage_mcap_files(frame_path):
        if not topic:
            raise ValueError("--topic is required when reading .mcap stage bags")
        return read_mcap_stage_frames(frame_path, topic)
    return read_frame_manifest(frame_path)


def _ensure_same_shape(reference: VideoFrame, degraded: VideoFrame) -> None:
    if (
        reference.width != degraded.width
        or reference.height != degraded.height
        or reference.channels != degraded.channels
        or len(reference.data) != len(degraded.data)
    ):
        raise ValueError(
            "frame shape mismatch: "
            f"ref={reference.width}x{reference.height}x{reference.channels} "
            f"degraded={degraded.width}x{degraded.height}x{degraded.channels}"
        )


def mean_squared_error(reference: bytes, degraded: bytes) -> float:
    if len(reference) != len(degraded):
        raise ValueError(f"sample length mismatch: {len(reference)} != {len(degraded)}")
    if not reference:
        raise ValueError("cannot compare empty frames")
    total = 0
    for ref_sample, deg_sample in zip(reference, degraded, strict=True):
        delta = ref_sample - deg_sample
        total += delta * delta
    return total / len(reference)


def psnr_db(reference: bytes, degraded: bytes, *, max_value: float = 255.0) -> float:
    mse = mean_squared_error(reference, degraded)
    if mse == 0:
        return math.inf
    return 10.0 * math.log10((max_value * max_value) / mse)


def ssim(reference: bytes, degraded: bytes, *, max_value: float = 255.0) -> float:
    if len(reference) != len(degraded):
        raise ValueError(f"sample length mismatch: {len(reference)} != {len(degraded)}")
    if not reference:
        raise ValueError("cannot compare empty frames")
    n = float(len(reference))
    sum_x = sum(reference)
    sum_y = sum(degraded)
    mean_x = sum_x / n
    mean_y = sum_y / n
    var_x = sum((sample - mean_x) * (sample - mean_x) for sample in reference) / n
    var_y = sum((sample - mean_y) * (sample - mean_y) for sample in degraded) / n
    cov_xy = sum((x - mean_x) * (y - mean_y) for x, y in zip(reference, degraded, strict=True)) / n
    c1 = (0.01 * max_value) ** 2
    c2 = (0.03 * max_value) ** 2
    numerator = (2.0 * mean_x * mean_y + c1) * (2.0 * cov_xy + c2)
    denominator = (mean_x * mean_x + mean_y * mean_y + c1) * (var_x + var_y + c2)
    return numerator / denominator


def _resolve_alignment(
    reference: Sequence[VideoFrame],
    degraded: Sequence[VideoFrame],
    align: Alignment,
) -> ResolvedAlignment:
    if align == "index":
        return "index"
    if align == "pts":
        return "pts"
    if (
        reference
        and degraded
        and all(frame.pts is not None for frame in reference)
        and all(frame.pts is not None for frame in degraded)
    ):
        return "pts"
    return "index"


def _unique_pts(frames: Sequence[VideoFrame], label: str) -> dict[int, VideoFrame]:
    by_pts: dict[int, VideoFrame] = {}
    for frame in frames:
        if frame.pts is None:
            raise ValueError(f"{label} frame {frame.index} has no pts; use --align index")
        if frame.pts in by_pts:
            raise ValueError(f"{label} has duplicate pts {frame.pts}; use --align index")
        by_pts[frame.pts] = frame
    return by_pts


def _metric_value(value: float, digits: int = 6) -> float | str:
    if math.isinf(value):
        return "inf"
    return round(value, digits)


def _percentile(values: Sequence[float], percentile: float) -> float:
    if not values:
        raise ValueError("percentile requires at least one value")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * percentile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[int(position)]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _metric_summary(values: Sequence[float]) -> dict[str, float | str | None]:
    if not values:
        return {"mean": None, "median": None, "p5": None, "min": None}
    return {
        "mean": _metric_value(sum(values) / len(values)),
        "median": _metric_value(median(values)),
        "p5": _metric_value(_percentile(values, 0.05)),
        "min": _metric_value(min(values)),
    }


def _compare_pair(reference: VideoFrame, degraded: VideoFrame) -> dict[str, Any]:
    _ensure_same_shape(reference, degraded)
    frame_psnr = psnr_db(reference.data, degraded.data)
    frame_ssim = ssim(reference.data, degraded.data)
    return {
        "status": "compared",
        "ref_index": reference.index,
        "degraded_index": degraded.index,
        "pts": reference.pts,
        "recorded_time_ns": reference.recorded_time_ns,
        "width": reference.width,
        "height": reference.height,
        "encoding": reference.encoding,
        "psnr_db": _metric_value(frame_psnr),
        "ssim": _metric_value(frame_ssim),
        "_psnr_raw": frame_psnr,
        "_ssim_raw": frame_ssim,
    }


def _lost_frame(reference: VideoFrame) -> dict[str, Any]:
    return {
        "status": "lost",
        "ref_index": reference.index,
        "degraded_index": None,
        "pts": reference.pts,
        "recorded_time_ns": reference.recorded_time_ns,
        "width": reference.width,
        "height": reference.height,
        "encoding": reference.encoding,
        "psnr_db": None,
        "ssim": None,
    }


def _extra_frame(degraded: VideoFrame) -> dict[str, Any]:
    return {
        "degraded_index": degraded.index,
        "pts": degraded.pts,
        "recorded_time_ns": degraded.recorded_time_ns,
        "width": degraded.width,
        "height": degraded.height,
        "encoding": degraded.encoding,
    }


def _strip_private_metrics(rows: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    stripped = []
    for row in rows:
        clean = dict(row)
        clean.pop("_psnr_raw", None)
        clean.pop("_ssim_raw", None)
        stripped.append(clean)
    return stripped


def compare_frame_sequences(
    reference: Sequence[VideoFrame],
    degraded: Sequence[VideoFrame],
    *,
    align: Alignment = "auto",
) -> dict[str, Any]:
    resolved_align = _resolve_alignment(reference, degraded, align)
    rows: list[dict[str, Any]] = []
    extras: list[dict[str, Any]] = []

    if resolved_align == "pts":
        degraded_by_pts = _unique_pts(degraded, "degraded")
        reference_by_pts = _unique_pts(reference, "reference")
        for ref in reference:
            assert ref.pts is not None
            deg = degraded_by_pts.get(ref.pts)
            rows.append(_compare_pair(ref, deg) if deg is not None else _lost_frame(ref))
        for deg in degraded:
            assert deg.pts is not None
            if deg.pts not in reference_by_pts:
                extras.append(_extra_frame(deg))
    else:
        for index, ref in enumerate(reference):
            rows.append(_compare_pair(ref, degraded[index]) if index < len(degraded) else _lost_frame(ref))
        extras = [_extra_frame(frame) for frame in degraded[len(reference) :]]

    compared = [row for row in rows if row["status"] == "compared"]
    lost = [row for row in rows if row["status"] == "lost"]
    psnr_values = [float(row["_psnr_raw"]) for row in compared]
    ssim_values = [float(row["_ssim_raw"]) for row in compared]
    worst_row = min(compared, key=lambda row: (float(row["_ssim_raw"]), float(row["_psnr_raw"])), default=None)
    worst_frame = None if worst_row is None else _strip_private_metrics([worst_row])[0]
    delivery_ratio = len(compared) / len(reference) if reference else 1.0
    loss_pct = (len(lost) / len(reference) * 100.0) if reference else 0.0

    return {
        "schema_version": 1,
        "alignment": resolved_align,
        "delivery": {
            "reference_frames": len(reference),
            "degraded_frames": len(degraded),
            "compared_frames": len(compared),
            "lost_frames": len(lost),
            "extra_degraded_frames": len(extras),
            "delivery_ratio": round(delivery_ratio, 6),
            "loss_pct": round(loss_pct, 6),
        },
        "summary": {
            "psnr_db": _metric_summary(psnr_values),
            "ssim": _metric_summary(ssim_values),
            "worst_frame": worst_frame,
        },
        "frames": _strip_private_metrics(rows),
        "extra_degraded_frames": extras,
    }


def compare_inputs(
    reference: str | Path,
    degraded: str | Path,
    *,
    topic: str | None,
    reference_topic: str | None = None,
    degraded_topic: str | None = None,
    align: Alignment,
) -> dict[str, Any]:
    ref_topic = reference_topic or topic
    deg_topic = degraded_topic or topic
    return compare_frame_sequences(
        load_frames(reference, topic=ref_topic),
        load_frames(degraded, topic=deg_topic),
        align=align,
    )


def _summary_metric_as_float(value: float | str | None) -> float | None:
    if value == "inf":
        return math.inf
    if isinstance(value, int | float):
        return float(value)
    return None


def threshold_failures(
    report: dict[str, Any],
    *,
    min_mean_psnr: float | None = None,
    min_mean_ssim: float | None = None,
    max_loss_pct: float | None = None,
) -> list[str]:
    failures: list[str] = []
    summary = report.get("summary") or {}
    delivery = report.get("delivery") or {}
    psnr_mean = _summary_metric_as_float((summary.get("psnr_db") or {}).get("mean"))
    ssim_mean = _summary_metric_as_float((summary.get("ssim") or {}).get("mean"))
    loss_pct = _summary_metric_as_float(delivery.get("loss_pct"))
    if min_mean_psnr is not None and (psnr_mean is None or psnr_mean < min_mean_psnr):
        failures.append(f"mean PSNR {psnr_mean} dB is below {min_mean_psnr} dB")
    if min_mean_ssim is not None and (ssim_mean is None or ssim_mean < min_mean_ssim):
        failures.append(f"mean SSIM {ssim_mean} is below {min_mean_ssim}")
    if max_loss_pct is not None and (loss_pct is None or loss_pct > max_loss_pct):
        failures.append(f"loss {loss_pct}% exceeds {max_loss_pct}%")
    return failures


def synthetic_frame(index: int, *, width: int, height: int, channels: int = 1, seed: int = 0) -> VideoFrame:
    if channels not in {1, 3}:
        raise ValueError("synthetic frames support 1 or 3 channels")
    values = bytearray(width * height * channels)
    offset = 0
    for y in range(height):
        for x in range(width):
            base = (x * 7 + y * 13 + index * 17 + seed * 31) % 256
            if channels == 1:
                values[offset] = base
                offset += 1
            else:
                values[offset : offset + 3] = bytes((base, (base + 53) % 256, (base + 101) % 256))
                offset += 3
    return VideoFrame(
        index=index,
        pts=index,
        width=width,
        height=height,
        encoding="mono8" if channels == 1 else "rgb8",
        channels=channels,
        data=bytes(values),
    )


def degrade_frame(frame: VideoFrame, *, quantization_step: int) -> VideoFrame:
    if quantization_step <= 1:
        degraded = frame.data
    else:
        degraded = bytes((sample // quantization_step) * quantization_step for sample in frame.data)
    return VideoFrame(
        index=frame.index,
        pts=frame.pts,
        width=frame.width,
        height=frame.height,
        encoding=frame.encoding,
        channels=frame.channels,
        data=degraded,
        recorded_time_ns=frame.recorded_time_ns,
        source=frame.source,
    )


def write_frame_manifest(frames: Sequence[VideoFrame], path: str | Path) -> Path:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": 1,
        "frames": [
            {
                "index": frame.index,
                "pts": frame.pts,
                "width": frame.width,
                "height": frame.height,
                "encoding": frame.encoding,
                "channels": frame.channels,
                "data_b64": base64.b64encode(frame.data).decode("ascii"),
            }
            for frame in frames
        ],
    }
    out.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return out


def write_synthetic_pair(
    output_dir: str | Path,
    *,
    frames: int = 12,
    width: int = 32,
    height: int = 24,
    channels: int = 1,
    seed: int = 0,
    quantization_step: int = 8,
    drop_every: int | None = None,
) -> tuple[Path, Path]:
    out_dir = Path(output_dir)
    reference = [synthetic_frame(i, width=width, height=height, channels=channels, seed=seed) for i in range(frames)]
    degraded = []
    for frame in reference:
        if drop_every is not None and drop_every > 0 and (frame.index + 1) % drop_every == 0:
            continue
        degraded.append(degrade_frame(frame, quantization_step=quantization_step))
    ref_path = write_frame_manifest(reference, out_dir / "reference-frames.json")
    degraded_path = write_frame_manifest(degraded, out_dir / "degraded-frames.json")
    return ref_path, degraded_path
