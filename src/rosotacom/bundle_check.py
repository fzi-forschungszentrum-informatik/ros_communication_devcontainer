# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Completeness checks for rosotacom session-instance artifact bundles."""

from __future__ import annotations

import json
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import yaml

CheckStatus = Literal["present", "missing", "empty", "invalid"]


@dataclass(frozen=True)
class ExpectedPath:
    path: str
    label: str | None = None
    required: bool = True

    @property
    def display_label(self) -> str:
        return self.label or self.path


@dataclass(frozen=True)
class BundleCheckConfig:
    peers: tuple[str, ...] = ()
    files: tuple[ExpectedPath, ...] = ()
    bags: tuple[ExpectedPath, ...] = ()


@dataclass(frozen=True)
class CheckResult:
    status: CheckStatus
    kind: str
    label: str
    path: str
    required: bool
    detail: str = ""

    @property
    def failed(self) -> bool:
        if self.status == "present":
            return False
        if not self.required and self.status == "missing":
            return False
        return True


@dataclass(frozen=True)
class BundleCheckReport:
    root: Path
    results: tuple[CheckResult, ...]

    @property
    def failures(self) -> tuple[CheckResult, ...]:
        return tuple(result for result in self.results if result.failed)

    @property
    def complete(self) -> bool:
        return not self.failures


_MANIFEST_KEYS = {
    "schema_version",
    "peers",
    "files",
    "bags",
    "required_files",
    "optional_files",
    "required_bags",
    "optional_bags",
}
_ENTRY_KEYS = {"path", "label", "required"}


def load_bundle_manifest(path: str | Path) -> BundleCheckConfig:
    manifest_path = Path(path)
    loaded = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
    if not isinstance(loaded, Mapping):
        raise RuntimeError(f"Bundle manifest must contain a mapping: {manifest_path}")
    unknown = sorted(set(loaded) - _MANIFEST_KEYS)
    if unknown:
        raise RuntimeError(f"Unsupported bundle manifest keys in {manifest_path}: {unknown}")
    schema_version = loaded.get("schema_version", 1)
    if schema_version != 1:
        raise RuntimeError(f"Unsupported bundle manifest schema_version {schema_version!r}; expected 1.")

    files = (
        *_parse_expected_path_entries(loaded.get("files", ()), default_required=True, allow_required_override=True),
        *_parse_expected_path_entries(
            loaded.get("required_files", ()), default_required=True, allow_required_override=False
        ),
        *_parse_expected_path_entries(
            loaded.get("optional_files", ()), default_required=False, allow_required_override=False
        ),
    )
    bags = (
        *_parse_expected_path_entries(loaded.get("bags", ()), default_required=True, allow_required_override=True),
        *_parse_expected_path_entries(
            loaded.get("required_bags", ()), default_required=True, allow_required_override=False
        ),
        *_parse_expected_path_entries(
            loaded.get("optional_bags", ()), default_required=False, allow_required_override=False
        ),
    )
    return BundleCheckConfig(peers=_parse_peer_names(loaded.get("peers", ())), files=files, bags=bags)


def merge_bundle_configs(*configs: BundleCheckConfig) -> BundleCheckConfig:
    peers: list[str] = []
    files: list[ExpectedPath] = []
    bags: list[ExpectedPath] = []
    for config in configs:
        for peer in config.peers:
            if peer not in peers:
                peers.append(peer)
        files.extend(config.files)
        bags.extend(config.bags)
    return BundleCheckConfig(peers=tuple(peers), files=tuple(files), bags=tuple(bags))


def check_bundle(root_path: str | Path, config: BundleCheckConfig | None = None) -> BundleCheckReport:
    root = Path(root_path)
    effective_config = config or BundleCheckConfig()
    results: list[CheckResult] = []

    if not root.exists():
        return BundleCheckReport(root, (CheckResult("missing", "directory", "bundle root", ".", True),))
    if not root.is_dir():
        return BundleCheckReport(root, (CheckResult("invalid", "directory", "bundle root", ".", True),))

    results.append(CheckResult("present", "directory", "bundle root", ".", True))
    peers = effective_config.peers or _discover_peer_names(root)
    if not peers:
        results.append(
            CheckResult(
                "missing",
                "peer",
                "peer status directories",
                "logs/*/status",
                True,
                "no peers discovered; pass --peer or manifest peers when discovery is insufficient",
            )
        )
    for peer in peers:
        _validate_peer_name(peer)
        results.append(_check_status_json(root, peer))
        results.append(_check_events_jsonl(root, peer))

    for expected_file in effective_config.files:
        results.append(_check_required_file(root, expected_file))
    for expected_bag in effective_config.bags:
        results.append(_check_bag(root, expected_bag))

    return BundleCheckReport(root, tuple(results))


def format_bundle_report(report: BundleCheckReport) -> str:
    lines = [f"Bundle check: {report.root}"]
    for result in report.results:
        required = "required" if result.required else "optional"
        detail = f" ({result.detail})" if result.detail else ""
        lines.append(f"{result.status:7} {required} {result.kind} {result.label}: {result.path}{detail}")
    if report.complete:
        lines.append("Bundle complete.")
    else:
        lines.append(f"Bundle incomplete: {len(report.failures)} failure(s).")
    return "\n".join(lines)


def _parse_peer_names(raw: Any) -> tuple[str, ...]:
    if raw is None:
        return ()
    if isinstance(raw, str):
        peers = [raw]
    elif isinstance(raw, Sequence):
        peers = list(raw)
    else:
        raise RuntimeError("Bundle manifest peers must be a string or list of strings.")
    parsed: list[str] = []
    for peer in peers:
        if not isinstance(peer, str):
            raise RuntimeError("Bundle manifest peers must be strings.")
        _validate_peer_name(peer)
        if peer not in parsed:
            parsed.append(peer)
    return tuple(parsed)


def _parse_expected_path_entries(
    raw: Any, *, default_required: bool, allow_required_override: bool
) -> tuple[ExpectedPath, ...]:
    if raw is None:
        return ()
    if isinstance(raw, (str, Mapping)):
        entries: Iterable[Any] = (raw,)
    elif isinstance(raw, Sequence):
        entries = raw
    else:
        raise RuntimeError("Bundle manifest path entries must be strings or mappings.")

    parsed: list[ExpectedPath] = []
    for entry in entries:
        if isinstance(entry, str):
            parsed.append(ExpectedPath(path=entry, required=default_required))
            continue
        if not isinstance(entry, Mapping):
            raise RuntimeError("Bundle manifest path entries must be strings or mappings.")
        unknown = sorted(set(entry) - _ENTRY_KEYS)
        if unknown:
            raise RuntimeError(f"Unsupported bundle manifest path entry keys: {unknown}")
        path = entry.get("path")
        if not isinstance(path, str) or not path:
            raise RuntimeError("Bundle manifest path entries need a non-empty string path.")
        label = entry.get("label")
        if label is not None and not isinstance(label, str):
            raise RuntimeError("Bundle manifest path entry label must be a string when provided.")
        required = default_required
        if allow_required_override and "required" in entry:
            required_raw = entry["required"]
            if not isinstance(required_raw, bool):
                raise RuntimeError("Bundle manifest path entry required must be true or false.")
            required = required_raw
        parsed.append(ExpectedPath(path=path, label=label, required=required))
    return tuple(parsed)


def _validate_peer_name(peer: str) -> None:
    if not peer or peer in {".", ".."} or Path(peer).name != peer:
        raise RuntimeError(f"Peer names must be single path tokens, got {peer!r}.")


def _discover_peer_names(root: Path) -> tuple[str, ...]:
    logs_dir = root / "logs"
    if not logs_dir.is_dir():
        return ()
    return tuple(sorted(child.name for child in logs_dir.iterdir() if child.is_dir() and (child / "status").is_dir()))


def _relative_path(path: Path, root: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return str(path)


def _resolve_path(root: Path, raw_path: str) -> Path:
    path = Path(raw_path)
    return path if path.is_absolute() else root / path


def _check_existing_file(
    root: Path,
    path: Path,
    *,
    kind: str,
    label: str,
    required: bool,
    detail: str = "non-empty",
) -> CheckResult:
    display_path = _relative_path(path, root)
    if not path.exists():
        return CheckResult("missing", kind, label, display_path, required)
    if not path.is_file():
        return CheckResult("invalid", kind, label, display_path, required, "not a file")
    if path.stat().st_size == 0:
        return CheckResult("empty", kind, label, display_path, required)
    return CheckResult("present", kind, label, display_path, required, detail)


def _check_status_json(root: Path, peer: str) -> CheckResult:
    rel = Path("logs") / peer / "status" / "status.json"
    path = root / rel
    base = _check_existing_file(root, path, kind="status", label=f"{peer} status.json", required=True)
    if base.status != "present":
        return base
    text = path.read_text(encoding="utf-8")
    if not text.strip():
        return CheckResult("empty", "status", f"{peer} status.json", rel.as_posix(), True)
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        return CheckResult("invalid", "status", f"{peer} status.json", rel.as_posix(), True, str(exc))
    if not isinstance(payload, Mapping):
        return CheckResult(
            "invalid",
            "status",
            f"{peer} status.json",
            rel.as_posix(),
            True,
            "JSON root is not an object",
        )
    return CheckResult("present", "status", f"{peer} status.json", rel.as_posix(), True, "valid JSON")


def _check_events_jsonl(root: Path, peer: str) -> CheckResult:
    rel = Path("logs") / peer / "status" / "events.jsonl"
    path = root / rel
    base = _check_existing_file(root, path, kind="events", label=f"{peer} events.jsonl", required=True)
    if base.status != "present":
        return base
    text = path.read_text(encoding="utf-8")
    if not text.strip():
        return CheckResult("empty", "events", f"{peer} events.jsonl", rel.as_posix(), True, "file is empty")

    transit_rows = 0
    parsed_rows = 0
    for lineno, line in enumerate(text.splitlines(), start=1):
        if not line.strip():
            continue
        parsed_rows += 1
        try:
            payload = json.loads(line)
        except json.JSONDecodeError as exc:
            return CheckResult(
                "invalid",
                "events",
                f"{peer} events.jsonl",
                rel.as_posix(),
                True,
                f"line {lineno}: {exc}",
            )
        if not isinstance(payload, Mapping):
            return CheckResult(
                "invalid",
                "events",
                f"{peer} events.jsonl",
                rel.as_posix(),
                True,
                f"line {lineno}: JSON root is not an object",
            )
        if payload.get("kind") == "transit":
            transit_rows += 1

    if parsed_rows == 0:
        return CheckResult("empty", "events", f"{peer} events.jsonl", rel.as_posix(), True, "file is empty")
    if transit_rows == 0:
        return CheckResult("empty", "events", f"{peer} events.jsonl", rel.as_posix(), True, "no transit rows")
    row_label = "row" if transit_rows == 1 else "rows"
    return CheckResult(
        "present",
        "events",
        f"{peer} events.jsonl",
        rel.as_posix(),
        True,
        f"{transit_rows} transit {row_label}",
    )


def _check_required_file(root: Path, expected: ExpectedPath) -> CheckResult:
    path = _resolve_path(root, expected.path)
    return _check_existing_file(root, path, kind="file", label=expected.display_label, required=expected.required)


def _check_bag(root: Path, expected: ExpectedPath) -> CheckResult:
    raw_path = _resolve_path(root, expected.path)
    metadata_path = raw_path if raw_path.name == "metadata.yaml" else raw_path / "metadata.yaml"
    bag_dir = metadata_path.parent
    display_path = _relative_path(raw_path, root)
    if not raw_path.exists():
        return CheckResult("missing", "bag", expected.display_label, display_path, expected.required)
    if metadata_path != raw_path and not raw_path.is_dir():
        return CheckResult(
            "invalid", "bag", expected.display_label, display_path, expected.required, "not a bag directory"
        )
    if not metadata_path.exists():
        return CheckResult(
            "missing", "bag", expected.display_label, _relative_path(metadata_path, root), expected.required
        )
    if not metadata_path.is_file():
        return CheckResult(
            "invalid",
            "bag",
            expected.display_label,
            _relative_path(metadata_path, root),
            expected.required,
            "not a file",
        )
    if metadata_path.stat().st_size == 0:
        return CheckResult(
            "empty", "bag", expected.display_label, _relative_path(metadata_path, root), expected.required
        )

    try:
        loaded = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as exc:
        return CheckResult(
            "invalid",
            "bag",
            expected.display_label,
            _relative_path(metadata_path, root),
            expected.required,
            str(exc),
        )
    if not isinstance(loaded, Mapping):
        return CheckResult(
            "invalid",
            "bag",
            expected.display_label,
            _relative_path(metadata_path, root),
            expected.required,
            "metadata root is not a mapping",
        )
    info = loaded.get("rosbag2_bagfile_information")
    if not isinstance(info, Mapping):
        return CheckResult(
            "invalid",
            "bag",
            expected.display_label,
            _relative_path(metadata_path, root),
            expected.required,
            "missing rosbag2_bagfile_information",
        )

    topics = info.get("topics_with_message_count") or ()
    if not isinstance(topics, Sequence) or isinstance(topics, (str, bytes)):
        return CheckResult(
            "invalid",
            "bag",
            expected.display_label,
            _relative_path(metadata_path, root),
            expected.required,
            "topics_with_message_count is not a list",
        )
    total_messages = 0
    topics_with_messages = 0
    for index, entry in enumerate(topics):
        if not isinstance(entry, Mapping):
            return CheckResult(
                "invalid",
                "bag",
                expected.display_label,
                _relative_path(metadata_path, root),
                expected.required,
                f"topic entry {index} is not a mapping",
            )
        try:
            count = int(entry.get("message_count") or 0)
        except (TypeError, ValueError):
            return CheckResult(
                "invalid",
                "bag",
                expected.display_label,
                _relative_path(metadata_path, root),
                expected.required,
                f"topic entry {index} has non-integer message_count",
            )
        if count > 0:
            topics_with_messages += 1
            total_messages += count
    if total_messages <= 0:
        return CheckResult(
            "empty", "bag", expected.display_label, display_path, expected.required, "metadata has no messages"
        )

    data_files = info.get("relative_file_paths") or ()
    if not isinstance(data_files, Sequence) or isinstance(data_files, (str, bytes)):
        return CheckResult(
            "invalid",
            "bag",
            expected.display_label,
            _relative_path(metadata_path, root),
            expected.required,
            "relative_file_paths is not a list",
        )
    for data_file in data_files:
        if not isinstance(data_file, str) or not data_file:
            return CheckResult(
                "invalid",
                "bag",
                expected.display_label,
                _relative_path(metadata_path, root),
                expected.required,
                "relative_file_paths entries must be non-empty strings",
            )
        data_path = bag_dir / data_file
        if not data_path.exists():
            return CheckResult(
                "missing",
                "bag",
                expected.display_label,
                _relative_path(data_path, root),
                expected.required,
                "metadata references missing data file",
            )
        if not data_path.is_file():
            return CheckResult(
                "invalid",
                "bag",
                expected.display_label,
                _relative_path(data_path, root),
                expected.required,
                "metadata references a non-file path",
            )
        if data_path.stat().st_size == 0:
            return CheckResult(
                "empty",
                "bag",
                expected.display_label,
                _relative_path(data_path, root),
                expected.required,
                "metadata references an empty data file",
            )

    topic_label = "topic" if topics_with_messages == 1 else "topics"
    message_label = "message" if total_messages == 1 else "messages"
    return CheckResult(
        "present",
        "bag",
        expected.display_label,
        display_path,
        expected.required,
        f"{topics_with_messages} {topic_label}, {total_messages} {message_label}",
    )
