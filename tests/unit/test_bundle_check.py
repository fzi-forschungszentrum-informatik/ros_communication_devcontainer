# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

import rosotacom.cli as rosotacom_cli
from rosotacom.bundle_check import BundleCheckConfig, check_bundle


def _write_status_artifacts(instance: Path, peer: str, *, events: str | None = None) -> None:
    status_dir = instance / "logs" / peer / "status"
    status_dir.mkdir(parents=True, exist_ok=True)
    (status_dir / "status.json").write_text(
        json.dumps({"schema_version": 1, "peer": peer, "summary": {"OK": 1}}) + "\n",
        encoding="utf-8",
    )
    events_payload = events
    if events_payload is None:
        events_payload = "\n".join(
            [
                json.dumps({"kind": "state_transition", "topic": "/heartbeat"}),
                json.dumps({"kind": "transit", "topic": "/heartbeat", "seq": 1, "status": "delivered"}),
                "",
            ]
        )
    (status_dir / "events.jsonl").write_text(events_payload, encoding="utf-8")


def _write_bag(bag_dir: Path) -> None:
    bag_dir.mkdir(parents=True)
    (bag_dir / "native_0.mcap").write_bytes(b"mcap")
    metadata = {
        "rosbag2_bagfile_information": {
            "storage_identifier": "mcap",
            "relative_file_paths": ["native_0.mcap"],
            "duration": {"nanoseconds": 1_000_000_000},
            "topics_with_message_count": [
                {
                    "topic_metadata": {
                        "name": "/heartbeat",
                        "type": "std_msgs/msg/String",
                        "serialization_format": "cdr",
                    },
                    "message_count": 3,
                }
            ],
        }
    }
    (bag_dir / "metadata.yaml").write_text(yaml.safe_dump(metadata), encoding="utf-8")


def _write_complete_bundle(instance: Path) -> Path:
    _write_status_artifacts(instance, "a")
    _write_status_artifacts(instance, "b")
    config_dir = instance / "config"
    config_dir.mkdir(parents=True)
    (config_dir / "resolved-session.yaml").write_text("schema_version: 1\n", encoding="utf-8")
    (instance / "operator-notes.md").write_text("checked before departure\n", encoding="utf-8")
    _write_bag(instance / "rosbags" / "a" / "native")
    manifest = instance / "bundle.yaml"
    manifest.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "peers": ["a", "b"],
                "required_files": [
                    {"path": "config/resolved-session.yaml", "label": "resolved session config"},
                    "operator-notes.md",
                ],
                "required_bags": [{"path": "rosbags/a/native", "label": "native bag"}],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return manifest


def test_bundle_check_accepts_complete_fixture_directory(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    instance = tmp_path / "session-instances" / "2026-07-01" / "drive_001"
    manifest = _write_complete_bundle(instance)

    rc = rosotacom_cli.main(["bundle", "check", str(instance), "--manifest", str(manifest)])

    assert rc == 0
    out = capsys.readouterr().out
    assert "Bundle complete." in out
    assert "present required status a status.json: logs/a/status/status.json (valid JSON)" in out
    assert "present required bag native bag: rosbags/a/native (1 topic, 3 messages)" in out


def test_bundle_check_reports_missing_peer_status_file(tmp_path: Path) -> None:
    instance = tmp_path / "instance"
    _write_status_artifacts(instance, "a")
    _write_status_artifacts(instance, "b")
    (instance / "logs" / "b" / "status" / "status.json").unlink()

    report = check_bundle(instance, BundleCheckConfig(peers=("a", "b")))

    assert not report.complete
    assert any(
        result.status == "missing" and result.kind == "status" and result.path == "logs/b/status/status.json"
        for result in report.failures
    )


def test_bundle_check_reports_empty_events_file(tmp_path: Path) -> None:
    instance = tmp_path / "instance"
    _write_status_artifacts(instance, "a", events="")
    _write_status_artifacts(instance, "b")

    report = check_bundle(instance, BundleCheckConfig(peers=("a", "b")))

    assert not report.complete
    assert any(
        result.status == "empty" and result.kind == "events" and result.path == "logs/a/status/events.jsonl"
        for result in report.failures
    )
