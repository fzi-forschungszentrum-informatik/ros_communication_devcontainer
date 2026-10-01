# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Offline checks for the public router collector; no network access."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("public_router_collector", ROOT / "tools/milesight_cellular_metrics.py")
assert spec and spec.loader
collector = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = collector
spec.loader.exec_module(collector)


def config_file(tmp_path):
    path = tmp_path / "router.json"
    path.write_text(
        json.dumps(
            {
                "base_url": "https://10.20.0.1",
                "username": "fixture",
                "password": "fixture-only",
                "expected_source_ipv4": "10.20.0.2",
                "tls_certificate_sha256": "a" * 64,
            }
        )
    )
    path.chmod(0o600)
    return path


def test_credentials_need_private_permissions(tmp_path):
    path = config_file(tmp_path)
    assert collector.load_config(path).host == "10.20.0.1"
    path.chmod(0o644)
    with pytest.raises(collector.CollectorError, match="0600"):
        collector.load_config(path)


def test_route_mismatch_is_rejected_before_network_request(tmp_path):
    config = collector.load_config(config_file(tmp_path))
    with pytest.raises(collector.CollectorError, match="refused"):
        collector.require_router_route(config, source_resolver=lambda *_: "10.30.0.2")


def test_output_excludes_credentials_and_preserves_raw_sinr():
    sample = collector.normalize_cellular_response(
        {
            "status": 0,
            "result": {
                "modem": {"net_type": "LTE", "imei": "excluded", "password": "excluded"},
                "more": {"rsrp": -100, "sinr": 10, "imsi": "excluded", "cqi": 7},
            },
        }
    )
    assert sample["sinr_raw"] == 10
    assert "sinr_db" not in sample
    assert sample["cqi"] == 7
    assert "excluded" not in json.dumps(sample)


def test_certificate_mismatch_sends_no_request(monkeypatch, tmp_path):
    class Socket:
        def getpeercert(self, **kwargs):
            return b"unexpected certificate"

    class Connection:
        sock = Socket()

        def __init__(self, *args, **kwargs):
            pass

        def connect(self):
            pass

        def request(self, *args, **kwargs):
            pytest.fail("credentials sent before certificate verification")

        def close(self):
            pass

    monkeypatch.setattr(collector.http.client, "HTTPSConnection", Connection)
    client = collector.MilesightHTTPSClient(collector.load_config(config_file(tmp_path)))
    with pytest.raises(collector.CollectorError, match="fingerprint changed"):
        client.post("/cgi", {"password": "fixture-only"})
