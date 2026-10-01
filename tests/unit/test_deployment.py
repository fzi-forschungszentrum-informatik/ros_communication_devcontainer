# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from pathlib import Path

import pytest

from rosotacom.deployment import (
    PeerBinding,
    load_deployment,
    parse_assignments,
    resolve_peer_bindings,
    resolve_value,
)


def test_load_deployment_and_resolve_nested_value(tmp_path: Path) -> None:
    path = tmp_path / "deployment.yaml"
    path.write_text(
        "\n".join(
            [
                "hosts:",
                "  workstation:",
                "    address: value:networks.test.workstation",
                "    ssh: null",
                "  robot:",
                "    address: 10.0.0.11",
                "    ssh: robot-b",
                "values:",
                "  networks:",
                "    test:",
                "      workstation: 10.0.0.10",
                "",
            ]
        ),
        encoding="utf-8",
    )

    deployment = load_deployment(path)

    assert deployment is not None
    assert resolve_value("value:networks.test.workstation", deployment) == "10.0.0.10"
    bindings = resolve_peer_bindings(
        {"peers": {"a": {"host": "workstation"}, "b": {"host": "robot"}}},
        deployment,
    )
    assert bindings == {
        "a": PeerBinding("a", "10.0.0.10", None, "workstation"),
        "b": PeerBinding("b", "10.0.0.11", "robot-b", "robot"),
    }


def test_cli_peer_overrides_are_explicit_and_independent(tmp_path: Path) -> None:
    path = tmp_path / "deployment.yaml"
    path.write_text(
        "hosts:\n  first: {address: 10.0.0.1, ssh: first-ssh}\n  second: {address: 10.0.0.2, ssh: second-ssh}\n",
        encoding="utf-8",
    )
    deployment = load_deployment(path)

    bindings = resolve_peer_bindings(
        {"peers": {"a": {"host": "first"}, "b": {}}},
        deployment,
        peer_assignments={"a": "second", "b": "first"},
        address_overrides={"b": "192.0.2.20"},
        ssh_overrides={"a": "local", "b": "custom-ssh"},
    )

    assert bindings == {
        "a": PeerBinding("a", "10.0.0.2", None, "second"),
        "b": PeerBinding("b", "192.0.2.20", "custom-ssh", "first"),
    }


@pytest.mark.parametrize(
    ("content", "message"),
    [
        ("inventory: {}\n", "Unsupported deployment keys"),
        ("hosts: []\n", "deployment.hosts must be a mapping"),
        ("hosts:\n  robot: {ssh: robot-b}\n", "address must be a non-empty string"),
        ("hosts:\n  robot: {address: 10.0.0.1, user: me}\n", "Unsupported deployment.hosts.robot keys"),
    ],
)
def test_deployment_schema_rejects_old_or_unknown_shapes(tmp_path: Path, content: str, message: str) -> None:
    path = tmp_path / "deployment.yaml"
    path.write_text(content, encoding="utf-8")

    with pytest.raises(RuntimeError, match=message):
        load_deployment(path)


def test_bindings_require_every_peer_and_known_assignments() -> None:
    cfg = {"peers": {"a": {}, "b": {}}}

    with pytest.raises(RuntimeError, match="Missing deployment address"):
        resolve_peer_bindings(cfg, None)
    with pytest.raises(RuntimeError, match="unknown peer"):
        resolve_peer_bindings(cfg, None, address_overrides={"c": "10.0.0.3"})
    with pytest.raises(RuntimeError, match="Duplicate"):
        parse_assignments(["a=one", "a=two"], option="--peer")


def test_raw_addresses_can_override_session_host_defaults_without_deployment() -> None:
    bindings = resolve_peer_bindings(
        {"peers": {"a": {"host": "usual-a"}, "b": {"host": "usual-b"}}},
        None,
        address_overrides={"a": "192.0.2.10", "b": "192.0.2.11"},
        ssh_overrides={"b": "robot-b"},
    )

    assert bindings == {
        "a": PeerBinding("a", "192.0.2.10", None, None),
        "b": PeerBinding("b", "192.0.2.11", "robot-b", None),
    }
