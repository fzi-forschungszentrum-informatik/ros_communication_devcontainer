#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Unified OTA DDS XML generator.

Loads an XML template from /ws/ota_configs/<config>(.template), detects which
placeholders it uses (#host_ip, #peer, #easy_mode_ip, #spdp_interval,
#fragment_size), substitutes already resolved addresses, and prints the
resulting XML to stdout.

Replaces the old per-template scripts (get_fastdds_xml.py,
get_fastdds_easy_mode_xml.py, get_cyclonedds_xml.py).
"""

import argparse
import os
import re
import sys
import tempfile

KNOWN_PLACEHOLDERS = {
    "#host_ip",
    "#peer",
    "#easy_mode_ip",
    "#spdp_interval",
    #: A template that scopes its settings to one ROS domain resolves these.
    #: Cyclone applies a <Domain> section to the domain its Id names, so a
    #: single configuration can hold the OTA settings for the OTA domain and
    #: leave the local domain alone — which is what lets every process on a
    #: host share one configuration instead of half of them carrying the OTA
    #: profile and half the local one.
    "#local_domain",
    "#ota_domain",
    #: How large an RTPS fragment this link sends. A template resolves it so
    #: that the number is a property of the session rather than of the
    #: installed package: 1024B and 1200B are both correct on CycloneDDS
    #: 0.10.5, only the smaller one is correct on 11.0.1, and a run comparing
    #: the two ROS distributions has to be able to say which it used.
    "#fragment_size",
}
PLACEHOLDER_PATTERN = re.compile(r"#[A-Za-z_][A-Za-z0-9_]*")
DEFAULT_SPDP_INTERVAL = "30s"
#: Strictly below the templates' MaxMessageSize; see cyclonedds_tuned.xml for
#: what making the two equal cost under ROS 2 Lyrical.
DEFAULT_FRAGMENT_SIZE = "1024B"

#: CycloneDDS size suffixes, and what each one is worth in bytes.
SIZE_UNITS = {"B": 1, "KB": 1000, "KIB": 1024, "MB": 1000_000, "MIB": 1024 * 1024}
SIZE_PATTERN = re.compile(r"(\d+)\s*(B|kB|KB|KiB|MB|MiB)?\Z")


def _template_path(name: str) -> str:
    """Resolve a config name to an on-disk template path.

    Accepts forms like 'fastdds_v1.xml', 'fastdds_v1.xml.template',
    or 'fastdds_v1' (we always look for a '<name>.template' file).
    Container path is preferred; falls back to the script directory for host runs.
    """
    name = name.strip()
    if not name:
        raise ValueError("config name must be non-empty.")
    if not name.endswith(".template"):
        name = name + ".template"

    container_path = f"/ws/ota_configs/{name}"
    if os.path.exists(container_path):
        return container_path
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(here, name)


def _resolved_address(value: str, label: str) -> str:
    address = value.strip()
    if not address:
        raise ValueError(f"{label} must be a non-empty resolved address.")
    return address


def _resolved_domain(value: str, label: str) -> str:
    """A ROS domain id, validated rather than pasted into the XML.

    An unusable Id is worse than a missing one: Cyclone keeps a <Domain>
    section it cannot match to any domain, so the settings vanish without a
    word and the link comes up on defaults.
    """
    domain = str(value).strip()
    if not domain.isdigit() or not 0 <= int(domain) <= 232:
        raise ValueError(f"{label} must be a ROS domain id between 0 and 232, got {value!r}.")
    return str(int(domain))


def _resolved_spdp_interval(value: str) -> str:
    interval = value.strip().lower()
    if not interval:
        raise ValueError("SPDP interval must be a non-empty duration.")
    match = re.fullmatch(r"(\d+(?:\.\d+)?)\s*(ms|s)?", interval)
    if not match:
        raise ValueError("SPDP interval must use seconds or milliseconds, e.g. '30s' or '500ms'.")
    number = float(match.group(1))
    if number <= 0.0:
        raise ValueError("SPDP interval must be > 0.")
    return interval


def _parse_size(value: str) -> int:
    """A CycloneDDS size, in bytes, or None when it is not one this script wrote.

    Used both to validate what a session asked for and to read the template's
    own MaxMessageSize back out of the rendered XML, so the warning below can
    compare the two.
    """
    match = SIZE_PATTERN.fullmatch(value.strip())
    if not match:
        return None
    return int(match.group(1)) * SIZE_UNITS[(match.group(2) or "B").upper()]


def _resolved_fragment_size(value: str) -> str:
    size = value.strip()
    if not size:
        raise ValueError("Fragment size must be a non-empty size.")
    parsed = _parse_size(size)
    if parsed is None:
        raise ValueError("Fragment size must be a byte size, e.g. '1024B', '1200B' or '4KiB'.")
    if parsed <= 0:
        raise ValueError("Fragment size must be > 0.")
    return size


def _warn_if_fragment_fills_the_datagram(content: str, path: str) -> None:
    """Say so when the rendered profile is the one Lyrical cannot carry.

    An RTPS message carries headers as well as its fragment, so a fragment of
    exactly MaxMessageSize cannot be sent at all. CycloneDDS 0.10.5 coped with
    that; 11.0.1 drops every fragmented sample and drops it silently, with the
    endpoints matched and small samples still arriving. That combination is a
    legitimate thing to ask for -- it is what the Kilted-era link ran, and
    reproducing it is the reason the fragment size is settable -- so this warns
    rather than refuses. Refusing would make the older profile unreachable; not
    saying anything would let it be chosen by accident.
    """
    fragment = re.search(r"<FragmentSize>([^<]+)</FragmentSize>", content)
    maximum = re.search(r"<MaxMessageSize>([^<]+)</MaxMessageSize>", content)
    if not fragment or not maximum:
        return
    fragment_bytes = _parse_size(fragment.group(1))
    maximum_bytes = _parse_size(maximum.group(1))
    if fragment_bytes is None or maximum_bytes is None or fragment_bytes < maximum_bytes:
        return
    print(
        f"WARNING - {os.path.basename(path)} renders FragmentSize {fragment.group(1)} against "
        f"MaxMessageSize {maximum.group(1)}. A fragment leaves no room for the RTPS header, so "
        "on CycloneDDS 11.0.1 (ROS 2 Lyrical) no fragmented sample will arrive, silently. "
        "Correct on 0.10.5 (Kilted), which is what this profile reproduces.",
        file=sys.stderr,
    )


def _resolve_peer_ips(value: str) -> list:
    seen = set()
    unique = []
    for address in value.split(","):
        address = address.strip()
        if not address or address in seen:
            continue
        seen.add(address)
        unique.append(address)
    if not unique:
        raise ValueError("Peer address must contain at least one resolved address.")
    return unique


PEER_BLOCK_OPEN = "<!--peer-block-->"
PEER_BLOCK_CLOSE = "<!--/peer-block-->"
PEER_BLOCK_PATTERN = re.compile(
    re.escape(PEER_BLOCK_OPEN) + r"(.*?)" + re.escape(PEER_BLOCK_CLOSE),
    re.DOTALL,
)


def _expand_peer_block(content: str, peer_ips: list) -> str:
    """Substitute #peer.

    Single peer: simple string replacement of every '#peer' with the IP.

    Multiple peers: the template MUST delimit the per-peer region with
    '<!--peer-block-->...<!--/peer-block-->' markers. The region between the
    markers is duplicated once per peer IP (markers stripped), keeping the
    indentation of the marker line so duplicates align.
    """
    occurrences = content.count("#peer")
    if occurrences == 0:
        return content

    has_markers = PEER_BLOCK_OPEN in content
    if has_markers and (PEER_BLOCK_CLOSE not in content):
        raise RuntimeError(f"Template has '{PEER_BLOCK_OPEN}' without matching '{PEER_BLOCK_CLOSE}'.")

    if len(peer_ips) == 1 and not has_markers:
        return content.replace("#peer", peer_ips[0])

    if not has_markers:
        raise RuntimeError(
            "Multiple peers requested but template has no peer-block markers. "
            f"Wrap the per-peer region with '{PEER_BLOCK_OPEN}' and "
            f"'{PEER_BLOCK_CLOSE}' (e.g. around the <locator>...</locator> "
            "block) to enable multi-peer expansion."
        )

    matches = list(PEER_BLOCK_PATTERN.finditer(content))
    if len(matches) != 1:
        raise RuntimeError(f"Template must contain exactly one peer-block region; found {len(matches)}.")
    m = matches[0]
    block = m.group(1)
    if "#peer" not in block:
        raise RuntimeError("Peer-block region does not contain '#peer'.")
    if content.count("#peer") != block.count("#peer"):
        raise RuntimeError("All '#peer' placeholders must live inside the peer-block region.")

    # Preserve the indentation of the marker line so duplicated blocks align.
    line_start = content.rfind("\n", 0, m.start()) + 1
    indent = content[line_start : m.start()]
    sep = ("\n" + indent) if not indent.strip() else ""

    rendered = sep.join(block.replace("#peer", ip) for ip in peer_ips)
    return content[: m.start()] + rendered + content[m.end() :]


def main(
    config: str,
    host_ip: str = None,
    peer: str = None,
    easy_mode_ip: str = None,
    spdp_interval: str = DEFAULT_SPDP_INTERVAL,
    fragment_size: str = DEFAULT_FRAGMENT_SIZE,
    local_domain: str = None,
    ota_domain: str = None,
) -> str:
    path = _template_path(config)
    if not os.path.exists(path):
        raise FileNotFoundError(f"Template not found: {path}")

    with open(path, encoding="utf-8") as f:
        content = f.read()

    found = set(PLACEHOLDER_PATTERN.findall(content))
    unknown = found - KNOWN_PLACEHOLDERS
    if unknown:
        raise RuntimeError(
            f"Template '{os.path.basename(path)}' contains unsupported placeholders: "
            f"{sorted(unknown)}. Supported: {sorted(KNOWN_PLACEHOLDERS)}."
        )

    if "#host_ip" in found:
        if not host_ip:
            raise RuntimeError(f"Template '{os.path.basename(path)}' uses #host_ip but --host-ip was not provided.")
        content = content.replace("#host_ip", _resolved_address(host_ip, "Host IP"))

    if "#easy_mode_ip" in found:
        if not easy_mode_ip:
            raise RuntimeError(
                f"Template '{os.path.basename(path)}' uses #easy_mode_ip but --easy-mode-ip was not provided."
            )
        content = content.replace("#easy_mode_ip", _resolved_address(easy_mode_ip, "Easy Mode IP"))

    if "#spdp_interval" in found:
        content = content.replace("#spdp_interval", _resolved_spdp_interval(spdp_interval))

    if "#fragment_size" in found:
        content = content.replace("#fragment_size", _resolved_fragment_size(fragment_size))

    for placeholder, value, option in (
        ("#local_domain", local_domain, "--local-domain"),
        ("#ota_domain", ota_domain, "--ota-domain"),
    ):
        if placeholder in found:
            if value is None or not str(value).strip():
                raise RuntimeError(
                    f"Template '{os.path.basename(path)}' uses {placeholder} but {option} was not provided."
                )
            content = content.replace(placeholder, _resolved_domain(value, placeholder))

    if "#peer" in found:
        if not peer:
            raise RuntimeError(f"Template '{os.path.basename(path)}' uses #peer but --peer was not provided.")
        content = _expand_peer_block(content, _resolve_peer_ips(peer))

    _warn_if_fragment_fills_the_datagram(content, path)
    return content


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Resolve placeholders in an OTA DDS XML template.")
    parser.add_argument(
        "-c",
        "--config",
        required=True,
        help="Template name (e.g. fastdds_v1.xml, cyclonedds.xml, fastdds_easy_mode.xml).",
    )
    parser.add_argument(
        "-i",
        "--host-ip",
        dest="host_ip",
        help="Resolved local host address (#host_ip).",
    )
    parser.add_argument(
        "-p",
        "--peer",
        dest="peer",
        help="Resolved peer address, or comma-separated addresses (#peer).",
    )
    parser.add_argument(
        "-e",
        "--easy-mode-ip",
        dest="easy_mode_ip",
        help="Resolved Fast DDS Easy Mode address (#easy_mode_ip).",
    )
    parser.add_argument(
        "-o",
        "--output",
        dest="output",
        help=(
            "Write the rendered XML to this path, atomically. Prefer this over a shell "
            "redirect: `> file` truncates the target before this script runs, so a "
            "failure here leaves a zero-byte profile behind, and a zero-byte profile is "
            "not an error to a DDS stack — it falls back to its defaults and the link "
            "comes up carrying nothing."
        ),
    )
    parser.add_argument(
        "--local-domain",
        dest="local_domain",
        help="Local ROS domain id, for a template that scopes a section to it (#local_domain).",
    )
    parser.add_argument(
        "--ota-domain",
        dest="ota_domain",
        help="OTA ROS domain id, for a template that scopes a section to it (#ota_domain).",
    )
    parser.add_argument(
        "--spdp-interval",
        dest="spdp_interval",
        default=DEFAULT_SPDP_INTERVAL,
        help=f"CycloneDDS SPDP interval duration (#spdp_interval, default: {DEFAULT_SPDP_INTERVAL}).",
    )
    parser.add_argument(
        "--fragment-size",
        dest="fragment_size",
        default=DEFAULT_FRAGMENT_SIZE,
        help=f"CycloneDDS RTPS fragment size (#fragment_size, default: {DEFAULT_FRAGMENT_SIZE}).",
    )
    args = parser.parse_args()
    output = args.output
    rendered = main(**{k: v for k, v in vars(args).items() if v is not None and k != "output"})
    if not output:
        print(rendered)
        raise SystemExit(0)
    if not rendered.strip():
        raise SystemExit(f"Refusing to write an empty profile to {output}.")
    directory = os.path.dirname(os.path.abspath(output)) or "."
    os.makedirs(directory, exist_ok=True)
    handle, temporary = tempfile.mkstemp(dir=directory, suffix=".partial")
    try:
        with os.fdopen(handle, "w", encoding="utf-8") as stream:
            stream.write(rendered)
        os.replace(temporary, output)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise
    print(f"Wrote {output} ({len(rendered)} bytes).", file=sys.stderr)
