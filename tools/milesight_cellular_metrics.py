#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Collect cellular status from a Milesight UR75 for rosotacom link traces.

Credentials and the pinned TLS certificate fingerprint are supplied in a
mode-0600 JSON file. Output contains only the selected measurement fields.
"""

from __future__ import annotations

import argparse
import hashlib
import http.client
import ipaddress
import json
import os
import re
import socket
import ssl
import stat
import subprocess
import sys
import tempfile
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from datetime import datetime, timezone
from http.cookies import CookieError, SimpleCookie
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

DEFAULT_CONFIG_PATH = Path(__file__).with_name(".milesight-router.json")
DEFAULT_STATE_PATH = Path("/tmp") / f"rosotacom-milesight-cellular-{os.getuid()}.json"
MAX_RESPONSE_BYTES = 1_048_576
REQUEST_TIMEOUT_S = 2.0
LOGIN_RETRY_INTERVAL_S = 60.0
LOGIN_KEY_HEX = b"1111111111111111".hex()
LOGIN_IV_HEX = b"2222222222222222".hex()


class CollectorError(RuntimeError):
    """Expected, sanitized collection failure."""


class HTTPStatusError(CollectorError):
    def __init__(self, status: int) -> None:
        super().__init__(f"router returned HTTP {status}")
        self.status = status


class APIStatusError(CollectorError):
    def __init__(self, operation: str, status: Any) -> None:
        if isinstance(status, (str, int, float)) and not isinstance(status, bool):
            rendered = repr(status)[:80]
        else:
            rendered = f"<{type(status).__name__}>"
        super().__init__(f"router {operation} returned API status {rendered}")
        self.operation = operation
        self.status = status


@dataclass(frozen=True)
class RouterConfig:
    host: str
    port: int
    username: str
    password: str
    expected_source_ipv4: str
    tls_certificate_sha256: str


def _private_regular_file(path: Path) -> None:
    try:
        metadata = path.lstat()
    except FileNotFoundError as exc:
        raise CollectorError(f"configuration is not installed: {path}") from exc
    if not stat.S_ISREG(metadata.st_mode):
        raise CollectorError(f"configuration must be a regular file: {path}")
    if stat.S_IMODE(metadata.st_mode) & 0o077:
        raise CollectorError(f"configuration permissions must be 0600: {path}")


def load_config(path: Path = DEFAULT_CONFIG_PATH) -> RouterConfig:
    """Load and strictly validate the local credential/configuration file."""
    _private_regular_file(path)
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CollectorError(f"configuration is not valid JSON: {path}") from exc
    if not isinstance(raw, dict):
        raise CollectorError("configuration root must be a JSON object")

    allowed = {
        "base_url",
        "username",
        "password",
        "expected_source_ipv4",
        "tls_certificate_sha256",
    }
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise CollectorError(f"configuration has unknown keys: {', '.join(unknown)}")

    base_url = str(raw.get("base_url") or "")
    parsed = urlsplit(base_url)
    if (
        parsed.scheme != "https"
        or not parsed.hostname
        or parsed.username is not None
        or parsed.password is not None
        or parsed.query
        or parsed.fragment
        or parsed.path not in ("", "/")
    ):
        raise CollectorError("base_url must be an HTTPS origin without credentials or a path")
    try:
        router_address = ipaddress.IPv4Address(parsed.hostname)
    except ipaddress.AddressValueError as exc:
        raise CollectorError("base_url must name the router by IPv4 address") from exc
    if not router_address.is_private or router_address.is_loopback or router_address.is_unspecified:
        raise CollectorError("base_url must name a private LAN address")
    try:
        port = parsed.port or 443
    except ValueError as exc:
        raise CollectorError("base_url has an invalid port") from exc
    if port != 443:
        raise CollectorError("base_url must use HTTPS port 443")

    username = raw.get("username")
    password = raw.get("password")
    if not isinstance(username, str) or not username.strip():
        raise CollectorError("configuration username must be a non-empty string")
    if not isinstance(password, str) or not password:
        raise CollectorError("configuration password must be a non-empty string")

    expected_source = str(raw.get("expected_source_ipv4") or "")
    try:
        source_address = ipaddress.ip_address(expected_source)
    except ValueError as exc:
        raise CollectorError("expected_source_ipv4 is not an IPv4 address") from exc
    if (
        source_address.version != 4
        or not source_address.is_private
        or source_address.is_loopback
        or source_address.is_unspecified
    ):
        raise CollectorError("expected_source_ipv4 must be the local private LAN address")

    fingerprint = str(raw.get("tls_certificate_sha256") or "").lower().replace(":", "")
    if re.fullmatch(r"[0-9a-f]{64}", fingerprint) is None:
        raise CollectorError("tls_certificate_sha256 must contain 64 hexadecimal digits")

    return RouterConfig(
        host=str(router_address),
        port=port,
        username=username.strip(),
        password=password,
        expected_source_ipv4=expected_source,
        tls_certificate_sha256=fingerprint,
    )


def local_source_ipv4(host: str, port: int) -> str:
    """Resolve the kernel's source address without sending a packet."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.connect((host, port))
        return str(probe.getsockname()[0])


def require_router_route(
    config: RouterConfig,
    *,
    source_resolver: Callable[[str, int], str] = local_source_ipv4,
) -> None:
    actual_source = source_resolver(config.host, config.port)
    if actual_source != config.expected_source_ipv4:
        raise CollectorError(
            "router query refused on this peer: "
            f"kernel selected source {actual_source}, expected {config.expected_source_ipv4}"
        )


def router_tls_context() -> ssl.SSLContext:
    """Use a pinned device certificate with the UR75's legacy TLS stack.

    The certificate fingerprint is checked before any credentials are sent.
    Security level 1 accommodates the embedded device's certificate.
    """
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    try:
        context.set_ciphers("DEFAULT:@SECLEVEL=1")
    except ssl.SSLError:
        # An OpenSSL without security levels already accepts what this needs.
        pass
    return context


def discover_certificate_sha256(host: str, port: int, timeout_s: float = 2.0) -> str:
    """Return the router certificate fingerprint for explicit one-time TOFU setup."""
    context = router_tls_context()
    with socket.create_connection((host, port), timeout=timeout_s) as plain:
        with context.wrap_socket(plain, server_hostname=host) as secure:
            certificate = secure.getpeercert(binary_form=True)
    if not certificate:
        raise CollectorError("router did not present a TLS certificate")
    return hashlib.sha256(certificate).hexdigest()


def encrypt_password(password: str) -> str:
    """Apply the AES-CBC encoding used by the router's own login page.

    The password is passed to OpenSSL on stdin, never through argv or the
    environment. The key and IV are protocol constants, not credentials.
    """
    try:
        result = subprocess.run(
            [
                "openssl",
                "enc",
                "-aes-128-cbc",
                "-K",
                LOGIN_KEY_HEX,
                "-iv",
                LOGIN_IV_HEX,
                "-nosalt",
                "-base64",
                "-A",
            ],
            input=password.encode("utf-8"),
            capture_output=True,
            check=False,
            timeout=1.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise CollectorError("could not invoke OpenSSL for router login") from exc
    if result.returncode != 0:
        raise CollectorError("OpenSSL rejected the router login encoding")
    try:
        return result.stdout.decode("ascii")
    except UnicodeDecodeError as exc:
        raise CollectorError("OpenSSL returned a non-ASCII login encoding") from exc


def _cookie_header(cookies: Mapping[str, str]) -> str:
    safe: list[str] = []
    for name, value in sorted(cookies.items()):
        if re.fullmatch(r"[!#$%&'*+.^_`|~0-9A-Za-z-]+", name) is None:
            continue
        if any(character in value for character in "\r\n;"):
            continue
        safe.append(f"{name}={value}")
    return "; ".join(safe)


class MilesightHTTPSClient:
    """Small HTTPS JSON client with certificate pinning and bounded responses."""

    def __init__(self, config: RouterConfig, timeout_s: float = REQUEST_TIMEOUT_S) -> None:
        self.config = config
        self.timeout_s = timeout_s
        self.context = router_tls_context()

    def post(
        self,
        path: str,
        payload: Mapping[str, Any],
        cookies: Mapping[str, str] | None = None,
    ) -> tuple[dict[str, Any], dict[str, str]]:
        connection = http.client.HTTPSConnection(
            self.config.host,
            self.config.port,
            timeout=self.timeout_s,
            context=self.context,
        )
        try:
            connection.connect()
            certificate = connection.sock.getpeercert(binary_form=True) if connection.sock else None
            actual_fingerprint = hashlib.sha256(certificate or b"").hexdigest()
            if actual_fingerprint != self.config.tls_certificate_sha256:
                raise CollectorError("router TLS certificate fingerprint changed")

            headers = {
                "Accept": "application/json",
                "Content-Type": "application/x-www-form-urlencoded",
                "Connection": "close",
            }
            cookie_value = _cookie_header(cookies or {})
            if cookie_value:
                headers["Cookie"] = cookie_value
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            connection.request("POST", path, body=body, headers=headers)
            response = connection.getresponse()
            raw = response.read(MAX_RESPONSE_BYTES + 1)
            if response.status < 200 or response.status >= 300:
                raise HTTPStatusError(response.status)
            if len(raw) > MAX_RESPONSE_BYTES:
                raise CollectorError("router response exceeded the 1 MiB limit")

            updated_cookies = dict(cookies or {})
            try:
                for header in response.headers.get_all("Set-Cookie", []):
                    parsed_cookie = SimpleCookie()
                    parsed_cookie.load(header)
                    for name, morsel in parsed_cookie.items():
                        updated_cookies[name] = morsel.value
            except CookieError as exc:
                raise CollectorError("router returned an invalid session cookie") from exc
            try:
                parsed = json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise CollectorError("router response was not valid JSON") from exc
            if not isinstance(parsed, dict):
                raise CollectorError("router response was not a JSON object")
            return parsed, updated_cookies
        except (OSError, ssl.SSLError, TimeoutError) as exc:
            raise CollectorError(f"router status request failed: {type(exc).__name__}") from exc
        finally:
            connection.close()


def _atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            json.dump(value, output, separators=(",", ":"), sort_keys=True)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
        path.chmod(0o600)
    finally:
        try:
            os.close(descriptor)
        except OSError:
            pass
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def write_private_json(path: Path, value: Mapping[str, Any]) -> None:
    """Write a private JSON configuration or session file atomically."""
    _atomic_json(path, value)


def _load_state(path: Path) -> dict[str, Any]:
    try:
        metadata = path.lstat()
    except FileNotFoundError:
        return {}
    if not stat.S_ISREG(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) & 0o077:
        raise CollectorError("runtime authentication state has unsafe permissions")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CollectorError("runtime authentication state is invalid") from exc
    return value if isinstance(value, dict) else {}


def _clean_cookies(value: Any) -> dict[str, str]:
    if not isinstance(value, dict):
        return {}
    result: dict[str, str] = {}
    for key, child in value.items():
        if isinstance(key, str) and isinstance(child, str) and len(key) <= 128 and len(child) <= 4096:
            result[key] = child
    return result


def _session_key(config: RouterConfig) -> str:
    identity = f"{config.host}:{config.port}:{config.username}:{config.tls_certificate_sha256}"
    return hashlib.sha256(identity.encode("utf-8")).hexdigest()


def _api_status(response: Mapping[str, Any], operation: str) -> None:
    if response.get("status") != 0:
        raise APIStatusError(operation, response.get("status"))


def _login_payload(config: RouterConfig) -> dict[str, Any]:
    return {
        "id": 1,
        "execute": 1,
        "core": "user",
        "function": "login",
        "values": [{"username": config.username, "password": encrypt_password(config.password)}],
    }


CELLULAR_PAYLOAD = {
    "id": 2,
    "execute": 1,
    "core": "yruo_status",
    "function": "get",
    "values": [{"base": "yruo_celluar"}],
}


def _login(
    client: MilesightHTTPSClient,
    config: RouterConfig,
    state_path: Path,
    state: dict[str, Any],
    now: float,
) -> dict[str, str]:
    last_failure = state.get("last_login_failure_at")
    if isinstance(last_failure, (int, float)) and now - float(last_failure) < LOGIN_RETRY_INTERVAL_S:
        remaining = LOGIN_RETRY_INTERVAL_S - (now - float(last_failure))
        raise CollectorError(f"router login retry suppressed for {remaining:.0f} s")
    try:
        response, cookies = client.post("/cgi", _login_payload(config))
        _api_status(response, "login")
    except CollectorError:
        _atomic_json(
            state_path,
            {
                "session_key": _session_key(config),
                "cookies": {},
                "last_login_failure_at": now,
            },
        )
        raise
    if not cookies:
        _atomic_json(
            state_path,
            {
                "session_key": _session_key(config),
                "cookies": {},
                "last_login_failure_at": now,
            },
        )
        raise CollectorError("router login succeeded without a session cookie")
    _atomic_json(
        state_path,
        {
            "session_key": _session_key(config),
            "cookies": cookies,
            "last_login_failure_at": None,
        },
    )
    return cookies


def read_cellular_response(
    config: RouterConfig,
    *,
    state_path: Path = DEFAULT_STATE_PATH,
    now: Callable[[], float] = time.time,
    client_factory: Callable[[RouterConfig], MilesightHTTPSClient] = MilesightHTTPSClient,
) -> dict[str, Any]:
    """Read one status response, reusing a runtime cookie and refreshing once."""
    state = _load_state(state_path)
    if state.get("session_key") != _session_key(config):
        state = {}
    cookies = _clean_cookies(state.get("cookies"))
    client = client_factory(config)
    if not cookies:
        cookies = _login(client, config, state_path, state, now())

    try:
        response, cookies = client.post("/cgi", CELLULAR_PAYLOAD, cookies)
        _api_status(response, "cellular status")
    except (APIStatusError, HTTPStatusError) as first_error:
        if isinstance(first_error, HTTPStatusError) and first_error.status not in (
            401,
            403,
        ):
            raise
        cookies = _login(client, config, state_path, {"cookies": {}}, now())
        response, cookies = client.post("/cgi", CELLULAR_PAYLOAD, cookies)
        _api_status(response, "cellular status")

    _atomic_json(
        state_path,
        {
            "session_key": _session_key(config),
            "cookies": cookies,
            "last_login_failure_at": None,
        },
    )
    return response


def _status_blocks(value: Any) -> tuple[dict[str, Any], dict[str, Any]]:
    modem: dict[str, Any] = {}
    more: dict[str, Any] = {}

    def visit(child: Any) -> None:
        nonlocal modem, more
        if isinstance(child, dict):
            candidate_modem = child.get("modem")
            candidate_more = child.get("more")
            if not modem and isinstance(candidate_modem, dict):
                modem = candidate_modem
            if not more and isinstance(candidate_more, dict):
                more = candidate_more
            for nested in child.values():
                if isinstance(nested, (dict, list)):
                    visit(nested)
        elif isinstance(child, list):
            for nested in child:
                visit(nested)

    visit(value)
    return modem, more


def _number(value: Any) -> int | float | None:
    if isinstance(value, bool) or value is None:
        return None
    match = re.search(r"[-+]?\d+(?:\.\d+)?", str(value))
    if match is None:
        return None
    parsed = float(match.group(0))
    return int(parsed) if parsed.is_integer() else parsed


def _text(value: Any) -> str | None:
    if value is None:
        return None
    parsed = str(value).strip()
    return parsed or None


def _put(output: dict[str, Any], key: str, value: Any) -> None:
    if value is not None:
        output[key] = value


# Downlink ranges needed by the European deployment. The ranges and frequency
# equation are from 3GPP TS 36.101, table 5.7.3-1. Unknown/NR channels remain
# explicit rather than being guessed.
LTE_DOWNLINK_BANDS = (
    (1, 0, 599, 2110.0, 0),
    (3, 1200, 1949, 1805.0, 1200),
    (7, 2750, 3449, 2620.0, 2750),
    (8, 3450, 3799, 925.0, 3450),
    (20, 6150, 6449, 791.0, 6150),
    (28, 9210, 9659, 758.0, 9210),
    (32, 9920, 10359, 1452.0, 9920),
    (38, 37750, 38249, 2570.0, 37750),
    (40, 38650, 39649, 2300.0, 38650),
    (41, 39650, 41589, 2496.0, 39650),
    (42, 41590, 43589, 3400.0, 41590),
    (43, 43590, 45589, 3600.0, 43590),
)


def lte_band_from_earfcn(earfcn: int) -> tuple[str, float] | None:
    for band, lower, upper, frequency_low_mhz, offset in LTE_DOWNLINK_BANDS:
        if lower <= earfcn <= upper:
            frequency = frequency_low_mhz + 0.1 * (earfcn - offset)
            return f"B{band}", round(frequency, 1)
    return None


def _normalize_router_band(value: Any, rat: str | None) -> str | None:
    text = _text(value)
    if text is None:
        return None
    compact = re.sub(r"\s+", "", text).upper().replace("BAND", "B")
    if re.fullmatch(r"B\d+", compact):
        return compact
    if re.fullmatch(r"N\d+", compact):
        return compact.lower()
    if compact.isdigit():
        return ("B" if (rat or "").upper() == "LTE" else "n") + compact
    return text


def normalize_cellular_response(response: Mapping[str, Any]) -> dict[str, Any]:
    """Whitelist and normalize the router response for link-trace storage."""
    _api_status(response, "cellular status")
    modem, more = _status_blocks(response.get("result"))
    if not modem and not more:
        raise CollectorError("cellular status response has no modem/more blocks")

    rat = _text(modem.get("net_type") or more.get("net_type"))
    output: dict[str, Any] = {
        "schema_version": 1,
        "source": "milesight_http_api:yruo_status/get:yruo_celluar",
        "collected_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    _put(output, "modem_status", _text(modem.get("modem_status")))
    _put(output, "registration_status", _text(modem.get("register")))
    _put(output, "rat", rat)
    _put(output, "cell_id", _text(modem.get("cellid")))
    _put(output, "lac", _text(modem.get("lac")))
    _put(output, "ecgi", _text(more.get("ecgi")))
    _put(output, "enodeb_id", _text(more.get("enodeb")))
    _put(output, "pci", _number(more.get("pcid") if "pcid" in more else more.get("pci")))
    _put(output, "cqi", _number(more.get("cqi")))
    _put(output, "earfcn", _number(more.get("earfcn")))
    _put(output, "nrarfcn", _number(more.get("nrarfcn")))
    _put(output, "rsrp_dbm", _number(more.get("rsrp")))
    _put(output, "rsrq_db", _number(more.get("rsrq")))
    # Preserve the router value without assuming its index is in decibels.
    _put(output, "sinr_raw", _number(more.get("sinr")))
    _put(output, "dl_bandwidth_mhz", _number(more.get("dl")))
    _put(output, "ul_bandwidth_mhz", _number(more.get("ul")))

    signal = _text(modem.get("signal"))
    if signal:
        asu = re.search(r"([-+]?\d+)\s*asu", signal, flags=re.IGNORECASE)
        rssi = re.search(r"\(\s*([-+]?\d+(?:\.\d+)?)\s*dBm\s*\)", signal, flags=re.IGNORECASE)
        _put(output, "signal_asu", int(asu.group(1)) if asu else None)
        _put(output, "rssi_dbm", _number(rssi.group(1)) if rssi else None)

    direct_band = _normalize_router_band(more.get("band") or modem.get("band") or more.get("cell_band"), rat)
    if direct_band:
        output["band"] = direct_band
        output["band_source"] = "router"
    elif (rat or "").upper() == "LTE" and isinstance(output.get("earfcn"), int):
        derived = lte_band_from_earfcn(output["earfcn"])
        if derived:
            output["band"], output["downlink_frequency_mhz"] = derived
            output["band_source"] = "derived_from_earfcn_3gpp_ts_36_101"

    if not any(key in output for key in ("rsrp_dbm", "rsrq_db", "sinr_raw", "cell_id", "rat")):
        raise CollectorError("cellular status response contained none of the required metrics")
    return output


def collect(
    config_path: Path = DEFAULT_CONFIG_PATH,
    *,
    state_path: Path = DEFAULT_STATE_PATH,
    source_resolver: Callable[[str, int], str] = local_source_ipv4,
) -> dict[str, Any]:
    config = load_config(config_path)
    require_router_route(config, source_resolver=source_resolver)
    response = read_cellular_response(config, state_path=state_path)
    return normalize_cellular_response(response)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument(
        "--normalize-response",
        type=Path,
        metavar="JSON",
        help="offline contract test: normalize a saved, non-sensitive API response",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.normalize_response is not None:
            raw = json.loads(args.normalize_response.read_text(encoding="utf-8"))
            if not isinstance(raw, dict):
                raise CollectorError("saved API response is not a JSON object")
            result = normalize_cellular_response(raw)
        else:
            result = collect(args.config)
    except (CollectorError, OSError, json.JSONDecodeError) as exc:
        print(f"milesight modem metrics unavailable: {exc}", file=sys.stderr)
        return 69
    print(json.dumps(result, separators=(",", ":"), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
