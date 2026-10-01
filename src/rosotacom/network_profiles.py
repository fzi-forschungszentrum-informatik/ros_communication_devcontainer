# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""network profiles reference — network profiles: schema + ``tc``/``netem`` command generation.

The environment axis of the fidelity ladder (network profiles reference): a declarative, named,
per-direction `tc`/`netem` condition applied *below* rosotacom on the OTA
interface, so the transport sees a realistic pipe and is unaware of it.

This module is the **pure, host-testable** half — everything that can be exercised
without a privileged interface:

* the profile **schema** (static + timeline, per-direction parameters);
* the **argv generation** (a profile/direction → the exact `tc`/`netem` command
  list), so a host test asserts the built command without touching a real `qdisc`;
* the **outage kinds** (`catchup` = `loss 100%` with the interface up, vs
  ``reconnect`` = link-down forcing RMW re-discovery — the two recovery semantics
  network profiles reference left as an open question, resolved here as named variants);
* **timeline expansion** (ordered segments + outage → a stepping schedule).

The *privileged* half — actually arming/tearing down on a real interface with the
fail-safe revert — lives next to the orchestration that owns the SSH/staging path,
and consumes the argv this module builds.
"""

from __future__ import annotations

import re
import shlex
from collections.abc import Collection, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

# --------------------------------------------------------------------------- #
# Outage kinds — the two recovery semantics (network profiles reference open question, resolved)
# --------------------------------------------------------------------------- #

OUTAGE_CATCHUP = "catchup"  # netem loss 100%, interface stays up — DDS survives, "catch up"
OUTAGE_RECONNECT = "reconnect"  # link-down — forces RMW re-discovery, the harsher reconnect
OUTAGE_KINDS = (OUTAGE_CATCHUP, OUTAGE_RECONNECT)

# Default token-bucket buffer/latency, mirroring the calibrated baseline profile
# (`tbf rate <r> burst 32kbit latency 100ms`); overridable per direction.
_DEFAULT_TBF_BURST = "32kbit"
_DEFAULT_TBF_LATENCY_MS = 100.0

# The "unimpaired" tbf stage for rate-less steps of a rated timeline. Seamless
# stepping needs the qdisc tree shape to stay constant, so the stage stays
# present but never binds: faster than any link we emulate, with a token bucket
# big enough (~10 ms of tokens at 10 Gbit/s) that no packet ever waits on it.
_UNSHAPED_TBF_ARGS = ("rate", "10gbit", "burst", "12500kb", "latency", "100ms")


# --------------------------------------------------------------------------- #
# Value parsing (human strings → normalized numbers)
# --------------------------------------------------------------------------- #


def _num(value: Any, what: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{what}: expected a number, got {value!r}") from exc


def parse_rate_bps(value: Any) -> float:
    """``"4mbit"`` / ``"500kbit"`` / ``2.5e6`` → bits per second.

    Bit-oriented (rosotacom profiles are stated in bit/s, like the operator's
    baseline); a bare number is taken as bit/s. Byte suffixes are rejected to keep
    the unit unambiguous on the wire.
    """
    if isinstance(value, (int, float)):
        rate = float(value)
    else:
        text = str(value).strip().lower()
        match = re.fullmatch(r"(\d+(?:\.\d+)?)\s*(g|m|k)?bit", text) or re.fullmatch(r"(\d+(?:\.\d+)?)", text)
        if not match:
            raise ValueError(f"rate: expected e.g. '4mbit', '500kbit' or a bit/s number, got {value!r}")
        scale = {"k": 1e3, "m": 1e6, "g": 1e9, None: 1.0}[match.group(2) if match.lastindex == 2 else None]
        rate = float(match.group(1)) * scale
    if rate <= 0:
        raise ValueError(f"rate must be > 0, got {value!r}")
    return rate


def parse_ms(value: Any, what: str = "duration") -> float:
    """``"120ms"`` / ``"0.18s"`` / ``120`` → milliseconds (a bare number is ms)."""
    if isinstance(value, (int, float)):
        ms = float(value)
    else:
        text = str(value).strip().lower()
        match = re.fullmatch(r"(\d+(?:\.\d+)?)\s*(ms|s)?", text)
        if not match:
            raise ValueError(f"{what}: expected e.g. '120ms' or '0.5s', got {value!r}")
        ms = float(match.group(1)) * (1000.0 if match.group(2) == "s" else 1.0)
    if ms < 0:
        raise ValueError(f"{what} must be >= 0, got {value!r}")
    return ms


def parse_seconds(value: Any, what: str = "duration") -> float:
    """``"30s"`` / ``"500ms"`` / ``30`` → seconds (a bare number is seconds)."""
    if isinstance(value, (int, float)):
        seconds = float(value)
    else:
        text = str(value).strip().lower()
        match = re.fullmatch(r"(\d+(?:\.\d+)?)\s*(ms|s)?", text)
        if not match:
            raise ValueError(f"{what}: expected e.g. '30s' or '500ms', got {value!r}")
        seconds = float(match.group(1)) / 1000.0 if match.group(2) == "ms" else float(match.group(1))
    if seconds <= 0:
        raise ValueError(f"{what} must be > 0, got {value!r}")
    return seconds


def parse_pct(value: Any, what: str = "percentage") -> float:
    """``"2%"`` / ``2`` → percent (0–100)."""
    text = str(value).strip().rstrip("%") if isinstance(value, str) else value
    pct = _num(text, what)
    if not 0.0 <= pct <= 100.0:
        raise ValueError(f"{what} must be within [0, 100]%, got {value!r}")
    return pct


def parse_seed(value: Any, what: str = "seed") -> int:
    """``12345`` / ``"12345"`` -> a positive netem RNG seed."""
    try:
        seed = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{what}: expected a positive integer, got {value!r}") from exc
    if not 1 <= seed <= 0xFFFFFFFF:
        raise ValueError(f"{what} must be within [1, 4294967295], got {value!r}")
    return seed


# --------------------------------------------------------------------------- #
# Schema
# --------------------------------------------------------------------------- #

# Distribution tables iproute2 ships; anything else needs a ``distribution_file``
# that the runner installs into the shaping netns as ``/usr/lib/tc/<name>.dist``.
_BUILTIN_DISTRIBUTIONS = ("normal", "pareto", "paretonormal")
_DISTRIBUTION_NAME_RE = re.compile(r"^[a-z0-9][a-z0-9_-]*$")


@dataclass(frozen=True)
class GemodelLoss:
    """netem ``loss gemodel`` — the Gilbert(-Elliott) two-state loss process.

    Per-*packet* state transitions (netem evaluates the chain once per packet):
    ``p_pct`` = P(good→bad), ``r_pct`` = P(bad→good), ``loss_bad_pct`` = loss
    probability while bad (netem's ``1-h`` argument), ``loss_good_pct`` = loss
    probability while good (netem's ``1-k``). All four are always emitted, so the
    argv never depends on iproute2 defaults. This is the vocabulary the plain
    2-arg ``loss p% corr%`` cannot express: short intense bad states inside long
    clean good states, as measured on real cellular links.
    """

    p_pct: float
    r_pct: float
    loss_bad_pct: float = 100.0
    loss_good_pct: float = 0.0

    def __post_init__(self) -> None:
        for name in ("p_pct", "r_pct", "loss_bad_pct", "loss_good_pct"):
            value = getattr(self, name)
            if not 0.0 <= value <= 100.0:
                raise ValueError(f"gemodel {name} must be within [0, 100]%, got {value!r}")
        if self.p_pct == 0.0 and self.loss_good_pct == 0.0:
            raise ValueError("gemodel with p=0 and loss_good=0 never loses anything; drop the block instead")


@dataclass(frozen=True)
class DirectionShaping:
    """`tc`/`netem` parameters for one direction's egress. Every field optional;
    ``None`` means "not applied". An all-``None`` instance shapes nothing."""

    rate_bps: float | None = None
    delay_ms: float | None = None
    jitter_ms: float | None = None
    distribution: str | None = None  # builtin (normal|pareto|paretonormal) or a custom table name
    distribution_file: str | None = None  # .dist file for a custom table (installed as /usr/lib/tc/<name>.dist)
    loss_pct: float | None = None
    loss_correlation_pct: float | None = None  # netem correlated loss (needs loss)
    loss_gemodel: GemodelLoss | None = None  # netem Gilbert-Elliott loss (exclusive with loss)
    reorder_pct: float | None = None  # netem reorder (needs delay)
    duplicate_pct: float | None = None
    seed: int | None = None
    burst: str = _DEFAULT_TBF_BURST  # tbf buffer, only used when rate_bps is set
    tbf_latency_ms: float = _DEFAULT_TBF_LATENCY_MS

    @property
    def is_empty(self) -> bool:
        return (
            all(
                getattr(self, name) is None
                for name in (
                    "rate_bps",
                    "delay_ms",
                    "jitter_ms",
                    "loss_pct",
                    "reorder_pct",
                    "duplicate_pct",
                )
            )
            and self.loss_gemodel is None
        )

    @property
    def has_netem(self) -> bool:
        return (
            any(
                getattr(self, name) is not None
                for name in ("delay_ms", "jitter_ms", "loss_pct", "reorder_pct", "duplicate_pct")
            )
            or self.loss_gemodel is not None
        )

    def __post_init__(self) -> None:
        if self.jitter_ms is not None and self.delay_ms is None:
            raise ValueError("jitter requires a delay")
        if self.distribution is not None and self.jitter_ms is None:
            raise ValueError("distribution requires a jitter")
        if self.loss_correlation_pct is not None and self.loss_pct is None:
            raise ValueError("loss_correlation requires a loss")
        if self.loss_gemodel is not None and self.loss_pct is not None:
            raise ValueError("loss_gemodel and loss are exclusive: one loss model per direction")
        if self.reorder_pct is not None and self.delay_ms is None:
            raise ValueError("reorder requires a delay (netem reorders within the delay window)")
        if self.distribution is not None:
            if not _DISTRIBUTION_NAME_RE.fullmatch(self.distribution):
                raise ValueError(f"unsupported distribution name {self.distribution!r}")
            if self.distribution not in _BUILTIN_DISTRIBUTIONS and self.distribution_file is None:
                raise ValueError(
                    f"distribution {self.distribution!r} is not built into iproute2 "
                    f"({', '.join(_BUILTIN_DISTRIBUTIONS)}); give distribution_file so the "
                    "runner can install /usr/lib/tc/" + self.distribution + ".dist"
                )
        if self.distribution_file is not None and self.distribution is None:
            raise ValueError("distribution_file requires a distribution name")


@dataclass(frozen=True)
class TimelineSegment:
    """One ordered step of a timeline profile: hold ``shaping`` (per direction) for
    ``for_s`` seconds, or apply an ``outage`` of one of the named kinds."""

    for_s: float
    uplink: DirectionShaping | None = None
    downlink: DirectionShaping | None = None
    outage: str | None = None  # None | OUTAGE_CATCHUP | OUTAGE_RECONNECT

    def __post_init__(self) -> None:
        if self.outage is not None and self.outage not in OUTAGE_KINDS:
            raise ValueError(f"outage must be one of {OUTAGE_KINDS}, got {self.outage!r}")
        if self.outage is not None and (self.uplink is not None or self.downlink is not None):
            raise ValueError("an outage segment cannot also carry direction shaping")


@dataclass(frozen=True)
class Profile:
    """A named environment spec: either ``static`` (constant per-direction shaping)
    or ``timeline`` (ordered segments — the substrate for the recovery genre)."""

    name: str
    kind: str  # "static" | "timeline"
    uplink: DirectionShaping | None = None
    downlink: DirectionShaping | None = None
    timeline: Sequence[TimelineSegment] = field(default_factory=tuple)

    @property
    def is_timeline(self) -> bool:
        return self.kind == "timeline"

    @property
    def total_duration_s(self) -> float | None:
        return sum(segment.for_s for segment in self.timeline) if self.is_timeline else None


# --------------------------------------------------------------------------- #
# Schema parsing (profiles.yaml → Profile objects)
# --------------------------------------------------------------------------- #

_DIRECTION_KEYS = frozenset(
    {
        "rate",
        "delay",
        "jitter",
        "distribution",
        "distribution_file",
        "loss",
        "loss_correlation",
        "loss_gemodel",
        "reorder",
        "duplicate",
        "seed",
        "burst",
        "tbf_latency",
    }
)

_GEMODEL_KEYS = frozenset({"p", "r", "loss_bad", "loss_good"})


def parse_gemodel(spec: Mapping[str, Any]) -> GemodelLoss:
    """Parse a ``loss_gemodel`` block (``p``/``r`` required, loss_bad/loss_good optional)."""
    if not isinstance(spec, Mapping):
        raise ValueError(f"loss_gemodel: expected a mapping with p/r[/loss_bad/loss_good], got {spec!r}")
    unknown = sorted(set(spec) - _GEMODEL_KEYS)
    if unknown:
        raise ValueError(f"unsupported loss_gemodel keys {unknown}; allowed: {sorted(_GEMODEL_KEYS)}")
    missing = sorted({"p", "r"} - set(spec))
    if missing:
        raise ValueError(f"loss_gemodel requires {missing}")
    return GemodelLoss(
        p_pct=parse_pct(spec["p"], "loss_gemodel p"),
        r_pct=parse_pct(spec["r"], "loss_gemodel r"),
        loss_bad_pct=parse_pct(spec["loss_bad"], "loss_gemodel loss_bad") if "loss_bad" in spec else 100.0,
        loss_good_pct=parse_pct(spec["loss_good"], "loss_gemodel loss_good") if "loss_good" in spec else 0.0,
    )


def parse_direction(spec: Mapping[str, Any] | None) -> DirectionShaping | None:
    """Parse one ``uplink``/``downlink`` block into a :class:`DirectionShaping`."""
    if spec is None:
        return None
    unknown = sorted(set(spec) - _DIRECTION_KEYS)
    if unknown:
        raise ValueError(f"unsupported direction keys {unknown}; allowed: {sorted(_DIRECTION_KEYS)}")
    return DirectionShaping(
        rate_bps=parse_rate_bps(spec["rate"]) if "rate" in spec else None,
        delay_ms=parse_ms(spec["delay"], "delay") if "delay" in spec else None,
        jitter_ms=parse_ms(spec["jitter"], "jitter") if "jitter" in spec else None,
        distribution=str(spec["distribution"]) if "distribution" in spec else None,
        distribution_file=str(spec["distribution_file"]) if "distribution_file" in spec else None,
        loss_pct=parse_pct(spec["loss"], "loss") if "loss" in spec else None,
        loss_correlation_pct=(
            parse_pct(spec["loss_correlation"], "loss_correlation") if "loss_correlation" in spec else None
        ),
        loss_gemodel=parse_gemodel(spec["loss_gemodel"]) if "loss_gemodel" in spec else None,
        reorder_pct=parse_pct(spec["reorder"], "reorder") if "reorder" in spec else None,
        duplicate_pct=parse_pct(spec["duplicate"], "duplicate") if "duplicate" in spec else None,
        seed=parse_seed(spec["seed"], "seed") if "seed" in spec else None,
        burst=str(spec.get("burst", _DEFAULT_TBF_BURST)),
        tbf_latency_ms=parse_ms(spec["tbf_latency"], "tbf_latency")
        if "tbf_latency" in spec
        else _DEFAULT_TBF_LATENCY_MS,
    )


def _parse_outage(value: Any) -> str:
    """``outage: catchup|reconnect`` (a bare ``true`` defaults to the milder ``catchup``)."""
    if value is True:
        return OUTAGE_CATCHUP
    kind = str(value).strip().lower()
    if kind not in OUTAGE_KINDS:
        raise ValueError(f"outage must be one of {OUTAGE_KINDS} (or true → {OUTAGE_CATCHUP}), got {value!r}")
    return kind


def parse_timeline_segment(spec: Mapping[str, Any]) -> TimelineSegment:
    unknown = sorted(set(spec) - {"for", "outage", "uplink", "downlink"})
    if unknown:
        raise ValueError(f"unsupported timeline-segment keys {unknown}; use for/outage/uplink/downlink")
    if "for" not in spec:
        raise ValueError("a timeline segment requires a 'for' duration")
    for_s = parse_seconds(spec["for"], "for")
    if "outage" in spec:
        if "uplink" in spec or "downlink" in spec:
            raise ValueError("an outage segment cannot also carry direction shaping")
        return TimelineSegment(for_s=for_s, outage=_parse_outage(spec["outage"]))
    return TimelineSegment(
        for_s=for_s,
        uplink=parse_direction(spec.get("uplink")),
        downlink=parse_direction(spec.get("downlink")),
    )


def parse_profile(name: str, spec: Mapping[str, Any]) -> Profile:
    """Parse one named profile block (static or timeline)."""
    if "timeline" in spec:
        unknown = sorted(set(spec) - {"timeline"})
        if unknown:
            raise ValueError(f"profile {name!r}: a timeline profile cannot also have {unknown}")
        segments = [parse_timeline_segment(segment) for segment in spec["timeline"]]
        if not segments:
            raise ValueError(f"profile {name!r}: timeline must have at least one segment")
        return Profile(name=name, kind="timeline", timeline=tuple(segments))
    unknown = sorted(set(spec) - {"uplink", "downlink"})
    if unknown:
        raise ValueError(f"profile {name!r}: unsupported keys {unknown}; use 'uplink'/'downlink' or 'timeline'")
    return Profile(
        name=name,
        kind="static",
        uplink=parse_direction(spec.get("uplink")),
        downlink=parse_direction(spec.get("downlink")),
    )


def parse_profiles(doc: Mapping[str, Any]) -> dict[str, Profile]:
    """Parse a ``profiles.yaml`` document (``{profiles: {<name>: <spec>}}``)."""
    profiles_block = doc.get("profiles", doc) if isinstance(doc, Mapping) else {}
    if not isinstance(profiles_block, Mapping):
        raise ValueError("profiles.yaml: top-level 'profiles' must be a mapping of name → spec")
    return {str(name): parse_profile(str(name), spec or {}) for name, spec in profiles_block.items()}


def load_profiles_file(path: str | Path) -> dict[str, Profile]:
    """Load a project-scoped ``profiles.yaml`` into ``{name: Profile}``.

    Relative ``distribution_file`` entries are resolved against the profiles
    file's own directory, so a profiles file stays self-contained wherever the
    project is checked out.
    """
    import yaml

    doc = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if not isinstance(doc, Mapping):
        raise ValueError(f"{path}: profiles file must be a mapping (with a top-level 'profiles:' block)")
    profiles = parse_profiles(doc)
    base = Path(path).resolve().parent
    return {name: _resolve_distribution_files(profile, base) for name, profile in profiles.items()}


def _resolve_distribution_files(profile: Profile, base: Path) -> Profile:
    from dataclasses import replace

    def fix(shaping: DirectionShaping | None) -> DirectionShaping | None:
        if shaping is None or shaping.distribution_file is None:
            return shaping
        file = Path(shaping.distribution_file)
        if not file.is_absolute():
            file = base / file
        return replace(shaping, distribution_file=str(file))

    if profile.is_timeline:
        segments = tuple(
            replace(segment, uplink=fix(segment.uplink), downlink=fix(segment.downlink)) for segment in profile.timeline
        )
        return replace(profile, timeline=segments)
    return replace(profile, uplink=fix(profile.uplink), downlink=fix(profile.downlink))


#: Where `tc` looks for a delay-distribution table, in the order to try.
#: iproute2 compiles the directory in (``LIBDIR "/tc"``), so on a multiarch
#: Debian or Ubuntu build it is ``/usr/lib/<triplet>/tc`` and ``/usr/lib/tc``
#: does not exist at all. A table written to the latter is then invisible with
#: no error of its own: netem answers ``No distribution data for <name>`` and
#: the run dies at arming time, which is what every profile carrying a fitted
#: table did until this was found. The directory holding iproute2's own tables
#: is the one it reads, so that is the evidence to look for rather than a
#: guessed triplet.
TC_LIB_DIR_CANDIDATES = ("/usr/lib/*-linux-gnu*/tc", "/usr/lib/tc")

#: A table iproute2 ships, used as the marker for "this is the directory".
TC_LIB_DIR_MARKERS = ("normal.dist", "pareto.dist")


def tc_lib_dir_probe(root: str = "") -> str:
    """A ``sh -c`` program that prints the directory ``tc`` reads tables from.

    Three rungs, in order: a candidate that already holds one of iproute2's own
    tables (proof, not inference); a candidate that merely exists; and finally
    ``/usr/lib/tc``, created, for an image that has neither. `root` exists so
    the rungs are testable without a container.
    """
    cands = " ".join(f"{root}{c}" for c in TC_LIB_DIR_CANDIDATES)
    marker = " || ".join(f'[ -f "$d/{m}" ]' for m in TC_LIB_DIR_MARKERS)
    fallback = f"{root}/usr/lib/tc"
    return (
        f"for d in {cands}; do "
        f'  if {marker}; then printf %s "$d"; exit 0; fi; '
        f"done; "
        f"for d in {cands}; do "
        f'  if [ -d "$d" ]; then printf %s "$d"; exit 0; fi; '
        f"done; "
        f"mkdir -p {fallback} && printf %s {fallback}"
    )


def required_dist_installs(profile: Profile) -> list[tuple[str, str]]:
    """``(table_name, file_path)`` pairs a runner must install as
    ``<tc lib dir>/<table_name>.dist`` inside the shaping netns before arming.

    Builtin distributions need nothing; duplicates collapse; conflicting files
    for one name are an error (two segments must mean the same table).
    """
    found: dict[str, str] = {}

    def visit(shaping: DirectionShaping | None) -> None:
        if shaping is None or shaping.distribution is None:
            return
        if shaping.distribution in _BUILTIN_DISTRIBUTIONS or shaping.distribution_file is None:
            return
        existing = found.get(shaping.distribution)
        if existing is not None and existing != shaping.distribution_file:
            raise ValueError(
                f"distribution {shaping.distribution!r} maps to two files: {existing!r} and "
                f"{shaping.distribution_file!r}"
            )
        found[shaping.distribution] = shaping.distribution_file

    if profile.is_timeline:
        for segment in profile.timeline:
            visit(segment.uplink)
            visit(segment.downlink)
    else:
        visit(profile.uplink)
        visit(profile.downlink)
    return sorted(found.items())


# Selecting nothing — the unshaped rung (0/1) or the "no emulation" sentinel.
PROFILE_NONE = "none"


def resolve_profile_selection(
    requested: str | None,
    *,
    shared_default: str | None = None,
    available: Collection[str] | None = None,
    allow_shaping: bool = True,
) -> str | None:
    """Resolve the active profile name (network profiles reference selection rule).

    ``--profile <name>`` wins; otherwise ``shared.profile`` is the default; ``none``
    (or nothing) is the unshaped rung → ``None``. A requested name is validated
    against ``available`` when a profiles file is configured (catching typos). A
    profile selected on a run that may not be shaped — the real fleet, where the link
    *is* the condition and is measured, never imposed — is an error."""
    name = requested if requested is not None else shared_default
    if name is None or name == PROFILE_NONE:
        return None
    if not allow_shaping:
        raise ValueError(
            f"profile {name!r} selected, but this run may not be shaped: the real fleet's link is the "
            "condition and is measured, never imposed (network profiles reference). Use --profile none."
        )
    if available is not None and name not in available:
        raise ValueError(f"unknown profile {name!r}; configured profiles: {sorted(available)}")
    return name


# --------------------------------------------------------------------------- #
# tc / netem command generation (argv lists — no shell)
# --------------------------------------------------------------------------- #


def _ms(value: float) -> str:
    return f"{value:g}ms"


def _pct(value: float) -> str:
    return f"{value:g}%"


def _netem_args(shaping: DirectionShaping) -> list[str]:
    """The netem parameter tokens (delay/jitter/distribution, loss, duplicate, reorder).

    Ordered so each modifier follows the base it qualifies (netem requires
    ``delay`` before ``reorder``, and the correlation value follows its base loss).
    """
    args: list[str] = []
    if shaping.delay_ms is not None:
        args += ["delay", _ms(shaping.delay_ms)]
        if shaping.jitter_ms is not None:
            args.append(_ms(shaping.jitter_ms))
            if shaping.distribution is not None:
                args += ["distribution", shaping.distribution]
    if shaping.loss_pct is not None:
        args += ["loss", _pct(shaping.loss_pct)]
        if shaping.loss_correlation_pct is not None:
            args.append(_pct(shaping.loss_correlation_pct))
    if shaping.loss_gemodel is not None:
        gem = shaping.loss_gemodel
        # netem's positional names are p, r, 1-h, 1-k: the 3rd/4th arguments ARE
        # the loss probabilities in bad/good state. All four always emitted.
        args += [
            "loss",
            "gemodel",
            _pct(gem.p_pct),
            _pct(gem.r_pct),
            _pct(gem.loss_bad_pct),
            _pct(gem.loss_good_pct),
        ]
    if shaping.duplicate_pct is not None:
        args += ["duplicate", _pct(shaping.duplicate_pct)]
    if shaping.reorder_pct is not None:
        args += ["reorder", _pct(shaping.reorder_pct)]
    if shaping.seed is not None:
        args += ["seed", str(shaping.seed)]
    return args


def _tbf_stage_command(interface: str, shaping: DirectionShaping | None) -> list[str]:
    """``tc qdisc replace`` for the tbf root stage (``1:``) — in place when it
    already exists. A step without a rate keeps the stage present but effectively
    unlimited (:data:`_UNSHAPED_TBF_ARGS`), so the tree shape never changes."""
    if shaping is not None and shaping.rate_bps is not None:
        args = [
            "rate",
            f"{int(round(shaping.rate_bps))}bit",
            "burst",
            shaping.burst,
            "latency",
            _ms(shaping.tbf_latency_ms),
        ]
    else:
        args = list(_UNSHAPED_TBF_ARGS)
    return ["tc", "qdisc", "replace", "dev", interface, "root", "handle", "1:", "tbf", *args]


def _netem_stage_command(interface: str, netem_args: Sequence[str], *, child_of_tbf: bool) -> list[str]:
    """``tc qdisc replace`` for the netem stage (``10:``) — in place when it
    already exists. Empty ``netem_args`` is a pass-through netem; a netem change
    replaces the whole parameter set, so anything omitted is reset, not inherited."""
    location = ["parent", "1:"] if child_of_tbf else ["root"]
    return ["tc", "qdisc", "replace", "dev", interface, *location, "handle", "10:", "netem", *netem_args]


def shaping_commands(interface: str, shaping: DirectionShaping) -> list[list[str]]:
    """The ordered ``tc`` argv list that arms ``shaping`` on ``interface`` egress.

    Layout mirrors the calibrated baseline: a ``tbf`` root for rate limiting with a
    child ``netem`` for delay/jitter/loss. When only one of the two is requested it
    becomes the root qdisc; an empty shaping arms nothing. Emitted as ``tc qdisc
    replace`` (add-or-change), so re-arming over an existing tree of the same
    shape updates it in place instead of failing.
    """
    if shaping.is_empty:
        return []
    commands: list[list[str]] = []
    if shaping.rate_bps is not None:
        commands.append(_tbf_stage_command(interface, shaping))
        if shaping.has_netem:
            commands.append(_netem_stage_command(interface, _netem_args(shaping), child_of_tbf=True))
    elif shaping.has_netem:
        commands.append(_netem_stage_command(interface, _netem_args(shaping), child_of_tbf=False))
    return commands


def teardown_command(interface: str) -> list[str]:
    """Idempotent root-qdisc removal. Always safe to run; non-zero exit (no qdisc
    present) is expected and tolerated by the caller — it must *always* run on
    teardown so a crashed run cannot leave a ``qdisc`` shaping later results."""
    return ["tc", "qdisc", "del", "dev", interface, "root"]


def restore_link_command(interface: str) -> list[str]:
    """Bring the interface back up — undoes a ``reconnect`` outage. Idempotent on an
    already-up interface, so it is safe to run unconditionally on teardown."""
    return ["ip", "link", "set", "dev", interface, "up"]


def safety_teardown_command(interface: str, max_duration_s: float) -> list[str]:
    """A self-contained watchdog that reverts ``interface`` after ``max_duration_s``.

    The hard fail-safe (network profiles reference): launched **detached** at arm time, it survives an
    orchestrator crash and still tears the qdisc down (and brings the link back up),
    so a killed run cannot leave shaping on the machine. The detachment itself is the
    launcher's job; this builds the inner argv it runs."""
    iface = shlex.quote(interface)
    seconds = max(1, int(round(max_duration_s)))
    return ["sh", "-c", f"sleep {seconds}; tc qdisc del dev {iface} root; ip link set dev {iface} up"]


# --------------------------------------------------------------------------- #
# Timeline expansion (segments → a stepping schedule)
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class TimelineStep:
    """A timeline segment placed on the absolute run clock: hold from ``start_s``
    to ``end_s``. ``commands`` arms the step; ``outage`` names the kind if any."""

    index: int
    start_s: float
    end_s: float
    outage: str | None
    commands: list[list[str]]


def expand_timeline(
    profile: Profile,
    interface: str,
    *,
    direction: str = "uplink",
) -> list[TimelineStep]:
    """Expand a timeline profile into absolute-clock steps for the given direction.

    Steps transition **seamlessly**: every step ``tc qdisc replace``\\ s the *same*
    qdisc tree in place instead of tearing it down and re-adding it — a del+add
    drops everything queued inside netem and leaves a brief unshaped window, which
    shows up as lost and under-delayed messages at every boundary (even between
    identical segments). The tree shape is therefore fixed for the whole timeline,
    decided up front: a tbf root (``1:``) with a netem child (``10:``) when any
    segment rate-limits, else a netem root (``10:``). A step that does not use a
    stage arms it as a pass-through (unlimited tbf / bare netem) rather than
    removing it; a direction that never shapes (and no outages) arms nothing.

    Outage segments fold into the same tree: ``catchup`` is an in-place
    ``netem loss 100%`` (interface stays up — seamless entry and exit, DDS
    endpoints survive → recovery is "catch up"); ``reconnect`` downs the link
    (disruptive by design — forces RMW re-discovery) and the following step
    restores it first. The stepping driver runs ``commands`` at ``start_s``; the
    recovery genre (benchmark commands reference) reads ``outage`` to know which restore semantics a
    given ``end_s`` represents.
    """
    if not profile.is_timeline:
        raise ValueError(f"profile {profile.name!r} is not a timeline profile")
    if direction not in ("uplink", "downlink"):
        raise ValueError(f"direction must be 'uplink' or 'downlink', got {direction!r}")

    shapings = [segment.uplink if direction == "uplink" else segment.downlink for segment in profile.timeline]
    rated = any(shaping is not None and shaping.rate_bps is not None for shaping in shapings)
    shaped = (
        rated
        or any(shaping is not None and shaping.has_netem for shaping in shapings)
        or any(segment.outage is not None for segment in profile.timeline)
    )

    steps: list[TimelineStep] = []
    clock = 0.0
    previous_outage: str | None = None
    for index, segment in enumerate(profile.timeline):
        start_s, end_s = clock, clock + segment.for_s
        commands: list[list[str]] = []
        if previous_outage == OUTAGE_RECONNECT and segment.outage != OUTAGE_RECONNECT:
            commands.append(restore_link_command(interface))
        if segment.outage == OUTAGE_RECONNECT:
            commands.append(["ip", "link", "set", "dev", interface, "down"])
        elif shaped:
            if segment.outage == OUTAGE_CATCHUP:
                shaping: DirectionShaping | None = None
                netem_args: list[str] = ["loss", "100%"]
            else:
                shaping = shapings[index]
                netem_args = _netem_args(shaping) if shaping is not None else []
            if rated:
                commands.append(_tbf_stage_command(interface, shaping))
            commands.append(_netem_stage_command(interface, netem_args, child_of_tbf=rated))
        steps.append(TimelineStep(index=index, start_s=start_s, end_s=end_s, outage=segment.outage, commands=commands))
        clock = end_s
        previous_outage = segment.outage
    return steps
