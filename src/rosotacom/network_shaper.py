# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""network profiles reference — fail-safe profile arming on a real OTA interface (privileged half).

The pure schema/argv lives in :mod:`rosotacom.network_profiles`; this is the
controller that *arms* those commands on a live interface with the hard fail-safe
network profiles reference demands — the highest-risk item, because a stuck ``qdisc`` silently
corrupts every later result on the machine:

* **revert on stop and on error** (a context manager that tears down on normal exit
  *and* on an exception, without suppressing it);
* a **detached safety watchdog** that reverts after a max duration even if the
  orchestrator crashes mid-run;
* an **idempotent teardown** that always runs (clears the root qdisc and brings the
  link back up, tolerating a missing qdisc / already-up link);
* a **refusal to shape the SSH/control interface** — shaping it would cut the very
  orchestration applying the profile.

Command execution is injected (a ``CommandRunner``), so the guard, the arm/teardown
ordering and the revert-on-exception are all host-testable with a fake runner; the
live adapter merely runs the argv over SSH/subprocess.
"""

from __future__ import annotations

import re
from collections.abc import Callable, Iterable, Sequence
from types import TracebackType
from typing import Literal

from rosotacom.network_profiles import (
    restore_link_command,
    safety_teardown_command,
    teardown_command,
)

# Runs one argv; raises on failure (e.g. subprocess.CalledProcessError).
CommandRunner = Callable[[Sequence[str]], None]


def ota_interface_from_route(route_output: str) -> str:
    """Egress interface from ``ip route get <peer-address>`` output.

    The profile shapes the interface that actually carries traffic to the peer —
    found by asking the kernel which ``dev`` the route to the peer's *data* address
    uses, never guessed. This is also how a caller checks the OTA interface differs
    from the SSH/control one before arming."""
    match = re.search(r"\bdev\s+(\S+)", route_output)
    if not match:
        raise ValueError(f"no egress interface ('dev <if>') in route output: {route_output!r}")
    return match.group(1)


class ProfileShaper:
    """Arms profile shaping on one interface with a guaranteed revert."""

    def __init__(
        self,
        interface: str,
        runner: CommandRunner,
        *,
        control_interface: str | None = None,
        safety_max_duration_s: float | None = None,
        watchdog_launcher: CommandRunner | None = None,
    ) -> None:
        if not interface:
            raise ValueError("interface must be a non-empty name")
        if control_interface is not None and interface == control_interface:
            raise ValueError(
                f"refusing to shape {interface!r}: it is the SSH/control interface — shaping it would "
                "cut the orchestration applying the profile (target the data interface only)"
            )
        self.interface = interface
        self._runner = runner
        self._safety_max_duration_s = safety_max_duration_s
        self._watchdog_launcher = watchdog_launcher
        self.armed = False

    def _run(self, argv: Sequence[str]) -> None:
        self._runner(list(argv))

    def teardown(self) -> None:
        """Always-safe revert: clear any root qdisc and bring the link back up.

        Tolerates failure of either step — a missing qdisc or an already-up link is
        the expected case, not an error — so teardown can run unconditionally."""
        for argv in (teardown_command(self.interface), restore_link_command(self.interface)):
            try:
                self._run(argv)
            except Exception:
                pass  # idempotent revert: nothing to undo is success, not failure
        self.armed = False

    def arm(self, commands: Iterable[Sequence[str]]) -> None:
        """Arm ``commands`` from a clean slate, launching the safety watchdog first.

        Order matters: the detached watchdog is launched *before* the shaping, so a
        crash at any point after this still reverts the interface."""
        self.teardown()  # clean slate (idempotent)
        if self._safety_max_duration_s is not None and self._watchdog_launcher is not None:
            self._watchdog_launcher(safety_teardown_command(self.interface, self._safety_max_duration_s))
        for argv in commands:
            self._run(argv)
        self.armed = True

    def apply(self, commands: Iterable[Sequence[str]]) -> None:
        """Run ``commands`` as-is, without a fresh clean-slate or watchdog.

        Used to step a timeline: each :func:`~rosotacom.network_profiles.expand_timeline`
        step updates the qdisc tree **in place** (``tc qdisc replace``), so there is
        no teardown between steps — a del+add would drop netem's queue and leave a
        brief unshaped window at every boundary. The watchdog is launched once via
        :meth:`arm` at the start of the run, not per step; a failing step command
        raises (the context manager still reverts)."""
        for argv in commands:
            self._run(argv)
        self.armed = True

    def __enter__(self) -> ProfileShaper:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> Literal[False]:
        self.teardown()
        return False  # never suppress: an error during the run still propagates
