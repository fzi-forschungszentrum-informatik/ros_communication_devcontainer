# Development Principles

## Definition Of Done

A change is complete only if:

1. Public behavior is implemented.
2. Unit or contract tests cover feasible host behavior.
3. Docker/E2E tests cover Docker runtime behavior.
4. README, docs, examples, and packaging metadata are updated when public behavior changes.
5. `just check` and relevant Docker checks pass.
6. Obsolete behavior is removed unless compatibility is explicitly requested.

## Compatibility Policy

Preserve documented CLI behavior. Do not add silent aliases for removed behavior
unless a migration explicitly requires them.

`rosotacom` is the only console script this package installs. The
`start_rosotacom` / `stop_rosotacom` entry points were the one standing
exception to rule 6 above; they were removed on 2026-08-14 because they were
nothing but `rosotacom start` and `rosotacom stop` under a second name, and a
second name is what makes a stale shim on a machine indistinguishable from a
live command. `rosotacom doctor` reports one that is still lying around.

## Testing Policy

Test public behavior and contracts.

Prefer:

- CLI parsing and dry-run-style dispatch tests,
- config path and environment precedence tests,
- packaged-resource and wheel smoke tests,
- docs and README example checks,
- Docker smoke tests for runtime behavior.

Avoid:

- broad mocks around pure path logic,
- tests that duplicate production code,
- hidden skips or xfails without an explicit reason.

## Docker Policy

Docker/runtime behavior requires `just test-e2e-fast` or a documented reason it
cannot sensibly be tested automatically.

Downloaded external binaries and source inputs should be versioned and, where
practical, checksum-verified in the lower-level `ros2docker` image layer.

## Dependency Policy

New dependencies require a clear runtime or dev-only reason, a license/security
check, and an explanation for why stdlib or existing dependencies are not enough.
