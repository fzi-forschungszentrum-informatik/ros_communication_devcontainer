# Validation

This snapshot contains local checks and tests. It does not contain GitHub
Actions workflows. Run the checks from a source checkout:

```bash
just setup
just check
```

| Check | What it verifies |
|---|---|
| `just lint` | Python lint, format and syntax |
| `just lint-build` | Shell scripts and packaged runtime resources |
| `just typecheck` | Host CLI type checks |
| `just test-nondocker-cov` | Unit and contract tests with a coverage floor |
| `just docs` | README examples, links and findings structure |
| `just package` | Source and wheel build, metadata and installation smoke |

`just check` runs these checks. It does not contact remote hosts or run
communication sessions. Python 3.10 through 3.14 are supported; a check on one
interpreter does not establish that the full interpreter matrix passed.

Docker tests exercise generated sessions and runtime nodes:

```bash
just test-e2e-slice heartbeat
just test-e2e-slice runtime-tools
just test-e2e-smoke
```

The slice registry is in `tests/e2e/conftest.py`; `just e2e-slice-costs`
lists its contents. Tests that require two physical machines or recorded bags
need those resources and explicit deployment configuration.

The [field-profile replay](../field-profiles/2026-08/README.md) is a separate
Docker experiment. It checks the fitted loss model using a published frame
plan. It does not test ROS discovery or codec decoding.

See [testing](testing.md) for the test taxonomy and
[paper artifacts](paper-artifacts.md) for the released measurement components.
