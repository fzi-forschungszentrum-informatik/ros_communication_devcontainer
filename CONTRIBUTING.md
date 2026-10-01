# Contributing

Use Python 3.10 through 3.14, Git and `just`. Docker is required for runtime tests.

```bash
just setup
just check
```

Setup creates a local virtual environment, installs the development dependencies
and configures pre-commit hooks. The checks cover lint, typing, unit and contract
tests, documentation links, source distributions and wheel installation.

Run the relevant Docker tests when changing session generation or runtime nodes:

```bash
just test-e2e-slice heartbeat
just test-e2e-slice runtime-tools
```

The complete local matrix is `just test-e2e-smoke`. It starts containers and can
take considerably longer than the host checks. See [testing.md](docs/testing.md)
and [validation](docs/ci.md) for the test boundaries.

Submit changes through a pull request. Include the affected behavior, tests run,
and any remaining limitation. Update the command examples when the interface
changes. See [Development Principles](DEVELOPMENT_PRINCIPLES.md).

Release snapshots are reviewed by the maintainer before publication. A Git tag
identifies the installable version; publishing a snapshot does not publish a
package to PyPI.
