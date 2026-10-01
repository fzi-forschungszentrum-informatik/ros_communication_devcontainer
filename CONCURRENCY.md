# Concurrency: what can run in parallel

`rosotacom` is designed so you never have to reason about leftover state before
starting something: everything that can safely run in parallel just works, and
anything that cannot aborts immediately — before allocating networks or staging
workspaces — with the list of conflicting containers and the exact command to
stop them.

Run `rosotacom ps` at any time to see this workspace's active containers,
classified by whether they block a new run.

## Concurrency matrix

| Target type | Supported in parallel? | Reason / boundary |
| :--- | :--- | :--- |
| **Local smoke tests** | **Yes** (different targets or workspaces) | Containers are instance-scoped and isolated inside separate Docker bridge networks. A second run of the *same target in the same workspace* aborts — it is almost always an accidental double start. |
| **Local sessions / scenarios** | **Yes** (different identities) | Starting `--identity a` and `--identity b` side by side is the normal two-peer local workflow. A second run for the *same identity* conflicts on the host network; the default `--force` replaces it (printed), `--no-force` aborts. |
| **Local benchmarks** | **No** (strongly discouraged) | Shared host CPU and virtual network queue contention inject scheduling jitter and distort latency/loss metrics. The benchmark aborts if *any* rosotacom container is running on the host, in any workspace. |
| **OTA (remote) smoke tests** | **No** | Multiple sessions cannot share the same physical hosts and tunnel interfaces simultaneously. |
| **OTA (remote) benchmarks** | **No** | Requires exclusive control of the peer hosts and the data interface `qdisc` to measure baseline performance. |

## How parallelism works

Every run allocates a short random **instance id** at startup (or joins one via
`--instance-id`). That id is baked into every dynamically allocated resource:

- **Container names** — `rosotacom_{install_id}_{instance_id}_com_to_{peer}` for
  communication containers and
  `rosotacom_{install_id}_{instance_id}_scenario_{scenario}_{identity}_{app}`
  for scenario applications. Two runs can never collide on a name, and one run
  can never stop another run's containers by accident.
- **Smoke networks and subnets** — smoke and benchmark peers live on a
  per-instance Docker bridge network with a subnet derived from the instance
  token, and the network carries labels (`rosotacom.install`,
  `rosotacom.target`) that identify which target it belongs to.

Because names are per-run, `stop`, `smoke --stop`, `scenario stop`, and attach
commands *discover* the matching containers from Docker instead of recomputing
fixed names — they clean up every instance for the requested identity/target,
including leftovers from crashed runs.

## Opting out: fixed, human-readable names

A project whose machines are read by people rather than by CI — a demo or
operations project — can set `com_container_prefix` in its `rosotacom.yaml`:

```yaml
com_container_prefix: remote-assist
```

Session communication containers are then named
`<prefix>_com-to-<remote-peer>` (e.g. `remote-assist_com-to-center`): stable
across runs and legible in `docker ps`, at the price of parallelism — a second
start of the same session on the same host is a name conflict. That conflict is
detected in the same preflight as everything else: `--force` (the session
default) replaces the running container, `--no-force` aborts with its name.
Conflict discovery matches **both** naming schemes, so a leftover
instance-scoped container is still found and replaced after a project switches
modes. Test and CI projects should leave the key unset and keep the
collision-free instance-scoped names.

## How conflicts are detected (fail-safe preflight)

Each start command checks Docker before it allocates anything, and aborts with
a message of this shape:

```
rosotacom: error: A smoke run for 1_heartbeat is already active in this workspace.
Active containers:
  - rosotacom_49b74744_a1b2c3d4_com_to_a
  - rosotacom_49b74744_a1b2c3d4_com_to_b
Stop it first with: rosotacom smoke 1_heartbeat --stop ...
```

- **`start` (sessions, scenario peers)** — conflicts only with a running
  communication container for the same identity in the same workspace
  (discovered by the `_com_to_{peer}` name pattern, ignoring smoke-isolated
  containers). `--force` (the default) stops the conflicting containers and
  proceeds; `--no-force` aborts.
- **`smoke` (local, interactive and non-interactive)** — conflicts only with an
  active smoke run of the same target in the same workspace, detected through
  the labelled smoke networks and their attached containers. Leftover networks
  without running containers are not conflicts.
- **`benchmark` (local)** — conflicts with any running `rosotacom_*` container
  on the host, across all workspaces, because host contention corrupts the
  measurement even when the runs are otherwise isolated.
- **`ota-smoke` / `ota-benchmark`** — during SSH preflight each peer is checked
  for running `rosotacom_*` containers and for active `netem`/`tbf`/`htb`
  shaping on the data interface toward the other peer (`tc qdisc show`); either
  aborts the run.

Every check has an escape hatch: `--skip-conflict-check` (and OTA's broader
`--skip-preflight`) proceeds anyway. Use it only when you know why the check
fired.
