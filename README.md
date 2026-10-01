# ROS Communication DevContainer

[![Project code license](https://img.shields.io/badge/project_code-BSD--3--Clause-blue.svg)](docs/licenses.md)

rosotacom transfers selected ROS 2 topics between two Linux machines using
Docker containers, DDS or Zenoh. Session files configure compression, topic
mapping, QoS and optional separation of local and OTA ROS domains.

For the cellular remote-assistance paper, start with the
[paper artifacts](docs/paper-artifacts.md): per-message instrumentation,
network traces, fitted profiles and the executable frame-loss replay.

<details>
<summary>Key Features</summary>

- **Deployment**: The host CLI uses Python and Docker; ROS runs in the containers.
- **Isolation**: Operates in a separate Docker container, ensuring minimal impact on existing ROS setups.
- **Centralized Configuration Management**: All configurations are stored and managed in this repository.
- **Compression**: Built-in compression capabilities for efficient data transfer.
- **QoS Configuration**: Flexible Quality of Service settings for optimized communication.

</details>

## Getting Started

### Prerequisites

- Docker installed on all machines
- Git for configuration management
- Python 3 with virtual environment support (`python3-venv` on Debian/Ubuntu;
  for versioned Python packages this may be named like `python3.14-venv`)
- `ros2docker` v0.1.5.dev10 or newer (below v0.2). The local installer below installs the pinned
  supported range into this checkout's virtual environment.
- `tmux` for the optional `rosotacom scenario` orchestration commands
- Machines connected to the same network (VPN or local WLAN)

### Convenience CLI: `rosotacom`

This repository's main entrypoint for starting ROS communication sessions is
the checkout-local `rosotacom` command. Each checkout owns its own `.venv`,
so multiple rosotacom versions can coexist without global symlink drift.

#### Install

Use a released version, or develop from a checkout — both with off-the-shelf
tooling, nothing rosotacom-specific:

```bash
# from this repository (append @<tag> for a fixed snapshot)
pipx install "git+https://github.com/fzi-forschungszentrum-informatik/ros_communication_devcontainer.git"

# from a source checkout (developers)
cd /path/to/ros_communication_devcontainer
./install.sh                      # builds a checkout-local .venv
source .venv/bin/activate         # use it in this terminal
#   ./install.sh --global-symlink # optional: put it on PATH via ~/.local/bin

rosotacom --version
python -m rosotacom --version
rosotacom doctor
```

The version comes from the repository's tags (`setuptools-scm`), so
`rosotacom --version` names the exact snapshot a result was produced with.

For a source checkout, that single `source .venv/bin/activate` command also
enables completion. Nothing needs to be added to your shell configuration.

For a pipx/global installation, add one line once to your shell startup file:

```bash
# zsh: add this line to ~/.zshrc
eval "$(rosotacom completion zsh)"

# bash: add this line to ~/.bashrc
eval "$(rosotacom completion bash)"
```

Every new terminal then loads completion automatically; you do not run the
command manually in each terminal. After reloading the shell,
`rosotacom <TAB><TAB>` lists commands,
`rosotacom smoke <TAB><TAB>` lists sessions from the active project, and a
prefix such as `rosotacom smoke 1<TAB>` expands to `1_heartbeat`. The same
session completion is available for `start`, `stop`, `status`, `test`, and the
probe commands; absolute and relative session-directory paths still complete
normally. `--identity <TAB><TAB>` lists the peer identities from the selected
session or scenario. `--peer <TAB><TAB>` completes logical peer keys, and
`--peer a=<TAB><TAB>` completes named hosts from the active deployment file.
`--peer-address <TAB><TAB>` completes logical peer keys for raw overrides.
Scenario `attach` and `stop` complete only active scenarios and identities.
Running `rosotacom completion` without a shell argument infers bash or zsh from
`$SHELL`.

Completion is not pinned to the version that registered it: each Tab press runs
the `rosotacom` currently selected by `PATH`. Activating another checkout's
virtual environment therefore switches both the command and its completion,
and `deactivate` switches back without a reset step. A pipx-suffixed executable
can be registered independently with, for example,
`eval "$(rosotacom@2.2.0 completion zsh)"`.

**Multiple versions / try-without-disturbing.** Because each checkout owns its
own `.venv` and `./install.sh` never touches your PATH on its own, you can keep a
pinned/production version in use while smoke-testing another in one terminal:

```bash
cd ~/checkouts/rosotacom-newest   # a second checkout (or git worktree)
./install.sh                      # builds .venv here; your other rosotacom is untouched
source .venv/bin/activate         # this terminal now runs the new commit
rosotacom smoke                   # built-in example, zero config
deactivate                        # back to your usual rosotacom
```

For coexisting versions, `pipx install "git+https://github.com/fzi-forschungszentrum-informatik/ros_communication_devcontainer.git@<tag>" --suffix=@<tag>`
gives you a `rosotacom@<tag>` alongside the default — standard pipx, no bespoke
version manager.

### Basic Setup

`rosotacom` uses five layers of configuration/runtime state:

- A **project setup** file (`rosotacom.yaml`) points to host-local resources such as `ros2docker.json`, static `sessions/`, ignored `session-instances/`, and an optional deployment file.
- A **session config** defines the communication behavior for one run: logical peers, topics, QoS, processing, and transport choices.
- An optional **scenario config** composes one communication session with identity-specific local application containers.
- An optional **deployment config** maps named physical hosts to reachable addresses and optional SSH targets.
- A **session instance** stores one concrete run: generated config, catmux pane logs, smoke debug output, and future rosbags.

`rosotacom` resolves the active `rosotacom.yaml` by scope (first wins), using the
`shell` / `local` / `global` vocabulary from `pyenv`:

1. `--project <path>` (alias of `--rosotacom-config`) on the command
2. **shell** — `ROSOTACOM_CONFIG` in the environment (needs `eval`, since only
   your shell can change its own environment)
3. **local** — the nearest `rosotacom.yaml` discovered upward from the cwd
4. **global** — a machine-wide default (`rosotacom config set project ... --global`)
5. a built-in example project, so a fresh install runs with zero setup

The built-in example is used **in place** (read-only) and writes nothing to
`$HOME`: its generated session output goes to a per-user tmpfs dir under
`$XDG_RUNTIME_DIR` (printed on `start`/`smoke`), so it is ephemeral. For
persistent, editable projects, create one with `rosotacom examples create`.

So the simplest workflow is just to be in a project directory (the **local**
scope — no command needed, just have the file):

```bash
rosotacom examples create ./rosotacom_examples
cd ./rosotacom_examples
rosotacom doctor                # picks up ./rosotacom.yaml automatically
```

Inspect or pin the selection with `rosotacom config`:

```bash
rosotacom config show                                            # every scope + the winner
rosotacom config set project ./rosotacom.yaml --global           # machine-wide default
eval "$(rosotacom config set project ./rosotacom.yaml --shell)"  # this terminal only
```

The copied packaged example project uses this layout:

```text
rosotacom.yaml
ros2docker.json
ros2docker.lyrical.json
deployment.example.yaml
sessions/
scenarios/
session-instances/
scripts/
```

`session_configs_dir` and `scenario_configs_dir` are ordered YAML lists. Put
project-local directories first and shared/example directories later when one
project should discover both.

`com_container_prefix` (optional) trades concurrency for readability: when set,
session communication containers get the stable name
`<prefix>_com-to-<remote-peer>` (e.g. `remote-assist_com-to-center`) instead of
the default workspace- and instance-scoped unique name. Set it in projects whose
machines are read by people (`docker ps` during a demo, next to colleagues'
containers); leave it unset in test/CI projects, where concurrent instances must
never collide on a name. [CONCURRENCY.md](CONCURRENCY.md) has the exact
conflict semantics.

### Locating packaged resources

rosotacom is installed in an isolated environment (pipx, or a project venv), so
another tool cannot `import rosotacom` from a system Python to find the files
that ship with it. Ask the CLI instead — it prints one absolute path, so it
composes directly in a shell:

```bash
rosotacom resources path com_msgs   # the ROS 2 message package
rosotacom resources path ws         # the packaged runtime workspace
rosotacom resources path ros2docker-lyrical  # the opt-in ROS 2 Lyrical image config
```

Use this wherever another project needs rosotacom's files on the host, for
example to bake `com_msgs` into its own container image. Ask for the resource by
name rather than reconstructing a path: the names are part of the CLI contract,
the layout behind them is not.

### ROS 2 distribution variants

ROS 2 Kilted remains the default. The packaged Lyrical/Ubuntu Resolute image is
an explicit opt-in and uses a separate image name, so trying it does not replace
the default image. To select it for one command, including the built-in example:

```bash
ROSOTACOM_ROS2DOCKER_CONFIG="$(rosotacom resources path ros2docker-lyrical)" \
  rosotacom smoke
```

An example project copied with `rosotacom examples create` also contains
`ros2docker.lyrical.json`. To opt that project in persistently, select it in
`rosotacom.yaml`:

```yaml
ros2docker_config: ros2docker.lyrical.json
```

The Lyrical config keeps every available ROS dependency on the official binary
packages. `domain_bridge` is the current exception: because no Lyrical binary
package exists, ros2docker's `domain-bridge` profile builds a pinned upstream
commit with a pinned compatibility patch. The build fails closed outside
Lyrical or if that patch no longer applies. Replace this fallback with
`ros-lyrical-domain-bridge` once the official package is released.

The communication container and any scenario application with a matching
`ros2docker_config_by_distro` entry follow this variant selection together.
The packaged chatter scenario supplies Lyrical application configs, because
[ROS 2 does not guarantee cross-distribution communication](https://docs.ros.org/en/ros2_documentation/rolling/Releases.html#cross-distribution-communications).
Applications without a matching entry retain their explicit default config.

The default switch is intentionally not part of this change. Before promoting
Lyrical, keep the two-variant E2E and RMW matrices green, point the packaged
default and example project at the Lyrical configs, and recalibrate the
distribution-specific performance budgets. Existing copied projects continue
to name their own config and do not change when the package default changes.

### Reusing an image instead of rebuilding it

Somewhere that starts many rosotacom sessions from the same tree — CI above all
— can build each image once, publish it, and have every later run pull it:

```bash
# What this project's images are called: <repository>:<hash of the build inputs>
rosotacom image references --repository ghcr.io/owner/rosotacom-e2e

# Build the one a reference names, then push it
rosotacom image build --reference ghcr.io/owner/rosotacom-e2e:<hash>
docker push ghcr.io/owner/rosotacom-e2e:<hash>

# Adopt published images anywhere rosotacom would otherwise build
export ROSOTACOM_IMAGE_CACHE=ghcr.io/owner/rosotacom-e2e
```

The tag is a hash of everything that decides the image — base image and its
pinned digest, apt and pip packages, the Dockerfile and entrypoint ros2docker
stages, the build UID — so a build-input change is a different tag and a
published image can never be stale for the tree that adopts it. Nothing accepts
a reference: rosotacom derives the one its own inputs allow and pulls exactly
that, and fails rather than quietly rebuilding if it is not there. With
`ROSOTACOM_IMAGE_CACHE` unset, nothing changes. See
[docs/ci.md](docs/ci.md) for how the merge gate uses this.

See the [example project README](src/rosotacom/resources/examples/README.md)
for the copyable example layout, the
[session configuration reference](session-configuration.md) for the session
schema, the [deployment configuration reference](deployment-configuration.md) for
the machine-specific schema and peer-binding precedence,
[terminology.md](terminology.md) for the project vocabulary, and
[CONCURRENCY.md](CONCURRENCY.md) for what can run in parallel and how
conflicting runs are detected and aborted.

The normal examples intentionally contain no machine addresses. `rosotacom
smoke` injects isolated Docker-network addresses, while manual and OTA runs
select physical hosts explicitly.

Write or edit session configs under `sessions/<name>/`:

- `session-definition.yaml` for a self-contained session
- `session-parametrization.yaml` for a template plus parameters

Run `rosotacom` on each peer with the same active setup but a different identity:

```bash
# on peer "a"
rosotacom start 1_heartbeat --identity a \
  --peer-address a=10.0.0.10 --peer-address b=10.0.0.11

# on peer "b"
rosotacom start 1_heartbeat --identity b \
  --peer-address a=10.0.0.10 --peer-address b=10.0.0.11
```

Before `start` stops a conflicting container, writes an instance, or builds an
image, it verifies that the selected identity's address belongs to this host and
that the route to the other peer selects that address as its source. A missing
VPN/data-plane interface or a route through the wrong network therefore fails
without disturbing a working session. Run the same check on its own with
`preflight`; deployments that require the other host to be online already can
also make three bounded ICMP probes mandatory:

```bash
rosotacom preflight 1_heartbeat --identity a \
  --peer-address a=10.0.0.10 --peer-address b=10.0.0.11 \
  --require-peer-reachable
```

Peer reachability is opt-in because some valid networks block ICMP and peers may
be started in either order. `rosotacom start` accepts the same
`--require-peer-reachable` policy when a deployment does require it.

`rosotacom` reads the static session input and creates generated files under `session-instances/<date>/<session>_<timestamp>_<id>/config/`, including per-peer plugin/session specs, topic lists, optional QoS, and optional `domain_bridge.yaml`. The instance's `manifest.yaml` records what produced it: `rosotacom_version`, the effective config SHA, and `source_revision` — the commit, branch and tracked-dirtiness of the session source directory at creation time, so a recording stays attributable to a revision after the checkout has moved on (`null` when the source is not a git repository). Catmux pane output is logged under the same instance in `logs/<peer>/catmux/`.

What else a run leaves under `logs/<peer>/`, and which file to open first when a
peer looked healthy but did nothing:

| File | Written by | Read it for |
|---|---|---|
| `catmux/<NN>-<WINDOW>/<pane>.log` | `catmux_log_setup.sh` (`tmux pipe-pane`) | everything a node printed, including the traceback it died with |
| `pane_failures.log` | the same script's prompt hook | one line per non-zero pane exit — **the file that answers "is any pane dead?"** |
| `status/startup_check.json` | `status_overview`, once, after `status_startup_grace_s` and one confirming look a grace later | the verdict: which outbound stages this peer produces and never published, plus the pane failures above |
| `status/status.{json,txt}` and `status/events.jsonl` | `status_overview`, continuously | live per-stage state and the transit records |
| `scenario/<component>.log` | `rosotacom start <scenario>` | each scenario application's own output |
| `launcher.log`, `docker.log` | `rosotacom start` | the exact command, and the container's own stdout where it has one |

A pane whose command exits drops back to a shell prompt and keeps looking alive
— `docker ps` shows a healthy container and the other panes keep working. That
is how a control centre ran for two hours on 2026-08-13 with no heartbeat
publisher, because Cyclone had no free participant index left on the local
domain and `heartbeat_echo` died two seconds after start. The last two rows of
that table exist so the next one announces itself.

### Complete use cases with scenarios

A session intentionally describes only the communication contract. When a use
case also needs a local ROS application, rosbag, or ROS command, define a
scenario above it:

```yaml
schema_version: 1
session: 2_native_chatter

applications:
  a:
    - name: native_application
      ros2docker_config: ../../scripts/2_native_chatter/machine_a/external.ros2docker.json
      ros2docker_config_by_distro:
        lyrical: ../../scripts/2_native_chatter/machine_a/external.lyrical.ros2docker.json
  b:
    - name: native_application
      ros2docker_config: ../../scripts/2_native_chatter/machine_b/external.ros2docker.json
      ros2docker_config_by_distro:
        lyrical: ../../scripts/2_native_chatter/machine_b/external.lyrical.ros2docker.json
```

Start one identity's communication and local application together:

```bash
rosotacom scenario start 2_native_chatter --identity a
rosotacom scenario list
rosotacom scenario attach
rosotacom scenario stop
```

`scenario list` shows both configured scenarios and currently active
scenario/identity pairs. `attach` and `stop` infer omitted values when exactly
one active choice exists; otherwise their completions and error messages show
the eligible active choices. `ros2docker_config` is always the fallback;
`ros2docker_config_by_distro` selects a matching application image from the
communication config's `BASE_IMAGE` distro, so changing the project between
Kilted and Lyrical changes the complete scenario rather than creating an
unsupported mixed-distro data path.

The outer view uses an isolated host tmux server with one full window for
communication and one full window per local application. Its prefix remains
`Ctrl-b`; switch windows with `Ctrl-b n`/`Ctrl-b p`, and send the inner
container-side catmux prefix with `Ctrl-b Ctrl-b`. Detaching from the outer
tmux does not stop the use case. `scenario stop` owns cleanup of the application
containers, communication container, and outer tmux session.

### Safe Replay Testing with Anonymized Rosbags

To test scenarios involving proprietary data safely, first record the processed
outbound handoff topics that the session pipeline actually forwards into the OTA
path. `rosotacom anonymize` consumes that processed trace bag and the matching
session/scenario definition. It:

1. Resolves the session pipeline and selects the outbound handoff topics that feed
   `/com/out/...` and `/ota/...`.
2. Fails closed if the input bag only contains raw source topics and is missing
   the processed handoff topics.
3. Renames the handoff topics to generic names like `/topic1`, `/topic2`, etc.
4. Replaces message contents with mock payloads while preserving message shape,
   timestamps, payload lengths, bag QoS metadata, and playback QoS overrides.
   For ffmpeg camera packets it additionally preserves the keyframe flag and
   `pts`, keeping the GOP burst structure analyzable — see
   [docs/ffmpeg-keyframes.md](docs/ffmpeg-keyframes.md).
5. Generates a self-contained `rosotacom` project containing the anonymized bag,
   processed-topic session definition, replay scenario, QoS overrides, and an
   anonymization manifest.

```bash
rosotacom anonymize /path/to/processed_handoff_bag \
  -s 3_comp_occ_grid \
  -o ./anonymized_project
```

## Usage Examples

Create the example project first (cwd discovery wires it automatically once you
`cd` in):

```bash
rosotacom examples create ./rosotacom_examples
cd ./rosotacom_examples
```

Run the local heartbeat smoke test:

```bash
rosotacom smoke
```

To smoke-test one specific configured session locally, pass its name:

```bash
rosotacom smoke 1_heartbeat
```

The smoke test verifies both directions through the communication path: it waits
for `/com/in/a/heartbeat_a` and `/heartbeat_a` in peer `b`, plus
`/com/in/b/heartbeat_b` and `/heartbeat_b` in peer `a`. For each checked topic it
also reports a `SMOKE_METRIC` line with the received rate (`hz`) and end-to-end
latency (`delay_s`) so rate and latency regressions are visible. It prints the
`session-instances/...` artifact path so failures (and the per-peer
`logs/<peer>/catmux/...` pane output) can be inspected after the containers stop.

For local debugging, run the same end-to-end idea interactively:

```bash
rosotacom smoke 2_native_chatter --interactive
rosotacom smoke --interactive --list
rosotacom smoke 2_native_chatter --interactive --stop
```

Interactive smoke starts all peers on one host, opens an isolated outer tmux
session, and keeps full windows for each peer's communication container. If the
target is a scenario, it also opens one window per local application container;
each application shares its peer communication container's isolated network
namespace so local ROS discovery behaves like colocated processes. A
`verification` window runs the local checks/status view while you inspect the
system. Its upper pane keeps the verification log; its lower pane watches live
status. The outer prefix is `Ctrl-b`; use `Ctrl-b n`/`Ctrl-b p` for windows and
`Ctrl-b Ctrl-b` for the inner catmux session.

`rosotacom smoke TARGET --interactive` accepts either a session or a scenario.
When both names exist, interactive smoke treats `TARGET` as a scenario unless
you pass `--target-type session`. Non-interactive `rosotacom smoke TARGET`
keeps its existing session-only behavior.

Local smoke runs of different targets are parallel-safe: every run gets
instance-scoped container names and its own isolated Docker network. Starting a
run that cannot coexist with an active one (same smoke target, same session
identity, any benchmark) aborts immediately with the conflicting containers and
the stop command. `rosotacom ps` shows what is active in this workspace;
[CONCURRENCY.md](CONCURRENCY.md) documents the boundaries.

For a real two-host OTA check with no deployment file, provide both addresses
and only the SSH target needed to reach the remote peer:

```bash
rosotacom ota-smoke 2_native_chatter \
  --peer-address a=10.0.0.10 \
  --peer-address b=10.0.0.11 \
  --peer-ssh b=robot-b
```

For reusable host names, reference one deployment file from `rosotacom.yaml`:

```yaml
# rosotacom.yaml
deployment: deployment.yaml
```

```yaml
# deployment.yaml
hosts:
  workstation:
    address: 10.0.0.10
    ssh: null
  robot:
    address: 10.0.0.11
    ssh: robot-b
```

```bash
rosotacom ota-smoke 2_native_chatter \
  --peer a=workstation \
  --peer b=robot
```

`ota-smoke` accepts sessions and scenarios. It puts rosotacom on each peer,
stages the active project, runs delivery and isolation checks, collects
artifacts, stops the run, and removes the remote workdir. For scenario targets
the checks start only once every declared application container is running
(bounded at 10 minutes, covering a cold peer's first image build); an
application that never comes up fails the run by name instead of as missing
publishers. Add `--interactive`
for a local control tmux with
one attachable communication/catmux window per peer with a live status pane below
each one. Scenario targets also get one native application-container window per
scenario app. Stop an interactive run with `rosotacom ota-smoke TARGET
--interactive --stop`; pass the same peer/deployment arguments if the local tmux
metadata is not available. Use `--keep-running` to leave components up,
`--keep-workdir` to retain staged files, or `--reuse` to reuse an existing
installation.

#### What the peers run: `--install-mode`

```bash
# default: stage and install what you are running, uncommitted changes included
rosotacom ota-smoke 2_native_chatter --peer a=workstation --peer b=robot

# rehearse a deployment: the peers install the published artefact
rosotacom ota-smoke 2_native_chatter --peer a=workstation --peer b=robot \
  --install-mode pin                # defaults to your own version
rosotacom ota-smoke 2_native_chatter --peer a=workstation --peer b=robot \
  --install-mode pin --install-pin 2.4

# test the machines as they are: no staging, no install, their own checkouts
rosotacom ota-smoke 2_native_chatter --peer a=workstation --peer b=robot \
  --peer-ssh b=robot-b --install-mode checkout \
  --peer-checkout a=/home/me/fleet_mgmt --peer-checkout b=/home/robot/fleet_mgmt
```

`source` is right while iterating on rosotacom itself — the peers run the code
in front of you. `pin` is right when the run is meant to stand in for a real
deployment: the peers install from the index, so a packaging defect (a file
missing from the wheel, an undeclared dependency) fails in the rehearsal instead
of on the target machine. `source` cannot catch that class of defect by
construction, because it never builds the artefact that ships.

`checkout` answers a different question again: not "does this artefact work" but
"is *this machine* ready". Nothing is staged and nothing is installed — each
peer runs the rosotacom it already has, from the project checkout it already
has, so what is under test is the state git and the machine's own install left
behind, dependency pins included. Ground truth moves into the repository: push,
pull, run. It is also the only mode in which a project may reference anything
outside its own directory — staging copies the project directory, so a mount or
config path leading above it cannot survive the copy and fails on the peer as a
missing path.

Because the same repository sits at a different absolute path on every machine,
`--peer-checkout` is required for every peer; the path *inside* the repository
is taken from your own checkout, so the project has to live in one. Preparation
becomes verification: each peer is asked whether the checkout, the project file
and `rosotacom` are there, and each version is printed. `--cleanup` does nothing
in this mode — there is no staging to undo, and the workdir is your repository.

The run manifest records which mode and which version the peers ran, so a result
can be attributed to an artefact rather than to a working copy.

#### How the peers are reached: `--peer-exec`

`--peer-ssh` gives the orchestrator a shell on the peer, which is more than a
run needs and more than some machines should hand out. `--peer-exec` names the
transport instead:

```bash
rosotacom ota-smoke 2_native_chatter --peer a=workstation --peer b=robot \
  --install-mode checkout \
  --peer-checkout a=/home/me/fleet_mgmt --peer-checkout b=/home/robot/fleet_mgmt \
  --peer-exec b='remote-rosotacom robot'
```

The command is run with the script as its final argument — the contract
`ssh host <script>` already has — so anything of that shape can carry a peer: a
wrapper that logs what a run asks for, a container exec, or a forced command
that accepts some scripts and refuses the rest. rosotacom does not know or care
which; it only stops assuming that reaching a peer means having a shell on it.

Two restrictions, both load-bearing. It applies to `--install-mode checkout`
only, because the staging modes push a tar stream and a file write by taking
the ssh client's own argv apart, and a transport that promises no more than
"run this script" cannot carry them. And a peer takes either `--peer-ssh` or
`--peer-exec`, never both: leaving it ambiguous hides which one carried the run.

`-t` and `-o BatchMode=yes` are dropped rather than forwarded. They are options
of one client, and a transport that is not ssh never agreed to understand them.

The same four options — `--peer-checkout`, `--peer-exec`, `--install-mode`,
`--install-pin` — are on every `ota-benchmark` genre, not just `ota-smoke`. A
measurement is the last thing that should require a shell on the peer: the
whole point of a narrower transport is that the runs which touch a machine
repeatedly can go through it, and a benchmark touches it far more often than a
smoke test does.

### Live status / debugging overview

Enable a continuously-updated, per-topic pipeline overview by setting
`shared.use_status_overview: true` in the session definition (see the
ready-made `1_heartbeat_status` example). For every configured topic it tracks
where the topic currently is in the communication pipeline (furthest stage
reached and the first stage that is missing/broken), plus last-message age, Hz,
mean size, latency, exact wrapped-topic sequence loss/reordering, and echo-derived
RTT/clock offset.

The running session writes, under
`session-instances/.../logs/<peer>/status/`:

- `status.json` — machine-readable snapshot (source of truth) for tools and scripts,
  refreshed on a short interval and on every state transition,
- `status.txt` — a human-rendered table, and
- `events.jsonl` — state transitions plus per-`(topic, epoch, seq)` transit
  records for wrapped topics (delivered/lost/reordered, section latency, size,
  inter-arrival, and jitter). `epoch` counts the restarts of the sending peer's
  wrapper: sequence numbers start at zero again after one, so it is what keeps
  the numbering unique over an instance that outlived a restart.

Which of the bandwidth numbers a session publishes answers which question —
per-stream payload, whole-interface wire traffic, and the overhead ratio
between them — is in [link bandwidth](docs/link-bandwidth.md).

For fixed-interval network-condition samples alongside the same session, enable
the [link trace recorder](docs/link-trace.md). It writes
`link_trace.jsonl` under the same status directory with `/proc/net/dev` counter
deltas, echo-heartbeat RTT/loss provenance, and an optional modem-metrics hook.

Those counters localise loss to a *direction* and no further. When the question
is which fragment was lost, whether a retransmit followed, or how one sample's
sub-messages were spaced, enable the [header-only OTA capture](docs/ota-pcap.md)
next to them — `shared.ota_pcap.enabled`, off by default, roughly 1 % of payload
volume, and every packet labelled uplink or downlink. It needs no capability,
no image change and no extra container — only the passwordless sudo the
container already has, and only until its socket is open.

Convert a recorded link trace into a profiles-file entry when you want to replay
the same drive conditions in a repeatable benchmark:

```bash
rosotacom profile from-trace session-instances/.../logs/a/status/link_trace.jsonl \
  --mode timeline --name drive_replay --out generated-profiles.yaml
rosotacom benchmark recovery --profiles-file generated-profiles.yaml --profile drive_replay
```

Timeline mode emits piecewise-constant network profiles reference segments, turns long sample gaps
into `outage: reconnect`, and turns sustained 100% probe loss into
`outage: catchup`. Static mode distills one profile with median valid rate, p90
one-way delay, delay-spread jitter, and mean loss; constrain it with
`--window START:END` when only part of a drive should calibrate the profile. RTT
is mapped to symmetric one-way delay. Passive `/proc/net/dev` throughput is only
a lower-bound observation, so `rate` is emitted only when a sample is marked as
saturated/probed/capacity-like; otherwise the converter omits rate instead of
fabricating capacity. The generated YAML carries a provenance comment with the
source path, SHA-256, parameters, and these caveats.

Render an offline route map when a run also has GPS or pose samples:

```bash
rosotacom geomap --bag session-instances/.../rosbags/a/native --gps-topic /fix \
  --trace session-instances/.../logs/a/status/link_trace.jsonl \
  --metric observed_tx_kbps \
  --out-csv artifacts/geo-link-quality.csv \
  --out-html artifacts/geo-link-quality.html
```

`geomap` writes georeferenced CSV samples, an HTML report, and a sibling
`.route.png` route image. The timestamp offset between trace/event time and
bag/GPS time is explicit; see [docs/geo-link-map.md](docs/geo-link-map.md) for
the alignment model, CSV fixture workflow, and supported metrics.

Read it from the host with the `status` command:

```bash
rosotacom status 1_heartbeat_status            # human-readable table
rosotacom status 1_heartbeat_status --json     # machine-readable, for tools and scripts
rosotacom status 1_heartbeat_status --watch    # live refresh
```

Join one or more peer timelines after a run:

```bash
rosotacom metrics session-instances/.../logs/a/status/events.jsonl \
  session-instances/.../logs/b/status/events.jsonl
```

Use `--records` for joined per-message rows instead of the loss and p50/p95
summary. Wrapped-topic corrected latency requires `shared.use_heartbeat: true`;
`status.json.clock_sync` reports the selected minimum-RTT sample, peer offset,
sample age, and symmetric-path assumption. Uncorrected OTA delay remains a
separate field.

Compare how a stream changes across recorded stages — for example native
pre-processing, processed handoff, and post-OTA transit rows:

```bash
rosotacom stream-stats \
  --bag pre=logs/b/metrics/stages_20260702_172900:/sensors/camera/front_medium/resized/image_rect_color \
  --bag handoff=logs/b/metrics/stages_20260702_172900:/sensors/camera/front_medium/resized/image_rect_color/compressed/restamped/drop1of2/ffmpeg \
  --events post=logs/b/status/events.jsonl:/sensors/camera/front_medium/resized/image_rect_color \
  --out stream-stats
```

The command emits a side-by-side Markdown table plus `stream-stats.json`, with
per-stage size distributions, observed rate, interval regularity, and FFMPEG
GOP/keyframe shape when available. MCAP bag sources use the Python `mcap`
reader; other rosbag2 storage backends require `rosbag2_py`. `events`
sources work from the recorded transit rows. See
[docs/ffmpeg-keyframes.md](docs/ffmpeg-keyframes.md) for the GOP methodology
and example camera interpretation.

Turn a whole recorded instance into an explanation of *where* and *how*
delivery degraded — loss bursts, latency excursions, and rate collapses with
exact boundaries, joined with link-trace/profile/keyframe context:

```bash
rosotacom report session-instances/<day>/<instance>
```

It writes `report/report.json`, `report/report.md`, and per-stream figures with
the events marked (needs the `[plots]` extra). See
[docs/forensics-report.md](docs/forensics-report.md) for the detection
semantics and the correlation-not-causation caveat.

For opt-in local per-step latency, record generated local stage topics:

```yaml
shared:
  use_status_overview: true
  metric_backbone:
    record_stages: true
```

The MCAP is written under `logs/<peer>/metrics/`. Analyze an ordered pipeline
with `ros2 run com_py stage_latency BAG TOPIC...`.

For decoded camera stages, compute offline image quality with PSNR/SSIM:

```bash
rosotacom videoquality \
  session-instances/.../logs/b/metrics/stages_... \
  session-instances/.../logs/a/metrics/stages_... \
  --ref-topic /camera/image \
  --degraded-topic /camera/image/ffmpeg/raw \
  --out videoquality.json \
  --plot videoquality.png \
  --min-mean-psnr 30 \
  --min-mean-ssim 0.90 \
  --max-loss-pct 0
```

Use the outbound pre-transport image topic as the reference (`native` when no
preprocessing runs; otherwise the generated topic immediately before
`transport`, such as a restamped/drop output) and the receiver-side decoded
`native_in` stage as the degraded stream. The reference topic may be decoded
`sensor_msgs/msg/Image` or JPEG/PNG `sensor_msgs/msg/CompressedImage`; the
receiver-side ffmpeg comparison topic should be the decoded `/raw` reverse
transport output. Lost frames are reported as delivery loss and are never
averaged into PSNR/SSIM. Encoded `ffmpeg_image_transport_msgs/msg/FFMPEGPacket`
topics carry packet bytes, not pixels, so select the decoded `/raw` topic when
computing image quality. The same command also accepts JSON frame manifests for
host-only checks:

```bash
rosotacom videoquality --make-synthetic /tmp/rosotacom-quality
rosotacom videoquality \
  /tmp/rosotacom-quality/reference-frames.json \
  /tmp/rosotacom-quality/degraded-frames.json \
  --min-mean-psnr 20 --max-loss-pct 0
```

After a recording run, validate that a session instance still contains the
artifacts needed for later analysis:

```bash
rosotacom bundle check session-instances/.../drive_run --peer a --peer b \
  --file config/resolved-session.yaml \
  --bag rosbags/a/native
```

`bundle check` validates each expected peer's `status.json` and `events.jsonl`,
requires transit rows in the event streams, and validates expected rosbag2 bags
from their `metadata.yaml` message counts. Extra expected artifacts can be passed
as repeated `--file`, `--optional-file`, `--bag`, and `--optional-bag` flags, or
via a generic YAML manifest:

```yaml
schema_version: 1
peers: [a, b]
required_files:
  - config/resolved-session.yaml
required_bags:
  - path: rosbags/a/native
    label: native bag
optional_files:
  - traces/link.jsonl
```

Phase 1 samples local-domain stages directly. OTA stages are graph-only and
their activity is inferred from adjacent local flow, so the status overview
never creates an additional OTA payload subscription. Combine both peers' files
for the full end-to-end picture; cross-peer confirmation is reserved for a later
phase.

Run the CI single-machine smoke matrix locally:

```bash
just test-e2e-smoke
```

Run the heartbeat example manually:

```bash
./scripts/1_heartbeat/run_machine_a.sh
./scripts/1_heartbeat/run_machine_b.sh
```

Example 2 is the pilot for one-command scenario orchestration:

```bash
rosotacom scenario start 2_native_chatter --identity a
```

The existing `run_external.py` and `run_communication.sh` helpers remain
available as a compatibility fallback. Examples 3–6 continue to use those
helpers for now.

The `sessions/` directory contains curated built-in session definitions:

- `1_heartbeat`: minimal heartbeat exchange
- `1_heartbeat_status`: heartbeat exchange with the live status overview enabled
- `2_native_chatter`: bridge `/chatter` from `machine_b` to `machine_a`
- `3_comp_occ_grid`: compressed occupancy grid over DDS
- `4_comp_occ_grid_zen`: compressed occupancy grid through Zenoh
- `5_sized_payload`: sized payload test over DDS
- `6_sized_payload_zen`: sized payload test through Zenoh
- `14_remote_assist_anonymized`: generic processed replay shape for the
  remote-assist OTA contract; run it with
  `rosotacom smoke 14_remote_assist_anonymized`
- `15_remote_assist_anonymized_costmap`: single-stream cut of example 14 —
  only `/topic5`, the anonymized compressed occupancy-grid stream; run it with
  `rosotacom smoke 15_remote_assist_anonymized_costmap`
- `16_remote_assist_anonymized_camera`: single-stream cut of example 14 —
  only `/topic9`, the anonymized ffmpeg camera stream; run it with
  `rosotacom smoke 16_remote_assist_anonymized_camera`
- `17_synthetic_camera_quality`: synthetic raw camera stream encoded through
  ffmpeg and reverse-republished as decoded `/raw` frames for offline
  `rosotacom videoquality` checks

Transport-combination coverage lives under `tests/sessions/rmw_matrix` and is
generated from `tests/sessions/generate_rmw_matrix.py`.

`rosotacom benchmark` uses the packaged `bench_1_*` sessions for local or OTA
benchmark probes. Benchmark sessions default to Cyclone DDS; pass `--rmw
fastdds`, `--rmw cyclone`, or another supported session RMW value to pin a run.
Each run writes a self-contained `result.json` under its benchmark artifact
directory with the selected RMW, configured load and offered bandwidth, profile
shaping context, thresholds, verdict, per-topic loss/latency/jitter metrics, and
the runner fingerprint. Deterministic runs gate against the committed two-sided
performance bands in `budgets.jsonl` via `rosotacom benchmark compare`, and
bands move only through `rosotacom benchmark ratchet` — see
[docs/performance-bands.md](docs/performance-bands.md) for the band schema,
verdicts, and the ratchet workflow (performance bands reference).

Two result sets taken in **different environments** — emulated versus a real
link, or shaped versus unshaped — are compared with `rosotacom benchmark delta`:

```bash
rosotacom benchmark delta emulated/ ota/ \
  --label-reference emulated --label-measured ota \
  --out delta.json --markdown delta.md
```

It pairs runs on genre, profile, RMW and session name, so a row that ran in one
environment and not the other is reported as unmatched rather than silently
dropped — "it got faster" and "it did not run" must not look the same. Every
report is labelled `monitor_only` and carries each side's command and profile
file, so a table pasted into a paper or an issue can be traced back without the
directory it came from.

It exits 0 whatever the numbers say, and non-zero only when the two sets share
no rows at all — a delta is an observation to explain, and an exit code would
turn "explain this" into "re-run until it is quiet".
For live benchmark probes, `--duration` is the shaped publish window; after that
rosotacom stops synthetic publishers and waits `--drain-s` seconds before
tearing down shaping, so delayed in-flight messages are not miscounted as loss.
Cyclone DDS benchmark sessions keep the default tuned OTA
`SPDPInterval=30s`. On tight rate-limited profiles, this periodic discovery
metatraffic can share the shaped link with payload traffic and appear as regular
p99/max latency spikes. rosotacom records a `cyclonedds_spdp` diagnostic in
`result.json` and prints a warning when the probe duration and offered/shaped
bandwidth make that effect plausible. Keep the default for end-to-end DDS
behavior. For payload-only characterization where discovery bursts would mask the
question under test, pass `--cyclone-spdp-interval 150s` or another longer
positive `ms`/`s` duration. Making SPDP too frequent improves stale-peer and
reconnect detection cadence but contaminates tight-link latency more often;
making it too lax quiets short probes but hides discovery/liveliness overhead and
delays detection of changed peer state. The living record for this quirk is the
[SPDP discovery-burst finding](docs/findings/cyclone-spdp-discovery-bursts.md).
Use `benchmark probe` when you want a fixed payload/rate under one profile
instead of a breakpoint search. It writes the normal `result.json` plus
`time-bins.jsonl` with per-second loss, delivered Hz, payload bandwidth, and
p50/p95 latency/jitter bins; by default it also renders `probe-timeseries.png`
when the optional plotting dependency is installed:

```bash
rosotacom benchmark probe --profile cellular-4g-typical --size 18000 --rate-hz 20 --duration 20 --repeats 1
rosotacom benchmark probe --profile cellular-4g-typical --size-pattern 1x20KB+1x0KB --rate-hz 10 --duration 20 --repeats 1
rosotacom benchmark probe --profile cellular-4g-typical --size 18000 --rate-hz 20 --duration 20 --cyclone-spdp-interval 150s
rosotacom benchmark plot time-bins.jsonl --type probe
```

Use `benchmark requirements` when you want rosotacom to search for a tight
network profile for a stream and quality target. The driver starts from ideal
conditions, uses a geometric boundary search for bandwidth, linear boundary
searches for latency-like axes, and finishes with an extra refinement pass around
the final candidate. By default jitter and loss are searched as a coupled
practical tradeoff curve; pass `--loss-coupling independent` if you want network
loss treated as its own axis. For an exact `--max-loss 0` target, rosotacom
automatically keeps shaped network loss at `0%` and searches jitter separately,
so the reproducible command encodes the same policy used for the measurement. If
the zero-loss boundary is noisy, repeat each candidate and require a pass count;
for example, `--probe-repeats 10 --probe-min-passes 9 --bad-lossy-count 10`
defines a good case as at least nine clean repeats and records bad-case neighbors
where all ten repeats lose messages. Add `--netem-seed <n>` when you want the
generated jitter/loss draw to be replayable across repeated boundary searches.
For local Docker benchmarks, rosotacom copies the host `tc` binary into seeded
benchmark containers when the container distro `tc` is too old for
`netem seed`; the host `tc` must support `seed SEED`. For a practical good-case
reference, `--jitter-guard-ratio 0.10` records the tight jitter boundary but
uses 10% less jitter for later axes and the final profile;
`--bandwidth-guard-ratio 0.10` does the symmetric thing for bandwidth by using
10% more bandwidth after finding the lower bound. The chosen profile, target
slack, repeat statistics, all probes, and the nearby failing neighbors are written to
`result.json`, `requirements.jsonl`, and `generated-profiles.yaml`:

```bash
rosotacom benchmark requirements --rate-hz 20 --size 18000 --qos-reliability best_effort --qos-depth 1 --max-loss 5 --max-latency-ms 250
rosotacom benchmark requirements --rate-hz 20 --size 18000 --qos-reliability best_effort --qos-depth 1 --max-loss 0 --max-latency-ms 250
rosotacom benchmark requirements --rate-hz 20 --size 18000 --qos-reliability best_effort --qos-depth 1 --max-loss 0 --max-latency-ms 250 --latency-base-ms 30 --downlink-mode lan --probe-repeats 10 --probe-min-passes 9 --bad-lossy-count 10 --jitter-guard-ratio 0.10 --bandwidth-guard-ratio 0.10 --netem-seed 424242
rosotacom benchmark requirements --rate-hz 20 --size 18000 --qos-reliability best_effort --qos-depth 1 --max-loss 0 --max-latency-ms 250 --latency-base-ms 30 --downlink-mode lan --axes jitter,bandwidth --bandwidth-high-factor 8 --bandwidth-low-factor 1 --jitter-high-ms 40 --search-iterations 7 --final-refine-iterations 0 --search-rounds 1 --min-duration 20 --min-messages 100 --probe-repeats 1 --bandwidth-probe-repeats 1 --netem-seed 424242 --jitter-guard-ratio 0.20 --bandwidth-guard-ratio 0.20
```

Use `benchmark loss-boundaries` when the question is specifically “where does
the first ROS 2 loss appear?” It searches bandwidth and jitter as discrete axes:
bandwidth reports the lowest loss-free rate and the adjacent lower bad rate at
`--bandwidth-step`; jitter reports the highest fully clean value, the first
non-clean neighbor, the lowest fully lossy value, and the mixed zone between
them at `--jitter-step-ms`. Seedless runs are the best empirical estimate for a
host/link, a single `--netem-seed` is useful for replay/debugging one random
jitter sequence, and `--netem-seeds 101,102,...` is the reproducible compromise
when you want several replayable jitter draws instead of overfitting one seed:

```bash
rosotacom benchmark loss-boundaries --rate-hz 20 --size 18000 --qos-reliability best_effort --qos-depth 1 --max-latency-ms 250 --latency-base-ms 30 --downlink-mode lan --bandwidth-low 2.8mbit --bandwidth-high 4mbit --bandwidth-step 0.1mbit --jitter-low-ms 0 --jitter-high-ms 40 --jitter-step-ms 1 --min-duration 20 --min-messages 100 --probe-repeats 10
rosotacom benchmark loss-boundaries --rate-hz 20 --size 18000 --qos-reliability best_effort --qos-depth 1 --max-latency-ms 250 --latency-base-ms 30 --downlink-mode lan --bandwidth-low 2.8mbit --bandwidth-high 4mbit --bandwidth-step 0.1mbit --jitter-low-ms 0 --jitter-high-ms 40 --jitter-step-ms 1 --min-duration 20 --min-messages 100 --probe-repeats 1 --netem-seeds 101,102,103,104,105,106,107,108,109,110
```

Both `requirements` and `loss-boundaries` also accept a **bag replay as the
load** instead of the synthetic `sized_publisher` stream: pass `--target
<session>` (a two-peer replay session such as `15_remote_assist_anonymized_costmap`)
and, optionally, `--bag <metadata.yaml>` for per-topic completeness ground truth.
A probe point is then one full replay run under the candidate profile, and the
oracle judges the **whole contract** — every carried topic must clear the
loss/latency bound and the `--oracle-min-completeness` floor — aggregated to a
run verdict that names any failing topics. This answers "which network could
carry *this bag*?" rather than "…this synthetic stream?". Because a probe now
costs a full bag loop, keep the iteration budget small. See
[docs/benchmark-bag-as-load.md](docs/benchmark-bag-as-load.md) for the oracle,
the local-vs-OTA execution split, and cost guidance:

```bash
rosotacom benchmark loss-boundaries --target 15_remote_assist_anonymized_costmap --target-type session --axes bandwidth --bandwidth-low 0.5mbit --bandwidth-high 2mbit --bandwidth-step 0.5mbit --rate-hz 10 --min-duration 20 --min-messages 1 --probe-repeats 1 --good-clean-count 1 --bad-lossy-count 1 --max-latency-ms 1000
```

Use `benchmark ab` when the question is a **tuning** one: "does candidate config
B beat baseline config A on the same load and profile?" Each `--baseline` and
`--candidate label=…` is a whole session config (a directory or a
`session-definition.yaml`) that shares the synthetic `a_to_b` load and differs
only in the pipeline knobs under test (QoS reliability/depth/lifespan,
compression, ffmpeg gop/bitrate/crf, … — knobs that keep the delivered topic
name, unlike `throttle_hz`/`drop` which republish to a renamed topic; see the
doc). The driver runs them interleaved with a rotating start over `--repeats`,
then classifies every candidate against the baseline per topic and metric as
**IMPROVED / WITHIN / REGRESSED** — the same verdict language the regression gate
uses — with a two-sided tolerance band (`--rel-tolerance`, `--abs-tolerance`).
The overall verdict fails if any watched metric regressed; the repeat spread
(min/median/max) travels with every cell so small-N effects are read honestly.
It writes `result.json`, a markdown table (`ab.md`), the per-config YAML diffs,
and `ab.jsonl`. See [docs/benchmark-ab.md](docs/benchmark-ab.md) for the config
model, interleaving, and the statistical-power note:

```bash
rosotacom benchmark ab --profile cellular-4g-degraded --baseline configs/gop30 --candidate gop15=configs/gop15 --size 18000 --rate-hz 20 --duration 20 --repeats 3
```

Use `rosotacom ota-benchmark` for the same probes over deployment peers, without
having to name a target:

```bash
rosotacom ota-benchmark probe --profile cellular-4g-degraded --size 18000 --rate-hz 20 --duration 20 --repeats 1 --peer a=vehicle --peer b=center
rosotacom ota-benchmark capacity --profile cellular-4g-degraded --knob size --low 1 --high 1 --max-loss 30 --max-latency-ms 1000 --duration 10 --repeats 1 --peer a=vehicle --peer b=center
rosotacom ota-benchmark requirements --rate-hz 20 --size 18000 --qos-reliability best_effort --qos-depth 1 --max-loss 5 --max-latency-ms 250 --peer a=vehicle --peer b=center
rosotacom ota-benchmark loss-boundaries --rate-hz 20 --size 18000 --qos-reliability best_effort --qos-depth 1 --max-latency-ms 250 --latency-base-ms 30 --downlink-mode lan --bandwidth-low 2.8mbit --bandwidth-high 4mbit --bandwidth-step 0.1mbit --jitter-high-ms 40 --jitter-step-ms 1 --probe-repeats 10 --peer a=vehicle --peer b=center
```

OTA benchmarks default to the benchmark session for the selected genre. Pass
`--target` and `--target-type` only when you deliberately want to benchmark a
project-specific session or scenario instead. Add `--interactive` to open a tmux
operator view with a high-level run window, one fullscreen attachable catmux
window per local peer, a network window split into qdisc status and tc/netem
command logs, and a results window that prints the final result once. Shaped OTA
benchmark profiles need sudo on the remote peers for the generated `tc`/`ip`
commands. The default `--sudo-mode passwordless` checks `sudo -n` and is the right
mode for unattended runs. For attended operator runs, pass `--sudo-mode askpass`
to prompt locally once per peer and feed the password to remote `sudo -S` over SSH
stdin without printing or storing it.

## Development

For contributor setup and local checks, see [CONTRIBUTING.md](CONTRIBUTING.md).
CI behavior is summarized in [docs/ci.md](docs/ci.md).

The test and measurement architecture is recorded in the
[implementation references](docs/reference/README.md), with measured public effects in the
[findings ledger](docs/findings/README.md); see [docs/testing.md](docs/testing.md)
for the test taxonomy. Field-fitted network profiles from the 2026-08 cellular
campaign are in [field-profiles/2026-08](field-profiles/2026-08/README.md).
Quality rules live in [DEVELOPMENT_PRINCIPLES.md](DEVELOPMENT_PRINCIPLES.md).

## Choosing the Transport Layer: CycloneDDS or Zenoh

- **Use CycloneDDS** when all machines share the **same `ROS_DOMAIN_ID`**.
  This is the simplest and most direct configuration.

- **Use Zenoh** when peers cannot rely on one shared DDS domain, or when you want to split local application nodes and OTA-facing bridge nodes into different ROS 2 domains on each peer.
  In that split-domain setup this repository generates a standard ROS 2 `domain_bridge` for `/com/...` topics locally, while Zenoh carries the `/ota/...` traffic between peers.

## Position in the OTA Communication Landscape

This repository fits into a broader set of ROS-based OTA communication approaches:

- **Direct ROS 2 DDS Communication**
  Native DDS (CycloneDDS, Fast DDS), often with custom configuration for constrained or long-range links.
  The examples in this repository use CycloneDDS to illustrate this approach.

- **ROS 2 over Router-like Backbones**
  Some RMW have their own DDS Routers such as [eProsima/DDS-Router](https://github.com/eProsima/DDS-Router).
  Example 4 uses Zenoh to act as a lightweight router layer.

- **MQTT-based Approaches**
  Common in cloud/IoT scenarios. Example:
  [ika-rwth-aachen/mqtt_client](https://github.com/ika-rwth-aachen/mqtt_client)

- **Custom TCP/UDP Teleoperation Stacks**
  Some frameworks implement their manual tcp/udp transportion layers. Example:
  [TUMFTM/teleoperated_driving](https://github.com/TUMFTM/teleoperated_driving)

## How to Cite

If you wish to cite the ROS Communication DevContainer in your work, please use the following citation:

```latex
@InProceedings{gontscharow_scalable,
  author    = {Gontscharow, Martin and Doll Jens and Schotschneider, Albert and Bogdoll, Daniel and Orf Stefan and Jestram Johannes and Zofka, Marc and Z\"{o}llner, J. Marius},
  title     = {{Scalable Remote Operation for Autonomous Vehicles: Integration of Cooperative Perception and Open Source Communication}},
  booktitle = {2024 IEEE Intelligent Vehicles Symposium (IV)},
  year      = {2024}
}
```
## Acknowledgements
The research leading to these results was conducted within
the project ÖV-LeitmotiF-KI and was funded by the German
Federal Ministry for Digital and Transport (BMDV), grant number 45AVF3004A-G.
Responsibility for the information and views set out in this
publication lies entirely with the authors.
