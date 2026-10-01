# Rosotacom Examples

This directory is a copyable rosotacom example project. It contains local
project setup, reusable session/scenario configs, and helper scripts.

## Use From An Installed CLI

```bash
rosotacom examples create ./rosotacom_examples
cd ./rosotacom_examples
rosotacom start 1_heartbeat --identity a
```

In another terminal on the same host:

```bash
cd ./rosotacom_examples
rosotacom start 1_heartbeat --identity b
```

Stop each side when done:

```bash
rosotacom stop 1_heartbeat --identity a
rosotacom stop 1_heartbeat --identity b
docker ps --filter name=rosotacom   # optional: confirm cleanup
```

## Complete native-chatter scenario

Example 2 composes its communication session with the matching local
application container:

```bash
rosotacom scenario start 2_native_chatter --identity a
```

Run identity `b` on the other machine in the same way. The outer tmux prefix is
`Ctrl-b`; communication and the local application use separate full windows.
Switch with `Ctrl-b n`/`Ctrl-b p`, and use `Ctrl-b Ctrl-b` to address the inner
catmux session. Detaching keeps the use case running:

```bash
rosotacom scenario list
rosotacom scenario attach
rosotacom scenario stop
```

The short `attach`/`stop` forms work when exactly one scenario/identity pair is
active. Otherwise tab completion lists only the active choices.

The existing scripts under `scripts/2_native_chatter/` remain as a fallback.

## Local smoke test

`rosotacom smoke` is the one-command local check. It starts both peers in
checkout-scoped containers, verifies the path, and removes its own containers
before returning:

```bash
rosotacom smoke
```

Expected result:

- two checkout-scoped containers start;
- `/com/in/a/heartbeat_a`, `/heartbeat_a`, `/com/in/b/heartbeat_b`, and
  `/heartbeat_b` report as publishing;
- each checked topic prints a `SMOKE_METRIC` line (rate `hz`, latency `delay_s`);
- the command removes its containers before returning.

`rosotacom smoke` injects isolated peer addresses internally. The tracked
session definitions contain only logical peers, so local testing never depends
on physical-machine configuration.

### Opt in to ROS 2 Lyrical

Kilted remains this project's default. The copied project includes a second,
independent `ros2docker.lyrical.json`; select it for one command with:

```bash
ROSOTACOM_ROS2DOCKER_CONFIG="$PWD/ros2docker.lyrical.json" rosotacom smoke
```

For a persistent project opt-in, set
`ros2docker_config: ros2docker.lyrical.json` in `rosotacom.yaml`. The Lyrical
image uses ros2docker's pinned `domain_bridge` source fallback until an official
Lyrical binary package is available. Scenario applications with a
`ros2docker_config_by_distro.lyrical` entry switch to their Lyrical image at the
same time; the default application config remains Kilted.

For an interactive local end-to-end debug session, use the same smoke target
with `--interactive`:

```bash
rosotacom smoke 2_native_chatter --interactive
rosotacom smoke --interactive --list
rosotacom smoke 2_native_chatter --interactive --stop
```

This opens an outer tmux session with full windows for each peer's communication
container, each scenario application, and a verification/status view. The
verification window keeps the check log in one pane and live status in another.
Scenario applications share their peer communication container's isolated
network namespace. The outer prefix is `Ctrl-b`; use `Ctrl-b Ctrl-b` for the
inner catmux sessions.

## Anonymized remote-assist replay shape

`14_remote_assist_anonymized` is the public, CI-safe replay shape of the
private remote-assist scenario. It keeps the two-way OTA contract, command
target-prefixing, receiver source-prefixing, processed message types, and QoS,
but uses generic `/topic1` ... `/topic20` names and synthetic smoke payloads:

```bash
rosotacom smoke 14_remote_assist_anonymized
```

The full replay scenario with bag-play applications is generated from a
processed handoff trace by `rosotacom anonymize`; this packaged session is the
small deterministic smoke target for that generated shape.

Two single-stream cuts isolate the heavy streams of that contract so each can
be measured over the link (rate regularity, latency, loss) without the other
19 topics competing for it — everything else (peers, RMW, OTA QoS,
heartbeat/status, expects) matches example 14:

```bash
rosotacom smoke 15_remote_assist_anonymized_costmap   # /topic5: compressed occupancy grid
rosotacom smoke 16_remote_assist_anonymized_camera    # /topic9: ffmpeg camera packets
```

Each session header documents the pre-send baseline measured on the source
recording (message-size distribution and inter-message regularity), the
reference point for judging how well the received stream preserves the input
cadence.

## Whole-bag expect generation

The example project includes a tiny rosbag2 `metadata.yaml` fixture for
`8_drop`. It lets you inspect the whole-bag expect generator without needing a
large MCAP:

```bash
rosotacom expect from-bag bags/8_drop_reference --session 8_drop --out whole-bag-expect.yaml
```

The generated fragment accounts for the session's configured `drop` stage before
emitting `min_count` and bag-relative completeness thresholds. Review the
comments in the output, then merge the `expect:` block into a session variant
when you want a replay gate that asserts the reference bag's carried topics.

## Two-machine runs

For a real two-machine run, pass both reachable addresses on both hosts:

```bash
rosotacom start 1_heartbeat --identity a \
  --peer-address a=10.0.0.10 --peer-address b=10.0.0.11
```

Use `--identity b` on the second host. Alternatively copy
`deployment.example.yaml` to `deployment.yaml`, reference it from
`rosotacom.yaml`, and use `--peer a=machine-a --peer b=machine-b`.

## Layout

- `rosotacom.yaml`: project-local rosotacom setup
- `ros2docker.json`: Docker runtime defaults used by the communication containers
- `ros2docker.lyrical.json`: opt-in ROS 2 Lyrical runtime configuration
- `deployment.example.yaml`: optional named-host deployment example
- `bags/`: small rosbag metadata fixtures used by documentation and host tests
- `sessions/`: tracked static session definitions/templates
- `scenarios/`: complete use cases that combine a session with local applications
- `session-instances/`: ignored generated runtime configs, catmux logs, smoke logs, and rosbags
- `scripts/`: convenience wrappers and external-node launchers

`rosotacom.yaml` keeps session and scenario roots as ordered lists, so adapted
projects can put their own directories first and this packaged example's
directories later.
