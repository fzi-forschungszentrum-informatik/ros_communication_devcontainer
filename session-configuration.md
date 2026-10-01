# Session Configuration Reference
This document is the **user reference** for writing the user-facing session configuration files in a **session directory**.

The generator is intentionally **strict**: **unknown keys are errors** so misconfigurations don’t silently do nothing.

## Quick start
Your session directory contains **one primary input file**:

- **`session-definition.yaml`**: self-contained, fully specified session configuration (no external templates, no parameters)
- **`session-parametrization.yaml`**: selects a template + provides parameters; the generator resolves it to a definition

When `session-parametrization.yaml` is used, the generator also writes a resolved **`session-definition.yaml`** into the session directory (semantic-diff protected; requires `--force` on mismatches).

## Session definition (`session-definition.yaml`)

### Hard constraints
These constraints are **enforced** by the generator (or by the base plugin it targets):

- **Exactly 2 peers**
  `peers:` must exist and must contain **exactly two** entries.
  Reason: the base plugin (`session_plugin_base.yaml`) is currently “1-remote”.

- **Direction keys in `topics`**
  Keys must be exactly `"<src>_to_<dst>"` (e.g. `a_to_b`).
  `<src>` and `<dst>` must be peer keys defined under `peers:`.

- **Feature slot limits (max 4 each)**
  The base plugin supports at most **4** entries per feature group:
  - **drop** (`drop`)
  - **throttle** (`throttle_hz`)
  - **pixel cap** (`pixel_cap_preset`)
  - **image transport** (`transport`)
  - **normalize** (`normalize_on_target`)

- **Not implemented / unsupported combinations (will error)**
  - `shared.compression.remove_algorithm_suffix_on_decompression: false` (must be `true` if set)
  - `peer_settings.<peer>.outbound.source_prefix.use_source_prefix: false`
  - `peer_settings.<peer>.inbound.keep_target_prefix: true`
  - `peer_settings.<peer>.outbound.target_prefix.native_have_outgoing_target_prefix != use_target_prefix`
  - `shared.rmw.ota=zenoh_connect_endpoints` without all of `shared.ota_domain_id` and `peer_settings.<peer>.domain_id` (every peer) being set to three distinct values (native rmw_zenoh_cpp's router forwards across ROS domains, so each logical graph — per-peer local + shared OTA — must live on its own domain).
  - `shared.rmw.ota=zenoh_connect_endpoints` with `shared.rmw.local != zenoh` (rmw_zenoh_cpp is not interoperable with DDS-based RMWs).
  - `shared.rmw.ota=zenoh_ros2dds` with `shared.rmw.local = zenoh` (the bridge speaks DDS locally; native zenoh locally is incompatible).
  - **Legacy keys** `shared.rmw_ota`, `shared.rmw_local`, `shared.zenoh` are rejected. Use `shared.rmw` (and `shared.local_domain_id` / `peer_settings.<peer>.domain_id`) instead.

### Root schema (definition)
Allowed top-level keys:

```yaml
peers: {}          # required
shared: {}         # optional
peer_settings: {}  # optional
topics: {}         # optional
```

### `peers` (required)

```yaml
peers:
  <peer_key>:
    host: <string>           # optional default name from deployment.yaml
    com-name: <scalar>       # optional, default: <peer_key>
```

- **`peer_key`**: used to form direction keys like `<src>_to_<dst>`.
- **`host`**: optional named-host default. It may be overridden with
  `--peer <peer_key>=<host>`.
- **`com-name`**: peer communication name (default = `peer_key`); affects e.g. topic prefixes.

Session definitions never contain physical addresses or SSH targets. Supply
them at launch with `--peer-address`, or map logical peers to named hosts from
the project's deployment file with `--peer`. See
[deployment-configuration.md](deployment-configuration.md).

### `shared` (optional)

```yaml
shared:
  use_topic_monitor: false    # default false
  use_status_overview: false  # default false
  use_heartbeat: false        # default false

  # Unified RMW configuration. Covers both the local ROS graph and the
  # over-the-air (OTA) bridge processes. See "RMW configuration" below.
  rmw: cyclone               # string shortcut => local=ota=cyclone (only cyclone|fastdds|zenoh allowed as shortcut)
  # rmw:                     # mapping form (per-side)
  #   local: cyclone         # optional per-side string or {impl: {config?, easy_mode_ip?, main_peer?, main_port?, transport?}}
  #   ota:
  #     fastdds:
  #       config: fastdds_v1.xml

  ota_domain_id: 48          # optional; ROS_DOMAIN_ID used by OTA bridge processes (IN/OUT, Z2D, domain_bridge).
                             # When it differs from a peer's local domain_id, a stock ROS 2 domain_bridge is spun up.
                             # Required with shared.rmw.ota=zenoh_connect_endpoints and must differ from each
                             # peer_settings.<peer>.domain_id.

  local_domain_id: 47        # optional shortcut; applied as `peer_settings.<peer>.domain_id` to every peer that did
                             # not set its own. A per-peer value overrides this shortcut, but setting both to
                             # different values is an error. Not useful with shared.rmw.ota=zenoh_connect_endpoints, which requires
                             # distinct per-peer domain ids.

  use_in: null               # default null (auto-enable based on topics)
  use_out: null              # default null (auto-enable based on topics)

  heartbeat_position:        # optional; default per direction: prepend
    <src>_to_<dst>: prepend  # prepend|append

  processing_suffixes:       # optional
    restamped: "/restamped"          # default
    framebridge_global: "/globalframe"  # default

  compression:               # optional
    algorithm: bz2           # bz2|zlib|lz4|zstd, default bz2
    remove_algorithm_suffix_on_decompression: true  # default true (false => error)

  qos:                       # optional (or any topic.qos triggers writing qos.yaml)
    defaults: {}             # free-form, written to qos.yaml
    for_role: {}             # free-form, written to qos.yaml
```

#### RMW configuration (`shared.rmw`)

`shared.rmw` is the **single** entry point for configuring the ROS middleware on
both sides of the bridge. It may be given in three forms:

1. **String shortcut** (applies to both local and OTA side):
   ```yaml
   rmw: cyclone          # local=cyclone, ota=cyclone
   rmw: fastdds          # local=fastdds, ota=fastdds
   rmw: zenoh            # shortcut for local=zenoh, ota=zenoh (legacy native shorthand)
   ```
2. **Per-side string** (no extra config):
   ```yaml
   rmw:
     local: cyclone
     ota: fastdds
   rmw:
     local: zenoh
     ota: zenoh_connect_endpoints
   ```
3. **Per-side tagged union** (with optional DDS / zenoh config block):
   ```yaml
   rmw:
     local:
       fastdds:
         config: fastdds_v3.xml
     ota:
       zenoh_ros2dds:
         transport: udp
         main_peer: a
         main_port: 7447
   ```

Recognized implementation names:

| name | `local` | `ota` | runtime | notes |
|---|---|---|---|---|
| `cyclone` / `cyclonedds` | ✓ | ✓ | `rmw_cyclonedds_cpp` | |
| `fastdds` / `fastrtps` | ✓ | ✓ | `rmw_fastrtps_cpp` | |
| `zenoh` | ✓ | legacy alias | `rmw_zenoh_cpp` (native) | use `zenoh_connect_endpoints` on OTA for transparent endpoint setup |
| `zenoh_connect_endpoints` | — | ✓ | `rmw_zenoh_cpp` (native) + `ZENOH_CONFIG_OVERRIDE` on non-main peers | single router per peer; see below |
| `zenoh_ros2dds` | — | ✓ | `zenoh_bridge_ros2dds` | DDS ↔ zenoh bridge; requires DDS locally |

Every side is **optional**. If a side is omitted, the runtime on that side does
not override `RMW_IMPLEMENTATION` and relies on the ambient ROS 2 defaults — no
`CYCLONEDDS_URI` / `FASTDDS_DEFAULT_PROFILES_FILE` is exported either.

Per-side config blocks:

- DDS implementations (`cyclone`, `fastdds`) accept:
  - `config: <template>` — template file under `ws/ota_configs/`
    (`cyclonedds_tuned.xml`, `cyclonedds_minimal.xml`,
    `cyclonedds_local_participants.xml`, `fastdds_tuned.xml`,
    `fastdds_unicast.xml`, `fastdds_easy_mode.xml`).
  - `easy_mode_ip: <string>` — resolved into a template's `#easy_mode_ip`
    placeholder; defaults to the first resolved peer address. Only
    `fastdds_easy_mode.xml` uses it.

  `fastdds_easy_mode.xml` is the discovery alternative: Fast DDS 3.1 spawns or
  reuses one Discovery Server per host and points the participant at the peer's
  server too, so two hosts find each other with no multicast and no per-peer
  locator list. It fails differently from `fastdds_tuned.xml` rather than
  better — an explicit peer list cannot discover what it was not told about,
  while easy mode depends on a server process being reachable on 11811 — which
  is why both are packaged. Neither caps the datagram (see above); easy mode
  expresses its transport settings as `<builtinTransports …>` attributes,
  because replacing the transports outright would take its discovery path with
  them.

  **On the OTA side, omitting `config` is not "no configuration".** Each
  implementation has a default template, and both defaults are the link-ready
  one: `cyclone` → `cyclonedds_tuned.xml`, `fastdds` → `fastdds_tuned.xml`.
  That is what makes `shared.rmw` an interchangeable choice — a session names a
  middleware and gets unicast discovery with the OTA interface pinned either
  way, rather than one middleware that survives a cellular tunnel and one that
  has to be configured into surviving it. `fastdds_unicast.xml` is the earlier,
  locator-only configuration, kept for a run that wants to measure the
  difference.

  The two defaults are **not** identical in what they put on the wire, and the
  difference is deliberate. CycloneDDS fragments at the RTPS layer (1200 B);
  Fast DDS hands a large sample to the kernel as one datagram and lets IP
  fragment it. Writing the same cap into the Fast DDS profile is the obvious
  move and it takes the link down: measured on the bench pair (2026-08-18/19,
  ROS 2 Kilted, Fast DDS 3.2.4) a `maxMessageSize` below the sample size stops
  the sample from ever arriving — at 1200 B and at 8192 B, synchronous writer
  and asynchronous — while an 84 B heartbeat on the same link keeps flowing.
  Plan the payload accordingly: the measured OpenVPN barrier on this path is
  around 110 kB per message.

  On the **local** side, omitting `config` really is no configuration: no
  template is applied and no `CYCLONEDDS_URI` / `FASTDDS_DEFAULT_PROFILES_FILE`
  is exported.

  On the **local** side of a busy machine, set
  `config: cyclonedds_local_participants.xml`. Cyclone allows 33 participants
  per domain per host by default, and a control centre running its own
  application next to the communication peer exceeds that: the processes that
  start last fail with `Failed to find a free participant index for domain N`
  and exit. That template raises the limit and changes nothing else — it pins no
  interface and sets no peer list, because local discovery must keep working.
  The OTA templates are not usable on the local side for exactly that reason.
- `zenoh_connect_endpoints` (native, OTA side only; `zenoh` is still accepted as a legacy OTA alias) accepts:
  - `transport: tcp|udp` — the inter-host transport between the two routers.
    Default `tcp`. On a lossy link TCP is the wrong choice for a best-effort
    stream: one retransmit stalls everything queued behind it, which turns a
    cellular dip into seconds of latency instead of one lost sample. The
    listening router keeps `tcp/[::]:7447` whatever this is set to, because
    every rmw_zenoh node on that host is a zenoh *client* of it over
    `tcp/localhost:7447`.
  - `main_peer: <peer_key>` — the peer whose zenoh router listens. Defaults to the first peer declared under `peers:`.
  - `main_port: 7447` — listening port. Default `7447`.
- `zenoh_ros2dds` (bridge) accepts (OTA side only):
  - `transport: udp|tcp` — default `tcp`, measured rather than assumed. On the
    bench pair (2026-08-19, the 2026-08-17 drive's loss process, 12 kB at 10 Hz)
    the UDP bridge delivered in one run of five and lost 66% of that one, while
    the TCP bridge delivered in every run at ~1% loss with the lowest median
    latency in the campaign. UDP stays available for a link where TCP's
    head-of-line blocking costs more than its retransmission buys.
  - `main_peer: <peer_key>` — defaults to the first peer.
  - `main_port: 7447` — default `7447`.

##### Native zenoh (`rmw.ota: zenoh_connect_endpoints`)

Native `rmw_zenoh_cpp` is only interoperable with itself, so the local and OTA
sides must both use native zenoh. Prefer the explicit mapping:

```yaml
rmw:
  local: zenoh
  ota: zenoh_connect_endpoints
```

The `rmw: zenoh` shortcut is still accepted, but `ota: zenoh_connect_endpoints`
is more honest about what the OTA mode does: the runtime starts exactly one
`rmw_zenohd` router per peer (the `ZEN` window) and the non-main peer opens a
TCP connect endpoint back to the main peer via `ZENOH_CONFIG_OVERRIDE`.

Because that single router forwards across **all** ROS domains, each logical
graph — every peer's local apps *and* the shared OTA bridge traffic — must live
on its own `ROS_DOMAIN_ID`; otherwise publications would leak between graphs.
The generator enforces this:

- `peer_settings.<peer>.domain_id` must be set for **every** peer, with distinct values.
- `shared.ota_domain_id` must be set and must differ from every peer's `domain_id`.
- Peer `address` expressions must be distinct across peers.

A stock ROS 2 `domain_bridge` process (the `COM` window) is then spun up on
each peer to bridge `/com/...` topics between that peer's `local_domain_id` and
`shared.ota_domain_id`, just like in the DDS split-domain case.

##### Bridged zenoh (`rmw.ota: zenoh_ros2dds`)

`zenoh_ros2dds` uses `zenoh_bridge_ros2dds` (binary: `zenohd`) to bridge DDS
topics to/from zenoh. The local graph still speaks DDS, so `rmw.local` must be
`cyclone` or `fastdds`. The runtime:

- Runs the bridge in the `Z2D` window on `ota_domain_id` (or the peer's
  `local_domain_id` if no split domain is configured).
- IN/OUT processes keep whatever DDS RMW was configured via `rmw.local`.
- `zen_pub_allow` / `zen_sub_allow` are derived from peer `com-name`s (`/ota/<com-name>/.*`).
- `zen_qos_pub` is derived from per-topic `zen_qos:` entries and baked into the generated `<peer>/plugin.yaml`.

#### Generated per peer: `topic_types.yaml`

Every session writes `<peer>/topic_types.yaml` — `<stage topic>: <message
type>` for every stage that peer takes part in — and the bridge and relay nodes
read it. It is generated whether or not `use_status_overview` is set, and is
derived from the same pipeline description the status node uses, so the two
cannot disagree about what a stage is called.

It exists because the alternative was the ROS graph, and the graph makes an
endpoint's creation depend on somebody else having created the matching one
first. Over a transport that carries data but not the graph — `rmw.ota:
zenoh_ros2dds`, where the bridge routes a topic only once a local DDS reader
exists — that is a deadlock, and a silent one: the receiving peer logs
`Initialized 1 pair(s). Pending=1` and the topic simply never arrives. The graph
is still consulted for anything the file does not name.

#### Domain bridging

If `shared.ota_domain_id` and `peer_settings.<peer>.domain_id` are both set and
differ, the generator creates a per-peer standard ROS 2 `domain_bridge.yaml` for
the `/com/...` topics and the runtime places:

- application / processing / relay nodes on the peer's `local_domain_id`
- OTA bridge (IN/OUT) processes on `ota_domain_id`
- a stock ROS 2 `domain_bridge` process between both domains (the `COM` window)

When split-domain bridging is in play and `rmw.local` is not set, the generator
falls back to `cyclone` because the stock `domain_bridge` executable needs a
deterministic DDS RMW.

If only `peer_settings.<peer>.domain_id` is set (no `shared.ota_domain_id`), the
peer's local graph is pinned to that domain but no bridge process is spun up —
typically what you want for native zenoh where the router handles cross-peer
routing.

#### DDS config generation

When the `config` field is set on either side, the generator records the template
name in each `<peer>/plugin.yaml` (`ota_config_template` / `local_config_template`)
and the runtime resolves it via the unified generator
[ws/ota_configs/get_ota_xml.py](src/rosotacom/resources/ws/ota_configs/get_ota_xml.py),
writing the resolved XML to `${peer_dir}/ota_dds.xml` (or
`${peer_dir}/local_dds.xml`) and exporting the right env var for the RMW:

- `implementation: cyclone` → `CYCLONEDDS_URI=file://<resolved_xml_path>`
- `implementation: fastdds` → `FASTDDS_DEFAULT_PROFILES_FILE=<resolved_xml_path>`

The OTA side is bootstrapped per-IN/OUT split (so the bridge can be on a different
RMW than the rest of the local graph). The local side is bootstrapped in the
session's `before_commands`, so all non-OTA splits inherit it.

Templates live under [ws/ota_configs/](src/rosotacom/resources/ws/ota_configs/)
and may reference these placeholders:

- `#host_ip` — the local peer's resolved address
- `#peer` — the remote peer's IP. For multiple peers, the template must wrap the
  per-peer region with `<!--peer-block-->...<!--/peer-block-->` markers; the
  marked region is duplicated once per peer IP. For a single peer the markers
  are optional (the generator just strips them).
- `#easy_mode_ip` — only used by `fastdds_easy_mode.xml`; resolved from
  the side-specific `easy_mode_ip` inside `shared.rmw.{local|ota}.fastdds`
  (defaults to the first peer's `address`).

Unknown `#…` placeholders in templates cause the generator to error.

#### Auto-enable of `use_in` / `use_out`
If `use_in` / `use_out` are not set, the generator derives them from the topic lists:
- Direction has topics → enabled
- Direction is empty → disabled

If you set `use_in: true`, you must also define the corresponding inbound direction (remote_to_local) under `topics:`,
otherwise the generator errors.

#### Heartbeat behavior
If `use_heartbeat: true`:
- Both directions must exist in `topics:` (they may be empty).
- `use_in` / `use_out` are set to `true` unless explicitly disabled.

Default heartbeat topic per peer: `/heartbeat_<com-name>`
Override: `peer_settings.<peer>.heartbeat_topic`
Placement in topic list: per direction via `shared.heartbeat_position` (default `prepend`).

The heartbeat is a symmetric `com_msgs/msg/EchoHeartbeat`. Each fixed-rate
message is both a new probe and a piggybacked response to the latest peer probe,
so request/response measurement does not double the configured rate. It supplies
RTT, sequence loss, health, and a minimum-RTT estimate of peer clock offset.
One-direction delay explicitly assumes symmetric paths.

Wrapped-topic offset-corrected latency requires `use_heartbeat: true`. Without
an echo estimate, exact sequence loss and uncorrected delay remain available,
while corrected latency is `null` rather than silently skew-dependent.

#### Status overview behavior
If `use_status_overview: true`:
- The generator emits a per-peer `pipeline_spec.yaml` enumerating, for each
  configured topic, the ordered pipeline stages observable on that peer.
- A `status_overview` node (catmux `STAT` window) tracks, per topic, the
  furthest stage reached and the first stage that is missing/broken, plus live
  metrics (last-message age, Hz, mean size, latency, sequence loss/reordering,
  RTT, and clock offset).
- It writes, under `session-instances/.../logs/<peer>/status/`:
  - `status.json` — machine-readable snapshot (source of truth), rewritten on a
    short interval and on every state transition;
  - `status.txt` — human-rendered table regenerated alongside the JSON;
  - `events.jsonl` — append-only state transitions and per-message wrapped-topic
    transit records.
- Read it from the host with `rosotacom status [<session>] [--json] [--watch]`.

Phase 1 samples local-domain stages directly. OTA-domain observation is
graph-only: the status node never subscribes to OTA payload topics, so enabling
the overview cannot create an additional OTA data stream. OTA activity is
inferred from the adjacent local stage plus publisher discovery and is marked as
inferred in the status output. Remote-side confirmation is reserved for a later
phase and reported as `unknown`. OTA graph observation assumes same-host
discovery of the OTA `ROS_DOMAIN_ID` (works for the bundled DDS /
`zenoh_ros2dds` examples; native `rmw_zenoh` OTA is not observed in Phase 1).

Optional deep local-stage measurement:

```yaml
shared:
  use_status_overview: true
  metric_backbone:
    record_stages: true
```

This records all generated local stages as MCAP under `logs/<peer>/metrics/`.
The in-container `stage_latency` tool joins ordered stage receive timestamps by
message index.

##### Phase 2 (planned, not implemented)

Phase 1 is per-peer local observation. A complete end-to-end view
(`source -> ... -> remote-republished`, including the remote's republish Hz and
latency) requires the two peers to exchange compact per-topic status. The
intended design, mirroring the bidirectional heartbeat path:

- Each peer publishes a small per-topic status summary on a dedicated OTA topic
  (e.g. `/ota/<peer>/__topic_status`, carried by the same transport), likely
  backed by a new `com_msgs` message type.
- The remote receives the source's "sent" view and the source receives the
  remote's "received and republished" view, so each side can fill the
  `remote_observation` field that Phase 1 leaves as `null` and render the full
  pipeline locally.

Until then, combine both peers' `status.json` files (read side by side) to reconstruct the end-to-end picture.

### `peer_settings` (optional)

```yaml
peer_settings:
  <peer_key>:
    heartbeat_topic: "/heartbeat_custom"   # optional
    domain_id: 47                          # optional; pins this peer's ROS_DOMAIN_ID for its local graph.
                                           # Required (and must be distinct across peers) when shared.rmw.ota=zenoh_connect_endpoints.
                                           # Combined with shared.ota_domain_id, enables split-domain bridging.

    inbound:
      keep_source_prefix: false            # default false
      keep_target_prefix: false            # default false (true => error)

    outbound:
      source_prefix:
        use_source_prefix: true            # default true (false => error)
        native_have_source_prefix: false   # default false

      target_prefix:
        use_target_prefix: false           # default false
        native_have_outgoing_target_prefix: false  # default = use_target_prefix (must be equal)

    framebridge:
      global_frame_prefix: "<string>"      # default: local com-name (trailing "_" removed)
      exclude_frames: ["foo", "bar"]       # default []
```

#### Behavioral notes
- `inbound.keep_source_prefix: true`
  Inbound topics keep the `/remote_name` prefix; derived inbound lists (e.g. decompression/normalize) are built accordingly.

- `outbound.target_prefix.use_target_prefix: true`
  Outbound becomes “explicitly addressed” (affects generated plugin parameters and topic monitor / heartbeat semantics).

### `topics` (optional)
You need `topics:` if you want actual bridging lists to be generated.

### Structure

```yaml
topics:
  <src>_to_<dst>:
    - "/tf"  # short form: just the base topic string
    - topic: "/camera/image"
      type: "sensor_msgs/msg/Image"
      processing: {}
      qos: {}
      zen_qos: {}
```

Each entry is either:
- A **string** (base topic only)
- A **mapping** with:
  - `topic` (**required**, string)
  - `type` (optional string; required when split-domain mode via `peer_settings.<peer>.domain_id` + `shared.ota_domain_id` is enabled)
  - `processing` (optional mapping)
  - `qos` (optional mapping)
  - `zen_qos` (optional mapping)
  - `expect` (optional mapping); `smoke_probe: false` keeps the topic in the
    generated contract but excludes it from synthetic local-smoke probes

    Two `expect` keys describe the *synthetic source* rather than the contract,
    for the runs (local smoke, `ota-smoke`, OTA benchmark genres) that drive a
    session with generated traffic instead of a replay:

    - `smoke_native_hz` — publish the native topic at this rate instead of
      deriving it from the `hz` bounds. Needed wherever the pipeline changes the
      rate (`drop`, `throttle_hz`, `trickle_hz`), because the asserted rate is
      the delivered one and the source has to run faster.
    - `smoke_native_bytes` — the payload size for a
      `com_msgs/msg/SizedPayload` topic (default 66000). The size belongs to the
      load a session wants driven: comparing two OTA middlewares, or two
      profiles, means holding the payload at a chosen size — a 38 kB camera
      keyframe, say — rather than at whatever constant the driver carries. It is
      also what the receiving side asserts, so both ends stay consistent.

In split-domain mode the generator must emit a standard ROS 2 `domain_bridge` YAML with explicit
message types for `/com/...` topics. For that reason, user-defined topic entries need `type:`.
The auto-generated heartbeat topic is the only exception because its type is known by the generator.

#### Allowed `processing` keys
Only these keys are allowed. The list is checked against the generator's
`KNOWN_PROCESSING_KEYS` by `tests/contract/test_session_configuration_doc.py`,
so a new knob cannot ship undocumented.

```yaml
processing:
  restamp_if: true                # bool OR common bool strings OR "<VAR_NAME>" (template param name)
  latch: false                    # bool, re-publish on value change only
  trickle_hz: 1                   # number > 0, receiver-side periodic re-publish
  drop:                           # optional, message drop configuration
    drop_count: 2                 # int >= 0, number of messages to drop
    window_size: 3                # int > 0, window size (drop_count must be < window_size)
  framebridge: local_to_global    # local_to_global | global_to_local
  normalize_on_target: false      # bool
  compress: false                 # bool
  use_ota_wrapper: false          # bool, wrap the payload for the OTA hop
  throttle_hz: 10                 # int > 0
  pixel_cap_preset: "wsvga"       # scalar, used as suffix only,
  transport:
    type: ffmpeg                  # ffmpeg | foxglove | compressed
    local_republish: raw          # raw | compressed; omit for no sender-side decode
    remote_republish: compressed  # raw | compressed; omit for no receiver-side decode
    # plus type-specific params (see below)
```

##### `latch` and `trickle_hz`: two answers to "the value did not change"

Both address a topic that publishes rarely, and they sit on opposite ends of
the link:

- **`latch` is a sender-side filter.** The latch stage forwards a message only
  when its value differs from the last one, to `<topic>{shared.processing_suffixes.latched}`
  (default `/latched`). It saves OTA bandwidth on a state topic that repeats
  itself. Its cost is that a receiver which subscribes *after* the last change
  gets nothing — what the delivered value is then depends entirely on the OTA
  QoS for the role (`depth`, `lifespan`, `durability`).
- **`trickle_hz` is a receiver-side re-publish.** A local timer republishes the
  last *delivered* value to `<final>/trickle` at that rate, whether or not
  anything new arrived. Nothing extra crosses the link: the trickle topic is
  not in the OTA topic lists. It exists for consumers that expect a steady
  stream — visualization, state diagrams, watchdogs — and would otherwise treat
  an unchanged value as a dead one.

They compose: `latch: true` with `trickle_hz: 1` sends a message only on change
and still gives the receiving side a 1 Hz stream.

Because the trickle output is the real delivered topic, it is the monitored
`native_in` stage rather than the pre-trickle `final` — otherwise the
re-publish would be an observability blind spot whose rate no `expect` could
assert. `expect.hz` on such a topic therefore describes the trickle rate plus
whatever arrives over the link, which is why `examples/sessions/11_trickle`
asserts `hz: {min: 2, max: 8}` for a 1 Hz source at `trickle_hz: 4`.

`trickle_hz` must be greater than 0. It also sets the expected rate for a topic
that has no `throttle_hz`.

#### Processing pipeline order (exact)
Given a base topic like `/tf`, stages are applied in this order:

1) base topic (e.g. `/tf`)
2) restamp → `+ shared.processing_suffixes.restamped` (default `/restamped`)
3) latch → `+ shared.processing_suffixes.latched` (default `/latched`)
4) drop → `+ /drop{drop_count}of{window_size}` (e.g. `/drop2of3`)
5) throttle → `+ /max{hz}hz`
6) pixel cap → `+ /{preset}`
7) framebridge:
   - `local_to_global`: appended to the current topic state via `+ /globalframe`
   - `global_to_local`: appended to the base topic via `+ /globalframe` (configured inbound-side)
8) compress → `+ /<algorithm>` (default `/bz2`)
9) transport → `+ /<type>` (e.g. `/ffmpeg`, `/foxglove`, `/compressed`)
10) OTA wrapper → `+ /ota_stamped`

The wrapper is **always last**. Its `seq` and send stamp are read as a statement
about the traffic on the link, so it has to wrap exactly what crosses it — an
encoded stream is wrapped as `<base>/<type>/ota_stamped`, not the other way
round. Nothing may append a suffix after it, or the delivered topic would be
renamed under a receiver that subscribes by a fixed name.

The receiver undoes the chain in reverse, and every stage there is computed from
the delivered topic rather than from the OTA topic:

1) unwrap → republishes on the pre-wrap name (so wrapping renames nothing)
2) `remote_republish: <transport>` → reverse transport decodes into `+ /<transport>`
3) decompress → republishes on the pre-compress name
4) framebridge `global_to_local` → the local base topic
5) trickle → `+ /trickle`, the only receiver-side stage that adds a suffix

##### The two reverse republishes

An encoded stream crosses the link as packets — `FFMPEGPacket`, `CompressedVideo`
— and something has to turn those back into an image. That decode can happen in
two places, and they answer different questions:

- **`remote_republish`** runs on the receiving peer and is the one that matters:
  it produces the topic the receiving application subscribes to (`<encoded>/raw`
  or `<encoded>/compressed`), and it is what `_delivered_topic` reports as the
  delivered stage.
- **`local_republish`** runs on the *sending* peer, against the stream it just
  encoded. Nothing crosses the link for it; it exists so the sending machine can
  see approximately what the receiver will get.

Both name the image transport they publish in rather than being switched on:

| value | delivered type | cost |
|---|---|---|
| `raw` | `sensor_msgs/msg/Image` | the transport undone and nothing else, at full frame size — width x height x 3 per frame |
| `compressed` | `sensor_msgs/msg/CompressedImage` | one JPEG encode per frame on top of the codec |

Omitting a key means that side does not decode. There is deliberately **no
default**: `type: compressed` already delivers an image, so a decode that
switched itself on would produce exactly the full-size copy a session chose that
transport to avoid.

**What the difference is worth**, measured on an operator drive rather than
estimated (1920x1200 H.264 at gop 5, the recording carrying both the packets and
the compressed twin of the same frames, so the raw twin could be reproduced by
decoding):

| | |
|---|---|
| size | **14.6x** — 6.91 MB vs 472 kB per frame, 65.2 vs 4.45 MB/s |
| quality | **PSNR 44.9 dB** mean (44.0 worst), **SSIM 0.9959** |

Read the second row next to what the codec itself costs on the same kind of
stream — p50 ~30 dB for the H.264 encode, ~19 dB for an artifact frame after a
lost keyframe — and the extra JPEG generation is roughly 15 dB below the loss
already present.

The decisive number is neither: what crossed the link for those frames was
**12.9 kB**. A raw twin is ~500x the payload it decodes, so the choice is about
what each machine publishes locally and writes into a `record -a`, not about the
link. Sessions that measure the *codec* — PSNR/SSIM against the pre-encoder
frame, as `examples/sessions/17_synthetic_camera_quality` does — want `raw` on
both sides for exactly that reason: a JPEG twin folds its own loss into the
number being reported.

#### Transport parameters
`type: ffmpeg` supports:
- `gop_size` (int)
- `bit_rate` (int)
- `encoder_av_options` (string)

#### Receiver-side playout pacing

A receiver that hands every packet to the decoder the moment it arrives turns
network delay jitter into display stutter. `com_py playout_pacer` re-times a
received packet stream to `stamp + budget` (budget adaptive above the fastest
observed path, clock-offset-proof; late packets pass through immediately, order
always preserved — the decoder chain sees every packet). On the 2026-08-17 cellular
field trace this removes ~95 % of delay-caused >200 ms display gaps at ~100 ms
median added age (adaptive mode).

Declaratively, add a `playout` block to the transport:

```yaml
        transport:
          type: "ffmpeg"
          gop_size: 3
          remote_republish: compressed
          playout:            # receiver-side; requires remote_republish
            adaptive: true    # default true; false uses target_ms as a fixed budget
            target_ms: 350    # fixed budget when adaptive: false
            min_ms: 100       # adaptive clamp above the observed delay floor
            max_ms: 800
            republish: paced  # paced (default) | unpaced | both
```

The receiving peer then runs the pacer in its own `PACE` window against the
arrived encoded stream, and the reverse republish decodes `<encoded>/paced`
instead of the raw arrival — the delivered topic name is unchanged, which is
why `playout` requires `remote_republish`: without a republish the pacer's
`/paced` suffix would rename what the receiving application subscribes to.
The sender's own preview decode (`local_republish`) is never paced — it reads
the local copy of what was just encoded, so there is no jitter to absorb.

##### `republish`: which stream reaches the display

Pacing under an unchanged topic name is what keeps applications working, and it
is also what makes the improvement invisible — nothing shows paced and unpaced
together. `republish` names which stream the receiver decodes:

| value | decodes | publishes |
|---|---|---|
| `paced` (default) | `<encoded>/paced` | `<encoded>/<remote_republish>` |
| `unpaced` | `<encoded>` | `<encoded>/<remote_republish>` |
| `both` | both | delivered as above, comparison on `<encoded>/unpaced/<remote_republish>` |

**The delivered name is the same in all three.** Only what feeds the decode
moves, so switching modes never changes what an application subscribes to.

`unpaced` still runs the pacer: `/paced` and its `budget_ms` / `queue_depth`
debug topics are published and recordable, they just do not reach the display.
That is the A/B control — measure a pacer on a live link without changing what
an operator sees.

`both` runs the receiver's expensive stage twice (a codec decode plus, for
`compressed`, a JPEG encode per frame) and adds one local topic; nothing extra
crosses the link. It also consumes one of the four reverse-transport slots. It
is a rehearsal and diagnosis switch — the comparison stream is deliberately not
a status pipeline stage, because it is not something the link promises to
deliver.

The node also runs standalone against any packet stream:

```bash
ros2 run com_py playout_pacer --ros-args \
  -p topic:=/cam/image/compressed/drop1of2/ffmpeg \
  -p target_ms:=350.0 -p adaptive:=true
```

It republishes on `<topic>/paced` plus three topics next to it:
`.../paced/budget_ms` and `.../paced/queue_depth` at 1 Hz, and
`.../paced/hold_ms` with every release, carrying how long *that* message was
actually held.

##### What the status overview does with it

A paced stage's age is the link plus a delay this session asked for, and one
number cannot be judged against a threshold meant for the link. So the
generator names the pacer's report on the stage it explains
(`paced_hold_topic` in `pipeline_spec.yaml`), and `status_overview` takes the
hold back out before classifying:

```
native_in  FLOWING  .../ffmpeg/compressed  10.0Hz 286ms(link 46+paced 240) ...
```

`latency_ms` stays the age on screen; `link_latency_ms` and `pacing_hold_ms`
are the split, and the quality verdict uses the former. Without this the
default 200 ms bad-threshold — set before anything on a link was held on
purpose — marks every healthy paced stream BAD, which is how an operator learns
to stop reading the panel.

It is the *applied* hold that is subtracted, never the configured budget: a
message that arrived later than its release time passes straight through, is
reported as held for 0 ms, and still flags. A budget-based correction would
credit that message a buffer it never received and quietly stop reporting a slow
link. Before the pacer has reported at all, nothing is subtracted — neither zero
nor the budget is a safe guess.

`type: foxglove` supports (CompressedVideo):
- `gop_size` (int)
- `bit_rate` (int)
- `encoder_av_options` (string)
- `qmax` (int)

`type: compressed` supports:
- `jpeg_quality` (int)

Other values: **error**.

#### `qos`
Any mapping keys are written to `qos.yaml`.
Special logic: if `depth` is set but `history` is missing, the generator adds `history: keep_last`.

#### `zen_qos`
Only used when `shared.rmw.ota: zenoh_ros2dds` is set (the bridge config is baked into the generated `zenoh.json5`). Ignored for native `rmw.ota: zenoh_connect_endpoints` and for non-zenoh RMWs.

```yaml
zen_qos:
  priority: real_time  # required string
  express: true        # optional bool
```

### What gets generated
For a session configuration input file, `rosotacom start` writes generated files
to the active runtime instance under `session-instances/<date>/<run>/config/`.
The static `sessions/<name>/` definition/template directory is left unchanged.
The generator typically produces:

- `*_to_*_topics.txt` (topic list files per direction)
- Per peer directory:
  - `<peer>/plugin.yaml`
  - `<peer>/session_specification.yaml`
  - `<peer>/domain_bridge.yaml` (only when `peer_settings.<peer>.domain_id` and `shared.ota_domain_id` are both set and differ)
- Optionally:
  - `qos.yaml` (if `shared.qos` or any `topic.qos` exists)
  - `<peer>/compression.yaml` (if that peer compresses outbound topics)
  - `<peer>/decompression.yaml` (if that peer needs to decompress inbound topics)
  - additional per-feature config files depending on enabled processing stages

### Examples
Examples can be created with `rosotacom examples create ./rosotacom_examples`.
The copied project stores sample session configuration files under `sessions/`.

## Session parametrization (`session-parametrization.yaml`)
If `session-parametrization.yaml` is present, the session is defined indirectly by selecting a template and providing its parameters.

```yaml
load_template:                      # required
  filepath: ./session-template.yaml # required
  parameters: {}                    # optional
```

### `load_template`

```yaml
load_template:
  filepath: ./session-template.yaml
  parameters:
    SOME_PARAM: value
```

### `filepath` resolution
- **Absolute path**: must exist.
- **Relative path**: resolved relative to the directory containing the session config input file.
- **`/session/...` convenience path**: the generator tries to map it to the repo’s `/ws/session/...` location
  (host vs container convenience).

## Template file
The template file is similar to a session definition, but it may contain:
- `input_parameters: { ... }` declarations, and
- `${VAR}` placeholders that get substituted from `load_template.parameters`.

### `parameters` rules
- Extra parameters not declared under `input_parameters` in the template: **error**.
- Missing parameters without a default in `input_parameters`: **error**.
- Missing parameters with a default: default is used.
