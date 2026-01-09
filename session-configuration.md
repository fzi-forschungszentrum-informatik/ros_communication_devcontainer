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
  - **throttle** (`throttle_hz`)
  - **pixel cap** (`pixel_cap_preset`)
  - **image transport** (`transport`)
  - **normalize** (`normalize_on_target`)

- **Not implemented / unsupported combinations (will error)**
  - `shared.compression.remove_algorithm_suffix_on_decompression: false` (must be `true` if set)
  - `peer_settings.<peer>.outbound.source_prefix.use_source_prefix: false`
  - `peer_settings.<peer>.inbound.keep_target_prefix: true`
  - `peer_settings.<peer>.outbound.target_prefix.native_have_outgoing_target_prefix != use_target_prefix`

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
    ip_key: <string>         # required (after template substitution: the final IP/hostname string)
    com-name: <scalar>       # optional, default: <peer_key>
```

- **`peer_key`**: used to form direction keys like `<src>_to_<dst>`.
- **`ip_key`**: becomes the address used in the generated `plugin.yaml` (`ip_local` / `ip_remote`).
- **`com-name`**: peer communication name (default = `peer_key`); affects e.g. topic prefixes.

### `shared` (optional)

```yaml
shared:
  use_topic_monitor: false   # default false
  use_heartbeat: false       # default false

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

  zenoh:                     # optional (if you want to use a Zenoh-Router for over the air communication rather than direct RMW communication via Cyclone DDS)
    transport: "udp"         # default udp
    main_peer: "<peer_key>"  # required if zenoh is set
    main_port: 7447          # required if zenoh is set (int)
```

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

### `peer_settings` (optional)

```yaml
peer_settings:
  <peer_key>:
    heartbeat_topic: "/heartbeat_custom"   # optional

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
      processing: {}
      qos: {}
      zen_qos: {}
```

Each entry is either:
- A **string** (base topic only)
- A **mapping** with:
  - `topic` (**required**, string)
  - `processing` (optional mapping)
  - `qos` (optional mapping)
  - `zen_qos` (optional mapping)

#### Allowed `processing` keys
Only these keys are allowed:

```yaml
processing:
  restamp_if: true                # bool OR common bool strings OR "<VAR_NAME>" (template param name)
  framebridge: local_to_global    # local_to_global | global_to_local
  normalize_on_target: false      # bool
  compress: false                 # bool
  throttle_hz: 10                 # int > 0
  pixel_cap_preset: "wsvga"       # scalar, used as suffix only,
  transport:
    type: ffmpeg                  # ffmpeg | compressed
    local_republish: false        # default false
    # plus type-specific params (see below)
```

#### Processing pipeline order (exact)
Given a base topic like `/tf`, stages are applied in this order:

1) base topic (e.g. `/tf`)  
2) restamp → `+ shared.processing_suffixes.restamped` (default `/restamped`)  
3) throttle → `+ /max{hz}hz`  
4) pixel cap → `+ /{preset}`  
5) framebridge:
   - `local_to_global`: appended to the restamped outbound topic via `+ /globalframe`
   - `global_to_local`: appended to the base topic via `+ /globalframe` (configured inbound-side)
6) compress → `+ /<algorithm>` (default `/bz2`)  
7) transport → `+ /<type>` (e.g. `/ffmpeg`, `/compressed`)  
8) optional `local_republish: true` triggers reverse-transport configuration.

#### Transport parameters
`type: ffmpeg` supports:
- `gop_size` (int)
- `bit_rate` (int)
- `encoder_av_options` (string)

`type: compressed` supports:
- `jpeg_quality` (int)

Other values: **error**.

#### `qos`
Any mapping keys are written to `qos.yaml`.
Special logic: if `depth` is set but `history` is missing, the generator adds `history: keep_last`.

#### `zen_qos`
Only used if `shared.zenoh` is set; otherwise ignored.

```yaml
zen_qos:
  priority: real_time  # required string
  express: true        # optional bool
```

### What gets generated
For a session directory containing a session configuration input file, the generator typically produces:

- `*_to_*_topics.txt` (topic list files per direction)
- Per peer directory:
  - `<peer>/plugin.yaml`
  - `<peer>/session_specification.yaml`
- Optionally:
  - `qos.yaml` (if `shared.qos` or any `topic.qos` exists)
  - `<peer>/compression.yaml` (if that peer compresses outbound topics)
  - `<peer>/decompression.yaml` (if that peer needs to decompress inbound topics)
  - additional per-feature config files depending on enabled processing stages

### Examples
Examples can be found in the `ws/example/` directory of this repository. For hands-on sample session configuration files.

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
