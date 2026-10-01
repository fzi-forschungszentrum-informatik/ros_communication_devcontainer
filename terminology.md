# Terminology

This document defines the canonical terminology used throughout this project.
All contributors **MUST** use the terms defined here. Synonyms are explicitly discouraged.

The terminology separates **orthogonal concerns**:
- *who communicates* (peers)
- *where code runs* (local vs remote)
- *what kind of topic it is* (application, com, OTA)
- *where a message is in the pipeline* (stages)
- *what transformations have been applied* (processing)

---

## 1. Peers and sessions

### Peer
A **peer** is a communication participant.
Peers are symmetric and may both send and receive messages.

> The term *peer* is used exclusively.
> Terms such as *actor*, *participant*, *partner*, or *node* are not used.

---

### Local peer
The **local peer** is the peer on which a given session instance is running
(i.e. “this machine / this ROS graph”).

---

### Remote peer
The **remote peer** is the peer on the other side of the communication session.

---

### Session
A **session** is a logical, bidirectional communication relationship between two peers.

---

### Session instance
A **session instance** is a concrete runtime instantiation of a session on a peer.

> A peer may host multiple session instances.

---

### Session runtime
A **session runtime** is the software runtime that implements a session instance on a peer
(e.g. App Relay + OTA Bridge and their configuration/orchestration).

In this repository, the session runtime is typically executed inside a Docker container started via `rosotacom`.

---

### Session container
A **session container** is the Docker container that hosts a session runtime for **one** session instance on **one** peer.

> If a peer runs multiple session instances, it runs multiple session containers (one per session instance).

---

### Peer key
A **peer key** is the peer identifier used in session configuration files.

Characteristics:
- Used under `peers:` (e.g. `peers.<peer_key>`)
- Used in direction keys like `<src>_to_<dst>`
- Used as the CLI `--identity` value (selects which peer directory to run on a given machine)

---

### Peer communication name (com-name)
A peer’s **communication name** (**com-name**) is the name used for communication-facing identifiers e.g. in the form of topic prefixes.

> **Peer key and com-name may be identical, but they are distinct concepts.**

---

### Session directory (session dir)
A **session directory** is the on-disk directory that contains:
- the user-authored session configuration input (definition or parametrization), and
- all generator outputs for that session (direction topic lists, shared config files, per-peer directories, ...).

---

### Peer directory (peer dir)
A **peer directory** is the per-peer subdirectory inside a session directory:

`<session_dir>/<peer_key>/`

It contains **only** peer-specific generated session configuration artifacts
(e.g. `plugin.yaml`, `session_specification.yaml`, optional compression/decompression configs, ...).

---

## 2. Message direction and endpoints

### Inbound
**Inbound** describes messages arriving *at the local peer*.

---

### Outbound
**Outbound** describes messages sent *by the local peer*.

---

### Source
The **source** is the logical originator of a message.

- For outbound messages: the local peer is the source.
- For inbound messages: the remote peer is the source.

---

### Target
The **target** is the intended recipient of a message.

- For outbound messages: the remote peer is the target.
- For inbound messages: the local peer is the target.

> **Source and target are semantic roles, not transport direction.**

---

## 3. Topic classes (message lifecycle)

Topics are classified by their role in the communication pipeline.

### Application topic
An **application topic** is any topic that is part of the application-facing ROS API.

An application topic is either:
- a **native topic** (raw), or
- a **processed topic** (native + processing pipeline suffix).

> Use **application topic** when you mean “native-or-processed”, i.e. you don’t want
> to commit to whether the topic is raw or processed.

Characteristics:
- Exists even if the communication stack is disabled
- No communication-specific prefixes
- May or may not have a processing pipeline suffix
- Represents application-level data (raw or processed)

Example: `/sensors/camera/image_raw`

---

### Native topic (raw application topic)
A **native topic** is the raw, application-owned topic (no processing suffix) that exists
independently of any communication mechanism.

Characteristics:
- Exists even if the communication stack is disabled
- No communication-specific prefixes
- No processing pipeline suffixes
- Represents raw application data

Example: `/sensors/camera/image_raw`

---

### Processed topic (application processed topic)
A **processed topic** is an application topic that has a **processing pipeline suffix**,
but still has **no special namespace** (i.e. it is not `/com/...` or `/ota/...`).

Processed topics exist **only for topics that need processing**.

Example: `/sensors/camera/image_raw/restamped/max20hz`

---

### Com topic (communication topic)
A **com topic** is a topic within the internal communication namespace.

Characteristics:
- Always under the `/com/...` namespace
- Annotated with direction and endpoints

Examples:
```
/com/out/A/to_B/sensors/camera/image_raw
/com/in/A/to_B/sensors/camera/image_raw
```

---

### OTA topic (transport topic)
An **OTA topic** is a topic under the transport namespace that is actually transmitted
over the air / network.

Characteristics:
- Always under the `/ota/...` namespace
- Transport-bound
- Represents serialized, transmitted data

Example: `/ota/A/to_B/sensors/camera/image_raw`

---

### Canonical topic lifecycle
A message typically flows through the following stages.

When no application-side processing is configured:
```
application topic (native)
→ com topic
→ ota topic
→ com topic
→ application topic (native)
```

When application-side processing is configured (outbound preprocessing + inbound postprocessing):
```
local native topic
→ local processed topic          (preprocessing; optional)
→ local /com/out topic
→ /ota topic
→ remote /com/in topic
→ remote processed topic         (postprocessing; optional)
→ remote native topic
```

For inbound messages the direction is analogous (swap outbound/inbound roles for the local peer).

---

## 4. Communication pipeline stages

### Stage prefix
A **stage prefix** indicates where a topic is located in the communication pipeline.

Examples:
```
/com/in
/com/out
/ota
```

Stage prefixes answer the question:

> “At which stage of the communication pipeline is this message?”

---

## 5. Endpoint addressing

### Endpoint prefix
An **endpoint prefix** encodes the source and target peers in the topic namespace.

Canonical form: {source}/to_{target}

Example: `A/to_B`

---

### Communication prefix
The **communication prefix** is the concatenation of:

stage prefix + endpoint prefix

Example: `/com/out/A/to_B`

---

## 6. Base topic

### Base topic
The **base topic** is the application-level topic name without any communication
or processing metadata.

Example: `sensors/camera/image_raw`

---

## 7. Processing and transformations

### Processing stage
A **processing stage** is a single transformation applied to a topic.

Examples:
```
restamped
max20hz
bz2
```

---

### Processing pipeline suffix
A **processing pipeline suffix** is an ordered sequence of processing stages
encoded as path suffixes.

Example: /restamped/max20hz/bz2

Processing stages are ordered left-to-right in the sequence they are applied.

Notes:
- Processing can exist on **application topics** (processed topics) and can also be present
  on **com** and **OTA** topics as suffixes.
- Processing has **no dedicated namespace** (currently). The suffix encoding is the signal.
- Preprocessing/postprocessing are just processing stages applied at different points:
  - outbound: native → processed (optional)
  - inbound: processed → native (optional)
- Being postprocessing does **not** by itself make a transformation a processing
  stage of this project. Which transformations belong here is decided by
  §9.1, not by where or when they run.

---

## 8. Fully qualified topic

### Fully qualified topic
A **fully qualified topic** consists of:
- communication prefix:
- base topic
- processing pipeline suffix


Example: `/com/out/A/to_B/sensors/camera/image_raw/restamped/max20hz/bz2`

---

## 9. Module responsibilities

### 9.1 Stage ownership (which transformations belong to this project)

A **processing stage** (§7) belongs to this project when it either

1. **undoes a stage this project applied** — decompression undoes compression,
   the OTA unwrapper undoes the OTA wrapper, `global_to_local` undoes
   `local_to_global`, `trickle` undoes `latch`; or
2. **compensates for the medium** — the playout pacer against the arrival
   jitter a link introduces.

Everything else that happens to a stream after the inbound native topic
belongs to whoever consumes it, and is that consumer's code in that
consumer's repository. The inbound native topic (§3) is where this project's
canonical lifecycle ends, and stage ownership ends with it.

**Two criteria that do not decide it**, because both are true of stages on
either side:

- *Where it runs.* Decompression, the unwrapper, `trickle` and the pacer all
  run at the receiving peer. So does an application transform that has nothing
  to do with the link.
- *That it is postprocessing.* §3 names inbound postprocessing as part of the
  lifecycle, so the word describes a position, not an owner.

`trickle` is the stage that makes the boundary feel ambiguous, and rule 1
resolves it. It can be read as a fix for a consumer that expects a periodic
stream — but the question is not what it is good for, it is why the value was
latched in the first place: repeating a state at a fixed rate over a metered
link is waste, so latching is a link decision and `trickle` is its inverse.
Without the link there would be nothing to un-latch.

**The quick test is a counterfactual.** Change the *link* — other radio, other
route, other codec — and ask whether the stage has to change; then change the
*consumer* — other display, other viewer, an automated pipeline instead of a
person — and ask the same. A stage that survives every link change and breaks
on a consumer change is the consumer's.

**The falsifiable test, for cases the counterfactual leaves open.** A stage
here becomes part of the session contract, so it must carry an `expect:` the
harness can assert. If no `hz`, `loss_pct`, `latency_ms` or
`completeness.vs_bag_ratio` on that stage would mean anything, the contract
cannot check it — and a stage the contract cannot check does not belong in the
contract.

This is about **stages, not about code**. A generic operation — resampling an
image, filtering a point cloud — may live here as a reusable node the day a
second session needs it. What must not move here is the decision that a
particular consumer wants a particular result.

---

### App Relay
The **App Relay** (application relay) maps between application topics and com topics.

Responsibilities:
- application → com (outbound)
- com → application (inbound)

The App Relay consists of two pub/sub pairs:
- **application-side pub/sub pair**: faces **application topics** (native or processed)
- **com-side pub/sub pair**: faces **com topics** (`/com/in/...` and `/com/out/...`)

---

### OTA Bridge
The **OTA Bridge** maps between com topics and OTA topics.

Responsibilities:
- com ↔ ota
- transport binding and serialization

The OTA Bridge consists of two pub/sub pairs:
- **com-side pub/sub pair**: faces **com topics**
- **OTA-side pub/sub pair**: faces **OTA topics** (`/ota/...`)

---

## 10. Terminology rules (normative)

- “Local” is used **only** in the sense of *local peer*.
- Native topics are **never** called “local topics”.
- Topic classes are named explicitly: *application* (native or processed), *native*, *processed*, *com*, *OTA*.
- All documentation and code comments **must** use these terms.
- New terminology must not be introduced without updating this file.
