# Link bandwidth: what each number means

Three bandwidth numbers exist in a running session, and they answer three
different questions. Mixing them up is how a link that delivered ~6 Mbit/s was
reported at ~28 Gbit/s for a whole field run.

| Topic / field | Measured where | Counts | Scope |
|---|---|---|---|
| `/topic_monitor/<dir>/<name>[/to_<peer>]/ros_topic_bandwidth_kbps` | subscriber on the local domain | serialized ROS payload of every `/com/<dir>/<name>/…` stage topic | one peer, one direction, application payload only |
| `/topic_monitor/<dir>/<name>[/to_<peer>]/link_bandwidth_kbps` | `/proc/net/dev` counters of the OTA interface | every byte the kernel moved over that interface | the whole interface, all peers and all protocols on it |
| `status.json` → `link` | the same counters, plus the overview's own per-stage sizes and rates | both of the above, and their ratio | one peer, both directions |

`kbps` here is **Kibit/s** — bytes × 8 / 1024 / seconds. It has meant that
since the topic existed; the name is kept so old recordings keep their meaning.
The factor to Mbit/s is 1.024/1000, i.e. 6000 `kbps` ≈ 6.1 Mbit/s.

## Which one answers "what does the camera cost the link"

Neither `link_bandwidth_kbps` alone. It is an *interface* number: it includes
DDS discovery, RTPS retransmits, ACKs, IP/UDP headers, and anything else
sharing the interface. That is what makes it useful — the overhead is exactly
what a payload sum cannot see — but it is not attributable to one stream.

The attributable number is the per-topic row in `topic_monitor`'s table (or the
per-stage `mean_size_bytes` × `hz` in `status.json`), and the honest way to
report the cost of a stream on the link is the pair: payload for the stream,
plus the session-level `overhead_ratio_out` / `overhead_ratio_in` from
`status.json`, which is `link / payload` at the OTA boundary. A ratio near 1
means the wire carries little beyond the payload; well above 1 means
retransmits, shadow connections, discovery chatter, or QoS that does not fit
the link. Small messages have a naturally higher ratio, because per-packet
headers are a larger share of them.

Per-remote-peer wire attribution is **not** available from these counters. On
the deployments this stack runs on, the OTA interface is a dedicated tunnel
carrying exactly one peer, so the interface number *is* the peer number. A
setup that multiplexes peers on one interface needs a per-peer filter (`iftop`
in the `NET` window, or packet accounting), and the `link` numbers should then
be read as an upper bound.

## How the interface is found

From `ip_local` — the peer's own OTA address — by `find_interface_for_ip`,
inside the node. `topic_monitor` and `status_overview` share
`resolve_link_interface`, so they cannot disagree. An explicit `interface`
parameter overrides it for a setup an address cannot describe.

**A loopback interface is refused** when `ip_local` is not itself a loopback
address, because loopback carries this host's own intra-process ROS traffic.
`topic_monitor` fails to start; `status_overview` disables the `link` block and
warns. Both are better than the number that made this rule necessary.

### What went wrong before (issue #267)

The interface used to be resolved in the session template:

```yaml
- interface=$(ip -o addr | awk '/'${ip_local//./\\.}'\/[0-9]+/ {print $2; exit}')
```

catmux substitutes `${name}` and nothing else — see
`tests/contract/test_session_template_substitution.py`. `${ip_local//./\\.}` is
not that form, so it reached the pane literally, bash expanded an unset shell
variable to the empty string, and the awk program became `/\/[0-9]+/` — "the
first address line with a prefix length", which is `lo` on every Linux host.

Both peers therefore reported loopback traffic as link bandwidth for the whole
2026-08-13 field session. On the centre it read ~5–7 Mbit/s and looked
plausible. On the vehicle, which was also writing a 36 GB `-a` recording over
the same loopback, it read ~28 Gbit/s — near-constant, and identical in both
directions, which is the loopback signature (`rx_bytes` and `tx_bytes` are the
same bytes). `status_overview`, which already resolved the interface in the
node, reported `tun1` correctly in the same run.
