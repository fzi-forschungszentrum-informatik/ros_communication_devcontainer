# Geo Link-Quality Map

`rosotacom geomap` joins recorded link-quality samples with GPS or pose samples
and writes three offline artifacts:

- a stable CSV of georeferenced samples, and
- an HTML report plus sibling `.route.png` image colored by one selected metric.

The command is for post-run analysis: it answers where along a drive route the
recorded link degraded. It does not run a live map and does not need a tile
server.

## Inputs

GPS/route samples can come from either:

- `--bag BAG --gps-topic TOPIC`, reading a `sensor_msgs/msg/NavSatFix`,
  `gps_msgs/msg/GPSFix`, `geometry_msgs/msg/PoseStamped`, or
  `nav_msgs/msg/Odometry` topic from a rosbag2 bag; or
- `--gps-csv gps.csv`, for host-only fixtures or legacy CSV exports.

Bag reading imports `rosbag2_py`, `rclpy`, and `rosidl_runtime_py` lazily, so a
plain Python host can still use the CSV workflow. Local pose and odometry topics
need `--origin-lat` plus `--origin-lon`; x is interpreted as east meters and y as
north meters from that origin.

Metric samples can come from one or both of:

- `--trace link_trace.jsonl`, using `observed_tx_kbps`,
  `observed_rx_kbps`, `rtt_ms`, or `loss_pct`; and
- `--events events.jsonl`, using binned transit `delivery_pct`,
  `event_loss_pct`, or `ota_hop_ms`.

## Time Alignment

The join is explicit about clocks. GPS samples keep bag/header/CSV time. Trace
and event samples keep their recorded timestamp. `--trace-to-gps-offset-s` is
added to every trace/event timestamp before nearest-neighbor matching:

```text
aligned_metric_time_s = source_metric_time_s + trace_to_gps_offset_s
```

Use `0` only when both sources already share the same epoch. If the GPS CSV uses
bag-relative seconds and a link trace uses epoch wall time, pass the offset that
converts trace time into the bag-relative domain. Samples whose nearest GPS point
is farther away than `--max-gap-s` are omitted from the CSV and map.

## Example

```bash
rosotacom geomap \
  --bag session-instances/.../rosbags/a/native \
  --gps-topic /fix \
  --trace session-instances/.../logs/a/status/link_trace.jsonl \
  --metric observed_tx_kbps \
  --trace-to-gps-offset-s 0 \
  --max-gap-s 1.0 \
  --out-csv artifacts/geo-link-quality.csv \
  --out-html artifacts/geo-link-quality.html
```

Open the HTML file in a browser next to its sibling `.route.png` file. The route
is drawn as an offline raster image: green is better quality, red is worse
quality, with the selected metric shown in the legend. Exact sample rows and
latitude/longitude values are written to the CSV artifact, not embedded in the
HTML.

## Host-Only Smoke

This fixture smoke runs without ROS installed:

```bash
tmp="$(mktemp -d)" && \
printf 'time_s,latitude,longitude\n100,49.0000,8.0000\n101,49.0002,8.0003\n' > "$tmp/gps.csv" && \
printf '%s\n' \
  '{"kind":"link_trace","monotonic_s":0,"passive_counter_delta":{"tx":{"observed_kbps":1200}}}' \
  '{"kind":"link_trace","monotonic_s":1,"passive_counter_delta":{"tx":{"observed_kbps":450}}}' \
  > "$tmp/link_trace.jsonl" && \
rosotacom geomap --gps-csv "$tmp/gps.csv" --trace "$tmp/link_trace.jsonl" \
  --trace-to-gps-offset-s 100 --max-gap-s 0.1 \
  --metric observed_tx_kbps --out-csv "$tmp/geo.csv" --out-html "$tmp/map.html"
```

Expected output:

```text
Wrote 2 georeferenced samples to .../geo.csv
Wrote geo link-quality map to .../map.html
```

Open `map.html`; it should show a two-point route colored by observed transmit
throughput using the sibling `map.route.png` image.
