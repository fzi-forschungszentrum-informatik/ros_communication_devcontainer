# ROS Communication DevContainer

The ROS Communication DevContainer is a Docker-based solution designed to streamline the bidirectional synchronization of ROS2 topics between two Linux machines. It provides built-in compression and routing capabilities for over-the-air (OTA) data transfer: selected topics are remapped into an OTA namespace and transmitted either via direct DDS (CycloneDDS) or through a Zenoh router. This project aligns with the publication *“Scalable Remote Operation for Autonomous Vehicles: Integration of Cooperative Perception and Open Source Communication.”*

<details>
<summary>Key Features</summary>

- **Minimal Dependencies**: Only Docker is needed to get started, simplifying the setup process.
- **Isolation**: Operates in a separate Docker container, ensuring minimal impact on existing ROS setups.
- **Centralized Configuration Management**: All configurations are stored and managed in this repository.
- **Compression**: Built-in compression capabilities for efficient data transfer.
- **QoS Configuration**: Flexible Quality of Service settings for optimized communication.

</details>

## Getting Started

### Prerequisites

- Docker installed on all machines
- Git for configuration management
- Machines connected to the same network (VPN or local WLAN)

### Convenience CLI: `rosotacom`

This repository's main entrypoint for starting ROS communication sessions is
`run_session_in_container.py`.
For convenience, you can expose it as a short command (e.g. `rosotacom`) via a symlink into `~/.local/bin`.

#### Install

```bash
cd /path/to/ros_communication_devcontainer && ./install_rosotacom.sh
```

Make sure `~/.local/bin` is in your `PATH` (often already true on Ubuntu).

### Basic Setup

1. Fork this repository

2. Configure your environment:
   - `data_dict.json`: Maps semantic names to resources (example in `data_dict.json.examples`)
   - `local.json`: Container-specific settings (example in `local.json.examples`)

3. Write the session configuration (user-facing) that defines the OTA communication behavior:
   - both peers
   - topic directions (`<src>_to_<dst>`)
   - optional QoS, processing (restamp, compression, transports, …), Zenoh routing, …
   - Reference: `session-configuratoin.md`
   - Terminology / naming conventions (peer, application/com/OTA topics, inbound/outbound, …): `terminology.md`

4. Put the config into a **session directory** (this directory is the input and also the output location for generated files):
   - `session-definition.yaml` (self-contained), OR
   - `session-parametrization.yaml` (template + parameters; generator also writes a resolved `session-definition.yaml`)

5. Run `rosotacom` on **each peer** with the same session directory, but with its local peer key (`--identity`):

```bash
# on peer "a"
rosotacom --session-dir /path/to/session_dir --identity a

# on peer "b"
rosotacom --session-dir /path/to/session_dir --identity b
```

That’s it: `rosotacom` will read the session config input file and automatically create/update all required generated files in that directory (per-peer plugin/session specs, direction topic lists, optional compression/decompression, optional `qos.yaml`, …).

## Usage Examples

### Starting Points: Examples

To help you get started, we have provided several examples that showcase the basic functionality of the ROS Communication DevContainer. These examples are designed for ease of use and quick setup.

<details>
<summary>Configuring Machine Data</summary>

The examples require two machines, which we will refer to as `machine_a` and `machine_b`. Choose your machines and fill out the `machine_a_ip` and `machine_b_ip` fields in your `data_dict.json`. Make sure this data is synchronized across both machines via Git to ensure seamless communication.

</details>

<details>
<summary>Example 1: Heartbeat</summary>

This example starts a minimal session that only exchanges heartbeat messages between two peers.

- On `machine_a`, run:
  ```bash
  ./example/1_heartbeat/run_machine_a.sh
  ```
- On `machine_b`, run:
  ```bash
  ./example/1_heartbeat/run_machine_b.sh
  ```
- Confirm the heartbeat topics are arriving at each peer.

</details>

<details>
<summary>Example 2: Native Chatter Topic (external containers)</summary>

This example bridges a native ROS 2 topic (`/chatter`) from `machine_b` to `machine_a`.
The ROS “application logic” runs in separate containers (to emulate an existing, unchanged ROS setup),
while the communication session runs via `rosotacom`.

- On `machine_a`, start the “logic” container and the communication session in separate terminals:
  ```bash
  cd example/2_native_chatter/machine_a
  ./run_external.py
  ```
  ```bash
  cd example/2_native_chatter/machine_a
  ./run_communication.sh
  ```
- On `machine_b`, start the “logic” container and the communication session in separate terminals:
  ```bash
  cd example/2_native_chatter/machine_b
  ./run_external.py
  ```
  ```bash
  cd example/2_native_chatter/machine_b
  ./run_communication.sh
  ```
- Confirm that `machine_a` receives and echoes messages from `machine_b` on `/chatter`.

</details>

<details>
<summary>Example 3: Compressed Occupancy Grid (CycloneDDS)</summary>

This example transfers a larger topic (an occupancy grid on `/costmap/costmap`) from `machine_b` to `machine_a`,
including processing steps like restamping and compression/decompression.

- On `machine_a`, start the “logic” container and the communication session in separate terminals:
  ```bash
  cd example/3_comp_occ_grid/machine_a
  ./run_external.py
  ```
  ```bash
  cd example/3_comp_occ_grid/machine_a
  ./run_communication.sh
  ```
- On `machine_b`, start the bag playback (“logic”) and the communication session in separate terminals:
  ```bash
  cd example/3_comp_occ_grid/machine_b
  ./run_external.py
  ```
  ```bash
  cd example/3_comp_occ_grid/machine_b
  ./run_communication.sh
  ```
- Confirm that `machine_a` receives the data stream and that the processed topics are visible (e.g. the restamped output).

</details>

<details>
<summary>Example 4: Compressed Occupancy Grid (Zenoh)</summary>

This is the same scenario as Example 3, but routed via a Zenoh router layer (useful when peers cannot share a single DDS domain).
The session config enables Zenoh with peer `a` as the main/router node.

- On `machine_a`, start the “logic” container and the communication session in separate terminals:
  ```bash
  cd example/4_comp_occ_grid_zen/machine_a
  ./run_external.py
  ```
  ```bash
  cd example/4_comp_occ_grid_zen/machine_a
  ./run_communication.sh
  ```
- On `machine_b`, start the bag playback (“logic”) and the communication session in separate terminals:
  ```bash
  cd example/4_comp_occ_grid_zen/machine_b
  ./run_external.py
  ```
  ```bash
  cd example/4_comp_occ_grid_zen/machine_b
  ./run_communication.sh
  ```
- Confirm that `machine_a` receives the data stream (now via Zenoh), including the heartbeat and occupancy grid topics.

</details>


## Choosing the Transport Layer: CycloneDDS or Zenoh

- **Use CycloneDDS** when all machines share the **same `ROS_DOMAIN_ID`**.  
  This is the simplest and most direct configuration.

- **Use Zenoh** when machines **require different domain IDs**.  
  Zenoh naturally bridges DDS domains and avoids additional ROS 2 domain-bridging complexity.

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