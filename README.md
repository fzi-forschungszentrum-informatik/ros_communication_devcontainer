# ROS Communication DevContainer

The ROS Communication DevContainer is a Docker-based solution designed to streamline the synchronization of ROS2 topics across Linux machines. It provides built-in compression and routing capabilities for over-the-air (OTA) data transfer: selected topics are remapped into an OTA namespace and transmitted either via direct DDS (CycloneDDS) or through a Zenoh router. This project aligns with the publication *“Scalable Remote Operation for Autonomous Vehicles: Integration of Cooperative Perception and Open Source Communication.”*

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

### Basic Setup

1. Fork this repository
2. Configure your environment:
   - `data_dict.json`: Maps semantic names to resources (example in `data_dict.json.examples`)
   - `local.json`: Container-specific settings (example in `local.json.examples`)

### Configuration Files

Two main configuration types:

1. **Session Configuration**:
   - `session_specification.yaml`: Defines communication session structure
   - `[plugin_name].yaml`: Individual plugin configurations

2. **Compression Configuration** (Optional):
   - Configure compression settings for specific topics
   - Support for multiple compression algorithms

## Usage Examples

### Starting Points: Examples

To help you get started, we have provided several examples that showcase the basic functionality of the ROS Communication DevContainer. These examples are designed for ease of use and quick setup.

<details>
<summary>Configuring Machine Data</summary>

The examples require two machines, which we will refer to as `machine_a` and `machine_b`. Choose your machines and fill out the `machine_a_ip` and `machine_b_ip` fields in your `data_dict.json`. Make sure this data is synchronized across both machines via Git to ensure seamless communication.

</details>

<details>
<summary>Example 1: Hello World</summary>

This example checks if the ROS Communication DevContainer can be started successfully.

- On any machine, run the script:
  ```bash
  ./example/1_hello_world_session/run.sh
  ```
- Confirm that a hello world statement gets printed.

</details>

<details>
<summary>Example 2: Multi-Machine Direct Communication</summary>

This example demonstrates ROS2 communication between distinct machines using direct communication.

- On `machine_a`, execute:
  ```bash
  ./example/2_multi_machine_direct/run_machine_a.sh
  ```
- On `machine_b`, execute:
  ```bash
  ./example/2_multi_machine_direct/run_machine_b.sh
  ```
- Confirm that the listener on `machine_a` acknowledges the messages from the talker on `machine_b`.

</details>

<details>
<summary>Example 3: Multi-Machine Relay Communication</summary>

This example demonstrates ROS2 communication between distinct machines using relay nodes.

- On `machine_a`, execute:
  ```bash
  ./example/3_multi_machine_relay/run_machine_a.sh
  ```
- On `machine_b`, execute:
  ```bash
  ./example/3_multi_machine_relay/run_machine_b.sh
  ```
- Confirm that the listener on `machine_a` acknowledges the messages from the talker on `machine_b`.

</details>

<details>
<summary>Example 4: External Container</summary>

This example adds a layer of separation between the main logic and communication, allowing existing local ROS1 code to remain unchanged.

- On `machine_a`, navigate to `example/4_external_container/machine_a` and execute the following commands in separate terminals:
  - `./run_external.py`
  - `./run_communication.sh`
- On `machine_b`, navigate to `example/4_external_container/machine_b` and execute the following commands in separate terminals:
  - `./run_external.py`
  - `./run_communication.sh`
- Confirm that the master on `machine_a` acknowledges the messages from the master on `machine_b`.

</details>

<details>
<summary>Example 5: Showcase</summary>

This example demonstrates handling more complex data, such as an occupancy grid map, which is larger and therefore uses compression.

- On `machine_a`, navigate to `example/5_showcase/machine_a` and execute the following commands in separate terminals:
  - `./run_external.py`
  - `./run_communication.sh`
- On `machine_b`, navigate to `example/5_showcase/machine_b` and execute the following commands in separate terminals:
  - `./run_external.py`
  - `./run_communication.sh`
- Confirm that the master on `machine_a` acknowledges the compressed messages from the master on `machine_b`.

The showcase example includes:
- Automatic topic compression and decompression
- QoS configuration for optimized communication
- Heartbeat monitoring between machines

</details>

## Advanced Features

### Compression Support

The system includes universal compression capabilities for ROS topics:

```yaml
compression:
  - topic_regex: ".*"
    algorithm: "bz2"  # Optional
    add_suffix: "_compressed"  # Optional
```

## Position in the OTA Communication Landscape

This repository fits into a broader set of ROS-based OTA communication approaches:

- **Direct ROS 2 DDS Communication**  
  Native DDS (CycloneDDS, Fast DDS), often with custom configuration for constrained or long-range links.  
  The examples in this repository use CycloneDDS to illustrate this approach.

- **ROS 2 over Router-like Backbones**  
  Some RMW have their own DDS Routers such as https://github.com/eProsima/DDS-Router
  Example 5 has a flag to use Zenoh + `ros2dds` acts as a lightweight router layer.

- **MQTT-based Approaches**  
  Common in cloud/IoT scenarios. Example:  
  <https://github.com/ika-rwth-aachen/mqtt_client>

- **Custom TCP/UDP Teleoperation Stacks**  
  Some frameworks implement their manual tcp/udp transportion layers. Example:  
  <https://github.com/TUMFTM/teleoperated_driving>

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