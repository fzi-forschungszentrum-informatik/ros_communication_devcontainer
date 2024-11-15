# ROS Communication DevContainer

The ROS Communication DevContainer is a Docker-based solution designed to streamline the synchronization of ROS topics across Linux machines. It's crafted to support scalable communications, facilitating easier remote operations for autonomous vehicles. This project aligns with the publication "Scalable Remote Operation for Autonomous Vehicles: Integration of Cooperative Perception and Open Source Communication."

<details>
<summary>Key Features</summary>

- **Minimal Dependencies**: Only Docker is needed to get started, simplifying the setup process.
- **Isolation**: Operates in a separate Docker container, ensuring minimal impact on existing ROS setups.
- **Centralized Configuration Management**: All configurations are stored and managed in this repository, allowing for easy synchronization across machines via Git.

</details>

## Getting Started

### Forking the Repository

To tailor the ROS Communication DevContainer to your specific needs, we recommend forking this repository. This allows you to make system-specific adjustments and keep your changes organized without affecting the original template.

### Basic Setup for Connected Machines

To prepare your machines for communication, follow these steps on each device:

<details>
<summary>Setup Machines</summary>

- Ensure all machines are connected to the same network. This could be an internet connection with VPN, or a local WLAN network.
- Install Docker: Required for running the communication module.

</details>

<details>
<summary>Configuration Adjustments</summary>

To set up the ROS Communication DevContainer, you need to adjust two main configuration files:

1. **`data_dict.json`**:
   - This file is a dictionary that maps semantic names to your data, such as IP addresses. It enables the ROS Communication DevContainer to recognize and reference your resources easily.
   - An example configuration file is provided in `config/examples/data_dict.json`.
   - You can either use the example configuration directly or create your own. If you create a new one, specify its path in `config/local.json`.

2. **`local.json`**:
   - This file contains repository-specific settings, such as container and image names, and optionally the paths to your configuration files.
   - An example of this file is available in `config/examples/local.json`.
   - You should create this file (or copy the example) and place it in `config/local.json`.

**Note**: Ensure that these configuration files are synchronized across all machines using Git to facilitate seamless communication.

</details>

## Demonstration and Usage

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
<summary>Example 2: Single Machine Multimaster</summary>

This example verifies if ROS communication works for multiple masters on a single machine.

- Open two terminals on the same machine.
- In the first terminal, execute:
  ```bash
  ./example/2_single_machine_multimaster/run_listener.sh
- In the second terminal, execute:
  ```bash
  ./example/2_single_machine_multimaster/run_talker.sh
- Confirm that the listener prints an acknowledgment that it has received messages from the talker.

</details>

<details>
<summary>Example 3: Multi-Machine Communication</summary>

This example tests ROS communication between distinct machines (and distinct masters).

- On `machine_a`, execute:
  ```bash
  ./example/3_multi_machine/run_machine_a.sh
- On `machine_b`, execute:
  ```bash
  ./example/3_multi_machine/run_machine_b.sh
- Confirm that the listener on `machine_a` acknowledges the messages sent by the talker on `machine_b`.

</details>

<details>
<summary>Example 4: External Master</summary>

This example adds a layer of separation between the main logic and communication, allowing existing local code to remain unchanged.

- On `machine_a`, navigate to `example/4_external_master/machine_a` and execute the following commands in separate terminals:
  - `./run_master.py`
  - `./run_communication.sh`
- On `machine_b`, navigate to `example/4_external_master/machine_b` and execute the following commands in separate terminals:
  - `./run_master.py`
  - `./run_communication.sh`
- Confirm that the master on `machine_a` acknowledges the messages from the master on `machine_b`.

</details>

<details>
<summary>Example 5: Showcase</summary>

This example demonstrates handling more complex data, such as an occupancy grid map, which is larger and therefore uses compression.

- On `machine_a`, navigate to `example/5_showcase/machine_a` and execute the following commands in separate terminals:
  - `./run_master.py`
  - `./run_communication.sh`
- On `machine_b`, navigate to `example/5_showcase/machine_b` and execute the following commands in separate terminals:
  - `./run_master.py`
  - `./run_communication.sh`
- Confirm that the master on `machine_a` acknowledges the messages from the master on `machine_b`.

</details>

### Expanding Communication Capabilities
The ROS Communication DevContainer is designed to be flexible, allowing you to expand and tailor its communication routines to meet specific project requirements. All routines are defined using two key file types:

- **`session_specification.yaml`**:
  - This file outlines the structure of your communication session by listing the plugins to be used.
  - It acts as a blueprint for combining plugins into a cohesive communication workflow.

- **`[plugin_name].yaml`**:
  - This file defines an individual plugin, which specifies a catmux session.
  - Plugins can be modular and reusable across different `session_specification.yaml` files, enabling easy customization and scalability.

To initiate a communication session, use the following command:
```bash
run_session_in_container.py --session-dir [/path/to/your/session_dir]
```
Here, `session_dir` refers to the directory containing your `session_specification.yaml` file. Customize your plugins and sessions by exploring the examples in `ws/example`, which provide a foundation for creating complex communication setups tailored to your needs.

By leveraging this modular approach, you can build advanced, scalable, and reusable communication routines for your project.

**Note**: Ensure that any changes you make are synchronized across all machines via Git to maintain consistent configurations and communication capabilities.

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