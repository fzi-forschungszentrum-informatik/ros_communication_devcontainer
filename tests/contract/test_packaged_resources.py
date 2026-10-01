# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import argparse
import os
import re
import subprocess
from importlib import resources
from pathlib import Path

import pytest
import yaml

import rosotacom.cli as cli

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
LYRICAL_EXAMPLE_CONFIG = cli.EXAMPLE_PROJECT_DIR / "ros2docker.lyrical.json"
PACKAGED_COMMUNICATION_CONFIGS = (
    cli.DEFAULT_ROS2DOCKER_CONFIG,
    cli.LYRICAL_ROS2DOCKER_CONFIG,
    cli.EXAMPLE_PROJECT_DIR / "ros2docker.json",
    LYRICAL_EXAMPLE_CONFIG,
)


def test_packaged_resources_include_curated_examples_and_runtime_workspace() -> None:
    package = resources.files("rosotacom")

    expected = (
        "py.typed",
        "resources/ros2docker.json.example",
        "resources/ros2docker.lyrical.json.example",
        "resources/examples/rosotacom.yaml",
        "resources/examples/.gitignore",
        "resources/examples/ros2docker.json",
        "resources/examples/ros2docker.lyrical.json",
        "resources/examples/profiles.yaml",
        "resources/examples/deployment.example.yaml",
        "resources/examples/bags/8_drop_reference/metadata.yaml",
        "resources/examples/sessions/1_heartbeat/session-definition.yaml",
        "resources/examples/sessions/1_heartbeat_status/session-definition.yaml",
        "resources/examples/sessions/14_remote_assist_anonymized/session-definition.yaml",
        "resources/examples/sessions/15_remote_assist_anonymized_costmap/session-definition.yaml",
        "resources/examples/sessions/16_remote_assist_anonymized_camera/session-definition.yaml",
        "resources/examples/sessions/17_synthetic_camera_quality/session-definition.yaml",
        "resources/examples/scenarios/2_native_chatter/scenario-definition.yaml",
        "resources/examples/scripts/2_native_chatter/machine_a/external.lyrical.ros2docker.json",
        "resources/examples/scripts/2_native_chatter/machine_b/external.lyrical.ros2docker.json",
        "resources/ws/session/creation/run_session.py",
        "resources/ws/session/creation/catmux_log_setup.sh",
        "resources/ws/session/creation/strip_ansi.py",
        "resources/ws/ota_configs/cyclonedds_tuned.xml.template",
        "resources/ws/ota_configs/cyclonedds_local_participants.xml.template",
        "resources/ws/ota_configs/fastdds_unicast.xml.template",
        "resources/ws/ota_configs/fastdds_tuned.xml.template",
        "resources/ws/ota_configs/fastdds_easy_mode.xml.template",
        "resources/ws/ros2src/com_msgs/msg/EchoHeartbeat.msg",
    )

    missing = [path for path in expected if not package.joinpath(path).is_file()]
    assert not missing

    ota_stamped = package.joinpath("resources/ws/ros2src/com_msgs/msg/OtaStamped.msg").read_text(encoding="utf-8")
    echo = package.joinpath("resources/ws/ros2src/com_msgs/msg/EchoHeartbeat.msg").read_text(encoding="utf-8")
    assert "source_stamp" not in ota_stamped
    assert "uint64 seq" in ota_stamped
    assert "builtin_interfaces/Time echo_t1" in echo
    assert "builtin_interfaces/Time echo_t2" in echo
    assert "builtin_interfaces/Time echo_t3" in echo


def test_packaged_example_profiles_parse() -> None:
    # The shipped illustrative profiles must actually load — a malformed example
    # (bad tc/netem combo, unknown key) would be a packaging regression.
    from rosotacom.network_profiles import load_profiles_file

    path = resources.files("rosotacom").joinpath("resources/examples/profiles.yaml")
    profiles = load_profiles_file(Path(str(path)))
    assert {"cellular-typical", "cellular-handover"} <= set(profiles)
    assert profiles["cellular-handover"].is_timeline


def test_cli_resource_constants_point_inside_package_resources() -> None:
    assert cli.RESOURCE_DIR == PACKAGE_ROOT / "src" / "rosotacom" / "resources"
    assert cli.WS_DIR == cli.RESOURCE_DIR / "ws"
    assert cli.DEFAULT_ROS2DOCKER_CONFIG == cli.RESOURCE_DIR / "ros2docker.json.example"
    assert cli.LYRICAL_ROS2DOCKER_CONFIG == cli.RESOURCE_DIR / "ros2docker.lyrical.json.example"
    assert cli.EXAMPLE_PROJECT_DIR == cli.RESOURCE_DIR / "examples"


def test_default_ros2docker_config_pins_supported_kilted_noble_image() -> None:
    default_config = cli.load_config(cli.DEFAULT_ROS2DOCKER_CONFIG)
    example_config = cli.load_config(cli.EXAMPLE_PROJECT_DIR / "ros2docker.json")

    assert default_config.get("profile") is None
    assert not default_config.get("profiles")
    assert example_config.get("profile") is None
    assert not example_config.get("profiles")

    default_build_args = default_config["build_args"]
    example_build_args = example_config["build_args"]

    # ros-base rather than desktop-full: nothing rosotacom ships runs a GUI, and
    # the difference is 299 MB compressed against 1422 MB — bytes every peer
    # transfers on a cold build and every e2e slice pulls from the published
    # image.
    assert default_build_args["BASE_IMAGE"] == "ros:kilted-ros-base-noble"
    assert default_build_args["DIGEST"].startswith("@sha256:")
    assert default_build_args["BASE_IMAGE"] == example_build_args["BASE_IMAGE"]
    assert default_build_args["DIGEST"] == example_build_args["DIGEST"]
    for build_args in (default_build_args, example_build_args):
        assert build_args["INSTALL_ZENOH"] == "1"
        apt_packages = set(str(build_args["APT_PACKAGES"]).split())
        assert "ros-kilted-domain-bridge" in apt_packages
        assert "ros-kilted-rmw-cyclonedds-cpp" in apt_packages
        assert "ros-kilted-rmw-zenoh-cpp" in apt_packages
        # The ffmpeg encoder plugin (image_transport/ffmpeg_pub) is what the
        # synthetic-camera-quality session needs to produce /camera/image/ffmpeg;
        # the -msgs package alone only provides FFMPEGPacket definitions.
        assert "ros-kilted-ffmpeg-image-transport" in apt_packages
        assert "ros-kilted-ffmpeg-image-transport-msgs" in apt_packages
        # gps_msgs/msg/GPSFix, published by the anonymized remote-assist
        # session. desktop-full carried it, so nothing declared it; on ros-base
        # its absence is a missing type at runtime, not a build error.
        assert "ros-kilted-gps-msgs" in apt_packages


def test_opt_in_lyrical_config_pins_resolute_and_source_domain_bridge() -> None:
    root_config = cli.load_config(cli.LYRICAL_ROS2DOCKER_CONFIG)
    example_config = cli.load_config(LYRICAL_EXAMPLE_CONFIG)

    for config in (root_config, example_config):
        assert config["image_name"] == "ros-communication-lyrical"
        assert config["profile"] == "domain-bridge"
        build_args = config["build_args"]
        assert build_args["BASE_IMAGE"] == "ros:lyrical-ros-base-resolute"
        assert build_args["DIGEST"].startswith("@sha256:")
        assert build_args["INSTALL_DOMAIN_BRIDGE"] == "1"
        assert build_args["INSTALL_ZENOH"] == "1"

        apt_packages = set(str(build_args["APT_PACKAGES"]).split())
        assert "ros-lyrical-domain-bridge" not in apt_packages
        for package in (
            "ros-lyrical-rmw-cyclonedds-cpp",
            "ros-lyrical-rmw-zenoh-cpp",
            "ros-lyrical-topic-tools",
            "ros-lyrical-image-transport",
            "ros-lyrical-compressed-image-transport",
            "ros-lyrical-ffmpeg-image-transport",
            "ros-lyrical-ffmpeg-image-transport-msgs",
            "ros-lyrical-gps-msgs",
        ):
            assert package in apt_packages

        pip_packages = str(build_args["PIP_PACKAGES"]).split()
        assert "numpy>=2,<3" in pip_packages

    assert root_config["build_args"] == example_config["build_args"]


def test_packaged_example_keeps_kilted_as_the_default_variant() -> None:
    project = yaml.safe_load((cli.EXAMPLE_PROJECT_DIR / "rosotacom.yaml").read_text(encoding="utf-8"))

    assert project["ros2docker_config"] == "ros2docker.json"
    assert cli.load_config(cli.EXAMPLE_PROJECT_DIR / project["ros2docker_config"])["build_args"][
        "BASE_IMAGE"
    ].startswith("ros:kilted-")


# `image_transport republish` resolves a transport name to a pluginlib class at
# runtime, so a transport nobody installed is not a build failure — it is a
# container that starts fine and aborts on the first image message. `raw` ships
# with image_transport itself; every other transport needs its own package.
IMAGE_TRANSPORT_APT_SUFFIXES = {
    "raw": "image-transport",
    "compressed": "compressed-image-transport",
    "ffmpeg": "ffmpeg-image-transport",
}


def _packaged_session_plugin() -> dict:
    return yaml.safe_load(
        (cli.WS_DIR / "session" / "content" / "base" / "session_plugin_base.yaml").read_text(encoding="utf-8")
    )


def test_every_image_transport_the_session_plugin_can_select_is_installed() -> None:
    """The packaged image must carry every transport the packaged session can ask for.

    This is the guard that was missing when the base image moved from
    desktop-full to ros-base: compressed_image_transport had arrived with
    desktop-full's perception variant, nothing in this repository declared it,
    and no e2e session has a /compressed input topic — so the suite stayed green
    while a vehicle camera link could no longer start. Deriving the transport set
    from the plugin rather than restating it means a new transport, or a thinner
    base image, fails here instead of on a vehicle.
    """
    plugin = _packaged_session_plugin()

    # Incoming republish: the transport the receiving peer decodes back to.
    selected = {
        str(value)
        for key, value in plugin["parameters"].items()
        if key.startswith("irt_") and key.endswith("_out_transport") and value
    }
    # Outgoing republish: the in-transport is chosen from the source topic name.
    it_window = next(window for window in plugin["windows"] if window["name"] == "IT")
    for split in it_window["splits"]:
        for command in split["commands"]:
            selected.update(re.findall(r'it_in_transport="([a-z]+)"', command))

    # Both branches of that choice, so a rewrite that drops one is visible here.
    assert {"raw", "compressed"} <= selected

    unknown = selected - set(IMAGE_TRANSPORT_APT_SUFFIXES)
    assert not unknown, f"no apt package recorded for image transport(s): {sorted(unknown)}"

    for config_path in PACKAGED_COMMUNICATION_CONFIGS:
        config = cli.load_config(config_path)
        base_image = str(config["build_args"]["BASE_IMAGE"])
        distro = base_image.split(":", 1)[1].split("-", 1)[0]
        required = {f"ros-{distro}-{IMAGE_TRANSPORT_APT_SUFFIXES[transport]}" for transport in selected}
        apt_packages = set(str(config["build_args"]["APT_PACKAGES"]).split())
        missing = required - apt_packages
        assert not missing, f"{config_path.name} does not install {sorted(missing)}"


def test_packaged_configs_do_not_fetch_rosdep_on_container_start() -> None:
    """Readiness must cover the workspace build, not a third-party index fetch.

    The packaged workspace dependencies come from the pinned base image and
    explicit image package lists, then Docker smoke exercises the real build.
    Enabling ros2docker's runtime check would run ``rosdep update`` for every
    communication container before that small build and make readiness depend
    on raw.githubusercontent.com.
    """
    for config_path in PACKAGED_COMMUNICATION_CONFIGS:
        config = cli.load_config(config_path)
        run_args = [str(value) for value in config["run_args"]]

        assert "BUILD_ROS2WS=1" in run_args
        assert not any(arg.startswith("CHECK_ROS2WS_DEPENDENCIES=") for arg in run_args)


def test_external_ros2docker_configs_install_selected_rmw_implementations() -> None:
    configs = sorted(cli.EXAMPLE_PROJECT_DIR.glob("scripts/**/external*.ros2docker.json"))
    assert configs

    offenders: list[str] = []
    for config_path in configs:
        config = cli.load_config(config_path, resolve_run_args=False)
        run_args = [str(value) for value in config.get("run_args", []) or []]
        if "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" not in run_args:
            continue

        build_args = config.get("build_args", {}) or {}
        apt_packages = set(str(build_args.get("APT_PACKAGES", "")).split())
        base_image = str(build_args.get("BASE_IMAGE", ""))
        match = re.search(r"ros:([a-z][a-z0-9_]*)-", base_image)
        expected_package = f"ros-{match.group(1)}-rmw-cyclonedds-cpp" if match else None
        if expected_package is None or expected_package not in apt_packages:
            rel = config_path.relative_to(cli.EXAMPLE_PROJECT_DIR)
            offenders.append(f"{rel} selects rmw_cyclonedds_cpp but does not install its distro-matched package")

    assert not offenders


def test_shell_entrypoints_pass_syntax_check() -> None:
    scripts = sorted(cli.EXAMPLE_PROJECT_DIR.glob("scripts/**/*.sh"))
    assert scripts
    for script in scripts:
        subprocess.run(["bash", "-n", str(script)], check=True)

    subprocess.run(["bash", "-n", str(cli.WS_DIR / "session" / "creation" / "catmux_log_setup.sh")], check=True)
    subprocess.run(["bash", "-n", str(PACKAGE_ROOT / "install.sh")], check=True)
    subprocess.run(
        ["python3", "-m", "py_compile", str(cli.WS_DIR / "session" / "creation" / "strip_ansi.py")],
        check=True,
    )


def test_all_owned_configs_parse_and_validate() -> None:
    from rosotacom.cli import load_config

    # 1. ros2docker.json.example
    load_config(cli.DEFAULT_ROS2DOCKER_CONFIG)

    # 1b. opt-in Lyrical configs
    load_config(cli.LYRICAL_ROS2DOCKER_CONFIG)
    load_config(LYRICAL_EXAMPLE_CONFIG)

    # 2. rosotacom.yaml example
    yaml.safe_load(cli.EXAMPLE_PROJECT_DIR.joinpath("rosotacom.yaml").read_text(encoding="utf-8"))

    # 3. profiles.yaml
    # Tested by test_packaged_example_profiles_parse

    # 4. deployment.example.yaml
    yaml.safe_load(cli.EXAMPLE_PROJECT_DIR.joinpath("deployment.example.yaml").read_text(encoding="utf-8"))

    # 5. ros2docker.json
    load_config(cli.EXAMPLE_PROJECT_DIR / "ros2docker.json")

    # 6. session and scenario definitions
    session_files = sorted(cli.EXAMPLE_PROJECT_DIR.glob("sessions/*/session-definition.yaml"))
    assert session_files
    for session_file in session_files:
        yaml.safe_load(session_file.read_text(encoding="utf-8"))

    scenario_files = sorted(cli.EXAMPLE_PROJECT_DIR.glob("scenarios/*/scenario-definition.yaml"))
    assert scenario_files
    for scenario_file in scenario_files:
        yaml.safe_load(scenario_file.read_text(encoding="utf-8"))


def test_base_plugin_catmux_commands_are_strings() -> None:
    plugin = yaml.safe_load(
        (cli.WS_DIR / "session" / "content" / "base" / "session_plugin_base.yaml").read_text(encoding="utf-8")
    )

    offenders: list[str] = []
    for window in plugin["windows"]:
        for split_index, split in enumerate(window.get("splits", [])):
            for command_index, command in enumerate(split.get("commands", [])):
                if not isinstance(command, str):
                    offenders.append(
                        f"{window.get('name', '<unnamed>')}.splits[{split_index}].commands[{command_index}]"
                    )

    assert not offenders


def test_domain_bridge_generates_ota_dds_config_before_launch() -> None:
    plugin = yaml.safe_load(
        (cli.WS_DIR / "session" / "content" / "base" / "session_plugin_base.yaml").read_text(encoding="utf-8")
    )
    com_window = next(window for window in plugin["windows"] if window["name"] == "COM")
    commands = com_window["splits"][0]["commands"]

    assert "/ws/ota_configs/get_ota_xml.py" in commands[0]
    assert commands[1].startswith("ros2 run domain_bridge domain_bridge")


def test_ota_bridge_static_peers_include_same_host_domain_bridge() -> None:
    plugin = yaml.safe_load(
        (cli.WS_DIR / "session" / "content" / "base" / "session_plugin_base.yaml").read_text(encoding="utf-8")
    )

    for window_name in ("IN", "OUT"):
        window = next(window for window in plugin["windows"] if window["name"] == window_name)
        bridge_commands = window["splits"][0]["commands"]

        assert 'export ROS_STATIC_PEERS="${ip_local};${ip_remote}"' in bridge_commands[3]


def test_native_chatter_waiter_reads_topic_info_without_broken_pipe(tmp_path: Path) -> None:
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    fake_ros2 = fake_bin / "ros2"
    fake_ros2.write_text(
        "\n".join(
            [
                "#!/usr/bin/env bash",
                'if [[ "$1 $2" == "topic info" ]]; then',
                '  for i in {1..100}; do echo "detail $i"; done',
                '  echo "Type: std_msgs/msg/String"',
                "  exit 0",
                "fi",
                'if [[ "$1 $2" == "topic echo" ]]; then',
                '  echo "echo started"',
                "  exit 0",
                "fi",
                "exit 2",
                "",
            ]
        ),
        encoding="utf-8",
    )
    fake_ros2.chmod(0o755)
    script = cli.EXAMPLE_PROJECT_DIR / "scripts" / "2_native_chatter" / "machine_a" / "wait_for_echo_topic.sh"

    result = subprocess.run(
        [str(script)],
        text=True,
        capture_output=True,
        check=True,
        env={**os.environ, "PATH": f"{fake_bin}:{os.environ['PATH']}"},
    )

    assert result.stdout.strip() == "echo started"
    assert "BrokenPipeError" not in result.stderr


def test_every_registered_subcommand_is_exempt_from_implicit_start(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # `rosotacom <session>` is shorthand for `rosotacom start <session>`, so
    # main() only skips the implicit "start" for names in TOP_LEVEL_COMMANDS.
    # A subparser missing from that set is unreachable: the CLI silently turns
    # `rosotacom foo ...` into `rosotacom start foo ...` and fails somewhere
    # unrelated. Capture what is actually registered and compare.
    captured: dict[str, argparse._SubParsersAction] = {}
    original = argparse.ArgumentParser.add_subparsers

    def spy(self: argparse.ArgumentParser, *args: object, **kwargs: object):
        action = original(self, *args, **kwargs)
        if self.prog == "rosotacom":
            captured["top"] = action
        return action

    monkeypatch.setattr(argparse.ArgumentParser, "add_subparsers", spy)
    with pytest.raises(SystemExit):
        cli.main(["--help"])

    registered = set(captured["top"].choices)
    assert registered, "no top-level subparsers were captured"

    unreachable = registered - cli.TOP_LEVEL_COMMANDS
    assert not unreachable, f"subcommands missing from TOP_LEVEL_COMMANDS: {sorted(unreachable)}"

    # The reverse direction may legitimately carry retired names (e.g. "verify"),
    # which stay guarded so they are not rewritten as `start verify`.
    assert "resources" in registered


def test_named_resources_point_at_real_packaged_paths() -> None:
    for name, path in cli.NAMED_RESOURCES.items():
        assert path.exists(), f"packaged resource {name!r} does not exist: {path}"

    assert cli.NAMED_RESOURCES["ros2docker-lyrical"].is_file()

    # com_msgs is consumed by external builds (fleet_mgmt bakes it into its
    # container images), so it must be a usable ROS 2 package, not just a folder.
    com_msgs = cli.NAMED_RESOURCES["com_msgs"]
    assert (com_msgs / "package.xml").is_file()
    assert (com_msgs / "CMakeLists.txt").is_file()
    assert list(com_msgs.glob("msg/*.msg"))


def _tracked_entries() -> list[tuple[str, str, str]]:
    """``(mode, sha, path)`` for every tracked file, or skip outside a checkout."""
    listing = subprocess.run(
        ["git", "ls-files", "-s", "-z"],
        cwd=PACKAGE_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    if listing.returncode != 0:
        pytest.skip("not a git checkout — nothing to say about tracked files")
    entries = []
    for record in listing.stdout.split("\0"):
        if not record:
            continue
        meta, _, path = record.partition("\t")
        mode, sha, _stage = meta.split()
        entries.append((mode, sha, path))
    return entries


def test_no_tracked_symlink_escapes_the_repository() -> None:
    """A tracked symlink is checked out over whatever already sits at that path.

    One pointing outside the tree can overwrite any path on any machine that
    has it, and setuptools_scm puts every tracked file into the sdist, so it
    ships too.
    """
    escaping: list[str] = []
    for mode, sha, path in _tracked_entries():
        if mode != "120000":
            continue
        # A symlink's blob *is* its target, so this reads the same bytes a
        # checkout would write — no dependence on the working tree.
        target = subprocess.run(
            ["git", "cat-file", "blob", sha],
            cwd=PACKAGE_ROOT,
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
        resolved = Path(os.path.normpath((PACKAGE_ROOT / path).parent / target))
        if os.path.isabs(target) or not resolved.is_relative_to(PACKAGE_ROOT):
            escaping.append(f"{path} -> {target}")

    assert not escaping, (
        f"tracked symlinks point outside the repository: {escaping}. Checking one out overwrites "
        "whatever is at that path, and it is shipped in the sdist."
    )


def test_dds_profiles_are_rendered_atomically_and_never_left_empty() -> None:
    """A profile that fails to render must not leave a usable-looking file.

    `get_ota_xml.py ... > file` truncates the target before the renderer runs, so
    a missing template leaves a zero-byte profile behind — and a zero-byte
    profile is not an error to either DDS stack. Fast DDS logs a parse error and
    falls back to multicast discovery, which a tunnelled OTA link cannot carry,
    so the session comes up healthy-looking and delivers nothing. That is how
    eight runs on 2026-08-19 were recorded as "Fast DDS delivers 0 messages"
    when the middleware had simply never been configured.
    """
    plugin_path = cli.WS_DIR / "session" / "content" / "base" / "session_plugin_base.yaml"
    text = plugin_path.read_text(encoding="utf-8")

    # Not [^;]* — the failure branch itself contains a ';', and stopping there
    # would silently check only the half of the statement that cannot fail.
    calls = [text[m.start() : m.start() + 400] for m in re.finditer(r"get_ota_xml\.py", text)]
    assert calls, "no get_ota_xml.py call sites found — did the plugin move?"
    for call in calls:
        assert "--output" in call, f"renders through a shell redirect: {call[:120]}"
        assert not re.search(r">\s*\"\$\{\w*config_file\}\"", call), f"still truncates its target: {call[:120]}"
        assert "exit 1" in call, f"a failed render does not stop the start: {call[:120]}"


def test_every_default_ota_profile_template_is_packaged() -> None:
    """The default a session falls back to must exist in the installed package.

    #299 pointed the Fast DDS default at fastdds_tuned.xml while the stack
    deployed on the bench machines predated that template, so every Fast DDS run
    rendered nothing.
    """
    from rosotacom.resources.ws.session.creation import generate_session_files as gsf

    ota_configs = cli.WS_DIR / "ota_configs"
    defaults = list(gsf._DDS_OTA_DEFAULT_CONFIG.values()) + list(gsf._DDS_LOCAL_DEFAULT_CONFIG.values())
    missing = [name for name in defaults if name and not (ota_configs / f"{name}.template").exists()]
    assert not missing, f"default profile templates absent from the package: {missing}"


def test_benchmark_forwards_every_shared_ota_peer_option() -> None:
    """A hand-copied field list drops new options silently, and did.

    `cli_benchmark` builds an `argparse.Namespace` for the OTA layer by naming
    each field it forwards. `--peer-cpuset` was added to the shared OTA argument
    group, reached the parser, and was accepted on the command line — and then
    never arrived, because this list did not mention it. Every unit test passed
    and the feature was inert on the machine it was built for.

    So the list is checked against the parser rather than maintained by hand:
    whatever `_add_ota_install_args` defines has to appear here too.
    """
    import argparse
    import inspect

    import rosotacom.cli as cli
    import rosotacom.cli_benchmark as cli_benchmark

    parser = argparse.ArgumentParser()
    cli._add_ota_install_args(parser)
    peer_dests = {
        action.dest for action in parser._actions if action.dest.startswith("peer_") and action.dest != "help"
    }
    assert peer_dests, "the shared OTA argument group defines no --peer-* options"

    forwarded = inspect.getsource(cli_benchmark)
    missing = sorted(dest for dest in peer_dests if f"{dest}=getattr(args" not in forwarded)
    assert not missing, (
        f"cli_benchmark does not forward {', '.join(missing)} into the OTA namespace, "
        "so the option is accepted on the command line and then ignored"
    )
