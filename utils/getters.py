#!/usr/bin/env python3

import os
import sys
import json

project_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(project_dir)

from utils.files import *

def get_local_config():
    return json_to_dict(f"{project_dir}/config/local.json")

def get_static_docker_run_args():
    local_config = get_local_config()

    template_file_path = f"{project_dir}/config/static_docker_run_args.json.template"

    substitutions = {
        "#uid": str(os.getuid()),
        "#gid": str(os.getgid()),
        "#container_name": local_config['container_name'],
        "#project_dir": project_dir,
        "#data_dict": local_config['path_to_data_dict'],
        "#network": local_config['network']
    }

    processed_content = process_template(template_file_path, substitutions)
    content_dict = json.loads(processed_content)
    static_docker_run_args = content_dict["static_docker_run_args"]
    return static_docker_run_args

def get_local_mount_args():
    local_config = get_local_config()
    mount_args = []

    ros_ws_mount = local_config.get("ros_ws_mount")
    if ros_ws_mount:
        mount_args.append("-v")
        mount_args.append(f"{ros_ws_mount}:/home/myuser/catkin_ws/src")

    custom_mounts = local_config.get("custom_mounts")
    if custom_mounts:
        for mount in custom_mounts:
            mount_args.append("-v")
            mount_args.append(f"{mount['host_path']}:{mount['container_path']}")

    return mount_args

def get_optional_docker_run_args():
    optional_docker_run_args = []

    if get_local_config().get("enable_gui_forwarding"):
        optional_docker_run_args.extend([
            "-v", "/tmp/.X11-unix:/tmp/.X11-unix",
            "-e", "DISPLAY",
        ])

    if get_local_config().get("forward_ssh_agent"):
        ssh_auth_sock = os.getenv('SSH_AUTH_SOCK')
        optional_docker_run_args.extend([
            "-e", "SSH_AUTH_SOCK",
            "-v", f"{ssh_auth_sock}:{ssh_auth_sock}"
        ])
    return optional_docker_run_args

def get_docker_run_args():
    return get_static_docker_run_args() + get_local_mount_args() + get_optional_docker_run_args()

def get_image_name():
    return get_local_config()["image_name"]

def get_container_name():
    return get_local_config()["container_name"]