# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from setuptools import find_packages, setup

package_name = 'com_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE.txt', 'LICENSES/Apache-2.0.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='ROS 2 compression (bz2, zlib, lz4), per-message instrumentation and relay utilities.',
    maintainer='Martin Gontscharow',
    maintainer_email='gontscharow@fzi.de',
    license='BSD-3-Clause AND Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_bandwidth_reducer = com_py.image_bandwidth_reducer:main',
            'image_pixel_capper = com_py.image_pixel_capper:main',
            'universal_compressor = com_py.universal_compressor:main',
            'universal_decompressor = com_py.universal_decompressor:main',
            'universal_ota_unwrapper = com_py.universal_ota_unwrapper:main',
            'universal_ota_wrapper = com_py.universal_ota_wrapper:main',
            'heartbeat_echo = com_py.heartbeat_echo:main',
            'topic_monitor = com_py.topic_monitor:main',
            'status_overview = com_py.status_overview:main',
            'bridge_in = com_py.bridge_in:main',
            'bridge_out = com_py.bridge_out:main',
            'relay_in = com_py.relay_in:main',
            'relay_out = com_py.relay_out:main',
            'hz_monitor = com_py.hz_monitor:main',
            'lanelet_saver = com_py.lanelet_saver:main',
            'localization_map_split_reduce_saver = com_py.localization_map_split_reduce_saver:main',
            'tf_map_to_cart_saver = com_py.tf_map_to_cart_saver:main',
            'local_global_frame_bridge = com_py.local_global_frame_bridge:main',
            'restamp = com_py.restamp:main',
            'latch_relay = com_py.latch_relay:main',
            'trickle = com_py.trickle:main',
            'playout_pacer = com_py.playout_pacer:main',
            'sized_publisher = com_py.sized_publisher:main',
            'stage_latency = com_py.stage_latency:main',
        ],
    },
)
