from setuptools import find_packages, setup

package_name = 'com_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Python implementation of compression and monitoring utilities for ROS2 communication. Provides universal compression/decompression nodes with support for multiple algorithms (bz2, gzip, zlib, lz4, zstd), network monitoring capabilities, and various communication tools including topic frequency monitoring, bandwidth reduction for images, and bridge/relay functionality.',
    maintainer='Martin Gontscharow',
    maintainer_email='gontscharow@fzi.de',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_bandwidth_reducer = com_py.image_bandwidth_reducer:main',
            'wifi_network_analysis_1 = com_py.wifi_network_analysis_1:main',
            'wifi_network_analysis_2 = com_py.wifi_network_analysis_2:main',
            'speedtest = com_py.speedtest:main',
            'traffic_monitor = com_py.traffic_monitor:main',
            'universal_compressor = com_py.universal_compressor:main',
            'universal_decompressor = com_py.universal_decompressor:main',
            'heartbeat = com_py.heartbeat:main',
            'topic_monitor = com_py.topic_monitor:main',
            'bridge_in = com_py.bridge_in:main',
            'bridge_out = com_py.bridge_out:main',
            'relay_in = com_py.relay_in:main',
            'relay_out = com_py.relay_out:main',
            'hz_monitor = com_py.hz_monitor:main',
            'costmap_remapper = com_py.costmap_remapper:main',
            'tf_remapper = com_py.tf_remapper:main',
            'marker_remapper = com_py.marker_remapper:main',
            'lanelet_saver = com_py.lanelet_saver:main',
            'localization_map_split_reduce_saver = com_py.localization_map_split_reduce_saver:main',
            'tf_map_to_cart_saver = com_py.tf_map_to_cart_saver:main',
        ],
    },
)
