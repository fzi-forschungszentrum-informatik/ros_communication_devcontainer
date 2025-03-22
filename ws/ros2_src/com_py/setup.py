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
    maintainer='myuser',
    maintainer_email='myuser@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
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
        ],
    },
)
