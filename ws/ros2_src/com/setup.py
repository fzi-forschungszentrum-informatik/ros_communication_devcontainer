from setuptools import find_packages, setup

package_name = 'com'

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
            'image_bandwidth_reducer = com.image_bandwidth_reducer:main',
            'wifi_network_analysis_1 = com.wifi_network_analysis_1:main',
            'wifi_network_analysis_2 = com.wifi_network_analysis_2:main',
            'speedtest = com.speedtest:main',
            'traffic_monitor = com.traffic_monitor:main',
            'universal_compress = com.universal_compress:main',
            'universal_decompressor = com.universal_decompressor:main',
            'heartbeat = com.heartbeat:main',
            'topic_monitor = com.topic_monitor:main',
            'bridge_in = com.bridge_in:main',
            'bridge_out = com.bridge_out:main',
            'relay_in = com.relay_in:main',
            'relay_out = com.relay_out:main',
        ],
    },
)
