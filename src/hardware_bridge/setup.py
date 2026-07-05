from setuptools import setup

package_name = 'hardware_bridge'

setup(
    name=package_name,
    version='3.0.0',
    packages=[
        package_name,
        package_name + '.serial_node',
        package_name + '.sensor_publishers',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='damith',
    maintainer_email='cldac@buyjustnow.com',
    description='Arduino serial bridge + sensor publishers.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_bridge = hardware_bridge.serial_node.arduino_bridge:main',
        ],
    },
)
