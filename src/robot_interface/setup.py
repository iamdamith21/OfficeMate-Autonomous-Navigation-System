from setuptools import setup

package_name = 'robot_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='damith',
    maintainer_email='cldac@buyjustnow.com',
    description='Serial bridge between ROS 2 and the OfficeMate Arduino Mega.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_bridge = robot_interface.arduino_bridge:main',
        ],
    },
)
