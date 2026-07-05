import os
from glob import glob
from setuptools import setup

package_name = 'web_bridge'

setup(
    name=package_name,
    version='3.0.0',
    packages=[
        package_name,
        package_name + '.rosbridge',
        package_name + '.api_interfaces',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='damith',
    maintainer_email='cldac@buyjustnow.com',
    description='ROS 2 <-> web app bridge.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'api_adapter = web_bridge.api_interfaces.api_adapter:main',
        ],
    },
)
