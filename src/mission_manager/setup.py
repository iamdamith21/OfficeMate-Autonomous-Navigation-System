import os
from glob import glob
from setuptools import setup

package_name = 'mission_manager'

setup(
    name=package_name,
    version='3.0.0',
    packages=[
        package_name,
        package_name + '.delivery_manager',
        package_name + '.task_scheduler',
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
    description='Delivery mission state machine + task scheduler.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'delivery_manager = mission_manager.delivery_manager.delivery_manager_node:main',
        ],
    },
)
