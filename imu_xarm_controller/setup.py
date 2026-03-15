import os
from glob import glob
from setuptools import setup

package_name = 'imu_xarm_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    entry_points={
        'console_scripts': [
            'master_slave = imu_xarm_controller.master_slave_node:main',
            'serial_publisher = imu_xarm_controller.serial_publisher:main',
            'serial_subscriber = imu_xarm_controller.serial_subscriber:main',
        ],
    },
)
