# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause


import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'go2_robot_sdk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'dae'), glob(os.path.join('dae', '*'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'calibration'), glob(os.path.join('calibration', '*'))),
        (os.path.join('share', package_name, 'external_lib'), ['external_lib/libvoxel.wasm']),
        (os.path.join('share', package_name, 'external_lib/aioice'), glob(os.path.join('external_lib/aioice/src/aioice', '*'))),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brimo',
    maintainer_email='abizov94@gmail.com',
    description='Go2 ROS2 SDK for Unitree Go2 Edu/Pro/Air models',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'go2_driver_node = go2_robot_sdk.main:main',
        ],
    },
)
