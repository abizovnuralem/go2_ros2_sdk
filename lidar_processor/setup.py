from setuptools import setup

package_name = 'lidar_processor'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nuralem Abizov',
    maintainer_email='abizov94@gmail.com',
    description='ROS2 package for processing LiDAR data from Go2 robot',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'lidar_to_pointcloud = lidar_processor.lidar_to_pointcloud_node:main',
            'pointcloud_aggregator = lidar_processor.pointcloud_aggregator_node:main',
        ],
    },
) 