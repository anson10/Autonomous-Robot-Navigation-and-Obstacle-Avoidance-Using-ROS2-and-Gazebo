# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Anson Antony

"""Setuptools configuration for my_robot_navigation ROS2 package."""

from glob import glob
import os

from setuptools import setup

package_name = 'my_robot_navigation'

setup(
    name=package_name,
    version='1.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz'),
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anson',
    maintainer_email='sansonmsa@gmail.com',
    description='Autonomous multi-strategy obstacle avoidance robot with ROS2 and Gazebo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = my_robot_navigation.obstacle_avoidance:main',
            'benchmark = my_robot_navigation.benchmark:main',
        ],
    },
)
