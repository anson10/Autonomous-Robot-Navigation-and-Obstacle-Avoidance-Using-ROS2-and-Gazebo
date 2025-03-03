from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anson',
    maintainer_email='sansonmsa@gmail.com',
    description='Obstacle avoidance robot with ROS2 and Gazebo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = my_robot_navigation.obstacle_avoidance:main',
        ],
    },
)
