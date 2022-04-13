import os
from glob import glob
from setuptools import setup

package_name = 'ros_ign_rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'),
            glob('models/*.xacro')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rhys Mainwaring',
    maintainer_email='rhys.mainwaring@me.com',
    description='Rover demo using Ignition Gazebo and ROS2',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_scan_transform_flu_to_frd = ros_ign_rover.laser_scan_transform_flu_to_frd:main',
            'tf_broadcaster = ros_ign_rover.tf_broadcaster:main',
        ],
    },
)
