import os
from glob import glob
from setuptools import setup

package_name = 'amr_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Edu',
    maintainer_email='edu@example.com',
    description='Full bringup package for ROS2 AMR',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_teleop = amr_bringup.web_teleop:main',
            'cmd_vel_odom = amr_bringup.cmd_vel_odom:main',
            'apriltag_docking_node = amr_bringup.apriltag_docking_node:main',
            'undock_node = amr_bringup.undock_node:main',
        ],
    },
)
