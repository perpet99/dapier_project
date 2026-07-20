import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'large_models_examples'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, package_name),
            glob(os.path.join(package_name, '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='perpet',
    maintainer_email='perpet99@gmail.com',
    description='Voice-driven large-model example nodes for ROSOrin Pro.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_control_move = large_models_examples.llm_control_move:main',
            'llm_visual_patrol = large_models_examples.llm_visual_patrol:main',
            'llm_color_track = large_models_examples.llm_color_track:main',
        ],
    },
)
