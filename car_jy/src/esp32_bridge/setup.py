import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'esp32_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='juyeon',
    maintainer_email='juyeon@todo.todo',
    description='ESP32 Serial Bridge for AMR',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_bridge = esp32_bridge.esp32_bridge:main',
        ],
    },
)
