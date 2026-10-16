from setuptools import setup

package_name = 'amr_servo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Edu',
    maintainer_email='edu@example.com',
    description='STS3215 wheel-mode controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sts_controller = amr_servo.sts_controller:main',
        ],
    },
)
