from setuptools import find_packages, setup

package_name = "myrobot_nav_service"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="perpet99",
    maintainer_email="perpet99@gmail.com",
    description="ROS 2 service node that drives the robot to a target x, y, yaw pose via Nav2",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "go_to_pose_service = myrobot_nav_service.go_to_pose_service:main",
            "go_to_pose_client = myrobot_nav_service.go_to_pose_client:main",
        ],
    },
)
