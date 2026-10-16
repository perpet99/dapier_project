from setuptools import setup, find_packages

setup(
    name="ros2_amr_bot",
    version="0.1.0",
    description="AMR Robot Core Control and Autonomous Navigation Modules",
    packages=find_packages(include=["bot_modules", "bot_modules.*"]),
    python_requires=">=3.8",
)
