from setuptools import setup, find_packages

setup(
    name="lerobot_robot_xarm",
    version="0.0.1",
    description="LeRobot XArm integration",
    author="Spes Robotics",
    author_email="contact@spes.ai",
    packages=find_packages(),
    install_requires=[
        "xarm-python-sdk",
        "numpy",
        "transforms3d",
        "teleop",
        "lerobot",
    ],
    python_requires=">=3.10",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache License",
        "Operating System :: OS Independent",
    ],
)
