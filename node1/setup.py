from setuptools import setup, find_packages
import os
from glob import glob

PACKAGE_NAME = "pi4_motor_control"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.py")),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pi4_user",
    maintainer_email="you@example.com",
    description="Differential drive motor controller for Raspberry Pi 4 — ROS2 Humble",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motor_controller = pi4_motor_control.motor_controller_node:main",
        ],
    },
)
