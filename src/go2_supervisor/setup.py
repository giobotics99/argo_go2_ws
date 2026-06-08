import os
from glob import glob
from setuptools import setup, find_packages

package_name = "go2_supervisor"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ament index registration
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # package.xml for colcon
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        # rclpy, action_msgs, geometry_msgs, std_msgs are provided by the
        # ROS 2 Foxy workspace and should NOT be pip-installed.  They are
        # declared as <exec_depend> in package.xml instead.
        #
        # urllib is part of the Python stdlib – no extra dep needed.
    ],
    zip_safe=True,
    maintainer="Giobotics",
    maintainer_email="giobotics@example.com",
    description=(
        "Explainable Path Planning supervisor for Unitree Go2. "
        "Queries Gemma 4 E2B via local vLLM on Jetson Orin to explain "
        "Nav2 failures and dispatch alternative navigation goals."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "supervisor_node = go2_supervisor.supervisor_node:main",
        ],
    },
)
