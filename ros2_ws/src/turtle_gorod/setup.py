from glob import glob
import os

from setuptools import find_packages, setup

package_name = "turtle_gorod"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="turtle_gorod",
    maintainer_email="alex@example.com",
    description="ROS 2 stack for autonomous RTK city driving robot",
    license="MIT",
    entry_points={
        "console_scripts": [
            "serial_bridge = turtle_gorod.serial_bridge:main",
            "collision_guard = turtle_gorod.collision_guard:main",
        ],
    },
)
